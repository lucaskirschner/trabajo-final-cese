/******************************************************************************
 * Copyright (c) 2026, Lucas Kirschner <kirschnerlucas1@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * SPDX-License-Identifier: MIT
 ******************************************************************************/

/**
 * @file    mrf24j40.c
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-02-23
 * @brief   MRF24J40 driver implementation.
 *
 * @details
 * Concurrency model:
 *  - Blocking SPI transfers from thread context.
 *  - The external interrupt callback is handled in application code, which
 *    notifies the driver by setting a generic interrupt pending flag.
 *  - Interrupt source decoding is deferred to thread context by reading
 *    INTSTAT in mrf24j40_update_interrupt_flags().
 *  - RX FIFO read is protected by temporarily holding the RX decoder.
 *
 * @ingroup mrf24j40
 * @{
 */

/* ============================= Includes ================================== */

#include "mrf24j40.h"
#include "mrf24j40_port.h"
#include "mrf24j40_reg.h"
#include "stm32h5xx_hal.h"

/* ============================ Local Macros =============================== */
/**
 * @brief Default RF channel used by the base initialization.
 *
 * @details
 * IEEE 802.15.4 channel 11 is encoded as value 0x00 in CHANNEL[3:0],
 * because the device uses:
 *
 * encoded_channel = ieee_channel - 11
 */
#define MRF24J40_INIT_DEFAULT_CHANNEL          ((uint8_t)0x00u)

/**
 * @brief Default RF option value recommended by the datasheet.
 */
#define MRF24J40_INIT_RFOPT_VALUE              ((uint8_t)0x03u)

/**
 * @brief Default VCO option value recommended by the datasheet.
 */
#define MRF24J40_INIT_VCOOPT_VALUE             ((uint8_t)0x02u)

/**
 * @brief Default TX power used by the base initialization.
 *
 * @details
 * This corresponds to the maximum nominal output power setting of the bare
 * MRF24J40 transceiver register definition (0 dB attenuation).
 */
#define MRF24J40_INIT_TX_POWER_VALUE           ((uint8_t)0x00u)

/**
 * @brief Default CCA energy detection threshold recommended by the datasheet.
 */
#define MRF24J40_INIT_CCAEDTH_VALUE            ((uint8_t)0x60u)

/**
 * @brief Delay after RF state machine reset, in milliseconds.
 */
#define MRF24J40_RF_STATE_RESET_DELAY_MS       ((uint16_t)1u)

/**
 * @brief ORDER register value for nonbeacon-enabled operation.
 *
 * @details
 * For a nonbeacon-enabled network, the datasheet requires:
 * - BO = 15
 * - SO = 15
 */
#define MRF24J40_ORDER_NONBEACON_VALUE         ((uint8_t)0xFFu)

/**
 * @brief RXMCR PAN coordinator bit mask.
 */
#define MRF24J40_RXMCR_PANCOORD_MASK           ((uint8_t)(1u << 3))

/**
 * @brief TXMCR slotted CSMA-CA bit mask.
 */
#define MRF24J40_TXMCR_SLOTTED_MASK            ((uint8_t)(1u << 5))

/**
 * @brief Length of the Frame Check Sequence (FCS) field in bytes.
 */
#define MRF24J40_FCS_LENGTH                    ((uint8_t)2u)

/* ============================ Local Types ================================ */

/* ======================= Local (static) Data ============================= */
static mrf24j40_context_t mrf24j40_ctx;

/* ========================== Private Prototypes =========================== */

static void mrf24j40_rf_state_reset(void);
static void mrf24j40_configure_nonbeacon_network(void);
static uint64_t mrf24j40_crc64_ecma(const uint8_t * data, uint32_t length);
static void mrf24j40_set_rxdecinv(bool enable);

/* ===================== Public Function Definitions ======================= */

void mrf24j40_init(void)
{
    /* Step 1: Perform software reset (RSTPWR, RSTBB and RSTMAC). */
    mrf24j40_port_write_short(SOFTRST,
                         	 (uint8_t)(SOFTRST_RSTPWR |
                         			   SOFTRST_RSTBB  |
									   SOFTRST_RSTMAC));

    /* Step 2: Initialize FIFOEN = 1 and TXONTS = 0x6. */
    mrf24j40_port_write_short(PACON2,
                         	 (uint8_t)(PACON2_FIFOEN  |
                         			   PACON2_TXONTS2 |
									   PACON2_TXONTS1));

    /* Step 3: Initialize RFSTBL = 0x9, MSIFS = reset default preserved. */
    mrf24j40_port_write_short(TXSTBL,
                         	 (uint8_t)(TXSTBL_RFSTBL3 |
                         			   TXSTBL_RFSTBL0 |
									   TXSTBL_MSIFS2  |
									   TXSTBL_MSIFS0));

    /* Step 4: Initialize RFOPT = 0x03 on RFCON0, keeping channel at default
     * encoded value 0x00 (IEEE channel 11).
     */
    mrf24j40_port_write_long(RFCON0,
    						(uint8_t)((MRF24J40_INIT_DEFAULT_CHANNEL << 4u) |
    		                           MRF24J40_INIT_RFOPT_VALUE));

    /* Step 5: Initialize VCOOPT = 0x02. */
    mrf24j40_port_write_long(RFCON1, MRF24J40_INIT_VCOOPT_VALUE);

    /* Step 6: Enable PLL. */
    mrf24j40_port_write_long(RFCON2, RFCON2_PLLEN);

    /* Step 7: Initialize TXFIL = 1 and 20MRECVR = 1. */
    mrf24j40_port_write_long(RFCON6,
                        	(uint8_t)(RFCON6_TXFIL    |
                        			  RFCON6_20MRECVR));

    /* Step 8: Select 100 kHz internal oscillator for sleep clock. */
    mrf24j40_port_write_long(RFCON7, RFCON7_SLPCLKSEL_100KHZ);

    /* Step 9: Initialize RFVCO = 1. */
    mrf24j40_port_write_long(RFCON8, RFCON8_RFVCO);

    /* Step 10: Initialize CLKOUTEN = 1 and SLPCLKDIV = 0x01. */
    mrf24j40_port_write_long(SLPCON1,
                        	(uint8_t)(SLPCON1_CLKOUTEN   |
                        			  SLPCON1_SLPCLKDIV0));

    /* Step 11: For nonbeacon-enabled operation, set CCA mode to ED. */
    mrf24j40_port_write_short(BBREG2, BBREG2_CCAMODE_1);

    /* Step 12: Set CCA energy detection threshold. */
    mrf24j40_port_write_short(CCAEDTH, MRF24J40_INIT_CCAEDTH_VALUE);

    /* Step 13: Append RSSI value to RX FIFO. */
    mrf24j40_port_write_short(BBREG6, BBREG6_RSSIMODE2);

    /* Step 14: Enable interrupts.
     *
     * INTCON bits are active-low enables:
     * 0 = interrupt enabled
     * 1 = interrupt disabled
     *
     * For this minimal base case, enable RXIE and TXNIE interrupts only.
     */
    mrf24j40_port_write_short(INTCON,
                         	 (uint8_t)(INTCON_SLPIE     |
                         			   INTCON_WAKEIE    |
									   INTCON_HSYMTMRIE |
									   INTCON_SECIE     |
									   INTCON_TXG2IE    |
									   INTCON_TXG1IE));

    /* Step 15: Set channel.
     *
     * Already applied in Step 4 using the default base-case value:
     * encoded channel 0x00 => IEEE 802.15.4 channel 11.
     */

    /* Step 16: Set transmitter power. */
    mrf24j40_port_write_long(RFCON3, MRF24J40_INIT_TX_POWER_VALUE);

    /* Steps 17, 18 and 19: Reset RF state machine and wait for calibration. */
    mrf24j40_rf_state_reset();
}

void mrf24j40_configure_nonbeacon_pan_coordinator(void)
{
    uint8_t reg_value;

    mrf24j40_configure_nonbeacon_network();
    mrf24j40_port_read_short(RXMCR, &reg_value);
    reg_value |= MRF24J40_RXMCR_PANCOORD_MASK;
    mrf24j40_port_write_short(RXMCR, reg_value);
}

void mrf24j40_configure_nonbeacon_device(void)
{
    uint8_t reg_value;

    mrf24j40_configure_nonbeacon_network();
    mrf24j40_port_read_short(RXMCR, &reg_value);
    reg_value &= (uint8_t)(~MRF24J40_RXMCR_PANCOORD_MASK);
    mrf24j40_port_write_short(RXMCR, reg_value);
}

void mrf24j40_set_pan_id(uint16_t pan_id)
{
    uint8_t pan_id_lsb;
    uint8_t pan_id_msb;

    /* Extract LSB and MSB */
    pan_id_lsb = (uint8_t)(pan_id & 0xFFu);
    pan_id_msb = (uint8_t)((pan_id >> 8u) & 0xFFu);

    /* Write PAN ID LSB */
    mrf24j40_port_write_short(PANIDL, pan_id_lsb);

    /* Write PAN ID MSB */
    mrf24j40_port_write_short(PANIDH, pan_id_msb);
}

void mrf24j40_set_short_address(uint16_t short_address)
{
    uint8_t short_address_lsb;
    uint8_t short_address_msb;

    /* Extract LSB and MSB */
    short_address_lsb = (uint8_t)(short_address & 0xFFu);
    short_address_msb = (uint8_t)((short_address >> 8u) & 0xFFu);

    /* Write Short Address LSB */
    mrf24j40_port_write_short(SADRL, short_address_lsb);

    /* Write Short Address MSB */
    mrf24j40_port_write_short(SADRH, short_address_msb);
}

void mrf24j40_set_extended_address(void)
{
    uint32_t uid_w0;
    uint32_t uid_w1;
    uint32_t uid_w2;
    uint8_t uid_bytes[12];
    uint64_t extended_address;
    uint8_t addr_byte;
    uint8_t i;

    /*uid_w0 = HAL_GetUIDw0();
    uid_w1 = HAL_GetUIDw1();
    uid_w2 = HAL_GetUIDw2();*/

    uid_w0 = 0x12345678u;
	uid_w1 = 0x9ABCDEF0u;
	uid_w2 = 0x0FEDCBA9u;

    uid_bytes[0]  = (uint8_t)(uid_w0 & 0xFFu);
    uid_bytes[1]  = (uint8_t)((uid_w0 >> 8u) & 0xFFu);
    uid_bytes[2]  = (uint8_t)((uid_w0 >> 16u) & 0xFFu);
    uid_bytes[3]  = (uint8_t)((uid_w0 >> 24u) & 0xFFu);

    uid_bytes[4]  = (uint8_t)(uid_w1 & 0xFFu);
    uid_bytes[5]  = (uint8_t)((uid_w1 >> 8u) & 0xFFu);
    uid_bytes[6]  = (uint8_t)((uid_w1 >> 16u) & 0xFFu);
    uid_bytes[7]  = (uint8_t)((uid_w1 >> 24u) & 0xFFu);

    uid_bytes[8]  = (uint8_t)(uid_w2 & 0xFFu);
    uid_bytes[9]  = (uint8_t)((uid_w2 >> 8u) & 0xFFu);
    uid_bytes[10] = (uint8_t)((uid_w2 >> 16u) & 0xFFu);
    uid_bytes[11] = (uint8_t)((uid_w2 >> 24u) & 0xFFu);

    extended_address = mrf24j40_crc64_ecma(uid_bytes, (uint32_t)sizeof(uid_bytes));

    /* Force the generated identifier to look like a locally administered
     * unicast EUI-64:
     * - clear I/G bit   (bit 0 of first octet)
     * - set U/L bit     (bit 1 of first octet)
     *
     * Here the "first octet" is mapped to bits [63:56].
     */
    extended_address &= ~(UINT64_C(0x01) << 56);
    extended_address |=  (UINT64_C(0x02) << 56);

    for (i = 0u; i < 8u; i++)
    {
        addr_byte = (uint8_t)((extended_address >> (8u * i)) & 0xFFu);

        mrf24j40_port_write_short((uint8_t)(EADR0 + i), addr_byte);
    }
}

void mrf24j40_set_interrupt_pending(void)
{
    mrf24j40_ctx.int_pending = true;
}

void mrf24j40_update_interrupt_flags(void)
{
	/* This minimal driver currently handles only RXIF and TXNIF, because only
	 * RXIE and TXNIE are enabled in the base initialization.
	 */

    uint8_t intstat;

    if (mrf24j40_ctx.int_pending == false)
    {
        return;
    }

    mrf24j40_port_read_short(INTSTAT, &intstat);

    if ((intstat & INTSTAT_RXIF) != 0u)
    {
        mrf24j40_ctx.rx_pending = true;
    }

    if ((intstat & INTSTAT_TXNIF) != 0u)
    {
        mrf24j40_ctx.tx_complete = true;
    }

    mrf24j40_ctx.int_pending = false;
}

bool mrf24j40_read_rx_fifo(mrf24j40_packet_t * p_packet)
{
    uint8_t rx_frame_length;
    uint16_t addr;
    uint8_t i;

    /* Validate output packet pointer. */
    if (p_packet == NULL)
    {
        return false;
    }

    /* Exit if no receive event was previously latched by application code. */
    if (mrf24j40_ctx.rx_pending == false)
    {
        return false;
    }

    /* Step 1: Disable host interrupts to avoid interruption while reading the
     * RXFIFO, as recommended by the datasheet.
     */
    __disable_irq();

    /* Step 2: Set RXDECINV = 1 to prevent the receiver from accepting a new
     * packet from the air while the current RXFIFO contents are being read.
     */
    mrf24j40_set_rxdecinv(true);

    /* Step 3: Read RXFIFO first byte at address 0x300.
     *
     * This byte contains the received frame length, which includes:
     * - MAC header
     * - payload
     * - FCS
     */
    mrf24j40_port_read_long(RX_FIFO, &rx_frame_length);

    /* A valid received frame must include at least the 2-byte FCS field. */
    if (rx_frame_length < MRF24J40_FCS_LENGTH)
    {
        mrf24j40_set_rxdecinv(false);
        mrf24j40_ctx.rx_pending = false;
        __enable_irq();
        return false;
    }

    /* The public packet representation excludes the FCS.
     * Convert RXFIFO length to:
     * - MAC header
     * - payload
     */
    p_packet->frame_length = (uint8_t)(rx_frame_length - MRF24J40_FCS_LENGTH);

    /* Reject invalid lengths that do not fit in the public packet buffer. */
    if (p_packet->frame_length > MRF24J40_MAX_FRAME_SIZE)
    {
        mrf24j40_set_rxdecinv(false);
        mrf24j40_ctx.rx_pending = false;
        __enable_irq();
        return false;
    }

    /* Step 4: Read RXFIFO frame bytes starting at address 0x301.
     *
     * Only MAC header and payload bytes are copied to the public packet
     * buffer. The trailing FCS bytes are intentionally not exposed by this
     * driver API.
     */
    addr = (uint16_t)(RX_FIFO + 1u);

    for (i = 0u; i < p_packet->frame_length; i++)
    {
        mrf24j40_port_read_long(addr, &p_packet->frame[i]);
        addr++;
    }

    /* Skip the 2-byte FCS field stored after the received frame data. */
    addr += MRF24J40_FCS_LENGTH;

    /* Step 5: Read LQI value, located immediately after the frame and FCS
     * bytes.
     */
    mrf24j40_port_read_long(addr, &p_packet->lqi);
    addr++;

    /* Step 6: Read RSSI value, located immediately after LQI. */
    mrf24j40_port_read_long(addr, &p_packet->rssi);

    /* Step 7: Clear RXDECINV = 0 to restore normal packet reception. */
    mrf24j40_set_rxdecinv(false);

    /* Clear local RX pending indication now that the packet has been consumed. */
    mrf24j40_ctx.rx_pending = false;

    /* Re-enable host interrupts after the complete RXFIFO read sequence. */
    __enable_irq();

    return true;
}

bool mrf24j40_write_tx_normal_fifo(const mrf24j40_packet_t * p_packet,
                                   bool ack_request)
{
    uint8_t txncon_value;
    uint16_t addr;
    uint8_t i;

    /* Validate input packet pointer. */
    if (p_packet == NULL)
    {
        return false;
    }

    /* Reject invalid frame lengths that do not fit in the public packet
     * representation.
     */
    if ((p_packet->frame_length == 0u) ||
        (p_packet->frame_length > MRF24J40_MAX_FRAME_SIZE))
    {
        return false;
    }

    /* Clear previous TX completion indication before starting a new
     * transmission.
     */
    mrf24j40_ctx.tx_complete = false;

    /* Step 1: Load TX Normal FIFO first byte.
     *
     * In unsecure mode, the header length field is ignored by the device.
     */
    mrf24j40_port_write_long(TX_NORMAL_FIFO, 0u);

    /* Step 2: Load TX Normal FIFO second byte with frame length.
     *
     * The public packet representation contains:
     * - MAC header
     * - payload
     *
     * The FCS is not part of the public packet and is appended automatically
     * by the MRF24J40 during transmission.
     */
    mrf24j40_port_write_long((uint16_t)(TX_NORMAL_FIFO + 1u),
                             p_packet->frame_length);

    /* Step 3: Load TX Normal FIFO frame bytes starting at address 0x002.
     *
     * Only MAC header and payload bytes are written by the host processor.
     */
    addr = (uint16_t)(TX_NORMAL_FIFO + 2u);

    for (i = 0u; i < p_packet->frame_length; i++)
    {
        mrf24j40_port_write_long(addr, p_packet->frame[i]);
        addr++;
    }

    /* Step 4: Configure TX Normal FIFO control bits.
     *
     * If acknowledgment is requested, TXNACKREQ is set before transmission is
     * triggered.
     */
    txncon_value = 0u;

    if (ack_request == true)
    {
        txncon_value |= TXNCON_TXNACKREQ;
    }

    /* Step 5: Trigger transmission by setting TXNTRIG = 1. */
    txncon_value |= TXNCON_TXNTRIG;

    mrf24j40_port_write_short(TXNCON, txncon_value);

    return true;
}

bool mrf24j40_get_tx_complete(void)
{
    if (mrf24j40_ctx.tx_complete == false)
    {
        return false;
    }

    mrf24j40_ctx.tx_complete = false;

    return true;
}

/* ===================== Private Function Definitions ====================== */

/**
 * @brief Reset the internal RF state machine and wait for calibration.
 *
 * @details
 * The datasheet requires setting RFRST, then clearing it, and waiting at least
 * 192 us for RF calibration to complete.
 */
static void mrf24j40_rf_state_reset(void)
{
	mrf24j40_port_write_short(RFCTL, RFCTL_RFRST);
	mrf24j40_port_write_short(RFCTL, 0x00u);
    mrf24j40_port_delay_ms(MRF24J40_RF_STATE_RESET_DELAY_MS);
}

/**
 * @brief Configure common MAC settings for nonbeacon-enabled operation.
 *
 * @details
 * This helper applies the common settings required by the datasheet for a
 * nonbeacon-enabled network:
 * - Unslotted CSMA-CA
 * - Beacon Order = 15
 * - Superframe Order = 15
 *
 * Role selection (PAN coordinator or device) is intentionally handled by the
 * public role-specific functions.
 */
static void mrf24j40_configure_nonbeacon_network(void)
{
    uint8_t reg_value;

    /* Configure BO = 15 and SO = 15. */
    mrf24j40_port_write_short(ORDER, MRF24J40_ORDER_NONBEACON_VALUE);
    /* Clear SLOTTED to select unslotted CSMA-CA mode. */
    mrf24j40_port_read_short(TXMCR, &reg_value);
    reg_value &= (uint8_t)(~MRF24J40_TXMCR_SLOTTED_MASK);
    mrf24j40_port_write_short(TXMCR, reg_value);
}

/**
 * @brief Compute CRC64-ECMA over a byte buffer.
 *
 * @param data Pointer to input buffer.
 * @param length Number of input bytes.
 *
 * @return 64-bit CRC value.
 *
 * @details
 * Polynomial: 0x42F0E1EBA9EA3693
 * Initial value: 0x0000000000000000
 * No reflection
 * No final XOR
 */
static uint64_t mrf24j40_crc64_ecma(const uint8_t * data, uint32_t length)
{
    uint64_t crc;
    uint32_t i;
    uint8_t bit;

    crc = UINT64_C(0x0000000000000000);

    for (i = 0u; i < length; i++)
    {
        crc ^= ((uint64_t)data[i] << 56);

        for (bit = 0u; bit < 8u; bit++)
        {
            if ((crc & UINT64_C(0x8000000000000000)) != 0u)
            {
                crc = (crc << 1u) ^ UINT64_C(0x42F0E1EBA9EA3693);
            }
            else
            {
                crc <<= 1u;
            }
        }
    }

    return crc;
}

/**
 * @brief Enable or disable RX decode inversion.
 *
 * @param enable Set to true to disable packet reception, false to enable it.
 *
 * @details
 * This function controls the RXDECINV bit in the BBREG1 register.
 *
 * When RXDECINV is set to 1, the receiver is prevented from accepting
 * incoming packets from the air. This is recommended by the datasheet
 * while reading the RXFIFO to avoid data corruption if a new packet
 * arrives during the read operation.
 *
 * When RXDECINV is cleared to 0, normal packet reception is resumed.
 *
 * This mechanism is used to ensure atomic access to the RXFIFO buffer
 * in systems where reception and SPI access may overlap.
 */
static void mrf24j40_set_rxdecinv(bool enable)
{
    uint8_t reg_value;

    mrf24j40_port_read_short(BBREG1, &reg_value);

    if (enable == true)
    {
        reg_value |= BBREG1_RXDECINV;
    }
    else
    {
        reg_value &= (uint8_t)(~BBREG1_RXDECINV);
    }

    mrf24j40_port_write_short(BBREG1, reg_value);
}
/** @} */
