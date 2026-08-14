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
 *****************************************************************************/

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
 *  - TX completion is latched after TXNIF and the final transmission result
 *    is obtained by reading TXSTAT from thread context.
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

/**
 * @brief TXSTAT TX Normal FIFO retry count mask.
 *
 * @details
 * TXNRETRY<1:0> occupies bits 7:6 and reports the number of retransmissions
 * performed for the most recent TX Normal FIFO transaction.
 */
#define MRF24J40_TXSTAT_TXNRETRY_MASK          ((uint8_t)0xC0u)

/**
 * @brief TXSTAT TX Normal FIFO retry count shift.
 */
#define MRF24J40_TXSTAT_TXNRETRY_SHIFT         ((uint8_t)6u)

/**
 * @brief TXSTAT CCA failure mask.
 *
 * @details
 * When set, the most recent transmission failed because CSMA-CA could not
 * obtain an idle channel.
 */
#define MRF24J40_TXSTAT_CCAFAIL_MASK           ((uint8_t)(1u << 5))

/**
 * @brief TXSTAT TX Normal FIFO release status mask.
 *
 * @details
 * A cleared TXNSTAT bit indicates a successful TX Normal FIFO transmission.
 * A set TXNSTAT bit indicates a failed transmission.
 */
#define MRF24J40_TXSTAT_TXNSTAT_MASK           ((uint8_t)(1u << 0))

/* ============================ Local Types ================================ */

/* ======================= Local (static) Data ============================= */

static mrf24j40_context_t mrf24j40_ctx;

/* ========================== Private Prototypes =========================== */

static mrf24j40_status_t mrf24j40_rf_state_reset(void);
static mrf24j40_status_t mrf24j40_configure_nonbeacon_network(void);
static uint64_t mrf24j40_crc64_ecma(const uint8_t * data, uint32_t length);
static mrf24j40_status_t mrf24j40_set_rxdecinv(bool enable);
static mrf24j40_status_t mrf24j40_from_port_status(
    mrf24j40_port_status_t status);

/* ===================== Public Function Definitions ======================= */

mrf24j40_status_t mrf24j40_init(void)
{
    mrf24j40_status_t status;

    /* Initialize driver runtime state. */
    mrf24j40_ctx.int_pending = false;
    mrf24j40_ctx.rx_pending = false;
    mrf24j40_ctx.tx_complete = false;
    mrf24j40_ctx.tx_ack_requested = false;

    /* Step 1: Perform software reset (RSTPWR, RSTBB and RSTMAC). */
    status = mrf24j40_from_port_status(
        mrf24j40_port_write_short(SOFTRST,
                                  (uint8_t)(SOFTRST_RSTPWR |
                                            SOFTRST_RSTBB  |
                                            SOFTRST_RSTMAC)));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    /* Step 2: Initialize FIFOEN = 1 and TXONTS = 0x6. */
    status = mrf24j40_from_port_status(
        mrf24j40_port_write_short(PACON2,
                                  (uint8_t)(PACON2_FIFOEN  |
                                            PACON2_TXONTS2 |
                                            PACON2_TXONTS1)));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    /* Step 3: Initialize RFSTBL = 0x9, MSIFS = reset default preserved. */
    status = mrf24j40_from_port_status(
        mrf24j40_port_write_short(TXSTBL,
                                  (uint8_t)(TXSTBL_RFSTBL3 |
                                            TXSTBL_RFSTBL0 |
                                            TXSTBL_MSIFS2  |
                                            TXSTBL_MSIFS0)));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    /* Step 4: Initialize RFOPT = 0x03 on RFCON0, keeping channel at default
     * encoded value 0x00 (IEEE channel 11).
     */
    status = mrf24j40_from_port_status(
        mrf24j40_port_write_long(
            RFCON0,
            (uint8_t)((MRF24J40_INIT_DEFAULT_CHANNEL << 4u) |
                      MRF24J40_INIT_RFOPT_VALUE)));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    /* Step 5: Initialize VCOOPT = 0x02. */
    status = mrf24j40_from_port_status(
        mrf24j40_port_write_long(RFCON1,
                                 MRF24J40_INIT_VCOOPT_VALUE));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    /* Step 6: Enable PLL. */
    status = mrf24j40_from_port_status(
        mrf24j40_port_write_long(RFCON2, RFCON2_PLLEN));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    /* Step 7: Initialize TXFIL = 1 and 20MRECVR = 1. */
    status = mrf24j40_from_port_status(
        mrf24j40_port_write_long(RFCON6,
                                 (uint8_t)(RFCON6_TXFIL |
                                           RFCON6_20MRECVR)));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    /* Step 8: Select 100 kHz internal oscillator for sleep clock. */
    status = mrf24j40_from_port_status(
        mrf24j40_port_write_long(RFCON7,
                                 RFCON7_SLPCLKSEL_100KHZ));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    /* Step 9: Initialize RFVCO = 1. */
    status = mrf24j40_from_port_status(
        mrf24j40_port_write_long(RFCON8, RFCON8_RFVCO));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    /* Step 10: Initialize CLKOUTEN = 1 and SLPCLKDIV = 0x01. */
    status = mrf24j40_from_port_status(
        mrf24j40_port_write_long(SLPCON1,
                                 (uint8_t)(SLPCON1_CLKOUTEN |
                                           SLPCON1_SLPCLKDIV0)));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    /* Step 11: For nonbeacon-enabled operation, set CCA mode to ED. */
    status = mrf24j40_from_port_status(
        mrf24j40_port_write_short(BBREG2, BBREG2_CCAMODE_1));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    /* Step 12: Set CCA energy detection threshold. */
    status = mrf24j40_from_port_status(
        mrf24j40_port_write_short(CCAEDTH,
                                  MRF24J40_INIT_CCAEDTH_VALUE));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    /* Step 13: Append RSSI value to RX FIFO. */
    status = mrf24j40_from_port_status(
        mrf24j40_port_write_short(BBREG6, BBREG6_RSSIMODE2));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    /* Step 14: Enable interrupts.
     *
     * INTCON bits are active-low enables:
     * 0 = interrupt enabled
     * 1 = interrupt disabled
     *
     * For this minimal base case, enable RXIE and TXNIE interrupts only.
     */
    status = mrf24j40_from_port_status(
        mrf24j40_port_write_short(
            INTCON,
            (uint8_t)(INTCON_SLPIE     |
                      INTCON_WAKEIE    |
                      INTCON_HSYMTMRIE |
                      INTCON_SECIE     |
                      INTCON_TXG2IE    |
                      INTCON_TXG1IE)));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    /* Step 15: Set channel.
     *
     * Already applied in Step 4 using the default base-case value:
     * encoded channel 0x00 => IEEE 802.15.4 channel 11.
     */

    /* Step 16: Set transmitter power. */
    status = mrf24j40_from_port_status(
        mrf24j40_port_write_long(RFCON3,
                                 MRF24J40_INIT_TX_POWER_VALUE));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    /* Steps 17, 18 and 19: Reset RF state machine and wait for calibration. */
    status = mrf24j40_rf_state_reset();
    if (status != MRF24J40_OK)
    {
        return status;
    }

    return MRF24J40_OK;
}

mrf24j40_status_t mrf24j40_configure_nonbeacon_pan_coordinator(void)
{
    mrf24j40_status_t status;
    uint8_t reg_value;

    status = mrf24j40_configure_nonbeacon_network();
    if (status != MRF24J40_OK)
    {
        return status;
    }

    status = mrf24j40_from_port_status(
        mrf24j40_port_read_short(RXMCR, &reg_value));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    reg_value |= MRF24J40_RXMCR_PANCOORD_MASK;

    status = mrf24j40_from_port_status(
        mrf24j40_port_write_short(RXMCR, reg_value));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    return MRF24J40_OK;
}

mrf24j40_status_t mrf24j40_configure_nonbeacon_device(void)
{
    mrf24j40_status_t status;
    uint8_t reg_value;

    status = mrf24j40_configure_nonbeacon_network();
    if (status != MRF24J40_OK)
    {
        return status;
    }

    status = mrf24j40_from_port_status(
        mrf24j40_port_read_short(RXMCR, &reg_value));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    reg_value &= (uint8_t)(~MRF24J40_RXMCR_PANCOORD_MASK);

    status = mrf24j40_from_port_status(
        mrf24j40_port_write_short(RXMCR, reg_value));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    return MRF24J40_OK;
}

mrf24j40_status_t mrf24j40_set_pan_id(uint16_t pan_id)
{
    mrf24j40_status_t status;
    uint8_t pan_id_lsb;
    uint8_t pan_id_msb;

    /* Extract LSB and MSB */
    pan_id_lsb = (uint8_t)(pan_id & 0xFFu);
    pan_id_msb = (uint8_t)((pan_id >> 8u) & 0xFFu);

    /* Write PAN ID LSB */
    status = mrf24j40_from_port_status(
        mrf24j40_port_write_short(PANIDL, pan_id_lsb));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    /* Write PAN ID MSB */
    status = mrf24j40_from_port_status(
        mrf24j40_port_write_short(PANIDH, pan_id_msb));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    return MRF24J40_OK;
}

mrf24j40_status_t mrf24j40_set_short_address(uint16_t short_address)
{
    mrf24j40_status_t status;
    uint8_t short_address_lsb;
    uint8_t short_address_msb;

    /* Extract LSB and MSB */
    short_address_lsb = (uint8_t)(short_address & 0xFFu);
    short_address_msb = (uint8_t)((short_address >> 8u) & 0xFFu);

    /* Write Short Address LSB */
    status = mrf24j40_from_port_status(
        mrf24j40_port_write_short(SADRL, short_address_lsb));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    /* Write Short Address MSB */
    status = mrf24j40_from_port_status(
        mrf24j40_port_write_short(SADRH, short_address_msb));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    return MRF24J40_OK;
}

mrf24j40_status_t mrf24j40_set_extended_address(void)
{
    mrf24j40_status_t status;
    uint32_t uid_w0;
    uint32_t uid_w1;
    uint32_t uid_w2;
    uint8_t uid_bytes[12];
    uint64_t extended_address;
    uint8_t addr_byte;
    uint8_t i;

    /*
    uid_w0 = HAL_GetUIDw0();
    uid_w1 = HAL_GetUIDw1();
    uid_w2 = HAL_GetUIDw2();
    */

    uid_w0 = 0x12345678u;    /* TODO hardcodeado */
    uid_w1 = 0x9ABCDEF0u;    /* TODO hardcodeado */
    uid_w2 = 0x0FEDCBA9u;    /* TODO hardcodeado */

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

    extended_address =
        mrf24j40_crc64_ecma(uid_bytes,
                            (uint32_t)sizeof(uid_bytes));

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
        addr_byte =
            (uint8_t)((extended_address >> (8u * i)) & 0xFFu);

        status = mrf24j40_from_port_status(
            mrf24j40_port_write_short(
                (uint8_t)(EADR0 + i),
                addr_byte));

        if (status != MRF24J40_OK)
        {
            return status;
        }
    }

    return MRF24J40_OK;
}

void mrf24j40_set_interrupt_pending(void)
{
    mrf24j40_ctx.int_pending = true;
}

mrf24j40_status_t mrf24j40_update_interrupt_flags(void)
{
    mrf24j40_status_t status;
    uint8_t intstat;

    /*
     * This minimal driver currently handles only RXIF and TXNIF, because only
     * RXIE and TXNIE are enabled in the base initialization.
     */

    if (mrf24j40_ctx.int_pending == false)
    {
        return MRF24J40_E_STATE;
    }

    /*
     * Consume the current interrupt indication before reading INTSTAT.
     *
     * If another interrupt occurs while the current one is being processed,
     * the external interrupt callback will set int_pending again and that new
     * indication will remain latched.
     */
    mrf24j40_ctx.int_pending = false;

    status = mrf24j40_from_port_status(
        mrf24j40_port_read_short(INTSTAT, &intstat));

    if (status != MRF24J40_OK)
    {
        /*
         * Restore the pending state because the interrupt sources could not
         * be decoded.
         */
        mrf24j40_ctx.int_pending = true;

        return status;
    }

    if ((intstat & INTSTAT_RXIF) != 0u)
    {
        mrf24j40_ctx.rx_pending = true;
    }

    if ((intstat & INTSTAT_TXNIF) != 0u)
    {
        mrf24j40_ctx.tx_complete = true;
    }

    return MRF24J40_OK;
}

mrf24j40_status_t mrf24j40_read_rx_fifo(
    mrf24j40_packet_t * p_packet)
{
    mrf24j40_status_t status;
    uint8_t rx_frame_length;
    uint16_t addr;
    uint8_t i;

    /* Validate output packet pointer. */
    if (p_packet == NULL)
    {
        return MRF24J40_E_NULL;
    }

    /* Exit if no receive event was previously latched by application code. */
    if (mrf24j40_ctx.rx_pending == false)
    {
        return MRF24J40_E_NO_RX_PACKET;
    }

    /* Step 1: Disable host interrupts to avoid interruption while reading the
     * RXFIFO, as recommended by the datasheet.
     */
    __disable_irq();

    /* Step 2: Set RXDECINV = 1 to prevent the receiver from accepting a new
     * packet from the air while the current RXFIFO contents are being read.
     */
    status = mrf24j40_set_rxdecinv(true);
    if (status != MRF24J40_OK)
    {
        __enable_irq();
        return status;
    }

    /* Step 3: Read RXFIFO first byte at address 0x300.
     *
     * This byte contains the received frame length, which includes:
     * - MAC header
     * - payload
     * - FCS
     */
    status = mrf24j40_from_port_status(
        mrf24j40_port_read_long(RX_FIFO, &rx_frame_length));
    if (status != MRF24J40_OK)
    {
        (void)mrf24j40_set_rxdecinv(false);
        __enable_irq();
        return status;
    }

    /* A valid received frame must include at least the 2-byte FCS field. */
    if (rx_frame_length < MRF24J40_FCS_LENGTH)
    {
        (void)mrf24j40_set_rxdecinv(false);
        mrf24j40_ctx.rx_pending = false;
        __enable_irq();
        return MRF24J40_E_FRAME;
    }

    /* The public packet representation excludes the FCS.
     * Convert RXFIFO length to:
     * - MAC header
     * - payload
     */
    p_packet->frame_length =
        (uint8_t)(rx_frame_length - MRF24J40_FCS_LENGTH);

    /* Reject invalid lengths that do not fit in the public packet buffer. */
    if (p_packet->frame_length > MRF24J40_MAX_FRAME_SIZE)
    {
        (void)mrf24j40_set_rxdecinv(false);
        mrf24j40_ctx.rx_pending = false;
        __enable_irq();
        return MRF24J40_E_FRAME;
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
        status = mrf24j40_from_port_status(
            mrf24j40_port_read_long(addr,
                                    &p_packet->frame[i]));
        if (status != MRF24J40_OK)
        {
            (void)mrf24j40_set_rxdecinv(false);
            __enable_irq();
            return status;
        }

        addr++;
    }

    /* Skip the 2-byte FCS field stored after the received frame data. */
    addr += MRF24J40_FCS_LENGTH;

    /* Step 5: Read LQI value, located immediately after the frame and FCS
     * bytes.
     */
    status = mrf24j40_from_port_status(
        mrf24j40_port_read_long(addr, &p_packet->lqi));
    if (status != MRF24J40_OK)
    {
        (void)mrf24j40_set_rxdecinv(false);
        __enable_irq();
        return status;
    }

    addr++;

    /* Step 6: Read RSSI value, located immediately after LQI. */
    status = mrf24j40_from_port_status(
        mrf24j40_port_read_long(addr, &p_packet->rssi));
    if (status != MRF24J40_OK)
    {
        (void)mrf24j40_set_rxdecinv(false);
        __enable_irq();
        return status;
    }

    /* Step 7: Clear RXDECINV = 0 to restore normal packet reception. */
    status = mrf24j40_set_rxdecinv(false);
    if (status != MRF24J40_OK)
    {
        __enable_irq();
        return status;
    }

    /* Clear local RX pending indication now that the packet has been
     * consumed.
     */
    mrf24j40_ctx.rx_pending = false;

    /* Re-enable host interrupts after the complete RXFIFO read sequence. */
    __enable_irq();

    return MRF24J40_OK;
}

mrf24j40_status_t mrf24j40_write_tx_normal_fifo(
    const mrf24j40_packet_t * p_packet,
    bool ack_request)
{
    mrf24j40_status_t status;
    uint8_t txncon_value;
    uint16_t addr;
    uint8_t i;

    /* Validate input packet pointer. */
    if (p_packet == NULL)
    {
        return MRF24J40_E_NULL;
    }

    /* Reject invalid frame lengths that do not fit in the public packet
     * representation.
     */
    if ((p_packet->frame_length == 0u) ||
        (p_packet->frame_length > MRF24J40_MAX_FRAME_SIZE))
    {
        return MRF24J40_E_PARAM;
    }

    /* Clear previous TX completion indication before starting a new
     * transmission.
     */
    mrf24j40_ctx.tx_complete = false;

    /* Store whether this transaction requires acknowledgment so that the
     * final TXSTAT information can be interpreted later.
     */
    mrf24j40_ctx.tx_ack_requested = ack_request;

    /* Step 1: Load TX Normal FIFO first byte.
     *
     * In unsecure mode, the header length field is ignored by the device.
     */
    status = mrf24j40_from_port_status(
        mrf24j40_port_write_long(TX_NORMAL_FIFO, 0u));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    /* Step 2: Load TX Normal FIFO second byte with frame length.
     *
     * The public packet representation contains:
     * - MAC header
     * - payload
     *
     * The FCS is not part of the public packet and is appended automatically
     * by the MRF24J40 during transmission.
     */
    status = mrf24j40_from_port_status(
        mrf24j40_port_write_long(
            (uint16_t)(TX_NORMAL_FIFO + 1u),
            p_packet->frame_length));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    /* Step 3: Load TX Normal FIFO frame bytes starting at address 0x002.
     *
     * Only MAC header and payload bytes are written by the host processor.
     */
    addr = (uint16_t)(TX_NORMAL_FIFO + 2u);

    for (i = 0u; i < p_packet->frame_length; i++)
    {
        status = mrf24j40_from_port_status(
            mrf24j40_port_write_long(addr,
                                     p_packet->frame[i]));
        if (status != MRF24J40_OK)
        {
            return status;
        }

        addr++;
    }

    /* Step 4: Configure TX Normal FIFO control bits.
     *
     * If acknowledgment is requested, TXNACKREQ is set before transmission is
     * triggered. The MRF24J40 then performs acknowledgment monitoring and
     * hardware retransmission automatically.
     */
    txncon_value = 0u;

    if (ack_request == true)
    {
        txncon_value |= TXNCON_TXNACKREQ;
    }

    /* Step 5: Trigger transmission by setting TXNTRIG = 1. */
    txncon_value |= TXNCON_TXNTRIG;

    status = mrf24j40_from_port_status(
        mrf24j40_port_write_short(TXNCON, txncon_value));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    return MRF24J40_OK;
}

mrf24j40_status_t mrf24j40_get_tx_result(
    mrf24j40_tx_result_t * const p_result)
{
    mrf24j40_status_t status;
    uint8_t txstat;
    bool tx_failed;
    bool cca_failed;

    /* Validate output result pointer. */
    if (p_result == NULL)
    {
        return MRF24J40_E_NULL;
    }

    /* Always initialize the complete result so the caller never receives
     * stale information when no TX transaction has completed.
     */
    p_result->complete = false;
    p_result->success = false;
    p_result->ack_requested = false;
    p_result->ack_received = false;
    p_result->retry_limit_reached = false;
    p_result->cca_failed = false;
    p_result->retries = 0u;

    /* A TX result is only valid after TXNIF has been decoded and latched by
     * mrf24j40_update_interrupt_flags().
     */
    if (mrf24j40_ctx.tx_complete == false)
    {
        return MRF24J40_OK;
    }

    /* Read TXSTAT for the most recently completed TX Normal FIFO
     * transmission.
     *
     * Relevant fields:
     * - TXNRETRY<1:0> bits 7:6
     * - CCAFAIL       bit 5
     * - TXNSTAT       bit 0
     */
    status = mrf24j40_from_port_status(
        mrf24j40_port_read_short(TXSTAT, &txstat));
    if (status != MRF24J40_OK)
    {
        /* Do not consume tx_complete on a read failure. This allows the caller
         * to retry obtaining the result later.
         */
        return status;
    }

    tx_failed =
        ((txstat & MRF24J40_TXSTAT_TXNSTAT_MASK) != 0u);

    cca_failed =
        ((txstat & MRF24J40_TXSTAT_CCAFAIL_MASK) != 0u);

    p_result->complete = true;

    p_result->success = (tx_failed == false);

    p_result->ack_requested =
        mrf24j40_ctx.tx_ack_requested;

    p_result->ack_received =
        ((mrf24j40_ctx.tx_ack_requested == true) &&
         (tx_failed == false));

    p_result->cca_failed = cca_failed;

    p_result->retry_limit_reached =
        ((tx_failed == true) &&
         (cca_failed == false));

    p_result->retries =
        (uint8_t)((txstat & MRF24J40_TXSTAT_TXNRETRY_MASK) >>
                  MRF24J40_TXSTAT_TXNRETRY_SHIFT);

    /* Consume the TX result only after TXSTAT was read and decoded
     * successfully.
     */
    mrf24j40_ctx.tx_complete = false;
    mrf24j40_ctx.tx_ack_requested = false;

    return MRF24J40_OK;
}

/* ===================== Private Function Definitions ====================== */

/**
 * @brief Reset the internal RF state machine and wait for calibration.
 *
 * @return
 * - MRF24J40_OK: RF state reset sequence completed successfully.
 * - MRF24J40_E_HW: A hardware communication error occurred.
 * - MRF24J40_E_TIMEOUT: A port-layer transaction timed out.
 * - MRF24J40_E_PARAM: An invalid register access parameter was detected by the
 *   port layer.
 *
 * @details
 * The datasheet requires setting RFRST, then clearing it, and waiting at least
 * 192 us for RF calibration to complete.
 */
static mrf24j40_status_t mrf24j40_rf_state_reset(void)
{
    mrf24j40_status_t status;

    status = mrf24j40_from_port_status(
        mrf24j40_port_write_short(RFCTL, RFCTL_RFRST));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    status = mrf24j40_from_port_status(
        mrf24j40_port_write_short(RFCTL, 0x00u));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    mrf24j40_port_delay_ms(MRF24J40_RF_STATE_RESET_DELAY_MS);

    return MRF24J40_OK;
}

/**
 * @brief Configure common MAC settings for nonbeacon-enabled operation.
 *
 * @return
 * - MRF24J40_OK: MAC configuration completed successfully.
 * - MRF24J40_E_HW: A hardware communication error occurred.
 * - MRF24J40_E_TIMEOUT: A port-layer transaction timed out.
 * - MRF24J40_E_PARAM: An invalid register access parameter was detected by the
 *   port layer.
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
static mrf24j40_status_t mrf24j40_configure_nonbeacon_network(void)
{
    mrf24j40_status_t status;
    uint8_t reg_value;

    /* Configure BO = 15 and SO = 15. */
    status = mrf24j40_from_port_status(
        mrf24j40_port_write_short(
            ORDER,
            MRF24J40_ORDER_NONBEACON_VALUE));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    /* Clear SLOTTED to select unslotted CSMA-CA mode. */
    status = mrf24j40_from_port_status(
        mrf24j40_port_read_short(TXMCR, &reg_value));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    reg_value &= (uint8_t)(~MRF24J40_TXMCR_SLOTTED_MASK);

    status = mrf24j40_from_port_status(
        mrf24j40_port_write_short(TXMCR, reg_value));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    return MRF24J40_OK;
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
 * CRC parameters:
 * - Polynomial: 0x42F0E1EBA9EA3693
 * - Initial value: 0x0000000000000000
 * - No reflection
 * - No final XOR
 */
static uint64_t mrf24j40_crc64_ecma(
    const uint8_t * data,
    uint32_t length)
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
                crc =
                    (crc << 1u) ^
                    UINT64_C(0x42F0E1EBA9EA3693);
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
 * @return
 * - MRF24J40_OK: RX decode inversion state updated successfully.
 * - MRF24J40_E_HW: A hardware communication error occurred.
 * - MRF24J40_E_TIMEOUT: A port-layer transaction timed out.
 * - MRF24J40_E_PARAM: An invalid register access parameter was detected by the
 *   port layer.
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
static mrf24j40_status_t mrf24j40_set_rxdecinv(bool enable)
{
    mrf24j40_status_t status;
    uint8_t reg_value;

    status = mrf24j40_from_port_status(
        mrf24j40_port_read_short(BBREG1, &reg_value));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    if (enable == true)
    {
        reg_value |= BBREG1_RXDECINV;
    }
    else
    {
        reg_value &= (uint8_t)(~BBREG1_RXDECINV);
    }

    status = mrf24j40_from_port_status(
        mrf24j40_port_write_short(BBREG1, reg_value));
    if (status != MRF24J40_OK)
    {
        return status;
    }

    return MRF24J40_OK;
}

/**
 * @brief Translate a port-layer status code to the driver-layer status space.
 *
 * @param status Status code returned by the MRF24J40 port layer.
 *
 * @return Equivalent status code in the @ref mrf24j40_status_t domain.
 *
 * @details
 * This helper preserves the abstraction boundary between the driver and the
 * underlying port layer by converting low-level access results into the
 * public driver status type.
 *
 * Any unknown port-layer status is conservatively mapped to
 * MRF24J40_E_HW.
 */
static mrf24j40_status_t mrf24j40_from_port_status(
    mrf24j40_port_status_t status)
{
    mrf24j40_status_t ret;

    switch (status)
    {
        case MRF24J40_PORT_OK:
            ret = MRF24J40_OK;
            break;

        case MRF24J40_PORT_E_NULL:
            ret = MRF24J40_E_NULL;
            break;

        case MRF24J40_PORT_E_PARAM:
            ret = MRF24J40_E_PARAM;
            break;

        case MRF24J40_PORT_E_HW:
            ret = MRF24J40_E_HW;
            break;

        case MRF24J40_PORT_E_TIMEOUT:
            ret = MRF24J40_E_TIMEOUT;
            break;

        default:
            ret = MRF24J40_E_HW;
            break;
    }

    return ret;
}

/** @} */
