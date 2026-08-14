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
 * @file    mrf24j40.h
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-02-23
 * @brief   MRF24J40 driver (MiWi-oriented low-level PHY/MAC access).
 *
 * @details
 * This driver provides:
 *  - basic transceiver initialization
 *  - RX FIFO safe read
 *  - TX Normal FIFO write + trigger (ACK optional)
 *  - TX completion and transmission result reporting
 *  - basic configuration helpers (PAN ID, short address, extended address)
 *  - interrupt event decoding
 *
 * It does NOT implement MiWi itself; it exposes primitives used by a MiWi stack.
 *
 * Public functions return @ref mrf24j40_status_t so that:
 *  - hardware and timeout errors propagated from the port layer can be reported
 *  - driver-level state and frame validation errors can be distinguished
 *
 * The MRF24J40 hardware performs acknowledgment handling and automatic
 * retransmissions when acknowledgment is requested. Transmission completion
 * information is obtained from TXSTAT after a TXNIF event and exposed through
 * @ref mrf24j40_tx_result_t.
 *
 * @ingroup mrf24j40
 * @{
 */

#ifndef MRF24J40_H_
#define MRF24J40_H_

/* ============================= Includes ================================== */

#include <stdint.h>
#include <stdbool.h>

/* ============================== Macros =================================== */

/**
 * @brief Maximum MAC frame size handled by the driver.
 *
 * @details
 * The MRF24J40 RXFIFO reports received frame length including the 2-byte FCS.
 * This driver hides the FCS from the public packet representation so that the
 * same packet type can be used consistently for both RX and TX.
 *
 * Therefore, the public frame buffer stores only:
 * - MAC Header (MHR)
 * - payload
 *
 * Maximum IEEE 802.15.4 PSDU size is 127 bytes, including FCS.
 * Public usable frame size is therefore 125 bytes.
 */
#define MRF24J40_MAX_FRAME_SIZE        ((uint8_t)125u)

/* ============================== Types ==================================== */

/**
 * @brief Status codes returned by the MRF24J40 driver.
 *
 * @details
 * These codes include:
 * - errors propagated from the underlying port layer
 * - errors detected by this driver layer itself
 *
 * Driver-level errors are used to report conditions such as:
 * - invalid API usage
 * - invalid internal state for the requested operation
 * - missing RX packet indication
 * - invalid received frame format or length
 */
typedef enum
{
    MRF24J40_OK = 0,
    MRF24J40_E_NULL,
    MRF24J40_E_PARAM,
    MRF24J40_E_HW,
    MRF24J40_E_TIMEOUT,
    MRF24J40_E_STATE,
    MRF24J40_E_NO_RX_PACKET,
    MRF24J40_E_FRAME
} mrf24j40_status_t;

/**
 * @brief MRF24J40 runtime context.
 *
 * @details
 * This structure stores basic runtime state used by the driver.
 *
 * The acknowledgment request state of the current TX Normal FIFO transaction
 * is retained so that the TX result can distinguish between transmissions that
 * required an acknowledgment and transmissions that did not.
 */
typedef struct
{
    volatile bool int_pending;
    volatile bool rx_pending;
    volatile bool tx_complete;
    volatile bool tx_ack_requested;
} mrf24j40_context_t;

/**
 * @brief MAC frame container used by both RX and TX paths.
 *
 * @details
 * This structure stores:
 * - MAC frame length, excluding FCS
 * - MAC frame bytes (MHR + payload)
 * - link quality indicator
 * - received signal strength indicator
 *
 * The @p lqi and @p rssi fields are only meaningful for received packets.
 */
typedef struct
{
    uint8_t frame_length;
    uint8_t frame[MRF24J40_MAX_FRAME_SIZE];
    uint8_t lqi;
    uint8_t rssi;
} mrf24j40_packet_t;

/**
 * @brief Result of the most recent TX Normal FIFO transmission.
 *
 * @details
 * This structure reports the transmission result obtained from the MRF24J40
 * TXSTAT register after a TXNIF event.
 *
 * Fields:
 *
 * - complete:
 *   A TX Normal FIFO transaction has completed and the remaining fields contain
 *   valid information.
 *
 * - success:
 *   The MRF24J40 reports the transmission as successful.
 *
 * - ack_requested:
 *   The completed transaction had acknowledgment handling enabled through
 *   TXNACKREQ.
 *
 * - ack_received:
 *   An acknowledgment was requested and the transmission completed
 *   successfully.
 *
 * - retry_limit_reached:
 *   The transmission failed after the hardware retry mechanism was exhausted.
 *   This condition excludes failures explicitly identified as CCA failures.
 *
 * - cca_failed:
 *   The most recent transmission failed because the CSMA-CA procedure could
 *   not obtain an idle channel.
 *
 * - retries:
 *   Number of hardware retries reported by TXNRETRY<1:0>.
 *
 * @note
 * When @p ack_requested is false, @p ack_received remains false even when the
 * transmission itself is successful.
 */
typedef struct
{
    bool complete;
    bool success;
    bool ack_requested;
    bool ack_received;
    bool retry_limit_reached;
    bool cca_failed;
    uint8_t retries;
} mrf24j40_tx_result_t;

/* ===================== Public Function Prototypes ======================== */

/**
 * @brief Initialize the MRF24J40 with the basic datasheet-recommended setup.
 *
 * @return
 * - MRF24J40_OK: Initialization completed successfully.
 * - MRF24J40_E_HW: A hardware communication error occurred.
 * - MRF24J40_E_TIMEOUT: A port-layer transaction timed out.
 * - MRF24J40_E_PARAM: An invalid register access parameter was detected by the
 *   port layer.
 *
 * @details
 * This function performs the base transceiver initialization recommended by the
 * datasheet for normal operation.
 *
 * The sequence performed is:
 * - software reset
 * - base MAC/RF timing configuration
 * - PLL enable
 * - sleep clock source selection
 * - nonbeacon CCA configuration
 * - RSSI append to RX FIFO
 * - interrupt enable
 * - default RF channel selection
 * - default TX power setup
 * - RF state machine reset
 *
 * This function does not configure the node role in the network. The caller
 * must explicitly select the desired MAC operating role afterwards, for
 * example:
 * - mrf24j40_configure_nonbeacon_device()
 * - mrf24j40_configure_nonbeacon_pan_coordinator()
 *
 * Assumptions:
 * - The device power supply and external oscillator are already stable.
 * - The SPI peripheral is already initialized.
 * - Low-level short/long register access functions are operational.
 *
 * @note
 * This is a minimal base-case initialization intended for early bring-up.
 * PAN ID, addresses and higher-layer protocol settings are expected to be
 * configured separately.
 */
mrf24j40_status_t mrf24j40_init(void);

/**
 * @brief Configure the MRF24J40 as a nonbeacon-enabled PAN coordinator.
 *
 * @return
 * - MRF24J40_OK: Configuration completed successfully.
 * - MRF24J40_E_HW: A hardware communication error occurred.
 * - MRF24J40_E_TIMEOUT: A port-layer transaction timed out.
 * - MRF24J40_E_PARAM: An invalid register access parameter was detected by the
 *   port layer.
 *
 * @details
 * This function applies the MAC settings required by the datasheet for a
 * nonbeacon-enabled coordinator:
 * - PANCOORD = 1
 * - SLOTTED = 0
 * - BO = 15
 * - SO = 15
 *
 * @note
 * This function configures only the MAC operating mode and coordinator role.
 * It does not assign PAN ID, short address or extended address.
 */
mrf24j40_status_t mrf24j40_configure_nonbeacon_pan_coordinator(void);

/**
 * @brief Configure the MRF24J40 as a nonbeacon-enabled device.
 *
 * @return
 * - MRF24J40_OK: Configuration completed successfully.
 * - MRF24J40_E_HW: A hardware communication error occurred.
 * - MRF24J40_E_TIMEOUT: A port-layer transaction timed out.
 * - MRF24J40_E_PARAM: An invalid register access parameter was detected by the
 *   port layer.
 *
 * @details
 * This function applies the MAC settings required by the datasheet for a
 * nonbeacon-enabled device:
 * - PANCOORD = 0
 * - SLOTTED = 0
 * - BO = 15
 * - SO = 15
 *
 * @note
 * This function configures only the MAC operating mode and device role.
 * It does not assign PAN ID, short address or extended address.
 */
mrf24j40_status_t mrf24j40_configure_nonbeacon_device(void);

/**
 * @brief Set the PAN ID of the device.
 *
 * @param pan_id 16-bit PAN identifier.
 *
 * @return
 * - MRF24J40_OK: PAN ID programmed successfully.
 * - MRF24J40_E_HW: A hardware communication error occurred.
 * - MRF24J40_E_TIMEOUT: A port-layer transaction timed out.
 * - MRF24J40_E_PARAM: An invalid register access parameter was detected by the
 *   port layer.
 *
 * @details
 * The PAN ID identifies the network to which the device belongs.
 * This value must match across all nodes in the same network.
 *
 * The PAN ID is stored in:
 * - PANIDL (LSB)
 * - PANIDH (MSB)
 */
mrf24j40_status_t mrf24j40_set_pan_id(uint16_t pan_id);

/**
 * @brief Set the short address of the device.
 *
 * @param short_address 16-bit short address.
 *
 * @return
 * - MRF24J40_OK: Short address programmed successfully.
 * - MRF24J40_E_HW: A hardware communication error occurred.
 * - MRF24J40_E_TIMEOUT: A port-layer transaction timed out.
 * - MRF24J40_E_PARAM: An invalid register access parameter was detected by the
 *   port layer.
 *
 * @details
 * The short address identifies the node within the PAN.
 * This value is stored in:
 * - SADRL (LSB)
 * - SADRH (MSB)
 */
mrf24j40_status_t mrf24j40_set_short_address(uint16_t short_address);

/**
 * @brief Generate and program the extended address from the STM32 unique ID.
 *
 * @return
 * - MRF24J40_OK: Extended address programmed successfully.
 * - MRF24J40_E_HW: A hardware communication error occurred.
 * - MRF24J40_E_TIMEOUT: A port-layer transaction timed out.
 * - MRF24J40_E_PARAM: An invalid register access parameter was detected by the
 *   port layer.
 *
 * @details
 * This function reads the 96-bit STM32 unique device identifier, computes a
 * deterministic 64-bit CRC-based value from it, adjusts the resulting address
 * as a locally administered unicast identifier, and writes it into:
 * - EADR0
 * - EADR1
 * - EADR2
 * - EADR3
 * - EADR4
 * - EADR5
 * - EADR6
 * - EADR7
 *
 * The generated value is deterministic for a given microcontroller and can be
 * used as a practical extended address in closed systems.
 *
 * @note
 * The resulting address is not an IEEE-assigned global EUI-64.
 */
mrf24j40_status_t mrf24j40_set_extended_address(void);

/**
 * @brief Notify the driver that the MRF24J40 INT pin asserted.
 *
 * @details
 * This function is intended to be called from the external interrupt callback
 * in application code when the MRF24J40 INT pin asserts.
 *
 * The specific interrupt source is resolved later by reading INTSTAT in
 * mrf24j40_update_interrupt_flags().
 */
void mrf24j40_set_interrupt_pending(void);

/**
 * @brief Decode the pending MRF24J40 interrupt source flags.
 *
 * @return
 * - MRF24J40_OK: Interrupt flags were decoded successfully.
 * - MRF24J40_E_STATE: No pending interrupt had been previously latched.
 * - MRF24J40_E_HW: A hardware communication error occurred.
 * - MRF24J40_E_TIMEOUT: A port-layer transaction timed out.
 * - MRF24J40_E_PARAM: An invalid register access parameter was detected by the
 *   port layer.
 *
 * @details
 * If a pending INT pin event was previously latched by
 * mrf24j40_set_interrupt_pending(), this function reads INTSTAT and updates
 * the internal driver flags accordingly.
 *
 * Only RXIF and TXNIF are currently used by this minimal driver.
 *
 * A TXNIF condition only latches the TX completion state. TXSTAT is read later
 * from thread context by mrf24j40_get_tx_result().
 *
 * @note
 * INTSTAT bits are cleared by hardware when the register is read.
 */
mrf24j40_status_t mrf24j40_update_interrupt_flags(void);

/**
 * @brief Read one received packet from the RXFIFO.
 *
 * @param p_packet Pointer to packet container.
 *
 * @return
 * - MRF24J40_OK: RX packet was read successfully.
 * - MRF24J40_E_NULL: Null pointer passed in @p p_packet.
 * - MRF24J40_E_NO_RX_PACKET: No RX packet was pending.
 * - MRF24J40_E_FRAME: The received frame length was invalid.
 * - MRF24J40_E_HW: A hardware communication error occurred.
 * - MRF24J40_E_TIMEOUT: A port-layer transaction timed out.
 * - MRF24J40_E_PARAM: An invalid register access parameter was detected by the
 *   port layer.
 *
 * @details
 * This function follows the basic RXFIFO read sequence described in the
 * datasheet:
 * - check pending RX indication
 * - disable host interrupts
 * - set RXDECINV = 1
 * - read RXFIFO length
 * - read frame bytes, LQI and RSSI
 * - clear RXDECINV = 0
 * - enable host interrupts
 *
 * On successful completion, the internal RX pending flag is cleared.
 */
mrf24j40_status_t mrf24j40_read_rx_fifo(mrf24j40_packet_t * p_packet);

/**
 * @brief Load and trigger transmission through the TX Normal FIFO.
 *
 * @param p_packet Pointer to MAC frame container.
 * @param ack_request Set to true if an acknowledgment is expected.
 *
 * @return
 * - MRF24J40_OK: The frame was loaded and transmission was triggered.
 * - MRF24J40_E_NULL: Null pointer passed in @p p_packet.
 * - MRF24J40_E_PARAM: Invalid frame length or invalid register access
 *   parameter detected by the port layer.
 * - MRF24J40_E_HW: A hardware communication error occurred.
 * - MRF24J40_E_TIMEOUT: A port-layer transaction timed out.
 *
 * @details
 * The packet format used by this driver is:
 * - frame_length = MHR + payload
 * - frame[]      = MHR + payload
 *
 * The FCS is not part of the public packet representation. The MRF24J40
 * appends the FCS automatically during transmission.
 *
 * This function clears the previous TX complete indication, stores whether
 * acknowledgment was requested, loads the TX Normal FIFO and triggers
 * transmission.
 *
 * If @p ack_request is true, TXNACKREQ is set and acknowledgment monitoring
 * and retransmission are delegated to the MRF24J40 hardware.
 *
 * @note
 * If @p ack_request is true, the caller must also set the acknowledgment
 * request bit in the MAC Frame Control field.
 */
mrf24j40_status_t mrf24j40_write_tx_normal_fifo(
    const mrf24j40_packet_t * p_packet,
    bool ack_request);

/**
 * @brief Get and consume the result of the most recent TX Normal FIFO
 *        transmission.
 *
 * @param[out] p_result Pointer where the transmission result will be stored.
 *
 * @return
 * - MRF24J40_OK: TX state was obtained successfully.
 * - MRF24J40_E_NULL: Null pointer passed in @p p_result.
 * - MRF24J40_E_HW: A hardware communication error occurred while reading
 *   TXSTAT.
 * - MRF24J40_E_TIMEOUT: A port-layer transaction timed out while reading
 *   TXSTAT.
 * - MRF24J40_E_PARAM: An invalid register access parameter was detected by the
 *   port layer.
 *
 * @details
 * If no TXNIF event is pending, the function returns MRF24J40_OK with
 * p_result->complete set to false and does not access TXSTAT.
 *
 * If a TXNIF event is pending, TXSTAT is read and the following information is
 * returned:
 *
 * - transmission success or failure
 * - whether acknowledgment was requested
 * - whether the requested acknowledgment was received
 * - whether the hardware retry limit was reached
 * - whether CSMA-CA failed because the channel remained busy
 * - number of hardware retries
 *
 * The internal TX completion indication is consumed only after TXSTAT has been
 * read successfully.
 *
 * @note
 * This function is the preferred API for handling TX completion because it
 * preserves the diagnostic information provided by the MRF24J40 hardware.
 */
mrf24j40_status_t mrf24j40_get_tx_result(
    mrf24j40_tx_result_t * const p_result);

#endif /* MRF24J40_H_ */

/** @} */
