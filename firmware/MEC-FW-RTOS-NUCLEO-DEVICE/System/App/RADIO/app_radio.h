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
 * @file    app_radio.h
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-05-31
 * @brief   Radio application task interface.
 *
 * @details
 * This module provides a minimal CMSIS-RTOS2-based interface for communication
 * between one PAN coordinator and up to 16 remote devices using the MRF24J40.
 *
 * The network uses fixed 16-bit IEEE 802.15.4 short addresses:
 *
 * - PAN coordinator: 0x0000.
 * - Remote devices:  0x0001 through 0x0010.
 *
 * Node role and local short address are selected at compile time using:
 *
 * - APP_RADIO_NODE_ROLE.
 * - APP_RADIO_SHORT_ADDRESS.
 *
 * The network operates in nonbeacon mode using short addressing and a fixed
 * PAN ID. No association, discovery or routing procedure is implemented.
 *
 * The public API exposes only:
 *
 * - app_radio_send(): enqueue one byte for transmission to another node.
 * - app_radio_receive(): read one received byte and its source address.
 * - app_radio_task(): task entry point used by the RTOS.
 *
 * All transmitted data frames request an IEEE 802.15.4 acknowledgment.
 * Acknowledgment generation, acknowledgment timeout and retransmission are
 * delegated to the MRF24J40 hardware.
 *
 * When a transmission completes, the radio task obtains the final hardware
 * result from the MRF24J40 driver. This allows the application layer to
 * distinguish between:
 *
 * - successful acknowledged transmission
 * - acknowledgment failure after hardware retransmissions
 * - CSMA-CA channel access failure
 *
 * Internal task wake-up events are separated from public fault reporting:
 *
 * - radioEventHandle is used only to wake the radio task.
 * - radioFaultHandle is used to report deferred driver, transmission or RTOS
 *   faults.
 *
 * @note
 * radioInputQueueHandle and radioOutputQueueHandle must be configured with an
 * element size of 4 bytes because radio messages are internally represented as
 * uint32_t values containing one 16-bit node address and one 8-bit payload.
 *
 * @ingroup app_radio
 * @{
 */

#ifndef APP_RADIO_H_
#define APP_RADIO_H_

/* ============================= Includes ================================== */

#include <stdint.h>

/* ============================= Defines =================================== */

/**
 * @brief Device role identifier.
 */
#define APP_RADIO_ROLE_DEVICE                 ((uint8_t)0u)

/**
 * @brief PAN coordinator role identifier.
 */
#define APP_RADIO_ROLE_PAN_COORDINATOR        ((uint8_t)1u)

/**
 * @brief Local node role.
 *
 * @details
 * Select one of:
 *
 * - APP_RADIO_ROLE_PAN_COORDINATOR.
 * - APP_RADIO_ROLE_DEVICE.
 */
#define APP_RADIO_NODE_ROLE                   APP_RADIO_ROLE_DEVICE

/**
 * @brief Local IEEE 802.15.4 short address.
 *
 * @details
 * Valid values are:
 *
 * - 0x0000 for the PAN coordinator.
 * - 0x0001 through 0x0010 for remote devices.
 *
 * This value must be changed together with APP_RADIO_NODE_ROLE when building
 * the firmware for a different node.
 */
#define APP_RADIO_SHORT_ADDRESS               ((uint16_t)0x0001u)

/**
 * @brief PAN coordinator fixed short address.
 */
#define APP_RADIO_COORDINATOR_ADDRESS         ((uint16_t)0x0000u)

/**
 * @brief First valid remote device short address.
 */
#define APP_RADIO_FIRST_DEVICE_ADDRESS        ((uint16_t)0x0001u)

/**
 * @brief Last valid remote device short address.
 */
#define APP_RADIO_LAST_DEVICE_ADDRESS         ((uint16_t)0x0010u)

/**
 * @brief Number of remote devices supported by the network.
 */
#define APP_RADIO_MAX_REMOTE_DEVICES          ((uint8_t)16u)

/**
 * @brief MRF24J40 interrupt event.
 *
 * @details
 * This internal event wakes the radio task when the MRF24J40 INT pin asserts.
 */
#define APP_RADIO_EVT_IRQ                     ((uint32_t)(1u << 0u))

/**
 * @brief TX queue ready event.
 *
 * @details
 * This internal event wakes the radio task after a new message is enqueued for
 * transmission.
 */
#define APP_RADIO_EVT_TX_READY                ((uint32_t)(1u << 1u))

/**
 * @brief Generic hardware or low-level access fault.
 */
#define APP_RADIO_FAULT_HW_ERROR              ((uint32_t)(1u << 0u))

/**
 * @brief Radio driver or low-level port timeout fault.
 */
#define APP_RADIO_FAULT_TIMEOUT               ((uint32_t)(1u << 1u))

/**
 * @brief Invalid parameter fault reported by the radio driver.
 */
#define APP_RADIO_FAULT_PARAM_ERROR           ((uint32_t)(1u << 2u))

/**
 * @brief Null pointer fault reported by the radio driver.
 */
#define APP_RADIO_FAULT_NULL_ERROR            ((uint32_t)(1u << 3u))

/**
 * @brief Invalid received frame fault.
 */
#define APP_RADIO_FAULT_FRAME_ERROR           ((uint32_t)(1u << 4u))

/**
 * @brief Invalid internal driver state fault.
 */
#define APP_RADIO_FAULT_STATE_ERROR           ((uint32_t)(1u << 5u))

/**
 * @brief RX application queue full fault.
 */
#define APP_RADIO_FAULT_RX_QUEUE_FULL         ((uint32_t)(1u << 6u))

/**
 * @brief TX application queue full fault.
 */
#define APP_RADIO_FAULT_TX_QUEUE_FULL         ((uint32_t)(1u << 7u))

/**
 * @brief Unexpected CMSIS-RTOS2 object or service fault.
 */
#define APP_RADIO_FAULT_OS_ERROR              ((uint32_t)(1u << 8u))

/**
 * @brief TX acknowledgment failure.
 *
 * @details
 * This fault indicates that an IEEE 802.15.4 acknowledgment was requested but
 * the MRF24J40 could not complete the transmission successfully after its
 * hardware retransmission mechanism was exhausted.
 *
 * Retransmission is performed entirely by the MRF24J40. No application-level
 * retry is performed by app_radio.
 */
#define APP_RADIO_FAULT_TX_NO_ACK             ((uint32_t)(1u << 9u))

/**
 * @brief TX CSMA-CA channel access failure.
 *
 * @details
 * This fault indicates that the MRF24J40 could not obtain an idle channel for
 * the transmission according to its CSMA-CA procedure.
 *
 * This condition is kept separate from APP_RADIO_FAULT_TX_NO_ACK because no
 * transmitted frame necessarily reached the destination in this case.
 */
#define APP_RADIO_FAULT_TX_CCA_FAILED         ((uint32_t)(1u << 10u))

/**
 * @brief Mask containing all radio fault flags.
 */
#define APP_RADIO_FAULT_ERROR_MASK            (APP_RADIO_FAULT_HW_ERROR       | \
                                               APP_RADIO_FAULT_TIMEOUT        | \
                                               APP_RADIO_FAULT_PARAM_ERROR    | \
                                               APP_RADIO_FAULT_NULL_ERROR     | \
                                               APP_RADIO_FAULT_FRAME_ERROR    | \
                                               APP_RADIO_FAULT_STATE_ERROR    | \
                                               APP_RADIO_FAULT_RX_QUEUE_FULL  | \
                                               APP_RADIO_FAULT_TX_QUEUE_FULL  | \
                                               APP_RADIO_FAULT_OS_ERROR       | \
                                               APP_RADIO_FAULT_TX_NO_ACK      | \
                                               APP_RADIO_FAULT_TX_CCA_FAILED)

/* ============================== Types ==================================== */

/**
 * @brief Status codes returned by the radio application API.
 *
 * @details
 * These status codes report only immediate API-level results.
 *
 * APP_RADIO_OK returned by app_radio_send() means that the message was
 * successfully enqueued for transmission. It does not mean that the frame has
 * already been transmitted or acknowledged.
 *
 * Deferred faults detected inside app_radio_task() are reported through
 * radioFaultHandle.
 *
 * Transmission acknowledgment failure is therefore reported asynchronously
 * using APP_RADIO_FAULT_TX_NO_ACK rather than through the return value of
 * app_radio_send().
 */
typedef enum
{
    APP_RADIO_OK = 0,
    APP_RADIO_E_NULL,
    APP_RADIO_E_PARAM,
    APP_RADIO_E_QUEUE_EMPTY,
    APP_RADIO_E_QUEUE_FULL,
    APP_RADIO_E_OS
} app_radio_status_t;

/* ===================== Public Function Prototypes ======================== */

/**
 * @brief Radio application task.
 *
 * @param argument Task argument provided by the RTOS.
 *
 * @details
 * This task is the only application-level owner of the MRF24J40 driver.
 *
 * It waits for internal wake-up events:
 *
 * - APP_RADIO_EVT_IRQ: process MRF24J40 interrupt sources.
 * - APP_RADIO_EVT_TX_READY: process pending TX queue data.
 *
 * Received IEEE 802.15.4 data frames are decoded and the source address and
 * one-byte payload are placed into the RX queue.
 *
 * Pending transmit messages are converted into IEEE 802.15.4 data frames and
 * written to the MRF24J40 TX Normal FIFO.
 *
 * When a TX Normal FIFO completion interrupt is received, the task obtains the
 * detailed result from the MRF24J40 driver and reports acknowledgment or
 * channel access failures through radioFaultHandle.
 */
void app_radio_task(void * argument);

/**
 * @brief Enqueue one byte for radio transmission.
 *
 * @param destination Short address of the destination node.
 * @param data Byte to be transmitted.
 *
 * @return
 * - APP_RADIO_OK: Message enqueued successfully.
 * - APP_RADIO_E_PARAM: Invalid destination address.
 * - APP_RADIO_E_QUEUE_FULL: TX queue is full.
 * - APP_RADIO_E_OS: Unexpected RTOS error.
 *
 * @details
 * The PAN coordinator can transmit to addresses 0x0001 through 0x0010.
 *
 * A remote device can transmit only to the PAN coordinator at address 0x0000.
 *
 * This function does not access the MRF24J40 directly. It stores the
 * destination address and payload in the TX queue and wakes the radio task
 * using APP_RADIO_EVT_TX_READY.
 *
 * The generated IEEE 802.15.4 data frame requests an acknowledgment. ACK
 * handling and retransmission are performed by the MRF24J40 hardware.
 *
 * The final transmission result is obtained asynchronously by app_radio_task().
 */
app_radio_status_t app_radio_send(uint16_t destination, uint8_t data);

/**
 * @brief Read one received radio message from the RX queue.
 *
 * @param[out] p_source Pointer where the sender short address will be stored.
 * @param[out] p_data Pointer where the received byte will be stored.
 *
 * @return
 * - APP_RADIO_OK: One message was read successfully.
 * - APP_RADIO_E_NULL: A null output pointer was provided.
 * - APP_RADIO_E_QUEUE_EMPTY: No received message is currently available.
 * - APP_RADIO_E_OS: Unexpected RTOS error.
 *
 * @details
 * This function is non-blocking. It checks the RX queue once and returns
 * immediately.
 */
app_radio_status_t app_radio_receive(uint16_t * p_source, uint8_t * p_data);

#endif /* APP_RADIO_H_ */

/** @} */
