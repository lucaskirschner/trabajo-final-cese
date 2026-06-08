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
 * @file    app_radio.h
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-05-31
 * @brief   Radio application task interface.
 *
 * @details
 * This module provides a minimal CMSIS-RTOS2-based interface for the MRF24J40
 * radio application task.
 *
 * The public API exposes only:
 * - app_radio_send(): enqueue one byte for transmission.
 * - app_radio_receive(): read one received byte from the RX queue.
 * - app_radio_task(): task entry point used by the RTOS.
 *
 * Internal task wake-up events are separated from public fault reporting:
 * - radioEventHandle is used only to wake the radio task.
 * - radioFaultEventHandle is used to report deferred driver or RTOS faults.
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
 * @brief MRF24J40 interrupt event.
 *
 * @details
 * This internal event wakes the radio task when the MRF24J40 INT pin asserts.
 */
#define APP_RADIO_EVT_IRQ                 ((uint32_t)(1u << 0u))

/**
 * @brief TX queue ready event.
 *
 * @details
 * This internal event wakes the radio task after a new byte is enqueued for
 * transmission.
 */
#define APP_RADIO_EVT_TX_READY            ((uint32_t)(1u << 1u))

/**
 * @brief Generic hardware or low-level access fault.
 */
#define APP_RADIO_FAULT_HW_ERROR          ((uint32_t)(1u << 0u))

/**
 * @brief Radio driver or low-level port timeout fault.
 */
#define APP_RADIO_FAULT_TIMEOUT           ((uint32_t)(1u << 1u))

/**
 * @brief Invalid parameter fault reported by the radio driver.
 */
#define APP_RADIO_FAULT_PARAM_ERROR       ((uint32_t)(1u << 2u))

/**
 * @brief Null pointer fault reported by the radio driver.
 */
#define APP_RADIO_FAULT_NULL_ERROR        ((uint32_t)(1u << 3u))

/**
 * @brief Invalid received frame fault.
 */
#define APP_RADIO_FAULT_FRAME_ERROR       ((uint32_t)(1u << 4u))

/**
 * @brief Invalid internal driver state fault.
 */
#define APP_RADIO_FAULT_STATE_ERROR       ((uint32_t)(1u << 5u))

/**
 * @brief RX application queue full fault.
 */
#define APP_RADIO_FAULT_RX_QUEUE_FULL     ((uint32_t)(1u << 6u))

/**
 * @brief TX application queue full fault.
 */
#define APP_RADIO_FAULT_TX_QUEUE_FULL     ((uint32_t)(1u << 7u))

/**
 * @brief Unexpected CMSIS-RTOS2 object or service fault.
 */
#define APP_RADIO_FAULT_OS_ERROR          ((uint32_t)(1u << 8u))

/**
 * @brief Mask containing all radio fault flags.
 */
#define APP_RADIO_FAULT_ERROR_MASK        (APP_RADIO_FAULT_HW_ERROR      | \
                                           APP_RADIO_FAULT_TIMEOUT       | \
                                           APP_RADIO_FAULT_PARAM_ERROR   | \
                                           APP_RADIO_FAULT_NULL_ERROR    | \
                                           APP_RADIO_FAULT_FRAME_ERROR   | \
                                           APP_RADIO_FAULT_STATE_ERROR   | \
                                           APP_RADIO_FAULT_RX_QUEUE_FULL | \
                                           APP_RADIO_FAULT_TX_QUEUE_FULL | \
                                           APP_RADIO_FAULT_OS_ERROR)

/* ============================== Types ==================================== */

/**
 * @brief Status codes returned by the radio application API.
 *
 * @details
 * These status codes report only immediate API-level results.
 *
 * For example, APP_RADIO_OK returned by app_radio_send() means that the byte
 * was successfully enqueued for transmission. It does not mean that the byte
 * has already been physically transmitted by the radio.
 *
 * Deferred faults detected inside app_radio_task() are reported through
 * radioFaultEventHandle.
 */
typedef enum
{
    APP_RADIO_OK = 0,
    APP_RADIO_E_NULL,
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
 * - APP_RADIO_EVT_IRQ: process MRF24J40 interrupt sources.
 * - APP_RADIO_EVT_TX_READY: process pending TX queue data.
 *
 * Received bytes are extracted from radio frames and placed into the RX queue.
 * Transmit bytes are taken from the TX queue and written to the radio driver.
 */
void app_radio_task(void * argument);

#endif /* APP_RADIO_H_ */

/** @} */
