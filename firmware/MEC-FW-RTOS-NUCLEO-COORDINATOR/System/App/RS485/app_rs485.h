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
 * @file    app_rs485.h
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-08-04
 * @brief   RS485 validation application task interface.
 *
 * @details
 * This module provides a minimal CMSIS-RTOS2-based interface for the simple
 * four-byte RS485 echo protocol implemented by rs485.c.
 *
 * The public API exposes:
 *
 * - app_rs485_send(): Enqueue one data byte for transmission.
 * - app_rs485_receive(): Read one valid received data byte.
 * - app_rs485_task(): Task entry point used by the RTOS.
 *
 * The task is the only application-level owner of the RS485 driver. External
 * application code does not call rs485_send_frame(), rs485_receive_start(),
 * rs485_receive_frame() or rs485_receive_abort() directly.
 *
 * RX path:
 *
 * - The task starts an interrupt-driven reception of one four-byte frame.
 * - APP_RS485_EVT_RX_READY wakes the task when reception completes.
 * - APP_RS485_EVT_RX_ERROR wakes the task when UART7 reports an error.
 * - The completed frame is validated and decoded in task context.
 * - Valid ECHO_REQUEST and ECHO_RESPONSE data bytes are copied to the RX
 *   application queue.
 * - An ECHO_REQUEST is answered automatically with an ECHO_RESPONSE containing
 *   the same data byte.
 *
 * TX path:
 *
 * - app_rs485_send() places one byte in the TX application queue.
 * - APP_RS485_EVT_TX_READY notifies the task that pending data is available.
 * - The task aborts any active reception before changing the half-duplex
 *   transceiver to transmit mode.
 * - The queued byte is transmitted using an ECHO_REQUEST frame.
 * - Interrupt-driven reception is restarted after transmission.
 *
 * Internal task wake-up events are separated from public fault reporting:
 *
 * - rs485EventHandle is used for internal task notification.
 * - rs485FaultHandle is used to report deferred driver, protocol or RTOS
 *   faults.
 *
 * The API is intended for functional RS485 validation and is not an industrial
 * communication protocol.
 *
 * @ingroup app_rs485
 * @{
 */

#ifndef APP_RS485_H_
#define APP_RS485_H_

/* ============================= Includes ================================== */

#include <stdint.h>

/* ============================= Defines =================================== */

/**
 * @brief TX application queue ready event.
 *
 * @details
 * This event indicates that one or more data bytes are pending in the RS485 TX
 * application queue.
 */
#define APP_RS485_EVT_TX_READY             ((uint32_t)(1u << 0u))

/**
 * @brief Complete RS485 frame received event.
 *
 * @details
 * This event is generated from UART7 interrupt context after the complete
 * four-byte frame has been received.
 *
 * The event wakes app_rs485_task(), where frame validation and protocol
 * processing are performed.
 */
#define APP_RS485_EVT_RX_READY             ((uint32_t)(1u << 1u))

/**
 * @brief UART7 reception error event.
 *
 * @details
 * This event is generated from UART7 interrupt context when the HAL reports a
 * reception error.
 *
 * The event wakes app_rs485_task(), where the pending driver error is consumed
 * and the reception is restarted.
 */
#define APP_RS485_EVT_RX_ERROR             ((uint32_t)(1u << 2u))

/**
 * @brief Mask containing all internal RS485 task wake-up events.
 */
#define APP_RS485_EVT_MASK                 (APP_RS485_EVT_TX_READY | \
                                            APP_RS485_EVT_RX_READY | \
                                            APP_RS485_EVT_RX_ERROR)

/**
 * @brief RS485 driver initialization fault.
 */
#define APP_RS485_FAULT_INIT_ERROR          ((uint32_t)(1u << 0u))

/**
 * @brief Generic hardware or low-level access fault.
 */
#define APP_RS485_FAULT_HW_ERROR            ((uint32_t)(1u << 1u))

/**
 * @brief RS485 blocking transmission timeout fault.
 *
 * @details
 * Interrupt-driven reception does not use an application polling timeout.
 * This flag is reserved for timeout conditions reported by blocking
 * transmission or another lower-layer operation.
 */
#define APP_RS485_FAULT_TIMEOUT             ((uint32_t)(1u << 2u))

/**
 * @brief Invalid parameter fault reported by the RS485 driver.
 */
#define APP_RS485_FAULT_PARAM_ERROR         ((uint32_t)(1u << 3u))

/**
 * @brief Null pointer fault reported by the RS485 driver.
 */
#define APP_RS485_FAULT_NULL_ERROR          ((uint32_t)(1u << 4u))

/**
 * @brief Invalid internal driver state fault.
 */
#define APP_RS485_FAULT_STATE_ERROR         ((uint32_t)(1u << 5u))

/**
 * @brief Invalid start-of-frame marker fault.
 */
#define APP_RS485_FAULT_SOF_ERROR           ((uint32_t)(1u << 6u))

/**
 * @brief Invalid frame checksum fault.
 */
#define APP_RS485_FAULT_CHECKSUM_ERROR      ((uint32_t)(1u << 7u))

/**
 * @brief Unsupported or unexpected protocol command fault.
 */
#define APP_RS485_FAULT_COMMAND_ERROR       ((uint32_t)(1u << 8u))

/**
 * @brief RX application queue full fault.
 */
#define APP_RS485_FAULT_RX_QUEUE_FULL       ((uint32_t)(1u << 9u))

/**
 * @brief TX application queue full fault.
 */
#define APP_RS485_FAULT_TX_QUEUE_FULL       ((uint32_t)(1u << 10u))

/**
 * @brief Unexpected CMSIS-RTOS2 object or service fault.
 */
#define APP_RS485_FAULT_OS_ERROR            ((uint32_t)(1u << 11u))

/**
 * @brief Mask containing all RS485 application fault flags.
 */
#define APP_RS485_FAULT_ERROR_MASK          \
    (APP_RS485_FAULT_INIT_ERROR     |       \
     APP_RS485_FAULT_HW_ERROR       |       \
     APP_RS485_FAULT_TIMEOUT        |       \
     APP_RS485_FAULT_PARAM_ERROR    |       \
     APP_RS485_FAULT_NULL_ERROR     |       \
     APP_RS485_FAULT_STATE_ERROR    |       \
     APP_RS485_FAULT_SOF_ERROR      |       \
     APP_RS485_FAULT_CHECKSUM_ERROR |       \
     APP_RS485_FAULT_COMMAND_ERROR  |       \
     APP_RS485_FAULT_RX_QUEUE_FULL  |       \
     APP_RS485_FAULT_TX_QUEUE_FULL  |       \
     APP_RS485_FAULT_OS_ERROR)

/* ============================== Types ==================================== */

/**
 * @brief Status codes returned by the RS485 application API.
 *
 * @details
 * These values report only immediate API-level queue operations.
 *
 * APP_RS485_OK returned by app_rs485_send() means that the data byte was
 * successfully enqueued. It does not guarantee that the corresponding frame
 * has already been physically transmitted.
 *
 * Deferred driver, protocol and RTOS faults detected by app_rs485_task() are
 * reported through rs485FaultHandle.
 */
typedef enum
{
    APP_RS485_OK = 0,
    APP_RS485_E_NULL,
    APP_RS485_E_QUEUE_EMPTY,
    APP_RS485_E_QUEUE_FULL,
    APP_RS485_E_OS
} app_rs485_status_t;

/* ===================== Public Function Prototypes ======================== */

/**
 * @brief RS485 validation application task.
 *
 * @param argument  Task argument provided by the RTOS.
 *
 * @details
 * This task is the only application-level owner of the RS485 protocol driver.
 *
 * During initialization, the task:
 *
 * - Initializes the RS485 driver.
 * - Places the transceiver in receive mode.
 * - Starts an interrupt-driven reception of one four-byte frame.
 *
 * During normal operation, the task waits indefinitely for:
 *
 * - APP_RS485_EVT_TX_READY: Process pending TX queue data.
 * - APP_RS485_EVT_RX_READY: Validate and process a completed frame.
 * - APP_RS485_EVT_RX_ERROR: Process a deferred UART reception error.
 *
 * UART callbacks only update the lower-layer state and set internal event
 * flags. Frame validation, message queue access, printing and transmission are
 * performed from task context.
 */
void app_rs485_task(void * argument);

/**
 * @brief Enqueue one byte for RS485 transmission.
 *
 * @param data  Eight-bit data payload to transmit.
 *
 * @return
 * - APP_RS485_OK: Byte enqueued successfully.
 * - APP_RS485_E_QUEUE_FULL: TX queue is full.
 * - APP_RS485_E_OS: Unexpected RTOS error.
 *
 * @details
 * This function does not access the RS485 hardware directly. It enqueues one
 * byte into rs485OutputQueueHandle and sets APP_RS485_EVT_TX_READY.
 *
 * app_rs485_task() later serializes the byte as an
 * RS485_COMMAND_ECHO_REQUEST frame.
 */
app_rs485_status_t app_rs485_send(uint8_t data);

/**
 * @brief Read one valid byte received through RS485.
 *
 * @param[out] p_data  Destination for the received data byte.
 *
 * @return
 * - APP_RS485_OK: One byte was read successfully.
 * - APP_RS485_E_NULL: @p p_data is NULL.
 * - APP_RS485_E_QUEUE_EMPTY: No received byte is currently available.
 * - APP_RS485_E_OS: Unexpected RTOS error.
 *
 * @details
 * This function is non-blocking. It checks rs485InputQueueHandle once and
 * returns immediately.
 *
 * The queue may contain data received in either an ECHO_REQUEST or an
 * ECHO_RESPONSE frame.
 */
app_rs485_status_t app_rs485_receive(uint8_t * p_data);

#endif /* APP_RS485_H_ */

/** @} */
