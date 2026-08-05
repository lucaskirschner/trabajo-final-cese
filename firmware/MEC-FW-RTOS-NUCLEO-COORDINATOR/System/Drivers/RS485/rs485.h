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
 * @file    rs485.h
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-08-04
 * @brief   Interrupt-driven single-byte RS485 driver.
 *
 * @details
 * This module provides a minimal RS485 driver intended for functional
 * validation of the UART and RS485 transceiver using a conventional serial
 * terminal such as PuTTY.
 *
 * The driver exchanges individual eight-bit data values without adding:
 *
 * - Start-of-frame markers.
 * - Protocol commands.
 * - Checksums.
 * - Multi-byte framing.
 *
 * Transmission is blocking and sends exactly one byte.
 *
 * Reception is interrupt-driven and follows three stages:
 *
 * 1. rs485_receive_start() arms reception of one byte.
 * 2. The UART interrupt completes the reception and invokes the internal port
 *    notification hook.
 * 3. rs485_receive() obtains the completed byte from task context.
 *
 * The application layer must provide:
 *
 * - rs485_rx_complete_callback().
 * - rs485_error_callback().
 *
 * These callbacks execute in interrupt context and must remain short and
 * ISR-safe. A typical RTOS application uses them only to set event flags and
 * defer all processing to a task.
 *
 * The driver does not depend on an RTOS, does not use DMA and is not
 * thread-safe. A single upper-level task should own all driver operations.
 *
 * @ingroup rs485
 * @{
 */

#ifndef RS485_H_
#define RS485_H_

/* ============================= Includes ================================== */

#include <stdint.h>

/* ============================== Types ==================================== */

/**
 * @brief Public status codes for the RS485 driver.
 */
typedef enum
{
    RS485_OK = 0,
    RS485_E_NULL,
    RS485_E_PARAM,
    RS485_E_STATE,
    RS485_E_HW,
    RS485_E_TIMEOUT
} rs485_status_t;

/**
 * @brief Public RS485 driver handle.
 */
typedef struct
{
    uint32_t reserved;
} rs485_handle_t;

/* ===================== Public Function Prototypes ======================== */

/**
 * @brief Initialize the RS485 driver.
 *
 * @param[in,out] handle  RS485 driver handle.
 *
 * @return
 * - RS485_OK: Driver initialized successfully.
 * - RS485_E_NULL: @p handle is NULL.
 * - RS485_E_STATE: Driver is already initialized.
 * - Other status: Lower-layer initialization or direction-control error.
 *
 * @details
 * This function initializes the lower RS485 port layer, clears the internal
 * reception state and leaves the transceiver in receive mode.
 *
 * The function does not automatically start reception. The application must
 * subsequently call rs485_receive_start().
 */
rs485_status_t rs485_init(
    rs485_handle_t * handle);

/**
 * @brief Deinitialize the RS485 driver.
 *
 * @param[in,out] handle  RS485 driver handle.
 *
 * @return
 * - RS485_OK: Driver deinitialized successfully.
 * - RS485_E_NULL: @p handle is NULL.
 * - RS485_E_STATE: Driver is not initialized.
 * - Other status: Lower-layer error.
 *
 * @details
 * Any active interrupt-driven reception is aborted before the lower RS485 port
 * layer is deinitialized.
 */
rs485_status_t rs485_deinit(
    rs485_handle_t * handle);

/**
 * @brief Send one byte through the RS485 interface.
 *
 * @param[in] data        Eight-bit value to transmit.
 * @param[in] timeout_ms  Blocking transmission timeout in milliseconds.
 *
 * @return
 * - RS485_OK: Byte transmitted successfully.
 * - RS485_E_STATE: Driver is not initialized or reception is currently active.
 * - RS485_E_TIMEOUT: Blocking transmission timed out.
 * - RS485_E_HW: Lower-layer hardware error.
 * - Other status: Lower-layer parameter or state error.
 *
 * @details
 * This function:
 *
 * 1. Switches the transceiver to transmit mode.
 * 2. Sends exactly one byte.
 * 3. Restores receive mode before returning.
 *
 * An active interrupt-driven reception must be completed or aborted before
 * this function is called.
 */
rs485_status_t rs485_send(
    uint8_t data,
    uint32_t timeout_ms);

/**
 * @brief Start interrupt-driven reception of one byte.
 *
 * @return
 * - RS485_OK: Reception started successfully.
 * - RS485_E_STATE: Driver is not initialized, reception is already active or a
 *   previously received byte has not yet been consumed.
 * - Other status: Lower-layer error.
 *
 * @details
 * This function places the transceiver in receive mode and starts an
 * interrupt-driven UART reception of exactly one byte.
 *
 * The function returns immediately. Completion is reported asynchronously
 * through rs485_rx_complete_callback().
 */
rs485_status_t rs485_receive_start(void);

/**
 * @brief Obtain the byte completed by the interrupt-driven reception.
 *
 * @param[out] p_data  Destination for the received byte.
 *
 * @return
 * - RS485_OK: Byte obtained successfully.
 * - RS485_E_NULL: @p p_data is NULL.
 * - RS485_E_STATE: Driver is not initialized or no completed byte is available.
 * - RS485_E_HW: The most recent reception ended with a UART error.
 *
 * @details
 * This function does not wait for UART data. It must be called after the
 * receive-complete notification generated by rs485_rx_complete_callback().
 *
 * After the function returns successfully, the completed reception is consumed
 * and a new reception may be started with rs485_receive_start().
 */
rs485_status_t rs485_receive(
    uint8_t * p_data);

/**
 * @brief Abort the current interrupt-driven reception.
 *
 * @return
 * - RS485_OK: Reception aborted or no reception was active.
 * - RS485_E_STATE: Driver is not initialized.
 * - Other status: Lower-layer abort error.
 *
 * @details
 * This function clears the active, complete and error reception states.
 *
 * It is intended to be called before an application-requested transmission when
 * the driver is normally waiting for an incoming byte.
 */
rs485_status_t rs485_receive_abort(void);

/**
 * @brief Echo one previously received byte.
 *
 * @param[out] p_data     Optional destination for the echoed byte. May be NULL.
 * @param[in]  timeout_ms Blocking transmission timeout in milliseconds.
 *
 * @return RS485_OK on success, error code otherwise.
 *
 * @details
 * This helper performs the responder-side echo operation after a
 * receive-complete notification:
 *
 * 1. Obtain the completed byte through rs485_receive().
 * 2. Transmit the same byte through rs485_send().
 * 3. Optionally copy the echoed byte to @p p_data.
 *
 * The function does not start the next reception. The application must call
 * rs485_receive_start() after the echo transmission completes.
 */
rs485_status_t rs485_process_echo(
    uint8_t * p_data,
    uint32_t timeout_ms);

/**
 * @brief Notify the upper application that byte reception completed.
 *
 * @details
 * This function is invoked from interrupt context after the requested byte has
 * been received.
 *
 * The application layer must provide the implementation, typically to set an
 * RTOS event flag. The implementation must remain short and ISR-safe.
 */
void rs485_rx_complete_callback(void);

/**
 * @brief Notify the upper application that a UART reception error occurred.
 *
 * @param[in] error_code  STM32 HAL UART error flags.
 *
 * @details
 * This function is invoked from interrupt context after a UART reception
 * error.
 *
 * The application layer must provide the implementation, typically to store
 * the error flags and set an RTOS event flag. The implementation must remain
 * short and ISR-safe.
 */
void rs485_error_callback(
    uint32_t error_code);

#endif /* RS485_H_ */

/** @} */
