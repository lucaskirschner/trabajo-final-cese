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
 * @file    rs485_port.h
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-08-04
 * @brief   BSP port layer for the RS485 interface.
 *
 * @details
 * This module provides a thin wrapper around STM32 HAL UART and GPIO services.
 * It controls the RS485 transceiver direction pins and exposes:
 *
 * - Blocking UART transmission.
 * - Interrupt-driven UART reception.
 * - Reception abort for error recovery.
 *
 * Reception is started with rs485_port_receive_it(). The operation returns
 * immediately and completes asynchronously through HAL_UART_RxCpltCallback().
 *
 * The module assumes a conventional half-duplex RS485 transceiver with:
 *
 * - DE active high.
 * - /RE active low.
 *
 * The default idle state is receive mode:
 *
 * - DE = 0: Driver disabled.
 * - /RE = 0: Receiver enabled.
 *
 * This layer does not use RTOS services and does not process protocol frames.
 *
 * @ingroup rs485_port
 * @{
 */

#ifndef RS485_PORT_H_
#define RS485_PORT_H_

/* ============================= Includes ================================== */

#include <stdint.h>

/* ============================== Types ==================================== */

/**
 * @brief Public status codes for the RS485 port layer.
 */
typedef enum
{
    RS485_PORT_OK = 0,
    RS485_PORT_E_NULL,
    RS485_PORT_E_PARAM,
    RS485_PORT_E_STATE,
    RS485_PORT_E_HW,
    RS485_PORT_E_TIMEOUT
} rs485_port_status_t;

/**
 * @brief RS485 transceiver operating mode.
 */
typedef enum
{
    RS485_PORT_MODE_RX = 0,
    RS485_PORT_MODE_TX
} rs485_port_mode_t;

/**
 * @brief Public RS485 port handle.
 */
typedef struct
{
    uint32_t reserved;
} rs485_port_handle_t;

/* ===================== Public Function Prototypes ======================== */

/**
 * @brief Initialize the RS485 port layer.
 *
 * @param[in,out] handle  RS485 port handle.
 *
 * @return RS485_PORT_OK on success, error code otherwise.
 *
 * @details
 * The function initializes the internal port state and leaves the transceiver
 * in receive mode. UART7 and its GPIO pins must already have been initialized
 * by the generated STM32 initialization code.
 */
rs485_port_status_t rs485_port_init(
    rs485_port_handle_t * handle);

/**
 * @brief Deinitialize the RS485 port layer.
 *
 * @param[in,out] handle  RS485 port handle.
 *
 * @return RS485_PORT_OK on success, error code otherwise.
 *
 * @details
 * Any pending interrupt-driven reception is aborted and the transceiver is
 * returned to receive mode.
 */
rs485_port_status_t rs485_port_deinit(
    rs485_port_handle_t * handle);

/**
 * @brief Set the RS485 transceiver operating mode.
 *
 * @param[in] mode  Desired transceiver mode.
 *
 * @return RS485_PORT_OK on success, error code otherwise.
 */
rs485_port_status_t rs485_port_set_mode(
    rs485_port_mode_t mode);

/**
 * @brief Transmit data through UART7 using a blocking operation.
 *
 * @param[in] data        Pointer to the source buffer.
 * @param[in] size        Number of bytes to transmit.
 * @param[in] timeout_ms  Transmission timeout in milliseconds.
 *
 * @return RS485_PORT_OK on success, error code otherwise.
 *
 * @details
 * Direction control is not performed by this function. The upper RS485 driver
 * must select transmit mode before calling it and restore receive mode after
 * the operation finishes.
 */
rs485_port_status_t rs485_port_transmit(
    const uint8_t * data,
    uint16_t size,
    uint32_t timeout_ms);

/**
 * @brief Start an interrupt-driven UART7 reception.
 *
 * @param[out] data  Destination buffer that remains valid until completion.
 * @param[in]  size  Number of bytes to receive.
 *
 * @return
 * - RS485_PORT_OK: Reception started successfully.
 * - RS485_PORT_E_NULL: @p data is NULL.
 * - RS485_PORT_E_PARAM: @p size is zero.
 * - RS485_PORT_E_STATE: Port is not initialized or UART is busy.
 * - RS485_PORT_E_HW: HAL rejected the operation.
 *
 * @details
 * This function calls HAL_UART_Receive_IT() and returns immediately. The
 * caller must keep @p data valid until HAL_UART_RxCpltCallback() or
 * HAL_UART_ErrorCallback() reports completion.
 *
 * Only one interrupt-driven reception may be active at a time.
 */
rs485_port_status_t rs485_port_receive_it(
    uint8_t * data,
    uint16_t size);

/**
 * @brief Abort the current UART7 reception operation.
 *
 * @return RS485_PORT_OK on success, error code otherwise.
 *
 * @details
 * This function is intended for recovery after UART errors, task shutdown or
 * explicit cancellation of a pending interrupt-driven reception.
 */
rs485_port_status_t rs485_port_abort_receive(void);

#endif /* RS485_PORT_H_ */

/** @} */
