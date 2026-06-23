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
 * @date    2026-06-22
 * @brief   BSP port layer for the RS485 interface.
 *
 * @details
 * This module provides a thin wrapper around STM32 HAL UART and GPIO services.
 * It controls the RS485 transceiver direction pins and exposes blocking
 * transmit/receive primitives to the upper RS485 driver layer.
 *
 * The module assumes a conventional RS485 transceiver with:
 * - DE active high.
 * - /RE active low.
 *
 * @ingroup rs485_port
 * @{
 */

#ifndef RS485_PORT_H_
#define RS485_PORT_H_

/* ============================= Includes ================================== */

#include <stdint.h>
#include <stdbool.h>

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
 */
rs485_port_status_t rs485_port_init(rs485_port_handle_t *handle);

/**
 * @brief Deinitialize the RS485 port layer.
 *
 * @param[in,out] handle  RS485 port handle.
 *
 * @return RS485_PORT_OK on success, error code otherwise.
 */
rs485_port_status_t rs485_port_deinit(rs485_port_handle_t *handle);

/**
 * @brief Set the RS485 transceiver mode.
 *
 * @param[in] mode  Desired transceiver mode.
 *
 * @return RS485_PORT_OK on success, error code otherwise.
 */
rs485_port_status_t rs485_port_set_mode(rs485_port_mode_t mode);

/**
 * @brief Transmit data through the RS485 UART.
 *
 * @param[in] data        Pointer to the data buffer.
 * @param[in] size        Number of bytes to transmit.
 * @param[in] timeout_ms  Timeout in milliseconds.
 *
 * @return RS485_PORT_OK on success, error code otherwise.
 */
rs485_port_status_t rs485_port_transmit(
    const uint8_t *data,
    uint16_t size,
    uint32_t timeout_ms);

/**
 * @brief Receive data through the RS485 UART.
 *
 * @param[out] data       Pointer to the destination buffer.
 * @param[in] size        Number of bytes to receive.
 * @param[in] timeout_ms  Timeout in milliseconds.
 *
 * @return RS485_PORT_OK on success, error code otherwise.
 */
rs485_port_status_t rs485_port_receive(
    uint8_t *data,
    uint16_t size,
    uint32_t timeout_ms);

#endif /* RS485_PORT_H_ */

/** @} */
