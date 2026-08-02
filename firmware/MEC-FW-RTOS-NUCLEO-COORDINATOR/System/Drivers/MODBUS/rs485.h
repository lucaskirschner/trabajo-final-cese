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
 * @date    2026-06-24
 * @brief   RS485 transceiver driver.
 *
 * @details
 * This module provides a hardware-independent RS485 driver layer over the
 * RS485 port layer. It controls the transceiver direction and exposes blocking
 * byte-oriented send and receive services.
 *
 * This driver does not implement any application protocol. It is intended to
 * be used as the transport base for protocols such as Modbus RTU master or
 * slave.
 *
 * @ingroup rs485
 * @{
 */

#ifndef RS485_H_
#define RS485_H_

/* ============================= Includes ================================== */

#include <stdint.h>
#include <stdbool.h>

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
 * @brief RS485 driver operating mode.
 */
typedef enum
{
    RS485_MODE_RX = 0,
    RS485_MODE_TX
} rs485_mode_t;

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
 * @return RS485_OK on success, error code otherwise.
 */
rs485_status_t rs485_init(rs485_handle_t *handle);

/**
 * @brief Deinitialize the RS485 driver.
 *
 * @param[in,out] handle  RS485 driver handle.
 *
 * @return RS485_OK on success, error code otherwise.
 */
rs485_status_t rs485_deinit(rs485_handle_t *handle);

/**
 * @brief Set the RS485 transceiver mode.
 *
 * @param[in] mode  Desired transceiver mode.
 *
 * @return RS485_OK on success, error code otherwise.
 */
rs485_status_t rs485_set_mode(rs485_mode_t mode);

/**
 * @brief Send a data buffer through the RS485 interface.
 *
 * @details
 * This function automatically switches the transceiver to transmit mode,
 * sends the requested bytes and then returns the transceiver to receive mode.
 *
 * @param[in] data        Pointer to the data buffer.
 * @param[in] size        Number of bytes to transmit.
 * @param[in] timeout_ms  Transmission timeout in milliseconds.
 *
 * @return RS485_OK on success, error code otherwise.
 */
rs485_status_t rs485_send(
    const uint8_t *data,
    uint16_t size,
    uint32_t timeout_ms);

/**
 * @brief Receive a fixed number of bytes through the RS485 interface.
 *
 * @details
 * This function places the transceiver in receive mode and waits until the
 * requested number of bytes is received or until the timeout expires.
 *
 * @param[out] data       Pointer to the destination buffer.
 * @param[in] size        Number of bytes to receive.
 * @param[in] timeout_ms  Reception timeout in milliseconds.
 *
 * @return RS485_OK on success, error code otherwise.
 */
rs485_status_t rs485_receive(
    uint8_t *data,
    uint16_t size,
    uint32_t timeout_ms);

#endif /* RS485_H_ */

/** @} */
