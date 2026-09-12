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
 * @file    io_port.h
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-04-20
 * @brief   Shared SPI1 port layer for industrial input/output modules.
 *
 * @details
 * This module isolates MCU/HAL dependencies for the devices connected to SPI1.
 * It provides:
 * - shared SPI1 access,
 * - chip select arbitration,
 * - blocking transmit/receive services.
 *
 * The current implementation supports:
 * - SCLT3 digital input module,
 * - VNI8200XP-32 digital output module.
 *
 * Access is serialized internally. Only one device can keep its chip select
 * asserted at a time.
 *
 * @ingroup io_port
 * @{
 */

#ifndef IO_PORT_H_
#define IO_PORT_H_

/* ============================= Includes ================================== */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/* ============================== Types ==================================== */

/**
 * @brief Devices connected to the shared industrial I/O SPI bus.
 */
typedef enum
{
    IO_PORT_DEVICE_NONE = 0,
    IO_PORT_DEVICE_SCLT3,
    IO_PORT_DEVICE_VNI8200XP32
} io_port_device_t;

/**
 * @brief Return codes for I/O port operations.
 */
typedef enum
{
    IO_PORT_OK = 0,
    IO_PORT_E_NULL,
    IO_PORT_E_PARAM,
    IO_PORT_E_HW,
    IO_PORT_E_TIMEOUT,
    IO_PORT_E_BUSY
} io_port_status_t;

/* ===================== Public Function Prototypes ======================== */

/**
 * @brief Transmit and receive data over SPI1 for the selected device.
 *
 * @param device Target device.
 * @param[in] tx_buffer Pointer to the transmit buffer.
 * @param[out] rx_buffer Pointer to the receive buffer.
 * @param length Number of bytes to transfer.
 *
 * @return
 * - IO_PORT_OK: Transfer completed successfully.
 * - IO_PORT_E_NULL: Null pointer passed in @p tx_buffer or @p rx_buffer.
 * - IO_PORT_E_PARAM: Invalid device identifier or zero length.
 * - IO_PORT_E_BUSY: Shared SPI1 bus is currently in use.
 * - IO_PORT_E_TIMEOUT: SPI transfer timed out.
 * - IO_PORT_E_HW: SPI transaction failed.
 *
 * @details
 * This function asserts the chip select corresponding to @p device before the
 * SPI transaction and deasserts it after the transfer completes.
 */
io_port_status_t io_port_transmit_receive(io_port_device_t device,
                                          const uint8_t * const tx_buffer,
                                          uint8_t * const rx_buffer,
                                          size_t length);

/**
 * @brief Transmit data over SPI1 for the selected device.
 *
 * @param device Target device.
 * @param[in] tx_buffer Pointer to the transmit buffer.
 * @param length Number of bytes to transmit.
 *
 * @return
 * - IO_PORT_OK: Transfer completed successfully.
 * - IO_PORT_E_NULL: Null pointer passed in @p tx_buffer.
 * - IO_PORT_E_PARAM: Invalid device identifier or zero length.
 * - IO_PORT_E_BUSY: Shared SPI1 bus is currently in use.
 * - IO_PORT_E_TIMEOUT: SPI transfer timed out.
 * - IO_PORT_E_HW: SPI transaction failed.
 *
 * @details
 * This function asserts the chip select corresponding to @p device before the
 * SPI transaction and deasserts it after the transfer completes.
 */
io_port_status_t io_port_transmit(io_port_device_t device,
                                  const uint8_t * const tx_buffer,
                                  size_t length);

/**
 * @brief Receive data over SPI1 for the selected device.
 *
 * @param device Target device.
 * @param[out] rx_buffer Pointer to the receive buffer.
 * @param length Number of bytes to receive.
 *
 * @return
 * - IO_PORT_OK: Transfer completed successfully.
 * - IO_PORT_E_NULL: Null pointer passed in @p rx_buffer.
 * - IO_PORT_E_PARAM: Invalid device identifier or zero length.
 * - IO_PORT_E_BUSY: Shared SPI1 bus is currently in use.
 * - IO_PORT_E_TIMEOUT: SPI transfer timed out.
 * - IO_PORT_E_HW: SPI transaction failed.
 *
 * @details
 * This function asserts the chip select corresponding to @p device before the
 * SPI transaction and deasserts it after the transfer completes.
 *
 * @note
 * This implementation uses HAL_SPI_Receive(). The SPI peripheral must already
 * be configured in a way compatible with this receive operation.
 */
io_port_status_t io_port_receive(io_port_device_t device,
                                 uint8_t * const rx_buffer,
                                 size_t length);

#endif /* IO_PORT_H_ */

/** @} */
