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
 * @file    sn65hvd82_port.h
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-04-22
 * @brief   Port layer for SN65HVD82 (STM32H5 HAL).
 *
 * @details
 * This module isolates MCU/HAL dependencies (UART, GPIO, delays) required by
 * the SN65HVD82 RS-485 transceiver.
 *
 * The SN65HVD82 itself does not expose registers or a digital configuration
 * interface. Therefore, this port layer provides:
 * - UART data transfer primitives through UART4
 * - Direction-control primitives through DE and RE pins
 * - Basic blocking delay service
 *
 * @ingroup sn65hvd82
 * @{
 */

#ifndef SN65HVD82_PORT_H_
#define SN65HVD82_PORT_H_

/* ============================= Includes ================================== */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ============================== Macros =================================== */

/* ============================== Types ==================================== */

typedef enum
{
    SN65HVD82_PORT_OK = 0,
    SN65HVD82_PORT_E_NULL,
    SN65HVD82_PORT_E_PARAM,
    SN65HVD82_PORT_E_HW,
    SN65HVD82_PORT_E_TIMEOUT
} sn65hvd82_port_status_t;

/* ===================== Public Function Prototypes ======================== */

/**
 * @brief Configure the transceiver for RS-485 transmit mode.
 *
 * @return
 * - SN65HVD82_PORT_OK: Operation completed successfully.
 *
 * @details
 * This function enables the driver and disables the receiver:
 * - DE = 1
 * - RE = 1
 *
 * This is a common half-duplex configuration used to avoid local echo during
 * transmission.
 */
sn65hvd82_port_status_t sn65hvd82_port_set_tx_mode(void);

/**
 * @brief Configure the transceiver for RS-485 transmit mode with loopback enabled.
 *
 * @return
 * - SN65HVD82_PORT_OK: Operation completed successfully.
 *
 * @details
 * This function enables the driver and keeps the receiver enabled:
 * - DE = 1
 * - RE = 0
 *
 * In this configuration, the node can receive its own transmitted data through
 * the RS-485 bus path, allowing local echo verification.
 */
sn65hvd82_port_status_t sn65hvd82_port_set_tx_loopback_mode(void);

/**
 * @brief Configure the transceiver for RS-485 receive mode.
 *
 * @return
 * - SN65HVD82_PORT_OK: Operation completed successfully.
 *
 * @details
 * This function disables the driver and enables the receiver:
 * - DE = 0
 * - RE = 0
 */
sn65hvd82_port_status_t sn65hvd82_port_set_rx_mode(void);

/**
 * @brief Configure the transceiver for standby mode.
 *
 * @return
 * - SN65HVD82_PORT_OK: Operation completed successfully.
 *
 * @details
 * This function disables both driver and receiver:
 * - DE = 0
 * - RE = 1
 *
 * In this state, the transceiver enters its low-power standby condition.
 */
sn65hvd82_port_status_t sn65hvd82_port_set_standby_mode(void);

/**
 * @brief Transmit a byte buffer through UART4.
 *
 * @param data Pointer to the buffer containing the bytes to transmit.
 * @param length Number of bytes to transmit.
 *
 * @return
 * - SN65HVD82_PORT_OK: Operation completed successfully.
 * - SN65HVD82_PORT_E_NULL: Null pointer passed in @p data.
 * - SN65HVD82_PORT_E_PARAM: Invalid length.
 * - SN65HVD82_PORT_E_HW: UART transaction failed.
 * - SN65HVD82_PORT_E_TIMEOUT: UART transaction timed out.
 *
 * @details
 * This function performs a blocking UART transmission using the underlying HAL.
 */
sn65hvd82_port_status_t sn65hvd82_port_uart_transmit(const uint8_t * data,
                                                     size_t length);

/**
 * @brief Receive a byte buffer through UART4.
 *
 * @param[out] data Pointer to the destination buffer where received bytes
 *                  will be stored.
 * @param length Number of bytes to receive.
 *
 * @return
 * - SN65HVD82_PORT_OK: Operation completed successfully.
 * - SN65HVD82_PORT_E_NULL: Null pointer passed in @p data.
 * - SN65HVD82_PORT_E_PARAM: Invalid length.
 * - SN65HVD82_PORT_E_HW: UART transaction failed.
 * - SN65HVD82_PORT_E_TIMEOUT: UART transaction timed out.
 *
 * @details
 * This function performs a blocking UART reception using the underlying HAL.
 */
sn65hvd82_port_status_t sn65hvd82_port_uart_receive(uint8_t * const data,
                                                    size_t length);

/**
 * @brief Wait until the UART transmission is fully completed on the line.
 *
 * @return
 * - SN65HVD82_PORT_OK: Transmission complete flag detected.
 * - SN65HVD82_PORT_E_TIMEOUT: Timeout while waiting for transmission complete.
 *
 * @details
 * This function polls the UART transmission complete flag (TC) to ensure that
 * the last frame bit has physically left the UART before the RS-485 driver is
 * disabled.
 *
 * @note
 * This function is intended to be called after @ref sn65hvd82_port_uart_transmit
 * and before switching the transceiver back to receive mode.
 */
sn65hvd82_port_status_t sn65hvd82_port_uart_wait_tx_complete(void);

/**
 * @brief Flush the UART receive data path.
 *
 * @return
 * - SN65HVD82_PORT_OK: Flush request issued successfully.
 *
 * @details
 * This function clears pending UART receive data using the HAL request macro.
 * It can be used before starting a new reception sequence to discard stale
 * bytes left in the peripheral.
 */
sn65hvd82_port_status_t sn65hvd82_port_uart_flush_rx(void);

/**
 * @brief Transmit a frame using RS-485 half-duplex direction control.
 *
 * @param data Pointer to the buffer containing the bytes to transmit.
 * @param length Number of bytes to transmit.
 *
 * @return
 * - SN65HVD82_PORT_OK: Frame transmitted successfully and transceiver returned
 *   to receive mode.
 * - SN65HVD82_PORT_E_NULL: Null pointer passed in @p data.
 * - SN65HVD82_PORT_E_PARAM: Invalid length.
 * - SN65HVD82_PORT_E_HW: UART transaction failed.
 * - SN65HVD82_PORT_E_TIMEOUT: UART transaction or completion wait timed out.
 *
 * @details
 * This helper performs the complete blocking RS-485 transmit sequence:
 * - set transceiver to transmit mode,
 * - transmit the provided frame through UART4,
 * - wait until the UART transmission is physically complete,
 * - return the transceiver to receive mode.
 *
 * If transmission or completion wait fails, the function still attempts to
 * restore receive mode before returning the error status.
 */
sn65hvd82_port_status_t sn65hvd82_port_transmit_frame(const uint8_t * data,
                                                      size_t length);

/**
 * @brief Transmit a frame and read back the loopback data.
 *
 * @param tx_data Pointer to the buffer containing the bytes to transmit.
 * @param[out] rx_data Pointer to the destination buffer where loopback data
 *                     will be stored.
 * @param length Number of bytes to transmit and receive.
 *
 * @return
 * - SN65HVD82_PORT_OK: Frame transmitted successfully, loopback data received,
 *   and transceiver returned to receive mode.
 * - SN65HVD82_PORT_E_NULL: Null pointer passed in @p tx_data or @p rx_data.
 * - SN65HVD82_PORT_E_PARAM: Invalid length.
 * - SN65HVD82_PORT_E_HW: UART transaction failed.
 * - SN65HVD82_PORT_E_TIMEOUT: UART transaction or completion wait timed out.
 *
 * @details
 * This helper performs a blocking RS-485 transmit sequence with loopback
 * reception enabled:
 * - flush UART receive path,
 * - set transceiver to transmit mode with receiver enabled,
 * - transmit the provided frame through UART4,
 * - wait until the UART transmission is physically complete,
 * - receive the echoed bytes through UART4,
 * - return the transceiver to receive mode.
 *
 * This function is intended for local validation and debugging of transmitted
 * data through the transceiver path.
 */
sn65hvd82_port_status_t sn65hvd82_port_transmit_frame_loopback(const uint8_t * tx_data,
                                                               uint8_t * const rx_data,
                                                               size_t length);

/**
 * @brief Delay execution for the specified number of milliseconds.
 *
 * @param delay_ms Delay time in milliseconds.
 *
 * @details
 * This function provides a blocking delay using the underlying HAL time base.
 * It is intended for non-time-critical waits such as transceiver mode settling
 * or basic initialization sequences.
 *
 * @note
 * This implementation relies on HAL_Delay() and therefore its resolution is
 * limited to the HAL tick period. It is not suitable for precise short delays
 * in the microsecond range.
 */
void sn65hvd82_port_delay_ms(uint32_t delay_ms);

#endif /* SN65HVD82_PORT_H_ */

/** @} */
