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
 * @date    2026-08-08
 * @brief   Interrupt-driven single-byte RS485 driver interface.
 *
 * @details
 * This module implements a minimal half-duplex RS485 communication driver
 * based on the hardware-dependent services provided by rs485_port.
 *
 * Both transmission and reception are asynchronous and use interrupt-driven
 * UART operations through the underlying port layer.
 *
 * The driver handles one byte per operation and does not implement:
 *
 * - Message framing.
 * - Start-of-frame markers.
 * - Device addressing.
 * - Checksums or CRC.
 * - Protocol commands.
 * - Automatic retransmission.
 * - Multi-byte buffering.
 *
 * Transmission is started through rs485_send(). The function returns after the
 * UART transfer has been successfully requested. Actual transmission completion
 * is reported asynchronously through rs485_tx_complete_callback().
 *
 * Reception is armed through rs485_receive_start(). Once one byte has been
 * received, rs485_rx_complete_callback() is invoked and the received value may
 * be retrieved through rs485_receive().
 *
 * The driver controls the half-duplex RS485 operating mode:
 *
 * - Before transmission, the transceiver is switched to transmit mode.
 * - After transmission completes, the transceiver returns to receive mode.
 * - Reception operations are performed with the transceiver in receive mode.
 *
 * The application-level callbacks are weak by default and may be overridden
 * by an upper application layer.
 *
 * This module does not use DMA, dynamic memory or RTOS services.
 *
 * @defgroup rs485 RS485 Driver
 * @{
 */

#ifndef RS485_H_
#define RS485_H_

/* ============================= Includes ================================== */

#include <stdint.h>

/* ============================ Public Types ================================ */

/**
 * @brief RS485 driver operation status.
 */
typedef enum
{
    RS485_OK = 0,

    RS485_E_NULL,
    RS485_E_PARAM,
    RS485_E_STATE,
    RS485_E_TIMEOUT,
    RS485_E_HW

} rs485_status_t;

/**
 * @brief Public RS485 driver handle.
 *
 * @details
 * The current implementation supports a single RS485 interface.
 *
 * The reserved field keeps the public interface handle-based and allows future
 * extensions without modifying the existing API.
 */
typedef struct
{
    uint32_t reserved;

} rs485_handle_t;

/* ===================== Public Function Prototypes ========================= */

/**
 * @brief Initialize the RS485 driver.
 *
 * @param[in,out] handle  Pointer to the RS485 driver handle.
 *
 * @return RS485_OK on success, error code otherwise.
 *
 * @details
 * Initializes the underlying RS485 port layer, clears the internal driver
 * state and places the external transceiver in receive mode.
 */
rs485_status_t rs485_init(
    rs485_handle_t * handle);

/**
 * @brief Deinitialize the RS485 driver.
 *
 * @param[in,out] handle  Pointer to the RS485 driver handle.
 *
 * @return RS485_OK on success, error code otherwise.
 *
 * @details
 * Aborts any active communication operation, restores receive mode and
 * deinitializes the underlying RS485 port layer.
 */
rs485_status_t rs485_deinit(
    rs485_handle_t * handle);

/**
 * @brief Start transmission of one RS485 byte.
 *
 * @param[in] data  Byte to transmit.
 *
 * @return RS485_OK if transmission was started successfully.
 * @return RS485_E_STATE if the driver is not initialized or another
 *         communication operation is active.
 * @return Other RS485 error code if the underlying port operation fails.
 *
 * @details
 * The byte is copied into persistent driver storage before starting the
 * interrupt-driven UART transmission.
 *
 * This function is non-blocking. RS485_OK indicates that transmission was
 * successfully started, not that the byte has already been transmitted.
 *
 * Actual completion is reported through rs485_tx_complete_callback().
 */
rs485_status_t rs485_send(
    uint8_t data);

/**
 * @brief Start reception of one RS485 byte.
 *
 * @return RS485_OK if reception was started successfully.
 * @return RS485_E_STATE if the driver is not initialized, another operation is
 *         active or a previously received byte has not yet been consumed.
 * @return Other RS485 error code if the underlying port operation fails.
 *
 * @details
 * Configures the transceiver for receive mode and starts an interrupt-driven
 * one-byte UART reception.
 *
 * Actual completion is reported asynchronously through
 * rs485_rx_complete_callback().
 */
rs485_status_t rs485_receive_start(void);

/**
 * @brief Retrieve the last completed RS485 byte reception.
 *
 * @param[out] p_data  Pointer where the received byte will be stored.
 *
 * @return RS485_OK on success.
 * @return RS485_E_NULL if p_data is NULL.
 * @return RS485_E_STATE if the driver is not initialized or no received byte
 *         is currently available.
 *
 * @details
 * This function does not access the UART peripheral. It only returns the byte
 * previously stored by the interrupt-driven reception.
 *
 * Once successfully retrieved, the byte is marked as consumed and a new
 * reception may be started through rs485_receive_start().
 */
rs485_status_t rs485_receive(
    uint8_t * p_data);

/**
 * @brief Abort an active RS485 reception.
 *
 * @return RS485_OK on success, error code otherwise.
 *
 * @details
 * If a reception is currently active, the underlying interrupt-driven UART
 * receive operation is aborted.
 *
 * Any pending received byte is discarded and the transceiver is restored to
 * receive mode.
 */
rs485_status_t rs485_receive_abort(void);

/**
 * @brief Abort an active RS485 transmission.
 *
 * @return RS485_OK on success, error code otherwise.
 *
 * @details
 * If a transmission is currently active, the underlying interrupt-driven UART
 * transmit operation is aborted.
 *
 * The transceiver is restored to receive mode after the abort operation.
 */
rs485_status_t rs485_transmit_abort(void);

/* ===================== Notification Hook Prototypes ====================== */

/**
 * @brief Notify completion of a one-byte RS485 transmission.
 *
 * @param[in] data  Byte whose transmission has completed.
 *
 * @details
 * This callback is invoked from interrupt context after the underlying UART
 * transmission has completed and the RS485 transceiver has been restored to
 * receive mode.
 *
 * The default implementation is weak and performs no operation. An upper
 * application layer may provide a strong definition.
 *
 * The overriding implementation must remain short and ISR-safe.
 */
void rs485_tx_complete_callback(
    uint8_t data);

/**
 * @brief Notify completion of a one-byte RS485 reception.
 *
 * @details
 * This callback is invoked from interrupt context after one byte has been
 * received successfully.
 *
 * The received byte is stored internally and may later be obtained from task
 * or application context through rs485_receive().
 *
 * The default implementation is weak and performs no operation. An upper
 * application layer may provide a strong definition.
 *
 * The overriding implementation must remain short and ISR-safe.
 */
void rs485_rx_complete_callback(void);

/**
 * @brief Notify an asynchronous RS485 UART error.
 *
 * @param[in] error_code  UART error flags reported by the port layer.
 *
 * @details
 * This callback is invoked from interrupt context when the underlying UART
 * reports a communication error.
 *
 * The driver clears the active communication state and restores receive mode
 * before invoking this callback.
 *
 * The default implementation is weak and performs no operation. An upper
 * application layer may provide a strong definition.
 *
 * The overriding implementation must remain short and ISR-safe.
 */
void rs485_error_callback(
    uint32_t error_code);

/** @} */

#endif /* RS485_H_ */
