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
 * @date    2026-08-08
 * @brief   BSP port layer interface for the RS485 communication driver.
 *
 * @details
 * This module provides the hardware-dependent interface required by the RS485
 * driver.
 *
 * It wraps the STM32 HAL services associated with UART7 and the GPIO signals
 * used to control the external half-duplex RS485 transceiver:
 *
 * - DE: Driver Enable.
 * - /RE: Receiver Enable, active low.
 *
 * Both transmission and reception are implemented using interrupt-driven UART
 * operations:
 *
 * - HAL_UART_Transmit_IT().
 * - HAL_UART_Receive_IT().
 *
 * Completion and error events are propagated to the upper RS485 driver through
 * notification callbacks:
 *
 * - rs485_port_tx_complete_callback().
 * - rs485_port_rx_complete_callback().
 * - rs485_port_error_callback().
 *
 * The default implementations of these callbacks are weak and perform no
 * operation. The upper RS485 driver may provide strong definitions.
 *
 * This module does not implement RS485 protocol logic, message framing,
 * application processing, RTOS synchronization or dynamic memory allocation.
 *
 * @defgroup rs485_port RS485 Port
 * @{
 */

#ifndef RS485_PORT_H_
#define RS485_PORT_H_

/* ============================= Includes ================================== */

#include <stdint.h>

/* ============================ Public Types ================================ */

/**
 * @brief RS485 port operation status.
 */
typedef enum
{
    RS485_PORT_OK = 0,

    RS485_PORT_E_NULL,
    RS485_PORT_E_PARAM,
    RS485_PORT_E_STATE,
    RS485_PORT_E_TIMEOUT,
    RS485_PORT_E_HW

} rs485_port_status_t;

/**
 * @brief RS485 transceiver operating mode.
 */
typedef enum
{
    /**
     * @brief Receive mode.
     *
     * DE  = 0: line driver disabled.
     * /RE = 0: receiver enabled.
     */
    RS485_PORT_MODE_RX = 0,

    /**
     * @brief Transmit mode.
     *
     * DE  = 1: line driver enabled.
     * /RE = 1: receiver disabled.
     */
    RS485_PORT_MODE_TX

} rs485_port_mode_t;

/**
 * @brief Public RS485 port handle.
 *
 * @details
 * The current implementation supports a single RS485 interface connected to
 * UART7. The reserved field keeps the public API handle-based and allows future
 * extensions without changing the function interface.
 */
typedef struct
{
    uint32_t reserved;

} rs485_port_handle_t;

/* ===================== Public Function Prototypes ========================= */

/**
 * @brief Initialize the RS485 hardware port layer.
 *
 * @param[in,out] handle  Pointer to the RS485 port handle.
 *
 * @return RS485_PORT_OK on success, error code otherwise.
 *
 * @details
 * Initializes the internal port state and places the external RS485
 * transceiver in receive mode.
 *
 * UART7 and the associated GPIO peripherals must already have been initialized
 * by the board initialization code before calling this function.
 */
rs485_port_status_t rs485_port_init(
    rs485_port_handle_t * handle);

/**
 * @brief Deinitialize the RS485 hardware port layer.
 *
 * @param[in,out] handle  Pointer to the RS485 port handle.
 *
 * @return RS485_PORT_OK on success, error code otherwise.
 *
 * @details
 * Aborts any active interrupt-driven UART operation, restores the transceiver
 * to receive mode and clears the internal port state.
 */
rs485_port_status_t rs485_port_deinit(
    rs485_port_handle_t * handle);

/**
 * @brief Configure the operating mode of the RS485 transceiver.
 *
 * @param[in] mode  Requested RS485 transceiver mode.
 *
 * @return RS485_PORT_OK on success.
 * @return RS485_PORT_E_PARAM if the requested mode is invalid.
 *
 * @details
 * This function controls the DE and /RE GPIO signals associated with the
 * external half-duplex RS485 transceiver.
 *
 * The function only modifies the transceiver control signals. It does not
 * start, stop or abort UART transfers.
 */
rs485_port_status_t rs485_port_set_mode(
    rs485_port_mode_t mode);

/**
 * @brief Start an interrupt-driven UART transmission.
 *
 * @param[in] data  Pointer to the transmit buffer.
 * @param[in] size  Number of bytes to transmit.
 *
 * @return RS485_PORT_OK if the transmission was started successfully.
 * @return RS485_PORT_E_NULL if data is NULL.
 * @return RS485_PORT_E_PARAM if size is zero.
 * @return RS485_PORT_E_STATE if the port is not initialized or another
 *         UART transfer is active.
 * @return RS485_PORT_E_HW if the HAL operation fails.
 *
 * @details
 * Starts a non-blocking transmission using HAL_UART_Transmit_IT().
 *
 * The data buffer must remain valid and unmodified until
 * rs485_port_tx_complete_callback() is invoked.
 *
 * The function returns immediately after the UART interrupt-driven transfer
 * has been successfully started.
 */
rs485_port_status_t rs485_port_transmit_it(
    uint8_t * data,
    uint16_t size);

/**
 * @brief Start an interrupt-driven UART reception.
 *
 * @param[out] data  Pointer to the receive buffer.
 * @param[in]  size  Number of bytes to receive.
 *
 * @return RS485_PORT_OK if reception was started successfully.
 * @return RS485_PORT_E_NULL if data is NULL.
 * @return RS485_PORT_E_PARAM if size is zero.
 * @return RS485_PORT_E_STATE if the port is not initialized or another
 *         UART transfer is active.
 * @return RS485_PORT_E_HW if the HAL operation fails.
 *
 * @details
 * Starts a non-blocking reception using HAL_UART_Receive_IT().
 *
 * The receive buffer must remain valid until
 * rs485_port_rx_complete_callback() is invoked or the operation is aborted.
 */
rs485_port_status_t rs485_port_receive_it(
    uint8_t * data,
    uint16_t size);

/**
 * @brief Abort an active interrupt-driven UART transmission.
 *
 * @return RS485_PORT_OK on success, error code otherwise.
 *
 * @details
 * Calls HAL_UART_AbortTransmit() and clears the internal transmit-active
 * state when the operation completes successfully.
 *
 * This function does not change the RS485 transceiver operating mode.
 */
rs485_port_status_t rs485_port_abort_transmit(void);

/**
 * @brief Abort an active interrupt-driven UART reception.
 *
 * @return RS485_PORT_OK on success, error code otherwise.
 *
 * @details
 * Calls HAL_UART_AbortReceive() and clears the internal receive-active state
 * when the operation completes successfully.
 *
 * This function does not change the RS485 transceiver operating mode.
 */
rs485_port_status_t rs485_port_abort_receive(void);

/* ===================== Notification Hook Prototypes ====================== */

/**
 * @brief Notify completion of an interrupt-driven UART transmission.
 *
 * @details
 * This callback is invoked from HAL_UART_TxCpltCallback() after the requested
 * number of bytes has been transmitted.
 *
 * The default implementation is weak and performs no operation. The upper
 * RS485 driver may provide a strong implementation.
 *
 * This callback executes in interrupt context and must remain short and
 * ISR-safe.
 */
void rs485_port_tx_complete_callback(void);

/**
 * @brief Notify completion of an interrupt-driven UART reception.
 *
 * @details
 * This callback is invoked from HAL_UART_RxCpltCallback() after the requested
 * number of bytes has been received.
 *
 * The default implementation is weak and performs no operation. The upper
 * RS485 driver may provide a strong implementation.
 *
 * This callback executes in interrupt context and must remain short and
 * ISR-safe.
 */
void rs485_port_rx_complete_callback(void);

/**
 * @brief Notify an UART communication error.
 *
 * @param[in] error_code  UART error flags reported by HAL_UART_GetError().
 *
 * @details
 * This callback is invoked from HAL_UART_ErrorCallback().
 *
 * The default implementation is weak and performs no operation. The upper
 * RS485 driver may provide a strong implementation.
 *
 * This callback executes in interrupt context and must remain short and
 * ISR-safe.
 */
void rs485_port_error_callback(
    uint32_t error_code);

/** @} */

#endif /* RS485_PORT_H_ */
