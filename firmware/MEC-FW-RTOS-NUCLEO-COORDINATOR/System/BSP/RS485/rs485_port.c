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
 * @file    rs485_port.c
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-08-04
 * @brief   BSP port layer implementation for the RS485 interface.
 *
 * @details
 * This module wraps STM32 HAL UART and GPIO services for a half-duplex RS485
 * interface connected to UART7.
 *
 * Transmission uses the blocking HAL_UART_Transmit() service. Reception uses
 * HAL_UART_Receive_IT(), allowing the upper application task to wait for an
 * event instead of polling the UART periodically.
 *
 * The interrupt-driven receive operation completes through the STM32 HAL
 * callbacks:
 *
 * - HAL_UART_RxCpltCallback(): Requested byte count received.
 * - HAL_UART_ErrorCallback(): UART reception error detected.
 *
 * These HAL callbacks are implemented in this file and filtered so that only
 * events associated with UART7 are processed.
 *
 * The callbacks update the local port state and invoke weak notification hooks:
 *
 * - rs485_port_rx_complete_callback().
 * - rs485_port_error_callback().
 *
 * The upper RS485 driver or application layer may provide strong definitions
 * of these functions to receive asynchronous notifications. The default weak
 * implementations perform no operation.
 *
 * The default idle state is receive mode:
 *
 * - DE = 0: RS485 driver disabled.
 * - /RE = 0: RS485 receiver enabled.
 *
 * This module does not use DMA or RTOS services.
 *
 * @ingroup rs485_port
 * @{
 */

/* ============================= Includes ================================== */

#include "rs485_port.h"

#include "main.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "swo.h"

/* ============================ Local Types ================================ */

/**
 * @brief Internal RS485 port context.
 */
typedef struct
{
    bool initialized;
    volatile bool rx_active;
    volatile uint32_t last_uart_error;
} rs485_port_ctx_t;

/* ======================= Local Static Data ================================ */

static rs485_port_ctx_t g_ctx = {
    .initialized = false,
    .rx_active = false,
    .last_uart_error = HAL_UART_ERROR_NONE
};

/* ======================= External Peripheral Data ======================== */

extern UART_HandleTypeDef huart7;

/* ===================== Private Function Prototypes ======================= */

static rs485_port_status_t rs485_port_validate_handle(
    const rs485_port_handle_t * handle);

static rs485_port_status_t rs485_port_hal_to_status(
    HAL_StatusTypeDef hal_status);

/* ===================== Notification Hook Prototypes ====================== */

/**
 * @brief Notify completion of an interrupt-driven UART7 reception.
 *
 * @details
 * This weak function is called from HAL_UART_RxCpltCallback() after the
 * requested number of bytes has been received.
 *
 * The upper driver or application layer may override this function. The
 * overriding implementation must remain short and ISR-safe.
 */
__weak void rs485_port_rx_complete_callback(void);

/**
 * @brief Notify an UART7 reception error.
 *
 * @param[in] error_code  UART error flags reported by HAL_UART_GetError().
 *
 * @details
 * This weak function is called from HAL_UART_ErrorCallback(). The upper driver
 * or application layer may override it to wake a task or record an error.
 *
 * The overriding implementation must remain short and ISR-safe.
 */
__weak void rs485_port_error_callback(uint32_t error_code);

/* ===================== Public Function Definitions ======================= */

rs485_port_status_t rs485_port_init(
    rs485_port_handle_t * handle)
{
    rs485_port_status_t status;

    status = rs485_port_validate_handle(handle);
    if (status != RS485_PORT_OK)
    {
        return status;
    }

    if (g_ctx.initialized != false)
    {
        return RS485_PORT_E_STATE;
    }

    status = rs485_port_set_mode(RS485_PORT_MODE_RX);
    if (status != RS485_PORT_OK)
    {
        return status;
    }

    g_ctx.rx_active = false;
    g_ctx.last_uart_error = HAL_UART_ERROR_NONE;
    g_ctx.initialized = true;

    return RS485_PORT_OK;
}

rs485_port_status_t rs485_port_deinit(
    rs485_port_handle_t * handle)
{
    rs485_port_status_t status;

    status = rs485_port_validate_handle(handle);
    if (status != RS485_PORT_OK)
    {
        return status;
    }

    if (g_ctx.initialized == false)
    {
        return RS485_PORT_E_STATE;
    }

    /*
     * Cancel any pending interrupt-driven reception. The return value is
     * intentionally ignored because the UART may already be idle.
     */
    (void)HAL_UART_AbortReceive(&huart7);

    g_ctx.rx_active = false;
    g_ctx.last_uart_error = HAL_UART_ERROR_NONE;

    status = rs485_port_set_mode(RS485_PORT_MODE_RX);
    if (status != RS485_PORT_OK)
    {
        return status;
    }

    g_ctx.initialized = false;

    return RS485_PORT_OK;
}

rs485_port_status_t rs485_port_set_mode(
    rs485_port_mode_t mode)
{
    switch (mode)
    {
        case RS485_PORT_MODE_RX:
            /*
             * Disable the line driver before enabling the receiver.
             */
            HAL_GPIO_WritePin(
                RS485_DE_GPIO_Port,
                RS485_DE_Pin,
                GPIO_PIN_RESET);

            HAL_GPIO_WritePin(
                RS485_RE_GPIO_Port,
                RS485_RE_Pin,
                GPIO_PIN_RESET);
            break;

        case RS485_PORT_MODE_TX:
            /*
             * Disable the receiver before enabling the line driver. This
             * prevents the transmitted frame from being received locally.
             */
            HAL_GPIO_WritePin(
                RS485_RE_GPIO_Port,
                RS485_RE_Pin,
                GPIO_PIN_SET);

            HAL_GPIO_WritePin(
                RS485_DE_GPIO_Port,
                RS485_DE_Pin,
                GPIO_PIN_SET);
            break;

        default:
            return RS485_PORT_E_PARAM;
    }

    return RS485_PORT_OK;
}

rs485_port_status_t rs485_port_transmit(
    const uint8_t * data,
    uint16_t size,
    uint32_t timeout_ms)
{
    HAL_StatusTypeDef hal_status;

    if (data == NULL)
    {
        return RS485_PORT_E_NULL;
    }

    if (size == 0u)
    {
        return RS485_PORT_E_PARAM;
    }

    if (g_ctx.initialized == false)
    {
        return RS485_PORT_E_STATE;
    }

    /*
     * A transmission must not be started while an interrupt-driven reception
     * remains active. The upper layer must abort or complete reception first.
     */
    if (g_ctx.rx_active != false)
    {
        return RS485_PORT_E_STATE;
    }

    hal_status = HAL_UART_Transmit(
        &huart7,
        (uint8_t *)data,
        size,
        timeout_ms);

    return rs485_port_hal_to_status(hal_status);
}

rs485_port_status_t rs485_port_receive_it(
    uint8_t * data,
    uint16_t size)
{
    HAL_StatusTypeDef hal_status;

    if (data == NULL)
    {
        return RS485_PORT_E_NULL;
    }

    if (size == 0u)
    {
        return RS485_PORT_E_PARAM;
    }

    if (g_ctx.initialized == false)
    {
        return RS485_PORT_E_STATE;
    }

    if (g_ctx.rx_active != false)
    {
        return RS485_PORT_E_STATE;
    }

    g_ctx.last_uart_error = HAL_UART_ERROR_NONE;

    hal_status = HAL_UART_Receive_IT(
        &huart7,
        data,
        size);

    if (hal_status == HAL_OK)
    {
        g_ctx.rx_active = true;
    }

    return rs485_port_hal_to_status(hal_status);
}

rs485_port_status_t rs485_port_abort_receive(void)
{
    HAL_StatusTypeDef hal_status;

    if (g_ctx.initialized == false)
    {
        return RS485_PORT_E_STATE;
    }

    hal_status = HAL_UART_AbortReceive(&huart7);

    if (hal_status == HAL_OK)
    {
        g_ctx.rx_active = false;
        g_ctx.last_uart_error = HAL_UART_ERROR_NONE;
    }

    return rs485_port_hal_to_status(hal_status);
}

/* ===================== HAL Callback Definitions ========================== */

/**
 * @brief STM32 HAL UART reception-complete callback.
 *
 * @param[in,out] huart  UART handle associated with the completed reception.
 *
 * @details
 * HAL calls this function after HAL_UART_Receive_IT() has received the
 * requested number of bytes.
 *
 * Only UART7 events are processed. The callback clears the local active
 * reception state and invokes rs485_port_rx_complete_callback().
 *
 * This function executes in interrupt context and must remain short.
 */
void HAL_UART_RxCpltCallback(
    UART_HandleTypeDef * huart)
{
    if (huart == &huart7)
    {
        g_ctx.rx_active = false;
        g_ctx.last_uart_error = HAL_UART_ERROR_NONE;

        rs485_port_rx_complete_callback();
    }
}

/**
 * @brief STM32 HAL UART error callback.
 *
 * @param[in,out] huart  UART handle associated with the detected error.
 *
 * @details
 * Only UART7 errors are processed. The callback stores the UART error flags,
 * clears the local active reception state and invokes
 * rs485_port_error_callback().
 *
 * Possible HAL error flags include:
 *
 * - HAL_UART_ERROR_PE: Parity error.
 * - HAL_UART_ERROR_NE: Noise error.
 * - HAL_UART_ERROR_FE: Framing error.
 * - HAL_UART_ERROR_ORE: Overrun error.
 *
 * This function executes in interrupt context and must remain short.
 */
void HAL_UART_ErrorCallback(
    UART_HandleTypeDef * huart)
{
    uint32_t error_code;

    if (huart == &huart7)
    {
        error_code = HAL_UART_GetError(huart);

        g_ctx.last_uart_error = error_code;
        g_ctx.rx_active = false;

        rs485_port_error_callback(error_code);
    }
}

/* ================= Notification Hook Definitions ========================= */

/**
 * @brief Default reception-complete notification hook.
 *
 * @details
 * This weak implementation intentionally performs no operation. The upper
 * driver or application layer may provide a strong definition.
 */
__weak void rs485_port_rx_complete_callback(void)
{
    /* Upper-layer notification hook. */
}

/**
 * @brief Default UART-error notification hook.
 *
 * @param[in] error_code  UART error flags.
 *
 * @details
 * This weak implementation intentionally performs no operation. The upper
 * driver or application layer may provide a strong definition.
 */
__weak void rs485_port_error_callback(uint32_t error_code)
{
    (void)error_code;
}

/* ===================== Private Function Definitions ====================== */

/**
 * @brief Validate the public RS485 port handle.
 *
 * @param[in] handle  RS485 port handle.
 *
 * @return RS485_PORT_OK on success, error code otherwise.
 */
static rs485_port_status_t rs485_port_validate_handle(
    const rs485_port_handle_t * handle)
{
    if (handle == NULL)
    {
        return RS485_PORT_E_NULL;
    }

    return RS485_PORT_OK;
}

/**
 * @brief Convert STM32 HAL status codes to RS485 port status codes.
 *
 * @param[in] hal_status  Status returned by an STM32 HAL function.
 *
 * @return Equivalent RS485 port status code.
 */
static rs485_port_status_t rs485_port_hal_to_status(
    HAL_StatusTypeDef hal_status)
{
    rs485_port_status_t status;

    switch (hal_status)
    {
        case HAL_OK:
            status = RS485_PORT_OK;
            break;

        case HAL_TIMEOUT:
            status = RS485_PORT_E_TIMEOUT;
            break;

        case HAL_BUSY:
            status = RS485_PORT_E_STATE;
            break;

        case HAL_ERROR:
        default:
            status = RS485_PORT_E_HW;
            break;
    }

    return status;
}

/** @} */
