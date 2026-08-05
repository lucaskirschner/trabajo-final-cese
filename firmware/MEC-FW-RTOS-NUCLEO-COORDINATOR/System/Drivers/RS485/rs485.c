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
 * @file    rs485.c
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-08-04
 * @brief   Interrupt-driven single-byte RS485 driver implementation.
 *
 * @details
 * This module implements a minimal single-byte communication mechanism over
 * the RS485 port layer.
 *
 * It is intended for functional validation of the UART and RS485 transceiver
 * using a conventional serial terminal such as PuTTY.
 *
 * Unlike the previous framed implementation, this version does not use:
 *
 * - Start-of-frame markers.
 * - Protocol commands.
 * - Checksums.
 * - Multi-byte frames.
 *
 * Each receive operation waits for exactly one byte:
 *
 * @code
 * rs485_receive_start()
 *          |
 *          v
 * rs485_port_receive_it(..., 1)
 *          |
 *          v
 * HAL_UART_RxCpltCallback()
 *          |
 *          v
 * rs485_port_rx_complete_callback()
 *          |
 *          v
 * rs485_receive()
 * @endcode
 *
 * Transmission remains blocking. Reception is interrupt-driven.
 *
 * For an echo test, the upper application calls rs485_process_echo() after the
 * reception-complete notification. The received byte is then transmitted back
 * without modification.
 *
 * The port notification hooks are implemented by this module. They update the
 * internal driver state and invoke application-level notification callbacks.
 *
 * The application callbacks execute in interrupt context and must remain short
 * and ISR-safe.
 *
 * This module does not use DMA, dynamic memory or RTOS services.
 *
 * @ingroup rs485
 * @{
 */

/* ============================= Includes ================================== */

#include "rs485.h"

#include "rs485_port.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ============================ Local Macros =============================== */

/**
 * @brief Number of bytes requested by each interrupt-driven reception.
 */
#define RS485_RX_SIZE                       ((uint16_t)1u)

/**
 * @brief Number of bytes transmitted by rs485_send().
 */
#define RS485_TX_SIZE                       ((uint16_t)1u)

/* ============================ Local Types ================================ */

/**
 * @brief Internal RS485 driver context.
 */
typedef struct
{
    bool initialized;

    volatile bool rx_active;
    volatile bool rx_complete;
    volatile bool rx_error;

    volatile uint32_t last_uart_error;

    uint8_t rx_data;

    rs485_port_handle_t port_handle;
} rs485_ctx_t;

/* ======================= Local Static Data ================================ */

static rs485_ctx_t g_ctx = {
    .initialized = false,
    .rx_active = false,
    .rx_complete = false,
    .rx_error = false,
    .last_uart_error = 0u,
    .rx_data = 0u,
    .port_handle = {
        .reserved = 0u
    }
};

/* ===================== Private Function Prototypes ======================= */

static rs485_status_t rs485_validate_handle(
    const rs485_handle_t * handle);

static rs485_status_t rs485_port_status_to_status(
    rs485_port_status_t port_status);

/* ===================== Port Notification Prototypes ====================== */

/**
 * @brief Receive-complete notification called by rs485_port.c.
 *
 * @details
 * This function overrides the weak notification hook provided by the RS485
 * port layer.
 */
void rs485_port_rx_complete_callback(void);

/**
 * @brief UART-error notification called by rs485_port.c.
 *
 * @param[in] error_code  STM32 HAL UART error flags.
 *
 * @details
 * This function overrides the weak notification hook provided by the RS485
 * port layer.
 */
void rs485_port_error_callback(uint32_t error_code);

/* ===================== Public Function Definitions ======================= */

rs485_status_t rs485_init(
    rs485_handle_t * handle)
{
    rs485_port_status_t port_status;
    rs485_status_t status;

    status = rs485_validate_handle(handle);
    if (status != RS485_OK)
    {
        return status;
    }

    if (g_ctx.initialized != false)
    {
        return RS485_E_STATE;
    }

    port_status = rs485_port_init(
        &g_ctx.port_handle);

    status = rs485_port_status_to_status(
        port_status);

    if (status != RS485_OK)
    {
        return status;
    }

    port_status = rs485_port_set_mode(
        RS485_PORT_MODE_RX);

    status = rs485_port_status_to_status(
        port_status);

    if (status != RS485_OK)
    {
        (void)rs485_port_deinit(
            &g_ctx.port_handle);

        return status;
    }

    g_ctx.rx_active = false;
    g_ctx.rx_complete = false;
    g_ctx.rx_error = false;
    g_ctx.last_uart_error = 0u;
    g_ctx.rx_data = 0u;
    g_ctx.initialized = true;

    return RS485_OK;
}

rs485_status_t rs485_deinit(
    rs485_handle_t * handle)
{
    rs485_port_status_t port_status;
    rs485_status_t status;

    status = rs485_validate_handle(handle);
    if (status != RS485_OK)
    {
        return status;
    }

    if (g_ctx.initialized == false)
    {
        return RS485_E_STATE;
    }

    if (g_ctx.rx_active != false)
    {
        port_status = rs485_port_abort_receive();

        status = rs485_port_status_to_status(
            port_status);

        if (status != RS485_OK)
        {
            return status;
        }
    }

    port_status = rs485_port_set_mode(
        RS485_PORT_MODE_RX);

    status = rs485_port_status_to_status(
        port_status);

    if (status != RS485_OK)
    {
        return status;
    }

    port_status = rs485_port_deinit(
        &g_ctx.port_handle);

    status = rs485_port_status_to_status(
        port_status);

    if (status != RS485_OK)
    {
        return status;
    }

    g_ctx.rx_active = false;
    g_ctx.rx_complete = false;
    g_ctx.rx_error = false;
    g_ctx.last_uart_error = 0u;
    g_ctx.rx_data = 0u;
    g_ctx.initialized = false;

    return RS485_OK;
}

rs485_status_t rs485_send(
    uint8_t data,
    uint32_t timeout_ms)
{
    rs485_port_status_t port_status;
    rs485_status_t transmit_status;
    rs485_status_t restore_status;

    if (g_ctx.initialized == false)
    {
        return RS485_E_STATE;
    }

    /*
     * An active interrupt-driven receive operation must be completed or
     * aborted before changing the half-duplex transceiver to transmit mode.
     */
    if (g_ctx.rx_active != false)
    {
        return RS485_E_STATE;
    }

    port_status = rs485_port_set_mode(
        RS485_PORT_MODE_TX);

    transmit_status = rs485_port_status_to_status(
        port_status);

    if (transmit_status != RS485_OK)
    {
        return transmit_status;
    }

    port_status = rs485_port_transmit(
        &data,
        RS485_TX_SIZE,
        timeout_ms);

    transmit_status = rs485_port_status_to_status(
        port_status);

    /*
     * Always attempt to return the transceiver to receive mode, even when the
     * blocking transmission reports an error.
     */
    port_status = rs485_port_set_mode(
        RS485_PORT_MODE_RX);

    restore_status = rs485_port_status_to_status(
        port_status);

    if (transmit_status != RS485_OK)
    {
        return transmit_status;
    }

    if (restore_status != RS485_OK)
    {
        return restore_status;
    }

    return RS485_OK;
}

rs485_status_t rs485_receive_start(void)
{
    rs485_port_status_t port_status;
    rs485_status_t status;

    if (g_ctx.initialized == false)
    {
        return RS485_E_STATE;
    }

    /*
     * A new reception cannot be started while another reception is active or
     * while a previously received byte remains unconsumed.
     */
    if ((g_ctx.rx_active != false) ||
        (g_ctx.rx_complete != false))
    {
        return RS485_E_STATE;
    }

    port_status = rs485_port_set_mode(
        RS485_PORT_MODE_RX);

    status = rs485_port_status_to_status(
        port_status);

    if (status != RS485_OK)
    {
        return status;
    }

    g_ctx.rx_error = false;
    g_ctx.last_uart_error = 0u;
    g_ctx.rx_data = 0u;

    port_status = rs485_port_receive_it(
        &g_ctx.rx_data,
        RS485_RX_SIZE);

    status = rs485_port_status_to_status(
        port_status);

    if (status == RS485_OK)
    {
        g_ctx.rx_active = true;
    }

    return status;
}

rs485_status_t rs485_receive(
    uint8_t * p_data)
{
    if (p_data == NULL)
    {
        return RS485_E_NULL;
    }

    if (g_ctx.initialized == false)
    {
        return RS485_E_STATE;
    }

    if (g_ctx.rx_error != false)
    {
        g_ctx.rx_error = false;
        g_ctx.rx_complete = false;
        g_ctx.last_uart_error = 0u;
        g_ctx.rx_data = 0u;

        return RS485_E_HW;
    }

    if (g_ctx.rx_complete == false)
    {
        return RS485_E_STATE;
    }

    *p_data = g_ctx.rx_data;

    /*
     * Mark the received byte as consumed. The upper application may now call
     * rs485_receive_start() to arm the next reception.
     */
    g_ctx.rx_complete = false;
    g_ctx.rx_data = 0u;

    return RS485_OK;
}

rs485_status_t rs485_receive_abort(void)
{
    rs485_port_status_t port_status;
    rs485_status_t status;

    if (g_ctx.initialized == false)
    {
        return RS485_E_STATE;
    }

    if (g_ctx.rx_active != false)
    {
        port_status = rs485_port_abort_receive();

        status = rs485_port_status_to_status(
            port_status);

        if (status != RS485_OK)
        {
            return status;
        }
    }

    g_ctx.rx_active = false;
    g_ctx.rx_complete = false;
    g_ctx.rx_error = false;
    g_ctx.last_uart_error = 0u;
    g_ctx.rx_data = 0u;

    return RS485_OK;
}

rs485_status_t rs485_process_echo(
    uint8_t * p_data,
    uint32_t timeout_ms)
{
    uint8_t received_data;
    rs485_status_t status;

    status = rs485_receive(
        &received_data);

    if (status != RS485_OK)
    {
        return status;
    }

    status = rs485_send(
        received_data,
        timeout_ms);

    if (status != RS485_OK)
    {
        return status;
    }

    if (p_data != NULL)
    {
        *p_data = received_data;
    }

    return RS485_OK;
}

/* ===================== Port Notification Definitions ===================== */

/**
 * @brief Handle completion of the UART reception started by the port layer.
 *
 * @details
 * This function executes in interrupt context. It updates only volatile state
 * flags and invokes the application-level receive-complete callback.
 */
void rs485_port_rx_complete_callback(void)
{
    if (g_ctx.initialized != false)
    {
        g_ctx.rx_active = false;
        g_ctx.rx_complete = true;
        g_ctx.rx_error = false;
        g_ctx.last_uart_error = 0u;

        rs485_rx_complete_callback();
    }
}

/**
 * @brief Handle a UART reception error reported by the port layer.
 *
 * @param[in] error_code  STM32 HAL UART error flags.
 *
 * @details
 * This function executes in interrupt context. It records the error, clears the
 * active reception state and invokes the application-level error callback.
 */
void rs485_port_error_callback(
    uint32_t error_code)
{
    if (g_ctx.initialized != false)
    {
        g_ctx.rx_active = false;
        g_ctx.rx_complete = false;
        g_ctx.rx_error = true;
        g_ctx.last_uart_error = error_code;

        rs485_error_callback(
            error_code);
    }
}

/* ===================== Private Function Definitions ====================== */

/**
 * @brief Validate the public RS485 driver handle.
 *
 * @param[in] handle  RS485 driver handle.
 *
 * @return RS485_OK on success, error code otherwise.
 */
static rs485_status_t rs485_validate_handle(
    const rs485_handle_t * handle)
{
    if (handle == NULL)
    {
        return RS485_E_NULL;
    }

    return RS485_OK;
}

/**
 * @brief Convert RS485 port status codes to driver status codes.
 *
 * @param[in] port_status  Status returned by the RS485 port layer.
 *
 * @return Equivalent RS485 driver status code.
 */
static rs485_status_t rs485_port_status_to_status(
    rs485_port_status_t port_status)
{
    rs485_status_t status;

    switch (port_status)
    {
        case RS485_PORT_OK:
            status = RS485_OK;
            break;

        case RS485_PORT_E_NULL:
            status = RS485_E_NULL;
            break;

        case RS485_PORT_E_PARAM:
            status = RS485_E_PARAM;
            break;

        case RS485_PORT_E_STATE:
            status = RS485_E_STATE;
            break;

        case RS485_PORT_E_TIMEOUT:
            status = RS485_E_TIMEOUT;
            break;

        case RS485_PORT_E_HW:
        default:
            status = RS485_E_HW;
            break;
    }

    return status;
}

/** @} */
