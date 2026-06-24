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
 * @date    2026-06-24
 * @brief   RS485 transceiver driver implementation.
 *
 * @details
 * This module implements a simple blocking RS485 driver over the RS485 port
 * layer. The driver controls the TX/RX direction and leaves the transceiver in
 * receive mode when idle.
 *
 * The module does not perform frame parsing, CRC calculation or timing rules
 * required by higher-level protocols. Those responsibilities belong to the
 * protocol layer, for example Modbus RTU master or slave.
 *
 * @ingroup rs485
 * @{
 */

/* ============================= Includes ================================== */

#include "rs485.h"

#include "rs485_port.h"

#include <stddef.h>

/* ============================ Local Types ================================ */

typedef struct
{
    bool initialized;
    rs485_port_handle_t port_handle;
} rs485_ctx_t;

/* ======================= Local (static) Data ============================= */

static rs485_ctx_t g_ctx = {
    .initialized = false,
    .port_handle = {
        .reserved = 0u
    }
};

/* ===================== Local Function Prototypes ========================= */

static rs485_status_t rs485_validate_handle(const rs485_handle_t *handle);

static rs485_status_t rs485_port_status_to_driver_status(
    rs485_port_status_t port_status);

static rs485_port_mode_t rs485_mode_to_port_mode(rs485_mode_t mode);

/* ===================== Public Function Definitions ======================= */

rs485_status_t rs485_init(rs485_handle_t *handle)
{
    rs485_status_t status = RS485_OK;
    rs485_port_status_t port_status = RS485_PORT_OK;

    status = rs485_validate_handle(handle);
    if (status != RS485_OK)
    {
        return status;
    }

    if (g_ctx.initialized != false)
    {
        return RS485_E_STATE;
    }

    port_status = rs485_port_init(&g_ctx.port_handle);
    status = rs485_port_status_to_driver_status(port_status);
    if (status != RS485_OK)
    {
        return status;
    }

    status = rs485_set_mode(RS485_MODE_RX);
    if (status != RS485_OK)
    {
        (void)rs485_port_deinit(&g_ctx.port_handle);
        return status;
    }

    g_ctx.initialized = true;

    return RS485_OK;
}

rs485_status_t rs485_deinit(rs485_handle_t *handle)
{
    rs485_status_t status = RS485_OK;
    rs485_port_status_t port_status = RS485_PORT_OK;

    status = rs485_validate_handle(handle);
    if (status != RS485_OK)
    {
        return status;
    }

    if (g_ctx.initialized == false)
    {
        return RS485_E_STATE;
    }

    status = rs485_set_mode(RS485_MODE_RX);
    if (status != RS485_OK)
    {
        return status;
    }

    port_status = rs485_port_deinit(&g_ctx.port_handle);
    status = rs485_port_status_to_driver_status(port_status);
    if (status != RS485_OK)
    {
        return status;
    }

    g_ctx.initialized = false;

    return RS485_OK;
}

rs485_status_t rs485_set_mode(rs485_mode_t mode)
{
    rs485_port_status_t port_status = RS485_PORT_OK;
    rs485_port_mode_t port_mode = RS485_PORT_MODE_RX;

    if ((mode != RS485_MODE_RX) && (mode != RS485_MODE_TX))
    {
        return RS485_E_PARAM;
    }

    port_mode = rs485_mode_to_port_mode(mode);

    port_status = rs485_port_set_mode(port_mode);

    return rs485_port_status_to_driver_status(port_status);
}

rs485_status_t rs485_send(
    const uint8_t *data,
    uint16_t size,
    uint32_t timeout_ms)
{
    rs485_status_t status = RS485_OK;
    rs485_port_status_t port_status = RS485_PORT_OK;

    if (data == NULL)
    {
        return RS485_E_NULL;
    }

    if (size == 0u)
    {
        return RS485_E_PARAM;
    }

    if (g_ctx.initialized == false)
    {
        return RS485_E_STATE;
    }

    status = rs485_set_mode(RS485_MODE_TX);
    if (status != RS485_OK)
    {
        return status;
    }

    port_status = rs485_port_transmit(data, size, timeout_ms);
    status = rs485_port_status_to_driver_status(port_status);

    (void)rs485_set_mode(RS485_MODE_RX);

    return status;
}

rs485_status_t rs485_receive(
    uint8_t *data,
    uint16_t size,
    uint32_t timeout_ms)
{
    rs485_status_t status = RS485_OK;
    rs485_port_status_t port_status = RS485_PORT_OK;

    if (data == NULL)
    {
        return RS485_E_NULL;
    }

    if (size == 0u)
    {
        return RS485_E_PARAM;
    }

    if (g_ctx.initialized == false)
    {
        return RS485_E_STATE;
    }

    status = rs485_set_mode(RS485_MODE_RX);
    if (status != RS485_OK)
    {
        return status;
    }

    port_status = rs485_port_receive(data, size, timeout_ms);

    return rs485_port_status_to_driver_status(port_status);
}

/* ===================== Local Function Definitions ======================== */

/**
 * @brief Validate the RS485 driver handle.
 *
 * @param[in] handle  RS485 driver handle.
 *
 * @return RS485_OK on success, error code otherwise.
 */
static rs485_status_t rs485_validate_handle(const rs485_handle_t *handle)
{
    if (handle == NULL)
    {
        return RS485_E_NULL;
    }

    return RS485_OK;
}

/**
 * @brief Convert RS485 port status codes to RS485 driver status codes.
 *
 * @param[in] port_status  Status code returned by the RS485 port layer.
 *
 * @return Equivalent RS485 driver status code.
 */
static rs485_status_t rs485_port_status_to_driver_status(
    rs485_port_status_t port_status)
{
    rs485_status_t status = RS485_OK;

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

/**
 * @brief Convert RS485 driver mode to RS485 port mode.
 *
 * @param[in] mode  RS485 driver mode.
 *
 * @return Equivalent RS485 port mode.
 */
static rs485_port_mode_t rs485_mode_to_port_mode(rs485_mode_t mode)
{
    rs485_port_mode_t port_mode = RS485_PORT_MODE_RX;

    switch (mode)
    {
        case RS485_MODE_RX:
            port_mode = RS485_PORT_MODE_RX;
            break;

        case RS485_MODE_TX:
            port_mode = RS485_PORT_MODE_TX;
            break;

        default:
            port_mode = RS485_PORT_MODE_RX;
            break;
    }

    return port_mode;
}

/** @} */
