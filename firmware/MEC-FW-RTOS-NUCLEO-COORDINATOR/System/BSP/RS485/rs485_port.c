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
 * @date    2026-06-22
 * @brief   BSP port layer implementation for the RS485 interface.
 *
 * @details
 * This module wraps STM32 HAL UART and GPIO services for a blocking RS485
 * implementation. It does not use DMA or interrupts.
 *
 * The default idle state is receive mode:
 * - DE = 0: RS485 driver disabled.
 * - /RE = 0: RS485 receiver enabled.
 *
 * @ingroup rs485_port
 * @{
 */

/* ============================= Includes ================================== */

#include "rs485_port.h"
#include "main.h"
#include <stddef.h>

/* ============================ Local Types ================================ */

typedef struct
{
    bool initialized;
} rs485_port_ctx_t;

/* ======================= Local (static) Data ============================= */

static rs485_port_ctx_t g_ctx = {
    .initialized = false
};

/* ======================= External Peripheral Data ======================== */

extern UART_HandleTypeDef huart4;

/* ===================== Local Function Prototypes ========================= */

static rs485_port_status_t rs485_port_validate_handle(
    const rs485_port_handle_t *handle);

static rs485_port_status_t rs485_port_hal_to_status(HAL_StatusTypeDef hal_status);

/* ===================== Public Function Definitions ======================= */

rs485_port_status_t rs485_port_init(rs485_port_handle_t *handle)
{
    rs485_port_status_t status = RS485_PORT_OK;

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

    g_ctx.initialized = true;

    return RS485_PORT_OK;
}

rs485_port_status_t rs485_port_deinit(rs485_port_handle_t *handle)
{
    rs485_port_status_t status = RS485_PORT_OK;

    status = rs485_port_validate_handle(handle);
    if (status != RS485_PORT_OK)
    {
        return status;
    }

    if (g_ctx.initialized == false)
    {
        return RS485_PORT_E_STATE;
    }

    status = rs485_port_set_mode(RS485_PORT_MODE_RX);
    if (status != RS485_PORT_OK)
    {
        return status;
    }

    g_ctx.initialized = false;

    return RS485_PORT_OK;
}

rs485_port_status_t rs485_port_set_mode(rs485_port_mode_t mode)
{
    rs485_port_status_t status = RS485_PORT_OK;

    switch (mode)
    {
        case RS485_PORT_MODE_RX:
            HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET);
            break;

        case RS485_PORT_MODE_TX:
            HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
            break;

        default:
            status = RS485_PORT_E_PARAM;
            break;
    }

    return status;
}

rs485_port_status_t rs485_port_transmit(
    const uint8_t *data,
    uint16_t size,
    uint32_t timeout_ms)
{
    HAL_StatusTypeDef hal_status = HAL_OK;

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

    hal_status = HAL_UART_Transmit(&huart4, (uint8_t *)data, size, timeout_ms);

    return rs485_port_hal_to_status(hal_status);
}

rs485_port_status_t rs485_port_receive(
    uint8_t *data,
    uint16_t size,
    uint32_t timeout_ms)
{
    HAL_StatusTypeDef hal_status = HAL_OK;

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

    hal_status = HAL_UART_Receive(&huart4, data, size, timeout_ms);

    return rs485_port_hal_to_status(hal_status);
}

/* ===================== Private Function Definitions ====================== */

/**
 * @brief Validate the RS485 port handle.
 *
 * @param[in] handle  RS485 port handle.
 *
 * @return RS485_PORT_OK on success, error code otherwise.
 */
static rs485_port_status_t rs485_port_validate_handle(
    const rs485_port_handle_t *handle)
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
 * @param[in] hal_status  HAL status code returned by STM32 HAL functions.
 *
 * @return Equivalent RS485 port status code.
 */
static rs485_port_status_t rs485_port_hal_to_status(HAL_StatusTypeDef hal_status)
{
    rs485_port_status_t status = RS485_PORT_OK;

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
