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
 * @file    sn65hvd82_port.c
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-04-22
 * @brief   Port implementation for SN65HVD82 (STM32H5 HAL).
 *
 * @details
 * Uses:
 *  - UART4 via HAL_UART_Transmit / HAL_UART_Receive
 *  - DE pin : RS485_DE_Pin
 *  - RE pin : RS485_RE_Pin
 *  - main.h is included as requested to reuse CubeMX pin macros
 *
 * @ingroup sn65hvd82
 * @{
 */

/* ============================= Includes ================================== */

#include "sn65hvd82_port.h"
#include "main.h"
#include "stm32h5xx_hal.h"

/* ============================ Local Macros =============================== */

/**
 * @brief UART timeout used by blocking transfers.
 */
#define SN65HVD82_PORT_UART_TIMEOUT_MS             ((uint32_t)100u)

/**
 * @brief Maximum number of bytes accepted by HAL UART blocking API.
 */
#define SN65HVD82_PORT_UART_XFER_MAX               ((size_t)0xFFFFu)

/* ======================= Local (static) Data ============================= */

extern UART_HandleTypeDef huart4;
static UART_HandleTypeDef * const huart = &huart4;

/* ========================== Private Prototypes =========================== */

static void sn65hvd82_port_set_de(bool state);
static void sn65hvd82_port_set_re(bool state);

/* ===================== Public Function Definitions ======================= */

sn65hvd82_port_status_t sn65hvd82_port_set_tx_mode(void)
{
    sn65hvd82_port_set_de(true);
    sn65hvd82_port_set_re(true);

    return SN65HVD82_PORT_OK;
}

sn65hvd82_port_status_t sn65hvd82_port_set_tx_loopback_mode(void)
{
    sn65hvd82_port_set_de(true);
    sn65hvd82_port_set_re(false);

    return SN65HVD82_PORT_OK;
}

sn65hvd82_port_status_t sn65hvd82_port_set_rx_mode(void)
{
    sn65hvd82_port_set_de(false);
    sn65hvd82_port_set_re(false);

    return SN65HVD82_PORT_OK;
}

sn65hvd82_port_status_t sn65hvd82_port_set_standby_mode(void)
{
    sn65hvd82_port_set_de(false);
    sn65hvd82_port_set_re(true);

    return SN65HVD82_PORT_OK;
}

sn65hvd82_port_status_t sn65hvd82_port_uart_transmit(const uint8_t * data,
                                                     size_t length)
{
    HAL_StatusTypeDef hal_status;

    if (data == NULL)
    {
        return SN65HVD82_PORT_E_NULL;
    }

    if ((length == 0u) || (length > SN65HVD82_PORT_UART_XFER_MAX))
    {
        return SN65HVD82_PORT_E_PARAM;
    }

    hal_status = HAL_UART_Transmit(huart,
                                   data,
                                   (uint16_t)length,
                                   SN65HVD82_PORT_UART_TIMEOUT_MS);

    if (hal_status == HAL_TIMEOUT)
    {
        return SN65HVD82_PORT_E_TIMEOUT;
    }

    if (hal_status != HAL_OK)
    {
        return SN65HVD82_PORT_E_HW;
    }

    return SN65HVD82_PORT_OK;
}

sn65hvd82_port_status_t sn65hvd82_port_uart_receive(uint8_t * const data,
                                                    size_t length)
{
    HAL_StatusTypeDef hal_status;

    if (data == NULL)
    {
        return SN65HVD82_PORT_E_NULL;
    }

    if ((length == 0u) || (length > SN65HVD82_PORT_UART_XFER_MAX))
    {
        return SN65HVD82_PORT_E_PARAM;
    }

    hal_status = HAL_UART_Receive(huart,
                                  data,
                                  (uint16_t)length,
                                  SN65HVD82_PORT_UART_TIMEOUT_MS);

    if (hal_status == HAL_TIMEOUT)
    {
        return SN65HVD82_PORT_E_TIMEOUT;
    }

    if (hal_status != HAL_OK)
    {
        return SN65HVD82_PORT_E_HW;
    }

    return SN65HVD82_PORT_OK;
}

sn65hvd82_port_status_t sn65hvd82_port_uart_wait_tx_complete(void)
{
    uint32_t tick_start;

    tick_start = HAL_GetTick();

    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET)
    {
        if ((HAL_GetTick() - tick_start) >= SN65HVD82_PORT_UART_TIMEOUT_MS)
        {
            return SN65HVD82_PORT_E_TIMEOUT;
        }
    }

    return SN65HVD82_PORT_OK;
}

sn65hvd82_port_status_t sn65hvd82_port_uart_flush_rx(void)
{
    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);

    return SN65HVD82_PORT_OK;
}

sn65hvd82_port_status_t sn65hvd82_port_transmit_frame(const uint8_t * data,
                                                      size_t length)
{
    sn65hvd82_port_status_t status;

    if (data == NULL)
    {
        return SN65HVD82_PORT_E_NULL;
    }

    if ((length == 0u) || (length > SN65HVD82_PORT_UART_XFER_MAX))
    {
        return SN65HVD82_PORT_E_PARAM;
    }

    status = sn65hvd82_port_set_tx_mode();
    if (status != SN65HVD82_PORT_OK)
    {
        return status;
    }

    status = sn65hvd82_port_uart_transmit(data, length);
    if (status != SN65HVD82_PORT_OK)
    {
        (void)sn65hvd82_port_set_rx_mode();
        return status;
    }

    status = sn65hvd82_port_uart_wait_tx_complete();
    if (status != SN65HVD82_PORT_OK)
    {
        (void)sn65hvd82_port_set_rx_mode();
        return status;
    }

    status = sn65hvd82_port_set_rx_mode();
    if (status != SN65HVD82_PORT_OK)
    {
        return status;
    }

    return SN65HVD82_PORT_OK;
}

sn65hvd82_port_status_t sn65hvd82_port_transmit_frame_loopback(const uint8_t * tx_data,
                                                               uint8_t * const rx_data,
                                                               size_t length)
{
    sn65hvd82_port_status_t status;

    if ((tx_data == NULL) || (rx_data == NULL))
    {
        return SN65HVD82_PORT_E_NULL;
    }

    if ((length == 0u) || (length > SN65HVD82_PORT_UART_XFER_MAX))
    {
        return SN65HVD82_PORT_E_PARAM;
    }

    status = sn65hvd82_port_uart_flush_rx();
    if (status != SN65HVD82_PORT_OK)
    {
        return status;
    }

    status = sn65hvd82_port_set_tx_loopback_mode();
    if (status != SN65HVD82_PORT_OK)
    {
        return status;
    }

    status = sn65hvd82_port_uart_transmit(tx_data, length);
    if (status != SN65HVD82_PORT_OK)
    {
        (void)sn65hvd82_port_set_rx_mode();
        return status;
    }

    status = sn65hvd82_port_uart_wait_tx_complete();
    if (status != SN65HVD82_PORT_OK)
    {
        (void)sn65hvd82_port_set_rx_mode();
        return status;
    }

    status = sn65hvd82_port_uart_receive(rx_data, length);
    if (status != SN65HVD82_PORT_OK)
    {
        (void)sn65hvd82_port_set_rx_mode();
        return status;
    }

    status = sn65hvd82_port_set_rx_mode();
    if (status != SN65HVD82_PORT_OK)
    {
        return status;
    }

    return SN65HVD82_PORT_OK;
}

void sn65hvd82_port_delay_ms(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

/* ===================== Private Function Definitions ====================== */

/**
 * @brief Set the state of the RS-485 driver-enable pin.
 *
 * @param state
 * - true : Assert DE.
 * - false: Deassert DE.
 */
static void sn65hvd82_port_set_de(bool state)
{
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port,
                      RS485_DE_Pin,
                      (state == true) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief Set the state of the RS-485 receiver-enable pin.
 *
 * @param state
 * - true : Disable receiver (RE = 1).
 * - false: Enable receiver (RE = 0).
 *
 * @details
 * The SN65HVD82 receiver enable input is active low.
 */
static void sn65hvd82_port_set_re(bool state)
{
    HAL_GPIO_WritePin(RS485_RE_GPIO_Port,
                      RS485_RE_Pin,
                      (state == true) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/** @} */
