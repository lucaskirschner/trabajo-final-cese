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
 * @file    io_port.c
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-04-20
 * @brief   Shared SPI1 port implementation for industrial input/output modules.
 *
 * @details
 * Uses:
 *  - SPI1 via HAL SPI services
 *  - CS pin for VNI8200XP-32: SPI1_CS_VNI_Pin
 *  - CS pin for SCLT3: SPI1_CS_CLT_Pin
 *  - main.h is included to reuse CubeMX pin macros
 *
 * @ingroup io_port
 * @{
 */

/* ============================= Includes ================================== */

#include "io_port.h"
#include "main.h"
#include "stm32h5xx_hal.h"

/* ============================ Local Macros =============================== */

/**
 * @brief SPI timeout used by blocking transfers.
 */
#define IO_PORT_SPI_TIMEOUT_MS                  ((uint32_t)100u)

/* ======================= Local (static) Data ============================= */

extern SPI_HandleTypeDef hspi2;
static SPI_HandleTypeDef * const hspi = &hspi2;

/**
 * @brief Current owner of the shared SPI1 bus.
 */
static io_port_device_t io_port_owner = IO_PORT_DEVICE_NONE;

/* ========================== Private Prototypes =========================== */

static bool io_port_is_valid_device(io_port_device_t device);
static void io_port_cs_assert(io_port_device_t device);
static void io_port_cs_deassert(io_port_device_t device);

/* ===================== Public Function Definitions ======================= */

io_port_status_t io_port_transmit_receive(io_port_device_t device,
                                          const uint8_t * const tx_buffer,
                                          uint8_t * const rx_buffer,
                                          size_t length)
{
    HAL_StatusTypeDef hal_status;

    if (tx_buffer == NULL)
    {
        return IO_PORT_E_NULL;
    }

    if (rx_buffer == NULL)
    {
        return IO_PORT_E_NULL;
    }

    if (io_port_is_valid_device(device) == false)
    {
        return IO_PORT_E_PARAM;
    }

    if (length == 0u)
    {
        return IO_PORT_E_PARAM;
    }

    if (io_port_owner != IO_PORT_DEVICE_NONE)
    {
        return IO_PORT_E_BUSY;
    }

    io_port_owner = device;
    io_port_cs_assert(device);

    hal_status = HAL_SPI_TransmitReceive(hspi,
                                         (uint8_t *)tx_buffer,
                                         rx_buffer,
                                         (uint16_t)length,
                                         IO_PORT_SPI_TIMEOUT_MS);

    io_port_cs_deassert(device);
    io_port_owner = IO_PORT_DEVICE_NONE;

    if (hal_status == HAL_TIMEOUT)
    {
        return IO_PORT_E_TIMEOUT;
    }

    if (hal_status != HAL_OK)
    {
        return IO_PORT_E_HW;
    }

    return IO_PORT_OK;
}

io_port_status_t io_port_transmit(io_port_device_t device,
                                  const uint8_t * const tx_buffer,
                                  size_t length)
{
    HAL_StatusTypeDef hal_status;

    if (tx_buffer == NULL)
    {
        return IO_PORT_E_NULL;
    }

    if (io_port_is_valid_device(device) == false)
    {
        return IO_PORT_E_PARAM;
    }

    if (length == 0u)
    {
        return IO_PORT_E_PARAM;
    }

    if (io_port_owner != IO_PORT_DEVICE_NONE)
    {
        return IO_PORT_E_BUSY;
    }

    io_port_owner = device;
    io_port_cs_assert(device);

    hal_status = HAL_SPI_Transmit(hspi,
                                  (uint8_t *)tx_buffer,
                                  (uint16_t)length,
                                  IO_PORT_SPI_TIMEOUT_MS);

    io_port_cs_deassert(device);
    io_port_owner = IO_PORT_DEVICE_NONE;

    if (hal_status == HAL_TIMEOUT)
    {
        return IO_PORT_E_TIMEOUT;
    }

    if (hal_status != HAL_OK)
    {
        return IO_PORT_E_HW;
    }

    return IO_PORT_OK;
}

io_port_status_t io_port_receive(io_port_device_t device,
                                 uint8_t * const rx_buffer,
                                 size_t length)
{
    HAL_StatusTypeDef hal_status;

    if (rx_buffer == NULL)
    {
        return IO_PORT_E_NULL;
    }

    if (io_port_is_valid_device(device) == false)
    {
        return IO_PORT_E_PARAM;
    }

    if (length == 0u)
    {
        return IO_PORT_E_PARAM;
    }

    if (io_port_owner != IO_PORT_DEVICE_NONE)
    {
        return IO_PORT_E_BUSY;
    }

    io_port_owner = device;
    io_port_cs_assert(device);

    hal_status = HAL_SPI_Receive(hspi,
                                 rx_buffer,
                                 (uint16_t)length,
                                 IO_PORT_SPI_TIMEOUT_MS);

    io_port_cs_deassert(device);
    io_port_owner = IO_PORT_DEVICE_NONE;

    if (hal_status == HAL_TIMEOUT)
    {
        return IO_PORT_E_TIMEOUT;
    }

    if (hal_status != HAL_OK)
    {
        return IO_PORT_E_HW;
    }

    return IO_PORT_OK;
}

/* ===================== Private Function Definitions ====================== */

/**
 * @brief Check whether the specified device identifier is valid.
 *
 * @param device Device identifier to validate.
 *
 * @return
 * - true: Valid device identifier.
 * - false: Invalid device identifier.
 */
static bool io_port_is_valid_device(io_port_device_t device)
{
    bool is_valid = false;

    switch (device)
    {
        case IO_PORT_DEVICE_SCLT3:
        case IO_PORT_DEVICE_VNI8200XP32:
            is_valid = true;
            break;

        case IO_PORT_DEVICE_NONE:
        default:
            is_valid = false;
            break;
    }

    return is_valid;
}

/**
 * @brief Assert the selected device chip select signal.
 *
 * @param device Target device.
 *
 * @details
 * Chip select signals are active low.
 */
static void io_port_cs_assert(io_port_device_t device)
{
    switch (device)
    {
        case IO_PORT_DEVICE_SCLT3:
            HAL_GPIO_WritePin(SPI2_CS_CLT_GPIO_Port,
                              SPI2_CS_CLT_Pin,
                              GPIO_PIN_RESET);
            break;

        case IO_PORT_DEVICE_VNI8200XP32:
            HAL_GPIO_WritePin(SPI2_CS_VNI_GPIO_Port,
                              SPI2_CS_VNI_Pin,
                              GPIO_PIN_RESET);
            break;

        case IO_PORT_DEVICE_NONE:
        default:
            /* No action */
            break;
    }
}

/**
 * @brief Deassert the selected device chip select signal.
 *
 * @param device Target device.
 *
 * @details
 * Chip select signals are active low.
 */
static void io_port_cs_deassert(io_port_device_t device)
{
    switch (device)
    {
        case IO_PORT_DEVICE_SCLT3:
            HAL_GPIO_WritePin(SPI2_CS_CLT_GPIO_Port,
                              SPI2_CS_CLT_Pin,
                              GPIO_PIN_SET);
            break;

        case IO_PORT_DEVICE_VNI8200XP32:
            HAL_GPIO_WritePin(SPI2_CS_VNI_GPIO_Port,
                              SPI2_CS_VNI_Pin,
                              GPIO_PIN_SET);
            break;

        case IO_PORT_DEVICE_NONE:
        default:
            /* No action */
            break;
    }
}

/** @} */
