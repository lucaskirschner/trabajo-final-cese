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
 * @file    mrf24j40_port.c
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-02-23
 * @brief   Port implementation for MRF24J40 (STM32H5 HAL).
 *
 * @details
 * Uses:
 *  - SPI3 via HAL_SPI_TransmitReceive
 *  - CS pin: SPI3_CS_Pin
 *  - INT pin: INT_Pin
 *  - WAKE pin: WAKE_Pin
 *	- main.h is included as requested to reuse CubeMX pin macros
 *
 * @ingroup mrf24j40
 * @{
 */

/* ============================= Includes ================================== */

#include "mrf24j40_port.h"

#include "main.h"               /* GPIO pin mapping from user */
#include "stm32h5xx_hal.h"      /* HAL SPI/GPIO/Delay */

#include "cmsis_os2.h"

/* ============================ Local Macros =============================== */
/**
 * @brief SPI timeout used by blocking transfers.
 */
#define MRF24J40_SPI_TIMEOUT_MS                  ((uint32_t)100u)

/**
 * @brief Dummy byte used during SPI read operations.
 */
#define MRF24J40_SPI_DUMMY_BYTE                  ((uint8_t)0x00u)

/**
 * @brief Maximum valid short address.
 */
#define MRF24J40_SHORT_ADDR_MAX                  ((uint8_t)0x3Fu)

/**
 * @brief Maximum valid long address.
 */
#define MRF24J40_LONG_ADDR_MAX                   ((uint16_t)0x038Fu)

/* ======================= Local (static) Data ============================= */

extern SPI_HandleTypeDef hspi3;
static SPI_HandleTypeDef * const hspi = &hspi3;

/* ========================== Private Prototypes =========================== */
static void mrf24j40_port_cs_assert(void);
static void mrf24j40_port_cs_deassert(void);

/* ===================== Public Function Definitions ======================= */

mrf24j40_port_status_t mrf24j40_port_write_short(uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef hal_status;
    uint8_t tx_buffer[2];
    uint8_t rx_buffer[2];

    if (reg > MRF24J40_SHORT_ADDR_MAX)
    {
        return MRF24J40_PORT_E_PARAM;
    }

    tx_buffer[0] = (uint8_t)((reg << 1u) | 0x01u);
    tx_buffer[1] = value;

    mrf24j40_port_cs_assert();

    hal_status = HAL_SPI_TransmitReceive(hspi,
                                         tx_buffer,
                                         rx_buffer,
                                         (uint16_t)sizeof(tx_buffer),
                                         MRF24J40_SPI_TIMEOUT_MS);

    mrf24j40_port_cs_deassert();

    if (hal_status == HAL_TIMEOUT)
    {
        return MRF24J40_PORT_E_TIMEOUT;
    }

    if (hal_status != HAL_OK)
    {
        return MRF24J40_PORT_E_HW;
    }

    return MRF24J40_PORT_OK;
}

mrf24j40_port_status_t mrf24j40_port_read_short(uint8_t reg, uint8_t * const value)
{
    HAL_StatusTypeDef hal_status;
    uint8_t tx_buffer[2];
    uint8_t rx_buffer[2];

    if (value == NULL)
    {
        return MRF24J40_PORT_E_NULL;
    }

    if (reg > MRF24J40_SHORT_ADDR_MAX)
    {
        return MRF24J40_PORT_E_PARAM;
    }

    tx_buffer[0] = (uint8_t)((reg & 0b00111111) << 1u);
    tx_buffer[1] = MRF24J40_SPI_DUMMY_BYTE;

    mrf24j40_port_cs_assert();

    hal_status = HAL_SPI_TransmitReceive(hspi,
                                         tx_buffer,
                                         rx_buffer,
                                         (uint16_t)sizeof(tx_buffer),
                                         MRF24J40_SPI_TIMEOUT_MS);

    mrf24j40_port_cs_deassert();

    if (hal_status == HAL_TIMEOUT)
    {
        return MRF24J40_PORT_E_TIMEOUT;
    }

    if (hal_status != HAL_OK)
    {
        return MRF24J40_PORT_E_HW;
    }

    *value = rx_buffer[1];

    return MRF24J40_PORT_OK;
}

mrf24j40_port_status_t mrf24j40_port_write_long(uint16_t reg, uint8_t value)
{
    HAL_StatusTypeDef hal_status;
    uint8_t tx_buffer[3];
    uint8_t rx_buffer[3];

    if (reg > MRF24J40_LONG_ADDR_MAX)
    {
        return MRF24J40_PORT_E_PARAM;
    }

    tx_buffer[0] = (uint8_t)(((reg >> 3u) & 0x7Fu) | 0x80u);
    tx_buffer[1] = (uint8_t)((reg << 5u) | 0x10u);
    tx_buffer[2] = value;

    mrf24j40_port_cs_assert();

    hal_status = HAL_SPI_TransmitReceive(hspi,
                                         tx_buffer,
                                         rx_buffer,
                                         (uint16_t)sizeof(tx_buffer),
                                         MRF24J40_SPI_TIMEOUT_MS);

    mrf24j40_port_cs_deassert();

    if (hal_status == HAL_TIMEOUT)
    {
        return MRF24J40_PORT_E_TIMEOUT;
    }

    if (hal_status != HAL_OK)
    {
        return MRF24J40_PORT_E_HW;
    }

    return MRF24J40_PORT_OK;
}

mrf24j40_port_status_t mrf24j40_port_read_long(uint16_t reg, uint8_t * const value)
{
    HAL_StatusTypeDef hal_status;
    uint8_t tx_buffer[3];
    uint8_t rx_buffer[3];

    if (value == NULL)
    {
        return MRF24J40_PORT_E_NULL;
    }

    if (reg > MRF24J40_LONG_ADDR_MAX)
    {
        return MRF24J40_PORT_E_PARAM;
    }

    tx_buffer[0] = (uint8_t)(((reg >> 3u) & 0x7Fu) | 0x80u);
    tx_buffer[1] = (uint8_t)(reg << 5u);
    tx_buffer[2] = MRF24J40_SPI_DUMMY_BYTE;

    mrf24j40_port_cs_assert();

    hal_status = HAL_SPI_TransmitReceive(hspi,
                                         tx_buffer,
                                         rx_buffer,
                                         (uint16_t)sizeof(tx_buffer),
                                         MRF24J40_SPI_TIMEOUT_MS);

    mrf24j40_port_cs_deassert();

    if (hal_status == HAL_TIMEOUT)
    {
        return MRF24J40_PORT_E_TIMEOUT;
    }

    if (hal_status != HAL_OK)
    {
        return MRF24J40_PORT_E_HW;
    }

    *value = rx_buffer[2];

    return MRF24J40_PORT_OK;
}

void mrf24j40_port_delay_ms(uint32_t delay_ms)
{
    if (delay_ms == 0u)
    {
        return;
    }

    if (osKernelGetState() == osKernelRunning)
    {
        osDelay(delay_ms);
    }
    else
    {
        HAL_Delay(delay_ms);
    }
}

/* ===================== Private Function Definitions ====================== */

/**
 * @brief Assert the MRF24J40 chip select signal.
 */
static void mrf24j40_port_cs_assert(void)
{
    HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Deassert the MRF24J40 chip select signal.
 */
static void mrf24j40_port_cs_deassert(void)
{
    HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
}


/** @} */
