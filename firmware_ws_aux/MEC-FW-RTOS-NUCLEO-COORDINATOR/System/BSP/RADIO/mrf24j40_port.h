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
 * @file    mrf24j40_port.h
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-02-23
 * @brief   Port layer for MRF24J40 (STM32H5 HAL).
 *
 * @details
 * This module isolates MCU/HAL dependencies (SPI, GPIO, delays).
 *
 * @ingroup mrf24j40
 * @{
 */

#ifndef MRF24J40_PORT_H_
#define MRF24J40_PORT_H_

/* ============================= Includes ================================== */

#include <stdint.h>

/* ============================== Macros =================================== */

/* ============================== Types ==================================== */

typedef enum
{
    MRF24J40_PORT_OK = 0,
    MRF24J40_PORT_E_NULL,
    MRF24J40_PORT_E_PARAM,
    MRF24J40_PORT_E_HW,
    MRF24J40_PORT_E_TIMEOUT
} mrf24j40_port_status_t;

/* ===================== Public Function Prototypes ======================== */

/**
 * @brief Write one byte to a short address register.
 *
 * @param reg Short address register (0x00 to 0x3F).
 * @param value Data byte to write.
 *
 * @return
 * - MRF24J40_PORT_OK: Operation completed successfully.
 * - MRF24J40_PORT_E_PARAM: Invalid short address.
 * - MRF24J40_PORT_E_HW: SPI transaction failed.
 * - MRF24J40_PORT_E_TIMEOUT: SPI transaction timed out.
 *
 * @details
 * Short address write format:
 * - First byte: 0 A5 A4 A3 A2 A1 A0 1
 * - Second byte: data
 */
mrf24j40_port_status_t mrf24j40_port_write_short(uint8_t reg, uint8_t value);

/**
 * @brief Read one byte from a short address register.
 *
 * @param reg Short address register (0x00 to 0x3F).
 * @param[out] value Pointer to the destination where the read byte will be stored.
 *
 * @return
 * - MRF24J40_PORT_OK: Operation completed successfully.
 * - MRF24J40_PORT_E_NULL: Null pointer passed in @p value.
 * - MRF24J40_PORT_E_PARAM: Invalid short address.
 * - MRF24J40_PORT_E_HW: SPI transaction failed.
 * - MRF24J40_PORT_E_TIMEOUT: SPI transaction timed out.
 *
 * @details
 * Short address read format:
 * - First byte: 0 A5 A4 A3 A2 A1 A0 0
 * - Second byte: dummy byte transmitted by host, data returned by slave
 */
mrf24j40_port_status_t mrf24j40_port_read_short(uint8_t reg, uint8_t * const value);

/**
 * @brief Write one byte to a long address register or FIFO location.
 *
 * @param reg Long address register or FIFO location (0x000 to 0x38F).
 * @param value Data byte to write.
 *
 * @return
 * - MRF24J40_PORT_OK: Operation completed successfully.
 * - MRF24J40_PORT_E_PARAM: Invalid long address.
 * - MRF24J40_PORT_E_HW: SPI transaction failed.
 * - MRF24J40_PORT_E_TIMEOUT: SPI transaction timed out.
 *
 * @details
 * Long address write format:
 * - First byte : 1 A9 A8 A7 A6 A5 A4 A3
 * - Second byte: A2 A1 A0 1 X X X X
 * - Third byte : data
 *
 * In software, this is encoded as:
 * - header[0] = ((reg >> 3) & 0x7F) | 0x80
 * - header[1] = (reg << 5) | 0x10
 * - header[2] = value
 */
mrf24j40_port_status_t mrf24j40_port_write_long(uint16_t reg, uint8_t value);

/**
 * @brief Read one byte from a long address register or FIFO location.
 *
 * @param reg Long address register or FIFO location (0x000 to 0x38F).
 * @param[out] value Pointer to the destination where the read byte will be stored.
 *
 * @return
 * - MRF24J40_PORT_OK: Operation completed successfully.
 * - MRF24J40_PORT_E_NULL: Null pointer passed in @p value.
 * - MRF24J40_PORT_E_PARAM: Invalid long address.
 * - MRF24J40_PORT_E_HW: SPI transaction failed.
 * - MRF24J40_PORT_E_TIMEOUT: SPI transaction timed out.
 *
 * @details
 * Long address read format:
 * - First byte : 1 A9 A8 A7 A6 A5 A4 A3
 * - Second byte: A2 A1 A0 0 X X X X
 * - Third byte : dummy byte transmitted by host, data returned by slave
 *
 * In software, this is encoded as:
 * - header[0] = ((reg >> 3) & 0x7F) | 0x80
 * - header[1] = (reg << 5)
 */
mrf24j40_port_status_t mrf24j40_port_read_long(uint16_t reg, uint8_t * const value);

/**
 * @brief Delay execution for the specified number of milliseconds.
 *
 * @param delay_ms Delay time in milliseconds.
 *
 * @details
 * This function provides a blocking delay using the underlying HAL time base.
 * It is intended for non-time-critical waits such as device startup,
 * reset recovery, or basic initialization sequences.
 *
 * @note
 * This implementation relies on HAL_Delay() and therefore its resolution is
 * limited to the HAL tick period. It is not suitable for precise short delays
 * in the microsecond range.
 */
void mrf24j40_port_delay_ms(uint32_t delay_ms);


#endif /* MRF24J40_PORT_H_ */

/** @} */
