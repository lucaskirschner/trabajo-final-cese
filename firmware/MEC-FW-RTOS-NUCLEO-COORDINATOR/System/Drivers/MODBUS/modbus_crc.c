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
 * @file    modbus_crc.c
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-08-02
 * @brief   CRC-16 calculation implementation for MODBUS RTU frames.
 *
 * @details
 * This module implements the bitwise CRC generation procedure specified by
 * MODBUS over Serial Line Specification and Implementation Guide V1.02:
 *
 * - Section 2.5.1.2, CRC Checking, pages 14 and 15.
 * - Appendix B, section 6.2.2, CRC Generation, pages 39 to 43.
 *
 * The procedure defined by the specification is:
 *
 * 1. Initialize a 16-bit CRC register with 0xFFFF.
 * 2. XOR each message byte with the low-order part of the CRC register.
 * 3. Shift the register one bit toward the least significant bit.
 * 4. If the extracted LSB was one, XOR the shifted register with 0xA001.
 * 5. Repeat the shift and conditional XOR eight times for each message byte.
 * 6. Continue until every message byte has been processed.
 *
 * The reflected polynomial value used by the bitwise procedure is 0xA001,
 * corresponding to the generating polynomial:
 *
 *     1 + x^2 + x^15 + x^16
 *
 * The CRC covers the entire RTU message preceding the CRC field. Start bits,
 * stop bits and parity bits of the UART character format are not included in
 * the calculation.
 *
 * The function returns the native 16-bit CRC value without internally swapping
 * its bytes. MODBUS RTU requires the low-order byte to be appended to the frame
 * first, followed by the high-order byte. This serialization is performed by
 * the MODBUS RTU master module.
 *
 * @ingroup modbus_crc
 * @{
 */

/* ============================= Includes ================================== */

#include "modbus_crc.h"

#include <stddef.h>

/* ============================ Local Macros =============================== */

/**
 * @brief Initial value of the 16-bit MODBUS CRC register.
 *
 * MODBUS over Serial Line Specification and Implementation Guide V1.02,
 * Appendix B, section 6.2.2, specifies that the register is initially loaded
 * with all bits set to one.
 */
#define MODBUS_CRC_INITIAL_VALUE    (0xFFFFu)

/**
 * @brief Reflected polynomial used by the MODBUS CRC bitwise algorithm.
 *
 * The value 0xA001 is specified in MODBUS over Serial Line Specification and
 * Implementation Guide V1.02, Appendix B, section 6.2.2, pages 39 and 40.
 */
#define MODBUS_CRC_POLYNOMIAL       (0xA001u)

/**
 * @brief Number of CRC-processing iterations performed for each message byte.
 */
#define MODBUS_CRC_BITS_PER_BYTE    (8u)

/**
 * @brief Mask used to test the least significant bit of the CRC register.
 */
#define MODBUS_CRC_LSB_MASK         (0x0001u)

/* ===================== Public Function Definitions ======================= */

uint16_t modbus_crc_calculate(
    const uint8_t *data,
    uint16_t size)
{
    uint16_t crc = MODBUS_CRC_INITIAL_VALUE;
    uint16_t byte_index = 0u;
    uint8_t bit_index = 0u;

    /*
     * A zero-length message leaves the preloaded CRC register unchanged.
     * A NULL pointer is treated as an invalid caller condition and returns the
     * same initial value without dereferencing the pointer.
     */
    if ((data == NULL) || (size == 0u))
    {
        return crc;
    }

    for (byte_index = 0u; byte_index < size; byte_index++)
    {
        /*
         * XOR the next eight-bit message byte with the low-order portion of
         * the current 16-bit CRC register.
         */
        crc ^= (uint16_t)data[byte_index];

        for (bit_index = 0u;
             bit_index < MODBUS_CRC_BITS_PER_BYTE;
             bit_index++)
        {
            /*
             * The specification requires examining the LSB before shifting.
             * When it is one, the shifted register is XORed with 0xA001.
             */
            if ((crc & MODBUS_CRC_LSB_MASK) != 0u)
            {
                crc >>= 1u;
                crc ^= MODBUS_CRC_POLYNOMIAL;
            }
            else
            {
                crc >>= 1u;
            }
        }
    }

    return crc;
}

/** @} */
