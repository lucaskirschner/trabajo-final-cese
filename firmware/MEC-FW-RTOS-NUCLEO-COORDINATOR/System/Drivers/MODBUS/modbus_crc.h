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
 * @file    modbus_crc.h
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-08-02
 * @brief   CRC-16 calculation utilities for MODBUS RTU frames.
 *
 * @details
 * This module calculates the 16-bit cyclic redundancy check used by the
 * MODBUS RTU serial-line transmission mode.
 *
 * The MODBUS Application Protocol Specification V1.1b3 defines the generic
 * application data unit and establishes that a serial-line ADU contains an
 * additional server-address field and an error-checking field. The maximum
 * serial-line ADU size is 256 bytes. See sections 4.1 and 4.2, pages 3 to 5.
 *
 * The detailed CRC generation procedure is defined by MODBUS over Serial Line
 * Specification and Implementation Guide V1.02:
 *
 * - Section 2.5.1.2, CRC Checking, pages 14 and 15.
 * - Appendix B, section 6.2.2, CRC Generation, pages 39 to 43.
 *
 * According to that specification:
 *
 * - The CRC register is initialized to 0xFFFF.
 * - Each message byte is XORed with the low-order part of the CRC register.
 * - The register is shifted eight times toward its least significant bit.
 * - Polynomial 0xA001 is applied whenever the extracted LSB is one.
 * - The resulting low-order CRC byte is transmitted first.
 * - The resulting high-order CRC byte is transmitted second.
 *
 * The calculation covers the complete MODBUS RTU ADU except for the two CRC
 * bytes that are appended after the calculation.
 *
 * This module returns the CRC as a native uint16_t value. It does not swap its
 * bytes internally. The caller must explicitly append:
 *
 * - crc & 0x00FF as the first CRC byte.
 * - (crc >> 8) & 0x00FF as the second CRC byte.
 *
 * This representation is consistent with modbus_rtu_master.c, which performs
 * the byte serialization explicitly when building an RTU request and performs
 * the inverse reconstruction when validating a received response.
 *
 * @ingroup modbus_crc
 * @{
 */

#ifndef MODBUS_CRC_H_
#define MODBUS_CRC_H_

/* ============================= Includes ================================== */

#include <stdint.h>

/* ===================== Public Function Prototypes ======================== */

/**
 * @brief Calculate the CRC-16 of a MODBUS RTU message buffer.
 *
 * @details
 * The function implements the bitwise procedure described in MODBUS over
 * Serial Line Specification and Implementation Guide V1.02, section 2.5.1.2,
 * pages 14 and 15, and Appendix B, section 6.2.2, pages 39 to 43.
 *
 * The buffer must contain every RTU ADU byte that precedes the CRC field,
 * including:
 *
 * - Server address.
 * - Function code.
 * - Function-specific data.
 *
 * The two received or reserved CRC bytes must not be included in @p size.
 *
 * The returned value is not byte-swapped. To append it to an RTU ADU, the
 * caller must store the low-order byte first and the high-order byte second:
 *
 * @code
 * uint16_t crc;
 *
 * crc = modbus_crc_calculate(frame, frame_size_without_crc);
 *
 * frame[frame_size_without_crc] =
 *     (uint8_t)(crc & 0x00FFu);
 *
 * frame[frame_size_without_crc + 1u] =
 *     (uint8_t)((crc >> 8u) & 0x00FFu);
 * @endcode
 *
 * @param[in] data  Pointer to the message bytes included in the calculation.
 * @param[in] size  Number of message bytes included in the calculation.
 *
 * @return Calculated 16-bit MODBUS RTU CRC value.
 *
 * @pre @p data must reference at least @p size readable bytes when
 *      @p size is greater than zero.
 *
 * @note When @p size is zero, no message bytes are processed and the returned
 *       value is the initial CRC register value, 0xFFFF.
 *
 * @note When @p data is NULL, the function returns the initial CRC value
 *       0xFFFF as a defensive fallback. A NULL pointer with a nonzero
 *       @p size represents an invalid caller condition and must not be used
 *       as a valid protocol result.
 */
uint16_t modbus_crc_calculate(
    const uint8_t *data,
    uint16_t size);

#endif /* MODBUS_CRC_H_ */

/** @} */
