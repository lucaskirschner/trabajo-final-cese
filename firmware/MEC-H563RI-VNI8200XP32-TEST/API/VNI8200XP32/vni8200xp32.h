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
 * @file    vni8200xp32.h
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-04-21
 * @brief   High-level driver for the ST VNI8200XP-32 digital output driver.
 *
 * @details
 * This module provides a minimal high-level interface for commanding the eight
 * digital outputs of the VNI8200XP-32 through its 16-bit SPI mode.
 *
 * In this operating mode, the transmitted frame contains:
 * - 8 output-control bits
 * - 4 command parity bits
 *
 * At the same time, the device returns a 16-bit fault frame containing:
 * - 8 per-channel fault bits
 * - 4 general diagnostic bits:
 *   - FB_OK
 *   - TWARN
 *   - PC
 *   - PG
 * - 4 parity bits for the returned frame
 *
 * The driver builds the command frame, performs the SPI transaction, validates
 * the received diagnostic frame, and reports either successful output update or
 * the first detected fault condition.
 *
 * @ingroup vni8200xp32
 * @{
 */

#ifndef VNI8200XP32_H_
#define VNI8200XP32_H_

/* ============================= Includes ================================== */

#include <stdint.h>

/* ============================== Types ==================================== */

/**
 * @brief Return codes for VNI8200XP-32 operations.
 */
typedef enum
{
    VNI8200XP32_OK = 0,
    VNI8200XP32_E_NULL,
    VNI8200XP32_E_PARAM,
    VNI8200XP32_E_HW,
    VNI8200XP32_E_TIMEOUT,
    VNI8200XP32_E_RX_PARITY,
    VNI8200XP32_E_SPI,
    VNI8200XP32_E_FB,
    VNI8200XP32_E_TWARN,
    VNI8200XP32_E_PG,
    VNI8200XP32_E_OVT
} vni8200xp32_status_t;

/* ===================== Public Function Prototypes ======================== */

/**
 * @brief Write the eight output states to the VNI8200XP-32.
 *
 * @param outputs Output image to be applied to OUT1..OUT8.
 *
 * @return
 * - VNI8200XP32_OK: Outputs were written and the returned fault frame was valid.
 * - VNI8200XP32_E_NULL: Unexpected null pointer reported by the port layer.
 * - VNI8200XP32_E_PARAM: Invalid parameter detected by the port layer.
 * - VNI8200XP32_E_HW: Hardware communication error reported by the port layer.
 * - VNI8200XP32_E_TIMEOUT: Communication timeout reported by the port layer.
 * - VNI8200XP32_E_RX_PARITY: Returned fault frame parity check failed.
 * - VNI8200XP32_E_SPI: Device reported an SPI communication fault (PC bit set).
 * - VNI8200XP32_E_FB: DC-DC feedback status not OK (FB_OK bit low).
 * - VNI8200XP32_E_TWARN: Case temperature warning detected.
 * - VNI8200XP32_E_PG: Power Good diagnostic not asserted.
 * - VNI8200XP32_E_OVT: At least one channel overtemperature fault was reported.
 *
 * @details
 * The @p outputs byte is transmitted in 16-bit SPI mode together with the
 * command parity nibble required by the device.
 *
 * The returned 16-bit fault frame is validated before interpreting the
 * diagnostic bits.
 *
 * On any error, the function reports the first detected fault condition
 * according to the driver's validation sequence.
 */
vni8200xp32_status_t vni8200xp32_write_outputs(uint8_t outputs);

#endif /* VNI8200XP32_H_ */

/** @} */
