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
 * @file    sclt38bt8.h
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-04-21
 * @brief   High-level driver for the ST SCLT3-8BT8 digital input serializer.
 *
 * @details
 * This module provides a minimal high-level interface for reading the eight
 * digital input states from the SCLT3-8BT8 in 16-bit SPI mode.
 *
 * In this operating mode, the device returns:
 * - 8 input-state bits
 * - 4 parity bits
 * - 1 undervoltage alarm bit
 * - 1 overtemperature alarm bit
 * - 2 stop bits
 *
 * The driver validates the received frame and reports either successful input
 * acquisition or the first detected error/diagnostic condition.
 *
 * @ingroup sclt38bt8
 * @{
 */

#ifndef SCLT38BT8_H_
#define SCLT38BT8_H_

/* ============================= Includes ================================== */

#include <stdint.h>

/* ============================== Types ==================================== */

/**
 * @brief Return codes for SCLT3-8BT8 operations.
 */
typedef enum
{
    SCLT38BT8_OK = 0,
    SCLT38BT8_E_NULL,
    SCLT38BT8_E_PARAM,
    SCLT38BT8_E_HW,
    SCLT38BT8_E_TIMEOUT,
    SCLT38BT8_E_STOP_BITS,
    SCLT38BT8_E_PARITY,
    SCLT38BT8_E_UV,
    SCLT38BT8_E_OT,
    SCLT38BT8_E_POWER_LOSS
} sclt38bt8_status_t;

/* ===================== Public Function Prototypes ======================== */

/**
 * @brief Read and validate the eight digital inputs from the SCLT3-8BT8.
 *
 * @param[out] p_inputs Pointer to the byte where the eight input states will
 *                      be stored on successful read.
 *
 * @return
 * - SCLT38BT8_OK: Frame was received and validated successfully.
 * - SCLT38BT8_E_NULL: Null pointer passed in @p p_inputs.
 * - SCLT38BT8_E_PARAM: Invalid parameter detected by the port layer.
 * - SCLT38BT8_E_HW: Hardware communication error reported by the port layer.
 * - SCLT38BT8_E_TIMEOUT: Communication timeout reported by the port layer.
 * - SCLT38BT8_E_STOP_BITS: Invalid stop bits detected in the received frame.
 * - SCLT38BT8_E_PARITY: Parity check failed.
 * - SCLT38BT8_E_UV: Undervoltage alarm detected.
 * - SCLT38BT8_E_OT: Overtemperature alarm detected.
 * - SCLT38BT8_E_POWER_LOSS: Power-loss condition detected.
 *
 * @details
 * On success, the input states are written into @p p_inputs.
 * On any error, @p p_inputs is left unchanged and the caller is responsible for
 * deciding how to proceed.
 */
sclt38bt8_status_t sclt38bt8_read_inputs(uint8_t * const p_inputs);

#endif /* SCLT38BT8_H_ */

/** @} */
