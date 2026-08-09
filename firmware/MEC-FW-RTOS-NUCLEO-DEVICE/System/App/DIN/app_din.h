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
 * @file    app_din.h
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-05-27
 * @brief   Digital input application task interface.
 *
 * @details
 * This module provides a CMSIS-RTOS2-based interface for the digital input
 * application layer.
 *
 * The digital input task periodically reads the SCLT3-8BT8 driver and stores
 * the last valid input image internally. Application code can retrieve the
 * last valid image using app_din_read().
 *
 * Access to the last input image is protected with a CMSIS-RTOS2 mutex.
 *
 * Immediate API errors are returned through app_din_status_t.
 * Deferred driver or RTOS faults detected inside app_din_task() are reported
 * through dinFaultEventHandle.
 *
 * @ingroup app_din
 * @{
 */

#ifndef APP_DIN_H_
#define APP_DIN_H_

/* ============================= Includes ================================== */

#include <stdint.h>

/* ============================= Defines =================================== */

/**
 * @brief Digital input polling period, in RTOS ticks.
 *
 * @details
 * With a 1 ms RTOS tick, this value corresponds to a 20 ms sampling period.
 */
#define APP_DIN_POLL_PERIOD_TICKS      ((uint32_t)20u)

/**
 * @brief SCLT3-8BT8 invalid stop-bit fault.
 */
#define APP_DIN_FAULT_STOP_BITS_ERROR  ((uint32_t)(1u << 0u))

/**
 * @brief SCLT3-8BT8 returned-frame parity fault.
 */
#define APP_DIN_FAULT_PARITY_ERROR     ((uint32_t)(1u << 1u))

/**
 * @brief SCLT3-8BT8 undervoltage fault.
 */
#define APP_DIN_FAULT_UV_ERROR         ((uint32_t)(1u << 2u))

/**
 * @brief SCLT3-8BT8 overtemperature fault.
 */
#define APP_DIN_FAULT_OT_ERROR         ((uint32_t)(1u << 3u))

/**
 * @brief SCLT3-8BT8 power-loss fault.
 */
#define APP_DIN_FAULT_POWER_LOSS       ((uint32_t)(1u << 4u))

/**
 * @brief Generic hardware or low-level access fault.
 */
#define APP_DIN_FAULT_HW_ERROR         ((uint32_t)(1u << 5u))

/**
 * @brief Digital input timeout fault.
 */
#define APP_DIN_FAULT_TIMEOUT          ((uint32_t)(1u << 6u))

/**
 * @brief Invalid parameter fault.
 */
#define APP_DIN_FAULT_PARAM_ERROR      ((uint32_t)(1u << 7u))

/**
 * @brief Null pointer fault.
 */
#define APP_DIN_FAULT_NULL_ERROR       ((uint32_t)(1u << 8u))

/**
 * @brief Unexpected CMSIS-RTOS2 fault.
 */
#define APP_DIN_FAULT_OS_ERROR         ((uint32_t)(1u << 9u))

/**
 * @brief Mask containing all digital input fault flags.
 */
#define APP_DIN_FAULT_ERROR_MASK       (APP_DIN_FAULT_STOP_BITS_ERROR | \
                                        APP_DIN_FAULT_PARITY_ERROR    | \
                                        APP_DIN_FAULT_UV_ERROR        | \
                                        APP_DIN_FAULT_OT_ERROR        | \
                                        APP_DIN_FAULT_POWER_LOSS      | \
                                        APP_DIN_FAULT_HW_ERROR        | \
                                        APP_DIN_FAULT_TIMEOUT         | \
                                        APP_DIN_FAULT_PARAM_ERROR     | \
                                        APP_DIN_FAULT_NULL_ERROR      | \
                                        APP_DIN_FAULT_OS_ERROR)

/* ============================== Types ==================================== */

/**
 * @brief Status codes returned by the digital input application API.
 *
 * @details
 * These status codes report only immediate API-level results.
 *
 * APP_DIN_OK returned by app_din_read() means that the last valid input image
 * was copied successfully to the caller-provided destination.
 */
typedef enum
{
    APP_DIN_OK = 0,
    APP_DIN_E_NULL,
    APP_DIN_E_NOT_READY,
    APP_DIN_E_BUSY,
    APP_DIN_E_OS
} app_din_status_t;

/* ===================== Public Function Prototypes ======================== */

/**
 * @brief Digital input application task.
 *
 * @param argument Task argument provided by the RTOS.
 *
 * @details
 * This task periodically reads the SCLT3-8BT8 digital input driver.
 *
 * On successful acquisition, the last valid input image is updated under mutex
 * protection. On error, a diagnostic fault flag is set in dinFaultEventHandle.
 */
void app_din_task(void * argument);

/**
 * @brief Read the last valid digital input image.
 *
 * @param[out] p_input_image Pointer where the last valid input image will be
 *                           stored.
 *
 * @return
 * - APP_DIN_OK: Last valid input image copied successfully.
 * - APP_DIN_E_NULL: Null pointer passed in @p p_input_image.
 * - APP_DIN_E_NOT_READY: No valid input image has been acquired yet.
 * - APP_DIN_E_BUSY: Input image mutex is currently unavailable.
 * - APP_DIN_E_OS: Unexpected RTOS error.
 *
 * @details
 * This function is non-blocking. It attempts to acquire the input data mutex
 * with zero timeout. The stored input image is not consumed, so repeated calls
 * return the latest valid value until the task updates it again.
 */
app_din_status_t app_din_read(uint8_t * p_input_image);

#endif /* APP_DIN_H_ */

/** @} */
