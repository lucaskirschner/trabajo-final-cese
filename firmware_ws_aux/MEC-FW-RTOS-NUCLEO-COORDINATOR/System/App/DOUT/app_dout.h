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
 * @file    app_dout.h
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-05-26
 * @brief   Digital output application task interface.
 *
 * @details
 * This module provides a minimal CMSIS-RTOS2-based interface for controlling
 * the VNI8200XP-32 digital output driver.
 *
 * Public API:
 * - app_dout_set_outputs(): enqueue one complete output image.
 * - app_dout_task(): task entry point used by the RTOS.
 *
 * Immediate API errors are returned through app_dout_status_t.
 * Deferred driver or RTOS faults detected inside app_dout_task() are reported
 * through doutFaultEventHandle.
 *
 * @ingroup app_dout
 * @{
 */

#ifndef APP_DOUT_H_
#define APP_DOUT_H_

/* ============================= Includes ================================== */

#include <stdint.h>

/* ============================= Defines =================================== */

/**
 * @brief Returned VNI diagnostic frame parity fault.
 */
#define APP_DOUT_FAULT_RX_PARITY_ERROR    ((uint32_t)(1u << 0u))

/**
 * @brief VNI SPI communication fault reported by the device.
 */
#define APP_DOUT_FAULT_SPI_ERROR          ((uint32_t)(1u << 1u))

/**
 * @brief VNI internal DC-DC feedback fault.
 */
#define APP_DOUT_FAULT_FB_ERROR           ((uint32_t)(1u << 2u))

/**
 * @brief VNI case-temperature warning.
 */
#define APP_DOUT_FAULT_TWARN              ((uint32_t)(1u << 3u))

/**
 * @brief VNI Power Good diagnostic fault.
 */
#define APP_DOUT_FAULT_PG_ERROR           ((uint32_t)(1u << 4u))

/**
 * @brief VNI per-channel overtemperature fault.
 */
#define APP_DOUT_FAULT_OVT_ERROR          ((uint32_t)(1u << 5u))

/**
 * @brief Generic hardware or low-level access fault.
 */
#define APP_DOUT_FAULT_HW_ERROR           ((uint32_t)(1u << 6u))

/**
 * @brief VNI or shared I/O port timeout fault.
 */
#define APP_DOUT_FAULT_TIMEOUT            ((uint32_t)(1u << 7u))

/**
 * @brief Invalid parameter fault.
 */
#define APP_DOUT_FAULT_PARAM_ERROR        ((uint32_t)(1u << 8u))

/**
 * @brief Null pointer fault.
 */
#define APP_DOUT_FAULT_NULL_ERROR         ((uint32_t)(1u << 9u))

/**
 * @brief Output command queue full fault.
 */
#define APP_DOUT_FAULT_QUEUE_FULL         ((uint32_t)(1u << 10u))

/**
 * @brief Unexpected CMSIS-RTOS2 fault.
 */
#define APP_DOUT_FAULT_OS_ERROR           ((uint32_t)(1u << 11u))

/**
 * @brief Mask containing all digital output fault flags.
 */
#define APP_DOUT_FAULT_ERROR_MASK         (APP_DOUT_FAULT_RX_PARITY_ERROR | \
                                           APP_DOUT_FAULT_SPI_ERROR       | \
                                           APP_DOUT_FAULT_FB_ERROR        | \
                                           APP_DOUT_FAULT_TWARN           | \
                                           APP_DOUT_FAULT_PG_ERROR        | \
                                           APP_DOUT_FAULT_OVT_ERROR       | \
                                           APP_DOUT_FAULT_HW_ERROR        | \
                                           APP_DOUT_FAULT_TIMEOUT         | \
                                           APP_DOUT_FAULT_PARAM_ERROR     | \
                                           APP_DOUT_FAULT_NULL_ERROR      | \
                                           APP_DOUT_FAULT_QUEUE_FULL      | \
                                           APP_DOUT_FAULT_OS_ERROR)

/* ============================== Types ==================================== */

/**
 * @brief Status codes returned by the digital output application API.
 *
 * @details
 * These status codes report only immediate API-level results.
 *
 * APP_DOUT_OK returned by app_dout_set_outputs() means that the output image
 * was successfully enqueued. It does not mean that the VNI8200XP-32 has already
 * applied the output image.
 */
typedef enum
{
    APP_DOUT_OK = 0,
    APP_DOUT_E_QUEUE_FULL,
    APP_DOUT_E_OS
} app_dout_status_t;

/* ===================== Public Function Prototypes ======================== */

/**
 * @brief Digital output application task.
 *
 * @param argument Task argument provided by the RTOS.
 *
 * @details
 * This task blocks waiting for output images from the output queue. Each
 * received image is written to the VNI8200XP-32 driver.
 *
 * Deferred faults are reported through doutFaultEventHandle.
 */
void app_dout_task(void * argument);

/**
 * @brief Enqueue one complete output image.
 *
 * @param output_image Output image to be applied to OUT1..OUT8.
 *
 * @return
 * - APP_DOUT_OK: Output image enqueued successfully.
 * - APP_DOUT_E_QUEUE_FULL: Output queue is full.
 * - APP_DOUT_E_OS: Unexpected RTOS error.
 *
 * @details
 * This function does not access the VNI8200XP-32 directly. It only enqueues
 * the requested output image. The actual SPI transaction is performed later by
 * app_dout_task().
 */
app_dout_status_t app_dout_set_outputs(uint8_t output_image);

#endif /* APP_DOUT_H_ */

/** @} */
