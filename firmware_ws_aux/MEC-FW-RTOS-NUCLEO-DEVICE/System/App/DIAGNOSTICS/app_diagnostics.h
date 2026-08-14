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
 * @file    app_diagnostics.h
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-05-27
 * @brief   Diagnostics application task interface.
 *
 * @details
 * This module provides the public interface for the diagnostics application
 * task.
 *
 * The diagnostics task monitors application-level fault event objects generated
 * by the main firmware services and reports detected conditions through the
 * configured debug output interface.
 *
 * The current implementation monitors fault events associated with:
 *
 * - Digital-output application layer.
 * - Digital-input application layer.
 * - Radio communication application layer.
 * - RS485 communication application layer.
 *
 * Each application module maintains an independent fault event object. The
 * diagnostics task periodically checks these objects and reports any pending
 * diagnostic condition.
 *
 * @ingroup app_diagnostics
 * @{
 */

#ifndef APP_DIAGNOSTICS_H_
#define APP_DIAGNOSTICS_H_

/* ===================== Public Function Prototypes ======================== */

/**
 * @brief Diagnostics application task.
 *
 * @param argument Task argument provided by the RTOS.
 *
 * @details
 * This task monitors the fault event objects exposed by the application
 * modules and reports detected conditions through the configured debug output
 * interface.
 *
 * The task currently supervises digital-output, digital-input, radio and RS485
 * application faults.
 */
void app_diagnostics_task(
    void * argument);

#endif /* APP_DIAGNOSTICS_H_ */

/** @} */
