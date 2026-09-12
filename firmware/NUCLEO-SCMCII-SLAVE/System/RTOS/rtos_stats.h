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
 * @file    rtos_stats.h
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-06-18
 * @brief   FreeRTOS run-time statistics timer interface.
 *
 * @details
 * Provides the functions required by FreeRTOS to configure and read the
 * run-time statistics counter.
 *
 * @ingroup rtos_stats
 * @{
 */

#ifndef RTOS_STATS_H_
#define RTOS_STATS_H_

/* ===================== Public Function Prototypes ======================== */

/**
 * @brief Configure the run-time statistics counter.
 *
 * @details
 * Enables the Cortex-M trace unit and starts the DWT cycle counter.
 */
void configureTimerForRunTimeStats(void);

/**
 * @brief Get the current run-time statistics counter value.
 *
 * @return Current DWT cycle counter value.
 */
unsigned long getRunTimeCounterValue(void);

#endif /* RTOS_STATS_H_ */

/** @} */
