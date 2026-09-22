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
 * @file    app_user.h
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-05-27
 * @brief   User application task interface.
 *
 * @details
 * This module provides the public interface for the user application task.
 *
 * The current implementation generates periodic digital-output commands by
 * sending complete output images to the digital output message queue.
 *
 * @ingroup app_user
 * @{
 */

#ifndef APP_USER_H_
#define APP_USER_H_

/* ===================== Public Function Prototypes ======================== */

/**
 * @brief User application task.
 *
 * @param argument Task argument provided by the RTOS.
 *
 * @details
 * This task periodically sends complete digital-output images to the output
 * application task through a CMSIS-RTOS2 message queue.
 */
void app_user_task(void * argument);

#endif /* APP_USER_H_ */

/** @} */
