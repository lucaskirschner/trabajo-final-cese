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
 * @file    swo.h
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-04-07
 * @brief   SWO output redirection interface.
 *
 * @details
 * This module provides the declaration required to redirect standard output
 * functions, such as printf(), to the SWO/ITM debug interface available on
 * Cortex-M microcontrollers. It is intended for debug and development use in
 * environments where the debugger supports ITM stimulus ports.
 *
 * @ingroup swo
 * @{
 */

#ifndef INC_SWO_H_
#define INC_SWO_H_

/* ============================= Includes ================================== */

#include <stdio.h>

/* ============================== Macros =================================== */


/* ============================== Types ==================================== */


/* ===================== Public Function Prototypes ======================== */

/**
 * @brief Redirect standard output data through the SWO interface.
 *
 * @param file File descriptor associated with the output stream.
 *             This parameter is kept for compatibility with the standard
 *             library interface and is not used by the implementation.
 * @param ptr  Pointer to the data buffer to be transmitted.
 * @param len  Number of bytes to transmit.
 *
 * @return Number of bytes written to the SWO output channel.
 *
 * @note
 * This function is typically called internally by printf() and related
 * standard library output functions.
 *
 * @warning
 * This implementation is intended for debugging purposes only. Its correct
 * operation depends on proper debugger, SWO clock, and ITM configuration.
 */
int _write(int file, char *ptr, int len);

#endif /* INC_SWO_H_ */

/** @} */
