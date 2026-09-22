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
 * @file    swo.c
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-04-07
 * @brief   SWO output redirection implementation.
 *
 * @details
 * This module implements the low-level write hook used by the C standard
 * library to redirect formatted output, such as printf(), to the SWO/ITM
 * debug interface.
 *
 * The implementation sends each byte sequentially through the ITM stimulus
 * port by calling ITM_SendChar(). It is intended only for debugging and
 * should not be considered a real-time-safe or high-throughput logging
 * mechanism.
 *
 * @ingroup swo
 * @{
 */

/* ============================= Includes ================================== */

#include "swo.h"
#include "stm32h5xx.h"

/* ============================ Local Macros =============================== */


/* ============================ Local Types ================================ */


/* ======================= Local (static) Data ============================= */


/* ===================== Local Function Prototypes ========================= */


/* ===================== Public Function Definitions ======================= */

int _write(int file, char *ptr, int len)
{
    int i;

    (void)file;

    for (i = 0; i < len; i++)
    {
        ITM_SendChar(*ptr);
        ptr++;
    }

    return len;
}

/** @} */
