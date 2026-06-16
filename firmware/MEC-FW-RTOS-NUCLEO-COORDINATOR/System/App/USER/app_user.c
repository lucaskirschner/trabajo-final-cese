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
 * @file    app_user.c
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-05-27
 * @brief   User application task implementation.
 *
 * @details
 * This module implements a simple user application task.
 *
 * The task periodically checks the radio RX queue using the app_radio public
 * API. If one received byte is available, it prints the byte and then blocks
 * again using osDelay().
 *
 * @ingroup app_user
 * @{
 */

/* ============================= Includes ================================== */

#include "app_user.h"
#include "app_radio.h"
#include "swo.h"

#include "cmsis_os2.h"

#include <stdint.h>
#include <stdio.h>

/* ============================ Local Macros =============================== */

/**
 * @brief Period, in milliseconds, between radio RX queue checks.
 */
#define APP_USER_OUTPUT_UPDATE_PERIOD_MS   ((uint32_t)1000u)

/* ========================== Private Prototypes =========================== */

static void app_user_print_radio_byte(uint8_t data);

/* ===================== Public Function Definitions ======================= */

void app_user_task(void * argument)
{
    uint8_t data;
    app_radio_status_t radio_status;

    (void)argument;

    for (;;)
    {
        /* Check the radio RX queue once. This API is non-blocking and returns
         * immediately if no received byte is currently available.
         */
        radio_status = app_radio_receive(&data);

        if (radio_status == APP_RADIO_OK)
        {
            app_user_print_radio_byte(data);
        }
        else if (radio_status == APP_RADIO_E_QUEUE_EMPTY)
        {
            /* No received data available during this period. */
        }
        else if (radio_status != APP_RADIO_E_NULL)
        {
            printf("[USER] app_radio_receive() failed: %d\r\n",
                   (int)radio_status);
        }
        else
        {
            /* Null pointer is not expected here. */
        }

        osDelay(APP_USER_OUTPUT_UPDATE_PERIOD_MS);
    }
}

/* ===================== Private Function Definitions ====================== */

/**
 * @brief Print one received radio byte.
 *
 * @param data Received radio byte.
 */
static void app_user_print_radio_byte(uint8_t data)
{
    printf("[RADIO] RX byte: 0x%02X\r\n", data);
}

/** @} */
