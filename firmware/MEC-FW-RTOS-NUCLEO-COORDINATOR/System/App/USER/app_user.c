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
 * The task periodically toggles a complete digital-output image using the
 * app_dout public API and reads the latest digital-input image using the
 * app_din public API.
 *
 * @ingroup app_user
 * @{
 */

/* ============================= Includes ================================== */

#include "app_user.h"
#include "app_dout.h"
#include "app_din.h"
#include "swo.h"

#include "cmsis_os2.h"

#include <stdint.h>
#include <stdio.h>

/* ============================ Local Macros =============================== */

/**
 * @brief Initial digital output image.
 */
#define APP_USER_OUTPUT_IMAGE_INITIAL      ((uint8_t)0x00u)

/**
 * @brief Output image mask used to toggle all digital output channels.
 */
#define APP_USER_OUTPUT_IMAGE_TOGGLE_MASK  ((uint8_t)0xFFu)

/**
 * @brief Period, in milliseconds, between output image updates.
 */
#define APP_USER_OUTPUT_UPDATE_PERIOD_MS   ((uint32_t)1000u)

/* ========================== Private Prototypes =========================== */

static void app_user_print_inputs(uint8_t inputs);

/* ===================== Public Function Definitions ======================= */

void app_user_task(void * argument)
{
    uint8_t outputs;
    uint8_t inputs;
    app_dout_status_t dout_status;
    app_din_status_t din_status;

    (void)argument;

    outputs = APP_USER_OUTPUT_IMAGE_INITIAL;

    for (;;)
    {
        /* Toggle the complete output image used for this basic application
         * test. Each bit represents one digital output channel.
         */
        outputs ^= APP_USER_OUTPUT_IMAGE_TOGGLE_MASK;

        /* Enqueue the new output image using the digital-output application
         * API. The output task is the only module that accesses the
         * VNI8200XP-32 driver directly.
         */
        dout_status = app_dout_set_outputs(outputs);

        if (dout_status != APP_DOUT_OK)
        {
            printf("[USER] app_dout_set_outputs() failed: %d\r\n",
                   (int)dout_status);
        }

        /* Read the latest valid digital-input image using the digital-input
         * application API. The input task updates this value periodically and
         * protects it internally with a mutex.
         */
        din_status = app_din_read(&inputs);

        if (din_status == APP_DIN_OK)
        {
            app_user_print_inputs(inputs);
        }
        else if (din_status == APP_DIN_E_NOT_READY)
        {
            printf("[USER] DIN data not ready\r\n");
        }
        else if (din_status == APP_DIN_E_BUSY)
        {
            printf("[USER] DIN data busy\r\n");
        }
        else if (din_status != APP_DIN_E_NULL)
        {
            printf("[USER] app_din_read() failed: %d\r\n",
                   (int)din_status);
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
 * @brief Print the digital input image in binary format.
 *
 * @param inputs Digital input image.
 */
static void app_user_print_inputs(uint8_t inputs)
{
    printf("[DIN] Inputs: 0b%c%c%c%c%c%c%c%c\r\n",
           ((inputs & (1u << 7u)) != 0u) ? '1' : '0',
           ((inputs & (1u << 6u)) != 0u) ? '1' : '0',
           ((inputs & (1u << 5u)) != 0u) ? '1' : '0',
           ((inputs & (1u << 4u)) != 0u) ? '1' : '0',
           ((inputs & (1u << 3u)) != 0u) ? '1' : '0',
           ((inputs & (1u << 2u)) != 0u) ? '1' : '0',
           ((inputs & (1u << 1u)) != 0u) ? '1' : '0',
           ((inputs & (1u << 0u)) != 0u) ? '1' : '0');
}

/** @} */
