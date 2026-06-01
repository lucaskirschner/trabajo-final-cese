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
 * @file    app_diagnostics.c
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-05-27
 * @brief   Diagnostics application task implementation.
 *
 * @details
 * This module implements the diagnostics application task.
 *
 * The task monitors independent fault event objects for the digital input,
 * digital output and radio application layers and reports each detected
 * condition through printf().
 *
 * @ingroup app_diagnostics
 * @{
 */

/* ============================= Includes ================================== */

#include "app_diagnostics.h"
#include "app_dout.h"
#include "app_din.h"
#include "app_radio.h"

#include "cmsis_os2.h"

#include <stdint.h>
#include <stdio.h>

/* ======================= External RTOS Objects ============================ */

extern osEventFlagsId_t doutFaultEventHandle;
extern osEventFlagsId_t dinFaultEventHandle;
extern osEventFlagsId_t radioFaultEventHandle;

/* ========================== Private Prototypes =========================== */

static void app_diagnostics_print_dout_faults(uint32_t flags);
static void app_diagnostics_print_din_faults(uint32_t flags);
static void app_diagnostics_print_radio_faults(uint32_t flags);

/* ===================== Public Function Definitions ======================= */

void app_diagnostics_task(void * argument)
{
    uint32_t flags;

    (void)argument;

    for (;;)
    {
        flags = osEventFlagsWait(doutFaultEventHandle,
                                 APP_DOUT_FAULT_ERROR_MASK,
                                 osFlagsWaitAny,
                                 100U);

        if ((flags & osFlagsError) == 0U)
        {
            app_diagnostics_print_dout_faults(flags);
        }

        flags = osEventFlagsWait(dinFaultEventHandle,
                                 APP_DIN_FAULT_ERROR_MASK,
                                 osFlagsWaitAny,
                                 100U);

        if ((flags & osFlagsError) == 0U)
        {
            app_diagnostics_print_din_faults(flags);
        }

        flags = osEventFlagsWait(radioFaultEventHandle,
                                 APP_RADIO_FAULT_ERROR_MASK,
                                 osFlagsWaitAny,
                                 100U);

        if ((flags & osFlagsError) == 0U)
        {
            app_diagnostics_print_radio_faults(flags);
        }
    }
}

/* ===================== Private Function Definitions ====================== */

/**
 * @brief Print digital-output fault events.
 *
 * @param flags Event flags returned by the digital-output fault event object.
 */
static void app_diagnostics_print_dout_faults(uint32_t flags)
{
    if ((flags & APP_DOUT_FAULT_RX_PARITY_ERROR) != 0u)
    {
        printf("[DOUT] RX parity error\r\n");
    }

    if ((flags & APP_DOUT_FAULT_SPI_ERROR) != 0u)
    {
        printf("[DOUT] SPI communication error\r\n");
    }

    if ((flags & APP_DOUT_FAULT_FB_ERROR) != 0u)
    {
        printf("[DOUT] DC-DC feedback error\r\n");
    }

    if ((flags & APP_DOUT_FAULT_TWARN) != 0u)
    {
        printf("[DOUT] Temperature warning\r\n");
    }

    if ((flags & APP_DOUT_FAULT_PG_ERROR) != 0u)
    {
        printf("[DOUT] Power Good error\r\n");
    }

    if ((flags & APP_DOUT_FAULT_OVT_ERROR) != 0u)
    {
        printf("[DOUT] Channel overtemperature error\r\n");
    }

    if ((flags & APP_DOUT_FAULT_HW_ERROR) != 0u)
    {
        printf("[DOUT] Hardware error\r\n");
    }

    if ((flags & APP_DOUT_FAULT_TIMEOUT) != 0u)
    {
        printf("[DOUT] Timeout error\r\n");
    }

    if ((flags & APP_DOUT_FAULT_PARAM_ERROR) != 0u)
    {
        printf("[DOUT] Parameter error\r\n");
    }

    if ((flags & APP_DOUT_FAULT_NULL_ERROR) != 0u)
    {
        printf("[DOUT] Null pointer error\r\n");
    }

    if ((flags & APP_DOUT_FAULT_QUEUE_FULL) != 0u)
    {
        printf("[DOUT] Output queue full\r\n");
    }

    if ((flags & APP_DOUT_FAULT_OS_ERROR) != 0u)
    {
        printf("[DOUT] RTOS error\r\n");
    }
}

/**
 * @brief Print digital-input fault events.
 *
 * @param flags Event flags returned by the digital-input fault event object.
 */
static void app_diagnostics_print_din_faults(uint32_t flags)
{
    if ((flags & APP_DIN_FAULT_STOP_BITS_ERROR) != 0u)
    {
        printf("[DIN] Stop bits error\r\n");
    }

    if ((flags & APP_DIN_FAULT_PARITY_ERROR) != 0u)
    {
        printf("[DIN] Parity error\r\n");
    }

    if ((flags & APP_DIN_FAULT_UV_ERROR) != 0u)
    {
        printf("[DIN] Undervoltage alarm\r\n");
    }

    if ((flags & APP_DIN_FAULT_OT_ERROR) != 0u)
    {
        printf("[DIN] Overtemperature alarm\r\n");
    }

    if ((flags & APP_DIN_FAULT_POWER_LOSS) != 0u)
    {
        printf("[DIN] Power loss detected\r\n");
    }

    if ((flags & APP_DIN_FAULT_HW_ERROR) != 0u)
    {
        printf("[DIN] Hardware error\r\n");
    }

    if ((flags & APP_DIN_FAULT_TIMEOUT) != 0u)
    {
        printf("[DIN] Timeout error\r\n");
    }

    if ((flags & APP_DIN_FAULT_PARAM_ERROR) != 0u)
    {
        printf("[DIN] Parameter error\r\n");
    }

    if ((flags & APP_DIN_FAULT_NULL_ERROR) != 0u)
    {
        printf("[DIN] Null pointer error\r\n");
    }

    if ((flags & APP_DIN_FAULT_OS_ERROR) != 0u)
    {
        printf("[DIN] RTOS error\r\n");
    }
}

/**
 * @brief Print radio fault events.
 *
 * @param flags Event flags returned by the radio fault event object.
 */
static void app_diagnostics_print_radio_faults(uint32_t flags)
{
    if ((flags & APP_RADIO_FAULT_HW_ERROR) != 0u)
    {
        printf("[RADIO] Hardware error\r\n");
    }

    if ((flags & APP_RADIO_FAULT_TIMEOUT) != 0u)
    {
        printf("[RADIO] Timeout error\r\n");
    }

    if ((flags & APP_RADIO_FAULT_PARAM_ERROR) != 0u)
    {
        printf("[RADIO] Parameter error\r\n");
    }

    if ((flags & APP_RADIO_FAULT_NULL_ERROR) != 0u)
    {
        printf("[RADIO] Null pointer error\r\n");
    }

    if ((flags & APP_RADIO_FAULT_FRAME_ERROR) != 0u)
    {
        printf("[RADIO] Frame error\r\n");
    }

    if ((flags & APP_RADIO_FAULT_STATE_ERROR) != 0u)
    {
        printf("[RADIO] State error\r\n");
    }

    if ((flags & APP_RADIO_FAULT_RX_QUEUE_FULL) != 0u)
    {
        printf("[RADIO] RX queue full\r\n");
    }

    if ((flags & APP_RADIO_FAULT_TX_QUEUE_FULL) != 0u)
    {
        printf("[RADIO] TX queue full\r\n");
    }

    if ((flags & APP_RADIO_FAULT_OS_ERROR) != 0u)
    {
        printf("[RADIO] RTOS error\r\n");
    }
}

/** @} */
