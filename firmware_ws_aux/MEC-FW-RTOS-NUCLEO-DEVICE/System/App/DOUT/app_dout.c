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
 * @file    app_dout.c
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-05-26
 * @brief   Digital output application task implementation.
 *
 * @details
 * This module implements the CMSIS-RTOS2 digital output application task.
 *
 * The task owns the VNI8200XP-32 driver. External application code should not
 * call the driver directly. Instead, application code calls
 * app_dout_set_outputs(), which enqueues a complete output image.
 *
 * Output path:
 * - app_dout_set_outputs() enqueues one uint8_t output image.
 * - app_dout_task() blocks on the output queue.
 * - app_dout_task() writes the received image to the VNI8200XP-32 driver.
 *
 * Immediate enqueue errors are returned by app_dout_set_outputs().
 * Deferred driver or RTOS faults are reported through doutFaultEventHandle.
 *
 * @ingroup app_dout
 * @{
 */

/* ============================= Includes ================================== */

#include "app_dout.h"
#include "vni8200xp32.h"

#include "main.h"

#include "cmsis_os2.h"

#include <stdint.h>

/* ========================== Private Prototypes =========================== */

static uint32_t app_dout_status_to_fault(vni8200xp32_status_t status);

/* ======================= External RTOS Objects ============================ */

extern osMessageQueueId_t outOutputQueueHandle;
extern osEventFlagsId_t doutFaultHandle;

/* ===================== Public Function Definitions ======================= */

void app_dout_task(void * argument)
{
    uint8_t output_image;
    osStatus_t queue_status;
    vni8200xp32_status_t vni_status;
    uint32_t fault_flags;

    (void)argument;

    HAL_GPIO_WritePin(OUT_EN_GPIO_Port, OUT_EN_Pin, GPIO_PIN_SET);

    for (;;)
    {
        queue_status = osMessageQueueGet(outOutputQueueHandle,
                                         &output_image,
                                         NULL,
                                         osWaitForever);

        if (queue_status == osOK)
        {
            vni_status = vni8200xp32_write_outputs(output_image);

            if (vni_status != VNI8200XP32_OK)
            {
                fault_flags = app_dout_status_to_fault(vni_status);

                (void)osEventFlagsSet(doutFaultHandle, fault_flags);
            }
        }
        else
        {
            (void)osEventFlagsSet(doutFaultHandle,
                                  APP_DOUT_FAULT_OS_ERROR);
        }
    }
}

app_dout_status_t app_dout_set_outputs(uint8_t output_image)
{
    osStatus_t os_status;
    app_dout_status_t status;

    os_status = osMessageQueuePut(outOutputQueueHandle,
                                  &output_image,
                                  0u,
                                  0u);

    if (os_status == osOK)
    {
        status = APP_DOUT_OK;
    }
    else if (os_status == osErrorResource)
    {
        (void)osEventFlagsSet(doutFaultHandle,
                              APP_DOUT_FAULT_QUEUE_FULL);

        status = APP_DOUT_E_QUEUE_FULL;
    }
    else
    {
        (void)osEventFlagsSet(doutFaultHandle,
                              APP_DOUT_FAULT_OS_ERROR);

        status = APP_DOUT_E_OS;
    }

    return status;
}

/* ===================== Private Function Definitions ====================== */

/**
 * @brief Convert a VNI8200XP-32 driver status code to an application fault.
 *
 * @param status Status code returned by the VNI8200XP-32 driver.
 *
 * @return Application-level digital output fault flag.
 */
static uint32_t app_dout_status_to_fault(vni8200xp32_status_t status)
{
    uint32_t fault;

    switch (status)
    {
        case VNI8200XP32_E_RX_PARITY:
            fault = APP_DOUT_FAULT_RX_PARITY_ERROR;
            break;

        case VNI8200XP32_E_SPI:
            fault = APP_DOUT_FAULT_SPI_ERROR;
            break;

        case VNI8200XP32_E_FB:
            fault = APP_DOUT_FAULT_FB_ERROR;
            break;

        case VNI8200XP32_E_TWARN:
            fault = APP_DOUT_FAULT_TWARN;
            break;

        case VNI8200XP32_E_PG:
            fault = APP_DOUT_FAULT_PG_ERROR;
            break;

        case VNI8200XP32_E_OVT:
            fault = APP_DOUT_FAULT_OVT_ERROR;
            break;

        case VNI8200XP32_E_TIMEOUT:
            fault = APP_DOUT_FAULT_TIMEOUT;
            break;

        case VNI8200XP32_E_PARAM:
            fault = APP_DOUT_FAULT_PARAM_ERROR;
            break;

        case VNI8200XP32_E_NULL:
            fault = APP_DOUT_FAULT_NULL_ERROR;
            break;

        case VNI8200XP32_E_HW:
        default:
            fault = APP_DOUT_FAULT_HW_ERROR;
            break;
    }

    return fault;
}

/** @} */
