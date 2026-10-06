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
 * @file    app_din.c
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-05-27
 * @brief   Digital input application task implementation.
 *
 * @details
 * This module implements the CMSIS-RTOS2 digital input application task.
 *
 * The task owns the SCLT3-8BT8 digital input driver. External application code
 * should not call the driver directly. Instead, application code calls
 * app_din_read() to obtain the last valid input image.
 *
 * Input path:
 * - app_din_task() periodically reads the SCLT3-8BT8 driver.
 * - On success, the last valid input image is updated under mutex protection.
 * - app_din_read() copies the last valid image using the same mutex.
 *
 * Immediate API errors are returned by app_din_read().
 * Deferred driver or RTOS faults are reported through dinFaultEventHandle.
 *
 * @ingroup app_din
 * @{
 */

/* ============================= Includes ================================== */

#include "app_din.h"
#include "sclt38bt8.h"

#include "main.h"

#include "cmsis_os2.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "dwt.h"

/* ============================= Defines =================================== */

/**
 * @brief Enable digital-input execution-time measurement.
 */
#define APP_DIN_TEST_TIMING

/**
 * @brief Number of digital-input operations measured during the test.
 */
#define APP_DIN_TEST_SAMPLES    ((uint32_t)1000u)

/* ========================== Private Prototypes =========================== */

static void app_din_update_last_image(uint8_t input_image);
static uint32_t app_din_status_to_fault(sclt38bt8_status_t status);

#if defined(APP_DIN_TEST_TIMING)

static void app_din_print_timing_results(uint32_t sample_count,
                                         uint32_t valid_count,
                                         uint32_t error_count,
                                         uint32_t min_cycles,
                                         uint32_t average_cycles,
                                         uint32_t max_cycles);

#endif

/* ======================= External RTOS Objects ============================ */

extern osMutexId_t dinDataMutexHandle;
extern osEventFlagsId_t dinFaultHandle;

/* ======================= Local Static Data ================================ */

static uint8_t app_din_last_input_image = 0u;
static bool app_din_data_valid = false;

/* ===================== Public Function Definitions ======================= */

void app_din_task(void * argument)
{
    uint8_t input_image;
    uint32_t fault_flags;
    sclt38bt8_status_t din_status;

#if defined(APP_DIN_TEST_TIMING)

    uint32_t start_cycles;
    uint32_t end_cycles;
    uint32_t elapsed_cycles;

    uint32_t sample_count = 0u;
    uint32_t valid_count = 0u;
    uint32_t error_count = 0u;

    uint32_t min_cycles = UINT32_MAX;
    uint32_t max_cycles = 0u;
    uint32_t average_cycles = 0u;

    uint64_t total_cycles = 0u;

#endif

    (void)argument;

#if defined(APP_DIN_TEST_TIMING)

    cycle_counter_init();

#endif

    for (;;)
    {

#if defined(APP_DIN_TEST_TIMING)

        /*
         * Measure only the effective digital-input acquisition operation.
         *
         * Mutex handling, data publication and task delay are intentionally
         * excluded from the measurement.
         */
        start_cycles = cycle_counter_get();

#endif

        din_status = sclt38bt8_read_inputs(&input_image);

#if defined(APP_DIN_TEST_TIMING)

        end_cycles = cycle_counter_get();

        /*
         * Acquire only the configured number of timing samples.
         */
        if (sample_count < APP_DIN_TEST_SAMPLES)
        {
            /*
             * Unsigned subtraction correctly handles one possible DWT
             * counter wraparound.
             */
            elapsed_cycles = end_cycles - start_cycles;

            if (din_status == SCLT38BT8_OK)
            {
                total_cycles += (uint64_t)elapsed_cycles;

                if (elapsed_cycles < min_cycles)
                {
                    min_cycles = elapsed_cycles;
                }

                if (elapsed_cycles > max_cycles)
                {
                    max_cycles = elapsed_cycles;
                }

                valid_count++;
            }
            else
            {
                error_count++;
            }

            sample_count++;

            /*
             * Print the results once all requested samples have been
             * processed.
             */
            if (sample_count == APP_DIN_TEST_SAMPLES)
            {
                if (valid_count > 0u)
                {
                    average_cycles =
                        (uint32_t)(total_cycles /
                                   (uint64_t)valid_count);

                    app_din_print_timing_results(sample_count,
                                                 valid_count,
                                                 error_count,
                                                 min_cycles,
                                                 average_cycles,
                                                 max_cycles);
                }
                else
                {
                    printf("\r\n");
                    printf("[APP_DIN] Timing test\r\n");

                    printf("[APP_DIN] Samples : %lu\r\n",
                           (unsigned long)sample_count);

                    printf("[APP_DIN] Valid   : 0\r\n");

                    printf("[APP_DIN] Errors  : %lu\r\n",
                           (unsigned long)error_count);

                    printf("[APP_DIN] No valid timing samples\r\n");
                    printf("[APP_DIN] Timing test completed\r\n");
                    printf("\r\n");
                }
            }
        }

#endif

        if (din_status == SCLT38BT8_OK)
        {
            app_din_update_last_image(input_image);
        }
        else
        {
            fault_flags = app_din_status_to_fault(din_status);

            if (fault_flags != 0u)
            {
                (void)osEventFlagsSet(dinFaultHandle, fault_flags);
            }
        }

        (void)osDelay(APP_DIN_POLL_PERIOD_TICKS);
    }
}

app_din_status_t app_din_read(uint8_t * p_input_image)
{
    osStatus_t mutex_status;
    app_din_status_t status;

    if (p_input_image == NULL)
    {
        status = APP_DIN_E_NULL;
    }
    else
    {
        mutex_status = osMutexAcquire(dinDataMutexHandle, 0u);

        if (mutex_status == osOK)
        {
            if (app_din_data_valid == true)
            {
                *p_input_image = app_din_last_input_image;
                status = APP_DIN_OK;
            }
            else
            {
                status = APP_DIN_E_NOT_READY;
            }

            (void)osMutexRelease(dinDataMutexHandle);
        }
        else if ((mutex_status == osErrorResource) ||
                 (mutex_status == osErrorTimeout))
        {
            status = APP_DIN_E_BUSY;
        }
        else
        {
            (void)osEventFlagsSet(dinFaultHandle,
                                  APP_DIN_FAULT_OS_ERROR);

            status = APP_DIN_E_OS;
        }
    }

    return status;
}

/* ===================== Private Function Definitions ====================== */

/**
 * @brief Update the last valid input image.
 *
 * @param input_image Input image returned by the SCLT3-8BT8 driver.
 *
 * @details
 * This function is called only by app_din_task().
 *
 * The update is protected by dinDataMutexHandle so that app_din_read() can
 * safely copy the same value from another task.
 */
static void app_din_update_last_image(uint8_t input_image)
{
    osStatus_t mutex_status;

    mutex_status = osMutexAcquire(dinDataMutexHandle, osWaitForever);

    if (mutex_status == osOK)
    {
        app_din_last_input_image = input_image;
        app_din_data_valid = true;

        (void)osMutexRelease(dinDataMutexHandle);
    }
    else
    {
        (void)osEventFlagsSet(dinFaultHandle,
                              APP_DIN_FAULT_OS_ERROR);
    }
}

/**
 * @brief Convert a SCLT3-8BT8 driver status code to an application fault flag.
 *
 * @param status Status code returned by the SCLT3-8BT8 driver.
 *
 * @return Application-level digital input fault flag.
 */
static uint32_t app_din_status_to_fault(sclt38bt8_status_t status)
{
    uint32_t fault;

    switch (status)
    {
        case SCLT38BT8_E_STOP_BITS:
            fault = APP_DIN_FAULT_STOP_BITS_ERROR;
            break;

        case SCLT38BT8_E_PARITY:
            fault = APP_DIN_FAULT_PARITY_ERROR;
            break;

        case SCLT38BT8_E_UV:
            fault = APP_DIN_FAULT_UV_ERROR;
            break;

        case SCLT38BT8_E_OT:
            fault = APP_DIN_FAULT_OT_ERROR;
            break;

        case SCLT38BT8_E_POWER_LOSS:
            fault = APP_DIN_FAULT_POWER_LOSS;
            break;

        case SCLT38BT8_E_TIMEOUT:
            fault = APP_DIN_FAULT_TIMEOUT;
            break;

        case SCLT38BT8_E_PARAM:
            fault = APP_DIN_FAULT_PARAM_ERROR;
            break;

        case SCLT38BT8_E_NULL:
            fault = APP_DIN_FAULT_NULL_ERROR;
            break;

        case SCLT38BT8_E_HW:
        default:
            fault = APP_DIN_FAULT_HW_ERROR;
            break;
    }

    return fault;
}

#if defined(APP_DIN_TEST_TIMING)

/**
 * @brief Print the digital-input timing-test results.
 *
 * @param sample_count   Total number of processed samples.
 * @param valid_count    Number of successful input acquisitions.
 * @param error_count    Number of input acquisitions that returned an error.
 * @param min_cycles     Minimum execution time in CPU cycles.
 * @param average_cycles Average execution time in CPU cycles.
 * @param max_cycles     Maximum execution time in CPU cycles.
 */
static void app_din_print_timing_results(uint32_t sample_count,
                                         uint32_t valid_count,
                                         uint32_t error_count,
                                         uint32_t min_cycles,
                                         uint32_t average_cycles,
                                         uint32_t max_cycles)
{
    printf("\r\n");
    printf("[APP_DIN] Timing test\r\n");

    printf("[APP_DIN] Samples : %lu\r\n",
           (unsigned long)sample_count);

    printf("[APP_DIN] Valid   : %lu\r\n",
           (unsigned long)valid_count);

    printf("[APP_DIN] Errors  : %lu\r\n",
           (unsigned long)error_count);

    printf("[APP_DIN] CPU     : %lu Hz\r\n",
           (unsigned long)SystemCoreClock);

    printf("[APP_DIN] Min cycles : %lu\r\n",
           (unsigned long)min_cycles);

    printf("[APP_DIN] Avg cycles : %lu\r\n",
           (unsigned long)average_cycles);

    printf("[APP_DIN] Max cycles : %lu\r\n",
           (unsigned long)max_cycles);

    printf("[APP_DIN] Timing test completed\r\n");
    printf("\r\n");
}

#endif

/** @} */
