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
 * @file    app_rs485.c
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-08-04
 * @brief   RS485 single-byte echo application task implementation.
 *
 * @details
 * This module implements a minimal CMSIS-RTOS2 application task for validating
 * the RS485 interface through a single-byte echo test.
 *
 * The task is the only application-level owner of the RS485 driver and
 * serializes every operation performed on the half-duplex bus.
 *
 * External application code communicates with the task through two message
 * queues:
 *
 * - rs485InputQueueHandle stores bytes received through RS485.
 * - rs485OutputQueueHandle stores bytes requested for transmission.
 *
 * Reception path:
 *
 * - app_rs485_task() starts an interrupt-driven reception of one byte by
 *   calling rs485_receive_start().
 * - UART7 receives the byte through HAL_UART_Receive_IT().
 * - The lower callback chain invokes rs485_rx_complete_callback() from
 *   interrupt context.
 * - rs485_rx_complete_callback() sets APP_RS485_EVT_RX_READY.
 * - The task wakes and obtains the byte by calling rs485_receive().
 * - The received byte is placed into rs485InputQueueHandle.
 * - The same byte is transmitted back automatically, implementing the echo.
 *
 * UART error path:
 *
 * - A UART reception error invokes rs485_error_callback().
 * - The callback stores the HAL UART error flags and sets
 *   APP_RS485_EVT_RX_ERROR.
 * - The task consumes the pending driver error and restarts reception.
 *
 * Application transmission path:
 *
 * - app_rs485_send() places one byte into rs485OutputQueueHandle.
 * - APP_RS485_EVT_TX_READY wakes the task.
 * - The task aborts the active reception before switching the half-duplex
 *   transceiver to transmit mode.
 * - Each pending byte is transmitted using rs485_send().
 * - Interrupt-driven reception is restarted after transmission.
 *
 * The interrupt callbacks only record asynchronous information and set RTOS
 * event flags. Queue operations, printing, echo processing and transmission
 * are performed from task context.
 *
 * @ingroup app_rs485
 * @{
 */

/* ============================= Includes ================================== */

#include "app_rs485.h"

#include "rs485.h"
#include "swo.h"

#include "cmsis_os2.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/* ============================ Local Macros =============================== */

/**
 * @brief Blocking timeout used for one-byte transmission.
 */
#define APP_RS485_TX_TIMEOUT_MS             ((uint32_t)100u)

/**
 * @brief Delay before retrying driver initialization after a failure.
 */
#define APP_RS485_INIT_RETRY_DELAY_MS       ((uint32_t)1000u)

/* ========================== Private Prototypes =========================== */

static rs485_status_t app_rs485_start_reception(void);

static void app_rs485_process_rx(void);

static void app_rs485_process_rx_error(void);

static void app_rs485_process_tx_queue(void);

static uint32_t app_rs485_status_to_fault(
    rs485_status_t status);

static void app_rs485_report_fault(
    rs485_status_t status);

static uint32_t app_rs485_ms_to_ticks(
    uint32_t milliseconds);

/* ======================= External RTOS Objects =========================== */

extern osEventFlagsId_t rs485EventHandle;
extern osEventFlagsId_t rs485FaultHandle;
extern osMessageQueueId_t rs485InputQueueHandle;
extern osMessageQueueId_t rs485OutputQueueHandle;

/* ======================= Local Static Data ================================ */

static rs485_handle_t app_rs485_handle = {
    .reserved = 0u
};

/**
 * @brief Last UART error flags received from interrupt context.
 */
static volatile uint32_t app_rs485_last_uart_error = 0u;

/* ===================== Public Function Definitions ======================= */

void app_rs485_task(void * argument)
{
    rs485_status_t rs485_status;
    uint32_t flags;
    uint32_t retry_delay_ticks;
    bool initialized = false;

    (void)argument;

    retry_delay_ticks =
        app_rs485_ms_to_ticks(APP_RS485_INIT_RETRY_DELAY_MS);

    for (;;)
    {
        if (initialized == false)
        {
            printf("[APP_RS485] init start\r\n");

            rs485_status = rs485_init(&app_rs485_handle);

            if (rs485_status == RS485_OK)
            {
                rs485_status = app_rs485_start_reception();
            }

            if (rs485_status == RS485_OK)
            {
                initialized = true;

                printf("[APP_RS485] init complete\r\n");
            }
            else
            {
                (void)osEventFlagsSet(
                    rs485FaultHandle,
                    APP_RS485_FAULT_INIT_ERROR);

                app_rs485_report_fault(rs485_status);

                printf(
                    "[APP_RS485] init failed: %d\r\n",
                    (int)rs485_status);

                /*
                 * Ignore the deinitialization result because initialization
                 * may have failed before the driver reached its initialized
                 * state.
                 */
                (void)rs485_deinit(&app_rs485_handle);

                (void)osDelay(retry_delay_ticks);
            }
        }
        else
        {
            flags = osEventFlagsWait(
                rs485EventHandle,
                APP_RS485_EVT_MASK,
                osFlagsWaitAny,
                osWaitForever);

            if ((flags & osFlagsError) != 0u)
            {
                (void)osEventFlagsSet(
                    rs485FaultHandle,
                    APP_RS485_FAULT_OS_ERROR);

                continue;
            }

            /*
             * A UART error takes priority over a simultaneous reception
             * completion because the received byte cannot be considered valid.
             */
            if ((flags & APP_RS485_EVT_RX_ERROR) != 0u)
            {
                app_rs485_process_rx_error();
            }
            else if ((flags & APP_RS485_EVT_RX_READY) != 0u)
            {
                app_rs485_process_rx();
            }
            else
            {
                /* No reception event in this task wake-up. */
            }

            if ((flags & APP_RS485_EVT_TX_READY) != 0u)
            {
                app_rs485_process_tx_queue();
            }

            /*
             * RX processing, RX-error processing and TX processing leave the
             * driver without an active reception. Arm reception again after
             * all events from the current wake-up have been processed.
             */
            rs485_status = app_rs485_start_reception();

            if (rs485_status != RS485_OK)
            {
                app_rs485_report_fault(rs485_status);

                printf(
                    "[APP_RS485] RX restart failed: %d\r\n",
                    (int)rs485_status);
            }
        }
    }
}

app_rs485_status_t app_rs485_send(uint8_t data)
{
    osStatus_t os_status;
    uint32_t flags;

    os_status = osMessageQueuePut(
        rs485OutputQueueHandle,
        &data,
        0u,
        0u);

    if (os_status == osOK)
    {
        flags = osEventFlagsSet(
            rs485EventHandle,
            APP_RS485_EVT_TX_READY);

        if ((flags & osFlagsError) != 0u)
        {
            (void)osEventFlagsSet(
                rs485FaultHandle,
                APP_RS485_FAULT_OS_ERROR);

            return APP_RS485_E_OS;
        }

        return APP_RS485_OK;
    }

    if (os_status == osErrorResource)
    {
        (void)osEventFlagsSet(
            rs485FaultHandle,
            APP_RS485_FAULT_TX_QUEUE_FULL);

        return APP_RS485_E_QUEUE_FULL;
    }

    (void)osEventFlagsSet(
        rs485FaultHandle,
        APP_RS485_FAULT_OS_ERROR);

    return APP_RS485_E_OS;
}

app_rs485_status_t app_rs485_receive(uint8_t * p_data)
{
    osStatus_t os_status;

    if (p_data == NULL)
    {
        return APP_RS485_E_NULL;
    }

    os_status = osMessageQueueGet(
        rs485InputQueueHandle,
        p_data,
        NULL,
        0u);

    if (os_status == osOK)
    {
        return APP_RS485_OK;
    }

    if (os_status == osErrorResource)
    {
        return APP_RS485_E_QUEUE_EMPTY;
    }

    (void)osEventFlagsSet(
        rs485FaultHandle,
        APP_RS485_FAULT_OS_ERROR);

    return APP_RS485_E_OS;
}

/* ================= Driver Notification Definitions ======================= */

/**
 * @brief Handle completion of an interrupt-driven RS485 reception.
 *
 * @details
 * This function provides the application implementation of the callback
 * referenced by rs485.c.
 *
 * It executes in UART7 interrupt context and only sets an RTOS event flag.
 * Byte processing, queue access and echo transmission are deferred to
 * app_rs485_task().
 */
void rs485_rx_complete_callback(void)
{
    (void)osEventFlagsSet(
        rs485EventHandle,
        APP_RS485_EVT_RX_READY);
}

/**
 * @brief Handle an RS485 UART reception error.
 *
 * @param[in] error_code  STM32 HAL UART error flags.
 *
 * @details
 * This function provides the application implementation of the callback
 * referenced by rs485.c.
 *
 * It executes in UART7 interrupt context, stores the error flags and wakes
 * app_rs485_task() through APP_RS485_EVT_RX_ERROR.
 */
void rs485_error_callback(uint32_t error_code)
{
    app_rs485_last_uart_error = error_code;

    (void)osEventFlagsSet(
        rs485EventHandle,
        APP_RS485_EVT_RX_ERROR);
}

/* ===================== Private Function Definitions ====================== */

/**
 * @brief Start interrupt-driven reception of the next RS485 byte.
 *
 * @return RS485_OK on success, error code otherwise.
 */
static rs485_status_t app_rs485_start_reception(void)
{
    return rs485_receive_start();
}

/**
 * @brief Process one completed RS485 byte reception.
 *
 * @details
 * This function executes in task context after APP_RS485_EVT_RX_READY.
 *
 * The received byte is placed in the application RX queue and immediately
 * transmitted back through RS485, implementing a single-byte echo.
 */
static void app_rs485_process_rx(void)
{
    rs485_status_t rs485_status;
    osStatus_t queue_status;
    uint8_t data = 0u;

    rs485_status = rs485_receive(&data);

    if (rs485_status != RS485_OK)
    {
        app_rs485_report_fault(rs485_status);

        printf(
            "[APP_RS485] RX error: %d\r\n",
            (int)rs485_status);

        return;
    }

    queue_status = osMessageQueuePut(
        rs485InputQueueHandle,
        &data,
        0u,
        0u);

    if (queue_status == osErrorResource)
    {
        (void)osEventFlagsSet(
            rs485FaultHandle,
            APP_RS485_FAULT_RX_QUEUE_FULL);
    }
    else if (queue_status != osOK)
    {
        (void)osEventFlagsSet(
            rs485FaultHandle,
            APP_RS485_FAULT_OS_ERROR);
    }
    else
    {
        /* Received byte queued successfully. */
    }

    rs485_status = rs485_send(
        data,
        APP_RS485_TX_TIMEOUT_MS);

    if (rs485_status != RS485_OK)
    {
        app_rs485_report_fault(rs485_status);

        printf(
            "[APP_RS485] echo failed: %d\r\n",
            (int)rs485_status);

        return;
    }

    printf(
        "[APP_RS485] echo: 0x%02X '%c'\r\n",
        data,
        ((data >= 0x20u) && (data <= 0x7Eu)) ?
            (char)data :
            '.');
}

/**
 * @brief Process one deferred UART reception error.
 *
 * @details
 * rs485_receive() consumes the pending error stored by the driver and should
 * return RS485_E_HW after the UART error callback.
 *
 * The next reception is started by app_rs485_task() after this function
 * returns.
 */
static void app_rs485_process_rx_error(void)
{
    rs485_status_t rs485_status;
    uint8_t unused_data = 0u;

    rs485_status = rs485_receive(&unused_data);

    if (rs485_status == RS485_E_HW)
    {
        app_rs485_report_fault(RS485_E_HW);
    }
    else
    {
        /*
         * Report the actual result if the driver did not expose the expected
         * pending hardware-error state.
         */
        app_rs485_report_fault(rs485_status);
    }

    printf(
        "[APP_RS485] UART error: 0x%08lX\r\n",
        (unsigned long)app_rs485_last_uart_error);

    app_rs485_last_uart_error = 0u;
}

/**
 * @brief Transmit all bytes pending in the TX application queue.
 *
 * @details
 * The active interrupt-driven reception is aborted before transmission because
 * the transceiver cannot remain in receive mode while driving the half-duplex
 * bus.
 *
 * Each queued byte is transmitted without framing or modification. Reception
 * is restarted by app_rs485_task() after this function returns.
 */
static void app_rs485_process_tx_queue(void)
{
    osStatus_t queue_status;
    rs485_status_t rs485_status;
    uint8_t data = 0u;

    rs485_status = rs485_receive_abort();

    if (rs485_status != RS485_OK)
    {
        app_rs485_report_fault(rs485_status);

        printf(
            "[APP_RS485] RX abort failed: %d\r\n",
            (int)rs485_status);

        return;
    }

    for (;;)
    {
        queue_status = osMessageQueueGet(
            rs485OutputQueueHandle,
            &data,
            NULL,
            0u);

        if (queue_status == osErrorResource)
        {
            /* TX queue empty. */
            break;
        }

        if (queue_status != osOK)
        {
            (void)osEventFlagsSet(
                rs485FaultHandle,
                APP_RS485_FAULT_OS_ERROR);

            break;
        }

        rs485_status = rs485_send(
            data,
            APP_RS485_TX_TIMEOUT_MS);

        if (rs485_status != RS485_OK)
        {
            app_rs485_report_fault(rs485_status);

            printf(
                "[APP_RS485] TX failed: %d\r\n",
                (int)rs485_status);

            break;
        }

        printf(
            "[APP_RS485] TX: 0x%02X '%c'\r\n",
            data,
            ((data >= 0x20u) && (data <= 0x7Eu)) ?
                (char)data :
                '.');
    }
}

/**
 * @brief Convert an RS485 driver status into an application fault flag.
 *
 * @param[in] status  Status returned by the RS485 driver.
 *
 * @return Corresponding APP_RS485_FAULT_* flag, or zero when no fault exists.
 */
static uint32_t app_rs485_status_to_fault(
    rs485_status_t status)
{
    uint32_t fault;

    switch (status)
    {
        case RS485_OK:
            fault = 0u;
            break;

        case RS485_E_NULL:
            fault = APP_RS485_FAULT_NULL_ERROR;
            break;

        case RS485_E_PARAM:
            fault = APP_RS485_FAULT_PARAM_ERROR;
            break;

        case RS485_E_STATE:
            fault = APP_RS485_FAULT_STATE_ERROR;
            break;

        case RS485_E_TIMEOUT:
            fault = APP_RS485_FAULT_TIMEOUT;
            break;

        case RS485_E_HW:
        default:
            fault = APP_RS485_FAULT_HW_ERROR;
            break;
    }

    return fault;
}

/**
 * @brief Report one RS485 driver status through application fault flags.
 *
 * @param[in] status  Status returned by the RS485 driver.
 */
static void app_rs485_report_fault(
    rs485_status_t status)
{
    uint32_t fault_flags;

    fault_flags = app_rs485_status_to_fault(status);

    if (fault_flags != 0u)
    {
        (void)osEventFlagsSet(
            rs485FaultHandle,
            fault_flags);
    }
}

/**
 * @brief Convert milliseconds to CMSIS-RTOS2 kernel ticks.
 *
 * @param[in] milliseconds  Time interval in milliseconds.
 *
 * @return Equivalent kernel tick count, rounded up to at least one tick for a
 *         nonzero input value.
 */
static uint32_t app_rs485_ms_to_ticks(
    uint32_t milliseconds)
{
    uint32_t tick_frequency;
    uint64_t ticks;

    if (milliseconds == 0u)
    {
        return 0u;
    }

    tick_frequency = osKernelGetTickFreq();

    if (tick_frequency == 0u)
    {
        return milliseconds;
    }

    ticks =
        ((uint64_t)milliseconds *
         (uint64_t)tick_frequency +
         999u) /
        1000u;

    if (ticks == 0u)
    {
        ticks = 1u;
    }

    if (ticks > UINT32_MAX)
    {
        ticks = UINT32_MAX;
    }

    return (uint32_t)ticks;
}

/** @} */
