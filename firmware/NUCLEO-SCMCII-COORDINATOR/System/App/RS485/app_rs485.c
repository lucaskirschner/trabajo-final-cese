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
 * @date    2026-08-08
 * @brief   RS485 asynchronous application service implementation.
 *
 * @details
 * This module implements a CMSIS-RTOS2 service task that owns and manages the
 * single-byte interrupt-driven RS485 driver.
 *
 * External application tasks do not access rs485.c directly. Communication
 * with the RS485 service is performed through two RTOS message queues:
 *
 * - rs485InputQueueHandle stores bytes received from the RS485 bus.
 * - rs485OutputQueueHandle stores bytes requested for transmission.
 *
 * The public functions app_rs485_send() and app_rs485_receive() access these
 * queues without blocking.
 *
 * The RS485 task serializes all operations on the half-duplex interface.
 *
 * While idle, one interrupt-driven reception is normally active.
 *
 * When a transmission request becomes pending:
 *
 * - The active reception is aborted.
 * - One byte is removed from the transmission queue.
 * - rs485_send() starts an interrupt-driven transmission.
 * - The task returns to the blocked state.
 * - rs485_tx_complete_callback() wakes the task after transmission completes.
 * - The next queued byte is transmitted if one is available.
 * - Otherwise, reception is armed again.
 *
 * Reception processing follows the same asynchronous approach:
 *
 * - rs485_receive_start() arms reception of one byte.
 * - rs485_rx_complete_callback() wakes the task.
 * - rs485_receive() obtains the completed byte.
 * - The byte is placed into rs485InputQueueHandle.
 * - Reception is armed again when no transmission is pending.
 *
 * Driver callbacks execute in UART interrupt context and only store minimal
 * asynchronous information and set RTOS event flags. Queue processing,
 * communication sequencing and diagnostic printing are performed from task
 * context.
 *
 * This module intentionally does not implement an echo test or any other
 * application-specific communication behavior. Such behavior belongs to upper
 * application tasks such as app_user_task().
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
 * @brief Delay before retrying RS485 initialization after a failure.
 */
#define APP_RS485_INIT_RETRY_DELAY_MS      ((uint32_t)1000u)

/* ============================ Local Types ================================ */

/**
 * @brief Internal RS485 application service context.
 */
typedef struct
{
    /**
     * @brief Indicates whether the RS485 service has been initialized.
     */
    volatile bool initialized;

    /**
     * @brief Indicates that a driver receive operation is currently active.
     */
    bool rx_active;

    /**
     * @brief Indicates that a driver transmit operation is currently active.
     */
    bool tx_active;

    /**
     * @brief Byte associated with the current asynchronous transmission.
     */
    uint8_t tx_data;

    /**
     * @brief Last UART error flags received from interrupt context.
     */
    volatile uint32_t last_uart_error;

} app_rs485_ctx_t;

/* ========================== Private Prototypes =========================== */

static rs485_status_t app_rs485_start_reception(void);

static void app_rs485_process_rx(void);

static void app_rs485_process_error(void);

static bool app_rs485_start_next_tx(void);

static uint32_t app_rs485_status_to_fault(
    rs485_status_t status);

static void app_rs485_report_fault(
    rs485_status_t status);

static uint32_t app_rs485_ms_to_ticks(
    uint32_t milliseconds);

/* ======================= External RTOS Objects =========================== */

/**
 * @brief Event flags used to wake the RS485 application task.
 */
extern osEventFlagsId_t rs485EventHandle;

/**
 * @brief Event flags used to report persistent RS485 application faults.
 */
extern osEventFlagsId_t rs485FaultHandle;

/**
 * @brief Queue containing bytes received through RS485.
 */
extern osMessageQueueId_t rs485InputQueueHandle;

/**
 * @brief Queue containing bytes requested for RS485 transmission.
 */
extern osMessageQueueId_t rs485OutputQueueHandle;

/* ======================= Local Static Data ================================ */

/**
 * @brief Handle used by the RS485 application service.
 */
static rs485_handle_t app_rs485_handle = {
    .reserved = 0u
};

/**
 * @brief Internal RS485 application service context.
 */
static app_rs485_ctx_t g_ctx = {
    .initialized = false,
    .rx_active = false,
    .tx_active = false,
    .tx_data = 0u,
    .last_uart_error = 0u
};

/* ===================== Public Function Definitions ======================= */

void app_rs485_task(
    void * argument)
{
    rs485_status_t rs485_status;
    uint32_t flags;
    uint32_t retry_delay_ticks;
    bool tx_started;

    (void)argument;

    retry_delay_ticks =
        app_rs485_ms_to_ticks(
            APP_RS485_INIT_RETRY_DELAY_MS);

    for (;;)
    {
        /*
         * Initialize the RS485 driver from task context.
         *
         * The task remains responsible for retrying initialization if the
         * hardware or driver is temporarily unavailable.
         */
        if (g_ctx.initialized == false)
        {
            printf(
                "[APP_RS485] init start\r\n");

            rs485_status = rs485_init(
                &app_rs485_handle);

            if (rs485_status == RS485_OK)
            {
                g_ctx.rx_active = false;
                g_ctx.tx_active = false;
                g_ctx.tx_data = 0u;
                g_ctx.last_uart_error = 0u;

                rs485_status =
                    app_rs485_start_reception();
            }

            if (rs485_status == RS485_OK)
            {
                g_ctx.initialized = true;

                printf("[APP_RS485] init complete\r\n");
            }
            else
            {
                (void)osEventFlagsSet(
                    rs485FaultHandle,
                    APP_RS485_FAULT_INIT_ERROR);

                app_rs485_report_fault(
                    rs485_status);

                printf(
                    "[APP_RS485] init failed: %d\r\n",
                    (int)rs485_status);

                /*
                 * Initialization may have failed before the RS485 driver
                 * reached its initialized state. The deinitialization result
                 * is therefore intentionally ignored.
                 */
                (void)rs485_deinit(
                    &app_rs485_handle);

                g_ctx.rx_active = false;
                g_ctx.tx_active = false;
                g_ctx.tx_data = 0u;
                g_ctx.last_uart_error = 0u;

                (void)osDelay(
                    retry_delay_ticks);

                continue;
            }
        }

        /*
         * Remain blocked while no asynchronous RS485 event is pending.
         */
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
         * UART errors take priority over normal transfer-completion events.
         *
         * A transfer involved in a UART error cannot be considered valid.
         */
        if ((flags & APP_RS485_EVT_ERROR) != 0u)
        {
            app_rs485_process_error();
        }
        else
        {
            if ((flags & APP_RS485_EVT_RX_READY) != 0u)
            {
                g_ctx.rx_active = false;

                app_rs485_process_rx();
            }

            if ((flags & APP_RS485_EVT_TX_COMPLETE) != 0u)
            {
                g_ctx.tx_active = false;

                printf(
                    "[APP_RS485] TX complete: "
                    "0x%02X '%c'\r\n",
                    g_ctx.tx_data,
                    ((g_ctx.tx_data >= 0x20u) &&
                     (g_ctx.tx_data <= 0x7Eu)) ?
                        (char)g_ctx.tx_data :
                        '.');
            }
        }

        /*
         * If no transmission is currently active, always inspect the output
         * queue.
         *
         * This is intentionally done regardless of whether TX_REQUEST was part
         * of the current event flags. The queue itself is the authoritative
         * storage for pending transmission requests.
         *
         * This also allows a sequence of queued bytes to be transmitted one at
         * a time after each TX-complete interrupt.
         */
        tx_started = false;

        if (g_ctx.tx_active == false)
        {
            tx_started =
                app_rs485_start_next_tx();
        }

        /*
         * When no transmission is active or pending, ensure that reception is
         * armed.
         */
        if ((g_ctx.tx_active == false) &&
            (tx_started == false) &&
            (g_ctx.rx_active == false))
        {
            rs485_status =
                app_rs485_start_reception();

            if (rs485_status != RS485_OK)
            {
                app_rs485_report_fault(
                    rs485_status);

                printf(
                    "[APP_RS485] RX start failed: %d\r\n",
                    (int)rs485_status);
            }
        }
    }
}

app_rs485_status_t app_rs485_send(
    uint8_t data)
{
    osStatus_t os_status;
    uint32_t flags;

    if (g_ctx.initialized == false)
    {
        return APP_RS485_E_STATE;
    }

    /*
     * Queue the transmission request without blocking the calling task.
     */
    os_status = osMessageQueuePut(
        rs485OutputQueueHandle,
        &data,
        0u,
        0u);

    if (os_status == osOK)
    {
        /*
         * Wake the RS485 service task.
         *
         * The queue remains the authoritative source of the pending byte.
         * The event flag only informs the task that new work is available.
         */
        flags = osEventFlagsSet(
            rs485EventHandle,
            APP_RS485_EVT_TX_REQUEST);

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

app_rs485_status_t app_rs485_receive(
    uint8_t * p_data)
{
    osStatus_t os_status;

    if (p_data == NULL)
    {
        return APP_RS485_E_NULL;
    }

    if (g_ctx.initialized == false)
    {
        return APP_RS485_E_STATE;
    }

    /*
     * Retrieve one received byte without blocking the calling task.
     */
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
 * declared by rs485.h.
 *
 * It executes in UART interrupt context and only wakes app_rs485_task().
 *
 * The received byte itself remains stored in the lower RS485 driver until the
 * task calls rs485_receive().
 */
void rs485_rx_complete_callback(void)
{
    (void)osEventFlagsSet(
        rs485EventHandle,
        APP_RS485_EVT_RX_READY);
}

/**
 * @brief Handle completion of an interrupt-driven RS485 transmission.
 *
 * @param[in] data  Byte whose physical transmission has completed.
 *
 * @details
 * This function provides the application implementation of the callback
 * declared by rs485.h.
 *
 * It executes in UART interrupt context.
 *
 * No further communication operation is started from this callback. The RS485
 * task is only notified that the current transmission has completed.
 */
void rs485_tx_complete_callback(
    uint8_t data)
{
    /*
     * Store the completed byte for task-context diagnostics.
     */
    g_ctx.tx_data = data;

    (void)osEventFlagsSet(
        rs485EventHandle,
        APP_RS485_EVT_TX_COMPLETE);
}

/**
 * @brief Handle an asynchronous RS485 UART error.
 *
 * @param[in] error_code  UART error flags reported by the RS485 driver.
 *
 * @details
 * This function provides the application implementation of the callback
 * declared by rs485.h.
 *
 * It executes in UART interrupt context, stores the error flags and wakes
 * app_rs485_task().
 */
void rs485_error_callback(
    uint32_t error_code)
{
    g_ctx.last_uart_error = error_code;

    (void)osEventFlagsSet(
        rs485EventHandle,
        APP_RS485_EVT_ERROR);
}

/* ===================== Private Function Definitions ====================== */

/**
 * @brief Start interrupt-driven reception of the next RS485 byte.
 *
 * @return RS485_OK on success, error code otherwise.
 *
 * @details
 * Reception is started only when no transmission or reception is currently
 * active according to the application service state.
 */
static rs485_status_t app_rs485_start_reception(void)
{
    rs485_status_t status;

    if ((g_ctx.rx_active != false) ||
        (g_ctx.tx_active != false))
    {
        return RS485_E_STATE;
    }

    status = rs485_receive_start();

    if (status == RS485_OK)
    {
        g_ctx.rx_active = true;
    }

    return status;
}

/**
 * @brief Process one completed RS485 byte reception.
 *
 * @details
 * This function executes exclusively from app_rs485_task() context.
 *
 * The completed byte is retrieved from the lower RS485 driver and placed into
 * the application receive queue without blocking.
 *
 * No application-specific processing is performed here.
 */
static void app_rs485_process_rx(void)
{
    rs485_status_t rs485_status;
    osStatus_t queue_status;
    uint8_t data = 0u;

    rs485_status = rs485_receive(
        &data);

    if (rs485_status != RS485_OK)
    {
        app_rs485_report_fault(
            rs485_status);

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
        /*
         * The input queue is full. The newly received byte cannot be retained
         * and is therefore discarded.
         */
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
        printf(
            "[APP_RS485] RX: 0x%02X '%c'\r\n",
            data,
            ((data >= 0x20u) &&
             (data <= 0x7Eu)) ?
                (char)data :
                '.');
    }
}

/**
 * @brief Process a deferred RS485 UART communication error.
 *
 * @details
 * This function executes from app_rs485_task() context.
 *
 * The lower RS485 driver has already terminated its active communication state
 * and restored the transceiver to receive mode before invoking the
 * application-level error callback.
 *
 * The application service therefore clears its local state and allows normal
 * scheduling to restart reception after this function returns.
 */
static void app_rs485_process_error(void)
{
    g_ctx.rx_active = false;
    g_ctx.tx_active = false;

    (void)osEventFlagsSet(
        rs485FaultHandle,
        APP_RS485_FAULT_HW_ERROR);

    printf(
        "[APP_RS485] UART error: 0x%08lX\r\n",
        (unsigned long)g_ctx.last_uart_error);

    g_ctx.last_uart_error = 0u;
}

/**
 * @brief Start transmission of the next byte pending in the output queue.
 *
 * @return true if a transmission was started successfully.
 * @return false if the queue is empty or transmission could not be started.
 *
 * @details
 * This function executes from app_rs485_task() context.
 *
 * Only one byte is removed from the output queue per invocation because
 * rs485_send() starts an asynchronous transmission.
 *
 * The next byte, if any, will be started only after
 * APP_RS485_EVT_TX_COMPLETE wakes the task again.
 *
 * If an interrupt-driven reception is active, it is aborted before starting
 * transmission because the interface operates in half-duplex mode.
 */
static bool app_rs485_start_next_tx(void)
{
    osStatus_t queue_status;
    rs485_status_t rs485_status;
    uint32_t pending_count;
    uint8_t data = 0u;

    if (g_ctx.tx_active != false)
    {
        return false;
    }

    /*
     * Check whether transmission work is actually pending before aborting an
     * active reception.
     */
    pending_count = osMessageQueueGetCount(
        rs485OutputQueueHandle);

    if (pending_count == 0u)
    {
        return false;
    }

    /*
     * The half-duplex transceiver cannot transmit while an interrupt-driven
     * reception remains active.
     */
    if (g_ctx.rx_active != false)
    {
        rs485_status = rs485_receive_abort();

        if (rs485_status != RS485_OK)
        {
            app_rs485_report_fault(
                rs485_status);

            printf(
                "[APP_RS485] RX abort failed: %d\r\n",
                (int)rs485_status);

            return false;
        }

        g_ctx.rx_active = false;
    }

    /*
     * Remove exactly one byte from the transmission queue.
     */
    queue_status = osMessageQueueGet(
        rs485OutputQueueHandle,
        &data,
        NULL,
        0u);

    if (queue_status == osErrorResource)
    {
        return false;
    }

    if (queue_status != osOK)
    {
        (void)osEventFlagsSet(
            rs485FaultHandle,
            APP_RS485_FAULT_OS_ERROR);

        return false;
    }

    g_ctx.tx_data = data;

    rs485_status = rs485_send(
        data);

    if (rs485_status != RS485_OK)
    {
        app_rs485_report_fault(
            rs485_status);

        printf(
            "[APP_RS485] TX start failed: %d\r\n",
            (int)rs485_status);

        return false;
    }

    /*
     * The driver accepted the asynchronous transmission. Physical completion
     * will be reported later through rs485_tx_complete_callback().
     */
    g_ctx.tx_active = true;

    printf(
        "[APP_RS485] TX start: 0x%02X '%c'\r\n",
        data,
        ((data >= 0x20u) &&
         (data <= 0x7Eu)) ?
            (char)data :
            '.');

    return true;
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

    fault_flags =
        app_rs485_status_to_fault(
            status);

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

    tick_frequency =
        osKernelGetTickFreq();

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
