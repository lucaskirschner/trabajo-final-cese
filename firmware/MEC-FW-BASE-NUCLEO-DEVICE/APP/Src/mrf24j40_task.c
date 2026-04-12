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
 * @file    mrf24j40_task.c
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-04-06
 * @brief   Cooperative task implementation for the MRF24J40 driver.
 *
 * @details
 * This module implements a simple state machine that services the low-level
 * MRF24J40 driver from thread context.
 *
 * Concurrency model:
 * - The low-level driver handles SPI accesses in blocking mode.
 * - The external interrupt callback is expected to notify the low-level driver
 *   separately through mrf24j40_set_interrupt_pending().
 * - This task periodically calls mrf24j40_update_interrupt_flags() to decode
 *   pending interrupt sources.
 * - RX packet retrieval and TX request execution are serialized by the task
 *   state machine.
 *
 * This module is intended for cooperative scheduling and does not create any
 * RTOS task by itself.
 *
 * @ingroup mrf24j40
 * @{
 */

/* ============================= Includes ================================== */

#include "mrf24j40_task.h"

#include <string.h>

/* ============================== Types ==================================== */

/**
 * @brief Internal execution states of the MRF24J40 cooperative task.
 *
 * @details
 * These states control initialization, idle servicing and the transmission
 * sequence driven by the task state machine.
 */
typedef enum
{
    MRF24J40_TASK_STATE_UNINITIALIZED = 0,
    MRF24J40_TASK_STATE_INIT,
    MRF24J40_TASK_STATE_IDLE,
    MRF24J40_TASK_STATE_TX_START,
    MRF24J40_TASK_STATE_TX_WAIT
} mrf24j40_task_state_t;

/**
 * @brief Internal runtime context of the MRF24J40 cooperative task.
 *
 * @details
 * This structure stores:
 * - current task state
 * - static task configuration
 * - initialization and readiness flags
 * - TX request and TX result flags
 * - one buffered RX packet
 * - one buffered TX packet
 *
 * Only one TX request and one RX packet are buffered at a time in this minimal
 * implementation.
 */
typedef struct
{
    mrf24j40_task_state_t state;
    mrf24j40_task_config_t config;

    bool initialized;
    bool ready;

    bool tx_pending;
    bool tx_ack_request;
    bool tx_done;
    bool tx_success;

    bool rx_available;

    mrf24j40_packet_t tx_packet;
    mrf24j40_packet_t rx_packet;
} mrf24j40_task_context_t;

/* ======================= Local (static) Data ============================= */

static mrf24j40_task_context_t mrf24j40_task_ctx;

/* ========================== Private Prototypes =========================== */

static void mrf24j40_task_clear_context(void);
static void mrf24j40_task_process_rx(void);
static void mrf24j40_task_process_tx_done(void);

/* ===================== Public Function Definitions ======================= */

void mrf24j40_task_init(const mrf24j40_task_config_t * p_config)
{
    if (p_config == NULL)
    {
        return;
    }

    mrf24j40_task_clear_context();
    mrf24j40_task_ctx.config = *p_config;
    mrf24j40_task_ctx.state = MRF24J40_TASK_STATE_INIT;
    mrf24j40_task_ctx.initialized = true;
}

void mrf24j40_task_run(void)
{
    if (mrf24j40_task_ctx.initialized == false)
    {
        return;
    }

    /* First decode any interrupt event previously latched by the low-level
     * driver.
     */
    mrf24j40_update_interrupt_flags();

    /* Process RX as early as possible so the received packet can be copied from
     * the low-level driver into the local task buffer.
     */
    mrf24j40_task_process_rx();

    /* Process TX completion bookkeeping. Reserved for future extensions. */
    mrf24j40_task_process_tx_done();

    switch (mrf24j40_task_ctx.state)
    {
        case MRF24J40_TASK_STATE_UNINITIALIZED:
        default:
            /* No action required. */
            break;

        case MRF24J40_TASK_STATE_INIT:
            mrf24j40_init();

            if (mrf24j40_task_ctx.config.role == MRF24J40_ROLE_PAN_COORDINATOR)
            {
                mrf24j40_configure_nonbeacon_pan_coordinator();
            }
            else
            {
                mrf24j40_configure_nonbeacon_device();
            }

            mrf24j40_set_pan_id(mrf24j40_task_ctx.config.pan_id);
            mrf24j40_set_short_address(mrf24j40_task_ctx.config.short_address);
            mrf24j40_set_extended_address();

            mrf24j40_task_ctx.ready = true;
            mrf24j40_task_ctx.state = MRF24J40_TASK_STATE_IDLE;
            break;

        case MRF24J40_TASK_STATE_IDLE:
            if (mrf24j40_task_ctx.tx_pending == true)
            {
                mrf24j40_task_ctx.state = MRF24J40_TASK_STATE_TX_START;
            }
            break;

        case MRF24J40_TASK_STATE_TX_START:
            if (mrf24j40_write_tx_normal_fifo(&mrf24j40_task_ctx.tx_packet,
                                              mrf24j40_task_ctx.tx_ack_request) == true)
            {
                mrf24j40_task_ctx.state = MRF24J40_TASK_STATE_TX_WAIT;
            }
            else
            {
                mrf24j40_task_ctx.tx_pending = false;
                mrf24j40_task_ctx.tx_done = true;
                mrf24j40_task_ctx.tx_success = false;
                mrf24j40_task_ctx.state = MRF24J40_TASK_STATE_IDLE;
            }
            break;

        case MRF24J40_TASK_STATE_TX_WAIT:
            if (mrf24j40_get_tx_complete() == true)
            {
                mrf24j40_task_ctx.tx_pending = false;
                mrf24j40_task_ctx.tx_done = true;
                mrf24j40_task_ctx.tx_success = true;
                mrf24j40_task_ctx.state = MRF24J40_TASK_STATE_IDLE;
            }
            break;
    }
}

bool mrf24j40_task_is_ready(void)
{
    return mrf24j40_task_ctx.ready;
}

bool mrf24j40_task_has_rx_packet(void)
{
    return mrf24j40_task_ctx.rx_available;
}

bool mrf24j40_task_get_rx_packet(mrf24j40_packet_t * p_packet)
{
    if (p_packet == NULL)
    {
        return false;
    }

    if (mrf24j40_task_ctx.rx_available == false)
    {
        return false;
    }

    *p_packet = mrf24j40_task_ctx.rx_packet;
    mrf24j40_task_ctx.rx_available = false;

    return true;
}

mrf24j40_task_status_t mrf24j40_task_request_tx(const mrf24j40_packet_t * p_packet,
                                                bool ack_request)
{
    if (p_packet == NULL)
    {
        return MRF24J40_TASK_E_PARAM;
    }

    if (mrf24j40_task_ctx.ready == false)
    {
        return MRF24J40_TASK_E_BUSY;
    }

    if (mrf24j40_task_ctx.state != MRF24J40_TASK_STATE_IDLE)
    {
        return MRF24J40_TASK_E_BUSY;
    }

    mrf24j40_task_ctx.tx_packet = *p_packet;
    mrf24j40_task_ctx.tx_ack_request = ack_request;
    mrf24j40_task_ctx.tx_pending = true;
    mrf24j40_task_ctx.tx_done = false;
    mrf24j40_task_ctx.tx_success = false;

    return MRF24J40_TASK_OK;
}

bool mrf24j40_task_get_last_tx_done(void)
{
    bool tx_done;

    tx_done = mrf24j40_task_ctx.tx_done;
    mrf24j40_task_ctx.tx_done = false;

    return tx_done;
}

bool mrf24j40_task_get_last_tx_success(void)
{
    return mrf24j40_task_ctx.tx_success;
}

/* ===================== Private Function Definitions ====================== */

/**
 * @brief Reset the internal task context to its default state.
 *
 * @details
 * This helper clears the complete internal task context structure and restores
 * the internal state machine to the uninitialized state.
 */
static void mrf24j40_task_clear_context(void)
{
    (void)memset(&mrf24j40_task_ctx, 0, sizeof(mrf24j40_task_ctx));
    mrf24j40_task_ctx.state = MRF24J40_TASK_STATE_UNINITIALIZED;
}

/**
 * @brief Attempt to retrieve one received packet from the low-level driver.
 *
 * @details
 * If the local RX buffer is already occupied, this function does nothing.
 *
 * Otherwise, it requests one packet from the low-level driver and stores it in
 * the local task RX buffer if available.
 */
static void mrf24j40_task_process_rx(void)
{
    if (mrf24j40_task_ctx.rx_available == true)
    {
        return;
    }

    if (mrf24j40_read_rx_fifo(&mrf24j40_task_ctx.rx_packet) == true)
    {
        mrf24j40_task_ctx.rx_available = true;
    }
}

/**
 * @brief Process TX completion side effects.
 *
 * @details
 * This helper is reserved for future improvements of the task layer, such as:
 * - reading TXSTAT after TX completion
 * - reporting retry count
 * - detecting CCA failures
 * - refining the TX success criterion
 *
 * The current implementation does not perform any additional action here.
 */
static void mrf24j40_task_process_tx_done(void)
{
    /* Reserved for future extensions:
     * - read TXSTAT
     * - count retries
     * - detect CCAFAIL
     */
}

/** @} */
