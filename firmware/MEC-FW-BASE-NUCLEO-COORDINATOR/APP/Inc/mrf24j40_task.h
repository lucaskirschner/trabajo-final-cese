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
 * @file    mrf24j40_task.h
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-04-06
 * @brief   Cooperative task layer for the MRF24J40 driver.
 *
 * @details
 * This module implements a simple cooperative task that wraps the low-level
 * MRF24J40 driver and provides a higher-level execution flow suitable for
 * bare-metal main loops or periodic scheduler calls.
 *
 * The task is responsible for:
 * - one-time transceiver initialization
 * - role-dependent MAC configuration
 * - PAN ID and address assignment
 * - interrupt flag update handling
 * - RX packet retrieval from the driver
 * - queued TX request execution
 * - basic TX completion tracking
 *
 * This module does not build IEEE 802.15.4 frames by itself. It assumes that
 * the application provides already formatted MAC frame bytes in the packet
 * container expected by the low-level driver.
 *
 * @ingroup mrf24j40
 * @{
 */

#ifndef MRF24J40_TASK_H_
#define MRF24J40_TASK_H_

/* ============================= Includes ================================== */

#include <stdint.h>
#include <stdbool.h>

#include "mrf24j40.h"

/* ============================== Types ==================================== */

/**
 * @brief Logical network role assigned to the local MRF24J40 node.
 *
 * @details
 * This role selection determines which nonbeacon MAC configuration helper is
 * applied during task initialization.
 */
typedef enum
{
    MRF24J40_ROLE_DEVICE = 0,
    MRF24J40_ROLE_PAN_COORDINATOR
} mrf24j40_role_t;

/**
 * @brief Status codes returned by the task-layer TX request API.
 *
 * @details
 * These status codes report whether a transmission request was accepted by the
 * cooperative task state machine.
 */
typedef enum
{
    MRF24J40_TASK_OK = 0,
    MRF24J40_TASK_E_BUSY,
    MRF24J40_TASK_E_PARAM
} mrf24j40_task_status_t;

/**
 * @brief Static configuration used to initialize the MRF24J40 task.
 *
 * @details
 * This structure contains the minimum configuration required by the task to
 * initialize the low-level driver and assign the local node identity within a
 * nonbeacon-enabled IEEE 802.15.4 network.
 */
typedef struct
{
    uint16_t pan_id;
    uint16_t short_address;
    mrf24j40_role_t role;
} mrf24j40_task_config_t;

/* ===================== Public Function Prototypes ======================== */

/**
 * @brief Initialize the MRF24J40 task context.
 *
 * @param p_config Pointer to the static task configuration.
 *
 * @details
 * This function clears the internal task context, stores the supplied
 * configuration and places the internal state machine in its initialization
 * state.
 *
 * The actual transceiver initialization sequence is not executed immediately
 * here. It is executed later from mrf24j40_task_run() when the task is first
 * serviced.
 *
 * @note
 * If @p p_config is NULL, the function returns without modifying the internal
 * context.
 */
void mrf24j40_task_init(const mrf24j40_task_config_t * p_config);

/**
 * @brief Execute one iteration of the cooperative MRF24J40 task.
 *
 * @details
 * This function advances the internal state machine by one step and should be
 * called periodically from the main loop or from a scheduler.
 *
 * The execution flow includes:
 * - pending interrupt flag update
 * - RX processing
 * - TX completion processing
 * - initialization state handling
 * - queued TX execution
 *
 * @note
 * This function is non-blocking at the task-state level, but it relies on the
 * underlying low-level driver, which performs blocking SPI accesses.
 */
void mrf24j40_task_run(void);

/**
 * @brief Check whether the task completed its initialization sequence.
 *
 * @return true if the task is ready for normal operation, otherwise false.
 *
 * @details
 * A return value of true indicates that the low-level transceiver
 * initialization, role configuration and local address assignment have already
 * been completed.
 */
bool mrf24j40_task_is_ready(void);

/**
 * @brief Check whether one received packet is available in the task buffer.
 *
 * @return true if a packet is available for retrieval, otherwise false.
 *
 * @details
 * The task stores at most one received packet internally. This function allows
 * the application to check whether that packet has already been captured from
 * the low-level driver and is waiting to be consumed.
 */
bool mrf24j40_task_has_rx_packet(void);

/**
 * @brief Retrieve the last received packet buffered by the task.
 *
 * @param[out] p_packet Pointer to the destination packet container.
 *
 * @return
 * - true: A packet was available and copied successfully.
 * - false: No packet was available or @p p_packet was NULL.
 *
 * @details
 * On success, the internal RX available flag is cleared so that the buffered
 * packet is considered consumed by the application.
 */
bool mrf24j40_task_get_rx_packet(mrf24j40_packet_t * p_packet);

/**
 * @brief Queue one packet for transmission through the cooperative task.
 *
 * @param p_packet Pointer to the packet to be transmitted.
 * @param ack_request Set to true if the transmission should request an ACK.
 *
 * @return
 * - MRF24J40_TASK_OK: Request accepted successfully.
 * - MRF24J40_TASK_E_BUSY: Task not ready or another TX request is still pending.
 * - MRF24J40_TASK_E_PARAM: Invalid packet pointer.
 *
 * @details
 * This function stores a copy of the user packet in the internal task context
 * and marks it as pending for later transmission.
 *
 * The actual FIFO loading and TX trigger occur later from
 * mrf24j40_task_run().
 *
 * @note
 * If @p ack_request is true, the caller is still responsible for setting the
 * ACK request bit in the MAC Frame Control field of the frame itself.
 */
mrf24j40_task_status_t mrf24j40_task_request_tx(const mrf24j40_packet_t * p_packet,
                                                bool ack_request);

/**
 * @brief Get and clear the latched TX completion indication.
 *
 * @return true if a TX completion event was latched, otherwise false.
 *
 * @details
 * This function reports whether the task observed completion of the last
 * transmission attempt. The completion flag is cleared when this function
 * returns true.
 */
bool mrf24j40_task_get_last_tx_done(void);

/**
 * @brief Get the success status associated with the last completed TX attempt.
 *
 * @return true if the last completed TX attempt was considered successful,
 *         otherwise false.
 *
 * @details
 * In the current implementation, success is inferred from reception of the
 * low-level TX completion indication only.
 *
 * @note
 * This function does not clear the stored result.
 * Future improvements may refine this result by checking TXSTAT to detect
 * CCA failures or retry-related failures.
 */
bool mrf24j40_task_get_last_tx_success(void);

#endif /* MRF24J40_TASK_H_ */

/** @} */
