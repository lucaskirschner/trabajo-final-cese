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
 * @file    app_radio.c
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-05-31
 * @brief   Radio application task implementation.
 *
 * @details
 * This module implements the CMSIS-RTOS2 radio application task.
 *
 * The task owns the MRF24J40 driver and serializes all access to the radio.
 * External application code does not call the radio driver directly.
 *
 * TX path:
 * - app_radio_send() enqueues one byte into radioOutputQueueHandle.
 * - APP_RADIO_EVT_TX_READY wakes the radio task.
 * - The task writes the byte to the MRF24J40 TX Normal FIFO.
 *
 * RX path:
 * - The MRF24J40 INT pin wakes the radio task through APP_RADIO_EVT_IRQ.
 * - The task decodes INTSTAT and reads the RX FIFO when a packet is pending.
 * - The first payload byte is placed into radioInputQueueHandle.
 *
 * Faults detected asynchronously by the task are reported through
 * radioFaultEventHandle.
 *
 * @ingroup app_radio
 * @{
 */

/* ============================= Includes ================================== */

#include "app_radio.h"
#include "mrf24j40.h"
#include "main.h"

#include "cmsis_os2.h"

#include <stdbool.h>
#include <stdint.h>

/* ============================ Local Macros =============================== */

/**
 * @brief Application payload length currently handled by this module.
 */
#define APP_RADIO_FRAME_LENGTH        ((uint8_t)1u)

/* ========================== Private Prototypes =========================== */

static void app_radio_process_irq(void);
static void app_radio_process_tx_queue(void);
static uint32_t app_radio_status_to_fault(mrf24j40_status_t status);

/* ======================= External RTOS Objects ============================ */

extern osEventFlagsId_t radioEventHandle;
extern osEventFlagsId_t radioFaultEventHandle;
extern osMessageQueueId_t radioInputQueueHandle;
extern osMessageQueueId_t radioOutputQueueHandle;

/* ======================= Local Static Data ================================ */

static bool app_radio_tx_busy = false;

/* ===================== Public Function Definitions ======================= */

void app_radio_task(void * argument)
{
    uint32_t flags;

    (void)argument;

    for (;;)
    {
        flags = osEventFlagsWait(radioEventHandle,
                                 APP_RADIO_EVT_IRQ | APP_RADIO_EVT_TX_READY,
                                 osFlagsWaitAny,
                                 osWaitForever);

        if ((flags & APP_RADIO_EVT_IRQ) != 0u)
        {
            app_radio_process_irq();
        }

        if ((flags & APP_RADIO_EVT_TX_READY) != 0u)
        {
            app_radio_process_tx_queue();
        }
    }
}

app_radio_status_t app_radio_send(uint8_t data)
{
    osStatus_t os_status;
    app_radio_status_t status;

    os_status = osMessageQueuePut(radioOutputQueueHandle,
                                  &data,
                                  0u,
                                  0u);

    if (os_status == osOK)
    {
        (void)osEventFlagsSet(radioEventHandle, APP_RADIO_EVT_TX_READY);
        status = APP_RADIO_OK;
    }
    else if (os_status == osErrorResource)
    {
        (void)osEventFlagsSet(radioFaultEventHandle,
                              APP_RADIO_FAULT_TX_QUEUE_FULL);

        status = APP_RADIO_E_QUEUE_FULL;
    }
    else
    {
        (void)osEventFlagsSet(radioFaultEventHandle,
                              APP_RADIO_FAULT_OS_ERROR);

        status = APP_RADIO_E_OS;
    }

    return status;
}

app_radio_status_t app_radio_receive(uint8_t * p_data)
{
    osStatus_t os_status;
    app_radio_status_t status;

    if (p_data == NULL)
    {
        status = APP_RADIO_E_NULL;
    }
    else
    {
        os_status = osMessageQueueGet(radioInputQueueHandle,
                                      p_data,
                                      NULL,
                                      0u);

        if (os_status == osOK)
        {
            status = APP_RADIO_OK;
        }
        else if (os_status == osErrorResource)
        {
            status = APP_RADIO_E_QUEUE_EMPTY;
        }
        else
        {
            (void)osEventFlagsSet(radioFaultEventHandle,
                                  APP_RADIO_FAULT_OS_ERROR);

            status = APP_RADIO_E_OS;
        }
    }

    return status;
}

/* ===================== HAL Callback ====================================== */

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == INT_Pin)
    {
        (void)osEventFlagsSet(radioEventHandle, APP_RADIO_EVT_IRQ);
    }
}

/* ===================== Private Function Definitions ====================== */

/**
 * @brief Process a pending MRF24J40 interrupt.
 *
 * @details
 * This function is executed only from app_radio_task() context.
 *
 * It reads and decodes the MRF24J40 INTSTAT register through the driver.
 * If an RX packet is pending, it reads the RX FIFO and enqueues the first
 * received byte into radioInputQueueHandle.
 *
 * It also checks TX completion and, if the previous transmission finished,
 * allows the next queued byte to be transmitted.
 */
static void app_radio_process_irq(void)
{
    mrf24j40_status_t radio_status;
    mrf24j40_packet_t packet;
    uint32_t fault_flags;
    osStatus_t queue_status;
    bool tx_complete;
    uint8_t data;

    radio_status = mrf24j40_update_interrupt_flags();

    if (radio_status != MRF24J40_OK)
    {
        fault_flags = app_radio_status_to_fault(radio_status);
        (void)osEventFlagsSet(radioFaultEventHandle, fault_flags);
    }
    else
    {
        radio_status = mrf24j40_read_rx_fifo(&packet);

        if (radio_status == MRF24J40_OK)
        {
            if (packet.frame_length >= APP_RADIO_FRAME_LENGTH)
            {
                data = packet.frame[0];

                queue_status = osMessageQueuePut(radioInputQueueHandle,
                                                 &data,
                                                 0u,
                                                 0u);

                if (queue_status == osErrorResource)
                {
                    (void)osEventFlagsSet(radioFaultEventHandle,
                                          APP_RADIO_FAULT_RX_QUEUE_FULL);
                }
                else if (queue_status != osOK)
                {
                    (void)osEventFlagsSet(radioFaultEventHandle,
                                          APP_RADIO_FAULT_OS_ERROR);
                }
                else
                {
                    /* RX byte queued successfully. */
                }
            }
            else
            {
                (void)osEventFlagsSet(radioFaultEventHandle,
                                      APP_RADIO_FAULT_FRAME_ERROR);
            }
        }
        else if (radio_status != MRF24J40_E_NO_RX_PACKET)
        {
            fault_flags = app_radio_status_to_fault(radio_status);
            (void)osEventFlagsSet(radioFaultEventHandle, fault_flags);
        }
        else
        {
            /* No RX packet pending. */
        }

        radio_status = mrf24j40_get_tx_complete(&tx_complete);

        if (radio_status == MRF24J40_OK)
        {
            if (tx_complete == true)
            {
                app_radio_tx_busy = false;
                app_radio_process_tx_queue();
            }
        }
        else
        {
            fault_flags = app_radio_status_to_fault(radio_status);
            (void)osEventFlagsSet(radioFaultEventHandle, fault_flags);
        }
    }
}

/**
 * @brief Process one pending TX queue item if the radio is available.
 *
 * @details
 * This function is executed only from app_radio_task() context.
 *
 * If the radio is not busy and the TX queue contains one byte, the byte is
 * wrapped into a minimal MRF24J40 packet and transmitted using the TX Normal
 * FIFO.
 *
 * If the driver reports an error, the status is translated into an application
 * fault flag and reported through radioFaultEventHandle.
 */
static void app_radio_process_tx_queue(void)
{
    osStatus_t queue_status;
    mrf24j40_status_t radio_status;
    mrf24j40_packet_t packet;
    uint32_t fault_flags;
    uint8_t data;

    if (app_radio_tx_busy == false)
    {
        queue_status = osMessageQueueGet(radioOutputQueueHandle,
                                         &data,
                                         NULL,
                                         0u);

        if (queue_status == osOK)
        {
            packet.frame_length = APP_RADIO_FRAME_LENGTH;
            packet.frame[0] = data;
            packet.lqi = 0u;
            packet.rssi = 0u;

            radio_status = mrf24j40_write_tx_normal_fifo(&packet, false);

            if (radio_status == MRF24J40_OK)
            {
                app_radio_tx_busy = true;
            }
            else
            {
                app_radio_tx_busy = false;

                fault_flags = app_radio_status_to_fault(radio_status);
                (void)osEventFlagsSet(radioFaultEventHandle, fault_flags);
            }
        }
        else if (queue_status == osErrorResource)
        {
            /* TX queue empty. Nothing to transmit. */
        }
        else
        {
            (void)osEventFlagsSet(radioFaultEventHandle,
                                  APP_RADIO_FAULT_OS_ERROR);
        }
    }
}

/**
 * @brief Convert an MRF24J40 driver status code to an application fault flag.
 *
 * @param status Status code returned by the MRF24J40 driver.
 *
 * @return Application-level radio fault flag.
 */
static uint32_t app_radio_status_to_fault(mrf24j40_status_t status)
{
    uint32_t fault;

    switch (status)
    {
        case MRF24J40_E_NULL:
            fault = APP_RADIO_FAULT_NULL_ERROR;
            break;

        case MRF24J40_E_PARAM:
            fault = APP_RADIO_FAULT_PARAM_ERROR;
            break;

        case MRF24J40_E_TIMEOUT:
            fault = APP_RADIO_FAULT_TIMEOUT;
            break;

        case MRF24J40_E_FRAME:
            fault = APP_RADIO_FAULT_FRAME_ERROR;
            break;

        case MRF24J40_E_STATE:
            fault = APP_RADIO_FAULT_STATE_ERROR;
            break;

        case MRF24J40_E_NO_RX_PACKET:
            fault = 0u;
            break;

        case MRF24J40_E_HW:
        default:
            fault = APP_RADIO_FAULT_HW_ERROR;
            break;
    }

    return fault;
}

/** @} */
