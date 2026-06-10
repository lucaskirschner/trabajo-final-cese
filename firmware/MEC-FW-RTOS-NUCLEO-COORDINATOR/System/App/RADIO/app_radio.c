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
 * The MRF24J40 is initialized and configured when app_radio_task() starts.
 * The radio is configured in non-beacon mode either as PAN Coordinator or
 * Device according to the local configuration.
 *
 * RX path:
 * - The MRF24J40 INT pin wakes the radio task through APP_RADIO_EVT_IRQ.
 * - The task updates the internal interrupt flags.
 * - If an RX packet is pending, the RX FIFO is read.
 * - The received frame is printed through SWO.
 * - The first received frame byte is placed into radioInputQueueHandle.
 *
 * TX path:
 * - app_radio_send() enqueues one byte into radioOutputQueueHandle.
 * - APP_RADIO_EVT_TX_READY wakes the radio task.
 * - The task writes the byte to the MRF24J40 TX Normal FIFO.
 *
 * Faults detected asynchronously by the task are reported through
 * radioFaultHandle.
 *
 * @ingroup app_radio
 * @{
 */

/* ============================= Includes ================================== */

#include "app_radio.h"

#include "mrf24j40.h"
#include "mrf24j40_port.h"
#include "mrf24j40_reg.h"

#include "main.h"
#include "swo.h"

#include "cmsis_os2.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* ============================ Local Macros =============================== */

#define APP_RADIO_PAN_ID                 ((uint16_t)0x1234u)
#define APP_RADIO_SHORT_ADDRESS          ((uint16_t)0x0001u)

#define APP_RADIO_EXPECTED_PANIDL        ((uint8_t)0x34u)
#define APP_RADIO_EXPECTED_PANIDH        ((uint8_t)0x12u)

#define APP_RADIO_EXPECTED_INTCON        ((uint8_t)(INTCON_SLPIE     | \
                                                    INTCON_WAKEIE    | \
                                                    INTCON_HSYMTMRIE | \
                                                    INTCON_SECIE     | \
                                                    INTCON_TXG2IE    | \
                                                    INTCON_TXG1IE))

#define APP_RADIO_FRAME_LENGTH           ((uint8_t)1u)

/* ========================== Local Types ================================== */

typedef enum
{
    APP_RADIO_ROLE_DEVICE = 0,
    APP_RADIO_ROLE_PAN_COORDINATOR
} app_radio_role_t;

typedef struct
{
    uint16_t pan_id;
    uint16_t short_address;
    app_radio_role_t role;
} app_radio_config_t;

/* ========================== Private Prototypes =========================== */

static void app_radio_init(void);
static void app_radio_verify_register_readback(void);
static void app_radio_process_irq(void);
static void app_radio_process_tx_queue(void);
static void app_radio_print_packet(const mrf24j40_packet_t * p_packet);
static uint32_t app_radio_status_to_fault(mrf24j40_status_t status);

/* ======================= External RTOS Objects ============================ */

extern osEventFlagsId_t radioEventHandle;
extern osEventFlagsId_t radioFaultHandle;
extern osMessageQueueId_t radioInputQueueHandle;
extern osMessageQueueId_t radioOutputQueueHandle;

/* ======================= Local Static Data ================================ */

static bool app_radio_tx_busy = false;

static const app_radio_config_t app_radio_cfg =
{
    .pan_id = APP_RADIO_PAN_ID,
    .short_address = APP_RADIO_SHORT_ADDRESS,
    .role = APP_RADIO_ROLE_PAN_COORDINATOR
};

/* ===================== Public Function Definitions ======================= */

void app_radio_task(void * argument)
{
    uint32_t flags;

    (void)argument;

    app_radio_init();

    printf("[APP_RADIO] INT initial level: %lu\r\n",
           (uint32_t)HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin));

    for (;;)
    {
        flags = osEventFlagsWait(radioEventHandle,
                                 APP_RADIO_EVT_IRQ | APP_RADIO_EVT_TX_READY,
                                 osFlagsWaitAny,
                                 osWaitForever);

        if ((flags & osFlagsError) == 0u)
        {
            if ((flags & APP_RADIO_EVT_IRQ) != 0u)
            {
                printf("[APP_RADIO] IRQ flag detected\r\n");
                app_radio_process_irq();
            }

            if ((flags & APP_RADIO_EVT_TX_READY) != 0u)
            {
                app_radio_process_tx_queue();
            }
        }
        else
        {
            (void)osEventFlagsSet(radioFaultHandle,
                                  APP_RADIO_FAULT_OS_ERROR);
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
        (void)osEventFlagsSet(radioFaultHandle,
                              APP_RADIO_FAULT_TX_QUEUE_FULL);

        status = APP_RADIO_E_QUEUE_FULL;
    }
    else
    {
        (void)osEventFlagsSet(radioFaultHandle,
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
            (void)osEventFlagsSet(radioFaultHandle,
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
        mrf24j40_set_interrupt_pending();

        (void)osEventFlagsSet(radioEventHandle,
                              APP_RADIO_EVT_IRQ);
    }
}

/* ===================== Private Function Definitions ====================== */

static void app_radio_init(void)
{
    printf("[APP_RADIO] init start\r\n");

    (void)mrf24j40_init();
    printf("[APP_RADIO] mrf24j40 initialized\r\n");

    if (app_radio_cfg.role == APP_RADIO_ROLE_PAN_COORDINATOR)
    {
        (void)mrf24j40_configure_nonbeacon_pan_coordinator();
    }
    else
    {
        (void)mrf24j40_configure_nonbeacon_device();
    }

    printf("[APP_RADIO] role configured\r\n");

    (void)mrf24j40_set_pan_id(app_radio_cfg.pan_id);
    (void)mrf24j40_set_short_address(app_radio_cfg.short_address);
    (void)mrf24j40_set_extended_address();

    app_radio_verify_register_readback();

    printf("[APP_RADIO] init complete\r\n");
}

static void app_radio_verify_register_readback(void)
{
    mrf24j40_port_status_t port_status;
    uint8_t reg_value;

    port_status = mrf24j40_port_read_short(INTCON, &reg_value);

    if (port_status == MRF24J40_PORT_OK)
    {
        printf("[APP_RADIO] INTCON read: 0x%02X expected: 0x%02X %s\r\n",
               reg_value,
               APP_RADIO_EXPECTED_INTCON,
               (reg_value == APP_RADIO_EXPECTED_INTCON) ? "OK" : "FAIL");
    }
    else
    {
        printf("[APP_RADIO] INTCON read failed: %d\r\n",
               (int)port_status);
    }

    port_status = mrf24j40_port_read_short(PANIDL, &reg_value);

    if (port_status == MRF24J40_PORT_OK)
    {
        printf("[APP_RADIO] PANIDL read: 0x%02X expected: 0x%02X %s\r\n",
               reg_value,
               APP_RADIO_EXPECTED_PANIDL,
               (reg_value == APP_RADIO_EXPECTED_PANIDL) ? "OK" : "FAIL");
    }
    else
    {
        printf("[APP_RADIO] PANIDL read failed: %d\r\n",
               (int)port_status);
    }

    port_status = mrf24j40_port_read_short(PANIDH, &reg_value);

    if (port_status == MRF24J40_PORT_OK)
    {
        printf("[APP_RADIO] PANIDH read: 0x%02X expected: 0x%02X %s\r\n",
               reg_value,
               APP_RADIO_EXPECTED_PANIDH,
               (reg_value == APP_RADIO_EXPECTED_PANIDH) ? "OK" : "FAIL");
    }
    else
    {
        printf("[APP_RADIO] PANIDH read failed: %d\r\n",
               (int)port_status);
    }
}

static void app_radio_process_irq(void)
{
    mrf24j40_status_t radio_status;
    mrf24j40_packet_t packet;
    uint32_t fault_flags;
    osStatus_t queue_status;
    bool tx_complete;
    uint8_t data;

    radio_status = mrf24j40_update_interrupt_flags();

    printf("[APP_RADIO] update_interrupt_flags: %d\r\n",
           (int)radio_status);

    if (radio_status != MRF24J40_OK)
    {
        fault_flags = app_radio_status_to_fault(radio_status);

        if (fault_flags != 0u)
        {
            (void)osEventFlagsSet(radioFaultHandle, fault_flags);
        }
    }
    else
    {
        radio_status = mrf24j40_read_rx_fifo(&packet);

        printf("[APP_RADIO] read_rx_fifo: %d\r\n",
               (int)radio_status);

        if (radio_status == MRF24J40_OK)
        {
            app_radio_print_packet(&packet);

            if (packet.frame_length >= APP_RADIO_FRAME_LENGTH)
            {
                data = packet.frame[0];

                queue_status = osMessageQueuePut(radioInputQueueHandle,
                                                 &data,
                                                 0u,
                                                 0u);

                if (queue_status == osErrorResource)
                {
                    (void)osEventFlagsSet(radioFaultHandle,
                                          APP_RADIO_FAULT_RX_QUEUE_FULL);
                }
                else if (queue_status != osOK)
                {
                    (void)osEventFlagsSet(radioFaultHandle,
                                          APP_RADIO_FAULT_OS_ERROR);
                }
                else
                {
                    /* RX byte queued successfully. */
                }
            }
            else
            {
                (void)osEventFlagsSet(radioFaultHandle,
                                      APP_RADIO_FAULT_FRAME_ERROR);
            }
        }
        else if (radio_status != MRF24J40_E_NO_RX_PACKET)
        {
            fault_flags = app_radio_status_to_fault(radio_status);

            if (fault_flags != 0u)
            {
                (void)osEventFlagsSet(radioFaultHandle, fault_flags);
            }
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

            if (fault_flags != 0u)
            {
                (void)osEventFlagsSet(radioFaultHandle, fault_flags);
            }
        }
    }
}

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

                if (fault_flags != 0u)
                {
                    (void)osEventFlagsSet(radioFaultHandle, fault_flags);
                }
            }
        }
        else if (queue_status == osErrorResource)
        {
            /* TX queue empty. Nothing to transmit. */
        }
        else
        {
            (void)osEventFlagsSet(radioFaultHandle,
                                  APP_RADIO_FAULT_OS_ERROR);
        }
    }
}

static void app_radio_print_packet(const mrf24j40_packet_t * p_packet)
{
    uint8_t i;

    if (p_packet != NULL)
    {
        printf("[APP_RADIO] RX frame length: %u\r\n",
               p_packet->frame_length);

        printf("[APP_RADIO] RX frame: ");

        for (i = 0u; i < p_packet->frame_length; i++)
        {
            printf("%02X ", p_packet->frame[i]);
        }

        printf("\r\n");

        printf("[APP_RADIO] LQI: 0x%02X RSSI: 0x%02X\r\n",
               p_packet->lqi,
               p_packet->rssi);
    }
}

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
