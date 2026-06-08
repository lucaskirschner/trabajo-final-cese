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
 * The radio task owns the MRF24J40 driver and serializes all access to the
 * radio transceiver. Application modules shall not call the MRF24J40 driver
 * directly.
 *
 * During startup, the task performs a hardware reset of the MRF24J40, then
 * initializes the transceiver as a nonbeacon PAN coordinator, configures the
 * PAN ID, the local short address and the extended address.
 *
 * RX path:
 * - The MRF24J40 INT pin wakes the radio task through APP_RADIO_EVT_IRQ.
 * - The task updates the internal interrupt flags from the MRF24J40 INTSTAT
 *   register.
 * - If an RX packet is pending, the task reads the RX FIFO.
 * - The first application payload byte is copied into radioInputQueueHandle.
 *
 * TX path:
 * - app_radio_send() enqueues one application byte into radioOutputQueueHandle.
 * - APP_RADIO_EVT_TX_READY wakes the radio task.
 * - The task builds a basic IEEE 802.15.4 data frame and writes it to the
 *   MRF24J40 TX Normal FIFO.
 *
 * Faults detected asynchronously by the task are reported through
 * radioFaultEventHandle.
 *
 * @ingroup app_radio
 * @{
 */

/* ============================= Includes ================================== */

#include "app_radio.h"
#include "main.h"
#include "mrf24j40.h"
#include "swo.h"

#include "stm32h5xx_hal_gpio.h"

#include "cmsis_os2.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* ============================ Local Macros =============================== */

/* ============================ Local Types ================================ */

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

/* ======================= Local Static Data ================================ */
static app_radio_config_t app_radio_cfg;

/* ============================= RTOS objects ============================== */
extern osEventFlagsId_t radioEventHandle;

/* ===================== Public Function Definitions ======================= */

void app_radio_task(void * argument)
{
    uint32_t flags;
    mrf24j40_status_t radio_status;
    mrf24j40_packet_t packet;

    (void)argument;

    app_radio_init();

    for (;;)
    {
        flags = osEventFlagsWait(radioEventHandle,
                                 APP_RADIO_EVT_IRQ,
                                 osFlagsWaitAny,
                                 osWaitForever);

        if ((flags & APP_RADIO_EVT_IRQ) != 0u)
        {
            radio_status = mrf24j40_update_interrupt_flags();

            if (radio_status == MRF24J40_OK)
            {
                radio_status = mrf24j40_read_rx_fifo(&packet);

                if (radio_status == MRF24J40_OK)
                {
                    printf("[RADIO] RX frame received. Length: %u\r\n",
                           packet.frame_length);
                }
                else if (radio_status == MRF24J40_E_NO_RX_PACKET)
                {
                    /* The interrupt was not caused by an RX packet. */
                }
                else
                {
                    printf("[RADIO] RX FIFO read failed: %d\r\n",
                           (int)radio_status);
                }
            }
            else
            {
                printf("[RADIO] interrupt flag update failed: %d\r\n",
                       (int)radio_status);
            }
        }
    }
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
 * @brief Initialize the MRF24J40 radio transceiver.
 */
static void app_radio_init(void)
{
	mrf24j40_status_t status;

    printf("[RADIO] init start\r\n");

    app_radio_cfg.pan_id = 0x1234u;
    app_radio_cfg.short_address = 0x0001u;
    app_radio_cfg.role = APP_RADIO_ROLE_PAN_COORDINATOR;

    status = mrf24j40_init();
    if(status == MRF24J40_OK)
    {
    	printf("[RADIO] mrf24j40 OK initialized\r\n");
    }
    else
    {
    	printf("[RADIO] mrf24j40 ERROR initialized\r\n");
    }

    if (app_radio_cfg.role == APP_RADIO_ROLE_PAN_COORDINATOR)
    {
    	status = mrf24j40_configure_nonbeacon_pan_coordinator();
        if(status == MRF24J40_OK)
        {
            printf("[RADIO] mrf24j40 OK role configured\r\n");
        }
        else
        {
            printf("[RADIO] mrf24j40 ERROR role configured\r\n");
        }
    }
    else
    {
        (void)mrf24j40_configure_nonbeacon_device();
    }

    status = mrf24j40_set_pan_id(app_radio_cfg.pan_id);
    if(status == MRF24J40_OK)
    {
    	printf("[RADIO] mrf24j40 OK pan ID configured\r\n");
    }
    else
    {
    	printf("[RADIO] mrf24j40 ERROR pan ID configured\r\n");
    }

    status = mrf24j40_set_short_address(app_radio_cfg.short_address);
    if(status == MRF24J40_OK)
    {
    	printf("[RADIO] mrf24j40 OK short address configured\r\n");
    }
    else
    {
    	printf("[RADIO] mrf24j40 ERROR short address configured\r\n");
    }

    status = mrf24j40_set_extended_address();
    if(status == MRF24J40_OK)
    {
    	printf("[RADIO] mrf24j40 OK extended address configured\r\n");
    }
    else
    {
    	printf("[RADIO] mrf24j40 ERROR extended address configured\r\n");
    }
}

/** @} */
