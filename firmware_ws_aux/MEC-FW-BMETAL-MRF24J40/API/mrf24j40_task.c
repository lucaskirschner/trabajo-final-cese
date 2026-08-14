/*
 * mrf24j40_task.c
 *
 *  Created on: Jun 4, 2026
 *      Author: Noxie
 */

#include "mrf24j40_task.h"

#include "mrf24j40.h"
#include "mrf24j40_port.h"
#include "mrf24j40_reg.h"
#include "main.h"
#include "swo.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

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

static void app_init(void);
static void app_verify_register_readback(void);
static void app_process_radio_irq(void);
static void app_print_packet(const mrf24j40_packet_t * p_packet);

static volatile bool app_radio_irq_flag = false;

static const app_radio_config_t app_radio_cfg =
{
    .pan_id = APP_RADIO_PAN_ID,
    .short_address = APP_RADIO_SHORT_ADDRESS,
    .role = APP_RADIO_ROLE_PAN_COORDINATOR
};

void app_task(void)
{
    app_init();

    printf("[APP] INT initial level: %lu\r\n",
           (uint32_t)HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin));

    for (;;)
    {
        if (app_radio_irq_flag == true)
        {
            app_radio_irq_flag = false;

            printf("[APP] IRQ flag detected\r\n");

            app_process_radio_irq();
        }
    }
}

void mrf24j40_task_notify_irq(void)
{
    app_radio_irq_flag = true;
    mrf24j40_set_interrupt_pending();
}

static void app_init(void)
{
    printf("[APP] init start\r\n");

    (void)mrf24j40_init();
    printf("[APP] mrf24j40 initialized\r\n");

    if (app_radio_cfg.role == APP_RADIO_ROLE_PAN_COORDINATOR)
    {
        (void)mrf24j40_configure_nonbeacon_pan_coordinator();
    }
    else
    {
        (void)mrf24j40_configure_nonbeacon_device();
    }

    printf("[APP] role configured\r\n");

    (void)mrf24j40_set_pan_id(app_radio_cfg.pan_id);
    (void)mrf24j40_set_short_address(app_radio_cfg.short_address);
    (void)mrf24j40_set_extended_address();

    app_verify_register_readback();

    printf("[APP] init complete\r\n");
}

static void app_verify_register_readback(void)
{
    mrf24j40_port_status_t port_status;
    uint8_t reg_value;

    port_status = mrf24j40_port_read_short(INTCON, &reg_value);

    if (port_status == MRF24J40_PORT_OK)
    {
        printf("[APP] INTCON read: 0x%02X expected: 0x%02X %s\r\n",
               reg_value,
               APP_RADIO_EXPECTED_INTCON,
               (reg_value == APP_RADIO_EXPECTED_INTCON) ? "OK" : "FAIL");
    }
    else
    {
        printf("[APP] INTCON read failed: %d\r\n",
               (int)port_status);
    }

    port_status = mrf24j40_port_read_short(PANIDL, &reg_value);

    if (port_status == MRF24J40_PORT_OK)
    {
        printf("[APP] PANIDL read: 0x%02X expected: 0x%02X %s\r\n",
               reg_value,
               APP_RADIO_EXPECTED_PANIDL,
               (reg_value == APP_RADIO_EXPECTED_PANIDL) ? "OK" : "FAIL");
    }
    else
    {
        printf("[APP] PANIDL read failed: %d\r\n",
               (int)port_status);
    }

    port_status = mrf24j40_port_read_short(PANIDH, &reg_value);

    if (port_status == MRF24J40_PORT_OK)
    {
        printf("[APP] PANIDH read: 0x%02X expected: 0x%02X %s\r\n",
               reg_value,
               APP_RADIO_EXPECTED_PANIDH,
               (reg_value == APP_RADIO_EXPECTED_PANIDH) ? "OK" : "FAIL");
    }
    else
    {
        printf("[APP] PANIDH read failed: %d\r\n",
               (int)port_status);
    }
}

static void app_process_radio_irq(void)
{
    mrf24j40_status_t status;
    mrf24j40_packet_t packet;

    status = mrf24j40_update_interrupt_flags();

    printf("[APP] update_interrupt_flags: %d\r\n",
           (int)status);

    if (status == MRF24J40_OK)
    {
        status = mrf24j40_read_rx_fifo(&packet);

        printf("[APP] read_rx_fifo: %d\r\n",
               (int)status);

        if (status == MRF24J40_OK)
        {
            app_print_packet(&packet);
        }
    }
}

static void app_print_packet(const mrf24j40_packet_t * p_packet)
{
    uint8_t i;

    if (p_packet != NULL)
    {
        printf("[APP] RX frame length: %u\r\n",
               p_packet->frame_length);

        printf("[APP] RX frame: ");

        for (i = 0u; i < p_packet->frame_length; i++)
        {
            printf("%02X ", p_packet->frame[i]);
        }

        printf("\r\n");

        printf("[APP] LQI: 0x%02X RSSI: 0x%02X\r\n",
               p_packet->lqi,
               p_packet->rssi);
    }
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == INT_Pin)
    {
        mrf24j40_task_notify_irq();
    }
}
