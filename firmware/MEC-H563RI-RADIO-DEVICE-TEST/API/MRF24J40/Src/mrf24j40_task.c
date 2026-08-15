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
#define APP_RADIO_SHORT_ADDRESS          ((uint16_t)0x0002u)
#define APP_RADIO_DESTINATION_ADDRESS    ((uint16_t)0x0001u)

#define APP_RADIO_EXPECTED_PANIDL        ((uint8_t)0x34u)
#define APP_RADIO_EXPECTED_PANIDH        ((uint8_t)0x12u)

#define APP_RADIO_EXPECTED_INTCON        ((uint8_t)(INTCON_SLPIE     | \
                                                    INTCON_WAKEIE    | \
                                                    INTCON_HSYMTMRIE | \
                                                    INTCON_SECIE     | \
                                                    INTCON_TXG2IE    | \
                                                    INTCON_TXG1IE))

/*
 * IEEE 802.15.4 Frame Control Field:
 *
 * Bits 0..2   Frame type = Data (001)
 * Bit  5      ACK request = 1
 * Bit  6      PAN ID compression = 1
 * Bits 10..11 Destination addressing = short (10)
 * Bits 14..15 Source addressing = short (10)
 *
 * FCF = 0x8841
 *
 * The FCF is transmitted LSB first:
 * frame[0] = 0x41
 * frame[1] = 0x88
 */
#define APP_RADIO_FCF_LOW                ((uint8_t)0x41u)
#define APP_RADIO_FCF_HIGH               ((uint8_t)0x88u)

#define APP_RADIO_MAC_HEADER_LENGTH      ((uint8_t)9u)
#define APP_RADIO_PAYLOAD_LENGTH         ((uint8_t)2u)
#define APP_RADIO_FRAME_LENGTH           ((uint8_t)(APP_RADIO_MAC_HEADER_LENGTH + \
                                                    APP_RADIO_PAYLOAD_LENGTH))

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
static void app_transmit_packet(void);
static void app_print_packet(const mrf24j40_packet_t * p_packet);

static volatile bool app_radio_irq_flag = false;
static volatile bool app_button_pressed_flag = false;

static uint8_t app_sequence_number = 0u;
static uint8_t app_packet_counter = 0u;

static const app_radio_config_t app_radio_cfg =
{
    .pan_id = APP_RADIO_PAN_ID,
    .short_address = APP_RADIO_SHORT_ADDRESS,
    .role = APP_RADIO_ROLE_DEVICE
};

void app_task(void)
{
    app_init();

    printf("[APP] INT initial level: %lu\r\n",
           (uint32_t)HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin));

    printf("[APP] Press BUTTON to transmit a packet\r\n");

    for (;;)
    {
        /*
         * Process radio interrupts.
         *
         * This is required to detect events such as:
         * - RXIF
         * - TXNIF
         */
        if (app_radio_irq_flag == true)
        {
            app_radio_irq_flag = false;

            app_process_radio_irq();
        }

        /*
         * A button interrupt only sets the flag.
         * The actual SPI transaction and packet transmission
         * are executed here, outside interrupt context.
         */
        if (app_button_pressed_flag == true)
        {
            app_button_pressed_flag = false;

            app_transmit_packet();
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

    printf("[APP] role: %s\r\n",
           (app_radio_cfg.role == APP_RADIO_ROLE_PAN_COORDINATOR) ?
           "PAN_COORDINATOR" :
           "DEVICE");

    printf("[APP] PAN ID: 0x%04X\r\n",
           app_radio_cfg.pan_id);

    printf("[APP] short address: 0x%04X\r\n",
           app_radio_cfg.short_address);

    printf("[APP] TX destination: 0x%04X\r\n",
           APP_RADIO_DESTINATION_ADDRESS);
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
    bool tx_complete;

    status = mrf24j40_update_interrupt_flags();

    if (status != MRF24J40_OK)
    {
        printf("[APP] update_interrupt_flags failed: %d\r\n",
               (int)status);

        return;
    }

    /*
     * Check whether the interrupt corresponds to the end
     * of a TX Normal FIFO transmission.
     */
    tx_complete = false;

    status = mrf24j40_get_tx_complete(&tx_complete);

    if (status == MRF24J40_OK)
    {
        if (tx_complete == true)
        {
            printf("[APP] TX complete\r\n");
        }
    }
    else
    {
        printf("[APP] get_tx_complete failed: %d\r\n",
               (int)status);
    }

    /*
     * Try to consume a received frame.
     *
     * If the interrupt was only TXNIF,
     * MRF24J40_E_NO_RX_PACKET is expected and is not an error.
     */
    status = mrf24j40_read_rx_fifo(&packet);

    if (status == MRF24J40_OK)
    {
        app_print_packet(&packet);
    }
    else if (status != MRF24J40_E_NO_RX_PACKET)
    {
        printf("[APP] read_rx_fifo failed: %d\r\n",
               (int)status);
    }
}

static void app_transmit_packet(void)
{
    mrf24j40_status_t status;
    mrf24j40_packet_t packet;

    /*
     * MAC Header
     *
     *  Byte 0 : Frame Control Field LSB
     *  Byte 1 : Frame Control Field MSB
     *  Byte 2 : Sequence Number
     *  Byte 3 : Destination PAN ID LSB
     *  Byte 4 : Destination PAN ID MSB
     *  Byte 5 : Destination short address LSB
     *  Byte 6 : Destination short address MSB
     *  Byte 7 : Source short address LSB
     *  Byte 8 : Source short address MSB
     *
     * PAN ID compression is enabled, therefore the source PAN ID
     * is omitted because source and destination belong to the same PAN.
     */

    packet.frame_length = APP_RADIO_FRAME_LENGTH;

    packet.frame[0] = APP_RADIO_FCF_LOW;
    packet.frame[1] = APP_RADIO_FCF_HIGH;

    packet.frame[2] = app_sequence_number;

    packet.frame[3] = (uint8_t)(app_radio_cfg.pan_id & 0x00FFu);
    packet.frame[4] = (uint8_t)((app_radio_cfg.pan_id >> 8u) & 0x00FFu);

    packet.frame[5] =
        (uint8_t)(APP_RADIO_DESTINATION_ADDRESS & 0x00FFu);

    packet.frame[6] =
        (uint8_t)((APP_RADIO_DESTINATION_ADDRESS >> 8u) & 0x00FFu);

    packet.frame[7] =
        (uint8_t)(app_radio_cfg.short_address & 0x00FFu);

    packet.frame[8] =
        (uint8_t)((app_radio_cfg.short_address >> 8u) & 0x00FFu);

    /*
     * Test payload.
     *
     * Byte 0:
     *     Fixed application identifier.
     *
     * Byte 1:
     *     Incrementing packet counter.
     */
    packet.frame[9] = 0x01u;
    packet.frame[10] = app_packet_counter;

    packet.lqi = 0u;
    packet.rssi = 0u;

    printf("[APP] BUTTON pressed\r\n");

    printf("[APP] TX packet #%u SEQ=%u -> 0x%04X\r\n",
           app_packet_counter,
           app_sequence_number,
           APP_RADIO_DESTINATION_ADDRESS);

    printf("[APP] TX frame: ");

    for (uint8_t i = 0u; i < packet.frame_length; i++)
    {
        printf("%02X ", packet.frame[i]);
    }

    printf("\r\n");

    status = mrf24j40_write_tx_normal_fifo(&packet, true);

    if (status == MRF24J40_OK)
    {
        printf("[APP] TX triggered\r\n");

        app_sequence_number++;
        app_packet_counter++;
    }
    else
    {
        printf("[APP] TX failed: %d\r\n",
               (int)status);
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
    /*
     * MRF24J40 interrupt.
     */
    if (GPIO_Pin == INT_Pin)
    {
        mrf24j40_task_notify_irq();
    }

    /*
     * User button interrupt.
     *
     * Do not access the radio or SPI here.
     * Only notify the application task.
     */
    if (GPIO_Pin == BUTTON_Pin)
    {
        app_button_pressed_flag = true;
    }
}
