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
 *****************************************************************************/

/**
 * @file    app_radio.c
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-05-31
 * @brief   Radio application task implementation.
 *
 * @details
 * This module implements the CMSIS-RTOS2 radio application task for a minimal
 * IEEE 802.15.4 star network composed of:
 *
 * - one PAN coordinator with short address 0x0000
 * - up to 16 remote devices with short addresses 0x0001 through 0x0010
 *
 * The task owns the MRF24J40 driver and serializes all access to the radio.
 * External application code does not call the radio driver directly.
 *
 * The MRF24J40 is initialized and configured when app_radio_task() starts.
 * The radio operates in nonbeacon mode either as PAN coordinator or device
 * according to APP_RADIO_NODE_ROLE.
 *
 * Node addresses are statically configured at compile time. No IEEE 802.15.4
 * association, discovery or dynamic address assignment procedure is performed.
 *
 * RX path:
 * - The MRF24J40 INT pin wakes the radio task through APP_RADIO_EVT_IRQ.
 * - The task updates the internal interrupt flags.
 * - If an RX packet is pending, the RX FIFO is read.
 * - The IEEE 802.15.4 MAC header is validated.
 * - The source short address and one-byte payload are placed into
 *   radioInputQueueHandle.
 *
 * TX path:
 * - app_radio_send() enqueues one destination address and one payload byte.
 * - APP_RADIO_EVT_TX_READY wakes the radio task.
 * - The task builds an IEEE 802.15.4 data frame using short addressing.
 * - The ACK Request bit is set in the MAC Frame Control field.
 * - The frame is written to the MRF24J40 TX Normal FIFO with ACK request
 *   enabled.
 * - When TXNIF is detected, the task obtains the final transmission result
 *   through mrf24j40_get_tx_result().
 *
 * Acknowledgment reception, automatic acknowledgment generation and packet
 * retransmission are handled entirely by the MRF24J40 hardware.
 *
 * The application does not perform software retransmissions. It only reports
 * the final result produced by the hardware after the transmission procedure
 * has completed.
 *
 * A missing acknowledgment after the hardware retry mechanism is exhausted is
 * reported through APP_RADIO_FAULT_TX_NO_ACK. A CSMA-CA channel access failure
 * is reported separately through APP_RADIO_FAULT_TX_CCA_FAILED.
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

/**
 * @brief Fixed IEEE 802.15.4 PAN identifier.
 */
#define APP_RADIO_PAN_ID                     ((uint16_t)0x1234u)

/**
 * @brief Expected PAN ID low byte.
 */
#define APP_RADIO_EXPECTED_PANIDL            ((uint8_t)0x34u)

/**
 * @brief Expected PAN ID high byte.
 */
#define APP_RADIO_EXPECTED_PANIDH            ((uint8_t)0x12u)

/**
 * @brief Expected MRF24J40 interrupt configuration.
 *
 * @details
 * INTCON interrupt enables are active-low. RXIE and TXNIE are therefore
 * intentionally omitted from this mask so that RX and TX Normal interrupts
 * remain enabled.
 */
#define APP_RADIO_EXPECTED_INTCON            ((uint8_t)(INTCON_SLPIE     | \
                                                        INTCON_WAKEIE    | \
                                                        INTCON_HSYMTMRIE | \
                                                        INTCON_SECIE     | \
                                                        INTCON_TXG2IE    | \
                                                        INTCON_TXG1IE))

/**
 * @brief IEEE 802.15.4 Frame Control Field used by application data frames.
 *
 * @details
 * The frame uses:
 *
 * - Frame Type = Data.
 * - Security Enabled = 0.
 * - Frame Pending = 0.
 * - ACK Request = 1.
 * - PAN ID Compression = 1.
 * - Destination Addressing Mode = 16-bit short address.
 * - Frame Version = legacy IEEE 802.15.4 format supported by MRF24J40.
 * - Source Addressing Mode = 16-bit short address.
 *
 * The resulting 16-bit Frame Control Field is 0x8861.
 */
#define APP_RADIO_FRAME_CONTROL              ((uint16_t)0x8861u)

/**
 * @brief Frame Control Field low byte.
 */
#define APP_RADIO_FRAME_CONTROL_LSB          ((uint8_t)0x61u)

/**
 * @brief Frame Control Field high byte.
 */
#define APP_RADIO_FRAME_CONTROL_MSB          ((uint8_t)0x88u)

/**
 * @brief IEEE 802.15.4 MAC header length used by this application.
 *
 * @details
 * Header layout:
 *
 * Byte 0: Frame Control Field LSB.
 * Byte 1: Frame Control Field MSB.
 * Byte 2: Sequence Number.
 * Byte 3: Destination PAN ID LSB.
 * Byte 4: Destination PAN ID MSB.
 * Byte 5: Destination Short Address LSB.
 * Byte 6: Destination Short Address MSB.
 * Byte 7: Source Short Address LSB.
 * Byte 8: Source Short Address MSB.
 */
#define APP_RADIO_MAC_HEADER_LENGTH          ((uint8_t)9u)

/**
 * @brief Application payload length in bytes.
 */
#define APP_RADIO_PAYLOAD_LENGTH             ((uint8_t)1u)

/**
 * @brief Complete MAC frame length excluding FCS.
 */
#define APP_RADIO_FRAME_LENGTH               ((uint8_t)(APP_RADIO_MAC_HEADER_LENGTH + \
                                                        APP_RADIO_PAYLOAD_LENGTH))

/**
 * @brief Frame Control Field LSB index.
 */
#define APP_RADIO_FRAME_FCF_L_INDEX          ((uint8_t)0u)

/**
 * @brief Frame Control Field MSB index.
 */
#define APP_RADIO_FRAME_FCF_H_INDEX          ((uint8_t)1u)

/**
 * @brief Sequence number index.
 */
#define APP_RADIO_FRAME_SEQUENCE_INDEX       ((uint8_t)2u)

/**
 * @brief Destination PAN ID LSB index.
 */
#define APP_RADIO_FRAME_DST_PAN_L_INDEX      ((uint8_t)3u)

/**
 * @brief Destination PAN ID MSB index.
 */
#define APP_RADIO_FRAME_DST_PAN_H_INDEX      ((uint8_t)4u)

/**
 * @brief Destination short address LSB index.
 */
#define APP_RADIO_FRAME_DST_ADDR_L_INDEX     ((uint8_t)5u)

/**
 * @brief Destination short address MSB index.
 */
#define APP_RADIO_FRAME_DST_ADDR_H_INDEX     ((uint8_t)6u)

/**
 * @brief Source short address LSB index.
 */
#define APP_RADIO_FRAME_SRC_ADDR_L_INDEX     ((uint8_t)7u)

/**
 * @brief Source short address MSB index.
 */
#define APP_RADIO_FRAME_SRC_ADDR_H_INDEX     ((uint8_t)8u)

/**
 * @brief Application payload index.
 */
#define APP_RADIO_FRAME_PAYLOAD_INDEX        ((uint8_t)9u)

/**
 * @brief Queue representation payload mask.
 */
#define APP_RADIO_QUEUE_DATA_MASK            ((uint32_t)0x000000FFu)

/**
 * @brief Queue representation address mask.
 */
#define APP_RADIO_QUEUE_ADDRESS_MASK         ((uint32_t)0x00FFFF00u)

/**
 * @brief Queue representation address shift.
 */
#define APP_RADIO_QUEUE_ADDRESS_SHIFT        ((uint8_t)8u)

/* ========================== Local Types ================================== */

typedef struct
{
    uint16_t pan_id;
    uint16_t short_address;
    uint8_t role;
} app_radio_config_t;

/* ========================== Private Prototypes =========================== */

static void app_radio_init(void);
static void app_radio_verify_register_readback(void);
static void app_radio_process_irq(void);
static void app_radio_process_tx_queue(void);

static void app_radio_build_data_frame(mrf24j40_packet_t * p_packet,
                                       uint16_t destination,
                                       uint8_t data);

static bool app_radio_decode_data_frame(
    const mrf24j40_packet_t * p_packet,
    uint16_t * p_source,
    uint8_t * p_data);

static bool app_radio_destination_is_valid(uint16_t destination);

static uint32_t app_radio_pack_queue_message(uint16_t address,
                                             uint8_t data);

static void app_radio_unpack_queue_message(uint32_t message,
                                           uint16_t * p_address,
                                           uint8_t * p_data);

static void app_radio_print_packet(
    const mrf24j40_packet_t * p_packet);

static uint32_t app_radio_status_to_fault(
    mrf24j40_status_t status);

/* ======================= External RTOS Objects ============================ */

extern osEventFlagsId_t radioEventHandle;
extern osEventFlagsId_t radioFaultHandle;
extern osMessageQueueId_t radioInputQueueHandle;
extern osMessageQueueId_t radioOutputQueueHandle;

/* ======================= Local Static Data ================================ */

static bool app_radio_tx_busy = false;
static uint8_t app_radio_sequence_number = 0u;

static const app_radio_config_t app_radio_cfg =
{
    .pan_id = APP_RADIO_PAN_ID,
    .short_address = APP_RADIO_SHORT_ADDRESS,
    .role = APP_RADIO_NODE_ROLE
};

/* ===================== Public Function Definitions ======================= */

void app_radio_task(void * argument)
{
    uint32_t flags;

    (void)argument;

    app_radio_init();

    printf("[APP_RADIO] role: %s\r\n",
           (app_radio_cfg.role == APP_RADIO_ROLE_PAN_COORDINATOR) ?
           "PAN_COORDINATOR" :
           "DEVICE");

    printf("[APP_RADIO] short address: 0x%04X\r\n",
           app_radio_cfg.short_address);

    printf("[APP_RADIO] INT initial level: %lu\r\n",
           (uint32_t)HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin));

    for (;;)
    {
        flags = osEventFlagsWait(radioEventHandle,
                                 APP_RADIO_EVT_IRQ |
                                 APP_RADIO_EVT_TX_READY,
                                 osFlagsWaitAny,
                                 osWaitForever);

        if ((flags & osFlagsError) == 0u)
        {
            if ((flags & APP_RADIO_EVT_IRQ) != 0u)
            {
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

app_radio_status_t app_radio_send(uint16_t destination, uint8_t data)
{
    osStatus_t os_status;
    app_radio_status_t status;
    uint32_t queue_message;

    if (app_radio_destination_is_valid(destination) == false)
    {
        status = APP_RADIO_E_PARAM;
    }
    else
    {
        queue_message = app_radio_pack_queue_message(destination, data);

        os_status = osMessageQueuePut(radioOutputQueueHandle,
                                      &queue_message,
                                      0u,
                                      0u);

        if (os_status == osOK)
        {
            (void)osEventFlagsSet(radioEventHandle,
                                  APP_RADIO_EVT_TX_READY);

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
    }

    return status;
}

app_radio_status_t app_radio_receive(uint16_t * p_source, uint8_t * p_data)
{
    osStatus_t os_status;
    app_radio_status_t status;
    uint32_t queue_message;

    if ((p_source == NULL) || (p_data == NULL))
    {
        status = APP_RADIO_E_NULL;
    }
    else
    {
        os_status = osMessageQueueGet(radioInputQueueHandle,
                                      &queue_message,
                                      NULL,
                                      0u);

        if (os_status == osOK)
        {
            app_radio_unpack_queue_message(queue_message,
                                           p_source,
                                           p_data);

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

/**
 * @brief Initialize and configure the radio for the local application node.
 *
 * @details
 * The radio is initialized and configured in nonbeacon mode according to
 * APP_RADIO_NODE_ROLE.
 *
 * PAN ID and short address are statically assigned. Extended addressing is
 * programmed by the lower-level driver but is not used by the minimal
 * application data frames.
 */
static void app_radio_init(void)
{
    mrf24j40_status_t radio_status;
    uint32_t fault_flags;

    printf("[APP_RADIO] init start\r\n");

    radio_status = mrf24j40_init();

    if (radio_status != MRF24J40_OK)
    {
        fault_flags = app_radio_status_to_fault(radio_status);

        if (fault_flags != 0u)
        {
            (void)osEventFlagsSet(radioFaultHandle, fault_flags);
        }

        printf("[APP_RADIO] mrf24j40 init failed: %d\r\n",
               (int)radio_status);

        return;
    }

    printf("[APP_RADIO] mrf24j40 initialized\r\n");

    if (app_radio_cfg.role == APP_RADIO_ROLE_PAN_COORDINATOR)
    {
        radio_status =
            mrf24j40_configure_nonbeacon_pan_coordinator();
    }
    else
    {
        radio_status =
            mrf24j40_configure_nonbeacon_device();
    }

    if (radio_status != MRF24J40_OK)
    {
        fault_flags = app_radio_status_to_fault(radio_status);

        if (fault_flags != 0u)
        {
            (void)osEventFlagsSet(radioFaultHandle, fault_flags);
        }

        printf("[APP_RADIO] role configuration failed: %d\r\n",
               (int)radio_status);

        return;
    }

    printf("[APP_RADIO] role configured\r\n");

    radio_status = mrf24j40_set_pan_id(app_radio_cfg.pan_id);

    if (radio_status != MRF24J40_OK)
    {
        fault_flags = app_radio_status_to_fault(radio_status);

        if (fault_flags != 0u)
        {
            (void)osEventFlagsSet(radioFaultHandle, fault_flags);
        }

        printf("[APP_RADIO] PAN ID configuration failed: %d\r\n",
               (int)radio_status);

        return;
    }

    radio_status =
        mrf24j40_set_short_address(app_radio_cfg.short_address);

    if (radio_status != MRF24J40_OK)
    {
        fault_flags = app_radio_status_to_fault(radio_status);

        if (fault_flags != 0u)
        {
            (void)osEventFlagsSet(radioFaultHandle, fault_flags);
        }

        printf("[APP_RADIO] short address configuration failed: %d\r\n",
               (int)radio_status);

        return;
    }

    radio_status = mrf24j40_set_extended_address();

    if (radio_status != MRF24J40_OK)
    {
        fault_flags = app_radio_status_to_fault(radio_status);

        if (fault_flags != 0u)
        {
            (void)osEventFlagsSet(radioFaultHandle, fault_flags);
        }

        printf("[APP_RADIO] extended address configuration failed: %d\r\n",
               (int)radio_status);

        return;
    }

    app_radio_verify_register_readback();

    printf("[APP_RADIO] init complete\r\n");
}

/**
 * @brief Verify basic MRF24J40 configuration registers.
 */
static void app_radio_verify_register_readback(void)
{
    mrf24j40_port_status_t port_status;
    uint8_t reg_value;
    uint8_t expected_value;

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

    expected_value =
        (uint8_t)(app_radio_cfg.short_address & 0x00FFu);

    port_status = mrf24j40_port_read_short(SADRL, &reg_value);

    if (port_status == MRF24J40_PORT_OK)
    {
        printf("[APP_RADIO] SADRL read: 0x%02X expected: 0x%02X %s\r\n",
               reg_value,
               expected_value,
               (reg_value == expected_value) ? "OK" : "FAIL");
    }
    else
    {
        printf("[APP_RADIO] SADRL read failed: %d\r\n",
               (int)port_status);
    }

    expected_value =
        (uint8_t)((app_radio_cfg.short_address >> 8u) & 0x00FFu);

    port_status = mrf24j40_port_read_short(SADRH, &reg_value);

    if (port_status == MRF24J40_PORT_OK)
    {
        printf("[APP_RADIO] SADRH read: 0x%02X expected: 0x%02X %s\r\n",
               reg_value,
               expected_value,
               (reg_value == expected_value) ? "OK" : "FAIL");
    }
    else
    {
        printf("[APP_RADIO] SADRH read failed: %d\r\n",
               (int)port_status);
    }
}

/**
 * @brief Process one pending MRF24J40 interrupt.
 *
 * @details
 * RX and TX completion conditions are processed after the lower-level driver
 * reads INTSTAT and updates its internal event flags.
 *
 * When a TX completion event is pending, mrf24j40_get_tx_result() is used to
 * obtain the final transmission status from TXSTAT.
 *
 * Successful transmissions and hardware retry counts are reported through the
 * debug console.
 *
 * Acknowledgment failure and CSMA-CA channel access failure are reported
 * through radioFaultHandle. No application-level retransmission is performed.
 */
static void app_radio_process_irq(void)
{
    mrf24j40_status_t radio_status;
    mrf24j40_packet_t packet;
    mrf24j40_tx_result_t tx_result;
    uint32_t fault_flags;
    uint32_t queue_message;
    osStatus_t queue_status;
    uint16_t source;
    uint8_t data;

    radio_status = mrf24j40_update_interrupt_flags();

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

        if (radio_status == MRF24J40_OK)
        {
            app_radio_print_packet(&packet);

            if (app_radio_decode_data_frame(&packet,
                                            &source,
                                            &data) == true)
            {
                queue_message =
                    app_radio_pack_queue_message(source, data);

                queue_status =
                    osMessageQueuePut(radioInputQueueHandle,
                                      &queue_message,
                                      0u,
                                      0u);

                if (queue_status == osErrorResource)
                {
                    (void)osEventFlagsSet(
                        radioFaultHandle,
                        APP_RADIO_FAULT_RX_QUEUE_FULL);
                }
                else if (queue_status != osOK)
                {
                    (void)osEventFlagsSet(
                        radioFaultHandle,
                        APP_RADIO_FAULT_OS_ERROR);
                }
                else
                {
                    printf("[APP_RADIO] RX src=0x%04X data=0x%02X\r\n",
                           source,
                           data);
                }
            }
            else
            {
                (void)osEventFlagsSet(
                    radioFaultHandle,
                    APP_RADIO_FAULT_FRAME_ERROR);
            }
        }
        else if (radio_status != MRF24J40_E_NO_RX_PACKET)
        {
            fault_flags = app_radio_status_to_fault(radio_status);

            if (fault_flags != 0u)
            {
                (void)osEventFlagsSet(radioFaultHandle,
                                      fault_flags);
            }
        }
        else
        {
            /* No RX packet pending. */
        }

        radio_status = mrf24j40_get_tx_result(&tx_result);

        if (radio_status == MRF24J40_OK)
        {
            if (tx_result.complete == true)
            {
                app_radio_tx_busy = false;

                if (tx_result.success == true)
                {
                    printf("[APP_RADIO] TX success retries=%u\r\n",
                           tx_result.retries);
                }
                else if (tx_result.cca_failed == true)
                {
                    printf("[APP_RADIO] TX failed: CCA retries=%u\r\n",
                           tx_result.retries);

                    (void)osEventFlagsSet(
                        radioFaultHandle,
                        APP_RADIO_FAULT_TX_CCA_FAILED);
                }
                else if ((tx_result.ack_requested == true) &&
                         (tx_result.ack_received == false))
                {
                    printf("[APP_RADIO] TX failed: no ACK retries=%u\r\n",
                           tx_result.retries);

                    (void)osEventFlagsSet(
                        radioFaultHandle,
                        APP_RADIO_FAULT_TX_NO_ACK);
                }
                else
                {
                    printf("[APP_RADIO] TX failed retries=%u\r\n",
                           tx_result.retries);

                    (void)osEventFlagsSet(
                        radioFaultHandle,
                        APP_RADIO_FAULT_HW_ERROR);
                }

                app_radio_process_tx_queue();
            }
        }
        else
        {
            fault_flags = app_radio_status_to_fault(radio_status);

            if (fault_flags != 0u)
            {
                (void)osEventFlagsSet(radioFaultHandle,
                                      fault_flags);
            }
        }
    }

    /*
     * Diagnostic: show the physical INT level after processing the current
     * interrupt sources.
     */
    printf("[APP_RADIO] IRQ processed, INT=%lu\r\n",
           (uint32_t)HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin));

    /*
     * If INT remains active-low after processing INTSTAT, force another radio
     * task wake-up instead of relying on a new falling edge.
     */
    if (HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin) == GPIO_PIN_RESET)
    {
        printf("[APP_RADIO] INT still low, reprocessing IRQ\r\n");

        mrf24j40_set_interrupt_pending();

        (void)osEventFlagsSet(radioEventHandle,
                              APP_RADIO_EVT_IRQ);
    }
}

/**
 * @brief Process one pending application transmit message.
 *
 * @details
 * A queued destination address and one-byte payload are converted into an
 * IEEE 802.15.4 data frame.
 *
 * The frame requests acknowledgment both:
 *
 * - in the MAC Frame Control field; and
 * - through the MRF24J40 TXNACKREQ control bit.
 *
 * The MRF24J40 hardware therefore performs ACK monitoring and retransmission
 * without application-level retry logic.
 *
 * The final transmission result is obtained asynchronously after TXNIF through
 * mrf24j40_get_tx_result().
 */
static void app_radio_process_tx_queue(void)
{
    osStatus_t queue_status;
    mrf24j40_status_t radio_status;
    mrf24j40_packet_t packet;
    uint32_t fault_flags;
    uint32_t queue_message;
    uint16_t destination;
    uint8_t data;

    if (app_radio_tx_busy == false)
    {
        queue_status = osMessageQueueGet(radioOutputQueueHandle,
                                         &queue_message,
                                         NULL,
                                         0u);

        if (queue_status == osOK)
        {
            app_radio_unpack_queue_message(queue_message,
                                           &destination,
                                           &data);

            app_radio_build_data_frame(&packet,
                                       destination,
                                       data);

            radio_status =
                mrf24j40_write_tx_normal_fifo(&packet, true);

            if (radio_status == MRF24J40_OK)
            {
                app_radio_tx_busy = true;

                printf("[APP_RADIO] TX dst=0x%04X data=0x%02X seq=%u\r\n",
                       destination,
                       data,
                       packet.frame[APP_RADIO_FRAME_SEQUENCE_INDEX]);
            }
            else
            {
                app_radio_tx_busy = false;

                fault_flags =
                    app_radio_status_to_fault(radio_status);

                if (fault_flags != 0u)
                {
                    (void)osEventFlagsSet(radioFaultHandle,
                                          fault_flags);
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

/**
 * @brief Build one minimal IEEE 802.15.4 data frame.
 *
 * @param[out] p_packet Destination packet container.
 * @param destination Destination short address.
 * @param data One-byte application payload.
 *
 * @details
 * The generated frame contains:
 *
 * - Frame Control Field.
 * - Sequence Number.
 * - Destination PAN ID.
 * - Destination Short Address.
 * - Source Short Address.
 * - One-byte application payload.
 *
 * Source PAN ID is omitted because PAN ID compression is enabled and both
 * source and destination belong to the same PAN.
 *
 * The ACK Request bit is always enabled because this application requires
 * confirmed communication between nodes.
 *
 * The FCS is not included because it is generated automatically by the
 * MRF24J40.
 */
static void app_radio_build_data_frame(mrf24j40_packet_t * p_packet,
                                       uint16_t destination,
                                       uint8_t data)
{
    if (p_packet != NULL)
    {
        p_packet->frame_length = APP_RADIO_FRAME_LENGTH;

        p_packet->frame[APP_RADIO_FRAME_FCF_L_INDEX] =
            APP_RADIO_FRAME_CONTROL_LSB;

        p_packet->frame[APP_RADIO_FRAME_FCF_H_INDEX] =
            APP_RADIO_FRAME_CONTROL_MSB;

        p_packet->frame[APP_RADIO_FRAME_SEQUENCE_INDEX] =
            app_radio_sequence_number;

        p_packet->frame[APP_RADIO_FRAME_DST_PAN_L_INDEX] =
            (uint8_t)(app_radio_cfg.pan_id & 0x00FFu);

        p_packet->frame[APP_RADIO_FRAME_DST_PAN_H_INDEX] =
            (uint8_t)((app_radio_cfg.pan_id >> 8u) & 0x00FFu);

        p_packet->frame[APP_RADIO_FRAME_DST_ADDR_L_INDEX] =
            (uint8_t)(destination & 0x00FFu);

        p_packet->frame[APP_RADIO_FRAME_DST_ADDR_H_INDEX] =
            (uint8_t)((destination >> 8u) & 0x00FFu);

        p_packet->frame[APP_RADIO_FRAME_SRC_ADDR_L_INDEX] =
            (uint8_t)(app_radio_cfg.short_address & 0x00FFu);

        p_packet->frame[APP_RADIO_FRAME_SRC_ADDR_H_INDEX] =
            (uint8_t)((app_radio_cfg.short_address >> 8u) & 0x00FFu);

        p_packet->frame[APP_RADIO_FRAME_PAYLOAD_INDEX] = data;

        p_packet->lqi = 0u;
        p_packet->rssi = 0u;

        app_radio_sequence_number++;
    }
}

/**
 * @brief Decode one application IEEE 802.15.4 data frame.
 *
 * @param p_packet Received packet.
 * @param[out] p_source Source short address.
 * @param[out] p_data One-byte application payload.
 *
 * @return
 * - true: Frame is valid for this application.
 * - false: Frame does not match the expected format.
 */
static bool app_radio_decode_data_frame(
    const mrf24j40_packet_t * p_packet,
    uint16_t * p_source,
    uint8_t * p_data)
{
    bool valid;
    uint16_t destination_pan;
    uint16_t destination;
    uint16_t source;

    valid = false;

    if ((p_packet != NULL) &&
        (p_source != NULL) &&
        (p_data != NULL))
    {
        if (p_packet->frame_length == APP_RADIO_FRAME_LENGTH)
        {
            if ((p_packet->frame[APP_RADIO_FRAME_FCF_L_INDEX] ==
                 APP_RADIO_FRAME_CONTROL_LSB) &&
                (p_packet->frame[APP_RADIO_FRAME_FCF_H_INDEX] ==
                 APP_RADIO_FRAME_CONTROL_MSB))
            {
                destination_pan =
                    (uint16_t)p_packet->
                        frame[APP_RADIO_FRAME_DST_PAN_L_INDEX];

                destination_pan |=
                    ((uint16_t)p_packet->
                        frame[APP_RADIO_FRAME_DST_PAN_H_INDEX] << 8u);

                destination =
                    (uint16_t)p_packet->
                        frame[APP_RADIO_FRAME_DST_ADDR_L_INDEX];

                destination |=
                    ((uint16_t)p_packet->
                        frame[APP_RADIO_FRAME_DST_ADDR_H_INDEX] << 8u);

                source =
                    (uint16_t)p_packet->
                        frame[APP_RADIO_FRAME_SRC_ADDR_L_INDEX];

                source |=
                    ((uint16_t)p_packet->
                        frame[APP_RADIO_FRAME_SRC_ADDR_H_INDEX] << 8u);

                if ((destination_pan == app_radio_cfg.pan_id) &&
                    (destination == app_radio_cfg.short_address) &&
                    (source <= APP_RADIO_LAST_DEVICE_ADDRESS) &&
                    (source != app_radio_cfg.short_address))
                {
                    if (app_radio_cfg.role ==
                        APP_RADIO_ROLE_PAN_COORDINATOR)
                    {
                        if ((source >=
                             APP_RADIO_FIRST_DEVICE_ADDRESS) &&
                            (source <=
                             APP_RADIO_LAST_DEVICE_ADDRESS))
                        {
                            *p_source = source;

                            *p_data =
                                p_packet->
                                    frame[APP_RADIO_FRAME_PAYLOAD_INDEX];

                            valid = true;
                        }
                    }
                    else
                    {
                        if (source ==
                            APP_RADIO_COORDINATOR_ADDRESS)
                        {
                            *p_source = source;

                            *p_data =
                                p_packet->
                                    frame[APP_RADIO_FRAME_PAYLOAD_INDEX];

                            valid = true;
                        }
                    }
                }
            }
        }
    }

    return valid;
}

/**
 * @brief Validate a transmission destination against the configured node role.
 *
 * @param destination Destination short address.
 *
 * @return
 * - true: Destination is valid.
 * - false: Destination is invalid for the local role.
 */
static bool app_radio_destination_is_valid(uint16_t destination)
{
    bool valid;

    if (app_radio_cfg.role == APP_RADIO_ROLE_PAN_COORDINATOR)
    {
        valid =
            ((destination >= APP_RADIO_FIRST_DEVICE_ADDRESS) &&
             (destination <= APP_RADIO_LAST_DEVICE_ADDRESS));
    }
    else
    {
        valid =
            (destination == APP_RADIO_COORDINATOR_ADDRESS);
    }

    return valid;
}

/**
 * @brief Encode one radio queue message into a 32-bit value.
 *
 * @param address Node short address.
 * @param data One-byte application payload.
 *
 * @return Encoded queue message.
 *
 * @details
 * Queue representation:
 *
 * Bits 23:8 = 16-bit node address.
 * Bits  7:0 = 8-bit application payload.
 *
 * Remaining bits are reserved and written as zero.
 */
static uint32_t app_radio_pack_queue_message(uint16_t address,
                                             uint8_t data)
{
    uint32_t message;

    message =
        ((uint32_t)address << APP_RADIO_QUEUE_ADDRESS_SHIFT);

    message |= (uint32_t)data;

    return message;
}

/**
 * @brief Decode one internal 32-bit radio queue message.
 *
 * @param message Encoded queue message.
 * @param[out] p_address Destination for the decoded short address.
 * @param[out] p_data Destination for the decoded payload.
 */
static void app_radio_unpack_queue_message(uint32_t message,
                                           uint16_t * p_address,
                                           uint8_t * p_data)
{
    if (p_address != NULL)
    {
        *p_address =
            (uint16_t)((message & APP_RADIO_QUEUE_ADDRESS_MASK) >>
                       APP_RADIO_QUEUE_ADDRESS_SHIFT);
    }

    if (p_data != NULL)
    {
        *p_data =
            (uint8_t)(message & APP_RADIO_QUEUE_DATA_MASK);
    }
}

/**
 * @brief Print one received MRF24J40 packet through the debug console.
 *
 * @param p_packet Packet to print.
 */
static void app_radio_print_packet(
    const mrf24j40_packet_t * p_packet)
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

/**
 * @brief Convert an MRF24J40 driver status into an application fault flag.
 *
 * @param status Driver status code.
 *
 * @return Corresponding APP_RADIO_FAULT_x flag, or zero when the condition is
 * not considered a fault.
 *
 * @details
 * This function maps errors returned directly by the MRF24J40 driver API.
 *
 * Transmission-level results such as missing acknowledgment or CSMA-CA failure
 * are not represented by mrf24j40_status_t and are therefore handled
 * separately from mrf24j40_tx_result_t inside app_radio_process_irq().
 */
static uint32_t app_radio_status_to_fault(
    mrf24j40_status_t status)
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
