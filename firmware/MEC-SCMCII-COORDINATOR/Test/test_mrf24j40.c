/****************************************************************************************
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
 ****************************************************************************************/

/**
 * @file    test_mrf24j40.c
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-09-26
 * @brief   Unit tests for the MRF24J40 radio transceiver driver.
 *
 * @details
 * These tests validate the public behavior of the MRF24J40 driver while
 * isolating the hardware-dependent port layer through CMock.
 *
 * The test suite covers:
 *
 * - Base device initialization.
 * - PAN coordinator and device configuration.
 * - PAN ID and short-address configuration.
 * - Interrupt pending and interrupt source processing.
 * - RX FIFO validation and packet extraction.
 * - TX Normal FIFO validation and loading.
 * - ACK-request configuration.
 * - TX result decoding.
 * - Port-layer error propagation.
 */

/* ============================= Includes ================================== */

#include "unity.h"
#include "Mockmrf24j40_port.h"
#include "mrf24j40.h"
#include "mrf24j40_reg.h"

/* ============================ Local Macros ================================ */

#define TEST_MRF24J40_PAN_ID              ((uint16_t)0x1234u)
#define TEST_MRF24J40_SHORT_ADDRESS       ((uint16_t)0x0001u)

#define TEST_MRF24J40_FRAME_LENGTH        ((uint8_t)3u)
#define TEST_MRF24J40_RX_FIFO_LENGTH      ((uint8_t)5u)

#define TEST_MRF24J40_FRAME_BYTE_0        ((uint8_t)0xAAu)
#define TEST_MRF24J40_FRAME_BYTE_1        ((uint8_t)0x55u)
#define TEST_MRF24J40_FRAME_BYTE_2        ((uint8_t)0x42u)

#define TEST_MRF24J40_LQI                 ((uint8_t)0xA5u)
#define TEST_MRF24J40_RSSI                ((uint8_t)0x5Au)

/* ======================= Local (static) Data ============================= */

static uint8_t test_read_short_intstat;
static uint8_t test_read_short_txstat;
static uint8_t test_read_short_rxmcr;
static uint8_t test_read_short_txmcr;
static uint8_t test_read_short_bbreg1;

static uint8_t test_rx_fifo_length;
static uint8_t test_rx_fifo_frame[TEST_MRF24J40_FRAME_LENGTH];

/* ========================== Private Prototypes =========================== */

static mrf24j40_port_status_t test_mrf24j40_read_short_callback(
    uint8_t address,
    uint8_t * data,
    int cmock_num_calls);

static mrf24j40_port_status_t test_mrf24j40_read_long_rx_callback(
    uint16_t address,
    uint8_t * data,
    int cmock_num_calls);

static void test_mrf24j40_allow_initialization(void);

static void test_mrf24j40_initialize_driver(void);

static void test_mrf24j40_latch_rx_interrupt(void);

static void test_mrf24j40_latch_tx_interrupt(void);

/* ======================== Test Setup / Teardown ========================== */

void setUp(void)
{
    test_read_short_intstat = 0u;
    test_read_short_txstat = 0u;
    test_read_short_rxmcr = 0u;
    test_read_short_txmcr = 0u;
    test_read_short_bbreg1 = 0u;

    test_rx_fifo_length = TEST_MRF24J40_RX_FIFO_LENGTH;

    test_rx_fifo_frame[0] = TEST_MRF24J40_FRAME_BYTE_0;
    test_rx_fifo_frame[1] = TEST_MRF24J40_FRAME_BYTE_1;
    test_rx_fifo_frame[2] = TEST_MRF24J40_FRAME_BYTE_2;

    /*
     * Reinitialize the driver before every test so that the static runtime
     * context starts from a known state.
     */
    test_mrf24j40_initialize_driver();

    /*
     * Clear expectations installed during initialization. The driver context
     * remains initialized, while each test starts with a clean mock state.
     */
    Mockmrf24j40_port_Verify();
    Mockmrf24j40_port_Destroy();
    Mockmrf24j40_port_Init();
}

void tearDown(void)
{
}

/* ============================= Test Cases ================================ */

/**
 * @brief Verify successful execution of the complete base initialization.
 */
void test_mrf24j40_init_should_complete_base_configuration(void)
{
    mrf24j40_status_t status;

    test_mrf24j40_allow_initialization();

    status = mrf24j40_init();

    TEST_ASSERT_EQUAL(MRF24J40_OK, status);
}

/**
 * @brief Verify propagation of a port error during initialization.
 */
void test_mrf24j40_init_should_propagate_port_error(void)
{
    mrf24j40_status_t status;

    mrf24j40_port_write_short_ExpectAndReturn(
        SOFTRST,
        (uint8_t)(SOFTRST_RSTPWR |
                  SOFTRST_RSTBB |
                  SOFTRST_RSTMAC),
        MRF24J40_PORT_E_HW);

    status = mrf24j40_init();

    TEST_ASSERT_EQUAL(MRF24J40_E_HW, status);
}

/**
 * @brief Verify PAN ID byte ordering.
 */
void test_mrf24j40_set_pan_id_should_write_lsb_and_msb(void)
{
    mrf24j40_status_t status;

    mrf24j40_port_write_short_ExpectAndReturn(
        PANIDL,
        0x34u,
        MRF24J40_PORT_OK);

    mrf24j40_port_write_short_ExpectAndReturn(
        PANIDH,
        0x12u,
        MRF24J40_PORT_OK);

    status = mrf24j40_set_pan_id(TEST_MRF24J40_PAN_ID);

    TEST_ASSERT_EQUAL(MRF24J40_OK, status);
}

/**
 * @brief Verify error propagation while writing the PAN ID.
 */
void test_mrf24j40_set_pan_id_should_propagate_port_error(void)
{
    mrf24j40_status_t status;

    mrf24j40_port_write_short_ExpectAndReturn(
        PANIDL,
        0x34u,
        MRF24J40_PORT_E_TIMEOUT);

    status = mrf24j40_set_pan_id(TEST_MRF24J40_PAN_ID);

    TEST_ASSERT_EQUAL(MRF24J40_E_TIMEOUT, status);
}

/**
 * @brief Verify short-address byte ordering.
 */
void test_mrf24j40_set_short_address_should_write_lsb_and_msb(void)
{
    mrf24j40_status_t status;

    mrf24j40_port_write_short_ExpectAndReturn(
        SADRL,
        0x01u,
        MRF24J40_PORT_OK);

    mrf24j40_port_write_short_ExpectAndReturn(
        SADRH,
        0x00u,
        MRF24J40_PORT_OK);

    status = mrf24j40_set_short_address(TEST_MRF24J40_SHORT_ADDRESS);

    TEST_ASSERT_EQUAL(MRF24J40_OK, status);
}

/**
 * @brief Verify error propagation while writing the short address.
 */
void test_mrf24j40_set_short_address_should_propagate_port_error(void)
{
    mrf24j40_status_t status;

    mrf24j40_port_write_short_ExpectAndReturn(
        SADRL,
        0x01u,
        MRF24J40_PORT_E_HW);

    status = mrf24j40_set_short_address(TEST_MRF24J40_SHORT_ADDRESS);

    TEST_ASSERT_EQUAL(MRF24J40_E_HW, status);
}

/**
 * @brief Verify nonbeacon PAN coordinator configuration.
 */
void test_mrf24j40_configure_pan_coordinator_should_set_coordinator_bit(void)
{
    mrf24j40_status_t status;

    test_read_short_txmcr = 0x20u;
    test_read_short_rxmcr = 0x00u;

    mrf24j40_port_write_short_IgnoreAndReturn(MRF24J40_PORT_OK);
    mrf24j40_port_read_short_Stub(test_mrf24j40_read_short_callback);

    status = mrf24j40_configure_nonbeacon_pan_coordinator();

    TEST_ASSERT_EQUAL(MRF24J40_OK, status);
}

/**
 * @brief Verify nonbeacon device configuration.
 */
void test_mrf24j40_configure_device_should_clear_coordinator_bit(void)
{
    mrf24j40_status_t status;

    test_read_short_txmcr = 0x20u;
    test_read_short_rxmcr = 0x08u;

    mrf24j40_port_write_short_IgnoreAndReturn(MRF24J40_PORT_OK);
    mrf24j40_port_read_short_Stub(test_mrf24j40_read_short_callback);

    status = mrf24j40_configure_nonbeacon_device();

    TEST_ASSERT_EQUAL(MRF24J40_OK, status);
}

/**
 * @brief Verify that interrupt processing is rejected without a pending event.
 */
void test_mrf24j40_update_interrupt_flags_should_return_state_without_pending_interrupt(void)
{
    mrf24j40_status_t status;

    status = mrf24j40_update_interrupt_flags();

    TEST_ASSERT_EQUAL(MRF24J40_E_STATE, status);
}

/**
 * @brief Verify processing of an RX interrupt source.
 */
void test_mrf24j40_update_interrupt_flags_should_latch_rx_interrupt(void)
{
    mrf24j40_status_t status;
    mrf24j40_packet_t packet;

    test_read_short_intstat = INTSTAT_RXIF;

    mrf24j40_set_interrupt_pending();

    mrf24j40_port_read_short_Stub(test_mrf24j40_read_short_callback);

    status = mrf24j40_update_interrupt_flags();

    TEST_ASSERT_EQUAL(MRF24J40_OK, status);

    /*
     * A latched RX event must allow the RX FIFO read path to progress beyond
     * the initial MRF24J40_E_NO_RX_PACKET check.
     *
     * Force the first BBREG1 access to fail so the test does not need to
     * emulate the complete FIFO here.
     */
    mrf24j40_port_read_short_IgnoreAndReturn(MRF24J40_PORT_E_HW);

    status = mrf24j40_read_rx_fifo(&packet);

    TEST_ASSERT_EQUAL(MRF24J40_E_HW, status);
}

/**
 * @brief Verify restoration of the pending state after INTSTAT read failure.
 */
void test_mrf24j40_update_interrupt_flags_should_restore_pending_on_read_error(void)
{
    mrf24j40_status_t status;

    mrf24j40_set_interrupt_pending();

    /*
     * Force the first INTSTAT read to fail. The driver must restore its
     * internal interrupt-pending indication.
     */
    mrf24j40_port_read_short_IgnoreAndReturn(MRF24J40_PORT_E_TIMEOUT);

    status = mrf24j40_update_interrupt_flags();

    TEST_ASSERT_EQUAL(MRF24J40_E_TIMEOUT, status);

    /*
     * Do not call mrf24j40_set_interrupt_pending() again. A second successful
     * processing attempt proves that the driver restored the pending state.
     */
    test_read_short_intstat = 0u;

    mrf24j40_port_read_short_Stub(test_mrf24j40_read_short_callback);

    status = mrf24j40_update_interrupt_flags();

    TEST_ASSERT_EQUAL(MRF24J40_OK, status);
}

/**
 * @brief Verify NULL pointer validation in RX FIFO access.
 */
void test_mrf24j40_read_rx_fifo_should_return_null_for_null_packet(void)
{
    mrf24j40_status_t status;

    status = mrf24j40_read_rx_fifo(NULL);

    TEST_ASSERT_EQUAL(MRF24J40_E_NULL, status);
}

/**
 * @brief Verify RX FIFO access without a previously latched RX interrupt.
 */
void test_mrf24j40_read_rx_fifo_should_return_no_packet_without_rx_interrupt(void)
{
    mrf24j40_status_t status;
    mrf24j40_packet_t packet;

    status = mrf24j40_read_rx_fifo(&packet);

    TEST_ASSERT_EQUAL(MRF24J40_E_NO_RX_PACKET, status);
}

/**
 * @brief Verify extraction of a valid packet from the RX FIFO.
 */
void test_mrf24j40_read_rx_fifo_should_decode_valid_packet(void)
{
    mrf24j40_status_t status;
    mrf24j40_packet_t packet;

    test_mrf24j40_latch_rx_interrupt();

    test_read_short_bbreg1 = 0x00u;

    mrf24j40_port_read_short_Stub(test_mrf24j40_read_short_callback);
    mrf24j40_port_write_short_IgnoreAndReturn(MRF24J40_PORT_OK);
    mrf24j40_port_read_long_Stub(test_mrf24j40_read_long_rx_callback);

    status = mrf24j40_read_rx_fifo(&packet);

    TEST_ASSERT_EQUAL(MRF24J40_OK, status);
    TEST_ASSERT_EQUAL_UINT8(TEST_MRF24J40_FRAME_LENGTH,
                            packet.frame_length);

    TEST_ASSERT_EQUAL_UINT8(TEST_MRF24J40_FRAME_BYTE_0,
                            packet.frame[0]);
    TEST_ASSERT_EQUAL_UINT8(TEST_MRF24J40_FRAME_BYTE_1,
                            packet.frame[1]);
    TEST_ASSERT_EQUAL_UINT8(TEST_MRF24J40_FRAME_BYTE_2,
                            packet.frame[2]);

    TEST_ASSERT_EQUAL_UINT8(TEST_MRF24J40_LQI, packet.lqi);
    TEST_ASSERT_EQUAL_UINT8(TEST_MRF24J40_RSSI, packet.rssi);
}

/**
 * @brief Verify rejection of an RX FIFO frame shorter than the FCS.
 */
void test_mrf24j40_read_rx_fifo_should_reject_invalid_frame_length(void)
{
    mrf24j40_status_t status;
    mrf24j40_packet_t packet;

    test_mrf24j40_latch_rx_interrupt();

    test_read_short_bbreg1 = 0x00u;
    test_rx_fifo_length = 1u;

    mrf24j40_port_read_short_Stub(test_mrf24j40_read_short_callback);
    mrf24j40_port_write_short_IgnoreAndReturn(MRF24J40_PORT_OK);
    mrf24j40_port_read_long_Stub(test_mrf24j40_read_long_rx_callback);

    status = mrf24j40_read_rx_fifo(&packet);

    TEST_ASSERT_EQUAL(MRF24J40_E_FRAME, status);
}

/**
 * @brief Verify NULL pointer validation in TX Normal FIFO access.
 */
void test_mrf24j40_write_tx_normal_fifo_should_return_null_for_null_packet(void)
{
    mrf24j40_status_t status;

    status = mrf24j40_write_tx_normal_fifo(NULL, false);

    TEST_ASSERT_EQUAL(MRF24J40_E_NULL, status);
}

/**
 * @brief Verify rejection of a zero-length TX frame.
 */
void test_mrf24j40_write_tx_normal_fifo_should_reject_zero_length_frame(void)
{
    mrf24j40_status_t status;
    mrf24j40_packet_t packet = { 0 };

    packet.frame_length = 0u;

    status = mrf24j40_write_tx_normal_fifo(&packet, false);

    TEST_ASSERT_EQUAL(MRF24J40_E_PARAM, status);
}

/**
 * @brief Verify loading and transmission of a valid frame without ACK request.
 */
void test_mrf24j40_write_tx_normal_fifo_should_write_valid_frame_without_ack(void)
{
    mrf24j40_status_t status;
    mrf24j40_packet_t packet = { 0 };

    packet.frame_length = TEST_MRF24J40_FRAME_LENGTH;
    packet.frame[0] = TEST_MRF24J40_FRAME_BYTE_0;
    packet.frame[1] = TEST_MRF24J40_FRAME_BYTE_1;
    packet.frame[2] = TEST_MRF24J40_FRAME_BYTE_2;

    mrf24j40_port_write_long_ExpectAndReturn(
        TX_NORMAL_FIFO,
        0x00u,
        MRF24J40_PORT_OK);

    mrf24j40_port_write_long_ExpectAndReturn(
        (uint16_t)(TX_NORMAL_FIFO + 1u),
        TEST_MRF24J40_FRAME_LENGTH,
        MRF24J40_PORT_OK);

    mrf24j40_port_write_long_ExpectAndReturn(
        (uint16_t)(TX_NORMAL_FIFO + 2u),
        TEST_MRF24J40_FRAME_BYTE_0,
        MRF24J40_PORT_OK);

    mrf24j40_port_write_long_ExpectAndReturn(
        (uint16_t)(TX_NORMAL_FIFO + 3u),
        TEST_MRF24J40_FRAME_BYTE_1,
        MRF24J40_PORT_OK);

    mrf24j40_port_write_long_ExpectAndReturn(
        (uint16_t)(TX_NORMAL_FIFO + 4u),
        TEST_MRF24J40_FRAME_BYTE_2,
        MRF24J40_PORT_OK);

    mrf24j40_port_write_short_ExpectAndReturn(
        TXNCON,
        TXNCON_TXNTRIG,
        MRF24J40_PORT_OK);

    status = mrf24j40_write_tx_normal_fifo(&packet, false);

    TEST_ASSERT_EQUAL(MRF24J40_OK, status);
}

/**
 * @brief Verify ACK request configuration for a TX Normal FIFO transaction.
 */
void test_mrf24j40_write_tx_normal_fifo_should_set_ack_request(void)
{
    mrf24j40_status_t status;
    mrf24j40_packet_t packet = { 0 };

    packet.frame_length = 1u;
    packet.frame[0] = TEST_MRF24J40_FRAME_BYTE_0;

    mrf24j40_port_write_long_IgnoreAndReturn(MRF24J40_PORT_OK);

    mrf24j40_port_write_short_ExpectAndReturn(
        TXNCON,
        (uint8_t)(TXNCON_TXNACKREQ | TXNCON_TXNTRIG),
        MRF24J40_PORT_OK);

    status = mrf24j40_write_tx_normal_fifo(&packet, true);

    TEST_ASSERT_EQUAL(MRF24J40_OK, status);
}

/**
 * @brief Verify default TX result before a TX completion interrupt.
 */
void test_mrf24j40_get_tx_result_should_report_incomplete_before_tx_interrupt(void)
{
    mrf24j40_status_t status;
    mrf24j40_tx_result_t result;

    status = mrf24j40_get_tx_result(&result);

    TEST_ASSERT_EQUAL(MRF24J40_OK, status);

    TEST_ASSERT_FALSE(result.complete);
    TEST_ASSERT_FALSE(result.success);
    TEST_ASSERT_FALSE(result.ack_requested);
    TEST_ASSERT_FALSE(result.ack_received);
    TEST_ASSERT_FALSE(result.retry_limit_reached);
    TEST_ASSERT_FALSE(result.cca_failed);
    TEST_ASSERT_EQUAL_UINT8(0u, result.retries);
}

/**
 * @brief Verify NULL pointer validation for TX result retrieval.
 */
void test_mrf24j40_get_tx_result_should_return_null_for_null_result(void)
{
    mrf24j40_status_t status;

    status = mrf24j40_get_tx_result(NULL);

    TEST_ASSERT_EQUAL(MRF24J40_E_NULL, status);
}

/**
 * @brief Verify successful ACK-requested transmission result decoding.
 */
void test_mrf24j40_get_tx_result_should_decode_success_with_ack(void)
{
    mrf24j40_status_t status;
    mrf24j40_packet_t packet = { 0 };
    mrf24j40_tx_result_t result;

    packet.frame_length = 1u;
    packet.frame[0] = TEST_MRF24J40_FRAME_BYTE_0;

    mrf24j40_port_write_long_IgnoreAndReturn(MRF24J40_PORT_OK);
    mrf24j40_port_write_short_IgnoreAndReturn(MRF24J40_PORT_OK);

    status = mrf24j40_write_tx_normal_fifo(&packet, true);

    TEST_ASSERT_EQUAL(MRF24J40_OK, status);

    test_mrf24j40_latch_tx_interrupt();

    test_read_short_txstat = 0x40u;

    mrf24j40_port_read_short_Stub(test_mrf24j40_read_short_callback);

    status = mrf24j40_get_tx_result(&result);

    TEST_ASSERT_EQUAL(MRF24J40_OK, status);

    TEST_ASSERT_TRUE(result.complete);
    TEST_ASSERT_TRUE(result.success);
    TEST_ASSERT_TRUE(result.ack_requested);
    TEST_ASSERT_TRUE(result.ack_received);
    TEST_ASSERT_FALSE(result.retry_limit_reached);
    TEST_ASSERT_FALSE(result.cca_failed);
    TEST_ASSERT_EQUAL_UINT8(1u, result.retries);
}

/**
 * @brief Verify TX result decoding when the retry limit is reached.
 */
void test_mrf24j40_get_tx_result_should_detect_retry_limit(void)
{
    mrf24j40_status_t status;
    mrf24j40_packet_t packet = { 0 };
    mrf24j40_tx_result_t result;

    packet.frame_length = 1u;
    packet.frame[0] = TEST_MRF24J40_FRAME_BYTE_0;

    mrf24j40_port_write_long_IgnoreAndReturn(MRF24J40_PORT_OK);
    mrf24j40_port_write_short_IgnoreAndReturn(MRF24J40_PORT_OK);

    status = mrf24j40_write_tx_normal_fifo(&packet, true);

    TEST_ASSERT_EQUAL(MRF24J40_OK, status);

    test_mrf24j40_latch_tx_interrupt();

    /*
     * TXNRETRY = 3
     * CCAFAIL  = 0
     * TXNSTAT  = 1
     */
    test_read_short_txstat = 0xC1u;

    mrf24j40_port_read_short_Stub(test_mrf24j40_read_short_callback);

    status = mrf24j40_get_tx_result(&result);

    TEST_ASSERT_EQUAL(MRF24J40_OK, status);

    TEST_ASSERT_TRUE(result.complete);
    TEST_ASSERT_FALSE(result.success);
    TEST_ASSERT_TRUE(result.ack_requested);
    TEST_ASSERT_FALSE(result.ack_received);
    TEST_ASSERT_TRUE(result.retry_limit_reached);
    TEST_ASSERT_FALSE(result.cca_failed);
    TEST_ASSERT_EQUAL_UINT8(3u, result.retries);
}

/**
 * @brief Verify TX result decoding for a CCA failure.
 */
void test_mrf24j40_get_tx_result_should_detect_cca_failure(void)
{
    mrf24j40_status_t status;
    mrf24j40_packet_t packet = { 0 };
    mrf24j40_tx_result_t result;

    packet.frame_length = 1u;
    packet.frame[0] = TEST_MRF24J40_FRAME_BYTE_0;

    mrf24j40_port_write_long_IgnoreAndReturn(MRF24J40_PORT_OK);
    mrf24j40_port_write_short_IgnoreAndReturn(MRF24J40_PORT_OK);

    status = mrf24j40_write_tx_normal_fifo(&packet, true);

    TEST_ASSERT_EQUAL(MRF24J40_OK, status);

    test_mrf24j40_latch_tx_interrupt();

    /*
     * CCAFAIL = 1
     * TXNSTAT = 1
     */
    test_read_short_txstat = 0x21u;

    mrf24j40_port_read_short_Stub(test_mrf24j40_read_short_callback);

    status = mrf24j40_get_tx_result(&result);

    TEST_ASSERT_EQUAL(MRF24J40_OK, status);

    TEST_ASSERT_TRUE(result.complete);
    TEST_ASSERT_FALSE(result.success);
    TEST_ASSERT_TRUE(result.ack_requested);
    TEST_ASSERT_FALSE(result.ack_received);
    TEST_ASSERT_FALSE(result.retry_limit_reached);
    TEST_ASSERT_TRUE(result.cca_failed);
}

/* ======================== Private Function Definitions =================== */

/**
 * @brief Simulate reads from short MRF24J40 registers.
 */
static mrf24j40_port_status_t test_mrf24j40_read_short_callback(
    uint8_t address,
    uint8_t * data,
    int cmock_num_calls)
{
    (void)cmock_num_calls;

    if (data == NULL)
    {
        return MRF24J40_PORT_E_NULL;
    }

    switch (address)
    {
        case INTSTAT:
            *data = test_read_short_intstat;
            break;

        case TXSTAT:
            *data = test_read_short_txstat;
            break;

        case RXMCR:
            *data = test_read_short_rxmcr;
            break;

        case TXMCR:
            *data = test_read_short_txmcr;
            break;

        case BBREG1:
            *data = test_read_short_bbreg1;
            break;

        default:
            *data = 0u;
            break;
    }

    return MRF24J40_PORT_OK;
}

/**
 * @brief Simulate the RX FIFO and associated metadata.
 */
static mrf24j40_port_status_t test_mrf24j40_read_long_rx_callback(
    uint16_t address,
    uint8_t * data,
    int cmock_num_calls)
{
    (void)cmock_num_calls;

    if (data == NULL)
    {
        return MRF24J40_PORT_E_NULL;
    }

    if (address == RX_FIFO)
    {
        *data = test_rx_fifo_length;
    }
    else if (address == (uint16_t)(RX_FIFO + 1u))
    {
        *data = test_rx_fifo_frame[0];
    }
    else if (address == (uint16_t)(RX_FIFO + 2u))
    {
        *data = test_rx_fifo_frame[1];
    }
    else if (address == (uint16_t)(RX_FIFO + 3u))
    {
        *data = test_rx_fifo_frame[2];
    }
    else if (address == (uint16_t)(RX_FIFO + 6u))
    {
        *data = TEST_MRF24J40_LQI;
    }
    else if (address == (uint16_t)(RX_FIFO + 7u))
    {
        *data = TEST_MRF24J40_RSSI;
    }
    else
    {
        *data = 0u;
    }

    return MRF24J40_PORT_OK;
}

/**
 * @brief Configure mocks required for successful driver initialization.
 */
static void test_mrf24j40_allow_initialization(void)
{
    mrf24j40_port_write_short_IgnoreAndReturn(MRF24J40_PORT_OK);
    mrf24j40_port_write_long_IgnoreAndReturn(MRF24J40_PORT_OK);
    mrf24j40_port_delay_ms_Ignore();
}

/**
 * @brief Initialize the driver and reset its internal runtime state.
 */
static void test_mrf24j40_initialize_driver(void)
{
    mrf24j40_status_t status;

    test_mrf24j40_allow_initialization();

    status = mrf24j40_init();

    TEST_ASSERT_EQUAL(MRF24J40_OK, status);
}

/**
 * @brief Generate and process an RX interrupt.
 */
static void test_mrf24j40_latch_rx_interrupt(void)
{
    mrf24j40_status_t status;

    test_read_short_intstat = INTSTAT_RXIF;

    mrf24j40_set_interrupt_pending();

    mrf24j40_port_read_short_Stub(test_mrf24j40_read_short_callback);

    status = mrf24j40_update_interrupt_flags();

    TEST_ASSERT_EQUAL(MRF24J40_OK, status);
}

/**
 * @brief Generate and process a TX Normal FIFO completion interrupt.
 */
static void test_mrf24j40_latch_tx_interrupt(void)
{
    mrf24j40_status_t status;

    test_read_short_intstat = INTSTAT_TXNIF;

    mrf24j40_set_interrupt_pending();

    mrf24j40_port_read_short_Stub(test_mrf24j40_read_short_callback);

    status = mrf24j40_update_interrupt_flags();

    TEST_ASSERT_EQUAL(MRF24J40_OK, status);
}
