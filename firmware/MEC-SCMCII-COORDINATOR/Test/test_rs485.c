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
 * @file    test_rs485.c
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-09-26
 * @brief   Unit tests for the interrupt-driven RS485 driver.
 *
 * @details
 * This test module validates the public interface and state transitions of the
 * RS485 driver using Ceedling, Unity, and CMock.
 *
 * Hardware-dependent operations provided by the RS485 port layer are replaced
 * by mocks so that initialization, transmission, reception, half-duplex state
 * handling, completion notifications, and abort operations can be verified
 * independently from the target hardware.
 *
 * @ingroup test_rs485
 * @{
 */

/* ============================= Includes ================================== */

#include "unity.h"
#include "Mockrs485_port.h"

#include "rs485.h"

#include <stddef.h>
#include <stdint.h>

/* ============================ Local Macros =============================== */

#define TEST_RS485_TX_DATA                 ((uint8_t)0xA5u)
#define TEST_RS485_RX_DATA                 ((uint8_t)0x5Au)
#define TEST_RS485_UART_ERROR              ((uint32_t)0x00000008u)

/* ========================== Private Prototypes =========================== */

static rs485_port_status_t test_rs485_port_init_callback(
    rs485_port_handle_t * handle,
    int cmock_num_calls);

static rs485_port_status_t test_rs485_port_deinit_callback(
    rs485_port_handle_t * handle,
    int cmock_num_calls);

static rs485_port_status_t test_rs485_port_transmit_callback(
    uint8_t * p_data,
    uint16_t size,
    int cmock_num_calls);

static rs485_port_status_t test_rs485_port_receive_callback(
    uint8_t * p_data,
    uint16_t size,
    int cmock_num_calls);

static rs485_port_status_t test_rs485_port_receive_data_callback(
    uint8_t * p_data,
    uint16_t size,
    int cmock_num_calls);

static void test_rs485_initialize_driver(void);
static void test_rs485_deinitialize_driver(void);

/* ======================= Test Setup and Teardown ========================== */

/**
 * @brief Prepare the test environment before each test case.
 */
void setUp(void)
{
    /*
     * No initialization is performed here because initialization state itself
     * is part of the behavior under test.
     */
}

/**
 * @brief Restore the test environment after each test case.
 */
void tearDown(void)
{
    /*
     * Each test that initializes the driver explicitly deinitializes it before
     * returning so that the static driver context is clean for the next test.
     */
}

/* ============================= Test Cases ================================ */

/**
 * @brief Verify rejection of a null handle during initialization.
 *
 * @test
 * - Handle: NULL.
 * - Expected result: RS485_E_NULL.
 */
void test_rs485_init_should_return_null_when_handle_is_null(void)
{
    rs485_status_t status;

    status = rs485_init(NULL);

    TEST_ASSERT_EQUAL(RS485_E_NULL, status);
}

/**
 * @brief Verify successful driver initialization.
 *
 * @test
 * - Port initialization succeeds.
 * - Driver explicitly selects receive mode.
 * - Expected result: RS485_OK.
 */
void test_rs485_init_should_initialize_driver_and_select_rx_mode(void)
{
    rs485_handle_t handle;
    rs485_status_t status;

    rs485_port_init_Stub(test_rs485_port_init_callback);
    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_RX,
        RS485_PORT_OK);

    status = rs485_init(&handle);

    TEST_ASSERT_EQUAL(RS485_OK, status);

    test_rs485_deinitialize_driver();
}

/**
 * @brief Verify that a second initialization is rejected.
 *
 * @test
 * - Driver is initialized.
 * - rs485_init() is called again.
 * - Expected result: RS485_E_STATE.
 */
void test_rs485_init_should_reject_second_initialization(void)
{
    rs485_handle_t handle;
    rs485_status_t status;

    test_rs485_initialize_driver();

    status = rs485_init(&handle);

    TEST_ASSERT_EQUAL(RS485_E_STATE, status);

    test_rs485_deinitialize_driver();
}

/**
 * @brief Verify transmission is rejected before initialization.
 *
 * @test
 * - Driver is not initialized.
 * - Expected result: RS485_E_STATE.
 */
void test_rs485_send_should_return_state_error_when_not_initialized(void)
{
    rs485_status_t status;

    status = rs485_send(TEST_RS485_TX_DATA);

    TEST_ASSERT_EQUAL(RS485_E_STATE, status);
}

/**
 * @brief Verify successful start of an interrupt-driven transmission.
 *
 * @test
 * - Driver is initialized.
 * - Requested byte: 0xA5.
 * - Driver selects transmit mode.
 * - Port receives one byte containing 0xA5.
 * - Expected result: RS485_OK.
 */
void test_rs485_send_should_start_single_byte_transmission(void)
{
    rs485_status_t status;

    test_rs485_initialize_driver();

    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_TX,
        RS485_PORT_OK);

    rs485_port_transmit_it_Stub(
        test_rs485_port_transmit_callback);

    status = rs485_send(TEST_RS485_TX_DATA);

    TEST_ASSERT_EQUAL(RS485_OK, status);

    /*
     * Complete the asynchronous transmission before deinitialization.
     */
    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_RX,
        RS485_PORT_OK);

    rs485_port_tx_complete_callback();

    test_rs485_deinitialize_driver();
}

/**
 * @brief Verify that a second transmission is rejected while TX is active.
 *
 * @test
 * - First transmission starts successfully.
 * - Second transmission is requested before completion.
 * - Expected result: RS485_E_STATE.
 */
void test_rs485_send_should_reject_transmission_while_tx_is_active(void)
{
    rs485_status_t status;

    test_rs485_initialize_driver();

    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_TX,
        RS485_PORT_OK);

    rs485_port_transmit_it_Stub(
        test_rs485_port_transmit_callback);

    status = rs485_send(TEST_RS485_TX_DATA);
    TEST_ASSERT_EQUAL(RS485_OK, status);

    status = rs485_send(0x11u);
    TEST_ASSERT_EQUAL(RS485_E_STATE, status);

    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_RX,
        RS485_PORT_OK);

    rs485_port_tx_complete_callback();

    test_rs485_deinitialize_driver();
}

/**
 * @brief Verify that TX completion releases the driver for another operation.
 *
 * @test
 * - A transmission is started.
 * - TX-complete notification is generated.
 * - Driver returns the transceiver to receive mode.
 * - A new transmission can subsequently be started.
 */
void test_rs485_tx_complete_should_release_transmission_state(void)
{
    rs485_status_t status;

    test_rs485_initialize_driver();

    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_TX,
        RS485_PORT_OK);

    rs485_port_transmit_it_Stub(
        test_rs485_port_transmit_callback);

    status = rs485_send(TEST_RS485_TX_DATA);
    TEST_ASSERT_EQUAL(RS485_OK, status);

    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_RX,
        RS485_PORT_OK);

    rs485_port_tx_complete_callback();

    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_TX,
        RS485_PORT_OK);

    rs485_port_transmit_it_Stub(
        test_rs485_port_transmit_callback);

    status = rs485_send(0x11u);
    TEST_ASSERT_EQUAL(RS485_OK, status);

    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_RX,
        RS485_PORT_OK);

    rs485_port_tx_complete_callback();

    test_rs485_deinitialize_driver();
}

/**
 * @brief Verify reception cannot be started before initialization.
 *
 * @test
 * - Driver is not initialized.
 * - Expected result: RS485_E_STATE.
 */
void test_rs485_receive_start_should_return_state_error_when_not_initialized(void)
{
    rs485_status_t status;

    status = rs485_receive_start();

    TEST_ASSERT_EQUAL(RS485_E_STATE, status);
}

/**
 * @brief Verify successful start of an interrupt-driven reception.
 *
 * @test
 * - Driver is initialized.
 * - Receive mode is selected.
 * - One-byte interrupt reception is requested.
 * - Expected result: RS485_OK.
 */
void test_rs485_receive_start_should_start_single_byte_reception(void)
{
    rs485_status_t status;

    test_rs485_initialize_driver();

    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_RX,
        RS485_PORT_OK);

    rs485_port_receive_it_Stub(
        test_rs485_port_receive_callback);

    status = rs485_receive_start();

    TEST_ASSERT_EQUAL(RS485_OK, status);

    /*
     * Abort the still-active reception before deinitialization.
     */
    rs485_port_abort_receive_ExpectAndReturn(RS485_PORT_OK);
    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_RX,
        RS485_PORT_OK);

    status = rs485_receive_abort();

    TEST_ASSERT_EQUAL(RS485_OK, status);

    test_rs485_deinitialize_driver();
}

/**
 * @brief Verify a second reception cannot start while RX is active.
 *
 * @test
 * - Reception is already active.
 * - A second reception is requested.
 * - Expected result: RS485_E_STATE.
 */
void test_rs485_receive_start_should_reject_reception_while_rx_is_active(void)
{
    rs485_status_t status;

    test_rs485_initialize_driver();

    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_RX,
        RS485_PORT_OK);

    rs485_port_receive_it_Stub(
        test_rs485_port_receive_callback);

    status = rs485_receive_start();
    TEST_ASSERT_EQUAL(RS485_OK, status);

    status = rs485_receive_start();
    TEST_ASSERT_EQUAL(RS485_E_STATE, status);

    rs485_port_abort_receive_ExpectAndReturn(RS485_PORT_OK);
    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_RX,
        RS485_PORT_OK);

    status = rs485_receive_abort();

    TEST_ASSERT_EQUAL(RS485_OK, status);

    test_rs485_deinitialize_driver();
}

/**
 * @brief Verify that transmission is rejected while reception is active.
 *
 * @test
 * - Reception is active.
 * - Transmission is requested.
 * - Expected result: RS485_E_STATE.
 */
void test_rs485_send_should_reject_transmission_while_rx_is_active(void)
{
    rs485_status_t status;

    test_rs485_initialize_driver();

    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_RX,
        RS485_PORT_OK);

    rs485_port_receive_it_Stub(
        test_rs485_port_receive_callback);

    status = rs485_receive_start();
    TEST_ASSERT_EQUAL(RS485_OK, status);

    status = rs485_send(TEST_RS485_TX_DATA);
    TEST_ASSERT_EQUAL(RS485_E_STATE, status);

    rs485_port_abort_receive_ExpectAndReturn(RS485_PORT_OK);
    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_RX,
        RS485_PORT_OK);

    status = rs485_receive_abort();

    TEST_ASSERT_EQUAL(RS485_OK, status);

    test_rs485_deinitialize_driver();
}

/**
 * @brief Verify rejection of a null receive-data pointer.
 *
 * @test
 * - Output pointer: NULL.
 * - Expected result: RS485_E_NULL.
 */
void test_rs485_receive_should_return_null_when_data_pointer_is_null(void)
{
    rs485_status_t status;

    test_rs485_initialize_driver();

    status = rs485_receive(NULL);

    TEST_ASSERT_EQUAL(RS485_E_NULL, status);

    test_rs485_deinitialize_driver();
}

/**
 * @brief Verify data cannot be consumed before reception completes.
 *
 * @test
 * - Driver is initialized.
 * - No completed reception is available.
 * - Expected result: RS485_E_STATE.
 */
void test_rs485_receive_should_return_state_error_when_no_data_is_available(void)
{
    rs485_status_t status;
    uint8_t data = 0u;

    test_rs485_initialize_driver();

    status = rs485_receive(&data);

    TEST_ASSERT_EQUAL(RS485_E_STATE, status);

    test_rs485_deinitialize_driver();
}

/**
 * @brief Verify reception and consumption of one byte.
 *
 * @test
 * - Reception is started.
 * - Port writes 0x5A into the persistent receive buffer.
 * - RX-complete notification is generated.
 * - rs485_receive() returns 0x5A.
 * - Expected result: RS485_OK.
 */
void test_rs485_receive_should_return_completed_received_byte(void)
{
    rs485_status_t status;
    uint8_t data = 0u;

    test_rs485_initialize_driver();

    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_RX,
        RS485_PORT_OK);

    rs485_port_receive_it_Stub(
        test_rs485_port_receive_data_callback);

    status = rs485_receive_start();
    TEST_ASSERT_EQUAL(RS485_OK, status);

    rs485_port_rx_complete_callback();

    status = rs485_receive(&data);

    TEST_ASSERT_EQUAL(RS485_OK, status);
    TEST_ASSERT_EQUAL_HEX8(TEST_RS485_RX_DATA, data);

    /*
     * The received byte must be consumable only once.
     */
    status = rs485_receive(&data);

    TEST_ASSERT_EQUAL(RS485_E_STATE, status);

    test_rs485_deinitialize_driver();
}

/**
 * @brief Verify that a pending received byte blocks a new reception.
 *
 * @test
 * - A reception completes.
 * - The received byte has not yet been consumed.
 * - A new reception is requested.
 * - Expected result: RS485_E_STATE.
 */
void test_rs485_receive_start_should_reject_when_received_byte_is_pending(void)
{
    rs485_status_t status;
    uint8_t data = 0u;

    test_rs485_initialize_driver();

    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_RX,
        RS485_PORT_OK);

    rs485_port_receive_it_Stub(
        test_rs485_port_receive_data_callback);

    status = rs485_receive_start();
    TEST_ASSERT_EQUAL(RS485_OK, status);

    rs485_port_rx_complete_callback();

    status = rs485_receive_start();

    TEST_ASSERT_EQUAL(RS485_E_STATE, status);

    /*
     * Consume the pending byte before deinitialization.
     */
    status = rs485_receive(&data);

    TEST_ASSERT_EQUAL(RS485_OK, status);
    TEST_ASSERT_EQUAL_HEX8(TEST_RS485_RX_DATA, data);

    test_rs485_deinitialize_driver();
}

/**
 * @brief Verify abortion of an active reception.
 *
 * @test
 * - Reception is active.
 * - Port reception is aborted.
 * - Driver restores receive mode.
 * - Expected result: RS485_OK.
 */
void test_rs485_receive_abort_should_abort_active_reception(void)
{
    rs485_status_t status;

    test_rs485_initialize_driver();

    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_RX,
        RS485_PORT_OK);

    rs485_port_receive_it_Stub(
        test_rs485_port_receive_callback);

    status = rs485_receive_start();
    TEST_ASSERT_EQUAL(RS485_OK, status);

    rs485_port_abort_receive_ExpectAndReturn(RS485_PORT_OK);

    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_RX,
        RS485_PORT_OK);

    status = rs485_receive_abort();

    TEST_ASSERT_EQUAL(RS485_OK, status);

    test_rs485_deinitialize_driver();
}

/**
 * @brief Verify abortion of an active transmission.
 *
 * @test
 * - Transmission is active.
 * - Port transmission is aborted.
 * - Driver restores receive mode.
 * - Expected result: RS485_OK.
 */
void test_rs485_transmit_abort_should_abort_active_transmission(void)
{
    rs485_status_t status;

    test_rs485_initialize_driver();

    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_TX,
        RS485_PORT_OK);

    rs485_port_transmit_it_Stub(
        test_rs485_port_transmit_callback);

    status = rs485_send(TEST_RS485_TX_DATA);
    TEST_ASSERT_EQUAL(RS485_OK, status);

    rs485_port_abort_transmit_ExpectAndReturn(RS485_PORT_OK);

    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_RX,
        RS485_PORT_OK);

    status = rs485_transmit_abort();

    TEST_ASSERT_EQUAL(RS485_OK, status);

    test_rs485_deinitialize_driver();
}

/**
 * @brief Verify recovery of driver state after a UART error.
 *
 * @test
 * - Transmission is active.
 * - UART error notification is generated.
 * - Driver clears the active operation and restores receive mode.
 * - A new transmission can subsequently be started.
 */
void test_rs485_error_callback_should_release_driver_state(void)
{
    rs485_status_t status;

    test_rs485_initialize_driver();

    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_TX,
        RS485_PORT_OK);

    rs485_port_transmit_it_Stub(
        test_rs485_port_transmit_callback);

    status = rs485_send(TEST_RS485_TX_DATA);
    TEST_ASSERT_EQUAL(RS485_OK, status);

    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_RX,
        RS485_PORT_OK);

    rs485_port_error_callback(TEST_RS485_UART_ERROR);

    /*
     * If the callback correctly released tx_active, another transmission must
     * now be accepted.
     */
    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_TX,
        RS485_PORT_OK);

    rs485_port_transmit_it_Stub(
        test_rs485_port_transmit_callback);

    status = rs485_send(0x11u);

    TEST_ASSERT_EQUAL(RS485_OK, status);

    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_RX,
        RS485_PORT_OK);

    rs485_port_tx_complete_callback();

    test_rs485_deinitialize_driver();
}

/* ===================== Private Function Definitions ====================== */

/**
 * @brief Simulate successful initialization of the RS485 port.
 */
static rs485_port_status_t test_rs485_port_init_callback(
    rs485_port_handle_t * handle,
    int cmock_num_calls)
{
    (void)cmock_num_calls;

    TEST_ASSERT_NOT_NULL(handle);

    return RS485_PORT_OK;
}

/**
 * @brief Simulate successful deinitialization of the RS485 port.
 */
static rs485_port_status_t test_rs485_port_deinit_callback(
    rs485_port_handle_t * handle,
    int cmock_num_calls)
{
    (void)cmock_num_calls;

    TEST_ASSERT_NOT_NULL(handle);

    return RS485_PORT_OK;
}

/**
 * @brief Verify the single-byte interrupt-driven transmission request.
 */
static rs485_port_status_t test_rs485_port_transmit_callback(
    uint8_t * p_data,
    uint16_t size,
    int cmock_num_calls)
{
    (void)cmock_num_calls;

    TEST_ASSERT_NOT_NULL(p_data);
    TEST_ASSERT_EQUAL_UINT16(1u, size);

    /*
     * Both bytes used by the transmission tests are valid. The first byte is
     * checked explicitly when it is the main test pattern.
     */
    TEST_ASSERT_TRUE((*p_data == TEST_RS485_TX_DATA) ||
                     (*p_data == 0x11u));

    return RS485_PORT_OK;
}

/**
 * @brief Simulate successful start of a single-byte reception.
 */
static rs485_port_status_t test_rs485_port_receive_callback(
    uint8_t * p_data,
    uint16_t size,
    int cmock_num_calls)
{
    (void)cmock_num_calls;

    TEST_ASSERT_NOT_NULL(p_data);
    TEST_ASSERT_EQUAL_UINT16(1u, size);

    return RS485_PORT_OK;
}

/**
 * @brief Simulate reception of one byte through the persistent RX buffer.
 */
static rs485_port_status_t test_rs485_port_receive_data_callback(
    uint8_t * p_data,
    uint16_t size,
    int cmock_num_calls)
{
    (void)cmock_num_calls;

    TEST_ASSERT_NOT_NULL(p_data);
    TEST_ASSERT_EQUAL_UINT16(1u, size);

    *p_data = TEST_RS485_RX_DATA;

    return RS485_PORT_OK;
}

/**
 * @brief Initialize the driver in its normal receive state.
 */
static void test_rs485_initialize_driver(void)
{
    rs485_handle_t handle;
    rs485_status_t status;

    rs485_port_init_Stub(test_rs485_port_init_callback);

    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_RX,
        RS485_PORT_OK);

    status = rs485_init(&handle);

    TEST_ASSERT_EQUAL(RS485_OK, status);
}

/**
 * @brief Deinitialize the driver and restore its initial state.
 */
static void test_rs485_deinitialize_driver(void)
{
    rs485_handle_t handle;
    rs485_status_t status;

    rs485_port_set_mode_ExpectAndReturn(
        RS485_PORT_MODE_RX,
        RS485_PORT_OK);

    rs485_port_deinit_Stub(test_rs485_port_deinit_callback);

    status = rs485_deinit(&handle);

    TEST_ASSERT_EQUAL(RS485_OK, status);
}

/** @} */
