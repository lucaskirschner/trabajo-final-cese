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
 * @file    test_sclt38bt8.c
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-09-26
 * @brief   Unit tests for the SCLT3-8BT8 digital input driver.
 *
 * @details
 * This test module validates the public interface of the SCLT3-8BT8 driver
 * using Ceedling, Unity, and CMock.
 *
 * Hardware-dependent operations provided by the port layer are replaced by
 * mocks so that the driver behavior can be verified independently from the
 * target hardware.
 *
 * @ingroup test_sclt38bt8
 * @{
 */

/* ============================= Includes ================================== */

#include "unity.h"
#include "Mockio_port.h"

#include "sclt38bt8.h"

#include <stddef.h>
#include <stdint.h>

/* ========================== Private Prototypes =========================== */

static io_port_status_t test_io_port_receive_callback_timeout(
    io_port_device_t device,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls);

static io_port_status_t test_io_port_receive_callback_busy(
    io_port_device_t device,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls);

static io_port_status_t test_io_port_receive_callback_valid_frame(
    io_port_device_t device,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls);

static io_port_status_t test_io_port_receive_callback_power_loss(
    io_port_device_t device,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls);

static io_port_status_t test_io_port_receive_callback_stop_bits(
    io_port_device_t device,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls);

static io_port_status_t test_io_port_receive_callback_parity(
    io_port_device_t device,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls);

static io_port_status_t test_io_port_receive_callback_undervoltage(
    io_port_device_t device,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls);

static io_port_status_t test_io_port_receive_callback_overtemperature(
    io_port_device_t device,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls);

/* ======================= Test Setup and Teardown ========================== */

/**
 * @brief Prepare the test environment before each test case.
 */
void setUp(void)
{
    /* No setup required. */
}

/**
 * @brief Restore the test environment after each test case.
 */
void tearDown(void)
{
    /* No teardown required. */
}

/* ============================= Test Cases ================================ */

/**
 * @brief Verify null-pointer validation when reading digital inputs.
 *
 * @details
 * The driver shall reject a null output pointer before attempting any access
 * to the underlying port layer.
 *
 * @test
 * - Input: NULL output pointer.
 * - Expected result: SCLT38BT8_E_NULL.
 */
void test_sclt38bt8_read_inputs_should_return_null_error_when_pointer_is_null(void)
{
    sclt38bt8_status_t status;

    status = sclt38bt8_read_inputs(NULL);

    TEST_ASSERT_EQUAL(SCLT38BT8_E_NULL, status);
}

/**
 * @brief Verify propagation of a timeout error from the port layer.
 *
 * @test
 * - Port result: IO_PORT_E_TIMEOUT.
 * - Expected result: SCLT38BT8_E_TIMEOUT.
 */
void test_sclt38bt8_read_inputs_should_return_timeout_when_port_times_out(void)
{
    sclt38bt8_status_t status;
    uint8_t inputs = 0u;

    io_port_receive_Stub(test_io_port_receive_callback_timeout);

    status = sclt38bt8_read_inputs(&inputs);

    TEST_ASSERT_EQUAL(SCLT38BT8_E_TIMEOUT, status);
}

/**
 * @brief Verify mapping of a busy port condition to a hardware error.
 *
 * @test
 * - Port result: IO_PORT_E_BUSY.
 * - Expected result: SCLT38BT8_E_HW.
 */
void test_sclt38bt8_read_inputs_should_return_hw_error_when_port_is_busy(void)
{
    sclt38bt8_status_t status;
    uint8_t inputs = 0u;

    io_port_receive_Stub(test_io_port_receive_callback_busy);

    status = sclt38bt8_read_inputs(&inputs);

    TEST_ASSERT_EQUAL(SCLT38BT8_E_HW, status);
}

/**
 * @brief Verify decoding of a valid SCLT3-8BT8 frame.
 *
 * @test
 * - Input byte: 0x55.
 * - Frame diagnostics and parity: valid.
 * - Expected result: SCLT38BT8_OK.
 * - Expected decoded inputs: 0x55.
 */
void test_sclt38bt8_read_inputs_should_decode_valid_frame(void)
{
    sclt38bt8_status_t status;
    uint8_t inputs = 0u;

    io_port_receive_Stub(test_io_port_receive_callback_valid_frame);

    status = sclt38bt8_read_inputs(&inputs);

    TEST_ASSERT_EQUAL(SCLT38BT8_OK, status);
    TEST_ASSERT_EQUAL_HEX8(0x55u, inputs);
}

/**
 * @brief Verify detection of a power-loss condition.
 *
 * @test
 * - High-state stop bit: low.
 * - Expected result: SCLT38BT8_E_POWER_LOSS.
 */
void test_sclt38bt8_read_inputs_should_detect_power_loss(void)
{
    sclt38bt8_status_t status;
    uint8_t inputs = 0u;

    io_port_receive_Stub(test_io_port_receive_callback_power_loss);

    status = sclt38bt8_read_inputs(&inputs);

    TEST_ASSERT_EQUAL(SCLT38BT8_E_POWER_LOSS, status);
}

/**
 * @brief Verify detection of an invalid stop-bit pattern.
 *
 * @test
 * - Stop bits: invalid.
 * - Expected result: SCLT38BT8_E_STOP_BITS.
 */
void test_sclt38bt8_read_inputs_should_detect_invalid_stop_bits(void)
{
    sclt38bt8_status_t status;
    uint8_t inputs = 0u;

    io_port_receive_Stub(test_io_port_receive_callback_stop_bits);

    status = sclt38bt8_read_inputs(&inputs);

    TEST_ASSERT_EQUAL(SCLT38BT8_E_STOP_BITS, status);
}

/**
 * @brief Verify detection of an invalid parity field.
 *
 * @test
 * - Stop bits: valid.
 * - Parity field: invalid.
 * - Expected result: SCLT38BT8_E_PARITY.
 */
void test_sclt38bt8_read_inputs_should_detect_invalid_parity(void)
{
    sclt38bt8_status_t status;
    uint8_t inputs = 0u;

    io_port_receive_Stub(test_io_port_receive_callback_parity);

    status = sclt38bt8_read_inputs(&inputs);

    TEST_ASSERT_EQUAL(SCLT38BT8_E_PARITY, status);
}

/**
 * @brief Verify detection of the active-low undervoltage alarm.
 *
 * @test
 * - /UVA: active.
 * - Remaining frame fields: valid.
 * - Expected result: SCLT38BT8_E_UV.
 */
void test_sclt38bt8_read_inputs_should_detect_undervoltage(void)
{
    sclt38bt8_status_t status;
    uint8_t inputs = 0u;

    io_port_receive_Stub(test_io_port_receive_callback_undervoltage);

    status = sclt38bt8_read_inputs(&inputs);

    TEST_ASSERT_EQUAL(SCLT38BT8_E_UV, status);
}

/**
 * @brief Verify detection of the active-low overtemperature alarm.
 *
 * @test
 * - /OTA: active.
 * - Remaining frame fields: valid.
 * - Expected result: SCLT38BT8_E_OT.
 */
void test_sclt38bt8_read_inputs_should_detect_overtemperature(void)
{
    sclt38bt8_status_t status;
    uint8_t inputs = 0u;

    io_port_receive_Stub(test_io_port_receive_callback_overtemperature);

    status = sclt38bt8_read_inputs(&inputs);

    TEST_ASSERT_EQUAL(SCLT38BT8_E_OT, status);
}

/* ===================== Private Function Definitions ====================== */

/**
 * @brief Simulate a timeout error from the I/O port layer.
 */
static io_port_status_t test_io_port_receive_callback_timeout(
    io_port_device_t device,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls)
{
    (void)device;
    (void)rx_buffer;
    (void)length;
    (void)cmock_num_calls;

    return IO_PORT_E_TIMEOUT;
}

/**
 * @brief Simulate a busy condition from the I/O port layer.
 */
static io_port_status_t test_io_port_receive_callback_busy(
    io_port_device_t device,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls)
{
    (void)device;
    (void)rx_buffer;
    (void)length;
    (void)cmock_num_calls;

    return IO_PORT_E_BUSY;
}

/**
 * @brief Provide a valid SPI frame containing input pattern 0x55.
 */
static io_port_status_t test_io_port_receive_callback_valid_frame(
    io_port_device_t device,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls)
{
    (void)device;
    (void)length;
    (void)cmock_num_calls;

    rx_buffer[0] = 0x55u;
    rx_buffer[1] = 0xFDu;

    return IO_PORT_OK;
}

/**
 * @brief Provide a frame representing loss of device power.
 */
static io_port_status_t test_io_port_receive_callback_power_loss(
    io_port_device_t device,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls)
{
    (void)device;
    (void)length;
    (void)cmock_num_calls;

    rx_buffer[0] = 0x55u;
    rx_buffer[1] = 0xC0u;

    return IO_PORT_OK;
}

/**
 * @brief Provide a frame with an invalid stop-bit pattern.
 */
static io_port_status_t test_io_port_receive_callback_stop_bits(
    io_port_device_t device,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls)
{
    (void)device;
    (void)length;
    (void)cmock_num_calls;

    rx_buffer[0] = 0x55u;
    rx_buffer[1] = 0xC3u;

    return IO_PORT_OK;
}

/**
 * @brief Provide a frame with valid stop bits and invalid parity.
 */
static io_port_status_t test_io_port_receive_callback_parity(
    io_port_device_t device,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls)
{
    (void)device;
    (void)length;
    (void)cmock_num_calls;

    rx_buffer[0] = 0x55u;
    rx_buffer[1] = 0xD1u;

    return IO_PORT_OK;
}

/**
 * @brief Provide a valid frame with the undervoltage alarm active.
 */
static io_port_status_t test_io_port_receive_callback_undervoltage(
    io_port_device_t device,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls)
{
    (void)device;
    (void)length;
    (void)cmock_num_calls;

    rx_buffer[0] = 0x55u;
    rx_buffer[1] = 0x7Du;

    return IO_PORT_OK;
}

/**
 * @brief Provide a valid frame with the overtemperature alarm active.
 */
static io_port_status_t test_io_port_receive_callback_overtemperature(
    io_port_device_t device,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls)
{
    (void)device;
    (void)length;
    (void)cmock_num_calls;

    rx_buffer[0] = 0x55u;
    rx_buffer[1] = 0xBDu;

    return IO_PORT_OK;
}

/** @} */
