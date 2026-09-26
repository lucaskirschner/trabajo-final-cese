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
 * @file    test_vni8200xp32.c
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-09-26
 * @brief   Unit tests for the VNI8200XP-32 digital output driver.
 *
 * @details
 * This test module validates the public interface of the VNI8200XP-32 driver
 * using Ceedling, Unity, and CMock.
 *
 * Hardware-dependent operations provided by the port layer are replaced by
 * mocks so that the driver behavior can be verified independently from the
 * target hardware.
 *
 * @ingroup test_vni8200xp32
 * @{
 */

/* ============================= Includes ================================== */

#include "unity.h"
#include "Mockio_port.h"

#include "vni8200xp32.h"

#include <stddef.h>
#include <stdint.h>

/* ========================== Private Prototypes =========================== */

static io_port_status_t test_io_port_transmit_receive_callback_timeout(
    io_port_device_t device,
    const uint8_t * const tx_buffer,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls);

static io_port_status_t test_io_port_transmit_receive_callback_busy(
    io_port_device_t device,
    const uint8_t * const tx_buffer,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls);

static io_port_status_t test_io_port_transmit_receive_callback_valid_frame(
    io_port_device_t device,
    const uint8_t * const tx_buffer,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls);

static io_port_status_t test_io_port_transmit_receive_callback_invalid_parity(
    io_port_device_t device,
    const uint8_t * const tx_buffer,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls);

static io_port_status_t test_io_port_transmit_receive_callback_spi_fault(
    io_port_device_t device,
    const uint8_t * const tx_buffer,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls);

static io_port_status_t test_io_port_transmit_receive_callback_fb_fault(
    io_port_device_t device,
    const uint8_t * const tx_buffer,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls);

static io_port_status_t test_io_port_transmit_receive_callback_twarn(
    io_port_device_t device,
    const uint8_t * const tx_buffer,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls);

static io_port_status_t test_io_port_transmit_receive_callback_pg_fault(
    io_port_device_t device,
    const uint8_t * const tx_buffer,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls);

static io_port_status_t test_io_port_transmit_receive_callback_channel_fault(
    io_port_device_t device,
    const uint8_t * const tx_buffer,
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
 * @brief Verify propagation of a timeout error from the port layer.
 *
 * @test
 * - Port result: IO_PORT_E_TIMEOUT.
 * - Expected result: VNI8200XP32_E_TIMEOUT.
 */
void test_vni8200xp32_write_outputs_should_return_timeout_when_port_times_out(void)
{
    vni8200xp32_status_t status;

    io_port_transmit_receive_Stub(
        test_io_port_transmit_receive_callback_timeout);

    status = vni8200xp32_write_outputs(0x55u);

    TEST_ASSERT_EQUAL(VNI8200XP32_E_TIMEOUT, status);
}

/**
 * @brief Verify mapping of a busy port condition to a hardware error.
 *
 * @test
 * - Port result: IO_PORT_E_BUSY.
 * - Expected result: VNI8200XP32_E_HW.
 */
void test_vni8200xp32_write_outputs_should_return_hw_error_when_port_is_busy(void)
{
    vni8200xp32_status_t status;

    io_port_transmit_receive_Stub(
        test_io_port_transmit_receive_callback_busy);

    status = vni8200xp32_write_outputs(0x55u);

    TEST_ASSERT_EQUAL(VNI8200XP32_E_HW, status);
}

/**
 * @brief Verify generation of a valid command and processing of a valid reply.
 *
 * @test
 * - Requested output image: 0x55.
 * - Expected transmitted output byte: 0x55.
 * - Expected transmitted control byte: 0x01.
 * - Returned diagnostic frame: valid and fault-free.
 * - Expected result: VNI8200XP32_OK.
 */
void test_vni8200xp32_write_outputs_should_accept_valid_frame(void)
{
    vni8200xp32_status_t status;

    io_port_transmit_receive_Stub(
        test_io_port_transmit_receive_callback_valid_frame);

    status = vni8200xp32_write_outputs(0x55u);

    TEST_ASSERT_EQUAL(VNI8200XP32_OK, status);
}

/**
 * @brief Verify detection of an invalid returned-frame parity field.
 *
 * @test
 * - Returned frame parity: invalid.
 * - Expected result: VNI8200XP32_E_RX_PARITY.
 */
void test_vni8200xp32_write_outputs_should_detect_invalid_rx_parity(void)
{
    vni8200xp32_status_t status;

    io_port_transmit_receive_Stub(
        test_io_port_transmit_receive_callback_invalid_parity);

    status = vni8200xp32_write_outputs(0x55u);

    TEST_ASSERT_EQUAL(VNI8200XP32_E_RX_PARITY, status);
}

/**
 * @brief Verify detection of the SPI communication fault diagnostic.
 *
 * @test
 * - PC diagnostic bit: active.
 * - Remaining returned-frame fields: valid.
 * - Expected result: VNI8200XP32_E_SPI.
 */
void test_vni8200xp32_write_outputs_should_detect_spi_fault(void)
{
    vni8200xp32_status_t status;

    io_port_transmit_receive_Stub(
        test_io_port_transmit_receive_callback_spi_fault);

    status = vni8200xp32_write_outputs(0x55u);

    TEST_ASSERT_EQUAL(VNI8200XP32_E_SPI, status);
}

/**
 * @brief Verify detection of an invalid DC-DC feedback condition.
 *
 * @test
 * - FB_OK diagnostic bit: inactive.
 * - Remaining returned-frame fields: valid.
 * - Expected result: VNI8200XP32_E_FB.
 */
void test_vni8200xp32_write_outputs_should_detect_fb_fault(void)
{
    vni8200xp32_status_t status;

    io_port_transmit_receive_Stub(
        test_io_port_transmit_receive_callback_fb_fault);

    status = vni8200xp32_write_outputs(0x55u);

    TEST_ASSERT_EQUAL(VNI8200XP32_E_FB, status);
}

/**
 * @brief Verify detection of a case-temperature warning.
 *
 * @test
 * - TWARN diagnostic bit: active.
 * - Remaining returned-frame fields: valid.
 * - Expected result: VNI8200XP32_E_TWARN.
 */
void test_vni8200xp32_write_outputs_should_detect_temperature_warning(void)
{
    vni8200xp32_status_t status;

    io_port_transmit_receive_Stub(
        test_io_port_transmit_receive_callback_twarn);

    status = vni8200xp32_write_outputs(0x55u);

    TEST_ASSERT_EQUAL(VNI8200XP32_E_TWARN, status);
}

/**
 * @brief Verify detection of a Power Good diagnostic fault.
 *
 * @test
 * - PG diagnostic bit: fault-active.
 * - Remaining returned-frame fields: valid.
 * - Expected result: VNI8200XP32_E_PG.
 */
void test_vni8200xp32_write_outputs_should_detect_pg_fault(void)
{
    vni8200xp32_status_t status;

    io_port_transmit_receive_Stub(
        test_io_port_transmit_receive_callback_pg_fault);

    status = vni8200xp32_write_outputs(0x55u);

    TEST_ASSERT_EQUAL(VNI8200XP32_E_PG, status);
}

/**
 * @brief Verify detection of a per-channel overtemperature fault.
 *
 * @test
 * - Channel fault F0: active.
 * - Remaining returned-frame fields: valid.
 * - Expected result: VNI8200XP32_E_OVT.
 */
void test_vni8200xp32_write_outputs_should_detect_channel_fault(void)
{
    vni8200xp32_status_t status;

    io_port_transmit_receive_Stub(
        test_io_port_transmit_receive_callback_channel_fault);

    status = vni8200xp32_write_outputs(0x55u);

    TEST_ASSERT_EQUAL(VNI8200XP32_E_OVT, status);
}

/* ===================== Private Function Definitions ====================== */

/**
 * @brief Simulate a timeout error from the I/O port layer.
 */
static io_port_status_t test_io_port_transmit_receive_callback_timeout(
    io_port_device_t device,
    const uint8_t * const tx_buffer,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls)
{
    (void)device;
    (void)tx_buffer;
    (void)rx_buffer;
    (void)length;
    (void)cmock_num_calls;

    return IO_PORT_E_TIMEOUT;
}

/**
 * @brief Simulate a busy condition from the I/O port layer.
 */
static io_port_status_t test_io_port_transmit_receive_callback_busy(
    io_port_device_t device,
    const uint8_t * const tx_buffer,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls)
{
    (void)device;
    (void)tx_buffer;
    (void)rx_buffer;
    (void)length;
    (void)cmock_num_calls;

    return IO_PORT_E_BUSY;
}

/**
 * @brief Verify the generated command frame and provide a valid fault-free reply.
 */
static io_port_status_t test_io_port_transmit_receive_callback_valid_frame(
    io_port_device_t device,
    const uint8_t * const tx_buffer,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls)
{
    (void)cmock_num_calls;

    TEST_ASSERT_EQUAL(IO_PORT_DEVICE_VNI8200XP32, device);
    TEST_ASSERT_EQUAL_UINT32(2u, length);
    TEST_ASSERT_EQUAL_HEX8(0x55u, tx_buffer[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01u, tx_buffer[1]);

    rx_buffer[0] = 0x00u;
    rx_buffer[1] = 0x85u;

    return IO_PORT_OK;
}

/**
 * @brief Provide a returned frame with invalid parity.
 */
static io_port_status_t test_io_port_transmit_receive_callback_invalid_parity(
    io_port_device_t device,
    const uint8_t * const tx_buffer,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls)
{
    (void)device;
    (void)tx_buffer;
    (void)length;
    (void)cmock_num_calls;

    rx_buffer[0] = 0x00u;
    rx_buffer[1] = 0x84u;

    return IO_PORT_OK;
}

/**
 * @brief Provide a valid returned frame with the PC diagnostic active.
 */
static io_port_status_t test_io_port_transmit_receive_callback_spi_fault(
    io_port_device_t device,
    const uint8_t * const tx_buffer,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls)
{
    (void)device;
    (void)tx_buffer;
    (void)length;
    (void)cmock_num_calls;

    rx_buffer[0] = 0x00u;
    rx_buffer[1] = 0xA1u;

    return IO_PORT_OK;
}

/**
 * @brief Provide a valid returned frame with FB_OK inactive.
 */
static io_port_status_t test_io_port_transmit_receive_callback_fb_fault(
    io_port_device_t device,
    const uint8_t * const tx_buffer,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls)
{
    (void)device;
    (void)tx_buffer;
    (void)length;
    (void)cmock_num_calls;

    rx_buffer[0] = 0x00u;
    rx_buffer[1] = 0x01u;

    return IO_PORT_OK;
}

/**
 * @brief Provide a valid returned frame with TWARN active.
 */
static io_port_status_t test_io_port_transmit_receive_callback_twarn(
    io_port_device_t device,
    const uint8_t * const tx_buffer,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls)
{
    (void)device;
    (void)tx_buffer;
    (void)length;
    (void)cmock_num_calls;

    rx_buffer[0] = 0x00u;
    rx_buffer[1] = 0xCDu;

    return IO_PORT_OK;
}

/**
 * @brief Provide a valid returned frame with PG fault-active.
 */
static io_port_status_t test_io_port_transmit_receive_callback_pg_fault(
    io_port_device_t device,
    const uint8_t * const tx_buffer,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls)
{
    (void)device;
    (void)tx_buffer;
    (void)length;
    (void)cmock_num_calls;

    rx_buffer[0] = 0x00u;
    rx_buffer[1] = 0x9Du;

    return IO_PORT_OK;
}

/**
 * @brief Provide a valid returned frame with channel fault F0 active.
 */
static io_port_status_t test_io_port_transmit_receive_callback_channel_fault(
    io_port_device_t device,
    const uint8_t * const tx_buffer,
    uint8_t * const rx_buffer,
    size_t length,
    int cmock_num_calls)
{
    (void)device;
    (void)tx_buffer;
    (void)length;
    (void)cmock_num_calls;

    rx_buffer[0] = 0x01u;
    rx_buffer[1] = 0x8Eu;

    return IO_PORT_OK;
}

/** @} */
