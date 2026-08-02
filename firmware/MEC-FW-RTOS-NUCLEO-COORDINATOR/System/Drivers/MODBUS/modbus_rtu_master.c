/****************************************************************************
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
 ****************************************************************************/

/**
 * @file    modbus_rtu_master.c
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-08-02
 * @brief   Basic blocking MODBUS RTU master implementation.
 *
 * @details
 * This module implements blocking unicast MODBUS RTU transactions for function
 * codes 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x0F and 0x10.
 *
 * Request and response layouts, quantity limits, bit packing and big-endian
 * data encoding follow MODBUS Application Protocol Specification V1.1b3,
 * sections 4.2, 6.1 to 6.6, 6.11 and 6.12.
 *
 * RTU ADUs contain a one-byte server address, a MODBUS PDU and a two-byte CRC
 * transmitted low byte first. The maximum RTU ADU size is 256 bytes according
 * to MODBUS over Serial Line Specification and Implementation Guide V1.02,
 * sections 2.5.1 and 2.5.1.2, pages 12 to 15.
 *
 * Responses are received in two stages to distinguish normal responses from
 * five-byte exception responses while using a fixed-length blocking receive
 * primitive.
 *
 * The lower RS485/UART layer must preserve continuous character transmission
 * and the MODBUS RTU silent intervals. This module does not implement t1.5 or
 * t3.5 timers and does not support broadcast transactions.
 *
 * @ingroup modbus_rtu_master
 * @{
 */

/* ============================= Includes ================================== */

#include "modbus_rtu_master.h"

#include "modbus_crc.h"
#include "rs485.h"

#include <stddef.h>

/* ============================ Local Macros =============================== */

#define MODBUS_RTU_FC_READ_COILS                      (0x01u)
#define MODBUS_RTU_FC_READ_DISCRETE_INPUTS            (0x02u)
#define MODBUS_RTU_FC_READ_HOLDING_REGISTERS          (0x03u)
#define MODBUS_RTU_FC_READ_INPUT_REGISTERS            (0x04u)
#define MODBUS_RTU_FC_WRITE_SINGLE_COIL               (0x05u)
#define MODBUS_RTU_FC_WRITE_SINGLE_REGISTER           (0x06u)
#define MODBUS_RTU_FC_WRITE_MULTIPLE_COILS            (0x0Fu)
#define MODBUS_RTU_FC_WRITE_MULTIPLE_REGISTERS        (0x10u)

#define MODBUS_RTU_EXCEPTION_FLAG                     (0x80u)

#define MODBUS_RTU_MAX_ADU_SIZE                       (256u)
#define MODBUS_RTU_CRC_SIZE                           (2u)
#define MODBUS_RTU_READ_RESPONSE_HEADER_SIZE          (3u)
#define MODBUS_RTU_WRITE_RESPONSE_SIZE                (8u)
#define MODBUS_RTU_EXCEPTION_RESPONSE_SIZE            (5u)
#define MODBUS_RTU_FIXED_REQUEST_SIZE                 (8u)
#define MODBUS_RTU_WRITE_MULTIPLE_HEADER_SIZE         (7u)

#define MODBUS_RTU_COIL_ON_VALUE                      (0xFF00u)
#define MODBUS_RTU_COIL_OFF_VALUE                     (0x0000u)

/* ============================ Local Types ================================ */

typedef struct
{
    bool initialized;
    rs485_handle_t rs485_handle;
    modbus_exception_code_t last_exception_code;
} modbus_rtu_master_ctx_t;

/* ======================= Local (static) Data ============================= */

static modbus_rtu_master_ctx_t g_ctx = {
    .initialized = false,
    .rs485_handle = {
        .reserved = 0u
    },
    .last_exception_code = MODBUS_EXCEPTION_NONE
};

/* ===================== Local Function Prototypes ========================= */

static modbus_rtu_master_status_t modbus_rtu_master_validate_server_address(
    uint8_t server_address);

static modbus_rtu_master_status_t modbus_rtu_master_validate_address_range(
    uint16_t start_address,
    uint16_t quantity);

static modbus_rtu_master_status_t modbus_rtu_master_validate_read_bits(
    uint8_t server_address,
    uint16_t start_address,
    uint16_t quantity,
    uint16_t maximum_quantity,
    const uint8_t *values);

static modbus_rtu_master_status_t modbus_rtu_master_validate_read_registers(
    uint8_t server_address,
    uint16_t start_address,
    uint16_t quantity,
    const uint16_t *registers);

static modbus_rtu_master_status_t modbus_rtu_master_validate_write_coils(
    uint8_t server_address,
    uint16_t start_address,
    uint16_t quantity,
    const uint8_t *coils);

static modbus_rtu_master_status_t modbus_rtu_master_validate_write_registers(
    uint8_t server_address,
    uint16_t start_address,
    uint16_t quantity,
    const uint16_t *registers);

static void modbus_rtu_master_build_fixed_request(
    uint8_t server_address,
    uint8_t function_code,
    uint16_t address,
    uint16_t value_or_quantity,
    uint8_t *frame);

static uint16_t modbus_rtu_master_build_write_multiple_coils_request(
    uint8_t server_address,
    uint16_t start_address,
    uint16_t quantity,
    const uint8_t *coils,
    uint8_t *frame);

static uint16_t modbus_rtu_master_build_write_multiple_registers_request(
    uint8_t server_address,
    uint16_t start_address,
    uint16_t quantity,
    const uint16_t *registers,
    uint8_t *frame);

static void modbus_rtu_master_append_crc(
    uint8_t *frame,
    uint16_t frame_size_without_crc);

static modbus_rtu_master_status_t modbus_rtu_master_send_request(
    const uint8_t *frame,
    uint16_t frame_size,
    uint32_t timeout_ms);

static modbus_rtu_master_status_t modbus_rtu_master_receive_read_response(
    uint8_t server_address,
    uint8_t function_code,
    uint16_t expected_byte_count,
    uint8_t *frame,
    uint16_t *frame_size,
    uint32_t timeout_ms);

static modbus_rtu_master_status_t modbus_rtu_master_receive_write_response(
    uint8_t server_address,
    uint8_t function_code,
    uint8_t *frame,
    uint32_t timeout_ms);

static modbus_rtu_master_status_t modbus_rtu_master_process_exception_response(
    const uint8_t *frame,
    uint8_t server_address,
    uint8_t function_code);

static modbus_rtu_master_status_t modbus_rtu_master_validate_read_response(
    const uint8_t *frame,
    uint16_t frame_size,
    uint8_t server_address,
    uint8_t function_code,
    uint16_t expected_byte_count);

static modbus_rtu_master_status_t modbus_rtu_master_validate_write_response(
    const uint8_t *frame,
    uint8_t server_address,
    uint8_t function_code,
    uint16_t expected_address,
    uint16_t expected_value);

static void modbus_rtu_master_unpack_bits(
    const uint8_t *packed_data,
    uint16_t quantity,
    uint8_t *values);

static bool modbus_rtu_master_is_crc_valid(
    const uint8_t *frame,
    uint16_t frame_size);

static modbus_rtu_master_status_t modbus_rtu_master_rs485_status_to_status(
    rs485_status_t rs485_status);

/* ===================== Public Function Definitions ======================= */

modbus_rtu_master_status_t modbus_rtu_master_init(void)
{
    rs485_status_t rs485_status = RS485_OK;
    modbus_rtu_master_status_t status = MODBUS_RTU_MASTER_OK;

    if (g_ctx.initialized != false)
    {
        return MODBUS_RTU_MASTER_E_STATE;
    }

    rs485_status = rs485_init(&g_ctx.rs485_handle);
    status = modbus_rtu_master_rs485_status_to_status(rs485_status);
    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    g_ctx.last_exception_code = MODBUS_EXCEPTION_NONE;
    g_ctx.initialized = true;

    return MODBUS_RTU_MASTER_OK;
}

modbus_rtu_master_status_t modbus_rtu_master_deinit(void)
{
    rs485_status_t rs485_status = RS485_OK;
    modbus_rtu_master_status_t status = MODBUS_RTU_MASTER_OK;

    if (g_ctx.initialized == false)
    {
        return MODBUS_RTU_MASTER_E_STATE;
    }

    rs485_status = rs485_deinit(&g_ctx.rs485_handle);
    status = modbus_rtu_master_rs485_status_to_status(rs485_status);
    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    g_ctx.last_exception_code = MODBUS_EXCEPTION_NONE;
    g_ctx.initialized = false;

    return MODBUS_RTU_MASTER_OK;
}

modbus_rtu_master_status_t modbus_rtu_master_read_coils(
    uint8_t server_address,
    uint16_t start_address,
    uint16_t quantity,
    uint8_t *coils,
    uint32_t timeout_ms)
{
    uint8_t tx_frame[MODBUS_RTU_FIXED_REQUEST_SIZE] = {0u};
    uint8_t rx_frame[MODBUS_RTU_MAX_ADU_SIZE] = {0u};
    uint16_t rx_frame_size = 0u;
    uint16_t expected_byte_count = 0u;
    modbus_rtu_master_status_t status = MODBUS_RTU_MASTER_OK;

    if (g_ctx.initialized == false)
    {
        return MODBUS_RTU_MASTER_E_STATE;
    }

    status = modbus_rtu_master_validate_read_bits(
        server_address,
        start_address,
        quantity,
        MODBUS_RTU_MASTER_MAX_READ_COILS,
        coils);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    modbus_rtu_master_build_fixed_request(
        server_address,
        MODBUS_RTU_FC_READ_COILS,
        start_address,
        quantity,
        tx_frame);

    g_ctx.last_exception_code = MODBUS_EXCEPTION_NONE;

    status = modbus_rtu_master_send_request(
        tx_frame,
        MODBUS_RTU_FIXED_REQUEST_SIZE,
        timeout_ms);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    expected_byte_count = (quantity + 7u) / 8u;

    status = modbus_rtu_master_receive_read_response(
        server_address,
        MODBUS_RTU_FC_READ_COILS,
        expected_byte_count,
        rx_frame,
        &rx_frame_size,
        timeout_ms);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    status = modbus_rtu_master_validate_read_response(
        rx_frame,
        rx_frame_size,
        server_address,
        MODBUS_RTU_FC_READ_COILS,
        expected_byte_count);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    modbus_rtu_master_unpack_bits(&rx_frame[3], quantity, coils);

    return MODBUS_RTU_MASTER_OK;
}

modbus_rtu_master_status_t modbus_rtu_master_read_discrete_inputs(
    uint8_t server_address,
    uint16_t start_address,
    uint16_t quantity,
    uint8_t *inputs,
    uint32_t timeout_ms)
{
    uint8_t tx_frame[MODBUS_RTU_FIXED_REQUEST_SIZE] = {0u};
    uint8_t rx_frame[MODBUS_RTU_MAX_ADU_SIZE] = {0u};
    uint16_t rx_frame_size = 0u;
    uint16_t expected_byte_count = 0u;
    modbus_rtu_master_status_t status = MODBUS_RTU_MASTER_OK;

    if (g_ctx.initialized == false)
    {
        return MODBUS_RTU_MASTER_E_STATE;
    }

    status = modbus_rtu_master_validate_read_bits(
        server_address,
        start_address,
        quantity,
        MODBUS_RTU_MASTER_MAX_READ_DISCRETE_INPUTS,
        inputs);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    modbus_rtu_master_build_fixed_request(
        server_address,
        MODBUS_RTU_FC_READ_DISCRETE_INPUTS,
        start_address,
        quantity,
        tx_frame);

    g_ctx.last_exception_code = MODBUS_EXCEPTION_NONE;

    status = modbus_rtu_master_send_request(
        tx_frame,
        MODBUS_RTU_FIXED_REQUEST_SIZE,
        timeout_ms);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    expected_byte_count = (quantity + 7u) / 8u;

    status = modbus_rtu_master_receive_read_response(
        server_address,
        MODBUS_RTU_FC_READ_DISCRETE_INPUTS,
        expected_byte_count,
        rx_frame,
        &rx_frame_size,
        timeout_ms);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    status = modbus_rtu_master_validate_read_response(
        rx_frame,
        rx_frame_size,
        server_address,
        MODBUS_RTU_FC_READ_DISCRETE_INPUTS,
        expected_byte_count);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    modbus_rtu_master_unpack_bits(&rx_frame[3], quantity, inputs);

    return MODBUS_RTU_MASTER_OK;
}

modbus_rtu_master_status_t modbus_rtu_master_read_holding_registers(
    uint8_t server_address,
    uint16_t start_address,
    uint16_t quantity,
    uint16_t *registers,
    uint32_t timeout_ms)
{
    uint8_t tx_frame[MODBUS_RTU_FIXED_REQUEST_SIZE] = {0u};
    uint8_t rx_frame[MODBUS_RTU_MAX_ADU_SIZE] = {0u};
    uint16_t rx_frame_size = 0u;
    uint16_t expected_byte_count = 0u;
    uint16_t register_index = 0u;
    uint16_t data_offset = 0u;
    modbus_rtu_master_status_t status = MODBUS_RTU_MASTER_OK;

    if (g_ctx.initialized == false)
    {
        return MODBUS_RTU_MASTER_E_STATE;
    }

    status = modbus_rtu_master_validate_read_registers(
        server_address,
        start_address,
        quantity,
        registers);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    modbus_rtu_master_build_fixed_request(
        server_address,
        MODBUS_RTU_FC_READ_HOLDING_REGISTERS,
        start_address,
        quantity,
        tx_frame);

    g_ctx.last_exception_code = MODBUS_EXCEPTION_NONE;

    status = modbus_rtu_master_send_request(
        tx_frame,
        MODBUS_RTU_FIXED_REQUEST_SIZE,
        timeout_ms);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    expected_byte_count = 2u * quantity;

    status = modbus_rtu_master_receive_read_response(
        server_address,
        MODBUS_RTU_FC_READ_HOLDING_REGISTERS,
        expected_byte_count,
        rx_frame,
        &rx_frame_size,
        timeout_ms);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    status = modbus_rtu_master_validate_read_response(
        rx_frame,
        rx_frame_size,
        server_address,
        MODBUS_RTU_FC_READ_HOLDING_REGISTERS,
        expected_byte_count);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    for (register_index = 0u;
         register_index < quantity;
         register_index++)
    {
        data_offset = 3u + (2u * register_index);

        registers[register_index] =
            ((uint16_t)rx_frame[data_offset] << 8u) |
            (uint16_t)rx_frame[data_offset + 1u];
    }

    return MODBUS_RTU_MASTER_OK;
}

modbus_rtu_master_status_t modbus_rtu_master_read_input_registers(
    uint8_t server_address,
    uint16_t start_address,
    uint16_t quantity,
    uint16_t *registers,
    uint32_t timeout_ms)
{
    uint8_t tx_frame[MODBUS_RTU_FIXED_REQUEST_SIZE] = {0u};
    uint8_t rx_frame[MODBUS_RTU_MAX_ADU_SIZE] = {0u};
    uint16_t rx_frame_size = 0u;
    uint16_t expected_byte_count = 0u;
    uint16_t register_index = 0u;
    uint16_t data_offset = 0u;
    modbus_rtu_master_status_t status = MODBUS_RTU_MASTER_OK;

    if (g_ctx.initialized == false)
    {
        return MODBUS_RTU_MASTER_E_STATE;
    }

    status = modbus_rtu_master_validate_read_registers(
        server_address,
        start_address,
        quantity,
        registers);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    modbus_rtu_master_build_fixed_request(
        server_address,
        MODBUS_RTU_FC_READ_INPUT_REGISTERS,
        start_address,
        quantity,
        tx_frame);

    g_ctx.last_exception_code = MODBUS_EXCEPTION_NONE;

    status = modbus_rtu_master_send_request(
        tx_frame,
        MODBUS_RTU_FIXED_REQUEST_SIZE,
        timeout_ms);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    expected_byte_count = 2u * quantity;

    status = modbus_rtu_master_receive_read_response(
        server_address,
        MODBUS_RTU_FC_READ_INPUT_REGISTERS,
        expected_byte_count,
        rx_frame,
        &rx_frame_size,
        timeout_ms);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    status = modbus_rtu_master_validate_read_response(
        rx_frame,
        rx_frame_size,
        server_address,
        MODBUS_RTU_FC_READ_INPUT_REGISTERS,
        expected_byte_count);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    for (register_index = 0u;
         register_index < quantity;
         register_index++)
    {
        data_offset = 3u + (2u * register_index);

        registers[register_index] =
            ((uint16_t)rx_frame[data_offset] << 8u) |
            (uint16_t)rx_frame[data_offset + 1u];
    }

    return MODBUS_RTU_MASTER_OK;
}

modbus_rtu_master_status_t modbus_rtu_master_write_single_coil(
    uint8_t server_address,
    uint16_t coil_address,
    bool value,
    uint32_t timeout_ms)
{
    uint8_t tx_frame[MODBUS_RTU_FIXED_REQUEST_SIZE] = {0u};
    uint8_t rx_frame[MODBUS_RTU_WRITE_RESPONSE_SIZE] = {0u};
    uint16_t coil_value = MODBUS_RTU_COIL_OFF_VALUE;
    modbus_rtu_master_status_t status = MODBUS_RTU_MASTER_OK;

    if (g_ctx.initialized == false)
    {
        return MODBUS_RTU_MASTER_E_STATE;
    }

    status = modbus_rtu_master_validate_server_address(server_address);
    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    coil_value =
        (value != false) ?
        MODBUS_RTU_COIL_ON_VALUE :
        MODBUS_RTU_COIL_OFF_VALUE;

    modbus_rtu_master_build_fixed_request(
        server_address,
        MODBUS_RTU_FC_WRITE_SINGLE_COIL,
        coil_address,
        coil_value,
        tx_frame);

    g_ctx.last_exception_code = MODBUS_EXCEPTION_NONE;

    status = modbus_rtu_master_send_request(
        tx_frame,
        MODBUS_RTU_FIXED_REQUEST_SIZE,
        timeout_ms);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    status = modbus_rtu_master_receive_write_response(
        server_address,
        MODBUS_RTU_FC_WRITE_SINGLE_COIL,
        rx_frame,
        timeout_ms);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    return modbus_rtu_master_validate_write_response(
        rx_frame,
        server_address,
        MODBUS_RTU_FC_WRITE_SINGLE_COIL,
        coil_address,
        coil_value);
}

modbus_rtu_master_status_t modbus_rtu_master_write_single_register(
    uint8_t server_address,
    uint16_t register_address,
    uint16_t value,
    uint32_t timeout_ms)
{
    uint8_t tx_frame[MODBUS_RTU_FIXED_REQUEST_SIZE] = {0u};
    uint8_t rx_frame[MODBUS_RTU_WRITE_RESPONSE_SIZE] = {0u};
    modbus_rtu_master_status_t status = MODBUS_RTU_MASTER_OK;

    if (g_ctx.initialized == false)
    {
        return MODBUS_RTU_MASTER_E_STATE;
    }

    status = modbus_rtu_master_validate_server_address(server_address);
    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    modbus_rtu_master_build_fixed_request(
        server_address,
        MODBUS_RTU_FC_WRITE_SINGLE_REGISTER,
        register_address,
        value,
        tx_frame);

    g_ctx.last_exception_code = MODBUS_EXCEPTION_NONE;

    status = modbus_rtu_master_send_request(
        tx_frame,
        MODBUS_RTU_FIXED_REQUEST_SIZE,
        timeout_ms);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    status = modbus_rtu_master_receive_write_response(
        server_address,
        MODBUS_RTU_FC_WRITE_SINGLE_REGISTER,
        rx_frame,
        timeout_ms);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    return modbus_rtu_master_validate_write_response(
        rx_frame,
        server_address,
        MODBUS_RTU_FC_WRITE_SINGLE_REGISTER,
        register_address,
        value);
}

modbus_rtu_master_status_t modbus_rtu_master_write_multiple_coils(
    uint8_t server_address,
    uint16_t start_address,
    uint16_t quantity,
    const uint8_t *coils,
    uint32_t timeout_ms)
{
    uint8_t tx_frame[MODBUS_RTU_MAX_ADU_SIZE] = {0u};
    uint8_t rx_frame[MODBUS_RTU_WRITE_RESPONSE_SIZE] = {0u};
    uint16_t tx_frame_size = 0u;
    modbus_rtu_master_status_t status = MODBUS_RTU_MASTER_OK;

    if (g_ctx.initialized == false)
    {
        return MODBUS_RTU_MASTER_E_STATE;
    }

    status = modbus_rtu_master_validate_write_coils(
        server_address,
        start_address,
        quantity,
        coils);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    tx_frame_size = modbus_rtu_master_build_write_multiple_coils_request(
        server_address,
        start_address,
        quantity,
        coils,
        tx_frame);

    g_ctx.last_exception_code = MODBUS_EXCEPTION_NONE;

    status = modbus_rtu_master_send_request(
        tx_frame,
        tx_frame_size,
        timeout_ms);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    status = modbus_rtu_master_receive_write_response(
        server_address,
        MODBUS_RTU_FC_WRITE_MULTIPLE_COILS,
        rx_frame,
        timeout_ms);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    return modbus_rtu_master_validate_write_response(
        rx_frame,
        server_address,
        MODBUS_RTU_FC_WRITE_MULTIPLE_COILS,
        start_address,
        quantity);
}

modbus_rtu_master_status_t modbus_rtu_master_write_multiple_registers(
    uint8_t server_address,
    uint16_t start_address,
    uint16_t quantity,
    const uint16_t *registers,
    uint32_t timeout_ms)
{
    uint8_t tx_frame[MODBUS_RTU_MAX_ADU_SIZE] = {0u};
    uint8_t rx_frame[MODBUS_RTU_WRITE_RESPONSE_SIZE] = {0u};
    uint16_t tx_frame_size = 0u;
    modbus_rtu_master_status_t status = MODBUS_RTU_MASTER_OK;

    if (g_ctx.initialized == false)
    {
        return MODBUS_RTU_MASTER_E_STATE;
    }

    status = modbus_rtu_master_validate_write_registers(
        server_address,
        start_address,
        quantity,
        registers);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    tx_frame_size =
        modbus_rtu_master_build_write_multiple_registers_request(
            server_address,
            start_address,
            quantity,
            registers,
            tx_frame);

    g_ctx.last_exception_code = MODBUS_EXCEPTION_NONE;

    status = modbus_rtu_master_send_request(
        tx_frame,
        tx_frame_size,
        timeout_ms);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    status = modbus_rtu_master_receive_write_response(
        server_address,
        MODBUS_RTU_FC_WRITE_MULTIPLE_REGISTERS,
        rx_frame,
        timeout_ms);

    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    return modbus_rtu_master_validate_write_response(
        rx_frame,
        server_address,
        MODBUS_RTU_FC_WRITE_MULTIPLE_REGISTERS,
        start_address,
        quantity);
}

modbus_exception_code_t modbus_rtu_master_get_last_exception_code(void)
{
    return g_ctx.last_exception_code;
}

/* ===================== Local Function Definitions ======================== */

/**
 * @brief Validate a unicast MODBUS server address.
 *
 * @param[in] server_address  Server address to validate.
 *
 * @return MODBUS_RTU_MASTER_OK on success, error code otherwise.
 */
static modbus_rtu_master_status_t modbus_rtu_master_validate_server_address(
    uint8_t server_address)
{
    if ((server_address < MODBUS_RTU_MASTER_MIN_SERVER_ADDRESS) ||
        (server_address > MODBUS_RTU_MASTER_MAX_SERVER_ADDRESS))
    {
        return MODBUS_RTU_MASTER_E_PARAM;
    }

    return MODBUS_RTU_MASTER_OK;
}

/**
 * @brief Validate that an address range fits in the 16-bit PDU address space.
 *
 * @param[in] start_address  First zero-based address.
 * @param[in] quantity       Number of consecutive elements.
 *
 * @return MODBUS_RTU_MASTER_OK on success, error code otherwise.
 */
static modbus_rtu_master_status_t modbus_rtu_master_validate_address_range(
    uint16_t start_address,
    uint16_t quantity)
{
    uint32_t last_address = 0u;

    if (quantity == 0u)
    {
        return MODBUS_RTU_MASTER_E_PARAM;
    }

    last_address =
        (uint32_t)start_address +
        (uint32_t)quantity -
        1u;

    if (last_address > UINT16_MAX)
    {
        return MODBUS_RTU_MASTER_E_PARAM;
    }

    return MODBUS_RTU_MASTER_OK;
}

/**
 * @brief Validate common parameters for bit-read functions.
 */
static modbus_rtu_master_status_t modbus_rtu_master_validate_read_bits(
    uint8_t server_address,
    uint16_t start_address,
    uint16_t quantity,
    uint16_t maximum_quantity,
    const uint8_t *values)
{
    modbus_rtu_master_status_t status = MODBUS_RTU_MASTER_OK;

    if (values == NULL)
    {
        return MODBUS_RTU_MASTER_E_NULL;
    }

    status = modbus_rtu_master_validate_server_address(server_address);
    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    if ((quantity == 0u) || (quantity > maximum_quantity))
    {
        return MODBUS_RTU_MASTER_E_PARAM;
    }

    return modbus_rtu_master_validate_address_range(
        start_address,
        quantity);
}

/**
 * @brief Validate common parameters for register-read functions.
 */
static modbus_rtu_master_status_t modbus_rtu_master_validate_read_registers(
    uint8_t server_address,
    uint16_t start_address,
    uint16_t quantity,
    const uint16_t *registers)
{
    modbus_rtu_master_status_t status = MODBUS_RTU_MASTER_OK;

    if (registers == NULL)
    {
        return MODBUS_RTU_MASTER_E_NULL;
    }

    status = modbus_rtu_master_validate_server_address(server_address);
    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    if ((quantity == 0u) ||
        (quantity > MODBUS_RTU_MASTER_MAX_READ_REGISTERS))
    {
        return MODBUS_RTU_MASTER_E_PARAM;
    }

    return modbus_rtu_master_validate_address_range(
        start_address,
        quantity);
}

/**
 * @brief Validate parameters for Write Multiple Coils.
 */
static modbus_rtu_master_status_t modbus_rtu_master_validate_write_coils(
    uint8_t server_address,
    uint16_t start_address,
    uint16_t quantity,
    const uint8_t *coils)
{
    modbus_rtu_master_status_t status = MODBUS_RTU_MASTER_OK;

    if (coils == NULL)
    {
        return MODBUS_RTU_MASTER_E_NULL;
    }

    status = modbus_rtu_master_validate_server_address(server_address);
    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    if ((quantity == 0u) ||
        (quantity > MODBUS_RTU_MASTER_MAX_WRITE_COILS))
    {
        return MODBUS_RTU_MASTER_E_PARAM;
    }

    return modbus_rtu_master_validate_address_range(
        start_address,
        quantity);
}

/**
 * @brief Validate parameters for Write Multiple Registers.
 */
static modbus_rtu_master_status_t modbus_rtu_master_validate_write_registers(
    uint8_t server_address,
    uint16_t start_address,
    uint16_t quantity,
    const uint16_t *registers)
{
    modbus_rtu_master_status_t status = MODBUS_RTU_MASTER_OK;

    if (registers == NULL)
    {
        return MODBUS_RTU_MASTER_E_NULL;
    }

    status = modbus_rtu_master_validate_server_address(server_address);
    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    if ((quantity == 0u) ||
        (quantity > MODBUS_RTU_MASTER_MAX_WRITE_REGISTERS))
    {
        return MODBUS_RTU_MASTER_E_PARAM;
    }

    return modbus_rtu_master_validate_address_range(
        start_address,
        quantity);
}

/**
 * @brief Build an eight-byte fixed-format MODBUS RTU request.
 *
 * @details
 * Used by functions 0x01 through 0x06, whose request data contains a two-byte
 * address followed by a two-byte value or quantity.
 */
static void modbus_rtu_master_build_fixed_request(
    uint8_t server_address,
    uint8_t function_code,
    uint16_t address,
    uint16_t value_or_quantity,
    uint8_t *frame)
{
    frame[0] = server_address;
    frame[1] = function_code;
    frame[2] = (uint8_t)((address >> 8u) & 0x00FFu);
    frame[3] = (uint8_t)(address & 0x00FFu);
    frame[4] = (uint8_t)((value_or_quantity >> 8u) & 0x00FFu);
    frame[5] = (uint8_t)(value_or_quantity & 0x00FFu);

    modbus_rtu_master_append_crc(frame, 6u);
}

/**
 * @brief Build a Write Multiple Coils request.
 *
 * @return Total ADU size including CRC.
 */
static uint16_t modbus_rtu_master_build_write_multiple_coils_request(
    uint8_t server_address,
    uint16_t start_address,
    uint16_t quantity,
    const uint8_t *coils,
    uint8_t *frame)
{
    uint16_t coil_index = 0u;
    uint16_t data_byte_index = 0u;
    uint16_t byte_count = 0u;
    uint16_t frame_size_without_crc = 0u;
    uint8_t bit_index = 0u;

    byte_count = (quantity + 7u) / 8u;

    frame[0] = server_address;
    frame[1] = MODBUS_RTU_FC_WRITE_MULTIPLE_COILS;
    frame[2] = (uint8_t)((start_address >> 8u) & 0x00FFu);
    frame[3] = (uint8_t)(start_address & 0x00FFu);
    frame[4] = (uint8_t)((quantity >> 8u) & 0x00FFu);
    frame[5] = (uint8_t)(quantity & 0x00FFu);
    frame[6] = (uint8_t)byte_count;

    for (data_byte_index = 0u;
         data_byte_index < byte_count;
         data_byte_index++)
    {
        frame[MODBUS_RTU_WRITE_MULTIPLE_HEADER_SIZE + data_byte_index] = 0u;
    }

    for (coil_index = 0u; coil_index < quantity; coil_index++)
    {
        data_byte_index = coil_index / 8u;
        bit_index = (uint8_t)(coil_index % 8u);

        if (coils[coil_index] != 0u)
        {
            frame[MODBUS_RTU_WRITE_MULTIPLE_HEADER_SIZE + data_byte_index] |=
                (uint8_t)(1u << bit_index);
        }
    }

    frame_size_without_crc =
        MODBUS_RTU_WRITE_MULTIPLE_HEADER_SIZE +
        byte_count;

    modbus_rtu_master_append_crc(
        frame,
        frame_size_without_crc);

    return frame_size_without_crc + MODBUS_RTU_CRC_SIZE;
}

/**
 * @brief Build a Write Multiple Registers request.
 *
 * @return Total ADU size including CRC.
 */
static uint16_t modbus_rtu_master_build_write_multiple_registers_request(
    uint8_t server_address,
    uint16_t start_address,
    uint16_t quantity,
    const uint16_t *registers,
    uint8_t *frame)
{
    uint16_t register_index = 0u;
    uint16_t data_offset = 0u;
    uint16_t byte_count = 0u;
    uint16_t frame_size_without_crc = 0u;

    byte_count = 2u * quantity;

    frame[0] = server_address;
    frame[1] = MODBUS_RTU_FC_WRITE_MULTIPLE_REGISTERS;
    frame[2] = (uint8_t)((start_address >> 8u) & 0x00FFu);
    frame[3] = (uint8_t)(start_address & 0x00FFu);
    frame[4] = (uint8_t)((quantity >> 8u) & 0x00FFu);
    frame[5] = (uint8_t)(quantity & 0x00FFu);
    frame[6] = (uint8_t)byte_count;

    for (register_index = 0u;
         register_index < quantity;
         register_index++)
    {
        data_offset =
            MODBUS_RTU_WRITE_MULTIPLE_HEADER_SIZE +
            (2u * register_index);

        frame[data_offset] =
            (uint8_t)((registers[register_index] >> 8u) & 0x00FFu);

        frame[data_offset + 1u] =
            (uint8_t)(registers[register_index] & 0x00FFu);
    }

    frame_size_without_crc =
        MODBUS_RTU_WRITE_MULTIPLE_HEADER_SIZE +
        byte_count;

    modbus_rtu_master_append_crc(
        frame,
        frame_size_without_crc);

    return frame_size_without_crc + MODBUS_RTU_CRC_SIZE;
}

/**
 * @brief Calculate and append the RTU CRC.
 *
 * @details
 * The CRC low-order byte is appended first and the high-order byte second, as
 * required by MODBUS over Serial Line Specification and Implementation Guide
 * V1.02, section 2.5.1.2, pages 14 and 15.
 */
static void modbus_rtu_master_append_crc(
    uint8_t *frame,
    uint16_t frame_size_without_crc)
{
    uint16_t crc = 0u;

    crc = modbus_crc_calculate(
        frame,
        frame_size_without_crc);

    frame[frame_size_without_crc] =
        (uint8_t)(crc & 0x00FFu);

    frame[frame_size_without_crc + 1u] =
        (uint8_t)((crc >> 8u) & 0x00FFu);
}

/**
 * @brief Send a MODBUS RTU request through the RS485 driver.
 */
static modbus_rtu_master_status_t modbus_rtu_master_send_request(
    const uint8_t *frame,
    uint16_t frame_size,
    uint32_t timeout_ms)
{
    rs485_status_t rs485_status = RS485_OK;

    rs485_status = rs485_send(
        frame,
        frame_size,
        timeout_ms);

    return modbus_rtu_master_rs485_status_to_status(rs485_status);
}

/**
 * @brief Receive a normal or exception response for a read function.
 */
static modbus_rtu_master_status_t modbus_rtu_master_receive_read_response(
    uint8_t server_address,
    uint8_t function_code,
    uint16_t expected_byte_count,
    uint8_t *frame,
    uint16_t *frame_size,
    uint32_t timeout_ms)
{
    uint16_t remaining_size = 0u;
    rs485_status_t rs485_status = RS485_OK;
    modbus_rtu_master_status_t status = MODBUS_RTU_MASTER_OK;

    rs485_status = rs485_receive(
        frame,
        MODBUS_RTU_READ_RESPONSE_HEADER_SIZE,
        timeout_ms);

    status = modbus_rtu_master_rs485_status_to_status(rs485_status);
    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    if (frame[0] != server_address)
    {
        return MODBUS_RTU_MASTER_E_ADDRESS;
    }

    if ((frame[1] & MODBUS_RTU_EXCEPTION_FLAG) != 0u)
    {
        remaining_size =
            MODBUS_RTU_EXCEPTION_RESPONSE_SIZE -
            MODBUS_RTU_READ_RESPONSE_HEADER_SIZE;

        rs485_status = rs485_receive(
            &frame[MODBUS_RTU_READ_RESPONSE_HEADER_SIZE],
            remaining_size,
            timeout_ms);

        status = modbus_rtu_master_rs485_status_to_status(rs485_status);
        if (status != MODBUS_RTU_MASTER_OK)
        {
            return status;
        }

        return modbus_rtu_master_process_exception_response(
            frame,
            server_address,
            function_code);
    }

    if (frame[1] != function_code)
    {
        return MODBUS_RTU_MASTER_E_FUNCTION;
    }

    if ((uint16_t)frame[2] != expected_byte_count)
    {
        return MODBUS_RTU_MASTER_E_LENGTH;
    }

    remaining_size =
        expected_byte_count +
        MODBUS_RTU_CRC_SIZE;

    *frame_size =
        MODBUS_RTU_READ_RESPONSE_HEADER_SIZE +
        remaining_size;

    if (*frame_size > MODBUS_RTU_MAX_ADU_SIZE)
    {
        return MODBUS_RTU_MASTER_E_LENGTH;
    }

    rs485_status = rs485_receive(
        &frame[MODBUS_RTU_READ_RESPONSE_HEADER_SIZE],
        remaining_size,
        timeout_ms);

    return modbus_rtu_master_rs485_status_to_status(rs485_status);
}

/**
 * @brief Receive a normal or exception response for a write function.
 */
static modbus_rtu_master_status_t modbus_rtu_master_receive_write_response(
    uint8_t server_address,
    uint8_t function_code,
    uint8_t *frame,
    uint32_t timeout_ms)
{
    uint16_t remaining_size = 0u;
    rs485_status_t rs485_status = RS485_OK;
    modbus_rtu_master_status_t status = MODBUS_RTU_MASTER_OK;

    rs485_status = rs485_receive(
        frame,
        MODBUS_RTU_READ_RESPONSE_HEADER_SIZE,
        timeout_ms);

    status = modbus_rtu_master_rs485_status_to_status(rs485_status);
    if (status != MODBUS_RTU_MASTER_OK)
    {
        return status;
    }

    if (frame[0] != server_address)
    {
        return MODBUS_RTU_MASTER_E_ADDRESS;
    }

    if ((frame[1] & MODBUS_RTU_EXCEPTION_FLAG) != 0u)
    {
        remaining_size =
            MODBUS_RTU_EXCEPTION_RESPONSE_SIZE -
            MODBUS_RTU_READ_RESPONSE_HEADER_SIZE;

        rs485_status = rs485_receive(
            &frame[MODBUS_RTU_READ_RESPONSE_HEADER_SIZE],
            remaining_size,
            timeout_ms);

        status = modbus_rtu_master_rs485_status_to_status(rs485_status);
        if (status != MODBUS_RTU_MASTER_OK)
        {
            return status;
        }

        return modbus_rtu_master_process_exception_response(
            frame,
            server_address,
            function_code);
    }

    if (frame[1] != function_code)
    {
        return MODBUS_RTU_MASTER_E_FUNCTION;
    }

    remaining_size =
        MODBUS_RTU_WRITE_RESPONSE_SIZE -
        MODBUS_RTU_READ_RESPONSE_HEADER_SIZE;

    rs485_status = rs485_receive(
        &frame[MODBUS_RTU_READ_RESPONSE_HEADER_SIZE],
        remaining_size,
        timeout_ms);

    return modbus_rtu_master_rs485_status_to_status(rs485_status);
}

/**
 * @brief Validate and store a MODBUS exception response.
 */
static modbus_rtu_master_status_t modbus_rtu_master_process_exception_response(
    const uint8_t *frame,
    uint8_t server_address,
    uint8_t function_code)
{
    uint8_t expected_exception_function = 0u;

    if (modbus_rtu_master_is_crc_valid(
            frame,
            MODBUS_RTU_EXCEPTION_RESPONSE_SIZE) == false)
    {
        return MODBUS_RTU_MASTER_E_CRC;
    }

    if (frame[0] != server_address)
    {
        return MODBUS_RTU_MASTER_E_ADDRESS;
    }

    expected_exception_function =
        function_code |
        MODBUS_RTU_EXCEPTION_FLAG;

    if (frame[1] != expected_exception_function)
    {
        return MODBUS_RTU_MASTER_E_FUNCTION;
    }

    g_ctx.last_exception_code =
        (modbus_exception_code_t)frame[2];

    return MODBUS_RTU_MASTER_E_EXCEPTION;
}

/**
 * @brief Validate a normal read response.
 */
static modbus_rtu_master_status_t modbus_rtu_master_validate_read_response(
    const uint8_t *frame,
    uint16_t frame_size,
    uint8_t server_address,
    uint8_t function_code,
    uint16_t expected_byte_count)
{
    uint16_t expected_frame_size = 0u;

    expected_frame_size =
        MODBUS_RTU_READ_RESPONSE_HEADER_SIZE +
        expected_byte_count +
        MODBUS_RTU_CRC_SIZE;

    if (frame_size != expected_frame_size)
    {
        return MODBUS_RTU_MASTER_E_LENGTH;
    }

    if (modbus_rtu_master_is_crc_valid(frame, frame_size) == false)
    {
        return MODBUS_RTU_MASTER_E_CRC;
    }

    if (frame[0] != server_address)
    {
        return MODBUS_RTU_MASTER_E_ADDRESS;
    }

    if (frame[1] != function_code)
    {
        return MODBUS_RTU_MASTER_E_FUNCTION;
    }

    if ((uint16_t)frame[2] != expected_byte_count)
    {
        return MODBUS_RTU_MASTER_E_LENGTH;
    }

    return MODBUS_RTU_MASTER_OK;
}

/**
 * @brief Validate a normal fixed-length write response.
 *
 * @details
 * Functions 0x05 and 0x06 echo the target address and written value.
 * Functions 0x0F and 0x10 return the starting address and written quantity.
 */
static modbus_rtu_master_status_t modbus_rtu_master_validate_write_response(
    const uint8_t *frame,
    uint8_t server_address,
    uint8_t function_code,
    uint16_t expected_address,
    uint16_t expected_value)
{
    uint16_t received_address = 0u;
    uint16_t received_value = 0u;

    if (modbus_rtu_master_is_crc_valid(
            frame,
            MODBUS_RTU_WRITE_RESPONSE_SIZE) == false)
    {
        return MODBUS_RTU_MASTER_E_CRC;
    }

    if (frame[0] != server_address)
    {
        return MODBUS_RTU_MASTER_E_ADDRESS;
    }

    if (frame[1] != function_code)
    {
        return MODBUS_RTU_MASTER_E_FUNCTION;
    }

    received_address =
        ((uint16_t)frame[2] << 8u) |
        (uint16_t)frame[3];

    received_value =
        ((uint16_t)frame[4] << 8u) |
        (uint16_t)frame[5];

    if ((received_address != expected_address) ||
        (received_value != expected_value))
    {
        return MODBUS_RTU_MASTER_E_DATA;
    }

    return MODBUS_RTU_MASTER_OK;
}

/**
 * @brief Unpack MODBUS bit data into one byte per value.
 */
static void modbus_rtu_master_unpack_bits(
    const uint8_t *packed_data,
    uint16_t quantity,
    uint8_t *values)
{
    uint16_t value_index = 0u;
    uint16_t byte_index = 0u;
    uint8_t bit_index = 0u;

    for (value_index = 0u; value_index < quantity; value_index++)
    {
        byte_index = value_index / 8u;
        bit_index = (uint8_t)(value_index % 8u);

        values[value_index] =
            (uint8_t)((packed_data[byte_index] >> bit_index) & 0x01u);
    }
}

/**
 * @brief Validate the CRC of a complete MODBUS RTU ADU.
 */
static bool modbus_rtu_master_is_crc_valid(
    const uint8_t *frame,
    uint16_t frame_size)
{
    uint16_t calculated_crc = 0u;
    uint16_t received_crc = 0u;

    if ((frame == NULL) || (frame_size < MODBUS_RTU_CRC_SIZE))
    {
        return false;
    }

    calculated_crc = modbus_crc_calculate(
        frame,
        frame_size - MODBUS_RTU_CRC_SIZE);

    received_crc =
        (uint16_t)frame[frame_size - 2u] |
        ((uint16_t)frame[frame_size - 1u] << 8u);

    return calculated_crc == received_crc;
}

/**
 * @brief Convert RS485 status codes to MODBUS master status codes.
 */
static modbus_rtu_master_status_t modbus_rtu_master_rs485_status_to_status(
    rs485_status_t rs485_status)
{
    modbus_rtu_master_status_t status = MODBUS_RTU_MASTER_OK;

    switch (rs485_status)
    {
        case RS485_OK:
            status = MODBUS_RTU_MASTER_OK;
            break;

        case RS485_E_NULL:
            status = MODBUS_RTU_MASTER_E_NULL;
            break;

        case RS485_E_PARAM:
            status = MODBUS_RTU_MASTER_E_PARAM;
            break;

        case RS485_E_STATE:
            status = MODBUS_RTU_MASTER_E_STATE;
            break;

        case RS485_E_TIMEOUT:
            status = MODBUS_RTU_MASTER_E_TIMEOUT;
            break;

        case RS485_E_HW:
        default:
            status = MODBUS_RTU_MASTER_E_HW;
            break;
    }

    return status;
}

/** @} */
