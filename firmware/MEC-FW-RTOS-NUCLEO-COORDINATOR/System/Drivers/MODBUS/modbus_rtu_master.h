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
 * @file    modbus_rtu_master.h
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-08-02
 * @brief   Basic blocking MODBUS RTU master implementation.
 *
 * @details
 * This module implements the most commonly used public MODBUS data-access
 * function codes over the RS485 driver layer:
 *
 * - 01 (0x01): Read Coils.
 * - 02 (0x02): Read Discrete Inputs.
 * - 03 (0x03): Read Holding Registers.
 * - 04 (0x04): Read Input Registers.
 * - 05 (0x05): Write Single Coil.
 * - 06 (0x06): Write Single Register.
 * - 15 (0x0F): Write Multiple Coils.
 * - 16 (0x10): Write Multiple Registers.
 *
 * Function formats, quantity limits, bit packing and register encoding follow
 * MODBUS Application Protocol Specification V1.1b3, sections 4.2, 6.1 to 6.6,
 * 6.11 and 6.12 (pages 5, 11 to 20 and 29 to 31).
 *
 * MODBUS RTU framing, CRC byte order and maximum ADU length follow MODBUS over
 * Serial Line Specification and Implementation Guide V1.02, sections 2.3,
 * 2.5.1, 2.5.1.1 and 2.5.1.2 (pages 8 and 12 to 15).
 *
 * This implementation supports unicast transactions only. Broadcast address 0
 * is intentionally rejected because the current lower layers do not provide a
 * transport-independent turnaround-delay service. A compliant broadcast write
 * implementation must send no reply and must prevent a new transaction until
 * the configured turnaround delay has elapsed.
 *
 * Only one transaction may be active at a time. The module is blocking and is
 * not thread-safe. Concurrency protection, retries, request queues and
 * asynchronous operation must be implemented by an upper layer if required.
 *
 * @ingroup modbus_rtu_master
 * @{
 */

#ifndef MODBUS_RTU_MASTER_H_
#define MODBUS_RTU_MASTER_H_

/* ============================= Includes ================================== */

#include <stdbool.h>
#include <stdint.h>

/* ============================== Macros =================================== */

/** @brief Minimum valid unicast server address. */
#define MODBUS_RTU_MASTER_MIN_SERVER_ADDRESS             (1u)

/** @brief Maximum valid unicast server address. */
#define MODBUS_RTU_MASTER_MAX_SERVER_ADDRESS             (247u)

/**
 * @brief Maximum number of coils accepted by function code 0x01.
 *
 * See MODBUS Application Protocol Specification V1.1b3, section 6.1,
 * pages 11 and 12.
 */
#define MODBUS_RTU_MASTER_MAX_READ_COILS                 (2000u)

/**
 * @brief Maximum number of discrete inputs accepted by function code 0x02.
 *
 * See MODBUS Application Protocol Specification V1.1b3, section 6.2,
 * pages 12 to 14.
 */
#define MODBUS_RTU_MASTER_MAX_READ_DISCRETE_INPUTS       (2000u)

/**
 * @brief Maximum number of registers accepted by function codes 0x03 and 0x04.
 *
 * See MODBUS Application Protocol Specification V1.1b3, sections 6.3 and 6.4,
 * pages 15 to 17.
 */
#define MODBUS_RTU_MASTER_MAX_READ_REGISTERS             (125u)

/**
 * @brief Maximum number of coils accepted by function code 0x0F.
 *
 * See MODBUS Application Protocol Specification V1.1b3, section 6.11,
 * pages 29 and 30.
 */
#define MODBUS_RTU_MASTER_MAX_WRITE_COILS                (1968u)

/**
 * @brief Maximum number of registers accepted by function code 0x10.
 *
 * See MODBUS Application Protocol Specification V1.1b3, section 6.12,
 * pages 30 and 31.
 */
#define MODBUS_RTU_MASTER_MAX_WRITE_REGISTERS            (123u)

/* ============================== Types ==================================== */

/**
 * @brief Public status codes for the MODBUS RTU master.
 */
typedef enum
{
    MODBUS_RTU_MASTER_OK = 0,
    MODBUS_RTU_MASTER_E_NULL,
    MODBUS_RTU_MASTER_E_PARAM,
    MODBUS_RTU_MASTER_E_STATE,
    MODBUS_RTU_MASTER_E_HW,
    MODBUS_RTU_MASTER_E_TIMEOUT,
    MODBUS_RTU_MASTER_E_CRC,
    MODBUS_RTU_MASTER_E_ADDRESS,
    MODBUS_RTU_MASTER_E_FUNCTION,
    MODBUS_RTU_MASTER_E_LENGTH,
    MODBUS_RTU_MASTER_E_DATA,
    MODBUS_RTU_MASTER_E_EXCEPTION
} modbus_rtu_master_status_t;

/**
 * @brief Standard MODBUS exception codes.
 *
 * See MODBUS Application Protocol Specification V1.1b3, section 7,
 * pages 47 to 49.
 */
typedef enum
{
    MODBUS_EXCEPTION_NONE                        = 0x00u,
    MODBUS_EXCEPTION_ILLEGAL_FUNCTION            = 0x01u,
    MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS        = 0x02u,
    MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE          = 0x03u,
    MODBUS_EXCEPTION_SERVER_DEVICE_FAILURE       = 0x04u,
    MODBUS_EXCEPTION_ACKNOWLEDGE                 = 0x05u,
    MODBUS_EXCEPTION_SERVER_DEVICE_BUSY          = 0x06u,
    MODBUS_EXCEPTION_MEMORY_PARITY_ERROR         = 0x08u,
    MODBUS_EXCEPTION_GATEWAY_PATH_UNAVAILABLE    = 0x0Au,
    MODBUS_EXCEPTION_GATEWAY_TARGET_NO_RESPONSE  = 0x0Bu
} modbus_exception_code_t;

/* ===================== Public Function Prototypes ======================== */

/**
 * @brief Initialize the MODBUS RTU master.
 *
 * @return MODBUS_RTU_MASTER_OK on success, error code otherwise.
 */
modbus_rtu_master_status_t modbus_rtu_master_init(void);

/**
 * @brief Deinitialize the MODBUS RTU master.
 *
 * @return MODBUS_RTU_MASTER_OK on success, error code otherwise.
 */
modbus_rtu_master_status_t modbus_rtu_master_deinit(void);

/**
 * @brief Read contiguous coils from a remote MODBUS server.
 *
 * @details
 * Implements function 01 (0x01), Read Coils, as specified in MODBUS
 * Application Protocol Specification V1.1b3, section 6.1, pages 11 and 12.
 *
 * Response bits are unpacked into one byte per coil. A value of 0u represents
 * OFF and a value of 1u represents ON. The first requested coil is taken from
 * the least significant bit of the first response data byte.
 *
 * @param[in]  server_address  Server address in the range 1 to 247.
 * @param[in]  start_address   Zero-based address of the first coil.
 * @param[in]  quantity        Number of coils to read, from 1 to 2000.
 * @param[out] coils           Destination buffer with @p quantity bytes.
 * @param[in]  timeout_ms      Timeout for each blocking RS485 operation.
 *
 * @return MODBUS_RTU_MASTER_OK on success, error code otherwise.
 */
modbus_rtu_master_status_t modbus_rtu_master_read_coils(
    uint8_t server_address,
    uint16_t start_address,
    uint16_t quantity,
    uint8_t *coils,
    uint32_t timeout_ms);

/**
 * @brief Read contiguous discrete inputs from a remote MODBUS server.
 *
 * @details
 * Implements function 02 (0x02), Read Discrete Inputs, as specified in MODBUS
 * Application Protocol Specification V1.1b3, section 6.2, pages 12 to 14.
 *
 * Response bits are unpacked into one byte per input. A value of 0u represents
 * OFF and a value of 1u represents ON.
 *
 * @param[in]  server_address  Server address in the range 1 to 247.
 * @param[in]  start_address   Zero-based address of the first discrete input.
 * @param[in]  quantity        Number of inputs to read, from 1 to 2000.
 * @param[out] inputs          Destination buffer with @p quantity bytes.
 * @param[in]  timeout_ms      Timeout for each blocking RS485 operation.
 *
 * @return MODBUS_RTU_MASTER_OK on success, error code otherwise.
 */
modbus_rtu_master_status_t modbus_rtu_master_read_discrete_inputs(
    uint8_t server_address,
    uint16_t start_address,
    uint16_t quantity,
    uint8_t *inputs,
    uint32_t timeout_ms);

/**
 * @brief Read contiguous holding registers from a remote MODBUS server.
 *
 * @details
 * Implements function 03 (0x03), Read Holding Registers, as specified in
 * MODBUS Application Protocol Specification V1.1b3, section 6.3,
 * pages 15 and 16.
 *
 * @param[in]  server_address  Server address in the range 1 to 247.
 * @param[in]  start_address   Zero-based address of the first register.
 * @param[in]  quantity        Number of registers to read, from 1 to 125.
 * @param[out] registers       Destination buffer with @p quantity values.
 * @param[in]  timeout_ms      Timeout for each blocking RS485 operation.
 *
 * @return MODBUS_RTU_MASTER_OK on success, error code otherwise.
 */
modbus_rtu_master_status_t modbus_rtu_master_read_holding_registers(
    uint8_t server_address,
    uint16_t start_address,
    uint16_t quantity,
    uint16_t *registers,
    uint32_t timeout_ms);

/**
 * @brief Read contiguous input registers from a remote MODBUS server.
 *
 * @details
 * Implements function 04 (0x04), Read Input Registers, as specified in MODBUS
 * Application Protocol Specification V1.1b3, section 6.4, pages 16 and 17.
 *
 * @param[in]  server_address  Server address in the range 1 to 247.
 * @param[in]  start_address   Zero-based address of the first input register.
 * @param[in]  quantity        Number of registers to read, from 1 to 125.
 * @param[out] registers       Destination buffer with @p quantity values.
 * @param[in]  timeout_ms      Timeout for each blocking RS485 operation.
 *
 * @return MODBUS_RTU_MASTER_OK on success, error code otherwise.
 */
modbus_rtu_master_status_t modbus_rtu_master_read_input_registers(
    uint8_t server_address,
    uint16_t start_address,
    uint16_t quantity,
    uint16_t *registers,
    uint32_t timeout_ms);

/**
 * @brief Write one coil in a remote MODBUS server.
 *
 * @details
 * Implements function 05 (0x05), Write Single Coil, as specified in MODBUS
 * Application Protocol Specification V1.1b3, section 6.5, pages 17 to 19.
 * The protocol value 0xFF00 is used for ON and 0x0000 for OFF. The normal
 * response must echo the request address and value.
 *
 * @param[in] server_address  Server address in the range 1 to 247.
 * @param[in] coil_address    Zero-based coil address.
 * @param[in] value           true for ON, false for OFF.
 * @param[in] timeout_ms      Timeout for each blocking RS485 operation.
 *
 * @return MODBUS_RTU_MASTER_OK on success, error code otherwise.
 */
modbus_rtu_master_status_t modbus_rtu_master_write_single_coil(
    uint8_t server_address,
    uint16_t coil_address,
    bool value,
    uint32_t timeout_ms);

/**
 * @brief Write one holding register in a remote MODBUS server.
 *
 * @details
 * Implements function 06 (0x06), Write Single Register, as specified in
 * MODBUS Application Protocol Specification V1.1b3, section 6.6,
 * pages 19 and 20. The normal response must echo the request address and value.
 *
 * @param[in] server_address   Server address in the range 1 to 247.
 * @param[in] register_address Zero-based holding-register address.
 * @param[in] value            Register value to write.
 * @param[in] timeout_ms       Timeout for each blocking RS485 operation.
 *
 * @return MODBUS_RTU_MASTER_OK on success, error code otherwise.
 */
modbus_rtu_master_status_t modbus_rtu_master_write_single_register(
    uint8_t server_address,
    uint16_t register_address,
    uint16_t value,
    uint32_t timeout_ms);

/**
 * @brief Write contiguous coils in a remote MODBUS server.
 *
 * @details
 * Implements function 15 (0x0F), Write Multiple Coils, as specified in MODBUS
 * Application Protocol Specification V1.1b3, section 6.11,
 * pages 29 and 30.
 *
 * Input values are supplied as one byte per coil and packed into the request.
 * A zero value represents OFF; any nonzero value represents ON. Unused
 * high-order bits in the final request data byte are written as zero.
 *
 * @param[in] server_address  Server address in the range 1 to 247.
 * @param[in] start_address   Zero-based address of the first coil.
 * @param[in] quantity        Number of coils to write, from 1 to 1968.
 * @param[in] coils           Source buffer containing @p quantity values.
 * @param[in] timeout_ms      Timeout for each blocking RS485 operation.
 *
 * @return MODBUS_RTU_MASTER_OK on success, error code otherwise.
 */
modbus_rtu_master_status_t modbus_rtu_master_write_multiple_coils(
    uint8_t server_address,
    uint16_t start_address,
    uint16_t quantity,
    const uint8_t *coils,
    uint32_t timeout_ms);

/**
 * @brief Write contiguous holding registers in a remote MODBUS server.
 *
 * @details
 * Implements function 16 (0x10), Write Multiple Registers, as specified in
 * MODBUS Application Protocol Specification V1.1b3, section 6.12,
 * pages 30 and 31.
 *
 * @param[in] server_address  Server address in the range 1 to 247.
 * @param[in] start_address   Zero-based address of the first register.
 * @param[in] quantity        Number of registers to write, from 1 to 123.
 * @param[in] registers       Source buffer containing @p quantity values.
 * @param[in] timeout_ms      Timeout for each blocking RS485 operation.
 *
 * @return MODBUS_RTU_MASTER_OK on success, error code otherwise.
 */
modbus_rtu_master_status_t modbus_rtu_master_write_multiple_registers(
    uint8_t server_address,
    uint16_t start_address,
    uint16_t quantity,
    const uint16_t *registers,
    uint32_t timeout_ms);

/**
 * @brief Get the last MODBUS exception code received from a server.
 *
 * @return Last exception code, or MODBUS_EXCEPTION_NONE if no exception was
 *         recorded for the most recent transaction.
 */
modbus_exception_code_t modbus_rtu_master_get_last_exception_code(void);

#endif /* MODBUS_RTU_MASTER_H_ */

/** @} */
