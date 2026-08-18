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
 * @file    sclt38bt8.c
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-04-21
 * @brief   High-level driver for the ST SCLT3-8BT8 digital input serializer.
 *
 * @details
 * Concurrency model:
 *  - Blocking SPI transfers through the shared io_port layer.
 *  - No internal state is retained by this driver.
 *
 * The device is used in 16-bit SPI mode. In this mode, one SPI frame contains:
 *  - 8 bits of input data
 *  - 4 parity bits
 *  - 1 undervoltage alarm bit (/UVA)
 *  - 1 overtemperature alarm bit (/OTA)
 *  - 2 stop bits
 *
 * This driver reads the full frame, validates stop bits and parity, checks the
 * diagnostic bits, and returns either the decoded input byte or the first
 * detected fault condition.
 *
 * @ingroup sclt38bt8
 * @{
 */

/* ============================= Includes ================================== */

#include "sclt38bt8.h"
#include "io_port.h"

#include <stdint.h>
#include <stdbool.h>

#include "main.h"

#include "swo.h"

/* ============================ Local Macros =============================== */

/**
 * @brief SPI frame length in bytes for 16-bit mode.
 */
#define SCLT38BT8_FRAME_SIZE_BYTES         ((uint8_t)2u)

/**
 * @brief Dummy byte transmitted while receiving data from the device.
 */
#define SCLT38BT8_SPI_DUMMY_BYTE           ((uint8_t)0x00u)

/**
 * @brief Raw frame masks.
 */
#define SCLT38BT8_INPUTS_MASK              ((uint16_t)0xFF00u)
#define SCLT38BT8_CONTROL_MASK             ((uint16_t)0x00FFu)

/**
 * @brief Stop-bit field masks and expected value.
 *
 * @details
 * In 16-bit mode, the control byte ends with:
 * - bit 1 = low-state stop bit
 * - bit 0 = high-state stop bit
 *
 * Therefore, the expected two-bit pattern is binary 01.
 */
#define SCLT38BT8_STOP_BITS_MASK           ((uint8_t)0x03u)
#define SCLT38BT8_STOP_BITS_EXPECTED       ((uint8_t)0x01u)

/**
 * @brief Control-byte bit masks.
 */
#define SCLT38BT8_CTRL_UVA_N_MASK          ((uint8_t)(1u << 7))
#define SCLT38BT8_CTRL_OTA_N_MASK          ((uint8_t)(1u << 6))
#define SCLT38BT8_CTRL_PC1_MASK            ((uint8_t)(1u << 5))
#define SCLT38BT8_CTRL_PC2_MASK            ((uint8_t)(1u << 4))
#define SCLT38BT8_CTRL_PC3_MASK            ((uint8_t)(1u << 3))
#define SCLT38BT8_CTRL_PC4_MASK            ((uint8_t)(1u << 2))
#define SCLT38BT8_CTRL_STOP_LOW_MASK       ((uint8_t)(1u << 1))
#define SCLT38BT8_CTRL_STOP_HIGH_MASK      ((uint8_t)(1u << 0))

/* ========================== Private Prototypes =========================== */

static sclt38bt8_status_t sclt38bt8_from_port_status(io_port_status_t status);
static uint16_t sclt38bt8_build_word(const uint8_t * const p_rx_buffer);
static bool sclt38bt8_check_stop_bits(uint8_t control_raw);
static bool sclt38bt8_check_parity(uint8_t inputs, uint8_t control_raw);
static uint8_t sclt38bt8_calculate_pc1(uint8_t inputs);
static uint8_t sclt38bt8_calculate_pc2(uint8_t inputs);
static uint8_t sclt38bt8_calculate_pc3(uint8_t inputs);
static uint8_t sclt38bt8_calculate_pc4(uint8_t inputs);

/* ===================== Public Function Definitions ======================= */

sclt38bt8_status_t sclt38bt8_read_inputs(uint8_t * const p_inputs)
{
    sclt38bt8_status_t status;
    uint8_t rx_buffer[SCLT38BT8_FRAME_SIZE_BYTES];
    uint16_t raw_word;
    uint8_t inputs;
    uint8_t control_raw;
    bool undervoltage_alarm;
    bool overtemperature_alarm;
    bool power_loss;

    if (p_inputs == NULL)
    {
        return SCLT38BT8_E_NULL;
    }

    status = sclt38bt8_from_port_status(
        io_port_receive(IO_PORT_DEVICE_SCLT3,
                        rx_buffer,
                        SCLT38BT8_FRAME_SIZE_BYTES));

    if (status != SCLT38BT8_OK)
    {
        return status;
    }

    raw_word = sclt38bt8_build_word(rx_buffer);

    inputs = (uint8_t)((raw_word & SCLT38BT8_INPUTS_MASK) >> 8u);
	//printf("Inputs: 0x%02X\r\n", inputs);

	if(inputs != 0x00)
	{
		HAL_GPIO_WritePin(GPIO0_GPIO_Port, GPIO0_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIO0_GPIO_Port, GPIO0_Pin, GPIO_PIN_RESET);
	}

    control_raw = (uint8_t)(raw_word & SCLT38BT8_CONTROL_MASK);

    /* Validation order:
     * 1. Power-loss detection
     * 2. Stop-bit format validation
     * 3. Parity/integrity check of the received frame
     * 4. Undervoltage alarm
     * 5. Overtemperature alarm
     *
     * This order gives priority to conditions that invalidate the whole SPI
     * frame before evaluating diagnostic bits that are only meaningful if the
     * received frame can still be trusted.
     */

    /* Detect loss of VCC power supply.
     *
     * According to the datasheet, if the VCC supply is lost, the device enters
     * sleep mode and the MISO output is forced low during a SPI transfer
     * attempt.
     *
     * The last SPI control data bit is a stop bit that is normally held high.
     * Therefore, loss of power supply is detected by checking this stop bit:
     * if it is low, the output is disabled by the internal power reset (POR).
     *
     * The datasheet states that POR becomes active low when:
     * - VC is less than 9 V, or
     * - the internal VDD is less than 3.25 V.
     *
     * In this condition, the returned SPI frame cannot be considered valid for
     * further interpretation.
     */
    power_loss = ((control_raw & SCLT38BT8_CTRL_STOP_HIGH_MASK) == 0u);
    if (power_loss == true)
    {
        return SCLT38BT8_E_POWER_LOSS;
    }

    /* Validate the stop bits of the received 16-bit frame.
     *
     * In 16-bit mode, the SPI word ends with two stop bits in the control
     * field:
     * - one low-state stop bit
     * - one high-state stop bit
     *
     * These bits define the expected frame format. If they do not match the
     * expected pattern, the received control byte is considered malformed and
     * must not be trusted.
     */
    if (sclt38bt8_check_stop_bits(control_raw) == false)
    {
        return SCLT38BT8_E_STOP_BITS;
    }

    /* Check parity bits of the received frame.
     *
     * The datasheet defines four parity checksum bits, transmitted in control
     * bits #2 to #5, whose purpose is to detect one error in the transferred
     * SPI word.
     *
     * These parity bits are generated through datasheet-defined XNOR
     * ("exclusive NOR") combinations of the input-state bits.
     *
     * If the received parity bits do not match the values recalculated from
     * the received input byte, the returned SPI frame cannot be considered
     * reliable for further diagnostic interpretation.
     */
    if (sclt38bt8_check_parity(inputs, control_raw) == false)
    {
        return SCLT38BT8_E_PARITY;
    }

    /* Check undervoltage alarm status.
     *
     * The datasheet states that /UVA is an active-low alarm generated by the
     * power bus voltage monitoring circuit.
     *
     * /UVA becomes active low when the power bus voltage falls below the
     * activation threshold VCON, typically 17 V.
     *
     * The alarm is released and returns high when the voltage rises above the
     * deactivation threshold VCOFF, typically 18 V.
     */
    undervoltage_alarm = ((control_raw & SCLT38BT8_CTRL_UVA_N_MASK) == 0u);
    if (undervoltage_alarm == true)
    {
        return SCLT38BT8_E_UV;
    }

    /* Check overtemperature alarm status.
     *
     * The datasheet states that /OTA is an active-low overtemperature alarm.
     *
     * /OTA becomes active low when the device junction temperature exceeds the
     * activation threshold TON, typically 150 °C.
     *
     * The alarm is released when the temperature falls below the reset
     * threshold TOFF, typically 140 °C.
     */
    overtemperature_alarm = ((control_raw & SCLT38BT8_CTRL_OTA_N_MASK) == 0u);
    if (overtemperature_alarm == true)
    {
        return SCLT38BT8_E_OT;
    }

    *p_inputs = inputs;

    return SCLT38BT8_OK;
}

/* ===================== Private Function Definitions ====================== */

/**
 * @brief Translate a port-layer status code to the driver-layer status space.
 *
 * @param status Status code returned by the shared I/O port layer.
 *
 * @return Equivalent status code in the @ref sclt38bt8_status_t domain.
 *
 * @details
 * This helper preserves the abstraction boundary between the driver and the
 * underlying port layer by converting low-level access results into the
 * public driver status type.
 *
 * Any unknown port-layer status is conservatively mapped to
 * SCLT38BT8_E_HW.
 */
static sclt38bt8_status_t sclt38bt8_from_port_status(io_port_status_t status)
{
    sclt38bt8_status_t ret;

    switch (status)
    {
        case IO_PORT_OK:
            ret = SCLT38BT8_OK;
            break;

        case IO_PORT_E_NULL:
            ret = SCLT38BT8_E_NULL;
            break;

        case IO_PORT_E_PARAM:
            ret = SCLT38BT8_E_PARAM;
            break;

        case IO_PORT_E_HW:
            ret = SCLT38BT8_E_HW;
            break;

        case IO_PORT_E_TIMEOUT:
            ret = SCLT38BT8_E_TIMEOUT;
            break;

        case IO_PORT_E_BUSY:
        default:
            ret = SCLT38BT8_E_HW;
            break;
    }

    return ret;
}

/**
 * @brief Assemble the received SPI bytes into a 16-bit word.
 *
 * @param p_rx_buffer Pointer to the two-byte receive buffer.
 *
 * @return Raw 16-bit frame.
 *
 * @details
 * The SCLT3-8BT8 shifts out the most significant bit first. Therefore:
 * - the first received byte contains IN8..IN1
 * - the second received byte contains control bits
 */
static uint16_t sclt38bt8_build_word(const uint8_t * const p_rx_buffer)
{
    uint16_t raw_word;

    raw_word  = ((uint16_t)p_rx_buffer[0] << 8u);
    raw_word |=  (uint16_t)p_rx_buffer[1];

    return raw_word;
}

/**
 * @brief Validate the stop bits of the returned 16-bit SPI control field.
 *
 * @param control_raw Raw control byte extracted from the returned SPI frame.
 *
 * @return
 * - true: Returned stop bits are valid.
 * - false: Returned stop bits are invalid.
 *
 * @details
 * According to the SCLT3-8BT8 datasheet, in 16-bit SPI mode the transferred
 * frame ends with two stop bits placed in the control field:
 * - control bit #0: high-state stop bit
 * - control bit #1: low-state stop bit
 *
 * Therefore, the expected stop-bit pattern in the returned control byte is:
 *
 * stop[1:0] = 0b01
 *
 * If this pattern is not respected, the returned SPI frame must be considered
 * malformed and must not be trusted for further diagnostic interpretation.
 */
static bool sclt38bt8_check_stop_bits(uint8_t control_raw)
{
    return ((control_raw & SCLT38BT8_STOP_BITS_MASK) ==
            SCLT38BT8_STOP_BITS_EXPECTED);
}

/**
 * @brief Validate the parity checksum bits of the returned 16-bit SPI frame.
 *
 * @param inputs Input-state byte IN8..IN1 extracted from the returned frame.
 * @param control_raw Raw control/parity byte extracted from the returned frame.
 *
 * @return
 * - true: Returned frame parity is valid.
 * - false: Returned frame parity is invalid.
 *
 * @details
 * According to the SCLT3-8BT8 datasheet, the parity checksum bits are used to
 * detect a single error in the transferred SPI word.
 *
 * The parity bits are transmitted in control bits #2 to #5 and are generated
 * through XNOR operations over specific groups of input-state bits.
 *
 * For mathematical consistency, this driver expresses these equations using
 * XOR and inversion:
 *
 * - PC1 = NOT(IN8 XOR IN7 XOR IN6 XOR IN5 XOR IN4 XOR IN3 XOR IN2 XOR IN1)
 * - PC2 = NOT(IN8 XOR IN7 XOR IN6 XOR IN5)
 * - PC3 = NOT(IN4 XOR IN3 XOR IN2 XOR IN1)
 * - PC4 = NOT(IN6 XOR IN5 XOR IN4 XOR IN3)
 *
 * If any returned parity bit does not match the value recalculated from the
 * received input byte, the returned SPI frame cannot be considered reliable
 * for further diagnostic interpretation.
 */
static bool sclt38bt8_check_parity(uint8_t inputs, uint8_t control_raw)
{
    uint8_t pc1;
    uint8_t pc2;
    uint8_t pc3;
    uint8_t pc4;

    pc1 = sclt38bt8_calculate_pc1(inputs);
    pc2 = sclt38bt8_calculate_pc2(inputs);
    pc3 = sclt38bt8_calculate_pc3(inputs);
    pc4 = sclt38bt8_calculate_pc4(inputs);

    if (((control_raw & SCLT38BT8_CTRL_PC1_MASK) >> 5u) != pc1)
    {
        return false;
    }

    if (((control_raw & SCLT38BT8_CTRL_PC2_MASK) >> 4u) != pc2)
    {
        return false;
    }

    if (((control_raw & SCLT38BT8_CTRL_PC3_MASK) >> 3u) != pc3)
    {
        return false;
    }

    if (((control_raw & SCLT38BT8_CTRL_PC4_MASK) >> 2u) != pc4)
    {
        return false;
    }

    return true;
}

/**
 * @brief Calculate returned-frame parity bit PC1.
 *
 * @param inputs Input-state byte IN8..IN1.
 *
 * @return Calculated returned-frame PC1 bit.
 *
 * @details
 * According to the SCLT3-8BT8 datasheet, PC1 is generated as an XNOR over all
 * input-state bits.
 *
 * Expressed using XOR:
 *
 * PC1 = NOT(IN8 XOR IN7 XOR IN6 XOR IN5 XOR IN4 XOR IN3 XOR IN2 XOR IN1)
 */
static uint8_t sclt38bt8_calculate_pc1(uint8_t inputs)
{
    uint8_t bit;

    bit  = (uint8_t)((inputs >> 7u) & 0x01u);
    bit ^= (uint8_t)((inputs >> 6u) & 0x01u);
    bit ^= (uint8_t)((inputs >> 5u) & 0x01u);
    bit ^= (uint8_t)((inputs >> 4u) & 0x01u);
    bit ^= (uint8_t)((inputs >> 3u) & 0x01u);
    bit ^= (uint8_t)((inputs >> 2u) & 0x01u);
    bit ^= (uint8_t)((inputs >> 1u) & 0x01u);
    bit ^= (uint8_t)((inputs >> 0u) & 0x01u);

    return (uint8_t)(bit ^ 0x01u);
}

/**
 * @brief Calculate returned-frame parity bit PC2.
 *
 * @param inputs Input-state byte IN8..IN1.
 *
 * @return Calculated returned-frame PC2 bit.
 *
 * @details
 * According to the SCLT3-8BT8 datasheet, PC2 is generated from the upper
 * input group.
 *
 * PC2 = NOT(IN8 XOR IN7 XOR IN6 XOR IN5)
 */
static uint8_t sclt38bt8_calculate_pc2(uint8_t inputs)
{
    uint8_t bit;

    bit  = (uint8_t)((inputs >> 7u) & 0x01u);
    bit ^= (uint8_t)((inputs >> 6u) & 0x01u);
    bit ^= (uint8_t)((inputs >> 5u) & 0x01u);
    bit ^= (uint8_t)((inputs >> 4u) & 0x01u);

    return (uint8_t)(bit ^ 0x01u);
}

/**
 * @brief Calculate returned-frame parity bit PC3.
 *
 * @param inputs Input-state byte IN8..IN1.
 *
 * @return Calculated returned-frame PC3 bit.
 *
 * @details
 * According to the SCLT3-8BT8 datasheet, PC3 is generated from the lower
 * input group.
 *
 * PC3 = NOT(IN4 XOR IN3 XOR IN2 XOR IN1)
 */
static uint8_t sclt38bt8_calculate_pc3(uint8_t inputs)
{
    uint8_t bit;

    bit  = (uint8_t)((inputs >> 3u) & 0x01u);
    bit ^= (uint8_t)((inputs >> 2u) & 0x01u);
    bit ^= (uint8_t)((inputs >> 1u) & 0x01u);
    bit ^= (uint8_t)((inputs >> 0u) & 0x01u);

    return (uint8_t)(bit ^ 0x01u);
}

/**
 * @brief Calculate returned-frame parity bit PC4.
 *
 * @param inputs Input-state byte IN8..IN1.
 *
 * @return Calculated returned-frame PC4 bit.
 *
 * @details
 * According to the SCLT3-8BT8 datasheet, PC4 is generated from the middle
 * input group.
 *
 * PC4 = NOT(IN6 XOR IN5 XOR IN4 XOR IN3)
 */
static uint8_t sclt38bt8_calculate_pc4(uint8_t inputs)
{
    uint8_t bit;

    bit  = (uint8_t)((inputs >> 5u) & 0x01u);
    bit ^= (uint8_t)((inputs >> 4u) & 0x01u);
    bit ^= (uint8_t)((inputs >> 3u) & 0x01u);
    bit ^= (uint8_t)((inputs >> 2u) & 0x01u);

    return (uint8_t)(bit ^ 0x01u);
}

/** @} */
