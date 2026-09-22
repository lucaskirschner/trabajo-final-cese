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
 * @file    vni8200xp32.c
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-04-21
 * @brief   High-level driver for the ST VNI8200XP-32 digital output driver.
 *
 * @details
 * Concurrency model:
 *  - Blocking SPI transfers through the shared io_port layer.
 *  - No internal state is retained by this driver.
 *
 * This driver targets the 16-bit SPI mode of the VNI8200XP-32. One command
 * frame contains:
 *  - 8 output-control bits
 *  - 4 command parity bits
 *
 * At the same time, the device returns a 16-bit fault frame containing:
 *  - 8 per-channel fault bits
 *  - 4 general diagnostic bits
 *  - 4 returned-frame parity bits
 *
 * The driver writes the requested output image, validates the returned fault
 * frame parity, checks the device diagnostic bits, and reports either success
 * or the first detected error condition.
 *
 * @ingroup vni8200xp32
 * @{
 */

/* ============================= Includes ================================== */

#include "vni8200xp32.h"
#include "io_port.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ============================ Local Macros =============================== */

/**
 * @brief SPI frame length in bytes for 16-bit mode.
 */
#define VNI8200XP32_FRAME_SIZE_BYTES          ((uint8_t)2u)

/**
 * @brief Byte positions inside a 16-bit SPI frame buffer.
 */
#define VNI8200XP32_TX_OUTPUTS_INDEX          ((uint8_t)0u)
#define VNI8200XP32_TX_CONTROL_INDEX          ((uint8_t)1u)
#define VNI8200XP32_RX_FAULTS_INDEX           ((uint8_t)0u)
#define VNI8200XP32_RX_STATUS_INDEX           ((uint8_t)1u)

/**
 * @brief Command-frame parity nibble placement.
 *
 * @details
 * In 16-bit SPI mode, the transmitted frame is arranged as follows:
 * - first byte: OUT7..OUT0
 * - second byte:
 *   - bits [7:4]: not used, transmitted as zero
 *   - bit 3: P2
 *   - bit 2: P1
 *   - bit 1: P0
 *   - bit 0: nP0
 */
#define VNI8200XP32_CMD_P2_SHIFT              ((uint8_t)3u)
#define VNI8200XP32_CMD_P1_SHIFT              ((uint8_t)2u)
#define VNI8200XP32_CMD_P0_SHIFT              ((uint8_t)1u)
#define VNI8200XP32_CMD_NP0_SHIFT             ((uint8_t)0u)

/**
 * @brief Returned status byte bit masks.
 */
#define VNI8200XP32_STATUS_FB_OK_MASK         ((uint8_t)(1u << 7))
#define VNI8200XP32_STATUS_TWARN_MASK         ((uint8_t)(1u << 6))
#define VNI8200XP32_STATUS_PC_MASK            ((uint8_t)(1u << 5))
#define VNI8200XP32_STATUS_PG_MASK            ((uint8_t)(1u << 4))
#define VNI8200XP32_STATUS_P2_MASK            ((uint8_t)(1u << 3))
#define VNI8200XP32_STATUS_P1_MASK            ((uint8_t)(1u << 2))
#define VNI8200XP32_STATUS_P0_MASK            ((uint8_t)(1u << 1))
#define VNI8200XP32_STATUS_NP0_MASK           ((uint8_t)(1u << 0))

/* ========================== Private Prototypes =========================== */

static vni8200xp32_status_t vni8200xp32_from_port_status(io_port_status_t status);
static void vni8200xp32_build_tx_frame(uint8_t outputs,
                                       uint8_t * const p_tx_buffer);
static bool vni8200xp32_check_rx_parity(uint8_t faults, uint8_t status_raw);
static uint8_t vni8200xp32_calculate_cmd_p0(uint8_t outputs);
static uint8_t vni8200xp32_calculate_cmd_p1(uint8_t outputs);
static uint8_t vni8200xp32_calculate_cmd_p2(uint8_t outputs);
static uint8_t vni8200xp32_calculate_rx_p0(uint8_t faults);
static uint8_t vni8200xp32_calculate_rx_p1(uint8_t faults, uint8_t status_raw);
static uint8_t vni8200xp32_calculate_rx_p2(uint8_t faults, uint8_t status_raw);

/* ===================== Public Function Definitions ======================= */

vni8200xp32_status_t vni8200xp32_write_outputs(uint8_t outputs)
{
    vni8200xp32_status_t status;
    uint8_t tx_buffer[VNI8200XP32_FRAME_SIZE_BYTES];
    uint8_t rx_buffer[VNI8200XP32_FRAME_SIZE_BYTES];
    uint8_t faults;
    uint8_t status_raw;
    bool fb_ok;
    bool case_temp_warning;
    bool spi_fault;
    bool power_good;
    bool channel_fault;

    vni8200xp32_build_tx_frame(outputs, tx_buffer);

    status = vni8200xp32_from_port_status(
        io_port_transmit_receive(IO_PORT_DEVICE_VNI8200XP32,
                                 tx_buffer,
                                 rx_buffer,
                                 VNI8200XP32_FRAME_SIZE_BYTES));

    if (status != VNI8200XP32_OK)
    {
        return status;
    }

    faults = rx_buffer[VNI8200XP32_RX_FAULTS_INDEX];
    status_raw = rx_buffer[VNI8200XP32_RX_STATUS_INDEX];

    /* Validation order:
     * 1. Integrity of the received diagnostic frame
     * 2. SPI communication fault
     * 3. DC-DC feedback status
     * 4. Case temperature warning
     * 5. Power Good status
     * 6. Per-channel latched overtemperature faults
     */

    /* Reject the received diagnostic frame if its parity/check bits do not
     * match the datasheet-defined fault-frame equations.
     *
     * This indicates that the returned 16-bit status word cannot be trusted for
     * further diagnostic interpretation.
     */
    if (vni8200xp32_check_rx_parity(faults, status_raw) == false)
    {
        return VNI8200XP32_E_RX_PARITY;
    }

    /* PC = 1 indicates an SPI communication fault.
     *
     * According to the datasheet, this condition is set when the incoming SPI
     * frame parity check fails or when the number of received clock rising
     * edges is not a multiple of 8.
     *
     * In SPI mode, this fault also forces the FAULT indication active.
     */
    spi_fault = ((status_raw & VNI8200XP32_STATUS_PC_MASK) != 0u);
    if (spi_fault == true)
    {
        return VNI8200XP32_E_SPI;
    }

    /* FB_OK = 0 indicates that the internal DC-DC regulation status is not OK.
     *
     * At startup this bit remains low until FB rises above 90% of the nominal
     * feedback voltage and a correct SPI communication has occurred.
     *
     * If the FB voltage falls below 80% of the nominal value, this bit returns
     * to zero.
     */
    fb_ok = ((status_raw & VNI8200XP32_STATUS_FB_OK_MASK) != 0u);
    if (fb_ok == false)
    {
        return VNI8200XP32_E_FB;
    }

    /* TWARN = 1 is treated as an active IC case-temperature warning.
     *
     * Note: this validation intentionally follows the diagnostic convention used
     * by STMicroelectronics in the X-CUBE-PLC1 software package for the
     * X-NUCLEO-PLC01A1 expansion board.
     *
     * This differs from the direct active-low interpretation described in the
     * VNI8200XP-32 datasheet for the TWARN pin/indication. However, practical
     * testing with the ST development kit showed that TWARN = 0 corresponds to
     * the normal condition in the SPI status byte, while TWARN = 1 is handled as
     * the warning condition by the reference ST software.
     *
     * Therefore, the SPI TWARN bit is considered fault-active when set.
     */
    case_temp_warning = ((status_raw & VNI8200XP32_STATUS_TWARN_MASK) != 0u);
    if (case_temp_warning == true)
    {
        return VNI8200XP32_E_TWARN;
    }

    /* PG = 1 is treated as a Power Good diagnostic fault.
     *
     * Note: this validation intentionally follows the diagnostic convention used
     * by STMicroelectronics in the X-CUBE-PLC1 software package for the
     * X-NUCLEO-PLC01A1 expansion board.
     *
     * This differs from the direct interpretation described in the VNI8200XP-32
     * datasheet, where the 16-bit SPI PG status bit is described as set when
     * Power Good is active. Practical testing with the ST development kit showed
     * that PG = 0 corresponds to the normal condition in the SPI status byte,
     * while PG = 1 is handled as the fault condition by the reference ST
     * software.
     *
     * Therefore, the SPI PG bit is considered fault-active when set.
     */
    power_good = ((status_raw & VNI8200XP32_STATUS_PG_MASK) == 0u);
    if (power_good == false)
    {
        return VNI8200XP32_E_PG;
    }

    /* Any nonzero bit in F7..F0 indicates a channel overtemperature event.
     *
     * Each fault bit corresponds to one output channel and is set when that
     * channel has experienced junction overtemperature protection.
     *
     * The datasheet indicates that the SPI fault bits are latched and cleared
     * after communication.
     */
    channel_fault = (faults != 0u);
    if (channel_fault == true)
    {
        return VNI8200XP32_E_OVT;
    }

    return VNI8200XP32_OK;
}

/* ===================== Private Function Definitions ====================== */

/**
 * @brief Translate a port-layer status code to the driver-layer status space.
 *
 * @param status Status code returned by the shared I/O port layer.
 *
 * @return Equivalent status code in the @ref vni8200xp32_status_t domain.
 *
 * @details
 * This helper preserves the abstraction boundary between the driver and the
 * underlying port layer by converting low-level access results into the
 * public driver status type.
 *
 * Any unknown port-layer status is conservatively mapped to
 * VNI8200XP32_E_HW.
 */
static vni8200xp32_status_t vni8200xp32_from_port_status(io_port_status_t status)
{
    vni8200xp32_status_t ret;

    switch (status)
    {
        case IO_PORT_OK:
            ret = VNI8200XP32_OK;
            break;

        case IO_PORT_E_NULL:
            ret = VNI8200XP32_E_NULL;
            break;

        case IO_PORT_E_PARAM:
            ret = VNI8200XP32_E_PARAM;
            break;

        case IO_PORT_E_HW:
            ret = VNI8200XP32_E_HW;
            break;

        case IO_PORT_E_TIMEOUT:
            ret = VNI8200XP32_E_TIMEOUT;
            break;

        case IO_PORT_E_BUSY:
        default:
            ret = VNI8200XP32_E_HW;
            break;
    }

    return ret;
}

/**
 * @brief Build the 16-bit SPI command frame.
 *
 * @param outputs Requested output image.
 * @param[out] p_tx_buffer Pointer to the two-byte transmit buffer.
 *
 * @details
 * The first transmitted byte contains OUT7..OUT0.
 *
 * The second transmitted byte contains:
 * - bits [7:4] = 0
 * - bit 3: P2
 * - bit 2: P1
 * - bit 1: P0
 * - bit 0: nP0
 */
static void vni8200xp32_build_tx_frame(uint8_t outputs,
                                       uint8_t * const p_tx_buffer)
{
    uint8_t p0;
    uint8_t p1;
    uint8_t p2;
    uint8_t control;

    p0 = vni8200xp32_calculate_cmd_p0(outputs);
    p1 = vni8200xp32_calculate_cmd_p1(outputs);
    p2 = vni8200xp32_calculate_cmd_p2(outputs);

    control  = (uint8_t)(p2 << VNI8200XP32_CMD_P2_SHIFT);
    control |= (uint8_t)(p1 << VNI8200XP32_CMD_P1_SHIFT);
    control |= (uint8_t)(p0 << VNI8200XP32_CMD_P0_SHIFT);
    control |= (uint8_t)((p0 ^ 0x01u) << VNI8200XP32_CMD_NP0_SHIFT);

    p_tx_buffer[VNI8200XP32_TX_OUTPUTS_INDEX] = outputs;
    p_tx_buffer[VNI8200XP32_TX_CONTROL_INDEX] = control;
}

/**
 * @brief Validate the parity bits of the returned fault frame.
 *
 * @param faults Fault byte F7..F0.
 * @param status_raw Returned status/parity byte.
 *
 * @return
 * - true: Returned frame parity is valid.
 * - false: Returned frame parity is invalid.
 *
 * @details
 * According to the VNI8200XP-32 datasheet, the returned fault frame includes
 * parity bits intended to validate the received 16-bit SPI diagnostic word.
 *
 * All parity equations are expressed using XOR operations (no inversion is
 * applied unless explicitly specified by the datasheet).
 *
 * The validation is performed using:
 *
 * - P0  = F0 XOR F1 XOR F2 XOR F3 XOR F4 XOR F5 XOR F6 XOR F7
 * - P1  = PC XOR FB_OK XOR F1 XOR F3 XOR F5 XOR F7
 * - P2  = PG XOR TWARN XOR F0 XOR F2 XOR F4 XOR F6
 * - nP0 = NOT(P0)
 *
 * If any returned parity bit does not match the value recalculated from the
 * received fault and status fields, the returned SPI frame cannot be
 * considered reliable for further diagnostic interpretation.
 */
static bool vni8200xp32_check_rx_parity(uint8_t faults, uint8_t status_raw)
{
    uint8_t p0;
    uint8_t p1;
    uint8_t p2;
    uint8_t np0;

    p0 = vni8200xp32_calculate_rx_p0(faults);
    p1 = vni8200xp32_calculate_rx_p1(faults, status_raw);
    p2 = vni8200xp32_calculate_rx_p2(faults, status_raw);
    np0 = (uint8_t)(p0 ^ 0x01u);

    if (((status_raw & VNI8200XP32_STATUS_P2_MASK) >> 3u) != p2)
    {
        return false;
    }

    if (((status_raw & VNI8200XP32_STATUS_P1_MASK) >> 2u) != p1)
    {
        return false;
    }

    if (((status_raw & VNI8200XP32_STATUS_P0_MASK) >> 1u) != p0)
    {
        return false;
    }

    if ((status_raw & VNI8200XP32_STATUS_NP0_MASK) != np0)
    {
        return false;
    }

    return true;
}

/**
 * @brief Calculate command parity bit P0.
 *
 * @param outputs Requested output image.
 *
 * @return Calculated command-frame P0 bit.
 *
 * @details
 * According to the VNI8200XP-32 datasheet, in 16-bit SPI command mode, P0 is
 * generated as the XOR of all output-control bits:
 *
 * P0 = IN0 XOR IN1 XOR IN2 XOR IN3 XOR IN4 XOR IN5 XOR IN6 XOR IN7
 *
 * where XOR denotes the exclusive-OR operation.
 */
static uint8_t vni8200xp32_calculate_cmd_p0(uint8_t outputs)
{
    uint8_t bit;

    bit  = (uint8_t)((outputs >> 0u) & 0x01u);
    bit ^= (uint8_t)((outputs >> 1u) & 0x01u);
    bit ^= (uint8_t)((outputs >> 2u) & 0x01u);
    bit ^= (uint8_t)((outputs >> 3u) & 0x01u);
    bit ^= (uint8_t)((outputs >> 4u) & 0x01u);
    bit ^= (uint8_t)((outputs >> 5u) & 0x01u);
    bit ^= (uint8_t)((outputs >> 6u) & 0x01u);
    bit ^= (uint8_t)((outputs >> 7u) & 0x01u);

    return bit;
}

/**
 * @brief Calculate command parity bit P1.
 *
 * @param outputs Requested output image.
 *
 * @return Calculated command-frame P1 bit.
 *
 * @details
 * According to the VNI8200XP-32 datasheet, in 16-bit SPI command mode, P1 is
 * generated as the XOR of the odd-indexed output-control bits:
 *
 * P1 = IN1 XOR IN3 XOR IN5 XOR IN7
 *
 * where XOR denotes the exclusive-OR operation.
 */
static uint8_t vni8200xp32_calculate_cmd_p1(uint8_t outputs)
{
    uint8_t bit;

    bit  = (uint8_t)((outputs >> 1u) & 0x01u);
    bit ^= (uint8_t)((outputs >> 3u) & 0x01u);
    bit ^= (uint8_t)((outputs >> 5u) & 0x01u);
    bit ^= (uint8_t)((outputs >> 7u) & 0x01u);

    return bit;
}

/**
 * @brief Calculate command parity bit P2.
 *
 * @param outputs Requested output image.
 *
 * @return Calculated command-frame P2 bit.
 *
 * @details
 * According to the VNI8200XP-32 datasheet, in 16-bit SPI command mode, P2 is
 * generated as the XOR of the even-indexed output-control bits:
 *
 * P2 = IN0 XOR IN2 XOR IN4 XOR IN6
 *
 * where XOR denotes the exclusive-OR operation.
 */
static uint8_t vni8200xp32_calculate_cmd_p2(uint8_t outputs)
{
    uint8_t bit;

    bit  = (uint8_t)((outputs >> 0u) & 0x01u);
    bit ^= (uint8_t)((outputs >> 2u) & 0x01u);
    bit ^= (uint8_t)((outputs >> 4u) & 0x01u);
    bit ^= (uint8_t)((outputs >> 6u) & 0x01u);

    return bit;
}

/**
 * @brief Calculate returned-frame parity bit P0.
 *
 * @param faults Fault byte F7..F0.
 *
 * @return Calculated returned-frame P0 bit.
 *
 * @details
 * According to the VNI8200XP-32 datasheet, in the 16-bit SPI fault frame, P0
 * is generated as the XOR of all channel-fault bits:
 *
 * P0 = F0 XOR F1 XOR F2 XOR F3 XOR F4 XOR F5 XOR F6 XOR F7
 *
 * where XOR denotes the exclusive-OR operation.
 */
static uint8_t vni8200xp32_calculate_rx_p0(uint8_t faults)
{
    uint8_t bit;

    bit  = (uint8_t)((faults >> 0u) & 0x01u);
    bit ^= (uint8_t)((faults >> 1u) & 0x01u);
    bit ^= (uint8_t)((faults >> 2u) & 0x01u);
    bit ^= (uint8_t)((faults >> 3u) & 0x01u);
    bit ^= (uint8_t)((faults >> 4u) & 0x01u);
    bit ^= (uint8_t)((faults >> 5u) & 0x01u);
    bit ^= (uint8_t)((faults >> 6u) & 0x01u);
    bit ^= (uint8_t)((faults >> 7u) & 0x01u);

    return bit;
}

/**
 * @brief Calculate returned-frame parity bit P1.
 *
 * @param faults Fault byte F7..F0.
 * @param status_raw Returned status/parity byte.
 *
 * @return Calculated returned-frame P1 bit.
 *
 * @details
 * According to the VNI8200XP-32 datasheet, in the 16-bit SPI fault frame, P1
 * is generated as the XOR of the SPI communication diagnostic, the DC-DC
 * feedback diagnostic, and the odd-indexed channel-fault bits:
 *
 * P1 = PC XOR FB_OK XOR F1 XOR F3 XOR F5 XOR F7
 *
 * where XOR denotes the exclusive-OR operation.
 */
static uint8_t vni8200xp32_calculate_rx_p1(uint8_t faults, uint8_t status_raw)
{
    uint8_t bit;
    uint8_t fb_ok;
    uint8_t pc;

    fb_ok = (uint8_t)((status_raw & VNI8200XP32_STATUS_FB_OK_MASK) >> 7u);
    pc    = (uint8_t)((status_raw & VNI8200XP32_STATUS_PC_MASK) >> 5u);

    bit  = pc;
    bit ^= fb_ok;
    bit ^= (uint8_t)((faults >> 1u) & 0x01u);
    bit ^= (uint8_t)((faults >> 3u) & 0x01u);
    bit ^= (uint8_t)((faults >> 5u) & 0x01u);
    bit ^= (uint8_t)((faults >> 7u) & 0x01u);

    return bit;
}

/**
 * @brief Calculate returned-frame parity bit P2.
 *
 * @param faults Fault byte F7..F0.
 * @param status_raw Returned status/parity byte.
 *
 * @return Calculated returned-frame P2 bit.
 *
 * @details
 * According to the VNI8200XP-32 datasheet, in the 16-bit SPI fault frame, P2
 * is generated as the XOR of the Power Good diagnostic, the case-temperature
 * warning diagnostic, and the even-indexed channel-fault bits:
 *
 * P2 = PG XOR TWARN XOR F0 XOR F2 XOR F4 XOR F6
 *
 * where XOR denotes the exclusive-OR operation.
 */
static uint8_t vni8200xp32_calculate_rx_p2(uint8_t faults, uint8_t status_raw)
{
    uint8_t bit;
    uint8_t twarn;
    uint8_t pg;

    twarn = (uint8_t)((status_raw & VNI8200XP32_STATUS_TWARN_MASK) >> 6u);
    pg    = (uint8_t)((status_raw & VNI8200XP32_STATUS_PG_MASK) >> 4u);

    bit  = pg;
    bit ^= twarn;
    bit ^= (uint8_t)((faults >> 0u) & 0x01u);
    bit ^= (uint8_t)((faults >> 2u) & 0x01u);
    bit ^= (uint8_t)((faults >> 4u) & 0x01u);
    bit ^= (uint8_t)((faults >> 6u) & 0x01u);

    return bit;
}

/** @} */
