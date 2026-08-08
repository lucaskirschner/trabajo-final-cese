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
 * @file    rs485.c
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-08-08
 * @brief   Interrupt-driven single-byte RS485 driver implementation.
 *
 * @details
 * This module implements a minimal half-duplex RS485 communication mechanism
 * using the hardware-independent services exposed by rs485_port.
 *
 * Each communication operation handles exactly one byte.
 *
 * Transmission flow:
 *
 * @code
 * rs485_send()
 *      |
 *      v
 * RS485_PORT_MODE_TX
 *      |
 *      v
 * rs485_port_transmit_it()
 *      |
 *      v
 * HAL_UART_TxCpltCallback()
 *      |
 *      v
 * rs485_port_tx_complete_callback()
 *      |
 *      v
 * RS485_PORT_MODE_RX
 *      |
 *      v
 * rs485_tx_complete_callback()
 * @endcode
 *
 * Reception flow:
 *
 * @code
 * rs485_receive_start()
 *      |
 *      v
 * RS485_PORT_MODE_RX
 *      |
 *      v
 * rs485_port_receive_it()
 *      |
 *      v
 * HAL_UART_RxCpltCallback()
 *      |
 *      v
 * rs485_port_rx_complete_callback()
 *      |
 *      v
 * rs485_rx_complete_callback()
 *      |
 *      v
 * rs485_receive()
 * @endcode
 *
 * The transmit and receive bytes are stored in persistent driver context
 * because interrupt-driven UART operations continue after their initiating
 * functions have returned.
 *
 * Only one communication operation may be active at a time.
 *
 * The driver does not implement:
 *
 * - Message framing.
 * - Addressing.
 * - Checksums or CRC.
 * - Protocol commands.
 * - Retransmissions.
 * - Queues.
 * - RTOS synchronization.
 *
 * These responsibilities belong to upper software layers.
 *
 * This module does not use DMA or dynamic memory.
 *
 * @ingroup rs485
 * @{
 */

/* ============================= Includes ================================== */

#include "rs485.h"

#include "rs485_port.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ============================ Local Macros =============================== */

/**
 * @brief Number of bytes handled by each transmission operation.
 */
#define RS485_TX_SIZE                       ((uint16_t)1u)

/**
 * @brief Number of bytes handled by each reception operation.
 */
#define RS485_RX_SIZE                       ((uint16_t)1u)

/* ============================ Local Types ================================ */

/**
 * @brief Internal RS485 driver context.
 */
typedef struct
{
    /**
     * @brief Indicates whether the RS485 driver has been initialized.
     */
    bool initialized;

    /**
     * @brief Indicates that an interrupt-driven reception is active.
     */
    volatile bool rx_active;

    /**
     * @brief Indicates that a received byte is available for consumption.
     */
    volatile bool rx_complete;

    /**
     * @brief Indicates that an interrupt-driven transmission is active.
     */
    volatile bool tx_active;

    /**
     * @brief Last UART error flags reported by the port layer.
     */
    volatile uint32_t last_uart_error;

    /**
     * @brief Persistent storage for the received byte.
     */
    uint8_t rx_data;

    /**
     * @brief Persistent storage for the transmitted byte.
     */
    uint8_t tx_data;

    /**
     * @brief Handle associated with the underlying RS485 port layer.
     */
    rs485_port_handle_t port_handle;

} rs485_ctx_t;

/* ======================= Local Static Data ================================ */

/**
 * @brief Internal RS485 driver context.
 */
static rs485_ctx_t g_ctx = {
    .initialized = false,
    .rx_active = false,
    .rx_complete = false,
    .tx_active = false,
    .last_uart_error = 0u,
    .rx_data = 0u,
    .tx_data = 0u,
    .port_handle = {
        .reserved = 0u
    }
};

/* ===================== Private Function Prototypes ======================= */

static rs485_status_t rs485_validate_handle(
    const rs485_handle_t * handle);

static rs485_status_t rs485_port_status_to_status(
    rs485_port_status_t port_status);

static void rs485_clear_state(void);

/* ===================== Port Notification Prototypes ====================== */

/**
 * @brief Transmission-complete notification called by rs485_port.c.
 *
 * @details
 * This function overrides the weak notification hook provided by the RS485
 * port layer.
 *
 * It executes in interrupt context.
 */
void rs485_port_tx_complete_callback(void);

/**
 * @brief Reception-complete notification called by rs485_port.c.
 *
 * @details
 * This function overrides the weak notification hook provided by the RS485
 * port layer.
 *
 * It executes in interrupt context.
 */
void rs485_port_rx_complete_callback(void);

/**
 * @brief UART-error notification called by rs485_port.c.
 *
 * @param[in] error_code  UART error flags reported by the port layer.
 *
 * @details
 * This function overrides the weak notification hook provided by the RS485
 * port layer.
 *
 * It executes in interrupt context.
 */
void rs485_port_error_callback(
    uint32_t error_code);

/* ===================== Public Function Definitions ======================= */

rs485_status_t rs485_init(
    rs485_handle_t * handle)
{
    rs485_port_status_t port_status;
    rs485_status_t status;

    status = rs485_validate_handle(
        handle);

    if (status != RS485_OK)
    {
        return status;
    }

    if (g_ctx.initialized != false)
    {
        return RS485_E_STATE;
    }

    port_status = rs485_port_init(
        &g_ctx.port_handle);

    status = rs485_port_status_to_status(
        port_status);

    if (status != RS485_OK)
    {
        return status;
    }

    /*
     * Explicitly request the default receive state.
     *
     * rs485_port_init() already configures receive mode, but keeping the
     * requested driver state explicit makes the ownership of the half-duplex
     * communication policy clear at this layer.
     */
    port_status = rs485_port_set_mode(
        RS485_PORT_MODE_RX);

    status = rs485_port_status_to_status(
        port_status);

    if (status != RS485_OK)
    {
        (void)rs485_port_deinit(
            &g_ctx.port_handle);

        return status;
    }

    rs485_clear_state();

    g_ctx.initialized = true;

    return RS485_OK;
}

rs485_status_t rs485_deinit(
    rs485_handle_t * handle)
{
    rs485_port_status_t port_status;
    rs485_status_t status;

    status = rs485_validate_handle(
        handle);

    if (status != RS485_OK)
    {
        return status;
    }

    if (g_ctx.initialized == false)
    {
        return RS485_E_STATE;
    }

    /*
     * Abort any active receive operation before deinitializing the port.
     */
    if (g_ctx.rx_active != false)
    {
        port_status = rs485_port_abort_receive();

        status = rs485_port_status_to_status(
            port_status);

        if (status != RS485_OK)
        {
            return status;
        }

        g_ctx.rx_active = false;
    }

    /*
     * Abort any active transmit operation before deinitializing the port.
     */
    if (g_ctx.tx_active != false)
    {
        port_status = rs485_port_abort_transmit();

        status = rs485_port_status_to_status(
            port_status);

        if (status != RS485_OK)
        {
            return status;
        }

        g_ctx.tx_active = false;
    }

    /*
     * Restore the external transceiver to the default receive state.
     */
    port_status = rs485_port_set_mode(
        RS485_PORT_MODE_RX);

    status = rs485_port_status_to_status(
        port_status);

    if (status != RS485_OK)
    {
        return status;
    }

    port_status = rs485_port_deinit(
        &g_ctx.port_handle);

    status = rs485_port_status_to_status(
        port_status);

    if (status != RS485_OK)
    {
        return status;
    }

    rs485_clear_state();

    g_ctx.initialized = false;

    return RS485_OK;
}

rs485_status_t rs485_send(
    uint8_t data)
{
    rs485_port_status_t port_status;
    rs485_status_t status;

    if (g_ctx.initialized == false)
    {
        return RS485_E_STATE;
    }

    /*
     * Half-duplex communication permits only one operation at a time.
     *
     * An active reception must be completed or explicitly aborted by the
     * upper layer before a transmission can start.
     */
    if ((g_ctx.tx_active != false) ||
        (g_ctx.rx_active != false))
    {
        return RS485_E_STATE;
    }

    /*
     * Preserve the byte in driver-owned storage.
     *
     * The interrupt-driven UART transmission continues after rs485_send()
     * returns, so a local function parameter cannot be used directly as the
     * HAL transmission buffer.
     */
    g_ctx.tx_data = data;
    g_ctx.last_uart_error = 0u;

    port_status = rs485_port_set_mode(
        RS485_PORT_MODE_TX);

    status = rs485_port_status_to_status(
        port_status);

    if (status != RS485_OK)
    {
        return status;
    }

    /*
     * Mark transmission active before requesting the lower-layer operation.
     *
     * This avoids a possible race in which a very short UART operation could
     * complete and generate its interrupt before the driver state is updated.
     */
    g_ctx.tx_active = true;

    port_status = rs485_port_transmit_it(
        &g_ctx.tx_data,
        RS485_TX_SIZE);

    status = rs485_port_status_to_status(
        port_status);

    if (status != RS485_OK)
    {
        g_ctx.tx_active = false;

        /*
         * Transmission could not be started. Restore the default receive
         * state before returning the failure to the caller.
         */
        (void)rs485_port_set_mode(
            RS485_PORT_MODE_RX);

        return status;
    }

    return RS485_OK;
}

rs485_status_t rs485_receive_start(void)
{
    rs485_port_status_t port_status;
    rs485_status_t status;

    if (g_ctx.initialized == false)
    {
        return RS485_E_STATE;
    }

    /*
     * A reception cannot be started while another operation is active.
     *
     * A new reception is also rejected while a previously received byte
     * remains unconsumed.
     */
    if ((g_ctx.rx_active != false) ||
        (g_ctx.tx_active != false) ||
        (g_ctx.rx_complete != false))
    {
        return RS485_E_STATE;
    }

    port_status = rs485_port_set_mode(
        RS485_PORT_MODE_RX);

    status = rs485_port_status_to_status(
        port_status);

    if (status != RS485_OK)
    {
        return status;
    }

    g_ctx.rx_data = 0u;
    g_ctx.last_uart_error = 0u;

    /*
     * Mark reception active before starting the lower-layer UART operation to
     * avoid a possible state race with the completion interrupt.
     */
    g_ctx.rx_active = true;

    port_status = rs485_port_receive_it(
        &g_ctx.rx_data,
        RS485_RX_SIZE);

    status = rs485_port_status_to_status(
        port_status);

    if (status != RS485_OK)
    {
        g_ctx.rx_active = false;

        return status;
    }

    return RS485_OK;
}

rs485_status_t rs485_receive(
    uint8_t * p_data)
{
    if (p_data == NULL)
    {
        return RS485_E_NULL;
    }

    if (g_ctx.initialized == false)
    {
        return RS485_E_STATE;
    }

    if (g_ctx.rx_complete == false)
    {
        return RS485_E_STATE;
    }

    *p_data = g_ctx.rx_data;

    /*
     * Mark the received byte as consumed. A new interrupt-driven reception may
     * now be armed through rs485_receive_start().
     */
    g_ctx.rx_complete = false;
    g_ctx.rx_data = 0u;

    return RS485_OK;
}

rs485_status_t rs485_receive_abort(void)
{
    rs485_port_status_t port_status;
    rs485_status_t status;

    if (g_ctx.initialized == false)
    {
        return RS485_E_STATE;
    }

    if (g_ctx.rx_active != false)
    {
        port_status = rs485_port_abort_receive();

        status = rs485_port_status_to_status(
            port_status);

        if (status != RS485_OK)
        {
            return status;
        }
    }

    g_ctx.rx_active = false;
    g_ctx.rx_complete = false;
    g_ctx.rx_data = 0u;
    g_ctx.last_uart_error = 0u;

    port_status = rs485_port_set_mode(
        RS485_PORT_MODE_RX);

    return rs485_port_status_to_status(
        port_status);
}

rs485_status_t rs485_transmit_abort(void)
{
    rs485_port_status_t port_status;
    rs485_status_t status;

    if (g_ctx.initialized == false)
    {
        return RS485_E_STATE;
    }

    if (g_ctx.tx_active != false)
    {
        port_status = rs485_port_abort_transmit();

        status = rs485_port_status_to_status(
            port_status);

        if (status != RS485_OK)
        {
            return status;
        }
    }

    g_ctx.tx_active = false;
    g_ctx.tx_data = 0u;
    g_ctx.last_uart_error = 0u;

    /*
     * An aborted transmission must always leave the half-duplex transceiver in
     * receive mode.
     */
    port_status = rs485_port_set_mode(
        RS485_PORT_MODE_RX);

    return rs485_port_status_to_status(
        port_status);
}

/* ===================== Port Notification Definitions ===================== */

/**
 * @brief Handle completion of the UART transmission started by the port layer.
 *
 * @details
 * This function executes in interrupt context.
 *
 * It clears the active transmission state, restores the half-duplex
 * transceiver to receive mode and propagates the completion notification to
 * the upper application layer.
 */
void rs485_port_tx_complete_callback(void)
{
    if (g_ctx.initialized != false)
    {
        g_ctx.tx_active = false;
        g_ctx.last_uart_error = 0u;

        /*
         * Transmission has physically completed. Release the RS485 bus and
         * enable the receiver again.
         */
        (void)rs485_port_set_mode(
            RS485_PORT_MODE_RX);

        rs485_tx_complete_callback(
            g_ctx.tx_data);
    }
}

/**
 * @brief Handle completion of the UART reception started by the port layer.
 *
 * @details
 * This function executes in interrupt context.
 *
 * The received byte has already been written into g_ctx.rx_data by the UART
 * interrupt-driven receive operation.
 *
 * The callback updates the driver state and notifies the upper application
 * layer that the byte is ready to be consumed.
 */
void rs485_port_rx_complete_callback(void)
{
    if (g_ctx.initialized != false)
    {
        g_ctx.rx_active = false;
        g_ctx.rx_complete = true;
        g_ctx.last_uart_error = 0u;

        rs485_rx_complete_callback();
    }
}

/**
 * @brief Handle a UART communication error reported by the port layer.
 *
 * @param[in] error_code  UART error flags.
 *
 * @details
 * This function executes in interrupt context.
 *
 * Any active transmit or receive operation is considered terminated. Pending
 * reception data is discarded and the RS485 transceiver is restored to receive
 * mode before the error is propagated to the upper application layer.
 */
void rs485_port_error_callback(
    uint32_t error_code)
{
    if (g_ctx.initialized != false)
    {
        g_ctx.rx_active = false;
        g_ctx.rx_complete = false;
        g_ctx.tx_active = false;

        g_ctx.last_uart_error = error_code;

        g_ctx.rx_data = 0u;

        /*
         * Return the external half-duplex transceiver to its default idle
         * state after any UART communication error.
         */
        (void)rs485_port_set_mode(
            RS485_PORT_MODE_RX);

        rs485_error_callback(
            error_code);
    }
}

/* ================= Notification Hook Definitions ========================= */

/**
 * @brief Default transmission-complete notification hook.
 *
 * @param[in] data  Byte whose transmission has completed.
 *
 * @details
 * This weak implementation intentionally performs no operation.
 *
 * The upper application layer may provide a strong definition.
 */
__attribute__((weak)) void rs485_tx_complete_callback(
    uint8_t data)
{
    (void)data;
}

/**
 * @brief Default reception-complete notification hook.
 *
 * @details
 * This weak implementation intentionally performs no operation.
 *
 * The upper application layer may provide a strong definition.
 */
__attribute__((weak)) void rs485_rx_complete_callback(void)
{
    /* Upper-layer notification hook. */
}

/**
 * @brief Default RS485 UART-error notification hook.
 *
 * @param[in] error_code  UART error flags.
 *
 * @details
 * This weak implementation intentionally performs no operation.
 *
 * The upper application layer may provide a strong definition.
 */
__attribute__((weak)) void rs485_error_callback(
    uint32_t error_code)
{
    (void)error_code;
}

/* ===================== Private Function Definitions ====================== */

/**
 * @brief Validate the public RS485 driver handle.
 *
 * @param[in] handle  RS485 driver handle.
 *
 * @return RS485_OK on success.
 * @return RS485_E_NULL if handle is NULL.
 */
static rs485_status_t rs485_validate_handle(
    const rs485_handle_t * handle)
{
    if (handle == NULL)
    {
        return RS485_E_NULL;
    }

    return RS485_OK;
}

/**
 * @brief Convert RS485 port status codes to driver status codes.
 *
 * @param[in] port_status  Status returned by the RS485 port layer.
 *
 * @return Equivalent RS485 driver status code.
 */
static rs485_status_t rs485_port_status_to_status(
    rs485_port_status_t port_status)
{
    rs485_status_t status;

    switch (port_status)
    {
        case RS485_PORT_OK:

            status = RS485_OK;
            break;

        case RS485_PORT_E_NULL:

            status = RS485_E_NULL;
            break;

        case RS485_PORT_E_PARAM:

            status = RS485_E_PARAM;
            break;

        case RS485_PORT_E_STATE:

            status = RS485_E_STATE;
            break;

        case RS485_PORT_E_TIMEOUT:

            status = RS485_E_TIMEOUT;
            break;

        case RS485_PORT_E_HW:
        default:

            status = RS485_E_HW;
            break;
    }

    return status;
}

/**
 * @brief Clear the internal RS485 communication state.
 *
 * @details
 * Resets all transient communication flags and byte storage.
 *
 * This function does not modify the initialized state because initialization
 * ownership remains with rs485_init() and rs485_deinit().
 */
static void rs485_clear_state(void)
{
    g_ctx.rx_active = false;
    g_ctx.rx_complete = false;
    g_ctx.tx_active = false;

    g_ctx.last_uart_error = 0u;

    g_ctx.rx_data = 0u;
    g_ctx.tx_data = 0u;
}

/** @} */
