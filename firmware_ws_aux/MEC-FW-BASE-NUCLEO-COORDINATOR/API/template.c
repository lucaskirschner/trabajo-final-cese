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
 * @file    template.c
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-02-12
 * @brief   Brief description of this module implementation.
 *
 * @details
 * Implementation notes: concurrency model, ISR usage, and constraints.
 *
 * @ingroup module_name
 * @{
 */

/* ============================= Includes ================================== */

#include "template.h"

/* Standard headers (if needed) */
#include <stddef.h>

/* ============================ Local Macros =============================== */

/* Prefer static const over macros when possible. */
#define MODULE_NAME_INTERNAL_MAGIC   (0x4D4F4455UL) /* 'MODU' */

/* ============================ Local Types ================================ */

typedef struct
{
    uint32_t magic;
    bool initialized;
} module_name_ctx_t;

/* ======================= Local (static) Data ============================= */

static module_name_ctx_t g_ctx = {
    .magic = MODULE_NAME_INTERNAL_MAGIC,
    .initialized = false
};

/* ===================== Local Function Prototypes ========================= */

static module_name_status_t module_name_validate_handle(
    const module_name_handle_t *handle);

/* ===================== Public Function Definitions ======================= */

module_name_status_t module_name_init(module_name_handle_t *handle)
{
    module_name_status_t status = MODULE_NAME_OK;

    status = module_name_validate_handle(handle);
    if (status != MODULE_NAME_OK)
    {
        return status;
    }

    if (g_ctx.initialized != false)
    {
        return MODULE_NAME_E_STATE;
    }

    /* TODO: Initialize hardware/resources here. */

    g_ctx.initialized = true;

    return MODULE_NAME_OK;
}

void module_name_task(module_name_handle_t *handle)
{
    if (module_name_validate_handle(handle) != MODULE_NAME_OK)
    {
        return;
    }

    if (g_ctx.initialized == false)
    {
        return;
    }

    /* TODO: Periodic work here. Keep non-blocking if possible. */
}

module_name_status_t module_name_deinit(module_name_handle_t *handle)
{
    module_name_status_t status = MODULE_NAME_OK;

    status = module_name_validate_handle(handle);
    if (status != MODULE_NAME_OK)
    {
        return status;
    }

    if (g_ctx.initialized == false)
    {
        return MODULE_NAME_E_STATE;
    }

    /* TODO: Release resources here. */

    g_ctx.initialized = false;

    return MODULE_NAME_OK;
}

/* ===================== Local Function Definitions ======================== */

static module_name_status_t module_name_validate_handle(
    const module_name_handle_t *handle)
{
    if (handle == NULL)
    {
        return MODULE_NAME_E_NULL;
    }

    if (g_ctx.magic != MODULE_NAME_INTERNAL_MAGIC)
    {
        return MODULE_NAME_E_STATE;
    }

    return MODULE_NAME_OK;
}

/** @} */
