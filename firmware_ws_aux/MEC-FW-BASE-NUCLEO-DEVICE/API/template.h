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
 * @file    module_name.h
 * @author  Lucas Kirschner <kirschnerlucas1@gmail.com>
 * @date    2026-02-12
 * @brief   Brief description of this module.
 *
 * @details
 * Longer description: purpose, assumptions, and usage notes.
 *
 * @ingroup module_name
 * @{
 */

#ifndef MODULE_NAME_H_
#define MODULE_NAME_H_

/* ============================= Includes ================================== */

#include <stdint.h>
#include <stdbool.h>

/* ============================== Macros =================================== */

/* Public constants/macros go here. Keep macros uppercase. */

/* ============================== Types ==================================== */

/**
 * @brief Public status codes for this module.
 */
typedef enum
{
    MODULE_NAME_OK = 0,
    MODULE_NAME_E_NULL,
    MODULE_NAME_E_PARAM,
    MODULE_NAME_E_STATE,
    MODULE_NAME_E_HW,
    MODULE_NAME_E_TIMEOUT
} module_name_status_t;

/**
 * @brief Opaque handle (preferred) or public handle struct.
 *
 * If you want an opaque handle, forward-declare `struct module_name;`
 * and typedef the pointer. If you want a concrete handle, define the struct
 * here and keep it small and POD.
 */
typedef struct
{
    uint32_t reserved;
} module_name_handle_t;

/* ===================== Public Function Prototypes ======================== */

/**
 * @brief Initialize the module.
 *
 * @param[in,out] handle  Module handle.
 *
 * @return MODULE_NAME_OK on success, error code otherwise.
 */
module_name_status_t module_name_init(module_name_handle_t *handle);

/**
 * @brief Periodic task function (optional).
 *
 * @param[in,out] handle  Module handle.
 */
void module_name_task(module_name_handle_t *handle);

/**
 * @brief Deinitialize the module (optional).
 *
 * @param[in,out] handle  Module handle.
 *
 * @return MODULE_NAME_OK on success, error code otherwise.
 */
module_name_status_t module_name_deinit(module_name_handle_t *handle);

#endif /* MODULE_NAME_H_ */

/** @} */
