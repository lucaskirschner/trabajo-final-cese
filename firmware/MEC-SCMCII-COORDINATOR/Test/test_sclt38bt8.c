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
 * @date    2026-09-25
 * @brief   Unit tests for the SCLT3-8BT8 digital input driver.
 *
 * @details
 * This test module validates the public interface of the SCLT3-8BT8 driver
 * using Ceedling and Unity.
 *
 * Hardware-dependent operations provided by the port layer are isolated from
 * the driver tests and may be replaced by mocks when required by each test
 * case.
 *
 * @ingroup test_sclt38bt8
 * @{
 */

/* ============================= Includes ================================== */

#include "unity.h"
#include "Mockio_port.h"

#include "sclt38bt8.h"

#include <stddef.h>

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

/** @} */
