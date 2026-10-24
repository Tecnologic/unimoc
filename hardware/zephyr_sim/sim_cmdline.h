/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file sim_cmdline.h
 *    @brief Command-line options of the native_sim drive.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Value of `--node-id=<n>`.
 * @return The node-ID, or a negative value when the option was not given.
 */
int32_t UnimocSimNodeIdOption(void);

/**
 * @brief Value of `--cart-wheel=<n>`.
 * @return The wheel, or a negative value when the option was not given.
 */
int32_t UnimocSimCartWheelOption(void);

/**
 * @brief Value of `--describe-registers=<file>`.
 * @return The file name, or NULL when the option was not given.
 */
const char* UnimocSimDescribeRegistersOption(void);

#ifdef __cplusplus
}
#endif
