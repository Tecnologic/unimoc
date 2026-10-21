/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file cyphal_log_backend.h
 *    @brief Zephyr log backend that hands log lines to the Cyphal thread,
 *           which publishes them as uavcan.diagnostic.Record.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Takes the oldest queued log line.
 * @param level Zephyr log level of the line (LOG_LEVEL_ERR .. LOG_LEVEL_DBG).
 * @param text Destination; the line is NUL terminated and cut to @p size.
 * @param size Size of @p text in bytes.
 * @return False when no line is queued.
 */
bool UnimocCyphalLogGet(uint8_t* level, char* text, size_t size);

#ifdef __cplusplus
}
#endif
