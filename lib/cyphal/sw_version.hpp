/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file sw_version.hpp
 *    @brief Software version macros for Cyphal node reporting.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

// Zephyr generates <app_version.h> from the application's VERSION file.
// Hosted builds do not have it and use the fallback below.
#if __has_include(<app_version.h>)
#include <app_version.h>
#endif

namespace unimoc::cyphal {

#ifdef APP_VERSION_MAJOR
/// Software major version from the Zephyr application VERSION file.
inline constexpr unsigned kSwVersionMajor = APP_VERSION_MAJOR;
#else
/// Software major version (fallback without <app_version.h>).
inline constexpr unsigned kSwVersionMajor = 1;
#endif

#ifdef APP_VERSION_MINOR
/// Software minor version from the Zephyr application VERSION file.
inline constexpr unsigned kSwVersionMinor = APP_VERSION_MINOR;
#else
/// Software minor version (fallback without <app_version.h>).
inline constexpr unsigned kSwVersionMinor = 0;
#endif

}  // namespace unimoc::cyphal
