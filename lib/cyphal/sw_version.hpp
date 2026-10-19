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
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

// Zephyr generates <app_version.h> from the application's VERSION file.
// Hosted builds do not have it and use the fallback below.
#if __has_include(<app_version.h>)
#include <app_version.h>
#endif

#ifndef UNIMOC_SW_VERSION_MAJOR
#ifdef APP_VERSION_MAJOR
#define UNIMOC_SW_VERSION_MAJOR APP_VERSION_MAJOR
#else
#define UNIMOC_SW_VERSION_MAJOR 1
#endif
#endif

#ifndef UNIMOC_SW_VERSION_MINOR
#ifdef APP_VERSION_MINOR
#define UNIMOC_SW_VERSION_MINOR APP_VERSION_MINOR
#else
#define UNIMOC_SW_VERSION_MINOR 0
#endif
#endif
