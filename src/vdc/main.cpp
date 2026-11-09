/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file main.cpp
 *    @brief Entry point of the vehicle dynamics controller (VDC, apps/vdc).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "vdc_platform.hpp"

// Starts the Cyphal thread of the VDC and returns; the VDC has no hot path
// and no interrupt of its own (docs/system_plan.md, section 5.3).
int main() {
  unimoc::hardware::StartVdc();
  return 0;
}
