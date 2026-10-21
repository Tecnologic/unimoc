/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file usb_clock_recovery.cpp
 *    @brief Trims the HSI48 USB clock of the battery case controller on the USB start of frame.
 *
 *    The STM32G4 clock driver of Zephyr does not handle the `crs-usb-sof`
 *    property, so the clock recovery system (CRS) is enabled here. After
 *    reset the CRS synchronizes on the USB SOF (CRS_CFGR.SYNCSRC), which is
 *    kept. Thread-context init only; nothing here touches the hot path.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/init.h>

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(usb))

namespace {

int EnableUsbClockRecovery() {
  const struct device* const kRcc = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
  struct stm32_pclken crs_clock = {.bus = STM32_CLOCK_BUS_APB1, .enr = RCC_APB1ENR1_CRSEN};
  if (clock_control_on(kRcc, reinterpret_cast<clock_control_subsys_t>(&crs_clock)) != 0) {
    return -EIO;
  }
  CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;
  return 0;
}

}  // namespace

SYS_INIT(EnableUsbClockRecovery, PRE_KERNEL_2, 0);

#endif
