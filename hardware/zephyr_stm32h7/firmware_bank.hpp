/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file firmware_bank.hpp
 *    @brief The firmware update bank of the STM32H7 platform: slot 1 of MCUboot in flash bank 2.
 *
 *    Built with CONFIG_UNIMOC_FIRMWARE_UPDATE_FLASH (MCUboot in the build).
 *    Thread context only (the Cyphal thread).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>
#include "cyphal/firmware_update.hpp"

namespace unimoc::hardware {

/**
 * @brief Slot 1 of MCUboot as the bank of a firmware update.
 *
 * Writes go through the DFU image writer (32 byte flash words, sectors erased as the
 * writes reach them); activate() checks the MCUboot header of the image in the bank,
 * that its version is higher than the running one, and requests a test swap at the next
 * start.
 */
[[nodiscard]] const cyphal::FirmwareBank& UpdateBank() noexcept;

/**
 * @brief Confirms the running image once it has run long enough; call from the Cyphal thread.
 *
 * After a swap the new image is on trial: if it is not confirmed, the next reset swaps the old
 * image back. A normal start (or an image confirmed before) is confirmed already.
 *
 * @param now_us Time since the start.
 * @return True in the call that confirmed the image.
 */
bool ConfirmRunningImage(uint64_t now_us) noexcept;

}  // namespace unimoc::hardware
