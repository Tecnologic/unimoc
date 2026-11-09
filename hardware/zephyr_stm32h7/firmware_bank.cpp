/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file firmware_bank.cpp
 *    @brief The firmware update bank of the STM32H7 platform: slot 1 of MCUboot in flash bank 2.
 *
 *    Flash layout: dts/unimoc/stm32h743_flash_layout.dtsi. The update is a
 *    test swap (MCUboot swap using move): the new image runs once and calls
 *    boot_write_img_confirmed() after kConfirmDelayUs; a reset before that
 *    swaps the old image back.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "firmware_bank.hpp"
#include <zephyr/devicetree.h>
#include <zephyr/dfu/flash_img.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/storage/flash_map.h>
#include <algorithm>
#include <array>

namespace unimoc::hardware {
namespace {

constexpr uint8_t kSlot0 = PARTITION_ID(slot0_partition);
constexpr uint8_t kSlot1 = PARTITION_ID(slot1_partition);

/// Sector of the flash (all 128 KB on the STM32H743).
constexpr uint32_t kSectorSize = DT_PROP(DT_GPARENT(DT_NODELABEL(slot1_partition)), erase_block_size);

/// What an image of MCUboot's swap using move may take: the slot less the sector it moves
/// and the sector of the trailer (the application is linked to the same limit).
constexpr uint32_t kCapacity = PARTITION_SIZE(slot1_partition) - 2U * kSectorSize;

/// The new image runs this long (from the start of the system) before it confirms itself.
constexpr uint64_t kConfirmDelayUs = static_cast<uint64_t>(CONFIG_UNIMOC_UPDATE_CONFIRM_DELAY_MS) * 1000U;

flash_img_context g_image{};
bool g_open{false};
uint32_t g_next_offset{0U};  ///< Bytes passed to the writer (it counts only those it has flushed to flash).

bool Begin() noexcept {
  // A new download after an aborted one starts again; the sectors are erased as the writes reach them.
  g_open = (flash_img_init_id(&g_image, kSlot1) == 0);
  g_next_offset = 0U;
  return g_open;
}

bool Write(const uint32_t offset, const std::span<const std::byte> data) noexcept {
  if (!g_open || offset != g_next_offset) {  // the writer is sequential
    return false;
  }
  if (flash_img_buffered_write(&g_image, reinterpret_cast<const uint8_t*>(data.data()), data.size(), false) !=
      0) {
    return false;
  }
  g_next_offset += static_cast<uint32_t>(data.size());
  return true;
}

bool Finish() noexcept {
  if (!g_open) {
    return false;
  }
  g_open = false;
  return flash_img_buffered_write(&g_image, nullptr, 0U, true) == 0;
}

bool Read(const uint32_t offset, const std::span<std::byte> data) noexcept {
  const flash_area* area = nullptr;
  if (flash_area_open(kSlot1, &area) != 0) {
    return false;
  }
  const int kResult = flash_area_read(area, static_cast<off_t>(offset), data.data(), data.size());
  flash_area_close(area);
  return kResult == 0;
}

/// (major, minor, revision) of the image in a slot as MCUboot compares them; false: no image header.
bool ReadVersion(const uint8_t area_id, std::array<uint32_t, 3>& version) noexcept {
  mcuboot_img_header header{};
  if (boot_read_bank_header(area_id, &header, sizeof(header)) != 0 || header.mcuboot_version != 1U) {
    return false;
  }
  version = {header.h.v1.sem_ver.major, header.h.v1.sem_ver.minor, header.h.v1.sem_ver.revision};
  return true;
}

/// MCUboot (downgrade prevention) leaves an image that is not newer in the slot and starts the old one.
bool IsNewerThanRunning(const std::array<uint32_t, 3>& update) noexcept {
  std::array<uint32_t, 3> running{};
  return !ReadVersion(kSlot0, running) ||
         std::lexicographical_compare(running.begin(), running.end(), update.begin(), update.end());
}

cyphal::ImageStatus Activate() noexcept {
  std::array<uint32_t, 3> update{};
  if (!ReadVersion(kSlot1, update)) {
    return cyphal::ImageStatus::kNoBootImage;
  }
  if (!IsNewerThanRunning(update)) {
    return cyphal::ImageStatus::kNotNewer;
  }
  if (boot_request_upgrade(BOOT_UPGRADE_TEST) != 0) {
    return cyphal::ImageStatus::kNoBootImage;
  }
  return cyphal::ImageStatus::kOk;
}

constexpr cyphal::FirmwareBank kBank{Begin, Write, Finish, Read, Activate, kCapacity};

}  // namespace

const cyphal::FirmwareBank& UpdateBank() noexcept { return kBank; }

bool ConfirmRunningImage(const uint64_t now_us) noexcept {
  static bool confirmed = boot_is_img_confirmed();
  if (confirmed || now_us < kConfirmDelayUs) {
    return false;
  }
  if (boot_write_img_confirmed() != 0) {
    return false;
  }
  confirmed = true;
  return true;
}

}  // namespace unimoc::hardware
