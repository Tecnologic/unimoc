/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file ebike_io.cpp
 *    @brief native_sim e-bike hardware: a simulated rider (crank, torque) and display.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <zephyr/kernel.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include "ebike_interface.hpp"
#include "sim_ebike.hpp"

namespace unimoc::hardware {

using namespace unit;

namespace {

constexpr uint64_t kDisplayPollUs = 100'000U;
constexpr float kPulsesPerRev = 36.0F;
constexpr unit::Angle kPulseAngle = 2.0F * std::numbers::pi_v<float> * 1.0_rad / kPulsesPerRev;

// The Cyphal thread reads, a test or the registers write: one lock around the small state.
struct k_spinlock g_lock;

struct SimEbike {
  unit::AngularVelocity cadence;
  unit::Torque torque;
  unit::Angle angle;
  bool display_on{false};
  ebike::kunteng::DisplayRequest display;
  uint64_t next_poll_us{0U};
  uint64_t last_us{0U};
  bool started{false};
  bool manual_time{false};
  uint64_t manual_us{0U};
  std::array<uint8_t, ebike::kunteng::kDisplayFrameSize> rx{};
  std::size_t rx_size{0U};
  std::size_t rx_read{0U};
  std::array<uint8_t, ebike::kunteng::kControllerFrameSize> status{};
  uint32_t status_count{0U};
};
SimEbike g_sim;

uint64_t NowUs() noexcept {
  return g_sim.manual_time ? g_sim.manual_us : k_ticks_to_us_floor64(k_uptime_ticks());
}

unit::Torque ReadCrankTorque() noexcept {
  const k_spinlock_key_t kKey = k_spin_lock(&g_lock);
  const unit::Torque kTorque = g_sim.torque;
  k_spin_unlock(&g_lock, kKey);
  return kTorque;
}

std::size_t ReadCadencePulses(const std::span<unit::Time> ages) noexcept {
  const k_spinlock_key_t kKey = k_spin_lock(&g_lock);
  const uint64_t kNow = NowUs();
  std::size_t count = 0U;
  if (g_sim.started && kNow > g_sim.last_us) {
    const unit::Time kElapsed = 1.0_us * static_cast<float>(kNow - g_sim.last_us);
    const unit::Angle kBefore = g_sim.angle;
    g_sim.angle += g_sim.cadence * kElapsed;
    const float kFirst = std::floor((kBefore / kPulseAngle).Value()) + 1.0F;
    const float kLast = std::floor((g_sim.angle / kPulseAngle).Value());
    for (float pulse = kFirst; pulse <= kLast && count < ages.size(); pulse += 1.0F) {
      const unit::Ratio kFraction = ((pulse * kPulseAngle) - kBefore) / (g_sim.angle - kBefore);
      ages[count++] = kElapsed * (1.0_ratio - kFraction);
    }
  }
  g_sim.started = true;
  g_sim.last_us = kNow;
  k_spin_unlock(&g_lock, kKey);
  return count;
}

std::size_t ReadDisplay(const std::span<uint8_t> bytes) noexcept {
  const k_spinlock_key_t kKey = k_spin_lock(&g_lock);
  const uint64_t kNow = NowUs();
  if (g_sim.display_on && kNow >= g_sim.next_poll_us && g_sim.rx_read >= g_sim.rx_size) {
    g_sim.rx = ebike::kunteng::EncodeDisplayRequest(g_sim.display);
    g_sim.rx_size = g_sim.rx.size();
    g_sim.rx_read = 0U;
    g_sim.next_poll_us = kNow + kDisplayPollUs;
  }
  std::size_t count = 0U;
  while (count < bytes.size() && g_sim.rx_read < g_sim.rx_size) {
    bytes[count++] = g_sim.rx[g_sim.rx_read++];
  }
  k_spin_unlock(&g_lock, kKey);
  return count;
}

void WriteDisplay(const std::span<const uint8_t> bytes) noexcept {
  const k_spinlock_key_t kKey = k_spin_lock(&g_lock);
  if (bytes.size() == g_sim.status.size()) {
    std::copy(bytes.begin(), bytes.end(), g_sim.status.begin());
    ++g_sim.status_count;
  }
  k_spin_unlock(&g_lock, kKey);
}

const EbikeIo kIo{.read_crank_torque = ReadCrankTorque,
                  .read_cadence_pulses = ReadCadencePulses,
                  .read_display = ReadDisplay,
                  .write_display = WriteDisplay};

}  // namespace

const EbikeIo& GetEbikeIo() noexcept { return kIo; }

void StartEbikeIo() noexcept {}

namespace sim {

void SetPedal(const unit::AngularVelocity cadence, const unit::Torque torque) noexcept {
  const k_spinlock_key_t kKey = k_spin_lock(&g_lock);
  g_sim.cadence = cadence;
  g_sim.torque = torque;
  k_spin_unlock(&g_lock, kKey);
}

void SetDisplay(const uint8_t level, const uint8_t max_speed_kmh, const bool light) noexcept {
  const k_spinlock_key_t kKey = k_spin_lock(&g_lock);
  g_sim.display.assist_level = level;
  g_sim.display.max_speed = max_speed_kmh;
  g_sim.display.light = light;
  g_sim.display_on = true;
  g_sim.next_poll_us = 0U;
  k_spin_unlock(&g_lock, kKey);
}

void SilenceDisplay() noexcept {
  const k_spinlock_key_t kKey = k_spin_lock(&g_lock);
  g_sim.display_on = false;
  g_sim.status = {};
  g_sim.status_count = 0U;
  k_spin_unlock(&g_lock, kKey);
}

std::array<uint8_t, ebike::kunteng::kControllerFrameSize> LastDisplayStatus() noexcept {
  const k_spinlock_key_t kKey = k_spin_lock(&g_lock);
  const auto kStatus = g_sim.status;
  k_spin_unlock(&g_lock, kKey);
  return kStatus;
}

uint32_t DisplayStatusCount() noexcept {
  const k_spinlock_key_t kKey = k_spin_lock(&g_lock);
  const uint32_t kCount = g_sim.status_count;
  k_spin_unlock(&g_lock, kKey);
  return kCount;
}

void UseManualTime(const bool manual) noexcept {
  const k_spinlock_key_t kKey = k_spin_lock(&g_lock);
  g_sim.manual_time = manual;
  g_sim.started = false;
  k_spin_unlock(&g_lock, kKey);
}

void SetTime(const uint64_t now_us) noexcept {
  const k_spinlock_key_t kKey = k_spin_lock(&g_lock);
  g_sim.manual_us = now_us;
  k_spin_unlock(&g_lock, kKey);
}

}  // namespace sim

}  // namespace unimoc::hardware
