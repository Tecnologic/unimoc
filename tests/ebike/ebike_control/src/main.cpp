/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file main.cpp
 *    @brief ztest coverage of the e-bike control with a fake pedal sensor and display line.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <zephyr/ztest.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <numbers>
#include <vector>
#include "ebike_control.hpp"
#include "unimoc_test.hpp"

namespace {

using namespace unimoc::unit;
using namespace unimoc::ebike;
using unimoc::hardware::EbikeIo;

constexpr float kPi = std::numbers::pi_v<float>;
constexpr float kPulsesPerRev = 36.0F;
constexpr Angle kPulseAngle = 2.0F * kPi * 1.0_rad / kPulsesPerRev;
constexpr uint64_t kMs = 1000U;

/// State of the fake hardware: what the control reads and writes.
struct Fake {
  Torque torque;
  std::vector<Time> ages;
  std::deque<uint8_t> display_rx;
  std::vector<std::vector<uint8_t>> display_tx;
};
Fake g_fake;

Torque ReadTorque() noexcept { return g_fake.torque; }

std::size_t ReadPulses(const std::span<Time> ages) noexcept {
  const std::size_t kCount = std::min(ages.size(), g_fake.ages.size());
  for (std::size_t i = 0U; i < kCount; ++i) {
    ages[i] = g_fake.ages[i];
  }
  g_fake.ages.clear();
  return kCount;
}

std::size_t ReadDisplay(const std::span<uint8_t> bytes) noexcept {
  std::size_t count = 0U;
  while (count < bytes.size() && !g_fake.display_rx.empty()) {
    bytes[count++] = g_fake.display_rx.front();
    g_fake.display_rx.pop_front();
  }
  return count;
}

void WriteDisplay(const std::span<const uint8_t> bytes) noexcept {
  g_fake.display_tx.emplace_back(bytes.begin(), bytes.end());
}

const EbikeIo kIo{ReadTorque, ReadPulses, ReadDisplay, WriteDisplay};

/// Rider, vehicle and display around an EbikeControl; one call of Step() is one millisecond.
class Rig {
 public:
  Rig() : control_(kIo) {
    g_fake = Fake{};
    EbikeParameters parameters;
    parameters.assist.power_scale = 1.0e9_W;  // no exponential
    parameters.assist.torque_deadband = 0.0_Nm;
    parameters.engage_hold = 500.0_ms;
    parameters.rearm_pedal_stop = 500.0_ms;
    parameters.display_timeout = 1.0_s;
    parameters.pedal.zero_offset = 0.0_Nm;
    control_.Configure(parameters);
    vehicle.torque_min = -40.0_Nm;
    vehicle.torque_max = 40.0_Nm;
    vehicle.dc_link = 38.0_V;
    vehicle.motor_temperature = 40.0_degC;
  }

  /// The display asks for @p level with a speed limit of @p max_speed km/h, now and every 100 ms.
  void Request(const uint8_t level, const uint8_t max_speed = 25U, const bool light = false) {
    request_.assist_level = level;
    request_.max_speed = max_speed;
    request_.light = light;
    polling_ = true;
    SendRequest();
  }

  /// The display falls silent.
  void StopDisplay() { polling_ = false; }

  /// Pedals at @p cadence with a mean torque of @p pedal_torque (no stroke ripple); zero cadence: standing.
  void Pedal(const AngularVelocity cadence, const Torque pedal_torque) {
    cadence_ = cadence;
    pedal_torque_ = pedal_torque;
  }

  void Step(const int milliseconds = 1) {
    for (int i = 0; i < milliseconds; ++i) {
      now_us_ += kMs;
      if (polling_ && now_us_ % (100U * kMs) == 0U) {
        SendRequest();
      }
      const Time kStep = 1.0_ms;
      const Angle kBefore = angle_;
      angle_ += cadence_ * kStep;
      const float kFirst = std::floor((kBefore / kPulseAngle).Value()) + 1.0F;
      const float kLast = std::floor((angle_ / kPulseAngle).Value());
      for (float k = kFirst; k <= kLast; k += 1.0F) {
        const Ratio kFraction = (k * kPulseAngle - kBefore) / (angle_ - kBefore);
        g_fake.ages.push_back(kStep * (1.0_ratio - kFraction));
      }
      g_fake.torque = pedal_torque_;
      vehicle.engaged = output.engage;  // the drive follows the request at once in this rig
      output = control_.Update(now_us_, vehicle);
      if (!output.engage) {
        vehicle.engaged = false;
      }
    }
  }

  EbikeControl& control() { return control_; }

  EbikeVehicle vehicle;
  EbikeOutput output;

 private:
  void SendRequest() {
    const auto kFrame = kunteng::EncodeDisplayRequest(request_);
    g_fake.display_rx.insert(g_fake.display_rx.end(), kFrame.begin(), kFrame.end());
  }

  EbikeControl control_;
  kunteng::DisplayRequest request_;
  bool polling_{false};
  uint64_t now_us_{0U};
  Angle angle_;
  AngularVelocity cadence_;
  Torque pedal_torque_;
};

constexpr AngularVelocity kCadence = 6.0_rad_per_s;  // about 57 rpm

/// A rig whose display is alive at level 3 and whose rider pedals; settled.
void Riding(Rig& rig, const uint8_t level = 3U, const Torque pedal_torque = 20.0_Nm) {
  rig.Request(level);
  rig.Pedal(kCadence, pedal_torque);
  rig.Step(2500);
}

}  // namespace

ZTEST(ebike_control, test_no_assist_without_a_display) {
  Rig rig;
  rig.Pedal(kCadence, 20.0_Nm);
  rig.Step(3000);
  zexpect_true(rig.control().Pedal().Pedaling());
  zexpect_false(rig.control().DisplayAlive());
  zexpect_false(rig.output.engage);
  UNIMOC_EXPECT_NEAR(rig.output.torque, 0.0_Nm, 1e-6_Nm);
}

ZTEST(ebike_control, test_assist_follows_the_pedals_at_the_display_level) {
  Rig rig;
  Riding(rig);
  zexpect_true(rig.control().DisplayAlive());
  zexpect_true(rig.output.engage);
  // Level 3 (ratio 1.0) at a standing motor: torque ratio 0.5 times the pedal torque.
  UNIMOC_EXPECT_NEAR(rig.output.torque, 10.0_Nm, 0.3_Nm);
  UNIMOC_EXPECT_NEAR(rig.control().Pedal().Cadence(), kCadence, 0.1_rad_per_s);
}

ZTEST(ebike_control, test_torque_limit_of_the_drive) {
  Rig rig;
  rig.vehicle.torque_max = 4.0_Nm;
  Riding(rig);
  UNIMOC_EXPECT_NEAR(rig.output.torque, 4.0_Nm, 1e-3_Nm);
}

ZTEST(ebike_control, test_level_zero_gives_no_assist_and_no_engagement) {
  Rig rig;
  Riding(rig, 0U);
  zexpect_false(rig.output.engage);
  UNIMOC_EXPECT_NEAR(rig.output.torque, 0.0_Nm, 1e-6_Nm);
}

ZTEST(ebike_control, test_stopping_the_pedals_ramps_down_then_releases_the_drive) {
  Rig rig;
  Riding(rig);
  rig.Pedal(0.0_rad_per_s, 0.0_Nm);
  // The crank counts as stopped after 0.5 s without a pulse; the torque is gone long before.
  rig.Step(300);
  UNIMOC_EXPECT_NEAR(rig.output.torque, 0.0_Nm, 1e-6_Nm);
  rig.Step(400);
  zexpect_false(rig.control().Pedal().Pedaling());
  // The drive stays engaged for the engage hold (0.5 s) at zero torque ...
  zexpect_true(rig.output.engage);
  UNIMOC_EXPECT_NEAR(rig.output.torque, 0.0_Nm, 1e-6_Nm);
  // ... and is released after it.
  rig.Step(700);
  zexpect_false(rig.output.engage);
}

ZTEST(ebike_control, test_a_fault_latches_until_the_pedals_stood_still) {
  Rig rig;
  Riding(rig);
  zexpect_true(rig.output.engage);
  // The drive tripped and waits for a re-arm; the rider keeps pedaling.
  rig.vehicle.faulted = true;
  rig.Step(5);
  rig.vehicle.faulted = false;  // the drive's re-arm flag clears once the engage request ends
  rig.Step(3000);
  zexpect_false(rig.output.engage);
  UNIMOC_EXPECT_NEAR(rig.output.torque, 0.0_Nm, 1e-6_Nm);
  // The pedals stand still for the re-arm time, then the rider starts again.
  rig.Pedal(0.0_rad_per_s, 0.0_Nm);
  rig.Step(1500);
  rig.Pedal(kCadence, 20.0_Nm);
  rig.Step(1500);
  zexpect_true(rig.output.engage);
  UNIMOC_EXPECT_NEAR(rig.output.torque, 10.0_Nm, 0.5_Nm);
}

ZTEST(ebike_control, test_inhibit_takes_the_drive_off_until_the_pedals_stood_still) {
  Rig rig;
  Riding(rig);
  zexpect_true(rig.output.engage);
  rig.control().Inhibit();
  rig.Step(3000);  // the rider keeps pedaling
  zexpect_false(rig.output.engage);
  UNIMOC_EXPECT_NEAR(rig.output.torque, 0.0_Nm, 1e-6_Nm);
  rig.Pedal(0.0_rad_per_s, 0.0_Nm);
  rig.Step(1500);
  rig.Pedal(kCadence, 20.0_Nm);
  rig.Step(1500);
  zexpect_true(rig.output.engage);
}

ZTEST(ebike_control, test_a_lost_display_ends_the_assist) {
  Rig rig;
  Riding(rig);
  zexpect_true(rig.output.engage);
  // No further requests: after the display timeout the level counts as 0.
  rig.StopDisplay();
  rig.Step(1500);
  zexpect_false(rig.control().DisplayAlive());
  UNIMOC_EXPECT_NEAR(rig.output.torque, 0.0_Nm, 1e-6_Nm);
  rig.Step(1000);
  zexpect_false(rig.output.engage);
}

ZTEST(ebike_control, test_speed_limit_of_the_display) {
  Rig rig;
  // 25 km/h is 6.94 m/s: on a 2.1 m wheel the hub motor turns at 2 pi * 6.94 / 2.1 = 20.8 rad/s.
  Riding(rig);
  rig.vehicle.motor_speed = 19.0_rad_per_s;
  rig.Request(3U);
  rig.Step(1200);
  zexpect_true(rig.output.torque > 0.5_Nm);
  rig.vehicle.motor_speed = 21.0_rad_per_s;
  rig.Request(3U);
  rig.Step(1200);
  UNIMOC_EXPECT_NEAR(rig.output.torque, 0.0_Nm, 1e-6_Nm);
}

ZTEST(ebike_control, test_reversed_drive_flips_the_sign) {
  Rig rig;
  EbikeParameters parameters;
  parameters.assist.power_scale = 1.0e9_W;
  parameters.assist.torque_deadband = 0.0_Nm;
  parameters.pedal.zero_offset = 0.0_Nm;
  parameters.reversed = true;
  rig.control().Configure(parameters);
  Riding(rig);
  UNIMOC_EXPECT_NEAR(rig.output.torque, -10.0_Nm, 0.3_Nm);
}

ZTEST(ebike_control, test_reversed_drive_is_limited_by_the_negative_torque_limit) {
  Rig rig;
  EbikeParameters parameters;
  parameters.assist.power_scale = 1.0e9_W;
  parameters.assist.torque_deadband = 0.0_Nm;
  parameters.pedal.zero_offset = 0.0_Nm;
  parameters.reversed = true;
  rig.control().Configure(parameters);
  rig.vehicle.torque_min = -3.0_Nm;
  rig.vehicle.torque_max = 40.0_Nm;
  Riding(rig);
  UNIMOC_EXPECT_NEAR(rig.output.torque, -3.0_Nm, 1e-3_Nm);
}

ZTEST(ebike_control, test_every_request_is_answered_with_the_status_frame) {
  Rig rig;
  rig.vehicle.motor_speed = 10.0_rad_per_s;
  rig.vehicle.battery_current = 0.5_A;
  rig.Request(2U);
  rig.Step(5);
  zexpect_equal(g_fake.display_tx.size(), 1U);
  const auto& frame = g_fake.display_tx.front();
  zexpect_equal(frame.size(), kunteng::kControllerFrameSize);
  zexpect_equal(frame[0], 65U);
  zexpect_equal(frame[1], static_cast<uint8_t>(kunteng::BatteryLevel::kThreeBars));  // 3.8 V per cell
  zexpect_equal(frame[2], 36U);
  // One wheel revolution is 2 pi rad of the hub motor: 2 pi / 10 rad/s = 628 ms.
  const unsigned kPeriod = (static_cast<unsigned>(frame[3]) << 8U) | frame[4];
  zexpect_true(kPeriod >= 627U && kPeriod <= 629U, "period %u ms", kPeriod);
  zexpect_equal(frame[5], 0U);
  zexpect_equal(frame[8], 2U);   // 0.5 A in quarter amps
  zexpect_equal(frame[9], 25U);  // 40 degC minus 15
  // A second request is answered again.
  rig.Request(2U);
  rig.Step(5);
  zexpect_equal(g_fake.display_tx.size(), 2U);
}

ZTEST(ebike_control, test_standstill_reports_the_longest_wheel_period) {
  Rig rig;
  rig.Request(1U);
  rig.Step(5);
  const auto& frame = g_fake.display_tx.front();
  zexpect_equal(frame[3], 0xFFU);
  zexpect_equal(frame[4], 0xFFU);
}

ZTEST(ebike_control, test_battery_bars_have_hysteresis_and_hold_under_load) {
  Rig rig;
  const auto bars = [&rig]() {
    rig.Request(1U);
    rig.Step(5);
    return g_fake.display_tx.back()[1];
  };
  rig.vehicle.dc_link = 38.0_V;
  zexpect_equal(bars(), static_cast<uint8_t>(kunteng::BatteryLevel::kThreeBars));
  rig.vehicle.dc_link = 37.6_V;  // inside the hysteresis band of the 3.75 V threshold
  zexpect_equal(bars(), static_cast<uint8_t>(kunteng::BatteryLevel::kThreeBars));
  rig.vehicle.dc_link = 36.9_V;  // below it
  zexpect_equal(bars(), static_cast<uint8_t>(kunteng::BatteryLevel::kTwoBars));
  rig.vehicle.dc_link = 30.0_V;
  rig.vehicle.battery_current = 15.0_A;  // under load the bars hold
  zexpect_equal(bars(), static_cast<uint8_t>(kunteng::BatteryLevel::kTwoBars));
  rig.vehicle.battery_current = 0.0_A;
  zexpect_equal(bars(), static_cast<uint8_t>(kunteng::BatteryLevel::kBelowEmpty));
}

ZTEST(ebike_control, test_a_fault_is_reported_to_the_display) {
  Rig rig;
  rig.vehicle.faulted = true;
  rig.Request(1U);
  rig.Step(5);
  zexpect_equal(g_fake.display_tx.back()[5], 1U);
}

ZTEST(ebike_control, test_the_light_request_is_passed_on) {
  Rig rig;
  rig.Request(1U, 25U, true);
  rig.Step(5);
  zexpect_true(rig.output.light);
  rig.Request(1U, 25U, false);
  rig.Step(5);
  zexpect_false(rig.output.light);
}

ZTEST_SUITE(ebike_control, nullptr, nullptr, nullptr, nullptr, nullptr);
