/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file hall_fusion_test.cpp
 *    @brief ztest coverage of the Hall fusion with generated Hall edges (M5, M6).
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "hall_fusion.hpp"
#include <zephyr/ztest.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <vector>
#include "angle_wrap.hpp"
#include "hall_capture.hpp"
#include "mechanical_observer.hpp"
#include "unimoc_test.hpp"

namespace unimoc::observer::test {

using namespace unit;

namespace {

constexpr float kPi = std::numbers::pi_v<float>;
constexpr Angle kDegree = 1.0_deg;
constexpr Time kTickPeriod{1e-9F};         // nanosecond timestamps, as on native_sim
constexpr uint32_t kCycleTicks = 62'500U;  // 16 kHz
constexpr Time kDt{62.5e-6F};
const system::Rotor<Current> kNoCurrent{Current{}, Current{}};

// Code of an electrical angle for offset 0 (sector k starts at k * 60 degrees).
uint8_t CodeOf(const Angle angle) {
  constexpr std::array<uint8_t, 6> kCodes = {5U, 1U, 3U, 2U, 6U, 4U};
  // Table index: plain numbers (fmod keeps many revolutions exact).
  float wrapped = std::fmod(angle.Value(), 2.0F * kPi);
  wrapped = wrapped < 0.0F ? wrapped + 2.0F * kPi : wrapped;
  const auto kSector = static_cast<std::size_t>(wrapped / (kPi / 3.0F)) % kCodes.size();
  return kCodes[kSector];
}

struct Rig {
  explicit Rig(const AngleVariance edge_noise = 3e-3_rad2) {
    settings::NvmSettings settings{};
    settings.motor_j = 1e-3_kg_m2;
    settings.motor_omega_max = 10000.0_rad_per_s;
    settings.motor_omega_min = -10000.0_rad_per_s;
    settings.mech_obs_q = 1e-5F;
    settings.hall_enable = 1U;
    settings.hall_r_edge = edge_noise;
    settings.hall_r_sector = 3e-2_rad2;
    observer.Init(settings);
    fusion.Init(settings);
  }

  /// One observer cycle: the events since the last cycle, reference time `now`.
  void Cycle(const std::vector<HallEvent>& events) {
    observer.PredictCovariance(kDt);
    fusion.Update(events, now, kTickPeriod, observer);
    observer.Predict(kNoCurrent, kDt);
    now += kCycleTicks;
  }

  /// Runs a rotor at constant electrical speed from @p angle; returns the true angle at `now`.
  Angle Spin(Angle angle, const AngularVelocity speed, const int cycles) {
    constexpr Time kSubStep{1e-6F};
    for (int i = 0; i < cycles; ++i) {
      std::vector<HallEvent> events;
      // Sub-steps of 1 us find the edges of this cycle.
      for (uint32_t tick = 1000U; tick <= kCycleTicks; tick += 1000U) {
        const Angle kNext = angle + (speed * kSubStep);
        if (CodeOf(kNext) != CodeOf(angle)) {
          events.push_back(HallEvent{CodeOf(kNext), now - kCycleTicks + tick});
        }
        angle = kNext;
      }
      // The estimate refers to `now` after the cycle: evaluate before advancing.
      Cycle(events);
    }
    return angle;
  }

  MechanicalObserver<float> observer;
  HallFusion<float> fusion;
  uint32_t now{kCycleTicks};
};

}  // namespace

ZTEST(hall_fusion, test_standstill_estimate_moves_into_the_sector_and_stays) {
  Rig rig;
  const Angle kTrue = 100.0F * kDegree;  // sector 1: [60, 120) degrees
  rig.Cycle({HallEvent{CodeOf(kTrue), 0U}});
  for (int i = 0; i < 12000; ++i) {  // 0.75 s: past the standstill time
    rig.Cycle({});
  }
  const Angle kSettled = rig.observer.theta;
  zexpect_true(kSettled >= 59.0F * kDegree && kSettled <= 121.0F * kDegree,
               "theta %f deg",
               static_cast<double>((kSettled / kDegree).Value()));
  for (int i = 0; i < 16000; ++i) {  // one more second: no drift
    rig.Cycle({});
  }
  zexpect_within(rig.observer.theta,
                 kSettled,
                 1.0F * kDegree,
                 "theta %f deg after %f deg, omega %f",
                 static_cast<double>((rig.observer.theta / kDegree).Value()),
                 static_cast<double>((kSettled / kDegree).Value()),
                 static_cast<double>(rig.observer.omega.Value()));
  UNIMOC_EXPECT_NEAR(rig.observer.omega, 0.0_rad_per_s, 1.0_rad_per_s);
  zexpect_equal(rig.fusion.Edges(), 0U);
}

ZTEST(hall_fusion, test_edges_give_angle_and_speed) {
  Rig rig;
  constexpr AngularVelocity kSpeed = 300.0_rad_per_s;  // electrical, about 48 Hz
  Angle angle = 0.3_rad;
  rig.Cycle({HallEvent{CodeOf(angle), 0U}});
  angle = rig.Spin(angle, kSpeed, 3200);  // 0.2 s
  zexpect_true(rig.fusion.Edges() > 50U);
  UNIMOC_EXPECT_NEAR(rig.observer.omega, kSpeed, 0.03F * kSpeed);
  // The estimate after the cycle refers to the next reference time.
  const Angle kError = WrapAngle(rig.observer.theta - (angle + (kSpeed * kDt)));
  UNIMOC_EXPECT_NEAR(kError, 0.0_rad, 3.0F * kDegree);
  zexpect_equal(rig.fusion.InvalidTransitions(), 0U);
}

ZTEST(hall_fusion, test_reverse_rotation_uses_the_same_boundaries) {
  Rig rig;
  constexpr AngularVelocity kSpeed = -200.0_rad_per_s;
  Angle angle = -0.2_rad;
  rig.Cycle({HallEvent{CodeOf(angle), 0U}});
  angle = rig.Spin(angle, kSpeed, 16000);
  zexpect_within(rig.observer.omega,
                 kSpeed,
                 0.03F * -kSpeed,
                 "omega %f, edges %u, invalid %u",
                 static_cast<double>(rig.observer.omega.Value()),
                 rig.fusion.Edges(),
                 rig.fusion.InvalidTransitions());
  const Angle kError = WrapAngle(rig.observer.theta - (angle + (kSpeed * kDt)));
  UNIMOC_EXPECT_NEAR(kError, 0.0_rad, 3.0F * kDegree);
}

ZTEST(hall_fusion, test_missed_edges_and_invalid_codes_are_counted_not_fused) {
  Rig rig;
  rig.Cycle({HallEvent{5U, 0U}});  // sector 0
  const Angle kBefore = rig.observer.theta;
  rig.Cycle({HallEvent{3U, 100U}});  // sector 2: one edge missed
  zexpect_equal(rig.fusion.InvalidTransitions(), 1U);
  zexpect_equal(rig.fusion.Edges(), 0U);
  rig.Cycle({HallEvent{7U, 200U}, HallEvent{0U, 300U}});  // broken sensor
  zexpect_equal(rig.fusion.InvalidCodes(), 2U);
  zexpect_equal(rig.fusion.Code(), 0U);
  // Without a valid code no sector measurement: the estimate is left alone.
  const Angle kNow = rig.observer.theta;
  for (int i = 0; i < 100; ++i) {
    rig.Cycle({});
  }
  UNIMOC_EXPECT_NEAR(rig.observer.theta, kNow, 1e-4_rad);
  (void)kBefore;
}

ZTEST(hall_fusion, test_edge_noise_weights_the_edge) {
  Rig precise{3e-3_rad2};
  Rig noisy{3e-1_rad2};
  for (Rig* rig : {&precise, &noisy}) {
    rig->Cycle({HallEvent{5U, 0U}});  // sector 0: the estimate starts at its center, 30 degrees
    for (int i = 0; i < 100; ++i) {
      rig->Cycle({});
    }
  }
  const Angle kBefore = precise.observer.theta;
  UNIMOC_EXPECT_NEAR(kBefore, 30.0F * kDegree, 0.1F * kDegree);
  UNIMOC_EXPECT_FLOAT_EQ(noisy.observer.theta, kBefore);
  // The edge into sector 1 (60 degrees) comes just after the reference time:
  // only the edge is fused, the estimate still lies in the sector of the code.
  for (Rig* rig : {&precise, &noisy}) {
    rig->Cycle({HallEvent{1U, rig->now + 1000U}});
  }
  zexpect_equal(precise.fusion.Edges(), 1U);
  zexpect_equal(noisy.fusion.Edges(), 1U);
  const Angle kPreciseStep = precise.observer.theta - kBefore;
  const Angle kNoisyStep = noisy.observer.theta - kBefore;
  zexpect_true(kNoisyStep > 0.0_rad);
  zexpect_true(kPreciseStep > 10.0F * kNoisyStep);
}

ZTEST(hall_fusion, test_init_loads_the_hall_settings) {
  settings::NvmSettings settings{};
  HallFusion<float> fusion;
  fusion.Init(settings);
  zexpect_false(fusion.Enabled());
  settings.hall_enable = 1U;
  fusion.Init(settings);
  zexpect_true(fusion.Enabled());
}

}  // namespace unimoc::observer::test

ZTEST_SUITE(hall_fusion, nullptr, nullptr, nullptr, nullptr, nullptr);
