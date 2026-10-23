/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file pedal_observer_test.cpp
 *    @brief ztest coverage of the pedal observer with a simulated crank, torque ripple and pulse train.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "pedal_observer.hpp"
#include <zephyr/ztest.h>
#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>
#include "unimoc_test.hpp"

namespace unimoc::ebike::test {

using namespace unit;

namespace {

constexpr float kPi = std::numbers::pi_v<float>;
constexpr float kPulsesPerRev = 36.0F;
constexpr Angle kPulseAngle = 2.0F * kPi * 1.0_rad / kPulsesPerRev;
constexpr Time kSubStep = 100.0_us;
constexpr Torque kOffset = 1.0_Nm;
// One revolution per second.
constexpr AngularVelocity kRevPerSecond = 2.0F * kPi * 1.0_rad_per_s;

/// Crank with a pedal-stroke torque (twice per revolution) and the pulse train of the sensor ring.
class Rig {
 public:
  explicit Rig(const Torque mean_torque = 30.0_Nm, const float ripple = 0.9F)
      : mean_torque_(mean_torque), ripple_(ripple), observer_(MakeConfig()) {}

  /// Runs @p duration with the cadence given by @p cadence(time since the start of this call).
  template <typename CadenceFn>
  void Run(const Time duration, const Time step, CadenceFn cadence, const bool pulses = true) {
    const Time kStart = now_;
    while (now_ - kStart < duration) {
      std::vector<Time> ages;
      const Time kEnd = now_ + step;
      Time sub_start = now_;
      while (sub_start < kEnd) {
        const Time kSub = std::min(kSubStep, kEnd - sub_start);
        const Angle kBefore = angle_;
        angle_ += cadence(sub_start - kStart) * kSub;
        const Ratio kFirst{std::floor((kBefore / kPulseAngle).Value()) + 1.0F};
        const Ratio kLast{std::floor((angle_ / kPulseAngle).Value())};
        for (Ratio k = kFirst; k <= kLast; k += 1.0_ratio) {
          const Ratio kFraction = (k * kPulseAngle - kBefore) / (angle_ - kBefore);
          ages.push_back(kEnd - (sub_start + kSub * kFraction));
        }
        sub_start += kSub;
      }
      now_ = kEnd;
      if (!pulses) {
        ages.clear();
      }
      const Torque kStroke = mean_torque_ * (1.0F + ripple_ * std::cos(2.0F * angle_.Value()));
      observer_.Step(step, kOffset + kStroke + load_offset_, ages);
      fast_min_ = std::min(fast_min_, observer_.FastTorque());
      fast_max_ = std::max(fast_max_, observer_.FastTorque());
    }
  }

  /// Runs at constant cadence.
  void Run(const Time duration, const AngularVelocity cadence) {
    Run(duration, 1.0_ms, [cadence](const Time) { return cadence; });
  }

  void ResetFastRange() {
    fast_min_ = observer_.FastTorque();
    fast_max_ = fast_min_;
  }

  PedalObserver& observer() { return observer_; }
  [[nodiscard]] Torque fast_range() const { return fast_max_ - fast_min_; }
  void set_load_offset(const Torque offset) { load_offset_ = offset; }

 private:
  static PedalObserverConfig MakeConfig() {
    PedalObserverConfig config;
    config.zero_offset = kOffset;
    return config;
  }

  Torque mean_torque_;
  float ripple_;
  PedalObserver observer_;
  Time now_;
  Angle angle_;
  Torque load_offset_;
  Torque fast_min_;
  Torque fast_max_;
};

}  // namespace

ZTEST(pedal_observer, test_steady_cadence_and_ripple_free_torque) {
  Rig rig(30.0_Nm, 0.9F);
  rig.Run(2.0_s, kRevPerSecond);
  rig.ResetFastRange();
  rig.Run(1.0_s, kRevPerSecond);
  const PedalObserver& kObserver = rig.observer();
  zexpect_true(kObserver.Pedaling());
  UNIMOC_EXPECT_NEAR(kObserver.Cadence(), kRevPerSecond, 0.05_rad_per_s);
  UNIMOC_EXPECT_NEAR(kObserver.MeanTorque(), 30.0_Nm, 0.5_Nm);
  UNIMOC_EXPECT_NEAR(kObserver.PedalPower(), 30.0_Nm * kRevPerSecond, 6.0_W);
  // The plain low-pass follows the stroke (30 Nm mean, 90 % ripple), the mean does not.
  zexpect_true(rig.fast_range() > 30.0_Nm, "fast torque range too small");
}

ZTEST(pedal_observer, test_mean_torque_is_ripple_free_at_any_cadence) {
  for (const float revs_per_s : {0.4F, 0.7F, 1.3F, 2.0F}) {
    Rig rig(25.0_Nm, 1.0F);
    const AngularVelocity kCadence = kRevPerSecond * revs_per_s;
    rig.Run(5.0_s, kCadence);
    // Sampled over one more revolution: every sample is within a narrow band of the mean.
    const Time kRevolution = 1.0_s / revs_per_s;
    Torque low = rig.observer().MeanTorque();
    Torque high = low;
    for (int i = 0; i < 20; ++i) {
      rig.Run(kRevolution / 20.0F, kCadence);
      low = std::min(low, rig.observer().MeanTorque());
      high = std::max(high, rig.observer().MeanTorque());
    }
    zexpect_true(high - low < 1.5_Nm,
                 "mean ripples by %f Nm at %f rev/s",
                 static_cast<double>((high - low).Value()),
                 static_cast<double>(revs_per_s));
    UNIMOC_EXPECT_NEAR(rig.observer().MeanTorque(), 25.0_Nm, 1.0_Nm);
  }
}

ZTEST(pedal_observer, test_low_cadence) {
  Rig rig;
  rig.Run(4.0_s, kRevPerSecond / 3.0F);  // 20 rpm: one pulse every 83 ms
  UNIMOC_EXPECT_NEAR(rig.observer().Cadence(), kRevPerSecond / 3.0F, 0.05_rad_per_s);
  UNIMOC_EXPECT_NEAR(rig.observer().MeanTorque(), 30.0_Nm, 1.5_Nm);
}

ZTEST(pedal_observer, test_tracks_a_cadence_ramp) {
  Rig rig;
  rig.Run(2.0_s, kRevPerSecond);
  // 60 rpm to 100 rpm in 4 s.
  rig.Run(4.0_s, 1.0_ms, [](const Time elapsed) {
    return kRevPerSecond * (1.0_ratio + 0.1F * (elapsed / 1.0_s));
  });
  UNIMOC_EXPECT_NEAR(rig.observer().Cadence(), kRevPerSecond * 1.4F, 0.15_rad_per_s);
  // Keeps up: the error while the cadence moves stays small.
  UNIMOC_EXPECT_NEAR(rig.observer().MeanTorque(), 30.0_Nm, 2.0_Nm);
}

ZTEST(pedal_observer, test_start_from_rest) {
  Rig rig;
  zexpect_false(rig.observer().Pedaling());
  rig.Run(0.5_s, 0.0_rad_per_s);
  zexpect_false(rig.observer().Pedaling());
  UNIMOC_EXPECT_NEAR(rig.observer().Cadence(), 0.0_rad_per_s, 1e-6_rad_per_s);
  // The first pulses seed the cadence, which is usable after a few pulses.
  rig.Run(0.3_s, kRevPerSecond);
  zexpect_true(rig.observer().Pedaling());
  UNIMOC_EXPECT_NEAR(rig.observer().Cadence(), kRevPerSecond, 0.5_rad_per_s);
}

ZTEST(pedal_observer, test_stop_decays_the_cadence_and_ends_the_pedaling) {
  Rig rig;
  rig.Run(2.0_s, kRevPerSecond);
  zexpect_true(rig.observer().Pedaling());
  // The crank stops: pulses cease. The overdue pulse limits the cadence well before the timeout.
  rig.Run(0.2_s, 0.0_rad_per_s);
  zexpect_true(rig.observer().Cadence() < 0.5F * kRevPerSecond);
  zexpect_true(rig.observer().PedalPower() < 0.5F * 30.0_Nm * kRevPerSecond);
  rig.Run(0.5_s, 0.0_rad_per_s);
  zexpect_false(rig.observer().Pedaling());
  UNIMOC_EXPECT_NEAR(rig.observer().Cadence(), 0.0_rad_per_s, 1e-6_rad_per_s);
  UNIMOC_EXPECT_NEAR(rig.observer().PedalPower(), 0.0_W, 1e-3_W);
  UNIMOC_EXPECT_NEAR(rig.observer().MeanTorque(), 0.0_Nm, 1e-6_Nm);
}

ZTEST(pedal_observer, test_restart_after_a_stop) {
  Rig rig;
  rig.Run(2.0_s, kRevPerSecond);
  rig.Run(1.0_s, 0.0_rad_per_s);
  zexpect_false(rig.observer().Pedaling());
  rig.Run(3.0_s, kRevPerSecond * 1.5F);
  zexpect_true(rig.observer().Pedaling());
  UNIMOC_EXPECT_NEAR(rig.observer().Cadence(), kRevPerSecond * 1.5F, 0.1_rad_per_s);
  UNIMOC_EXPECT_NEAR(rig.observer().MeanTorque(), 30.0_Nm, 1.0_Nm);
}

ZTEST(pedal_observer, test_step_size_does_not_matter) {
  // 120 rpm: a pulse every 13.9 ms, so a 10 ms and a 20 ms step hold zero, one or two pulses.
  for (const Time step : {1.0_ms, 10.0_ms, 20.0_ms}) {
    Rig rig;
    rig.Run(3.0_s, step, [](const Time) { return kRevPerSecond * 2.0F; });
    UNIMOC_EXPECT_NEAR(rig.observer().Cadence(), kRevPerSecond * 2.0F, 0.2_rad_per_s);
    UNIMOC_EXPECT_NEAR(rig.observer().MeanTorque(), 30.0_Nm, 1.5_Nm);
  }
}

ZTEST(pedal_observer, test_pulse_jitter_is_filtered) {
  // The step is 5 ms with pulses every 27.8 ms; a pulse age wobbles by up to 0.4 ms (1.4 % of the period).
  Rig rig;
  PedalObserver& observer = rig.observer();
  Time now;
  Angle angle;
  uint32_t seed = 12345U;
  const Time kStep = 5.0_ms;
  Angle next_pulse = kPulseAngle;
  for (int i = 0; i < 1000; ++i) {
    std::vector<Time> ages;
    const Angle kEndAngle = angle + kRevPerSecond * kStep;
    while (next_pulse <= kEndAngle) {
      seed = seed * 1664525U + 1013904223U;
      const float kWobble = (static_cast<float>(seed >> 16U) / 65536.0F - 0.5F) * 0.8F;
      const Time kJitter = kWobble * 1.0_ms;
      const Time kAge = (kEndAngle - next_pulse) / kRevPerSecond + kJitter;
      ages.push_back(std::clamp(kAge, 0.0_s, kStep));
      next_pulse += kPulseAngle;
    }
    angle = kEndAngle;
    now += kStep;
    observer.Step(kStep, kOffset + 30.0_Nm, ages);
  }
  UNIMOC_EXPECT_NEAR(observer.Cadence(), kRevPerSecond, 0.1_rad_per_s);
  UNIMOC_EXPECT_NEAR(observer.MeanTorque(), 30.0_Nm, 0.5_Nm);
}

ZTEST(pedal_observer, test_crank_angle_stays_wrapped_over_many_revolutions) {
  Rig rig;
  rig.Run(30.0_s, kRevPerSecond * 1.5F);
  const Angle kAngle = rig.observer().CrankAngle();
  zexpect_true(kAngle > -kPi * 1.0_rad && kAngle <= kPi * 1.0_rad);
  UNIMOC_EXPECT_NEAR(rig.observer().Cadence(), kRevPerSecond * 1.5F, 0.1_rad_per_s);
}

ZTEST(pedal_observer, test_zero_offset_follows_a_small_drift_at_standstill) {
  Rig rig(0.0_Nm, 0.0F);
  rig.set_load_offset(1.5_Nm);  // sensor drift
  rig.Run(0.8_s, 0.0_rad_per_s);
  // Not yet: the crank must stand still for the hold time first.
  UNIMOC_EXPECT_NEAR(rig.observer().ZeroOffset(), kOffset, 1e-6_Nm);
  rig.Run(180.0_s, 10.0_ms, [](const Time) { return 0.0_rad_per_s; });
  UNIMOC_EXPECT_NEAR(rig.observer().ZeroOffset(), kOffset + 1.5_Nm, 0.1_Nm);
  UNIMOC_EXPECT_NEAR(rig.observer().FastTorque(), 0.0_Nm, 0.1_Nm);
}

ZTEST(pedal_observer, test_zero_offset_ignores_a_resting_foot) {
  Rig rig(0.0_Nm, 0.0F);
  rig.set_load_offset(12.0_Nm);  // a foot on the pedal, far outside the trim window
  rig.Run(120.0_s, 10.0_ms, [](const Time) { return 0.0_rad_per_s; });
  UNIMOC_EXPECT_NEAR(rig.observer().ZeroOffset(), kOffset, 1e-6_Nm);
  UNIMOC_EXPECT_NEAR(rig.observer().FastTorque(), 12.0_Nm, 0.1_Nm);
}

ZTEST(pedal_observer, test_zero_offset_can_be_restored) {
  Rig rig(0.0_Nm, 0.0F);
  rig.observer().SetZeroOffset(2.5_Nm);
  UNIMOC_EXPECT_FLOAT_EQ(rig.observer().ZeroOffset(), 2.5_Nm);
}

}  // namespace unimoc::ebike::test

ZTEST_SUITE(pedal_observer, nullptr, nullptr, nullptr, nullptr, nullptr);
