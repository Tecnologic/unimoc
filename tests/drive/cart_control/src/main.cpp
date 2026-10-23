/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file main.cpp
 *    @brief ztest coverage of the cart control: torque source, limp-home,
 *           ramps and the battery limits of the BMS.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <zephyr/ztest.h>
#include <cmath>
#include <cstdint>
#include <limits>
#include "cart_control.hpp"
#include "unimoc_test.hpp"

namespace {

using namespace unimoc::unit;
using unimoc::drive::CartBattery;
using unimoc::drive::CartControl;
using unimoc::drive::CartMode;
using unimoc::drive::CartOutput;
using unimoc::drive::CartParameters;
using unimoc::drive::CartPedals;
using unimoc::drive::Contactor;

constexpr uint64_t kMs = 1000U;
constexpr Ratio kNan{std::numeric_limits<float>::quiet_NaN()};

/// Timeout 100 ms, ramp 50 N m/s, limp-home 5 N m, standstill 1 rad/s, half the battery.
CartParameters Parameters(const bool reversed = false) {
  return CartParameters{.wheel = 0U,
                        .reversed = reversed,
                        .timeout = 0.1_s,
                        .torque_ramp = 50.0_Nm / 1.0_s,
                        .limp_home_torque = 5.0_Nm,
                        .standstill_speed = 1.0_rad_per_s,
                        .battery_share = 0.5_ratio};
}

CartPedals Pedals(const Ratio drive, const Ratio brake) {
  return CartPedals{.drive = {drive, drive},
                    .brake = {brake, brake},
                    .drive_plausible = true,
                    .brake_plausible = true};
}

/// The control at 1 ms steps, with the cart speed (cart frame) of the test.
struct Rig {
  explicit Rig(const bool reversed = false) { cart.Configure(Parameters(reversed)); }

  /// Runs @p milliseconds; returns the output of the last step.
  CartOutput Run(const unsigned milliseconds, const AngularVelocity cart_speed = AngularVelocity{}) {
    CartOutput output{};
    for (unsigned i = 0U; i < milliseconds; ++i) {
      now += kMs;
      output = cart.Update(now, cart.ToCart(cart_speed), true);
    }
    return output;
  }

  /// Runs @p milliseconds, with @p feed called before every step.
  template <typename Feed>
  CartOutput RunWith(const unsigned milliseconds, const Feed& feed, const AngularVelocity cart_speed) {
    CartOutput output{};
    for (unsigned i = 0U; i < milliseconds; ++i) {
      feed();
      output = Run(1U, cart_speed);
    }
    return output;
  }

  CartControl cart{};
  uint64_t now{1'000'000U};
};

}  // namespace

ZTEST(cart_control, test_off_until_the_first_wheel_command_then_applied_unramped) {
  Rig rig;
  CartOutput output = rig.Run(5U);
  zexpect_equal(output.mode, CartMode::kOff);
  zexpect_false(output.holds_engagement);

  rig.cart.OnWheelCommand(12.0_Nm, rig.now);
  output = rig.Run(1U);
  zexpect_equal(output.mode, CartMode::kVdc);
  UNIMOC_EXPECT_FLOAT_EQ(output.torque, 12.0_Nm);
  zexpect_false(output.holds_engagement, "the readiness decides while the VDC is there");

  rig.cart.OnWheelCommand(Torque{std::numeric_limits<float>::quiet_NaN()}, rig.now);
  UNIMOC_EXPECT_FLOAT_EQ(rig.Run(1U).torque, 0.0_Nm, "NaN counts as zero");
}

ZTEST(cart_control, test_a_reversed_wheel_turns_the_torque_and_the_speed) {
  Rig rig{true};
  rig.cart.OnWheelCommand(8.0_Nm, rig.now);
  UNIMOC_EXPECT_FLOAT_EQ(rig.Run(1U).torque, -8.0_Nm);
  UNIMOC_EXPECT_FLOAT_EQ(rig.cart.ToCart(-3.0_rad_per_s), 3.0_rad_per_s);
}

ZTEST(cart_control, test_disengaged_resets_to_off) {
  Rig rig;
  rig.cart.OnWheelCommand(5.0_Nm, rig.now);
  zexpect_equal(rig.Run(1U).mode, CartMode::kVdc);
  rig.now += kMs;
  const CartOutput kOutput = rig.cart.Update(rig.now, AngularVelocity{}, false);
  zexpect_equal(kOutput.mode, CartMode::kOff);
  UNIMOC_EXPECT_FLOAT_EQ(kOutput.torque, 0.0_Nm);
}

ZTEST(cart_control, test_limp_home_ramps_to_the_drive_pedal_and_holds_the_engagement) {
  Rig rig;
  rig.cart.OnWheelCommand(0.0_Nm, rig.now);
  zassert_equal(rig.Run(1U).mode, CartMode::kVdc);
  const auto kFeed = [&rig] { rig.cart.OnPedalState(Pedals(0.6_ratio, 0.0_ratio), rig.now); };

  // The wheel command, 1 ms old now, times out after 100 ms.
  CartOutput output = rig.RunWith(99U, kFeed, 5.0_rad_per_s);
  zexpect_equal(output.mode, CartMode::kVdc);
  output = rig.RunWith(2U, kFeed, 5.0_rad_per_s);
  zassert_equal(output.mode, CartMode::kLimpHome);
  zexpect_true(output.holds_engagement);
  // 0.6 * 5 N m = 3 N m, reached at 50 N m/s in 60 ms.
  zexpect_true(output.torque < 0.2_Nm, "ramped, not stepped: %f", static_cast<double>(output.torque.Value()));
  output = rig.RunWith(80U, kFeed, 5.0_rad_per_s);
  UNIMOC_EXPECT_NEAR(output.torque, 3.0_Nm, 1e-4_Nm);
}

ZTEST(cart_control, test_the_brake_pedal_wins_opposes_the_motion_and_fades_at_standstill) {
  Rig rig;
  rig.cart.OnWheelCommand(0.0_Nm, rig.now);
  zassert_equal(rig.Run(1U).mode, CartMode::kVdc);
  const auto kFeed = [&rig] { rig.cart.OnPedalState(Pedals(1.0_ratio, 0.8_ratio), rig.now); };
  rig.RunWith(102U, kFeed, 5.0_rad_per_s);

  // Forward: -0.8 * 5 N m, also with the drive pedal fully pressed.
  UNIMOC_EXPECT_NEAR(rig.RunWith(200U, kFeed, 5.0_rad_per_s).torque, -4.0_Nm, 1e-4_Nm);
  // Half the standstill speed: half the brake.
  UNIMOC_EXPECT_NEAR(rig.RunWith(200U, kFeed, 0.5_rad_per_s).torque, -2.0_Nm, 1e-4_Nm);
  // At standstill no torque, so the brake never drives backward.
  UNIMOC_EXPECT_NEAR(rig.RunWith(200U, kFeed, AngularVelocity{}).torque, 0.0_Nm, 1e-4_Nm);
  // Rolling backward, the brake pushes forward.
  UNIMOC_EXPECT_NEAR(rig.RunWith(200U, kFeed, -5.0_rad_per_s).torque, 4.0_Nm, 1e-4_Nm);
}

ZTEST(cart_control, test_implausible_and_failed_pedal_channels) {
  Rig rig;
  rig.cart.OnWheelCommand(0.0_Nm, rig.now);
  zassert_equal(rig.Run(1U).mode, CartMode::kVdc);

  // An implausible drive pedal counts as released.
  CartPedals pedals = Pedals(1.0_ratio, 0.0_ratio);
  pedals.drive_plausible = false;
  const auto kImplausible = [&] { rig.cart.OnPedalState(pedals, rig.now); };
  UNIMOC_EXPECT_NEAR(rig.RunWith(300U, kImplausible, 5.0_rad_per_s).torque, 0.0_Nm, 1e-4_Nm);

  // An implausible brake pedal brakes with the larger of its valid channels.
  pedals = Pedals(0.0_ratio, 0.0_ratio);
  pedals.brake = {0.4_ratio, kNan};
  pedals.brake_plausible = false;
  UNIMOC_EXPECT_NEAR(rig.RunWith(300U, kImplausible, 5.0_rad_per_s).torque, -2.0_Nm, 1e-4_Nm);

  // The drive pedal uses the lower channel; a failed channel counts as released.
  pedals = Pedals(0.0_ratio, 0.0_ratio);
  pedals.drive = {0.8_ratio, 0.4_ratio};
  UNIMOC_EXPECT_NEAR(rig.RunWith(300U, kImplausible, 5.0_rad_per_s).torque, 2.0_Nm, 1e-4_Nm);
  pedals.drive = {0.8_ratio, kNan};
  UNIMOC_EXPECT_NEAR(rig.RunWith(300U, kImplausible, 5.0_rad_per_s).torque, 0.0_Nm, 1e-4_Nm);
}

ZTEST(cart_control, test_without_pedals_the_torque_ramps_to_zero_then_the_engagement_is_released) {
  Rig rig;
  const auto kCommand = [&rig] { rig.cart.OnWheelCommand(5.0_Nm, rig.now); };
  zassert_equal(rig.RunWith(10U, kCommand, 5.0_rad_per_s).mode, CartMode::kVdc);

  // No pedals either: stopping, 5 N m at 50 N m/s take 100 ms.
  CartOutput output = rig.Run(102U, 5.0_rad_per_s);
  zassert_equal(output.mode, CartMode::kStopping);
  zexpect_true(output.holds_engagement);
  zexpect_true(output.torque > 4.5_Nm, "ramped: %f", static_cast<double>(output.torque.Value()));
  output = rig.Run(100U, 5.0_rad_per_s);
  UNIMOC_EXPECT_FLOAT_EQ(output.torque, 0.0_Nm);
  zexpect_false(output.holds_engagement);
}

ZTEST(cart_control, test_the_vdc_takes_over_again_only_at_standstill) {
  Rig rig;
  rig.cart.OnWheelCommand(0.0_Nm, rig.now);
  zassert_equal(rig.Run(1U).mode, CartMode::kVdc);
  const auto kPedals = [&rig] { rig.cart.OnPedalState(Pedals(0.0_ratio, 0.0_ratio), rig.now); };
  zassert_equal(rig.RunWith(102U, kPedals, 5.0_rad_per_s).mode, CartMode::kLimpHome);

  const auto kBoth = [&rig] {
    rig.cart.OnPedalState(Pedals(0.0_ratio, 0.0_ratio), rig.now);
    rig.cart.OnWheelCommand(3.0_Nm, rig.now);
  };
  zexpect_equal(rig.RunWith(50U, kBoth, 5.0_rad_per_s).mode, CartMode::kLimpHome);
  const CartOutput kOutput = rig.RunWith(1U, kBoth, 0.5_rad_per_s);
  zexpect_equal(kOutput.mode, CartMode::kVdc);
  UNIMOC_EXPECT_FLOAT_EQ(kOutput.torque, 3.0_Nm);
}

ZTEST(cart_control, test_a_lost_bus_stops_at_once) {
  Rig rig;
  const auto kBoth = [&rig] {
    rig.cart.OnPedalState(Pedals(0.0_ratio, 0.0_ratio), rig.now);
    rig.cart.OnWheelCommand(2.0_Nm, rig.now);
  };
  zassert_equal(rig.RunWith(10U, kBoth, 5.0_rad_per_s).mode, CartMode::kVdc);
  rig.cart.OnBusLost();
  const CartOutput kOutput = rig.Run(1U, 5.0_rad_per_s);
  zexpect_equal(kOutput.mode, CartMode::kStopping, "no limp-home on pedal states from before the bus-off");
  zexpect_true(kOutput.holds_engagement);
}

ZTEST(cart_control, test_battery_limits_of_the_bms) {
  Rig rig;
  zexpect_false(rig.cart.Battery(rig.now).valid, "no battery status: the settings apply");

  rig.cart.OnBatteryStatus(CartBattery{.discharge_limit = 40.0_A,
                                       .charge_limit = 10.0_A,
                                       .contactor = Contactor::kClosed},
                           rig.now);
  auto limits = rig.cart.Battery(rig.now);
  zassert_true(limits.valid);
  UNIMOC_EXPECT_FLOAT_EQ(limits.drive_current, 20.0_A, "half the battery for this drive");
  UNIMOC_EXPECT_FLOAT_EQ(limits.charge_current, 5.0_A);

  // An announced opening keeps the limits (the BMS ramps them); open or precharging gives none.
  rig.cart.OnBatteryStatus(CartBattery{.discharge_limit = 40.0_A,
                                       .charge_limit = 10.0_A,
                                       .contactor = Contactor::kOpening},
                           rig.now);
  UNIMOC_EXPECT_FLOAT_EQ(rig.cart.Battery(rig.now).charge_current, 5.0_A);
  rig.cart.OnBatteryStatus(CartBattery{.discharge_limit = 40.0_A,
                                       .charge_limit = 10.0_A,
                                       .contactor = Contactor::kOpen},
                           rig.now);
  limits = rig.cart.Battery(rig.now);
  zexpect_true(limits.valid);
  UNIMOC_EXPECT_FLOAT_EQ(limits.drive_current, 0.0_A);
  UNIMOC_EXPECT_FLOAT_EQ(limits.charge_current, 0.0_A);

  // Negative limits count as zero, NaN limits leave it to the settings.
  rig.cart.OnBatteryStatus(CartBattery{.discharge_limit = -1.0_A,
                                       .charge_limit = 10.0_A,
                                       .contactor = Contactor::kClosed},
                           rig.now);
  UNIMOC_EXPECT_FLOAT_EQ(rig.cart.Battery(rig.now).drive_current, 0.0_A);
  rig.cart.OnBatteryStatus(CartBattery{.discharge_limit = Current{std::numeric_limits<float>::quiet_NaN()},
                                       .charge_limit = 10.0_A,
                                       .contactor = Contactor::kClosed},
                           rig.now);
  zexpect_false(rig.cart.Battery(rig.now).valid);

  // A stale battery status, or a bus-off, returns to the settings.
  rig.cart.OnBatteryStatus(CartBattery{.discharge_limit = 40.0_A,
                                       .charge_limit = 10.0_A,
                                       .contactor = Contactor::kClosed},
                           rig.now);
  zexpect_true(rig.cart.Battery(rig.now + (100U * kMs)).valid);
  zexpect_false(rig.cart.Battery(rig.now + (101U * kMs)).valid);
  rig.cart.OnBusLost();
  zexpect_false(rig.cart.Battery(rig.now).valid);
}

ZTEST_SUITE(cart_control, nullptr, nullptr, nullptr, nullptr, nullptr);
