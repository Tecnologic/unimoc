/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file units_test.cpp
 *	@brief Tests of the dimension system and the machine-model unit operators.
 *
 *	SPDX-License-Identifier: GPL-3.0-or-later
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */

#include "units.hpp"
#include <zephyr/ztest.h>
#include <concepts>
#include <limits>
#include "unimoc_test.hpp"

using namespace unimoc::unit;

static_assert(std::same_as<decltype(AngularVelocity{} * MagneticFlux{}), Voltage>);
static_assert(std::same_as<decltype(MagneticFlux{} * AngularVelocity{}), Voltage>);
static_assert(std::same_as<decltype(MagneticFlux{} / Inductance{}), Current>);
static_assert(std::same_as<decltype(Resistance{} * Time{}), Inductance>);
static_assert(std::same_as<decltype(Time{} * Resistance{}), Inductance>);
static_assert(std::same_as<decltype(MagneticFlux{} * Current{}), Torque>);
static_assert(std::same_as<decltype(Current{} * MagneticFlux{}), Torque>);
static_assert(std::same_as<decltype(Torque{} / MagneticFlux{}), Current>);
static_assert(std::same_as<decltype(Torque{} / Inertia{}), AngularAcceleration>);
static_assert(std::same_as<decltype(Inertia{} * AngularAcceleration{}), Torque>);
static_assert(std::same_as<decltype(AngularAcceleration{} * Inertia{}), Torque>);
static_assert(std::same_as<decltype(TorquePerAngularVelocity{} * AngularVelocity{}), Torque>);
static_assert(std::same_as<decltype(AngularVelocity{} * TorquePerAngularVelocity{}), Torque>);

// Dimension exponents: products and quotients are derived by the compiler.
static_assert(std::same_as<decltype(Voltage{} * Current{}), Power>);
static_assert(std::same_as<decltype(Power{} / Voltage{}), Current>);
static_assert(std::same_as<decltype(Torque{} * AngularVelocity{}), Power>);
static_assert(std::same_as<decltype(Power{} / AngularVelocity{}), Torque>);
static_assert(std::same_as<decltype(Angle{} / Time{}), AngularVelocity>);
static_assert(std::same_as<decltype(AngularVelocity{} / Time{}), AngularAcceleration>);
static_assert(std::same_as<decltype(Voltage{} * Time{}), MagneticFlux>);
static_assert(std::same_as<decltype(Inductance{} * Current{}), MagneticFlux>);
static_assert(std::same_as<decltype(Voltage{} / Current{}), Resistance>);
static_assert(std::same_as<decltype(Inductance{} / Resistance{}), Time>);
static_assert(std::same_as<decltype(1.0F / Time{}), Frequency>);
static_assert(std::same_as<decltype(Angle{} * Frequency{}), AngularVelocity>);
static_assert(std::same_as<decltype(Current{} / Current{}), Ratio>);
static_assert(std::same_as<decltype(Current{} * Ratio{}), Current>);
// Gains get their type from the formula, with no operator written for them.
static_assert(std::same_as<decltype(Voltage{} / Current{}), VoltagePerCurrent>);
static_assert(std::same_as<decltype(VoltagePerCurrentTime{} * Current{} * Time{}), Voltage>);
static_assert(std::same_as<decltype(CurrentPerVoltageTime{} * Voltage{} * Time{}), Current>);
static_assert(std::same_as<decltype(AnglePerCurrent{} * Current{}), Angle>);
static_assert(std::same_as<decltype(AngularVelocityPerAngle{} * Angle{}), AngularVelocity>);
static_assert(std::same_as<decltype(MagneticFluxPerCurrentTime{} * Current{} * Time{}), MagneticFlux>);
static_assert(std::same_as<decltype(Torque{} / AngularVelocity{}), TorquePerAngularVelocity>);
// The radian is a dimension: rad/s and Hz are different types.
static_assert(!std::same_as<AngularVelocity, Frequency>);
static_assert(!std::same_as<Angle, Ratio>);
static_assert(std::same_as<Frequency, InverseTime>);
// Electromechanical coupling: the rules agree with each other.
static_assert(std::same_as<decltype(Voltage{} / AngularVelocity{}), MagneticFlux>);
static_assert(std::same_as<decltype(Torque{} / Current{}), MagneticFlux>);
static_assert(std::same_as<decltype(AngularVelocity{} * (Inductance{} * Current{})), Voltage>);
static_assert(std::same_as<decltype((AngularVelocity{} * MagneticFlux{}) * Current{}),
                           decltype(AngularVelocity{} * (MagneticFlux{} * Current{}))>);
// Only unambiguous coupling rules: R/L and V/Wb stay 1/s, they are no speed.
static_assert(std::same_as<decltype(Resistance{} / Inductance{}), InverseTime>);
static_assert(std::same_as<decltype(Voltage{} / MagneticFlux{}), InverseTime>);
static_assert(std::same_as<decltype((Voltage{} / MagneticFlux{}) * 1.0_rad), AngularVelocity>);
// Quantities of different dimensions neither add nor compare.
template <typename A, typename B>
concept Addable = requires(A lhs, B rhs) { lhs + rhs; };
template <typename A, typename B>
concept Comparable = requires(A lhs, B rhs) { lhs < rhs; };
static_assert(Addable<Current, Current>);
static_assert(!Addable<Current, Voltage>);
static_assert(!Addable<AngularVelocity, Frequency>);
static_assert(!Comparable<Torque, Power>);
// The dimension costs nothing: a unit has the layout of its float.
static_assert(sizeof(Current) == sizeof(float));
static_assert(sizeof(CurrentPerMagneticFluxTime) == sizeof(float));
static_assert(IsVoltage<decltype(Current{} * Resistance{})>::value);
static_assert(IsCurrent<decltype(1.0_mA)>::value);
static_assert(!IsCurrent<Voltage>::value);

ZTEST(units, test_derived_quantities_scale_with_the_periods) {
  // Converted to the base period on the assignment.
  const Current kCurrent = 2.0_V / 4.0_kOhm;
  const Power kPower = 48.0_V * 500.0_mA;
  const Ratio kShare = 250.0_mA / 1.0_A;
  const Frequency kSampleRate = 1.0F / 50.0_us;
  UNIMOC_EXPECT_FLOAT_EQ(kCurrent, 0.5e-3_A);
  UNIMOC_EXPECT_FLOAT_EQ(kPower, 24.0_W);
  UNIMOC_EXPECT_FLOAT_EQ(kShare, 0.25_ratio);
  UNIMOC_EXPECT_FLOAT_EQ(kSampleRate, 20000.0_Hz);
}

ZTEST(units, test_gains_follow_from_the_formula) {
  // A PI current controller: v = kp * i + ki * i * dt.
  const Time kDt = 10.0_ms;
  const Voltage kOutput = (2.0_V_per_A * 3.0_A) + (100.0_V_per_A_s * 3.0_A * kDt);
  UNIMOC_EXPECT_FLOAT_EQ(kOutput, 9.0_V);
  // Torque of a PMSM with the 3/2 of the amplitude-invariant Clarke transform.
  const Torque kTorque = 1.5F * (10.0_mWb * 20.0_A);
  UNIMOC_EXPECT_FLOAT_EQ(kTorque, 0.3_Nm);
  UNIMOC_EXPECT_FLOAT_EQ(kTorque * 100.0_rad_per_s, 30.0_W);
}

ZTEST(units, test_sqrt_halves_the_dimension) {
  static_assert(std::same_as<decltype(Sqrt(Current{} * Current{})), Current>);
  static_assert(std::same_as<decltype(Sqrt(Inductance{} / Inductance{})), Ratio>);
  UNIMOC_EXPECT_FLOAT_EQ(Sqrt((3.0_A * 3.0_A) + (4.0_A * 4.0_A)), 5.0_A);
  UNIMOC_EXPECT_NEAR(Sqrt(4.0_mA * 4.0_mA), 4.0e-3_A, 1.0e-9_A);
}

ZTEST(units, test_back_emf_is_speed_times_flux) {
  UNIMOC_EXPECT_FLOAT_EQ((100.0_rad_per_s * MagneticFlux{20.0_mWb}), 2.0_V);
  UNIMOC_EXPECT_FLOAT_EQ((0.02_Wb * 100.0_rad_per_s), 2.0_V);
}

ZTEST(units, test_flux_over_inductance_is_current) {
  UNIMOC_EXPECT_FLOAT_EQ((0.01_Wb / Inductance{0.2_mH}), 50.0_A);
}

ZTEST(units, test_resistance_times_time_is_inductance) {
  UNIMOC_EXPECT_FLOAT_EQ((0.5_Ohm * 2.0_s), 1.0_H);
  UNIMOC_EXPECT_FLOAT_EQ((2.0_s * 0.5_Ohm), 1.0_H);
}

ZTEST(units, test_flux_times_current_is_torque) {
  UNIMOC_EXPECT_FLOAT_EQ((0.01_Wb * 20.0_A), 0.2_Nm);
  UNIMOC_EXPECT_FLOAT_EQ((20.0_A * 0.01_Wb), 0.2_Nm);
}

ZTEST(units, test_torque_and_inertia_give_angular_acceleration) {
  UNIMOC_EXPECT_FLOAT_EQ((0.1_Nm / 1.0e-3_kg_m2), 100.0_rad_per_s2);
  UNIMOC_EXPECT_FLOAT_EQ((1.0e-3_kg_m2 * 100.0_rad_per_s2), 0.1_Nm);
  UNIMOC_EXPECT_FLOAT_EQ((100.0_rad_per_s2 * 1.0e-3_kg_m2), 0.1_Nm);
}

ZTEST(units, test_viscous_damping_times_speed_is_torque) {
  UNIMOC_EXPECT_FLOAT_EQ((1.0e-3_Nm_s_per_rad * 50.0_rad_per_s), 0.05_Nm);
  UNIMOC_EXPECT_FLOAT_EQ((50.0_rad_per_s * 2_Nm_s_per_rad), 100.0_Nm);
}

// Typed math: the result keeps the unit, so no Value() escape is needed.
static_assert(std::same_as<decltype(Abs(Current{})), Current>);
static_assert(std::same_as<decltype(Min(Voltage{}, Voltage{})), Voltage>);
static_assert(std::same_as<decltype(Hypot(Current{}, Current{})), Current>);
static_assert(Abs(-2.0_A) == 2.0_A);
static_assert(Max(-1.0_V, 3.0_V) == 3.0_V);
static_assert(Min(-1.0_V, 3.0_V) == -1.0_V);
// Clamp takes bounds of the same unit only.
template <typename U>
concept ClampsWithScalars = requires(U value) { value.Clamp(0.0F, 1.0F); };
static_assert(!ClampsWithScalars<Current>);

ZTEST(units, test_noise_variances_follow_from_the_formula) {
  // Angle noise of a speed-weighted measurement: R + k / omega^2.
  const AngleVariance kNoise = 1e-4_rad2 + (4.0_rad4_per_s2 / (2.0_rad_per_s * 2.0_rad_per_s));
  UNIMOC_EXPECT_FLOAT_EQ(kNoise, 1.0001_rad2);
  // Speed noise of an angle variance spread over a time: R / tau^2.
  const AngularVelocityVariance kSpeedNoise = 3e-2_rad2 / (0.1_s * 0.1_s);
  UNIMOC_EXPECT_FLOAT_EQ(kSpeedNoise, 3.0_rad_per_s * 1.0_rad_per_s);
  UNIMOC_EXPECT_FLOAT_EQ(Sqrt(0.25_rad2), 0.5_rad);
}

ZTEST(units, test_abs_min_max_keep_the_unit) {
  UNIMOC_EXPECT_FLOAT_EQ(Abs(-3.5_A), 3.5_A);
  UNIMOC_EXPECT_FLOAT_EQ(Abs(3.5_A), 3.5_A);
  UNIMOC_EXPECT_FLOAT_EQ(Min(1.0_A, -2.0_A), -2.0_A);
  UNIMOC_EXPECT_FLOAT_EQ(Max(1.0_A, -2.0_A), 1.0_A);
}

ZTEST(units, test_is_finite_rejects_nan_and_infinity) {
  zexpect_true(IsFinite(48.0_V));
  zexpect_false(IsFinite(Voltage{std::numeric_limits<float>::quiet_NaN()}));
  zexpect_false(IsFinite(Voltage{std::numeric_limits<float>::infinity()}));
}

ZTEST(units, test_torque_over_torque_constant_is_current) {
  UNIMOC_EXPECT_FLOAT_EQ((0.6_Nm / 0.03_Wb), 20.0_A);
}

ZTEST(units, test_hypot_is_the_vector_magnitude) {
  UNIMOC_EXPECT_FLOAT_EQ(Hypot(3.0_A, -4.0_A), 5.0_A);
  UNIMOC_EXPECT_FLOAT_EQ(Hypot(Current{}, Current{}), 0.0_A);
}

ZTEST(units, test_float_checks_take_values_of_one_unit) {
  UNIMOC_EXPECT_FLOAT_EQ(Hypot(3.0_A, -4.0_A), 5.0_A);
  UNIMOC_EXPECT_NEAR(48.0_V + 1e-4_V, 48.0_V, 1e-3_V);
  zexpect_false(::unimoc::test::NearAny(48.1_V, 48.0_V, 1e-3_V));
  zexpect_false(::unimoc::test::FloatEqAny(1.0_A, 1.001_A));
}

ZTEST_SUITE(units, nullptr, nullptr, nullptr, nullptr, nullptr);
