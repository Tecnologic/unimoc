/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file units_test.cpp
 *	@brief Tests of the machine-model unit operators.
 *
 *	SPDX-License-Identifier: GPL-3.0-or-later
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */

#include "units.hpp"
#include <zephyr/ztest.h>
#include <concepts>
#include "unimoc_test.hpp"

using namespace unimoc::unit;

static_assert(std::same_as<decltype(AngularVelocity{} * MagneticFlux{}), Voltage>);
static_assert(std::same_as<decltype(MagneticFlux{} * AngularVelocity{}), Voltage>);
static_assert(std::same_as<decltype(MagneticFlux{} / Inductance{}), Current>);
static_assert(std::same_as<decltype(Resistance{} * Time{}), Inductance>);
static_assert(std::same_as<decltype(Time{} * Resistance{}), Inductance>);
static_assert(std::same_as<decltype(MagneticFlux{} * Current{}), Torque>);
static_assert(std::same_as<decltype(Current{} * MagneticFlux{}), Torque>);
static_assert(std::same_as<decltype(Torque{} / Inertia{}), AngularAcceleration>);
static_assert(std::same_as<decltype(Inertia{} * AngularAcceleration{}), Torque>);
static_assert(std::same_as<decltype(AngularAcceleration{} * Inertia{}), Torque>);
static_assert(std::same_as<decltype(TorquePerAngularVelocity{} * AngularVelocity{}), Torque>);
static_assert(std::same_as<decltype(AngularVelocity{} * TorquePerAngularVelocity{}), Torque>);

ZTEST(units, test_back_emf_is_speed_times_flux) {
  UNIMOC_EXPECT_FLOAT_EQ((100.0_rad_per_s * MagneticFlux{20.0_mWb}).Value(), 2.0F);
  UNIMOC_EXPECT_FLOAT_EQ((MagneticFlux{0.02F} * 100.0_rad_per_s).Value(), 2.0F);
}

ZTEST(units, test_flux_over_inductance_is_current) {
  UNIMOC_EXPECT_FLOAT_EQ((MagneticFlux{0.01F} / Inductance{0.2_mH}).Value(), 50.0F);
}

ZTEST(units, test_resistance_times_time_is_inductance) {
  UNIMOC_EXPECT_FLOAT_EQ((0.5_Ohm * 2.0_s).Value(), 1.0F);
  UNIMOC_EXPECT_FLOAT_EQ((2.0_s * 0.5_Ohm).Value(), 1.0F);
}

ZTEST(units, test_flux_times_current_is_torque) {
  UNIMOC_EXPECT_FLOAT_EQ((MagneticFlux{0.01F} * 20.0_A).Value(), 0.2F);
  UNIMOC_EXPECT_FLOAT_EQ((20.0_A * MagneticFlux{0.01F}).Value(), 0.2F);
}

ZTEST(units, test_torque_and_inertia_give_angular_acceleration) {
  UNIMOC_EXPECT_FLOAT_EQ((0.1_Nm / 1.0e-3_kg_m2).Value(), 100.0F);
  UNIMOC_EXPECT_FLOAT_EQ((1.0e-3_kg_m2 * 100.0_rad_per_s2).Value(), 0.1F);
  UNIMOC_EXPECT_FLOAT_EQ((100.0_rad_per_s2 * 1.0e-3_kg_m2).Value(), 0.1F);
}

ZTEST(units, test_viscous_damping_times_speed_is_torque) {
  UNIMOC_EXPECT_FLOAT_EQ((1.0e-3_Nm_s_per_rad * 50.0_rad_per_s).Value(), 0.05F);
  UNIMOC_EXPECT_FLOAT_EQ((50.0_rad_per_s * 2_Nm_s_per_rad).Value(), 100.0F);
}

ZTEST_SUITE(units, nullptr, nullptr, nullptr, nullptr, nullptr);
