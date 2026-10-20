/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file sim_control.hpp
 *    @brief Inputs of the native_sim hardware layer that a test or shell can set.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <cstdint>
#include "motor_simulation.hpp"
#include "three_phase_system.hpp"
#include "units.hpp"

namespace unimoc::hardware::sim {

/// External enable (jumper, e-stop). Dropping it while enabled breaks like the hardware does.
void SetExternalEnable(bool present) noexcept;

/**
 * @brief Replaces the measured phase currents [A] (sensor fault injection).
 *
 * The plant keeps running; only what the simulated current sensors report changes.
 */
void SetPhaseCurrentOverride(float a, float b, float c) noexcept;

/// Reports the plant currents again.
void ClearPhaseCurrentOverride() noexcept;

/**
 * @brief The simulated drive (inverter, motor, shaft, Hall sensors).
 *
 * Initialize() configures it from the factory settings when it is not yet
 * configured. Change it only while the cycle timer cannot run (between
 * StepCycle() calls, with SetFreeRunning(false), or under irq_lock()).
 */
::unimoc::sim::MotorSimulation& Plant() noexcept;

/// Starts (default after Initialize()) or stops the free-running cycle timer.
void SetFreeRunning(bool running) noexcept;

/**
 * @brief Runs one control cycle synchronously.
 *
 * Same order as the timer: ControlCycleHandler() and ObserverHandler() on the
 * samples of the previous cycle, then the plant over the next
 * kHalfPeriodsPerCycle half periods. Use with SetFreeRunning(false).
 */
void StepCycle() noexcept;

/// Duties the plant applied in the half periods of the last cycle, in time order.
[[nodiscard]] std::array<system::ThreePhase<unit::Ratio>, 4> AppliedDuties() noexcept;

/// Control cycles run since Initialize().
[[nodiscard]] uint32_t CycleCount() noexcept;

}  // namespace unimoc::hardware::sim
