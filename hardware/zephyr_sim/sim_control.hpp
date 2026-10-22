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
 *    SPDX-License-Identifier: GPL-3.0-or-later
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
 * @brief Replaces the Hall code the simulated sensors report (sensor fault injection).
 *
 * The change is captured like an edge at the end of the next half period;
 * 0 or 7 is a broken or unpowered sensor set.
 */
void SetHallCodeOverride(uint8_t code) noexcept;

/// Reports the Hall code of the plant again.
void ClearHallCodeOverride() noexcept;

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

/// State of the simulated platform after a number of control cycles.
struct Snapshot {
  uint32_t cycle_count{};                                  ///< Control cycles since Initialize() or Reset().
  uint64_t virtual_time_ns{};                              ///< Virtual time of the cycle count [ns].
  ::unimoc::sim::SimulationOutput plant;                   ///< Plant output at the end of the last cycle.
  std::array<system::ThreePhase<unit::Ratio>, 4> applied;  ///< Duties of the last cycle, in time order.
};

/**
 * @brief Returns the platform to the state right after Initialize().
 *
 * The plant goes back to standstill (configuration, shaft mode and load are
 * kept), the cycle counter, the virtual time and the duty plan restart, the
 * current override is cleared and the power stage is disabled. The controller
 * is not part of the platform: restart it with StartRuntime(). Use with
 * SetFreeRunning(false).
 */
void Reset() noexcept;

/// Captures the platform state; consistent even while the cycle timer runs.
[[nodiscard]] Snapshot TakeSnapshot() noexcept;

/// Runs @p count control cycles synchronously (see StepCycle()).
void AdvanceCycles(uint32_t count) noexcept;

/// Virtual time since Initialize(): CycleCount() control periods, exact in nanoseconds.
[[nodiscard]] uint64_t VirtualTimeNs() noexcept;

/// Duties the plant applied in the half periods of the last cycle, in time order.
[[nodiscard]] std::array<system::ThreePhase<unit::Ratio>, 4> AppliedDuties() noexcept;

/// Control cycles run since Initialize().
[[nodiscard]] uint32_t CycleCount() noexcept;

/// Plant truth as plain floats for the traces (SI units); updated at the end of every cycle.
struct PlantTrace {
  float i_a{0.0F};               ///< Phase current a [A].
  float i_b{0.0F};               ///< Phase current b [A].
  float i_c{0.0F};               ///< Phase current c [A].
  float i_d{0.0F};               ///< d current in the true rotor frame [A].
  float i_q{0.0F};               ///< q current in the true rotor frame [A].
  float electrical_angle{0.0F};  ///< True electrical angle, [-pi, pi) [rad].
  float electrical_speed{0.0F};  ///< True electrical speed [rad/s].
  float mechanical_speed{0.0F};  ///< Shaft speed [rad/s].
  float torque{0.0F};            ///< Electromagnetic torque [N m].
};

/// The plant truth the control cycle of the next cycle sees; read by the trace device only.
[[nodiscard]] const PlantTrace& PlantTraceSignals() noexcept;

}  // namespace unimoc::hardware::sim
