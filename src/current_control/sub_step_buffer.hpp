/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file sub_step_buffer.hpp
 *    @brief Angle reference and cycle record shared by the control steps and the observer.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include "control_timing.hpp"
#include "rotor_system.hpp"
#include "sin_cos.hpp"
#include "stator_system.hpp"

/**
 * @namespace unimoc global namespace
 */

/**
 * @namespace current_control current-control subsystem namespace
 */
namespace unimoc::current_control {

/// Most control steps of one control cycle; the running count is
/// settings::ControlTiming::steps_per_cycle (the observer runs once per cycle).
inline constexpr uint8_t kMaxSubSteps = settings::kMaxStepsPerCycle;

/**
 * @brief Angle the observer hands to the control steps.
 *
 * The control step of every sample rotates its angle by `step`, so it needs
 * the observer only to correct the estimate: one reference per control cycle.
 * A late observer means a longer extrapolation at the last speed.
 */
struct AngleReference {
  /// Sample counter (CurrentControlState::samples) of the sample `sc` belongs to.
  uint32_t sample{0U};

  /// Park angle at that sample: the angle its measured currents show (the
  /// estimate less the rotation over the current filter delay).
  system::SinCos<unit::Ratio> sc;

  /// Rotation over one control step (omega x dt_fast).
  system::SinCos<unit::Ratio> step;

  /// Rotation from the Park angle to the middle of the interval the duties
  /// apply to (omega x (output delay + current filter delay), docs/hotpath_plan.md D4).
  system::SinCos<unit::Ratio> out;

  /// Electrical speed of the reference: the observer's estimate, or the
  /// speed of the identification's forced angle. The decoupling feedforward
  /// of the current PI works in this frame, so it takes this speed.
  unit::AngularVelocity speed;
};

/**
 * @brief Double-buffered angle reference with an atomic active index.
 *
 * The observer writes `buf[1 - active]` and then flips `active` (release);
 * the control step reads `buf[active]` (acquire). The control step preempts the
 * observer, so it never sees a half-written reference.
 */
struct AngleBuffer {
  /// The two ping-pong references.
  std::array<AngleReference, 2> buf{};

  /// Index of the reference the control steps use (0 or 1).
  std::atomic<uint8_t> active{0U};
};

/**
 * @brief What the control steps of one control cycle hand to the observer.
 *
 * Index k is the k-th sample (control step) of the cycle; the first
 * settings::ControlTiming::steps_per_cycle entries are filled.
 *
 * Ownership protocol
 * ------------------
 *  - The control steps fill one record of a pair, sample by sample. The last
 *    step of the cycle hands it to the observer (`observer_pending`) and the
 *    next cycle fills the other one.
 *  - The observer reads the handed-over record. It must finish before the
 *    control steps return to the record one cycle later.
 */
struct CycleRecord {
  /// Park angle of each sample (the estimate the control step used).
  std::array<system::SinCos<unit::Ratio>, kMaxSubSteps> sc{};

  /// Stator-frame (α/β) current samples of the cycle.
  std::array<system::Stator<unit::Current>, kMaxSubSteps> i_ab_samples{};

  /// Stator-frame (α/β) phase voltages of the cycle: Clarke of the measured
  /// terminal voltages (the common mode drops out); the back-EMF while the
  /// outputs are open.
  std::array<system::Stator<unit::Voltage>, kMaxSubSteps> u_ab_samples{};

  /// The platform measures the phase voltages (u_ab_samples are valid).
  bool phase_voltages_valid{false};

  /// Mean rotor-frame current of the cycle.
  system::Rotor<unit::Current> i_dq_mean;

  /// Mean voltage demand of the cycle, rotor frame.
  system::Rotor<unit::Voltage> u_dq;

  /// Timestamp of the last sample of the cycle; the clock of the Hall events
  /// (hardware::kTimestampFrequency).
  uint32_t timestamp{0U};

  /// Sample counter of the first sample of the cycle (the phase of the identification's sine).
  uint32_t first_sample{0U};

  /// Calibrated DC-link voltage of the last sample.
  unit::Voltage dc_link;

  /// Amplitude of the identification's sine in the duties of the cycle; zero: none.
  unit::Voltage injection_amplitude;

  /// Axis of that sine: 0 d, 1 q.
  uint8_t injection_axis{0U};
};

}  // namespace unimoc::current_control
