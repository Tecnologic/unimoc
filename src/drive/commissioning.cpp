/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file commissioning.cpp
 *    @brief Commissioning over Cyphal: the commands, the status message and the results of the startup aid.
 *
 *    SPDX-License-Identifier: GPL-3.0-or-later
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include "commissioning.hpp"
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <optional>
#include "cyphal/cyphal_interface.hpp"
#include "cyphal/register_value.hpp"
#include "units.hpp"

namespace unimoc::drive {

namespace {

using StatusMessage = unimoc_drive_CommissioningStatus_0_1;
using startup::FsmState;
using startup::StartupContinue;
using startup::StartupFault;
using startup::StartupNeeds;
using startup::StartupStart;

constexpr std::string_view kMotorConnected = "motor-connected";

/// Largest meter reading taken [V or A]: a plausibility bound, not a hardware limit.
constexpr float kMaxReading = 1000.0F;

std::string_view Text(const std::span<const uint8_t> parameter) noexcept {
  return {reinterpret_cast<const char*>(parameter.data()), parameter.size()};
}

/// A positive finite number in base units, the whole text.
std::optional<float> Number(const std::string_view text) noexcept {
  std::array<char, 24> buffer{};
  if (text.empty() || text.size() >= buffer.size()) {
    return std::nullopt;
  }
  std::memcpy(buffer.data(), text.data(), text.size());
  char* end = nullptr;
  const float kValue = std::strtof(buffer.data(), &end);
  if (end != buffer.data() + text.size() || !std::isfinite(kValue) || !(kValue > 0.0F) ||
      kValue > kMaxReading) {
    return std::nullopt;
  }
  return kValue;
}

uint8_t StateOf(const startup::StartupStatus& status) noexcept {
  if (status.active) {
    switch (status.needs) {
      case StartupNeeds::kDcLinkVoltage:
      case StartupNeeds::kPhaseCurrent:
        return unimoc_drive_CommissioningStatus_0_1_STATE_NEEDS_READING;
      default:
        return status.measured ? unimoc_drive_CommissioningStatus_0_1_STATE_WAITING
                               : unimoc_drive_CommissioningStatus_0_1_STATE_MEASURING;
    }
  }
  switch (status.step) {
    case FsmState::kDone:
      return unimoc_drive_CommissioningStatus_0_1_STATE_DONE;
    case FsmState::kFault:
      return (status.fault == StartupFault::kAborted || status.fault == StartupFault::kEmergencyStop)
                 ? unimoc_drive_CommissioningStatus_0_1_STATE_ABORTED
                 : unimoc_drive_CommissioningStatus_0_1_STATE_FAILED;
    default:
      return unimoc_drive_CommissioningStatus_0_1_STATE_IDLE;
  }
}

uint8_t NeedsOf(const StartupNeeds needs) noexcept {
  switch (needs) {
    case StartupNeeds::kDcLinkVoltage:
      return unimoc_drive_CommissioningStatus_0_1_NEEDS_DC_LINK_VOLTAGE;
    case StartupNeeds::kPhaseCurrent:
      return unimoc_drive_CommissioningStatus_0_1_NEEDS_PHASE_CURRENT;
    case StartupNeeds::kMotorConnected:
      return unimoc_drive_CommissioningStatus_0_1_NEEDS_MOTOR_CONNECTED;
    default:
      return unimoc_drive_CommissioningStatus_0_1_NEEDS_NONE;
  }
}

bool Same(const StatusMessage& a, const StatusMessage& b) noexcept {
  return a.procedure == b.procedure && a.step == b.step && a.state == b.state && a.needs == b.needs &&
         a.progress == b.progress && a.passed == b.passed && a.failed == b.failed && a.fault == b.fault &&
         a.counter == b.counter;
}

constexpr float kInf = std::numeric_limits<float>::infinity();

constexpr cyphal::RegisterDescription
    kResultsDescription{{},
                        -kInf,
                        kInf,
                        cyphal::RegisterEffect::kImmediate,
                        "Startup aid result per step: 1 passed, 0 failed or not run"};

/// The result registers of the identification, after unimoc.startup.results.
struct IdentifyRegister {
  const char* name{nullptr};
  cyphal::RegisterDescription description;
};
constexpr std::array<IdentifyRegister, 7> kIdentifyRegisters{{
    {cyphal::reg::kIdentifyResultR,
     {"Ohm",
      -kInf,
      kInf,
      cyphal::RegisterEffect::kImmediate,
      "Measured stator resistance (M1); NaN: not measured"}},
    {cyphal::reg::kIdentifyResultDeadTime,
     {"s",
      -kInf,
      kInf,
      cyphal::RegisterEffect::kImmediate,
      "Measured dead time for unimoc.control.dtc.dead_time (M1); NaN: not measured"}},
    {cyphal::reg::kIdentifyResultDeadTimeVoltage,
     {"V", -kInf, kInf, cyphal::RegisterEffect::kImmediate, "Dead-time voltage of the M1 regression"}},
    {cyphal::reg::kIdentifyResultLd,
     {"H",
      -kInf,
      kInf,
      cyphal::RegisterEffect::kImmediate,
      "Measured d-axis inductance (M2); NaN: not measured"}},
    {cyphal::reg::kIdentifyResultLq,
     {"H",
      -kInf,
      kInf,
      cyphal::RegisterEffect::kImmediate,
      "Measured q-axis inductance (M2); NaN: not measured"}},
    {cyphal::reg::kIdentifyResultL,
     {"H",
      -kInf,
      kInf,
      cyphal::RegisterEffect::kImmediate,
      "Mean of L_d and L_q (M2), for unimoc.motor.stator.L"}},
    {cyphal::reg::kIdentifyResultRAc,
     {"Ohm",
      -kInf,
      kInf,
      cyphal::RegisterEffect::kImmediate,
      "Resistance from the M2 impedance, a check of R (within 20 %)"}},
}};

uint8_t IdentificationStateOf(const identification::Status& status) noexcept {
  if (status.active) {
    return unimoc_drive_CommissioningStatus_0_1_STATE_MEASURING;
  }
  switch (status.step) {
    case identification::Step::kDone:
      return unimoc_drive_CommissioningStatus_0_1_STATE_DONE;
    case identification::Step::kFault:
      return (status.fault == identification::Fault::kAborted ||
              status.fault == identification::Fault::kEmergencyStop)
                 ? unimoc_drive_CommissioningStatus_0_1_STATE_ABORTED
                 : unimoc_drive_CommissioningStatus_0_1_STATE_FAILED;
    default:
      return unimoc_drive_CommissioningStatus_0_1_STATE_IDLE;
  }
}

/// The measurements of a START_IDENTIFICATION parameter: "M1", "M2", "M1,M2"; empty: all.
std::optional<uint8_t> Measurements(std::string_view text) noexcept {
  if (text.empty()) {
    return identification::kMeasureResistance | identification::kMeasureInductance;
  }
  uint8_t measurements = 0U;
  while (!text.empty()) {
    const std::size_t kComma = text.find(',');
    const std::string_view kToken = text.substr(0U, kComma);
    if (kToken == "M1") {
      measurements |= identification::kMeasureResistance;
    } else if (kToken == "M2") {
      measurements |= identification::kMeasureInductance;
    } else {
      return std::nullopt;
    }
    text = (kComma == std::string_view::npos) ? std::string_view{} : text.substr(kComma + 1U);
  }
  return measurements;
}

}  // namespace

bool Commissioning::Handles(const uint16_t command) noexcept {
  return command >= unimoc_drive_CommissioningStatus_0_1_COMMAND_START_STARTUP_AID &&
         command <= unimoc_drive_CommissioningStatus_0_1_COMMAND_APPLY_IDENTIFICATION;
}

cyphal::CommandResult Commissioning::OnCommand(const uint16_t command,
                                               const std::span<const uint8_t> parameter) noexcept {
  switch (command) {
    case unimoc_drive_CommissioningStatus_0_1_COMMAND_START_STARTUP_AID:
      return Start();
    case unimoc_drive_CommissioningStatus_0_1_COMMAND_START_IDENTIFICATION:
      return StartIdentification(Text(parameter));
    case unimoc_drive_CommissioningStatus_0_1_COMMAND_CONTINUE:
      if (identification_->IsActive()) {
        return {cyphal::cmd::kStatusBadState, "the identification takes no confirmation"};
      }
      return Continue(Text(parameter));
    case unimoc_drive_CommissioningStatus_0_1_COMMAND_SET_READING:
      return SetReading(Text(parameter));
    case unimoc_drive_CommissioningStatus_0_1_COMMAND_ABORT:
      fsm_->RequestAbort();  // no change outside a run
      identification_->Abort();
      return {cyphal::cmd::kStatusSuccess, ""};
    default:  // APPLY_IDENTIFICATION: milestone I6
      return {cyphal::cmd::kStatusBadCommand, "not implemented"};
  }
}

cyphal::CommandResult Commissioning::Start() noexcept {
  if (*engaged_) {
    return {cyphal::cmd::kStatusBadState, "engaged: disengage first"};
  }
  if (*emergency_stop_) {
    return {cyphal::cmd::kStatusBadState, "emergency stop latched: restart first"};
  }
  if (identification_->IsActive()) {
    return {cyphal::cmd::kStatusBadState, "the identification runs"};
  }
  switch (fsm_->RequestRun()) {
    case StartupStart::kStarted:
      identification_last_ = false;
      return {cyphal::cmd::kStatusSuccess, "started: phase 1 without the motor"};
    case StartupStart::kRunning:
      return {cyphal::cmd::kStatusBadState, "the startup aid runs already"};
    case StartupStart::kNotReleased:
      return {cyphal::cmd::kStatusFailure, "outputs not released: external enable missing"};
  }
  return {cyphal::cmd::kStatusInternalError, ""};
}

cyphal::CommandResult Commissioning::StartIdentification(const std::string_view measurements) noexcept {
  const std::optional<uint8_t> kMeasurements = Measurements(measurements);
  if (!kMeasurements.has_value()) {
    return {cyphal::cmd::kStatusBadParameter, "expected M1, M2 or M1,M2"};
  }
  if (*engaged_) {
    return {cyphal::cmd::kStatusBadState, "engaged: disengage first"};
  }
  if (*emergency_stop_) {
    return {cyphal::cmd::kStatusBadState, "emergency stop latched: restart first"};
  }
  if (fsm_->IsActive()) {
    return {cyphal::cmd::kStatusBadState, "the startup aid runs"};
  }
  switch (identification_->Start(*kMeasurements)) {
    case identification::StartResult::kStarted:
      identification_last_ = true;
      return {cyphal::cmd::kStatusSuccess, "started: the motor may jerk once while it aligns"};
    case identification::StartResult::kRunning:
      return {cyphal::cmd::kStatusBadState, "the identification runs already"};
    case identification::StartResult::kNotReleased:
      return {cyphal::cmd::kStatusFailure, "outputs not released: external enable missing"};
    case identification::StartResult::kNoMeasurement:
      return {cyphal::cmd::kStatusBadParameter, "expected M1, M2 or M1,M2"};
  }
  return {cyphal::cmd::kStatusInternalError, ""};
}

StatusMessage Commissioning::IdentificationStatus() const noexcept {
  const identification::Status kStatus = identification_->GetStatus();
  StatusMessage status{};
  status.procedure = unimoc_drive_CommissioningStatus_0_1_PROCEDURE_IDENTIFICATION;
  status.step = static_cast<uint8_t>(kStatus.step);
  status.state = IdentificationStateOf(kStatus);
  status.needs = unimoc_drive_CommissioningStatus_0_1_NEEDS_NONE;
  status.progress = kStatus.progress;
  status.passed = kStatus.passed;
  status.failed = kStatus.failed;
  status.fault = static_cast<uint8_t>(kStatus.fault);
  status.counter = kStatus.counter;
  return status;
}

cyphal::CommandResult Commissioning::Continue(const std::string_view confirmation) noexcept {
  const bool kMotorConfirmed = confirmation == kMotorConnected;
  if (!confirmation.empty() && !kMotorConfirmed) {
    return {cyphal::cmd::kStatusBadParameter, "unknown confirmation"};
  }
  switch (fsm_->RequestNextStep(kMotorConfirmed)) {
    case StartupContinue::kAccepted:
      return {cyphal::cmd::kStatusSuccess, ""};
    case StartupContinue::kNotRunning:
      return {cyphal::cmd::kStatusBadState, "the startup aid is not running"};
    case StartupContinue::kMeasuring:
      return {cyphal::cmd::kStatusBadState, "the step is still measuring"};
    case StartupContinue::kNeedsReading:
      return {cyphal::cmd::kStatusBadState, "the step needs its meter reading"};
    case StartupContinue::kNeedsConfirmation:
      return {cyphal::cmd::kStatusBadParameter, "confirm motor-connected"};
    case StartupContinue::kUnexpectedConfirmation:
      return {cyphal::cmd::kStatusBadParameter, "this step takes no confirmation"};
  }
  return {cyphal::cmd::kStatusInternalError, ""};
}

cyphal::CommandResult Commissioning::SetReading(const std::string_view reading) noexcept {
  if (!fsm_->IsActive()) {
    return {cyphal::cmd::kStatusBadState, "the startup aid is not running"};
  }
  const std::size_t kSeparator = reading.find('=');
  if (kSeparator == std::string_view::npos) {
    return {cyphal::cmd::kStatusBadParameter, "expected vdc=<V> or current=<A>"};
  }
  const std::string_view kName = reading.substr(0U, kSeparator);
  const std::optional<float> kValue = Number(reading.substr(kSeparator + 1U));
  if (!kValue.has_value()) {
    return {cyphal::cmd::kStatusBadParameter, "not a positive number"};
  }
  // The register boundary of the readings: a number from the bus in base units.
  if (kName == "vdc") {
    fsm_->SetExtVdc(unit::Voltage{*kValue});
    return {cyphal::cmd::kStatusSuccess, ""};
  }
  if (kName == "current") {
    fsm_->SetExtCurrent(unit::Current{*kValue});
    return {cyphal::cmd::kStatusSuccess, ""};
  }
  return {cyphal::cmd::kStatusBadParameter, "expected vdc=<V> or current=<A>"};
}

StatusMessage Commissioning::Status() const noexcept {
  if (identification_last_) {
    return IdentificationStatus();
  }
  const startup::StartupStatus kStartup = fsm_->Status();
  StatusMessage status{};
  status.procedure = kStartup.started ? unimoc_drive_CommissioningStatus_0_1_PROCEDURE_STARTUP_AID
                                      : unimoc_drive_CommissioningStatus_0_1_PROCEDURE_NONE;
  status.step = static_cast<uint8_t>(kStartup.step);
  status.state = StateOf(kStartup);
  status.needs = NeedsOf(kStartup.needs);
  status.progress = kStartup.progress;
  status.passed = kStartup.passed;
  status.failed = kStartup.failed;
  status.fault = static_cast<uint8_t>(kStartup.fault);
  return status;
}

bool Commissioning::Due(const uint64_t now_us, StatusMessage& status) noexcept {
  status = Status();
  const bool kChanged = !published_ || !Same(status, last_);
  if (!kChanged && now_us < next_us_) {
    return false;
  }
  const bool kActive = Active();
  next_us_ = now_us + (kActive ? kActivePeriodUs : kIdlePeriodUs);
  last_ = status;
  published_ = true;
  return true;
}

std::size_t Commissioning::Count() const noexcept { return 1U + kIdentifyRegisters.size(); }

std::string_view Commissioning::Name(const std::size_t index) const noexcept {
  if (index == 0U) {
    return cyphal::reg::kStartupResults;
  }
  return index <= kIdentifyRegisters.size() ? std::string_view{kIdentifyRegisters[index - 1U].name}
                                            : std::string_view{};
}

cyphal::RegisterDescription Commissioning::Describe(const std::size_t index) const noexcept {
  if (index == 0U) {
    return kResultsDescription;
  }
  return index <= kIdentifyRegisters.size() ? kIdentifyRegisters[index - 1U].description
                                            : cyphal::RegisterDescription{};
}

cyphal::RegisterFlags Commissioning::Read(const std::size_t index,
                                          cyphal::RegisterValue& value) const noexcept {
  if (index > kIdentifyRegisters.size()) {
    cyphal::SetEmpty(value);
    return {false, false};
  }
  if (index > 0U) {
    // The register boundary of the results: real32 in base units, NaN until measured.
    const identification::Results& results = identification_->GetResults();
    const float kNan = std::numeric_limits<float>::quiet_NaN();
    const bool kResistance = results.resistance_valid;
    const bool kInductance = results.inductance_valid;
    const std::array<float, kIdentifyRegisters.size()> kValues{
        kResistance ? results.resistance.Value() : kNan,
        kResistance ? results.dead_time.Value() : kNan,
        kResistance ? results.dead_time_voltage.Value() : kNan,
        kInductance ? results.l_d.Value() : kNan,
        kInductance ? results.l_q.Value() : kNan,
        kInductance ? results.l_mean.Value() : kNan,
        kInductance ? results.resistance_ac.Value() : kNan,
    };
    cyphal::SetReal32(value, kValues[index - 1U]);
    return {false, false};
  }
  uavcan_register_Value_1_0_select_natural8_(&value);
  const auto& passed = fsm_->results.passed;
  for (std::size_t step = 0U; step < passed.size(); ++step) {
    value.natural8.value.elements[step] = passed[step] ? 1U : 0U;
  }
  value.natural8.value.count = passed.size();
  return {false, false};
}

cyphal::RegisterWriteResult Commissioning::Write(const std::size_t /*index*/,
                                                 const cyphal::RegisterValue& /*value*/) noexcept {
  return cyphal::RegisterWriteResult::kReadOnly;
}

}  // namespace unimoc::drive
