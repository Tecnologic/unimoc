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

constexpr cyphal::RegisterDescription
    kResultsDescription{{},
                        -std::numeric_limits<float>::infinity(),
                        std::numeric_limits<float>::infinity(),
                        cyphal::RegisterEffect::kImmediate,
                        "Startup aid result per step: 1 passed, 0 failed or not run"};

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
    case unimoc_drive_CommissioningStatus_0_1_COMMAND_CONTINUE:
      return Continue(Text(parameter));
    case unimoc_drive_CommissioningStatus_0_1_COMMAND_SET_READING:
      return SetReading(Text(parameter));
    case unimoc_drive_CommissioningStatus_0_1_COMMAND_ABORT:
      fsm_->RequestAbort();  // no change outside a run
      return {cyphal::cmd::kStatusSuccess, ""};
    default:  // the motor identification is not there yet
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
  switch (fsm_->RequestRun()) {
    case StartupStart::kStarted:
      return {cyphal::cmd::kStatusSuccess, "started: phase 1 without the motor"};
    case StartupStart::kRunning:
      return {cyphal::cmd::kStatusBadState, "the startup aid runs already"};
    case StartupStart::kNotReleased:
      return {cyphal::cmd::kStatusFailure, "outputs not released: external enable missing"};
  }
  return {cyphal::cmd::kStatusInternalError, ""};
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
  const bool kActive = fsm_->IsActive();
  next_us_ = now_us + (kActive ? kActivePeriodUs : kIdlePeriodUs);
  last_ = status;
  published_ = true;
  return true;
}

std::size_t Commissioning::Count() const noexcept { return 1U; }

std::string_view Commissioning::Name(const std::size_t index) const noexcept {
  return index == 0U ? std::string_view{cyphal::reg::kStartupResults} : std::string_view{};
}

cyphal::RegisterDescription Commissioning::Describe(const std::size_t index) const noexcept {
  return index == 0U ? kResultsDescription : cyphal::RegisterDescription{};
}

cyphal::RegisterFlags Commissioning::Read(const std::size_t index,
                                          cyphal::RegisterValue& value) const noexcept {
  if (index != 0U) {
    cyphal::SetEmpty(value);
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
