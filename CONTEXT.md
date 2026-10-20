# UNIMOC Project Glossary

Document for consistent language in code, documentation, reviews,
and agent instructions. Use the preferred terms below unless a protocol or
external API requires different wording.

## Project Terms

| Preferred term | Meaning and usage |
|---|---|
| **UNIMOC** | The complete project: portable motor-control library plus firmware and hardware integrations. |
| **drive** | One physical motor-control device, including its power electronics, controller, firmware, and motor interface. |
| **controller** | A software control algorithm or, when the context is explicit, the drive electronics. Prefer **drive** for the complete device. |
| **motor** | The electromechanical machine being controlled. Do not use **motor** for the drive electronics. |
| **portable library** | The HAL-free code in `lib/`, intended to work on the host and on embedded targets. |
| **firmware** | The Zephyr application in `app/` and the code it builds from `src/`, `hardware/`, and `boards/`. |
| **Zephyr** | The RTOS that runs the firmware. It is the primary OS for everything outside the hot path. |
| **twister scenario** | One test application under `tests/` with a `testcase.yaml`, built and run by Zephyr's twister (ztest on `native_sim`, build-only on `bcc`). |
| **SCA** | Zephyr static code analysis; here clang-tidy through CodeChecker (`scripts/twister_sca.sh`). |
| **native_sim** | Zephyr's Linux simulation target. It runs the firmware application against the PMSM plant on the host. |
| **hardware layer** | Platform-specific code that connects the portable library to ADCs, PWM, timers, NVM, gate drivers, and other peripherals. Its platform is `zephyr_bcc` or `zephyr_sim`. |
| **BCC** | The battery case controller: the STM32G473 board, built with `-b bcc`. |
| **plant** | The simulated motor with its inverter, shaft and position sensor (`MotorSimulation` in `lib/sim/`). The firmware controls it on `native_sim`, including the ztest scenarios in `tests/sim/`. |
| **power stage** | The inverter output stage behind `PowerStageInterface`: enable, disable, trip, and a latched fault. |

## Runtime Terms

| Preferred term | Meaning and usage |
|---|---|
| **hot path** | The FOC work that runs in zero-latency IRQs: the sample ISR and the control cycle. Only this runs at zero latency. |
| **zero-latency IRQ** | An interrupt that bypasses the Zephyr scheduler and kernel locking (`IRQ_ZERO_LATENCY`). It may not call kernel APIs, log, or lock. |
| **sample ISR** | The end of the injected ADC sequence (JEOS), once per half PWM period. It copies the raw currents and checks the overcurrent limit. |
| **control cycle** | Four half PWM periods (16 kHz at 32 kHz PWM). It runs inside every fourth sample ISR: PI once, four duty entries for the next four half periods. `CurrentControlIsr::OnControlCycle()` in code. |
| **duty plan** | The four duty entries of one control cycle (`DutyPlan`). Entry 0 goes into the CCR preload, entries 1 to 3 are delivered by DMA at the next update events. |
| **hot-path interface** | `hardware/hot_path_interface.hpp`: the hardware access of the control cycle (`hot_path::GetControlCycleSamples`, `SetDutyPlan`, `TripPowerStage`) and the two handlers the platform calls. Bound at link time per platform, no function pointers. |
| **observer** | The flux observer, Kalman filter and HFI after each control cycle, in a software-pended normal-priority interrupt. It computes the angles of the next cycle. `SlowUpdate` in code. |
| **Zephyr thread** | Any work outside the zero-latency IRQs: logging, shell, USB, Cyphal, settings, and startup sequencing. |
| **trip** | An overcurrent or fault stop that switches the power stage off. A trip latches a fault until it is cleared. |
| **dead time** | The gap between the switching edges of a half-bridge that prevents shoot-through. It is set in the `foc0` devicetree node. |

## Motor-Control Terms

| Preferred term | Meaning and usage |
|---|---|
| **FOC** | Field-Oriented Control: control of motor currents in a rotating `d/q` reference frame. |
| **PMSM** | Permanent-Magnet Synchronous Motor. Includes surface-mounted and interior permanent-magnet motors. |
| **IPMSM** | Interior Permanent-Magnet Synchronous Motor. Use when discussing saliency-dependent HFI behavior. |
| **ASM** | Asynchronous Motor, also called an induction motor. Use **ASM** in code and project documentation. |
| **EESM** | Electrically Excited Synchronous Machine, or wound-rotor synchronous motor. |
| **stator** | The stationary electrical part of the motor. Stator resistance, inductance, voltage, and current use the stator terminology. |
| **rotor** | The rotating part of the motor. Rotor flux, rotor angle, rotor speed, and EESM excitation use the rotor terminology. |
| **phase current** | Current in a physical motor phase, normally `a`, `b`, or `c`. |
| **d/q current** | Current components after transforming the phase quantities into the rotating rotor reference frame. Prefer `i_d` and `i_q` when the axes matter. |
| **electrical angle/speed** | Angle or speed in the electrical reference frame. It includes the motor pole-pair relationship. |
| **mechanical angle/speed** | Physical shaft angle or speed. Do not call it electrical unless it has been transformed. |
| **stator reference** | The stationary `alpha/beta` reference frame. |
| **rotor reference** | The rotating `d/q` reference frame. |
| **SVM** | Space Vector Modulation, the modulation step that converts voltage demand into phase duty cycles. Use **SVPWM** when referring to the PWM method in prose. |
| **field weakening** | Reducing effective air-gap flux or `i_d` to keep the demanded voltage within the available DC-link voltage at higher speed. |
| **MTPA** | Maximum Torque Per Ampere, the current-reference strategy used primarily for salient PMSM control. |

## Control-Loop Terms

| Preferred term | Meaning and usage |
|---|---|
| **setpoint** | An external command received at the system boundary, normally through a Cyphal subject. Setpoints are runtime inputs and are not persisted. |
| **reference** | An internal desired value used by a control loop, such as a `d/q` current reference or rotor-angle reference. |
| **measurement** | A value directly obtained from hardware or a sensor, such as phase current or DC-link voltage. |
| **estimate** | A value calculated by an observer from measurements and a model. |
| **hot-path interface** | `hardware/hot_path_interface.hpp`: the hardware access of the control cycle (`hot_path::GetControlCycleSamples`, `SetDutyPlan`, `TripPowerStage`) and the two handlers the platform calls. Bound at link time per platform, no function pointers. |
| **observer** | An algorithm that estimates an unmeasured state, such as rotor angle, speed, or flux. It does not directly command the inverter. |
| **controller** | An algorithm that compares a reference with a measurement or estimate and produces a control output. |
| **inner loop** | The fast current or torque loop that remains active across the outer control modes. |
| **outer loop** | The speed or position loop that generates a torque or current reference. |
| **control mode** | The active outer-loop selection: `TORQUE`, `SPEED`, or `POSITION`. |
| **torque mode** | Direct torque or `q`-axis current command without a speed or position outer loop. |
| **speed mode** | A speed PI loop that generates the torque/current demand. |
| **position mode** | A cascaded position and speed loop. It requires a homed position tracker. |
| **homing** | The process of finding and latching the machine's mechanical zero position. |
| **fault** | A state in which operation must be stopped or limited because a safety or validity condition failed. Use `FAULT` for the enum/state name. |

## Network And Configuration Terms

| Preferred term | Meaning and usage |
|---|---|
| **Cyphal** | The network protocol used by UNIMOC. Write **Cyphal (UAVCAN v1)** when introducing it; use **Cyphal** afterward. |
| **node** | One networked drive on the Cyphal bus. |
| **node ID** | The numeric Cyphal identity in the range `1..127`; `0` means that plug-and-play allocation is requested. |
| **register** | A named, persistent configuration entry accessed through the Cyphal register service. |
| **subject** | A Cyphal publish/subscribe data channel. Use **input subject** for a subscribed command and **output subject** for published telemetry. |
| **service** | A Cyphal request/response interaction, distinct from a subject. |
| **setpoint subject** | An input subject carrying a runtime command such as torque, speed, position, or excitation. |
| **telemetry** | Runtime output published by the drive, such as estimated angle, speed, current, or DC voltage. |
| **NVM** | Non-volatile memory used to retain configuration across power cycles. Use `NvmSettings` for the aggregate settings type. |
| **persistent setting** | A configuration value exposed through a register. It is live in RAM at once and reaches flash only through an explicit save. |
| **live settings** | The RAM copy of `NvmSettings` that the controller uses and that live tuning changes without any flash access. |
| **save gate** | The `SettingsSaveGate` that allows writing settings to flash only while the power stage is disabled. |
| **runtime value** | A value used during operation but not retained in NVM, such as a setpoint or telemetry sample. |
| **factory default** | The known initial value restored when the NVM magic or layout version is invalid. |

## Naming Rules

- Use the exact code names `MotorType`, `ControlMode`, `NvmSettings`,
	`MechanicalObserver`, `PositionTracker`, and `CyphalInterface` when naming
	those types.
- Use `i_d` and `i_q` for rotating-frame current components, `R` and `L` for
	resistance and inductance only when the surrounding motor context is clear,
	and include units in prose or API names where ambiguity is possible.
- Use **command** or **setpoint** for data coming into the drive; use
	**telemetry**, **measurement**, or **estimate** for data leaving it.
- Use **current limit**, **voltage limit**, and **duty limit** for distinct
	constraints. Do not shorten all of them to **limit** in documentation.
