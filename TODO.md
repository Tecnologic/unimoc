# TODO list for open topics

## Zephyr port and hot path

See [docs/zephyr_port_plan.md](docs/zephyr_port_plan.md) for the decisions and
phases, [docs/zephyr_workflow.md](docs/zephyr_workflow.md) for the build and
interrupt layout, and [docs/hotpath_plan.md](docs/hotpath_plan.md) for the
hot-path design.

### P0 - Decisions

 - [x] Decide the hot-path layout: 32 kHz PWM, 16 kHz control cycle in the sample ISR, duties by CCR preload plus DMA (variant B), observer in a software-pended normal-priority interrupt (`docs/hotpath_plan.md`).
 - [ ] Confirm the CAN-FD transceiver change for the BCC (the board has an SN65HVD230, classic CAN).

### P1 - Integration

 - [x] Align the root `CMakeLists.txt` with the C++20 baseline.
 - [ ] Replace the RTT wording in the comment of `src/startup/StartupResults.hpp` with the Zephyr log.
 - [ ] Enable the USB CDC-ACM console and Zephyr logging on the BCC.
 - [ ] Add a Zephyr flash backend (NVS or ZMS) for `SettingsStorage`; until then settings are RAM only.
 - [ ] Integrate Cyphal over CAN-FD (libcanard, DSDL, register service, PnP, heartbeat, GetInfo).
 - [ ] Delete the legacy firmware in `hardware/batterycasecontroller/`, `cmake/Modm.cmake`, `cmake/ToolChain.cmake`, `requirements.txt`, and `.github/workflows/ci-firmware.yml` once the Zephyr BCC target builds. Its presets, CMake references, and launch config are already removed.
 - [ ] Implement the hot-path milestones M0 and M2 to M8 from `docs/hotpath_plan.md` (M1 is in the code, not run on the board).

## Prioritized software-in-the-loop plan

See [docs/software_in_loop_plan.md](docs/software_in_loop_plan.md) for the
architecture, design decisions, implementation phases, risks, and acceptance
criteria.

### P0 - Establish the deterministic runtime

 - [x] Define and test the virtual-time cycle contract around `CurrentControlIsr::on_control_cycle()` and `SlowUpdate::run_once()` (`tests/twister/sim_irq_cadence`).
 - [x] Introduce injectable ADC/PWM I/O for the current-control runtime while preserving the target hardware implementation.
 - [ ] Extract the current-control runtime into a reusable target independent of firmware `main.cpp`.
 - [ ] Add a HAL-free virtual clock and simulation runtime harness with reset, fast-step, slow-update, advance, and snapshot operations.

### P1 - Build the single-motor vertical slice

 - [x] Implement a validated PMSM plant model using `NvmSettings`, existing frame types, and unit types (`lib/sim/`, `MotorSimulation`, with 120 degree Hall sensors).
 - [x] Add plant tests for electrical dynamics, torque direction, pole-pair scaling, mechanics, friction, parameter validation, and repeatability (`lib/sim/tests/`).
 - [x] Connect one host runtime and its ISR I/O to the PMSM plant instead of the zero-value host stubs (`hardware/zephyr_sim`, native_sim).
 - [x] Add a deterministic end-to-end C++ step-response test through the actual current-control runtime (`tests/twister/sim_closed_loop`).
 - [ ] Add exact fast-loop/slow-loop cadence tests and finite-state checks for NaN/Inf and unstable output.

### P1 - Resolve the Cyphal transport path

 - [ ] Run a small Windows localhost Cyphal/UDP feasibility spike using `pycyphal`.
 - [ ] Select the C++ transport and DSDL generation approach for registers, setpoints, telemetry, and heartbeat.
 - [ ] Confirm one register exchange and one telemetry subject before designing the full simulator protocol.

### P2 - Add coordinated multi-motor simulation

 - [ ] Add a one-process simulator coordinator with N independent motor nodes and one shared virtual clock.
 - [ ] Define a barriered multi-motor step with coherent timestamps and deterministic ordering.
 - [ ] Add independent constant/viscous load models and tests for isolated motors, unique node IDs, reset, and repeatability.
 - [ ] Add a shared load interface suitable for shaft, wheel, battery, differential, and torque-vectoring models.

### P2 - Expose the simulator over Cyphal/UDP

 - [ ] Build a host simulator executable around the coordinator.
 - [ ] Map standard Cyphal register operations to the settings operation boundary.
 - [ ] Route torque, speed, position, and excitation setpoints into the normal controller input path.
 - [ ] Publish telemetry and heartbeat with explicit virtual-time semantics.
 - [ ] Add simulation-control commands for reset, fixed-cycle advance, run-until, snapshot, and clock status.
 - [ ] Add protocol and localhost end-to-end tests for single- and multi-node operation.

### P3 - Create the Python command-and-control GUI

 - [ ] Create a separate Python simulation package and dependency file; keep firmware `requirements.txt` focused on firmware tooling.
 - [ ] Implement a `pycyphal` client for node discovery, registers, setpoints, telemetry, reset, and virtual-time stepping.
 - [ ] Build a PySide6/pyqtgraph GUI for single-motor step responses and virtual-time control.
 - [ ] Add scenario definitions, run/pause/reset/step controls, CSV/JSON trace export, and replay.
 - [ ] Keep network I/O and long simulation batches outside the UI thread; support telemetry decimation.
 - [ ] Add headless Python protocol and smoke tests against the host simulator.

### P4 - Expand fidelity and application scope

 - [ ] Add ASM and EESM plants behind the common plant contract (`MotorModel` concept and the `MotorSimulation::Model` variant in `lib/sim/`).
 - [ ] Add startup, observer, field-weakening, homing, and control-mode transition scenarios.
 - [ ] Add vehicle-level models for shared battery/DC-link behavior, wheels, differential load, road load, and torque vectoring.
 - [ ] Add fault injection, profiling, and large-N motor trace tooling.
 - [ ] Consider process-per-motor hosting only after the one-process coordinator is stable and a concrete isolation need exists.

### Validation and documentation

 - [ ] Add focused CMake/Ninja and CTest targets for plant, runtime, coordinator, and protocol tests.
 - [ ] Run clang-format and clang-tidy on changed C++ code.
 - [ ] Preserve the existing focused three-phase validation while SIL remains outside the active production scope.
 - [ ] Update `AGENTS.md` when SIL targets become part of the validated production scope.
 - [ ] Document simulator setup, launch commands, supported model fidelity, and GUI workflows in `README.md`.