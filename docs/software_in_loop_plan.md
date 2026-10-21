# Host Software-in-the-Loop Plan

> **Status (2026-10-21):** partly superseded. The single-motor host runtime is
> the Zephyr `native_sim` platform (`hardware/zephyr_sim/`, see
> `docs/zephyr_port_plan.md`, Phase 2), not `hardware/host/`. The network
> interface is **Cyphal/CAN**, not Cyphal/UDP: the simulation is a real-time
> Cyphal node on SocketCAN `vcan0` or on slcan pseudo terminals, reachable
> with pycyphal and yakut (README.md, "Simulation over Cyphal"). The
> portable simulation library, the multi-motor coordinator, the
> virtual-time simulation commands and the Python GUI plans still apply.
> Where this file says `hardware/host`, read `hardware/zephyr_sim`; where it
> says Cyphal/UDP, read Cyphal/CAN.

## Goal

Build a deterministic software-in-the-loop (SIL) environment that runs the real UNIMOC control runtime against simulated motors on the host. The environment should support repeatable virtual-time experiments, integration tests, a Python command-and-control GUI, Cyphal/UDP access, and multiple motors running together for applications such as vehicle torque vectoring.

The first useful milestone is deliberately narrow:

> One real current-control runtime, one PMSM plant, explicit virtual time, and a deterministic C++ step-response test.

The GUI and network interface come after this vertical slice works.

## Architectural Decisions

- **First motor model:** PMSM. ASM and EESM are later implementations behind the same plant interface.
- **Fidelity target:** exercise the actual current-control runtime, including its fast ISR cadence and slow observer update, not only isolated portable controllers.
- **Time model:** explicit virtual time and fixed cycle advancement. Wall-clock execution is optional for visualization and never controls deterministic tests.
- **Host topology:** one C++ simulator process containing N independent motor nodes and one coordinator. Process-per-motor hosting is deferred.
- **Network interface:** the Python GUI is the command-and-control application and communicates with the simulator using real Cyphal/UDP semantics.
- **Protocol:** use standard Cyphal DSDL for registers, setpoints, telemetry, and heartbeat. Add a small vendor-specific simulation-control namespace for reset and virtual-time commands.
- **Separation:** motor equations belong in the simulation library, not in `hardware/zephyr_sim/hardware_interface.cpp`. The host hardware layer only binds callbacks and settings to the simulator.

## Current Repository Boundary

The existing code has three relevant layers:

1. `lib/` contains portable control algorithms, observers, coordinate transforms, units, and settings types.
2. `src/current_control/` contains the actual current-control ISR and slow update implementation. Its ADC, PWM, trigger, and timer operations are supplied by `HardwareInterface` callbacks.
3. `hardware/` contains target-specific hardware implementations. `hardware/zephyr_sim/` binds the public `HardwareInterface` callbacks and the hot-path interface for `native_sim` and drives the plant of `lib/sim/`.

The current-control ISR now calls the public hardware boundary for ADC samples, PWM duties, trigger timing, and applied-duty inspection. The BCC implementation binds those operations to STM32G4 peripherals through Zephyr's devicetree and LL drivers. The `native_sim` implementation drives the simulator plant. All tests run as ztest suites under twister; `tests/sim/` already runs the current-control runtime against the plant.

## Target Architecture

```text
Python GUI / scenario client
          |
       Cyphal/UDP
          |
+---------v-----------------------------------+
| Host simulator process                      |
|                                             |
|  Cyphal application and simulation control  |
|  N-motor coordinator + one virtual clock    |
|       |              |                      |
|  Motor node 0    Motor node 1 ...           |
|  runtime + plant runtime + plant            |
|       |              |                      |
|       +------- shared load / vehicle -------+
+---------------------------------------------+
```

The C++ simulator remains the source of truth for controller execution and physics. The Python side sends commands, receives telemetry, defines scenarios, and visualizes traces. It must not duplicate motor equations or controller behavior.

## Execution Model

Each motor runtime should expose operations equivalent to:

- `Reset()` - restore settings, controller state, plant state, and virtual time.
- `RunControlCycle()` - execute exactly one control cycle (`OnControlCycle()`, then the observer, then the plant over four half periods). On native_sim this is `hardware::sim::StepCycle()`.
- `AdvanceCycles(count)` - execute a fixed number of coordinated cycles.
- `Snapshot()` - return controller, plant, timing, and telemetry state.

On native_sim these exist in `hardware/zephyr_sim/sim_control.hpp` as `Reset()`, `StepCycle()`, `AdvanceCycles()`, `VirtualTimeNs()` and `TakeSnapshot()` (platform and plant state). The controller is restarted with `current_control::StartRuntime()` (`src/current_control/current_control_runtime.hpp`), the same call `main()` makes; controller state is read from `CurrentControlIsr::Instance().state`. Tests: `tests/sim/closed_loop`.

The exact cycle ordering must be specified and tested. A recommended ordering is:

1. Deliver inputs queued for the current virtual timestamp.
2. Sample simulated ADC/current/voltage values.
3. Execute one current-control fast sub-step.
4. Apply the PWM command to each plant and advance each plant by `dt_fast`.
5. Run the slow update after `kNumSubSteps` fast steps.
6. Publish or record one coherent snapshot.
7. Advance the virtual clock by the fixed interval.

The implementation must not depend on `std::chrono`, sleeps, thread scheduling, or real-time deadlines.

## Simulation Library

Add a HAL-free simulation-owned library, for example `lib/simulation/`, with these responsibilities:

### Virtual clock and runtime harness

The harness owns virtual time and makes the actual current-control runtime callable from tests and the simulator executable. It should expose deterministic reset, stepping, slow-update scheduling, and trace snapshots without depending on Qt, Python, Cyphal, or firmware `main.cpp`.

### Plant contract

Define a narrow plant contract with explicit input and output semantics:

- Input: normalized phase duties, DC-link voltage, external load torque, and `dt`.
- State: phase or rotor-frame currents, electrical angle/speed, mechanical angle/speed, flux state, and diagnostics.
- Output: measured phase currents, phase voltages, rotor angle/speed, electromagnetic torque, and fault indicators.
- Sampling: duty commands are held between samples; reads return the plant state at a defined sample boundary.
- Integration: begin with a stable fixed-step low-order integrator; keep more detailed inverter switching or solver choices behind the same interface.

### PMSM model

Start with a PMSM model using validated `NvmSettings` parameters:

- Stator resistance `R_s`
- `L_d` and `L_q`
- Permanent-magnet flux linkage `psi_pm`
- Pole-pair count
- Rotor/load inertia `J`
- DC-link voltage
- Constant load torque
- Viscous friction
- Optional Coulomb friction

The core equations should cover d/q electrical dynamics, electromagnetic torque, mechanical acceleration, and electrical/mechanical angle integration. Use the existing `ThreePhase`, `Stator`, `Rotor`, and unit types at frame boundaries. Validate physical parameters and reject invalid configurations deterministically.

### Host hardware adapter

Keep `HardwareInterface` as the public hardware boundary. The host adapter should delegate:

- `GetPhaseCurrents()` to plant measurements.
- `GetPhaseVoltages()` to plant voltage output.
- `SetPhaseDuties()` to the plant's held inverter command.
- ISR ADC reads to the same simulated measurement source.
- ISR timer writes to a captured PWM command source.

`hardware/zephyr_sim/hardware_interface.cpp` should connect these pieces and retain file-backed settings behavior, but should not contain motor equations.

## Implementation Phases

### Phase 0: Resolve the contracts

1. Specify the fast/slow cycle ordering and sample boundary semantics.
2. Introduce the injectable ADC/PWM I/O abstraction used by the ISR.
3. Keep target hardware bindings intact and add a host binding for SIL.
4. Confirm the C++ runtime can be built as a reusable target independently of firmware `main.cpp`.
5. Run a small Cyphal/UDP feasibility spike using maintained `pycyphal` tooling and confirm one register exchange plus one telemetry subject on localhost between `native_sim` and `pycyphal` (WSL2 on Windows).
6. Decide the C++ DSDL generation and transport stack before committing to the full network API. Existing `lib/cyphal/cyphal_interface.hpp` constants are useful identifiers but are not serialization or transport support.

### Phase 1: Single-motor C++ vertical slice

1. Add the virtual clock and runtime harness.
2. Add the PMSM plant and parameter validation.
3. Bind the plant to host hardware and ISR I/O.
4. Initialize the model from validated settings.
5. Add a deterministic one-motor executable or test harness that can reset, step, and snapshot.
6. Prove that repeated runs with the same initial state and cycle count produce identical traces.

### Phase 2: Integration tests

Add focused tests for:

- Zero-input and neutral-duty behavior.
- Balanced three-phase reconstruction.
- Current response to a known voltage.
- PMSM torque sign and pole-pair scaling.
- Inertia, load torque, and friction behavior.
- Invalid parameter rejection.
- Exact fast-loop and slow-loop cadence.
- No NaN or infinite state.
- Current, speed, or torque step response through the actual runtime.
- Reset and deterministic replay.

Keep SIL tests separate from the currently validated `three_phase_system_test` baseline until the project explicitly expands its validation scope.

### Phase 3: Multi-motor coordinator

Add N independent motor nodes in one C++ process. Each node owns its own settings, control runtime, plant, trace, and node ID. The coordinator owns:

- One virtual clock.
- Deterministic update order.
- Queued input delivery.
- Global reset.
- Coherent multi-node snapshots.
- Shared load and vehicle model updates.

Use a barriered step so all motors observe the same virtual timestamp and no motor's result depends on iteration order:

1. Deliver all inputs.
2. Sample all motors.
3. Run all fast controller steps.
4. Advance all plants.
5. Update shared mechanical or vehicle loads.
6. Run due slow updates.
7. Publish one coherent snapshot.

Begin with independent constant and viscous loads. Then add shared shaft, wheel, battery/DC-link, differential, and road-load models as separate system models. This supports torque-vectoring scenarios without coupling vehicle behavior into PMSM equations.

### Phase 4: Cyphal/UDP simulator

Build a host simulator executable around the coordinator. It should expose one Cyphal node per simulated motor plus a coordinator endpoint for simulation control.

The Cyphal application layer should:

- Read and write registers through the settings operation boundary.
- Deliver torque, speed, position, and excitation setpoints into the normal controller input path.
- Publish angle, speed, current, voltage, state, and heartbeat telemetry.
- Provide simulation controls for reset, fixed-cycle advance, run-until, snapshot, and virtual-clock status.
- Keep continuous best-effort mode optional and separate from deterministic stepping.

Add protocol-level tests for register operations, setpoint delivery, telemetry decoding, heartbeat, reset, stepping, and multi-node addressing. Add one end-to-end localhost UDP test that launches the simulator and verifies a known telemetry trace.

### Phase 5: Python command-and-control GUI

Create a separate Python package and dependency file. Do not add GUI and network dependencies to the firmware-focused `requirements.txt`.

Recommended initial stack:

- Python 3.11 or newer.
- `pycyphal`.
- PySide6.
- `pyqtgraph`.
- `pytest`.
- `qasync` if needed to integrate asyncio and the Qt event loop.

The GUI should provide:

- Node discovery and configuration.
- Register editing.
- Torque, speed, position, and excitation setpoints.
- Reset, step, pause, and run controls.
- Virtual clock and node health display.
- Live decimated telemetry.
- Full-resolution trace capture.
- CSV/JSON export and replay.

The first scenario workflow should reset one node, apply a step at virtual cycle zero, advance a fixed number of cycles as fast as possible, plot command and measured response, and report settling time, overshoot, steady-state error, and limit violations.

Keep long simulation batches and Cyphal I/O outside the UI thread. The GUI should remain responsive without imposing real-time behavior on the simulator.

### Phase 6: Expansion

After the PMSM runtime and tests are stable:

1. Add ASM and EESM plants behind the same contract.
2. Add startup, observers, field weakening, homing, and control-mode transitions one scenario at a time.
3. Add application models for shared batteries, wheels, differentials, road loads, and torque vectoring.
4. Add fault injection and trace/replay tooling.
5. Profile large N-motor runs.
6. Add process-per-motor hosting only if isolation or deployment realism justifies its coordination cost. Reuse the same Cyphal-facing protocol.

## Implemented Plant Library (`lib/sim/`)

The plant contract and the first model exist as the header-only target
`unimoc_sim` (`unimoc::sim`). It is not linked into `unimoc_lib`; the
`native_sim` hardware layer binds it in a separate step.

- `MotorSimulation` is the one API for every motor type. `Configure()` takes
  `SimulationParameters`; `ParametersFromSettings()` (or the `NvmSettings`
  overload of `Configure()`) makes the motor type follow
  `NvmSettings::motor_type`. A type without a model is rejected with
  `Status::kUnsupportedMotorType` and the previous configuration stays active.
- Per step (normally half a PWM period): averaged inverter voltage from the held
  duties, electrical model step with the mechanical state held, shaft step,
  Hall evaluation. `Output()` returns the sample at the end of the step: phase
  currents and voltages, DC-link voltage, torque, mechanical and electrical
  angle and speed, and the Hall signals.
- Electrical models fulfil the `MotorModel` concept (`motor_model.hpp`) and are
  held in a `std::variant`, so there is no heap and no virtual dispatch. Only
  `PmsmModel` exists so far (backward-Euler d/q model with saliency). ASM and
  EESM models plug in behind the same concept; `ElectricalInput` already carries
  the field voltage for the EESM.
- `MechanicalModel` integrates inertia, load torque, viscous and Coulomb
  friction, or imposes a speed (dynamometer, locked rotor). The electrical
  angle is derived exactly from the mechanical `RotorAngle` with
  `RotorAngle::FromAbsoluteRaw()`.
- `HallSensor` models three digital Hall sensors 120 degrees electrical apart
  with a mounting offset; it reports the code and the interpolated age of the
  last edge, as a timer-captured Hall input would.

Not modelled yet: dead time and switch drops, diode freewheeling of an open
bridge (an open stator forces zero current), saturation, temperature, Hall
faults and sensor noise.

## Suggested Repository Layout

```text
lib/simulation/
  virtual_clock.hpp
  motor_plant.hpp
  pmsm_plant.hpp
  simulation_runtime.hpp
  simulation_coordinator.hpp
  load_model.hpp
  trace.hpp

tools/simulator/
  main.cpp
  cyphal_node.cpp
  simulation_control.cpp

tools/sim_gui/
  pyproject.toml
  sim_gui/
  tests/

tests/
  pmsm_plant_test.cpp
  simulation_runtime_test.cpp
  simulation_coordinator_test.cpp
  cyphal_simulator_test.cpp
```

The exact directory names can follow existing project conventions. The important boundaries are that the simulation library is portable and testable, the simulator process owns transport integration, and the Python package owns presentation and scenarios.

## Verification and Acceptance Criteria

### C++

- Twister scenarios (ztest on `native_sim`) exist for plant, runtime, coordinator, and protocol tests.
- Plant tests cover equations, signs, bounds, validation, and repeatability.
- Runtime tests execute the actual fast and slow control cadence.
- No deterministic test uses wall-clock time.
- Clang-format and clang-tidy pass for changed C++ code.
- Existing focused three-phase validation remains green.

### Cyphal

- A client can discover or address each simulated motor.
- Register writes update the simulator through the normal settings path.
- Setpoints affect the real control input path.
- Telemetry decodes correctly and has coherent virtual timestamps.
- Reset and fixed-cycle advance are deterministic.
- Multiple nodes can be addressed independently.

### Python GUI

- The GUI can start or connect to a simulator on Windows.
- A one-motor step response can be run without real-time delay.
- A trace can be exported, reset, and replayed identically.
- A multi-motor scenario shows independent node telemetry on one virtual clock.
- A shared-load scenario demonstrates deterministic coupled behavior.
- Automated smoke tests run without a display.

## Main Risks

1. **DSDL and transport integration:** the repository has Cyphal names and a heartbeat serializer, but no complete transport stack. Resolve this before building the GUI around an assumed protocol.
2. **ISR coupling:** current ADC and timer hooks are local to the ISR implementation. The I/O seam must preserve target behavior while making host simulation observable and controllable.
3. **Numerical stability:** fixed-step integration must remain stable at the controller's configured PWM cadence. Add bounds and finite-state checks early.
4. **Timing semantics:** all motors must advance through a shared virtual-time barrier. Avoid a design where Python or network arrival time controls plant state.
5. **Scope creep:** vehicle dynamics, additional motor types, and polished GUI features should follow the single-motor runtime proof rather than precede it.

## Recommended Starting Point

Start with the first five tasks in `TODO.md`:

1. Write down and test the virtual-time cycle contract.
2. Add injectable ADC/PWM I/O to the current-control runtime.
3. Extract a reusable runtime target.
4. Implement and unit-test the PMSM plant.
5. Connect one host runtime to that plant and run a deterministic step response.

Once those pass, the Cyphal/UDP spike and Python client have a stable execution model to connect to.
