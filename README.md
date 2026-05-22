# UNIMOC — Universal Motor Control

[![CI](https://github.com/Tecnologic/unimoc/actions/workflows/ci.yml/badge.svg)](https://github.com/Tecnologic/unimoc/actions/workflows/ci.yml)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](LICENSE)
[![C++ Standard](https://img.shields.io/badge/C%2B%2B-23-blue.svg)](https://en.cppreference.com/w/cpp/23)

**UNIMOC** (**UNI**versal **MO**tor **C**ontrol) is a platform-independent,
header-only C++23 library for Field-Oriented Control (FOC) of multi-phase
electric motors.  It targets embedded micro-controllers (STM32 series) but the
algorithm headers are fully portable and tested on x86-64 Linux via
[Google Test](https://github.com/google/googletest).

---

## Table of Contents

1. [Features](#features)
2. [Supported Motor Types](#supported-motor-types)
3. [Library Structure](#library-structure)
4. [Cyphal (UAVCAN v1) Interface](#cyphal-uavcan-v1-interface)
   - [Node ID and Plug-and-Play](#node-id-and-plug-and-play)
   - [Node Identity](#node-identity)
   - [Settings via Registers](#settings-via-registers)
   - [Setpoints via Subjects](#setpoints-via-subjects)
   - [Telemetry Subjects](#telemetry-subjects)
5. [NVM-Backed Settings](#nvm-backed-settings)
6. [Getting Started](#getting-started)
   - [Building the Hosted Tests](#building-the-hosted-tests)
   - [Building Firmware (STM32)](#building-firmware-stm32)
7. [Contributing](#contributing)
8. [License](#license)

---

## Features

| Category | Capability |
|---|---|
| **Motor types** | PMSM, ASM (induction), EESM (wound-rotor synchronous) |
| **Observers** | Back-EMF / PLL (PMSM), HFI sensorless (IPMSM), ASM rotor-flux Luenberger, EESM excitation LPF |
| **Control** | MTPA, field-weakening, ASM flux PI, EESM excitation PI (current or flux mode) |
| **Position** | Absolute multi-turn tracker (±4096+ rev), cascaded P+PI position controller, homing state machine |
| **Modulation** | Space Vector PWM (SVPWM), dead-time compensation |
| **Network** | Full Cyphal (UAVCAN v1) register map — every setting configurable over the bus |
| **Persistence** | All settings stored in NVM; survive power cycles; validated with magic + version check |
| **Platform** | Header-only algorithms; HAL-free; tested on Linux x86-64 with GTest |

---

## Supported Motor Types

### PMSM — Permanent-Magnet Synchronous Motor
Surface-mount and interior-PM motors.  
Control path: MechanicalObserver (back-EMF PLL) → MTPA → FieldWeakening → SVM.  
Low-speed sensorless: HFI observer (requires inductance saliency, IPMSM only).

### ASM — Asynchronous (Induction) Motor
Squirrel-cage induction motors.  
Control path: AsmFluxObserver (Luenberger) → AsmFluxController (PI) → MechanicalObserver (shared PLL) → SVM.

### EESM — Electrically Excited Synchronous Machine
Wound-rotor synchronous motors with external excitation (slip-rings or brushless exciter).  
Stator control path identical to PMSM.  Additionally:
- **ExcitationController** — PI loop on rotor current I_f (CurrentMode) or flux ψ_f = L_m · I_f (FluxMode).  
- **ExcitationObserver** — first-order low-pass filter on measured I_f → adaptive ψ_f estimate fed into MTPA.  
- Excitation setpoint and mode received via Cyphal subject.

---

## Library Structure

```
lib/
├── control/
│   ├── AsmFluxController.hpp     # ASM rotor-flux PI controller
│   ├── DeadTimeCompensation.hpp  # Inverter dead-time compensation
│   ├── ExcitationController.hpp  # EESM rotor excitation PI (current / flux mode)
│   ├── FieldWeakening.hpp        # Voltage-headroom field-weakening integrator
│   ├── Mtpa.hpp                  # Maximum-Torque-Per-Ampere (IPMSM)
│   ├── PositionController.hpp    # Cascaded P+PI position/speed controller + homing FSM
│   └── Svm.hpp                   # Space Vector PWM modulator
├── observer/
│   ├── AsmFluxObserver.hpp       # ASM full-order Luenberger rotor-flux observer
│   ├── ExcitationObserver.hpp    # EESM excitation current LPF → flux estimate
│   ├── Hfi.hpp                   # 4-step High-Frequency Injection (IPMSM standstill)
│   ├── MechanicalObserver.hpp    # Back-EMF observer + PLL (PMSM / shared with ASM)
│   └── PositionTracker.hpp       # Absolute multi-turn position + homing
├── system/
│   ├── ControlMode.hpp           # ControlMode enum: TORQUE / SPEED / POSITION
│   ├── CyphalInterface.hpp       # Register names + subject port IDs (full API map)
│   ├── MotorType.hpp             # MotorType enum: PMSM / ASM / EESM
│   ├── NodeIdentity.hpp          # Node name + hw/sw version (UID read from hardware)
│   ├── NvmSettings.hpp           # Aggregate of all NVM-backed parameters
│   ├── RotorReference.hpp        # dq rotating reference frame
│   ├── SinCos.hpp                # Unit-circle helper
│   ├── StatorReference.hpp       # α/β stationary reference frame
│   └── ThreePhase.hpp            # Three-phase (a/b/c) vector
└── units/
    └── Units.hpp                 # SI unit wrappers
```

---

## Cyphal (UAVCAN v1) Interface

UNIMOC exposes its complete configuration and runtime control via
[Cyphal](https://opencyphal.org/) (UAVCAN v1).  No physical access to the
hardware is required after initial deployment.

### Node ID and Plug-and-Play

If the `uavcan.node.id` register is **0** (the factory default for a blank
device) the firmware automatically initiates the Cyphal plug-and-play
node-ID allocation protocol
(`uavcan.pnp.NodeIDAllocationData.2`).  Once an ID is granted by the
allocator it is written back to `uavcan.node.id` and persisted to NVM so that
the same ID is reused on every subsequent reset.

Set a fixed node ID by writing the register:
```
uavcan.node.id = <1..127>
```

### Node Identity

Each drive has a human-readable identity string (up to 50 UTF-8 bytes) that
is returned in `uavcan.node.GetInfo` responses and stored in NVM.

Set the node name via Cyphal:
```
uavcan.node.description = "unimoc.propulsion.left"
```

The `NodeIdentity` struct carries hardware and software version numbers
(read-only). The 16-byte MCU unique-ID used during PnP allocation is read
directly from hardware at runtime.

### Settings via Registers

Every parameter in `NvmSettings` is accessible as a named Cyphal register
(`uavcan.register.Access` service, `uavcan.register.List` enumeration), so all
in-RAM configuration values are traceable with tools like **Cymon**.
Writing a register updates the in-RAM value and schedules an NVM flush.

Full register name table (`lib/system/CyphalInterface.hpp`):

| Register | Type | Description |
|---|---|---|
| `uavcan.node.id` | `uavcan.primitive.scalar.Natural16.1.0` | Node ID (0 = PnP) |
| `uavcan.node.description` | `uavcan.primitive.String.1.0` | Human-readable node name |
| `unimoc.motor.type` | `uavcan.primitive.scalar.Natural8.1.0` | 0=PMSM, 1=ASM, 2=EESM |
| `unimoc.motor.pole_pairs` | `uavcan.primitive.scalar.Natural8.1.0` | Motor pole-pair count |
| `unimoc.control.mode` | `uavcan.primitive.scalar.Natural8.1.0` | Boot control mode (0=TORQUE, 1=SPEED, 2=POSITION) |
| `unimoc.motor.stator.R` | `uavcan.primitive.scalar.Real32.1.0` | Stator resistance [Ω] |
| `unimoc.motor.stator.L` | `uavcan.primitive.scalar.Real32.1.0` | Stator inductance [H] |
| `unimoc.motor.pmsm.flux_pm` | `uavcan.primitive.scalar.Real32.1.0` | PM flux linkage ψ_PM [Wb] |
| `unimoc.motor.pmsm.L_d` | `uavcan.primitive.scalar.Real32.1.0` | d-axis inductance [H] |
| `unimoc.motor.pmsm.L_q` | `uavcan.primitive.scalar.Real32.1.0` | q-axis inductance [H] |
| `unimoc.motor.asm.R_r` | `uavcan.primitive.scalar.Real32.1.0` | ASM rotor resistance [Ω] |
| `unimoc.motor.asm.R_s` | `uavcan.primitive.scalar.Real32.1.0` | ASM stator resistance [Ω] |
| `unimoc.motor.asm.L_s` | `uavcan.primitive.scalar.Real32.1.0` | ASM stator inductance [H] |
| `unimoc.motor.asm.L_r` | `uavcan.primitive.scalar.Real32.1.0` | ASM rotor inductance [H] |
| `unimoc.motor.asm.L_m` | `uavcan.primitive.scalar.Real32.1.0` | ASM mutual inductance [H] |
| `unimoc.observer.mech.g_i` | `uavcan.primitive.scalar.Real32.1.0` | Back-EMF observer current gain [1/s] |
| `unimoc.observer.mech.g_e` | `uavcan.primitive.scalar.Real32.1.0` | Back-EMF observer EMF gain [V/(A·s)] |
| `unimoc.observer.mech.pll_kp` | `uavcan.primitive.scalar.Real32.1.0` | PLL proportional gain |
| `unimoc.observer.mech.pll_ki` | `uavcan.primitive.scalar.Real32.1.0` | PLL integral gain |
| `unimoc.observer.asm_flux.g_i` | `uavcan.primitive.scalar.Real32.1.0` | ASM flux observer current gain [1/s] |
| `unimoc.observer.asm_flux.g_flux` | `uavcan.primitive.scalar.Real32.1.0` | ASM flux observer flux gain [Wb/(A·s)] |
| `unimoc.control.asm_flux.kp` | `uavcan.primitive.scalar.Real32.1.0` | ASM flux PI proportional gain [A/Wb] |
| `unimoc.control.asm_flux.ki` | `uavcan.primitive.scalar.Real32.1.0` | ASM flux PI integral gain [A/(Wb·s)] |
| `unimoc.control.asm_flux.i_d_min` | `uavcan.primitive.scalar.Real32.1.0` | ASM flux min d-axis current [A] |
| `unimoc.control.asm_flux.i_d_max` | `uavcan.primitive.scalar.Real32.1.0` | ASM flux max d-axis current [A] |
| `unimoc.control.fw.v_max` | `uavcan.primitive.scalar.Real32.1.0` | Field-weakening voltage limit (normalised) |
| `unimoc.control.fw.ki` | `uavcan.primitive.scalar.Real32.1.0` | Field-weakening integrator gain [A/(V·s)] |
| `unimoc.control.fw.i_d_min` | `uavcan.primitive.scalar.Real32.1.0` | Field-weakening min i_d [A] |
| `unimoc.control.svm.duty_min` | `uavcan.primitive.scalar.Real32.1.0` | SVM minimum duty cycle |
| `unimoc.control.svm.duty_max` | `uavcan.primitive.scalar.Real32.1.0` | SVM maximum duty cycle |
| `unimoc.control.dtc.dead_time` | `uavcan.primitive.scalar.Real32.1.0` | Dead time [s] |
| `unimoc.control.dtc.f_pwm` | `uavcan.primitive.scalar.Real32.1.0` | PWM frequency [Hz] |
| `unimoc.control.dtc.i_threshold` | `uavcan.primitive.scalar.Real32.1.0` | Dead-time zero-crossing threshold [A] |
| `unimoc.observer.hfi.v_inject` | `uavcan.primitive.scalar.Real32.1.0` | HFI injection voltage [V] |
| `unimoc.observer.hfi.error_gain` | `uavcan.primitive.scalar.Real32.1.0` | HFI angle-error gain [1/V] |
| `unimoc.control.excitation.mode` | `uavcan.primitive.scalar.Natural8.1.0` | EESM excitation mode (0=current, 1=flux) |
| `unimoc.control.excitation.L_m` | `uavcan.primitive.scalar.Real32.1.0` | EESM mutual inductance [H] |
| `unimoc.control.excitation.kp` | `uavcan.primitive.scalar.Real32.1.0` | Excitation PI proportional gain [V/A] |
| `unimoc.control.excitation.ki` | `uavcan.primitive.scalar.Real32.1.0` | Excitation PI integral gain [V/(A·s)] |
| `unimoc.control.excitation.i_f_min` | `uavcan.primitive.scalar.Real32.1.0` | Min excitation current [A] |
| `unimoc.control.excitation.i_f_max` | `uavcan.primitive.scalar.Real32.1.0` | Max excitation current [A] |
| `unimoc.observer.excitation.tau` | `uavcan.primitive.scalar.Real32.1.0` | Excitation observer LPF time constant [s] |
| `unimoc.observer.excitation.L_m` | `uavcan.primitive.scalar.Real32.1.0` | Excitation observer L_m [H] |
| `unimoc.control.pos.kp` | `uavcan.primitive.scalar.Real32.1.0` | Position loop P gain [rad/s per rad] |
| `unimoc.control.pos.kp_speed` | `uavcan.primitive.scalar.Real32.1.0` | Speed loop P gain |
| `unimoc.control.pos.ki_speed` | `uavcan.primitive.scalar.Real32.1.0` | Speed loop I gain |
| `unimoc.control.pos.speed_limit` | `uavcan.primitive.scalar.Real32.1.0` | Max velocity [rad/s] |
| `unimoc.control.pos.accel_limit` | `uavcan.primitive.scalar.Real32.1.0` | Max acceleration [rad/s²] |
| `unimoc.control.pos.position_tolerance` | `uavcan.primitive.scalar.Real32.1.0` | In-position threshold [rad] |
| `unimoc.control.pos.speed_tolerance` | `uavcan.primitive.scalar.Real32.1.0` | In-position speed threshold [rad/s] |
| `unimoc.control.pos.homing_speed` | `uavcan.primitive.scalar.Real32.1.0` | Homing search velocity [rad/s] |

### Setpoints via Subjects

Runtime setpoints are **ephemeral** (not stored in NVM) and arrive via Cyphal
publisher subjects each control cycle.  The active control mode determines
which subjects are acted on.

| Subject | Default port ID | Direction | Description |
|---|---|---|---|
| `torque_sp` | 100 | → (subscribe) | Torque / q-axis current setpoint |
| `speed_sp` | 101 | → (subscribe) | Speed setpoint [rad/s] |
| `position_sp` | 102 | → (subscribe) | Position setpoint [rad] (requires homing) |
| `control_mode` | 103 | → (subscribe) | Active control mode (0/1/2) |
| `excitation_sp` | 104 | → (subscribe) | EESM excitation setpoint (A or Wb) |
| `homing_trigger` | 105 | → (subscribe) | Any message starts homing sequence |

Port IDs are reconfigurable via `uavcan.sub.<name>.id` registers.
All input/output subjects can be inspected live in Cymon.

### Telemetry Subjects

| Subject | Default port ID | Direction | Description |
|---|---|---|---|
| `rotor_angle` | 200 | ← (publish) | Estimated electrical angle [rad] |
| `rotor_speed` | 201 | ← (publish) | Estimated electrical angular velocity [rad/s] |
| `shaft_position` | 202 | ← (publish) | Absolute shaft position referenced to home [rad] |
| `in_position` | 203 | ← (publish) | In-position flag |
| `homing_state` | 204 | ← (publish) | Homing FSM state (0–4) |
| `dc_voltage` | 205 | ← (publish) | DC-link voltage [V] |
| `phase_current` | 206 | ← (publish) | Phase current magnitude [A] |
| `excitation_current` | 207 | ← (publish) | EESM rotor excitation current î_f [A] |

---

## NVM-Backed Settings

All settings are aggregated in `unimoc::system::NvmSettings`
(`lib/system/NvmSettings.hpp`).  The struct is:

- **Validated** by a magic word (`0x554D4F43` = "UMOC") and a layout version
  number.  If either mismatches on load, factory defaults are restored.
- **Complete** — every tunable parameter from every controller and observer
  lives in a single flat struct, making it trivial to back up or flash a full
  drive configuration.
- **Platform-agnostic** — the NVM driver (flash page, EEPROM, external SPI
  flash) is provided by the hardware layer; the library only defines the data.

```cpp
#include "NvmSettings.hpp"

unimoc::system::NvmSettings cfg;

// Load from NVM (hardware-specific)
nvm_load(reinterpret_cast<uint8_t*>(&cfg), sizeof(cfg));

if (!cfg.is_valid()) {
    cfg.reset_to_defaults();  // blank or corrupt flash
    nvm_save(...);
}

// Apply to controller instances
mechanical_observer.R  = cfg.stator_R;
mechanical_observer.L  = cfg.stator_L;
// ...
```

---

## Getting Started

### Building the Hosted Tests

Requirements: GCC ≥ 13, Ninja, Python ≥ 3.8, CMake 4.x (`pip install cmake`).

```bash
# Install CMake 4.x
pip install cmake

# Configure
cmake -S . -B build/hosted \
      -DENABLE_TESTS=ON \
      -DHARDWARE_DIRECTORY="" \
      -G Ninja \
      -DCMAKE_BUILD_TYPE=Debug

# Build
cmake --build build/hosted --target system_test

# Run
./build/hosted/tests/system_test
```

Expected output: all tests pass.

### Building Firmware (STM32)

Requirements: ARM GCC toolchain (`arm-none-eabi-gcc`), Ninja, CMake 4.x,
Python 3 with `modm` dependencies (`pip install -r requirements.txt`).

```bash
# Battery Case Controller (STM32G473) — debug
cmake --preset "BatteryCaseController Debug"
cmake --build --preset "BatteryCaseController Debug"

# Battery Case Controller — release
cmake --preset "BatteryCaseController Release"
cmake --build --preset "BatteryCaseController Release"
```

The firmware binary and `.hex` file are placed in
`build/batterycasecontroller-{debug,release}/`.

---

## Contributing

Contributions are welcome!  Please:

1. Fork the repository and create a feature branch.
2. Follow the existing code style (`.clang-format` is provided).
3. Add Google Test unit tests for new algorithm headers in `tests/`.
4. Ensure all tests pass (`./build/hosted/tests/system_test`) before opening a
   pull request — CI will verify automatically.
5. Open a pull request against `main`.

---

## License

UNIMOC is free software released under the
**GNU General Public License v3.0**.  See the [`LICENSE`](LICENSE) file for
the full text.
