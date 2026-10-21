# Zephyr workflow (skeleton)

Scope: Zephyr is the primary OS. Logging, Cyphal, settings and startup run
in Zephyr threads. Only the FOC hot path (the sample ISR and the
control cycle it runs) runs in zero-latency IRQs; the observer runs in a
software-pended normal-priority interrupt (`docs/hotpath_plan.md`).

The repository is a Zephyr module (`zephyr/module.yml`). With `CONFIG_UNIMOC=y`
in a `prj.conf`, `zephyr/CMakeLists.txt` declares the library targets
(`unimoc_lib`, `unimoc_sim`), which the application and the tests link;
`zephyr/Kconfig` holds `CONFIG_UNIMOC`, the platform choice and the log level.
Platform selection is done by the build (board -> Kconfig `UNIMOC_PLATFORM_*` ->
`app/CMakeLists.txt`), never by `#ifdef`. Pinout and mapping live in
`boards/unimoc/bcc/bcc.dts` (node `foc0`, binding `unimoc,foc-stm32g4`).

## Setup (Linux, on Windows inside WSL)

The workspace root is the parent directory of this repository (`west init -l`
uses the directory above the manifest repository):

    mkdir ws && cd ws && git clone --recursive <repo-url> unimoc
    python3 -m venv .venv && . .venv/bin/activate
    pip install west codechecker clang-format==22.1.1   # clang-format pinned: the output depends on the version
    west init -l unimoc && west update
    west packages pip --install                  # Zephyr's Python requirements
    pip install -r unimoc/scripts/requirements-cyphal.txt   # Nunavut (DSDL code generation)
    west sdk install -t arm-zephyr-eabi          # Zephyr SDK 1.0.1; without GitHub API access:
    #   download zephyr-sdk-1.0.1_linux-x86_64_minimal.tar.xz from the sdk-ng release,
    #   unpack it and run ./setup.sh -t arm-zephyr-eabi -h -c
    sudo apt-get install clang-tidy              # LLVM 18, as on the CI runner

CMake >= 3.20 (Zephyr's minimum) and Ninja are required. The CI job
(`.github/workflows/ci-zephyr.yml`) does the same with
`zephyrproject-rtos/action-zephyr-setup`.

## Build

    west build -b native_sim/native/64 unimoc/app -d unimoc/app/build/sim   # host simulation (Cyphal: README.md)
    west build -b bcc unimoc/app -d unimoc/app/build/bcc                      # battery case controller

## Tests and static analysis

    unimoc/scripts/twister_sca.sh       # all scenarios, native_sim + bcc, clang-tidy as SCA (CI)
    unimoc/scripts/check_format.sh      # clang-format check (CI)
    west twister -T unimoc -p native_sim/native/64 -p bcc   # without static analysis

All tests are ztest suites: `tests/lib/<library>/` (one scenario per library
of `lib/`), `tests/sim/<name>/` (cycle cadence, overcurrent, closed loop on the
simulated drive). `twister_sca.sh` builds every scenario with
`ZEPHYR_SCA_VARIANT=codechecker` and clang-tidy as the analyzer, using the
repository `.clang-tidy` and `scripts/codechecker_skip.txt`; a finding in
UNIMOC code fails the build of that scenario.

## Interrupt architecture

| | BCC (`hardware/zephyr_bcc`) | native_sim (`hardware/zephyr_sim`) |
|---|---|---|
| ADC trigger | TIM8 TRGO2 = rising OC4REF (CC4 = ARR - lead, before the peak) or rising OC6REF (CC6 = lead, before the valley); ADC1..5 JEXTSEL 10 | - |
| sample (64 kHz) | ADC1 end of injected sequence (JEOS), zero latency: raw ia/ib/ic into the cycle array, raw overcurrent window, break on violation | plant step per half period, sample and overcurrent check after each step |
| control cycle (16 kHz) | inside every 4th sample ISR: `OnControlCycle()`, duty entry 0 into the CCR preload | kernel timer in simulated time, one expiry per cycle |
| duties | TIM8 RCR = 0, CCR1..3 preload; DMA1 ch1 (DMAMUX TIM8_UP) bursts 3 words per update event into CCR1..3 from 4 circular slots | entry 3 of the previous plan, then entries 0..2 of the new one, one per half period |
| observer (16 kHz) | TIM8 update vector, pended by software (UIE stays 0), highest normal priority | directly after the control callback |

`adc_trigger_offset` of the control core is the lead in timer ticks before
peak and valley. Sample times are set per channel in `sample-times` of the
foc0 node: 2.5 cycles for the INA240 current channels, 24.5 for dividers,
torque and NTCs. At 42.5 MHz ADC clock the sequence (2.5 + 24.5 + 2 x 12.5
cycles plus trigger sync) takes about 1.3 us after the trigger, the current
sample itself happens within the first 0.1 us. The core's default lead is 1 us
and is tuned by the startup sweep. The phase that switches next to a sample
(highest duty at a peak, lowest at a valley, within lead + 1 us of the
boundary) is reconstructed from the other two, so the duty range is limited
by the minimum pulse (factory 3 %..97 %), not by the sampling window. All
figures are estimates, not measurements.

The sample ISR is a zero-latency direct ISR (`IRQ_ZERO_LATENCY`,
`CONFIG_ZERO_LATENCY_IRQS`): never masked by `irq_lock()`, no kernel calls and
no logging in the handlers. It calls `ControlCycleHandler()` directly; the
core reaches the hardware through `hardware/hot_path_interface.hpp`, without
function pointers. The UNIMOC code is built with `-fno-math-errno`, so
`std::sqrt` in the hot path is an FPU instruction, not a libm call. The control cycle must finish before the next
boundary (about 14 us at 32 kHz); `foc_stm32g4::DutyOverruns()` counts misses.
The observer interrupt is preempted by the sample ISR and must finish within
one cycle (`state.observer_overruns`). `tests/sim/irq_cadence` checks
the cadence and the half periods each plan entry reaches,
`tests/sim/closed_loop` runs current steps through the core against
the plant.

## Cyphal thread

`main()` starts the Cyphal thread (`hardware/zephyr_common/cyphal_runtime.cpp`,
priority `CONFIG_UNIMOC_CYPHAL_THREAD_PRIORITY`, below every interrupt). It
wakes on each received frame and at least every millisecond and runs the node
and the drive application (`src/drive/drive_node.cpp`). It reaches the
control cycle only through `CurrentControlIsr::SetCurrentRef()`, reads its
state for the telemetry and restarts the runtime (`StartRuntime()`) with new
settings while the power stage is disabled.

| | BCC | native_sim |
|---|---|---|
| CAN interfaces | FDCAN3 (chosen `zephyr,canbus`), CAN FD frames at 1/1 Mbit/s (SN65HVD230), Rs pin low | SocketCAN `vcan0` (`--can-if`), plus slcan on `uart_1`..`uart_4` (pseudo terminals) |
| Node-ID | `uavcan.node.id`, 0 = plug-and-play | 10 (sim profile), `--node-id=<n>` |
| Unique-ID | hwinfo (STM32 UID) | hwinfo (`gethostid()`, `--device_id=<n>`) |
| Restart | `sys_reboot()` | re-executes the process (`CONFIG_NATIVE_SIM_REBOOT`) |
| Extra registers | none | `unimoc.sim.*`, `uavcan.pub.sim_plant.*` |

Kconfig: `CONFIG_UNIMOC_CYPHAL` (app `prj.conf`), `..._CAN_FD`, `..._CAN_BRS`
(off on the BCC), `..._ARENA_SIZE` (libcanard heap, 24 KB), `..._LOG_BACKEND`
and `..._LOG_LEVEL` (log lines up to info as diagnostic records),
`..._SLCAN` (native_sim). The simulation runs in real time
(`CONFIG_NATIVE_SIM_SLOWDOWN_TO_REAL_TIME`, `app/boards/native_sim_native_64.conf`).
Tests: `unimoc.lib.cyphal` (transport and node against a second libcanard
node), `unimoc.sim.cyphal_drive` (DriveNode with the plant) and
`unimoc.app.cyphal_integration`: twister's pytest harness starts the real
application, and pycyphal (`tests/integration/cyphal_sim/`) checks
heartbeat, GetInfo, the port list, registers, arming, setpoints, telemetry,
diagnostics, the simulation registers and a store across a restart over the
first slcan pseudo terminal. The DSDL is compiled into
`<tmp>/unimoc_pycyphal` on the first run; without pycyphal the tests are
skipped.

## Power stage release

`PowerStageInterface` (`hardware/power_stage_interface.hpp`): the outputs stay
off after init (MOE = 0, idle level driven). The controller releases them with
`Enable()`; `Disable()` and `Trip()` (overcurrent in the fast ISR, analog
watchdog, runaway controller) switch them off, `Trip()` also latches a fault
until `ClearFault()`. On the BCC the external enable is the break input
TIM8_BKIN (PA10, active low, internal pull-down): the Enable connector only
ties it to 3V3 through 1k and does not supply the logic. Removing it breaks in
hardware, and `Enable()` fails while it is missing or a fault is latched.
States: disabled, enabled, fault, blocked. The simulation mirrors this and
offers `sim::SetExternalEnable()`.

**Dead time.** `dead-time-ns` in the foc0 node (400 ns on the BCC: BSC040N08
with 22 R gate resistors, turn-off about 250 ns plus reserve; an estimate, to
be tuned with the scope). The BCC layer rounds it up to a DTG step
(`hardware/zephyr_bcc/dead_time.hpp`, 5.9 ns steps up to 747 ns) and fails
`Initialize()` if it is longer than the field can encode. Two parallel
MOSFETs per switch would need roughly twice the value.

**Hooks in the core.** `SetOvercurrentLimit()` (main passes
`max_phase_current` of the hardware profile) also sets the per-sample check of
the hardware layer, which trips the stage at the sample. The control cycle
(`CurrentControlIsr::OnControlCycle()`) checks the samples of the cycle
again before any control computation. Over the limit it calls
`kPowerStage.Trip()`, latches
`state.overcurrent_tripped` and holds the duties at 50 % until
`ClearOvercurrentTrip()`. `HwStartup::RequestRun()` is the explicit
acknowledgement: it clears a latched break and the trip, then calls
`Enable()`; without the external enable the run is not started and the FSM
stays IDLE. While a run is active `RunOnce()` leaves for FAULT as soon as the
stage is no longer enabled (trip, external break, e-stop), and
`EnterFault()` calls `Disable()`. `tests/sim/overcurrent` covers these paths.

## Known issues

- Cyphal on the BCC is built only: FDCAN3 with PCLK1 (170 MHz) as kernel
  clock, CAN FD frames without bit rate switch at 1 Mbit/s because the
  SN65HVD230 is a classic CAN transceiver; a bus with classic-only adapters
  needs `CONFIG_UNIMOC_CYPHAL_CAN_FD=n`. Bit timing, the Rs pin and the bus
  are checked on the board.
- The console and the log of the BCC are on SEGGER RTT. USB and the shell are
  not used (the CDC-ACM console and the HSI48/CRS clock recovery were removed
  on 2026-10-21); USB returns for DFU recovery.
- The pytest scenario runs pytest with `-p no:platformdirs`
  (`app/testcase.yaml`): platformdirs 4.13 registers a pytest plugin, and on
  the CI runner (`pip --user` on the system Python) an older platformdirs on
  the path makes its import fail before any test runs.
- The app board files of the simulation are named for the 64-bit target
  (`app/boards/native_sim_native_64.*`); the 32-bit `native_sim` build does
  not pick them up.
- Pins were taken from the KiCad netlist (Tecnologic/battery-case-controller).
  LEDs, CAN slope, break, hall and pulse signals are in the devicetree; FDCAN3
  is enabled for Cyphal; USB is not used. The gate drivers
  (LM5109B) have no enable pin. Not run on hardware yet.
- Limits (BCC and sim profile): `max_phase_current` 60 A is the hard trip
  limit of the power stage (the INA240A2 chain saturates at about +-66 A, so
  it stays inside the sensor range), `max_motor_current` and the factory
  `motor_i_max` 50 A limit the current controller. The battery current limits
  are still the old values.
- Nothing watches the power stage after the startup run is DONE except the
  per-sample overcurrent trip; the hardware break still works. The board has no
  comparators on the current channels; the analog watchdog is not configured.
- The schematic symbol of U43 is an STM32G491CxT, the board carries and the
  firmware targets the STM32G473CET6 (512 KB flash, dual bank) (FDCAN3 on PB3/PB4 and ADC4/ADC5 only exist there).
- The CAN transceiver is a SN65HVD230 (classic CAN, no CAN FD).
- Settings persist through the Zephyr settings subsystem on ZMS
  (`hardware/zephyr_common/zephyr_settings_storage.cpp`): flash simulator on
  native_sim, the 16 KB `storage_partition` on the BCC. The ZMS behavior on the
  STM32G4 flash (erase time, garbage collection, power loss) is not verified on
  the board.
- The sim plant (`lib/sim`) has no dead time, no ADC noise and no switching
  ripple; the sim checks logic and timing contract, not the time budget.
- The duty DMA, the CCR preload, the counter start at ARR/2 and the
  peak/valley detection are only build-tested, not run on the board.
- `cymon` needs CMake 4 and is not linked yet. Its C++ standard is unverified.
