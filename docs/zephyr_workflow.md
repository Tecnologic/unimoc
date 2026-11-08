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
`boards/unimoc/jacobi/unimoc_jacobi.dts` (node `foc0`, binding
`unimoc,foc-stm32h7`) and, for the NUCLEO-H743ZI, `app/boards/nucleo_h743zi.overlay`.

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

CMake >= 3.28 (Zephyr's minimum since 4.5) and Ninja are required. The CI job
(`.github/workflows/ci-zephyr.yml`) does the same with
`zephyrproject-rtos/action-zephyr-setup`.

## Build

    west build -b native_sim/native/64 unimoc/app -d unimoc/app/build/sim   # host simulation (Cyphal: README.md)
    west build -b unimoc_jacobi unimoc/app -d unimoc/app/build/jacobi         # Jacobi drive (STM32H743VI)
    west build -b nucleo_h743zi unimoc/app -d unimoc/app/build/h743           # STM32H7 platform on a NUCLEO-H743ZI

The Nucleo's on-board ST-LINK (ST firmware) is flashed and debugged through
OpenOCD's DAP interface (`app/boards/nucleo_h743zi_openocd.cfg`; the VS Code
task `Zephyr: flash h743` and the launch configuration use it):

    west flash -d unimoc/app/build/h743 --runner openocd --config unimoc/app/boards/nucleo_h743zi_openocd.cfg

The board's default OpenOCD configuration uses the hla interface, which cannot
drive the reset line or read the H7 debug registers with this probe. A plain
`west flash` uses STM32CubeProgrammer and needs `STM32_Programmer_CLI` installed.

Jacobi is flashed with a J-Link over SWD (`west flash -d unimoc/app/build/jacobi
--runner openocd`, `boards/unimoc/jacobi/support/openocd.cfg`, or `--runner
jlink`); its log is on SEGGER RTT.

CI builds Jacobi and the simulation (workflow `build-firmware.yml`, one matrix row per target) and uploads them as the artifacts
`unimoc-jacobi` (`.elf`, `.hex`, `.bin`, `.map`) and `unimoc-native_sim` (the
executable in a `.tar.gz`, which keeps its executable bit) of the workflow run.
A merged `develop` to `main` pull request attaches them to a GitHub release (see
[release_plan.md](release_plan.md)).

## Tests and static analysis

    unimoc/scripts/twister_sca.sh       # all scenarios, native_sim + unimoc_jacobi + nucleo_h743zi, clang-tidy as SCA (CI)
    unimoc/scripts/check_format.sh      # clang-format check (CI)
    west twister -T unimoc -p native_sim/native/64 -p unimoc_jacobi   # without static analysis

All tests are ztest suites: `tests/lib/<library>/` (one scenario per library
of `lib/`), `tests/sim/<name>/` (cycle cadence, overcurrent, closed loop on the
simulated drive). `twister_sca.sh` builds every scenario with
`ZEPHYR_SCA_VARIANT=codechecker` and clang-tidy as the analyzer, using the
repository `.clang-tidy` and `scripts/codechecker_skip.txt`; a finding in
UNIMOC code fails the build of that scenario.

## Interrupt architecture

| | STM32H7 (`hardware/zephyr_stm32h7`) | native_sim (`hardware/zephyr_sim`) |
|---|---|---|
| ADC trigger | TIM1 TRGO2 = rising OC4REF (CC4 = ARR - lead, before the peak) or rising OC6REF (CC6 = lead, before the valley); ADC1..3 JEXTSEL 8 (TIM8: 10) | - |
| sample (2 × f_pwm) | ADC1 end of injected sequence (JEOS), zero latency: raw ia/ib/ic and DC link, raw overcurrent window and overvoltage level, break on violation; trips kept for the next read | plant step per half period, sample and checks after each step |
| control step (2 f_pwm / divider) | inside the sample ISR after every sample (divider 1) or every valley: `SampleHandler()` → `OnSample()`, a control step every `CallsPerStep()` calls, duties into the CCR preload | after every sample or valley; a kernel timer in simulated time runs four half periods (two PWM periods) per expiry |
| duties | divider 1: TIM1 RCR = 0, an update at every boundary; else RCR = 1 with the repetition counter started at 0, updates at the peak only; CCR1..3 preload, written by the control step (no DMA) | the duties of a step take effect at the next update after its sample |
| observer (at most 16 kHz) | TIM1 update vector, pended by software after every `steps_per_cycle`-th step (DIER stays 0), highest normal priority | directly after the step that ends a control cycle |

`adc_trigger_offset` of the control core is the lead in timer ticks before
peak and valley. Sample times are set per channel in `sample-times` of the
foc0 node: 2.5 cycles for the INA240 current channels, 16.5 for dividers,
32.5 for the NTCs. At 30 MHz ADC clock the sequence (2.5 + 16.5 + 32.5
cycles plus 3 x 6.5 for the 12 bit conversions) takes about 2.4 us after the
trigger, the current sample itself happens within the first 0.1 us. The core's default lead is 1 us
and is tuned by the startup sweep. The phase that switches next to a sample
(highest duty at a peak, lowest at a valley, within lead + 1 us of the
boundary) is reconstructed from the other two, so the duty range is limited
by the minimum pulse (factory 3 %..97 %), not by the sampling window. The
sequence must end after the boundary of its trigger (lead below about 2.4 us),
or the duties of the step take effect half a period early. The sequence time
is an estimate; the interrupt times are measured (`docs/hotpath_plan.md`, section 4).

The sample ISR is a zero-latency direct ISR (`IRQ_ZERO_LATENCY`,
`CONFIG_ZERO_LATENCY_IRQS`): never masked by `irq_lock()`, no kernel calls and
no logging in the handlers. It calls `SampleHandler()` directly; the
core reaches the hardware through `hardware/hot_path_interface.hpp`, without
function pointers. The UNIMOC code is built with `-fno-math-errno`, so
`std::sqrt` in the hot path is an FPU instruction, not a libm call. The control step must finish before the next
boundary (about 13 us after the JEOS at 32 kHz; measured worst case 5.7 us on the Nucleo);
`foc_stm32h7::DutyOverruns()` counts misses.
The observer interrupt is preempted by the sample ISR and must finish within
one cycle (`state.observer_overruns`). `tests/sim/irq_cadence` checks
the cadence, the half periods the duties of each step reach and the
trips of unread samples,
`tests/sim/closed_loop` runs current steps through the core against
the plant.

## Cyphal thread

`main()` starts the Cyphal thread (`hardware/zephyr_common/cyphal_runtime.cpp`,
priority `CONFIG_UNIMOC_CYPHAL_THREAD_PRIORITY`, below every interrupt). It
wakes on each received frame and at least every millisecond and runs the node
and the drive application (`src/drive/drive_node.cpp`). It reaches the
control cycle only through `CurrentControlIsr::SetCurrentRef()`, reads its
state for the telemetry and restarts the runtime (`StartRuntime()`) with new
settings while the power stage is disabled. A restart first stops the cycle
source (`HardwareInterface::Stop()`; STM32H7: both interrupts off, TIM1
stopped, the injected sequences stopped and the ADCs disabled), so
the control state is not rewritten under the running interrupts and the ADC
calibration and `JSQR` writes of `Initialize()` meet the reference manual.

| | STM32H7 | native_sim |
|---|---|---|
| CAN interfaces | FDCAN1 (chosen `zephyr,canbus`), CAN FD frames at 1/1 Mbit/s, transceiver standby pin low (Jacobi: TCAN334) | SocketCAN `vcan0` (`--can-if`), plus slcan on `uart_1`..`uart_4` (pseudo terminals) |
| Node-ID | `uavcan.node.id`, 0 = plug-and-play | 10 (sim profile), `--node-id=<n>` |
| Unique-ID | hwinfo (STM32 UID) | hwinfo (`gethostid()`, `--device_id=<n>`) |
| Restart | `sys_reboot()` | re-executes the process (`CONFIG_NATIVE_SIM_REBOOT`) |
| Extra registers | none | `unimoc.sim.*`, `uavcan.pub.sim_plant.*` |

Kconfig: `CONFIG_UNIMOC_CYPHAL` (app `prj.conf`), `..._CAN_FD`, `..._CAN_BRS`
(off on Jacobi and the Nucleo), `..._ARENA_SIZE` (libcanard heap, 24 KB), `..._LOG_BACKEND`
and `..._LOG_LEVEL` (log lines up to info as diagnostic records),
`..._SLCAN` (native_sim); the traces have `CONFIG_UNIMOC_TRACE_BUFFER_BYTES`
(8192), `..._VARS` (32) and `..._CHANNELS` (8). The simulation runs in real time
(`CONFIG_NATIVE_SIM_SLOWDOWN_TO_REAL_TIME`, `app/boards/native_sim_native_64.conf`).
Tests: `unimoc.lib.cyphal` (transport and node against a second libcanard
node), `unimoc.lib.cymon` (cymon-lib with the Kconfig limits, captures and
the generated `cymon.*` serialization), `unimoc.sim.cyphal_drive` (DriveNode with the plant) and
`unimoc.app.cyphal_integration`: twister's pytest harness starts the real
application, and pycyphal (`tests/integration/cyphal_sim/`) checks
heartbeat, GetInfo, the port list, registers, arming, setpoints, telemetry,
diagnostics, the simulation registers, trace captures (cymon) and a store
across a restart over the
first slcan pseudo terminal. The DSDL is compiled into
`<tmp>/unimoc_pycyphal` on the first run; without pycyphal the tests are
skipped.
The tests wait for the event they check instead of a guessed duration
(`SimClient.expect()`: the first message from now on that satisfies a
condition; `expect_register()`, `expect_record()`), so a loaded machine only
makes them slower. Their durations and timeouts are virtual time, counted by
the control cycles of the telemetry (`SimClient.virtual_time`): on a loaded
machine the simulation falls behind the wall clock. The slcan ports buffer
16 KB each for a reader that falls behind (`SlcanMedia::kTxBufferSize`), so a
slow test client gets its responses late instead of not at all; the client
reads the terminal in bulk (python-can reads it byte by byte, too slow on a
loaded machine). The tests run with a 2 s `unimoc.control.command_timeout`
(factory 0.5 s): the client publishes the readiness in wall time, and a
starved client must not disengage the drive between two publications.

## Power stage release

`PowerStageInterface` (`hardware/power_stage_interface.hpp`): the outputs stay
off after init (MOE = 0, idle level driven). The controller releases them with
`Enable()`; `Disable()` and `Trip()` (overcurrent in the fast ISR, analog
watchdog, runaway controller) switch them off, `Trip()` also latches a fault
until `ClearFault()`. On the STM32H7 the external enable is the break input
TIM1_BKIN (PE15 on Jacobi and the Nucleo, active low, internal pull-down):
open, it breaks in hardware, and `Enable()` fails while it is missing or a fault is latched.
States: disabled, enabled, fault, blocked. The simulation mirrors this and
offers `sim::SetExternalEnable()`.

**Dead time.** `dead-time-ns` in the foc0 node (1 us on Jacobi, as in the
legacy firmware of the 4850 power board: BSC040N08 with LM5109B; to be tuned
with the scope). The layer rounds it up to a DTG step
(`hardware/stm32_dead_time.hpp`, 5 ns steps at 200 MHz up to 635 ns, coarser
above) and fails `Initialize()` if it is longer than the field can encode. Two parallel
MOSFETs per switch would need roughly twice the value.

**Hooks in the core.** `SetOvercurrentLimit()` (main passes
`max_phase_current` of the hardware profile) also sets the per-sample check of
the hardware layer, which trips the stage at the sample. The control step
(`CurrentControlIsr::OnSample()`) checks the calibrated sample again before
any control computation. Over the limit it calls
`kPowerStage.Trip()`, latches
`state.overcurrent_tripped` and holds the duties at 50 % until
`ClearTrips()`. `HwStartup::RequestRun()` is the explicit
acknowledgement: it clears a latched break and the trip, then calls
`Enable()`; without the external enable the run is not started and the FSM
stays IDLE. While a run is active `RunOnce()` leaves for FAULT as soon as the
stage is no longer enabled (trip, external break, e-stop), and
`EnterFault()` calls `Disable()`. `tests/sim/overcurrent` covers these paths.

## Known issues

- Cyphal on Jacobi is built only: FDCAN1 with PLL2 Q (80 MHz) as kernel
  clock, CAN FD frames without bit rate switch at 1 Mbit/s like the other
  nodes, although the TCAN334 handles CAN FD data rates; a bus with
  classic-only adapters needs `CONFIG_UNIMOC_CYPHAL_CAN_FD=n`. Bit timing,
  the standby pin and the bus are checked on the board.
- The console and the log of Jacobi are on SEGGER RTT, those of the Nucleo on
  the ST-LINK virtual COM port. USB and the shell are not used; USB returns
  for DFU recovery.
- The pytest scenario runs pytest with `-p no:platformdirs`
  (`app/testcase.yaml`): platformdirs 4.13 registers a pytest plugin, and on
  the CI runner (`pip --user` on the system Python) an older platformdirs on
  the path makes its import fail before any test runs.
- The app board files of the simulation are named for the 64-bit target
  (`app/boards/native_sim_native_64.*`); the 32-bit `native_sim` build does
  not pick them up.
- The Jacobi pins follow the proposal of `docs/drive4850_h7_plan.md`; the
  board is not drawn yet. The HSE (8 MHz) is an assumption. The gate driver
  enable `PWM_EN` (PE7) is in the devicetree but not driven by the platform
  yet. Not run on hardware yet.
- Limits (STM32H7 and sim profile): `max_phase_current` 60 A is the hard trip
  limit of the power stage (the INA240A2 chain saturates at about +-66 A, so
  it stays inside the sensor range), `max_motor_current` and the factory
  `motor_i_max` 50 A limit the current controller. The battery current limits
  are still the old values.
- Nothing watches the power stage after the startup run is DONE except the
  per-sample overcurrent trip; the hardware break still works. The board has no
  comparators on the current channels; the analog watchdog is not configured.
- Settings persist through the Zephyr settings subsystem on ZMS
  (`hardware/zephyr_common/zephyr_settings_storage.cpp`): flash simulator on
  native_sim, the 256 KB `storage_partition` in flash bank 2 on the STM32H7. The
  ZMS behavior on the H7 flash (128 KB sectors, erase time, garbage collection,
  power loss) is not verified on the board.
- The sim plant (`lib/sim`) has no dead time, no ADC noise and no switching
  ripple; the sim checks logic and timing contract, not the time budget.
- The control step runs on the Nucleo at 64 kHz without a late duty write
  (2026-11-03, outputs off); the PWM edges, the peak/valley detection and the
  ADC alignment are not checked on the scope yet.
- The cymon traces sample at the end of the control cycle (`cymon::Device::Tick()`,
  zero latency, inside the last step of the cycle); its cost is part of the
  measured sample interrupt (no capture armed).
