# Zephyr workflow (skeleton)

Scope: Zephyr is the primary OS. Logging, shell, USB, Cyphal, settings and
startup run in Zephyr threads. Only the FOC hot path (the sample ISR and the
control cycle it runs) runs in zero-latency IRQs; the observer runs in a
software-pended normal-priority interrupt (`docs/hotpath_plan.md`).

Platform selection is done by the build (board -> Kconfig `UNIMOC_PLATFORM_*` ->
`app/CMakeLists.txt`), never by `#ifdef`. Pinout and mapping live in
`boards/unimoc/bcc/bcc.dts` (node `foc0`, binding `unimoc,foc-stm32g4`).

## Setup (Linux, on Windows inside WSL)

    python3 -m venv .zvenv && . .zvenv/bin/activate && pip install west
    mkdir ws && cd ws && west init -l ../unimoc   # or: west init -m <repo> --mr feature/zephyr-port ws
    west update && west packages pip --install
    # Zephyr SDK 1.0.1 (arm-zephyr-eabi) and CMake >= 3.20 required (Zephyr's minimum)

## Build

    west build -b native_sim/native/64 unimoc/app -d unimoc/app/build/sim   # host simulation
    west build -b bcc unimoc/app -d unimoc/app/build/bcc                      # battery case controller

## Tests

    west twister -T unimoc -p native_sim/native/64 -p bcc

`tests/twister/lib_tests` runs every `lib/*/tests/*_test.cpp` unchanged under
GoogleTest on native_sim, one twister scenario per file.

## Interrupt architecture

| | BCC (`hardware/zephyr_bcc`) | native_sim (`hardware/zephyr_sim`) |
|---|---|---|
| ADC trigger | TIM8 TRGO2 = rising OC4REF (CC4 = ARR - lead, before the peak) or rising OC6REF (CC6 = lead, before the valley); ADC1..5 JEXTSEL 10 | - |
| sample (64 kHz) | ADC1 end of injected sequence (JEOS), zero latency: raw ia/ib/ic into the cycle array, raw overcurrent window, break on violation | plant step per half period, sample and overcurrent check after each step |
| control cycle (16 kHz) | inside every 4th sample ISR: `on_control_cycle()`, duty entry 0 into the CCR preload | kernel timer in simulated time, one expiry per cycle |
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
no logging in the callbacks. The control cycle must finish before the next
boundary (about 14 us at 32 kHz); `foc_stm32g4::DutyOverruns()` counts misses.
The observer interrupt is preempted by the sample ISR and must finish within
one cycle (`state.observer_overruns`). `tests/twister/sim_irq_cadence` checks
the cadence and the half periods each plan entry reaches,
`tests/twister/sim_closed_loop` runs current steps through the core against
the plant.

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

**Hooks in the core.** `set_overcurrent_limit()` (main passes
`max_phase_current` of the hardware profile) also sets the per-sample check of
the hardware layer, which trips the stage at the sample. The control cycle
(`CurrentControlIsr::on_control_cycle()`) checks the samples of the cycle
again before any control computation. Over the limit it calls
`power_stage.Trip()`, latches
`state.overcurrent_tripped` and holds the duties at 50 % until
`clear_overcurrent_trip()`. `HwStartup::request_run()` is the explicit
acknowledgement: it clears a latched break and the trip, then calls
`Enable()`; without the external enable the run is not started and the FSM
stays IDLE. While a run is active `run_once()` leaves for FAULT as soon as the
stage is no longer enabled (trip, external break, e-stop), and
`enter_fault()` calls `Disable()`. `tests/twister/sim_overcurrent` covers these paths.

## Known issues

- Not integrated yet: the Cyphal stack, the USB CDC-ACM console and the Zephyr
  flash settings backend. Logging goes to the Zephyr log; the BCC USB stays
  disabled (`docs/zephyr_port_plan.md`, Phases 4 and 5).
- `control_mode_test` and `nvm_settings_test` are skipped: they still use the
  old enumerator names (`POSITION`, `PMSM`) and do not compile against the lib.
- Pins were taken from the KiCad netlist (Tecnologic/battery-case-controller).
  LEDs, CAN slope, break, hall and pulse signals are in the devicetree; FDCAN3
  and USB have pinctrl but stay disabled until their steps. The gate drivers
  (LM5109B) have no enable pin. Not run on hardware yet.
- Limits (BCC and sim profile): `max_phase_current` 60 A is the hard trip
  limit of the power stage (the INA240A2 chain saturates at about +-66 A, so
  it stays inside the sensor range), `max_motor_current` and the factory
  `motor_i_max` 50 A limit the current controller. The battery current limits
  are still the old values.
- Nothing watches the power stage after the startup run is DONE except the
  per-sample overcurrent trip; the hardware break still works. The board has no
  comparators on the current channels; the analog watchdog is not configured.
- The schematic symbol of U43 is an STM32G491CxT, the firmware targets the
  STM32G473CBT6 (FDCAN3 on PB3/PB4 and ADC4/ADC5 only exist there).
- The CAN transceiver is a SN65HVD230 (classic CAN, no CAN FD).
- Settings are kept in RAM only (`ram_settings_storage.hpp`) until a flash
  backend exists.
- The sim plant (`lib/sim`) has no dead time, no ADC noise and no switching
  ripple; the sim checks logic and timing contract, not the time budget.
- The duty DMA, the CCR preload, the counter start at ARR/2 and the
  peak/valley detection are only build-tested, not run on the board.
- `cymon` needs CMake 4 and is not linked yet. Its C++ standard is unverified.
