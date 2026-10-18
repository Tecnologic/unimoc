# Zephyr workflow (skeleton)

Platform selection is done by the build (board -> Kconfig `UNIMOC_PLATFORM_*` ->
`app/CMakeLists.txt`), never by `#ifdef`. Pinout and mapping live in
`boards/unimoc/bcc/bcc.dts` (node `foc0`, binding `unimoc,foc-stm32g4`).

## Setup (Linux, on Windows inside WSL)

    python3 -m venv .zvenv && . .zvenv/bin/activate && pip install west
    mkdir ws && cd ws && west init -l ../unimoc   # or: west init -m <repo> --mr feature/zephyr-port ws
    west update && west packages pip --install
    # Zephyr SDK 1.0.1 (arm-zephyr-eabi) and CMake >= 4.0 required

## Build

    west build -b native_sim/native/64 unimoc/app   # host simulation
    west build -b bcc unimoc/app                    # battery case controller

## Tests

    west twister -T unimoc -p native_sim/native/64 -p bcc

`tests/twister/lib_tests` runs every `lib/*/tests/*_test.cpp` unchanged under
GoogleTest on native_sim, one twister scenario per file.

## Interrupt architecture

| | BCC (`hardware/zephyr_bcc`) | native_sim (`hardware/zephyr_sim`) |
|---|---|---|
| ADC trigger | TIM8 TRGO2 = rising OC4REF (CC4 = ARR - lead, before the peak) or rising OC6REF (CC6 = lead, before the valley); ADC1..5 JEXTSEL 10 | - |
| fast (current control) | ADC1 end of injected sequence (JEOS), once per half PWM period | kernel timer in simulated time, once per half PWM period |
| slow (every 4th) | TIM8 update event, RCR = 3 (update every RCR + 1 half periods), on the peak | every 4th timer expiry, after the fast callback |

`adc_trigger_offset` of the control core is the lead in timer ticks before
peak and valley. With the duty limited to 5 %..95 % nothing switches within
5 % of ARR after the valley / before the peak; the sample is centered there.
Sample times are set per channel in `sample-times` of the foc0 node: 2.5 cycles
for the INA240 current channels, 24.5 for dividers, torque and NTCs. At 42.5 MHz
ADC clock the sequence (2.5 + 24.5 + 2 x 12.5 cycles plus trigger sync) takes
about 1.3 us after the trigger, the current sample itself happens within the
first 0.1 us. At 20 kHz the quiet window is about 1.25 us on each side of a
boundary, so a lead of roughly 15-20 ticks centers the current sample and
leaves only a few 100 ns of the window for the fast ISR. The core's default
lead is 1 us and is tuned by the startup sweep. Tighten once the ISR run time
is known. All figures are estimates, not measurements.

Both BCC interrupts are zero-latency direct ISRs (`IRQ_ZERO_LATENCY`,
`CONFIG_ZERO_LATENCY_IRQS`): never masked by `irq_lock()`, no kernel calls and
no logging in the callbacks. They share one NVIC level and do not preempt each
other. `tests/twister/sim_irq_cadence` checks the 1/4 cadence of the simulation.

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

**Hooks in the core.** `CurrentControlIsr::on_jeoc()` checks every phase
current against `set_overcurrent_limit()` (main passes
`max_phase_current` of the hardware profile) before any control computation.
Over the limit it calls `power_stage.Trip()`, latches
`state.overcurrent_tripped` and holds the duties at 50 % until
`clear_overcurrent_trip()`. `HwStartup::request_run()` is the explicit
acknowledgement: it clears a latched break and the trip, then calls
`Enable()`; without the external enable the run is not started and the FSM
stays IDLE. While a run is active `run_once()` leaves for FAULT as soon as the
stage is no longer enabled (trip, external break, e-stop), and
`enter_fault()` calls `Disable()`. `tests/twister/sim_overcurrent` covers these paths.

## Known issues

- `control_mode_test` and `nvm_settings_test` are skipped: they still use the
  old enumerator names (`POSITION`, `PMSM`) and do not compile against the lib.
- Pins were taken from the KiCad netlist (Tecnologic/battery-case-controller).
  LEDs, CAN slope, break, hall and pulse signals are in the devicetree; FDCAN3
  and USB have pinctrl but stay disabled until their steps. The gate drivers
  (LM5109B) have no enable pin. Not run on hardware yet.
- The hard overcurrent limit is the profile's `max_phase_current` (100 A on the
  BCC), but the INA240A2 chain (25 mV/A, 3.3 V) saturates at about +-66 A, so
  that limit can never fire. Set the profile to a value inside the sensor range.
- Nothing watches the power stage after the startup run is DONE except the
  ISR overcurrent trip; the hardware break still works. The analog watchdog is
  not configured.
- The schematic symbol of U43 is an STM32G491CxT, the firmware targets the
  STM32G473CBT6 (FDCAN3 on PB3/PB4 and ADC4/ADC5 only exist there).
- The CAN transceiver is a SN65HVD230 (classic CAN, no CAN FD).
- Settings are kept in RAM only (`ram_settings_storage.hpp`) until a flash
  backend exists.
- The sim analog side is a stub (zero currents, 48 V DC link).
- The phase between the control core's sub-step counter and the TIM8 update
  event is not synchronised; `SlowUpdate` polls `samples_ready`, so this only
  costs jitter.
- `cymon` needs C++23/CMake 4 and is not linked yet.
