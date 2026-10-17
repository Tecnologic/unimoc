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
At 20 kHz that quiet window is about 1.25 us on each side of a boundary while
two injected conversions take about 1.7 us, so the fast ISR cannot finish
inside it yet; tighten once its run time is known.

Both BCC interrupts are zero-latency direct ISRs (`IRQ_ZERO_LATENCY`,
`CONFIG_ZERO_LATENCY_IRQS`): never masked by `irq_lock()`, no kernel calls and
no logging in the callbacks. They share one NVIC level and do not preempt each
other. `tests/twister/sim_irq_cadence` checks the 1/4 cadence of the simulation.

## Known issues

- `control_mode_test` and `nvm_settings_test` are skipped: they still use the
  old enumerator names (`POSITION`, `PMSM`) and do not compile against the lib.
- BCC pins for CAN-FD, USB, LEDs and gate-driver enables are not yet in the
  devicetree. TIM8 dead time and break input are not configured (as in the
  modm code); MOE is set at init. Not run on hardware yet.
- Settings are kept in RAM only (`ram_settings_storage.hpp`) until a flash
  backend exists.
- The sim analog side is a stub (zero currents, 48 V DC link).
- The phase between the control core's sub-step counter and the TIM8 update
  event is not synchronised; `SlowUpdate` polls `samples_ready`, so this only
  costs jitter.
- `cymon` needs C++23/CMake 4 and is not linked yet.
