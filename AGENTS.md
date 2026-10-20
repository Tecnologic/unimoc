# UNIMOC Agent Notes

UNIMOC is a platform-independent C++ motor-control library and Zephyr firmware
for field-oriented control (FOC) of multi-phase electric motors. One drive
typically controls one motor; multiple drives can be networked through Cyphal
(UAVCAN v1) over CAN-FD on the STM32G4 and over UDP on the STM32H7.

Read `README.md` for the supported motor types and public features, and
`CONTEXT.md` for the project's domain vocabulary. The port decisions are in
`docs/zephyr_port_plan.md`, the build and interrupt layout in
`docs/zephyr_workflow.md`, and the hot-path design in `docs/hotpath_plan.md`.

## Runtime Scope

- **Zephyr is the primary OS.** Logging, shell, USB, Cyphal, settings storage,
  startup sequencing, and all other high-level code run in Zephyr threads.
  The same application runs on the battery case controller (BCC) and on
  `native_sim` for host simulation.
- **Only the FOC hot path runs in zero-latency IRQs.** The hot path is the
  sample ISR (ADC end of injected sequence, once per half PWM period: raw copy
  and overcurrent check) and the control cycle it runs every fourth sample
  (16 kHz at 32 kHz PWM: PI once, four duties for the next four half periods,
  entry 0 into the CCR preload and entries 1 to 3 by DMA). The observer
  (`SlowUpdate`) runs after each control cycle in a software-pended
  normal-priority interrupt, not at zero latency (`docs/hotpath_plan.md`).
  Do not add other work to a zero-latency context.

## Project Layout

- `lib/motor_control/` contains the portable coordinate systems, control
  algorithms, and observers. Its public headers live in `src/`; its tests are
  in `tests/lib/motor_control/`.
- `lib/sim/` is the portable motor plant simulation (`MotorSimulation`: inverter,
  motor models, shaft, Hall sensors) for `native_sim`. It is not part of
  `unimoc_lib`.
- `lib/settings/`, `lib/units/`, and `lib/cyphal/` are separate library
  dependencies. The Zephyr build links them through `unimoc_lib`
  (`cmake/UnimocZephyr.cmake`).
- `src/` contains the application core: `current_control/` (hot-path ISR and
  slow update), `startup/` (startup FSM), and `main.cpp`. It stays free of
  Zephyr headers.
- `zephyr/` makes the repository a Zephyr module: `module.yml` (board and
  devicetree roots, test roots), `Kconfig` (`CONFIG_UNIMOC`, the platform
  choice `CONFIG_UNIMOC_PLATFORM_*`, the log level) and `CMakeLists.txt`,
  which declares the library targets (`unimoc_lib`, `unimoc_sim`) through
  `cmake/UnimocZephyr.cmake` when `CONFIG_UNIMOC=y`.
- `app/` is the Zephyr application: `CMakeLists.txt`, `prj.conf`, and the
  `native_sim` configuration. Platform sources are selected from the board's
  Kconfig symbol (`CONFIG_UNIMOC_PLATFORM_*`), not with `#ifdef`. `main()`
  sets the drive up and returns; the control runs in interrupts.
- `boards/unimoc/bcc/` is the BCC board definition. `bcc.dts` (node `foc0`)
  holds the pinout, timers, ADC trigger, dead time, and sample times.
- `hardware/zephyr_bcc/` is the BCC platform (STM32G4 hot-path bindings).
  `hardware/zephyr_sim/` is the `native_sim` platform. `hardware/zephyr_common/`
  is the shared Zephyr glue (system interface, RAM settings storage).
  `hardware/hardware_interface.hpp` and `hardware/power_stage_interface.hpp`
  are the platform-independent boundaries.
- `hardware/hot_path_interface.hpp` is the link-time bound hardware access of
  the zero-latency hot path.
- `hardware/batterycasecontroller/` is the legacy firmware, pending removal once
  the Zephyr BCC target is verified (`docs/zephyr_port_plan.md`). Do not extend it.
- `tests/` contains all tests, as ztest suites run by twister:
  `tests/lib/<library>/` covers `lib/<library>`, `tests/sim/<name>/` runs the
  application core against the `native_sim` platform and the motor plant
  (cycle cadence, overcurrent, closed loop). `tests/common/` holds the shared
  float checks.
- `scripts/` holds the validation scripts (`check_format.sh`,
  `twister_sca.sh`) that CI runs as well.
- `docs/` contains the plans and the workflow notes. `TODO.md` lists open topics.
- `lib/cymon/` is a submodule.
  Do not modify `lib/cymon/` unless the task explicitly concerns that
  dependency.

## Build And Test

### Active refactor validation scope

The system library is currently under active review and is not yet a
production baseline. Until the user explicitly expands this scope, treat only
the following as validated:

- `lib/units/units.hpp`
- `lib/motor_control/src/three_phase_system.hpp`
- `tests/lib/motor_control/src/three_phase_system_test.cpp` (suites
	`three_phase` and `unit_literal` of the `unimoc.lib.motor_control` scenario)

Failures outside this scope are expected refactor work rather than a
regression in the validated baseline. Revisit this section whenever the user
declares another path or test production-valid.

### Zephyr only: twister, ztest, clang-tidy as SCA

Everything is built and tested with Zephyr tooling. There is no hosted CMake
build, no CMake preset and no GoogleTest. The workflow is in
[the zephyr-build-test skill](.github/skills/zephyr-build-test/SKILL.md).
Run from the west workspace root, which contains `unimoc/` and `zephyr/`
(setup in `docs/zephyr_workflow.md`; on Windows in WSL2):

```sh
unimoc/scripts/check_format.sh     # clang-format check (--fix to apply)
unimoc/scripts/twister_sca.sh      # all scenarios, native_sim + bcc, with clang-tidy
```

These two are the standard validation and exactly what CI
(`.github/workflows/ci-zephyr.yml`) runs. Focused loops while iterating:

```sh
west twister -T unimoc/tests/lib/motor_control -p native_sim/native/64
west build -b native_sim/native/64 unimoc/app -d unimoc/app/build/sim   # host simulation
west build -b bcc unimoc/app -d unimoc/app/build/bcc                      # battery case controller
```

### Getting it right in a new session

Earlier sessions lost time on these points; do them from the start:

- **Set up the workspace first** when `west` or `zephyr/` is missing: a
  Python venv with `west` and `codechecker`, `west init -l unimoc && west
  update` from the parent directory of the repository, then the Python
  requirements (`zephyr/scripts/requirements-*.txt`) and the Zephyr SDK
  (minimal bundle plus `arm-zephyr-eabi`, `setup.sh -t arm-zephyr-eabi -h -c`).
  Initialize the `lib/cymon` submodule. Do not fall back to a hosted build.
- **Tests are ztest under twister.** Never add GoogleTest, a hosted CMake
  target or a test that only runs outside twister. New tests go into the
  matching `tests/lib/<library>/src/` or a new `tests/<area>/<name>/`
  scenario. Use `zexpect_*`/`zassert_*`, plus `UNIMOC_EXPECT_FLOAT_EQ` /
  `UNIMOC_EXPECT_NEAR` from `tests/common/unimoc_test.hpp` for floats.
- **`twister_sca.sh` must pass** (clang-tidy included) and
  **`check_format.sh` must pass** before a change is done or a PR is opened.
  Do not treat a successful build or editor diagnostics as a substitute.
- **Naming as enforced by `.clang-tidy`:** `PascalCase` types and functions
  (also methods), `snake_case` variables, parameters and members (private
  members end in `_`), `kPascalCase` for every `const`/`constexpr` variable
  that is not a parameter (also locals) and for enumerators.
- **`NOLINT`** only with the check name and a reason in the same comment.
  The directory `.clang-tidy` files in `hardware/zephyr_*` and `tests/` only
  waive checks that Zephyr's own macros cannot meet; do not widen them.

## Coding Guidelines

- prefer #pragma once over include guards. remove include guards when applicable.
- Use the Google C++ style configured in `.clang-format`.
- Run the repository's clang-format configuration when changing C++.
- Use `snake_case` for all file names, including C++ headers and sources.
- Use Google C++ identifier naming in code: `PascalCase` for types and
	functions, `snake_case` for variables and data members, and `kPascalCase`
	for constants and enumerators.
- Use C++20 as the project baseline (`docs/zephyr_port_plan.md`). The root
	`CMakeLists.txt` is set to C++20, so do not rely on
	C++23-only features.
- Keep algorithms deterministic, portable, and free of HAL and Zephyr dependencies.
- In embedded numeric code, keep calculations in the project's unit types and
	use their operators instead of extracting scalar values and rebuilding units.
	Avoid `double` and hand-written unit scale conversions unless a documented
	precision requirement needs them and the target supports them; prefer an
	existing typed API such as `RotorAngle::AbsoluteAngle()`.
- Prefer small, focused changes. Preserve public APIs unless the task requires
	a breaking change, and avoid unrelated refactors.
- Add or update ztest coverage in `tests/lib/` for changes to control,
	observer, system, or unit behavior.
- Treat compiler warnings as errors in project code and keep it compatible
	with the strict warning set. Zephyr and vendored headers are not held to
	that set (`cmake/UnimocZephyr.cmake`).
- Be especially careful with current limits, PWM, dead time, the power-stage
	enable, startup, NVM settings, devicetree pin mapping, and Cyphal register or
	subject IDs. Do not assume hardware behavior from the tests or the
	simulation; document hardware-specific assumptions and validate firmware
	changes with the appropriate target build.

### Zero-latency rules

Code reachable from a zero-latency IRQ (the sample ISR, the control cycle, and
their callbacks) must not:

- call Zephyr kernel or driver APIs, take locks, or allocate;
- log, print, or touch the shell, USB, Cyphal, or settings;
- call libm functions (use `-fno-math-errno` and builtins) or use function
  pointers for the hot-path hardware access (`docs/hotpath_plan.md`).

Everything else runs in a Zephyr thread. The zero-latency handlers reach the
hardware only through `hardware/hot_path_interface.hpp` (functions bound at
link time per platform, no function pointers); the thread context uses the
`HardwareInterface` callbacks. The Zephyr build compiles with
`-fno-math-errno` (`cmake/UnimocZephyr.cmake`).

## Git Safety

- Do not stage or commit changes unless the user explicitly requests it.
- Do not create branches, reset, checkout files, abort, or continue a merge or
  rebase unless the user explicitly requests that operation.
- Leave the working tree and merge state available for the user when a workflow
  reaches a git lifecycle boundary.

## Change Checklist

Before finishing a change:

1. Run `unimoc/scripts/check_format.sh` and `unimoc/scripts/twister_sca.sh`
	 from the west workspace root (twister on `native_sim` and `bcc`, clang-tidy
	 as Zephyr SCA). Use a focused `west twister -T ...` run while iterating.
	 If a step cannot run, report the blocker and the checks that did run.
2. Fix clang-tidy findings in the change, or add a `NOLINT` with the check
	 name and a justification.
3. Check the diff for accidental changes to generated files, build output, or
	 submodules. The west workspace (`zephyr/`, `modules/`) is not part of this
	 repository; `build/`, `app/build/` and `twister-out/` are git-ignored.
4. Update `README.md`, `CONTEXT.md`, and the affected `docs/` plan when
	 user-visible behavior, configuration, build steps, hardware bring-up
	 behavior, or a planned decision changes.

## Skills

Based on [mattpocock's skills](https://github.com/mattpocock/skills), adapted for
VS Code Copilot and this project. Skills are organized under `.github/skills/`:

- [Zephyr build and test](.github/skills/zephyr-build-test/SKILL.md) —
	west builds, twister (ztest), clang-tidy as Zephyr SCA, and clang-format.
- [File docs](.github/skills/file-docs/SKILL.md) — Doxygen comments and file headers
	for C++ headers and sources.

Use the repository's `SKILL.md` files as the source of truth for these
workflows.
