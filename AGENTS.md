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
- **Only the FOC hot path runs in zero-latency IRQs.** The fast current ISR
  (ADC end of injected sequence, once per half PWM period) is the hot path.
  The TIM8 quarter-cycle update (`SlowUpdate`) is also zero-latency today, but
  whether it stays an IRQ is an open decision (`docs/hotpath_plan.md`, section 7).
  Do not add other work to a zero-latency context.

## Project Layout

- `lib/motor_control/` contains the portable coordinate systems, control
  algorithms, and observers. Its public headers live in `src/` and focused
  tests live in `tests/`.
- `lib/settings/`, `lib/units/`, and `lib/cyphal/` are separate library
  dependencies. The Zephyr build links them through `unimoc_lib`
  (`cmake/UnimocZephyr.cmake`).
- `src/` contains the application core: `current_control/` (hot-path ISR and
  slow update), `startup/` (startup FSM), and `main.cpp`. It stays free of
  Zephyr headers.
- `app/` is the Zephyr application: `CMakeLists.txt`, `prj.conf`, `Kconfig`,
  and the `native_sim` configuration. Platform sources are selected from the
  board's Kconfig symbol (`CONFIG_UNIMOC_PLATFORM_*`), not with `#ifdef`.
- `boards/unimoc/bcc/` is the BCC board definition. `bcc.dts` (node `foc0`)
  holds the pinout, timers, ADC trigger, dead time, and sample times.
- `hardware/zephyr_bcc/` is the BCC platform (STM32G4 hot-path bindings).
  `hardware/zephyr_sim/` is the `native_sim` platform. `hardware/zephyr_common/`
  is the shared Zephyr glue (system interface, RAM settings storage).
  `hardware/hardware_interface.hpp` and `hardware/power_stage_interface.hpp`
  are the platform-independent boundaries.
- `hardware/host/` holds the older host stubs for the hosted build. It is
  replaced by `hardware/zephyr_sim/`.
- `hardware/batterycasecontroller/` is the legacy firmware, pending removal once
  the Zephyr BCC target is verified (`docs/zephyr_port_plan.md`). Do not extend it.
- `tests/` contains hosted GoogleTest coverage for the portable code.
  `tests/twister/` contains Zephyr scenarios: the `lib/*/tests` suites run
  unchanged on `native_sim`, and simulation checks cover IRQ cadence and
  overcurrent handling.
- `docs/` contains the plans and the workflow notes. `TODO.md` lists open topics.
- `lib/cymon/` is a submodule; `lib/gitversion/` is vendored project code.
  Do not modify `lib/cymon/` unless the task explicitly concerns that
  dependency.

## Build And Test

### Active refactor validation scope

The system library is currently under active review and is not yet a
production baseline. Until the user explicitly expands this scope, treat only
the following as validated:

- `lib/units/units.hpp`
- `lib/motor_control/src/three_phase_system.hpp`
- `lib/motor_control/tests/three_phase_system_test.cpp` and its
	`three_phase_system_test` target

For changes confined to this scope, use focused hosted validation first:

```sh
cmake --build --preset "Hosted build with tests" --target three_phase_system_test
ctest --preset "Hosted Test" -R "ThreePhaseTest" --output-on-failure
```

Do not run or require the full hosted workflow, or the Zephyr builds, merely
because an in-progress system, observer, control, firmware, or integration file
changed. Such code may be compiled or tested when needed to investigate the
specific change, but failures outside the active scope are expected refactor
work rather than a regression in the validated baseline. Revisit this section
whenever the user declares another path or test production-valid.

### Hosted (portable code)

Use the CMake presets and Ninja on both Windows and Linux hosts. The canonical
host validation workflow is documented in
[the hosted-build-test skill](.github/skills/hosted-build-test/SKILL.md).

```sh
cmake --workflow --preset "Hosted configure, build and test"
```

For static analysis and formatting, use the `hosted-build-test` skill. It
covers the `clang-tidy` workflow preset and `clang-format` checks on both host
platforms.

### Zephyr (firmware and simulation)

Run these from the west workspace root, which contains `unimoc/` and `zephyr/`.
Setup is in `docs/zephyr_workflow.md`. On Windows the Zephyr builds run in WSL2.

```sh
west build -b native_sim/native/64 unimoc/app   # host simulation
west build -b bcc unimoc/app                    # battery case controller
west twister -T unimoc -p native_sim/native/64 -p bcc
```


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
- Add or update GoogleTest coverage in `tests/` for changes to control,
	observer, system, or unit behavior.
- Treat compiler warnings as errors in project code and keep it compatible
	with the strict warning set. Zephyr and vendored headers are not held to
	that set (`cmake/UnimocZephyr.cmake`).
- Be especially careful with current limits, PWM, dead time, the power-stage
	enable, startup, NVM settings, devicetree pin mapping, and Cyphal register or
	subject IDs. Do not assume hardware behavior from hosted tests or the
	simulation; document hardware-specific assumptions and validate firmware
	changes with the appropriate target build.

### Zero-latency rules

Code reachable from a zero-latency IRQ (the fast ISR, the quarter-cycle update,
and their callbacks) must not:

- call Zephyr kernel or driver APIs, take locks, or allocate;
- log, print, or touch the shell, USB, Cyphal, or settings;
- call libm functions (use `-fno-math-errno` and builtins) or use function
  pointers for the hot-path hardware access (`docs/hotpath_plan.md`).

Everything else runs in a Zephyr thread, and the zero-latency handlers reach
the hardware only through the `HardwareInterface` callbacks.

## Git Safety

- Do not stage or commit changes unless the user explicitly requests it.
- Do not create branches, reset, checkout files, abort, or continue a merge or
  rebase unless the user explicitly requests that operation.
- Leave the working tree and merge state available for the user when a workflow
  reaches a git lifecycle boundary.

## Change Checklist

Before finishing a change:

1. Build and run the focused tests for the active refactor validation scope;
	 use the full hosted workflow only when the user expands the scope or the
	 change requires cross-library validation. For a Zephyr change, build the
	 target it touches (`native_sim` for `lib/` and simulation changes, `bcc`
	 for BCC hardware changes) and run the twister scenario that covers it.
2. Run clang-tidy for hosted C++ changes that affect diagnostics or public
	 headers. Fix issues in the change or add a `NOLINT` comment with a justification.
	 Run the repository's clang-tidy workflow from the hosted-build-test skill;
	 do not treat a successful build or editor diagnostics as a substitute. If
	 the workflow cannot run, report the blocker and the checks that did run.
3. Check the diff for accidental changes to generated files, build output, or
	 submodules. The west workspace (`zephyr/`, `modules/`, `build/`) is not part of this repository.
4. Update `README.md`, `CONTEXT.md`, and the affected `docs/` plan when
	 user-visible behavior, configuration, build steps, hardware bring-up
	 behavior, or a planned decision changes.

## Skills

Based on [mattpocock's skills](https://github.com/mattpocock/skills), adapted for
VS Code Copilot and this project. Skills are organized under `.github/skills/`:

- [Hosted build and test](.github/skills/hosted-build-test/SKILL.md) —
	Windows/Linux hosted validation, clang-tidy, and clang-format.
- [File docs](.github/skills/file-docs/SKILL.md) — Doxygen comments and file headers
	for C++ headers and sources.

Use the repository's `SKILL.md` files as the source of truth for these
workflows.
