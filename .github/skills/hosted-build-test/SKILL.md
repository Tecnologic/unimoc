---
name: hosted-build-test
description: Validate UNIMOC hosted code on Windows or Linux. Use for CMake/Ninja configure, build, GoogleTest, clang-tidy, and clang-format checks, or when deciding which hosted validation command to run.
---

# Hosted Build And Test

Use this skill for the portable library and hosted GoogleTest targets. The
workflow is host-independent in intent and works on Windows and Linux; only
the shell syntax and tool installation differ.

## Required tools

The following commands must be available on `PATH`:

- CMake 4.x
- Ninja
- A C++20 host compiler
- GoogleTest dependencies resolved by the CMake project

For static analysis, also require `clang++` and `clang-tidy`. For formatting,
require `clang-format`. On Windows, install LLVM and expose these commands to
the current VS Code terminal. On Linux, use the distribution packages or the
toolchain already used by CI.

Check the environment before configuring:

```text
cmake --version
ninja --version
clang-tidy --version
clang-format --version
```

The compiler and analysis tools are only required for the checks being run.

## Standard hosted workflow

Run the repository workflow preset from the project root:

```text
cmake --workflow --preset "Hosted configure, build and test"
```

This configures the `Hosted build with tests` preset, builds it, and runs the
`Hosted Test` preset with failure output.

## Hosted workflow with clang-tidy

Run the static-analysis workflow when changing C++ diagnostics, public
headers, control algorithms, observers, system types, or unit wrappers:

```text
cmake --workflow --preset "Hosted configure, build and test + clang tidy"
```

The preset uses `clang++` and enables the repository's clang-tidy integration.
If configuration fails because `clang++` or `clang-tidy` is not found, fix the
host environment before changing CMake settings.

## Focused validation

For a short feedback loop, configure and build the hosted target once, then
filter the tests with a focused regular expression:

```text
cmake --preset "Hosted build with tests"
cmake --build --preset "Hosted build with tests"
ctest --preset "Hosted Test" -R "<test-regex>" --output-on-failure
```

Use the full `Hosted Test` preset when the change affects shared behavior or
when no focused test covers the changed path.

## clang-format

Format only changed C++ files. First check formatting without modifying files:

```text
clang-format --dry-run --Werror <changed-cpp-files>
```

To apply formatting after the user agrees or the task requires it:

```text
clang-format -i <changed-cpp-files>
```

On PowerShell, obtain changed files with:

```powershell
git diff --name-only --diff-filter=ACMR | Where-Object { $_ -match '\.(c|cc|cpp|h|hh|hpp)$' } | ForEach-Object { clang-format --dry-run --Werror -- $_ }
```

On Linux or another POSIX shell, use:

```sh
git diff --name-only --diff-filter=ACMR -- '*.c' '*.cc' '*.cpp' '*.h' '*.hh' '*.hpp' | xargs -r clang-format --dry-run --Werror
```

## Firmware boundary

This skill validates hosted code only. For Zephyr firmware or simulation
changes, build the target the change touches from the west workspace root,
after confirming the Zephyr SDK and west setup in `docs/zephyr_workflow.md`:

```text
west build -b native_sim/native/64 unimoc/app
west build -b bcc unimoc/app
```

Hosted tests do not prove hardware
behavior, PWM safety, startup behavior, or NVM persistence.

## Completion

Report the exact preset or focused test command used, whether formatting was
checked, whether clang-tidy ran, and any host-specific prerequisite that was
unavailable. Do not stage or commit changes automatically.