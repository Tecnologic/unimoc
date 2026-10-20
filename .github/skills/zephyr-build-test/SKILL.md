---
name: zephyr-build-test
description: Build, test and statically check UNIMOC the Zephyr way. Use for west builds, twister (ztest) runs, clang-tidy through Zephyr's static code analysis (CodeChecker) and clang-format checks, or when deciding which validation command to run.
---

# Zephyr Build And Test

All UNIMOC code is built and tested with Zephyr tooling. There is no hosted
CMake build and no GoogleTest: every test is a ztest suite run by twister on
`native_sim`, and clang-tidy runs as Zephyr static code analysis
(`ZEPHYR_SCA_VARIANT=codechecker`). On Windows, run everything in WSL2.

## Workspace

Run the commands from the west workspace root (it contains `unimoc/`,
`zephyr/` and `modules/`). Setup is in `docs/zephyr_workflow.md`; checks:

```text
west --version
cmake --version
clang-tidy --version      # LLVM 18, as on the ubuntu-24.04 CI runner
clang-format --version    # exactly 22.1.1 (pip install clang-format==22.1.1); check_format.sh enforces it
CodeChecker version       # pip install codechecker
```

## Standard validation (what CI runs)

```text
unimoc/scripts/check_format.sh      # clang-format, all non-vendored C/C++ files
unimoc/scripts/twister_sca.sh       # all twister scenarios on native_sim and bcc, with clang-tidy
```

`twister_sca.sh` builds every scenario under `unimoc/` (the application and
all `tests/`), runs the ztest suites on `native_sim`, builds the `bcc` ones,
and analyses each build with clang-tidy using the repository `.clang-tidy`.
A clang-tidy finding in UNIMOC code fails that scenario's build; the report
is in the console (`--inline-logs`) and in
`twister-out/<platform>/.../sca/codechecker/`. Extra arguments go to twister,
for example `-p native_sim/native/64` or `-T unimoc/tests/lib/units`.

## Focused loops

```text
west twister -T unimoc/tests/lib/motor_control -p native_sim/native/64   # one suite, no SCA
west twister -T unimoc -p native_sim/native/64 -p bcc                    # everything, no SCA
west build -b native_sim/native/64 unimoc/app -d unimoc/app/build/sim
west build -b bcc unimoc/app -d unimoc/app/build/bcc
```

Use them while iterating; finish with the standard validation.

## Where tests live

- `tests/lib/<library>/`: ztest suites of `lib/<library>` (one twister
  scenario per library, sources in `src/`).
- `tests/sim/<name>/`: closed-loop and timing scenarios against the
  `native_sim` platform and the motor plant.
- `tests/common/unimoc_test.hpp`: float checks that ztest lacks
  (`UNIMOC_EXPECT_FLOAT_EQ`, `UNIMOC_EXPECT_NEAR`, `UNIMOC_ASSERT_NEAR`).
  Everything else uses the ztest macros (`zexpect_*`, `zassert_*`).

A new suite is a directory with `CMakeLists.txt`, `prj.conf`,
`testcase.yaml` and `src/`, copied from an existing one.

## clang-tidy rules

- The root `.clang-tidy` applies to `src/`, `hardware/`, `tests/` and `app/`
  sources and their headers. `lib/` headers are not checked yet.
- `hardware/zephyr_*` and `tests/` have a `.clang-tidy` that only disables
  checks Zephyr's macros cannot meet (registration objects, do-while,
  C casts, varargs). `src/startup/` allows its printf-style logging.
- Fix findings in the code. A `NOLINT` needs the check name and a reason in
  the same comment.

## clang-format

The formatting differs between clang-format versions, so the version is
pinned to 22.1.1 (the version Zephyr's CI uses). `check_format.sh` refuses
other versions; put the pip binary first on `PATH` or point `CLANG_FORMAT`
at it. Zephyr's Python requirements install an unpinned clang-format, which
is why CI installs the pinned one afterwards.

```text
unimoc/scripts/check_format.sh          # report (exit 1 when a file is not formatted)
unimoc/scripts/check_format.sh --fix    # format in place
```

## Firmware boundary

The simulation does not prove hardware behavior, PWM safety, startup behavior
or NVM persistence. Build the `bcc` target for BCC changes and say what was
not run on the board.

## Completion

Report the commands used, whether the formatting check and clang-tidy ran,
the twister result line, and any prerequisite that was unavailable.
