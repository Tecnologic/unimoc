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

## Known issues

- `control_mode_test` and `nvm_settings_test` are skipped: they still use the
  old enumerator names (`POSITION`, `PMSM`) and do not compile against the lib.
- The hardware layers are skeletons; BCC pins for CAN-FD, USB, LEDs and
  gate-driver enables are not yet in the devicetree.
- `cymon` needs C++23/CMake 4 and is not linked yet.
