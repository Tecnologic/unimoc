# Drive application

The motor drive of UNIMOC: field-oriented control in the zero-latency hot
path, everything else (Cyphal node, settings, startup aid, identification,
traces) in Zephyr threads. In the cart it is a wheel drive (and later the
pedal generator); on its own it runs from the bus, the accelerator input or
the e-bike control.

## Boards

| Board | Build target | Status |
|---|---|---|
| **Jacobi**: 36 V / 45 A drive, STM32H743VIT6 (`boards/unimoc/jacobi/`) | `unimoc_jacobi` | Builds; layout rev 0.3, not built yet |
| NUCLEO-H743ZI: bring-up of the STM32H7 platform (`boards/nucleo_h743zi.*`) | `nucleo_h743zi` | Runs on the bench: timing, Cyphal, settings, firmware update |
| Simulation with the motor plant (`boards/native_sim_native_64.*`) | `native_sim/native/64` | Tests and pycyphal integration tests |

Later drive boards (Barlow, Dolivo; `docs/system_plan.md`, section 5.0)
build this application too.

## Build

From the west workspace root:

```sh
west build -b native_sim/native/64 unimoc/apps/drive -d unimoc/apps/drive/build/sim
west build --sysbuild -b unimoc_jacobi unimoc/apps/drive -d unimoc/apps/drive/build/jacobi   # with MCUboot
west build --sysbuild -b nucleo_h743zi unimoc/apps/drive -d unimoc/apps/drive/build/h743
```

Variants: `boards/variants/` (low-side shunts, the 12 V bench profile of the
Nucleo adapter), `CONFIG_UNIMOC_EBIKE=y` for the e-bike control. The twister
scenarios are in `testcase.yaml` (`unimoc.app.drive.*`).

## More

- Features, registers and the bus interface: `README.md` of the repository.
- Hot path: `docs/hotpath_plan.md`; Jacobi: `docs/drive4850_h7_plan.md`.
- Build, flash, bootloader and bench: `docs/zephyr_workflow.md`.
