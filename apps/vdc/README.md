# VDC application

The vehicle dynamics controller: the central node of the cart. It arms the
wheel drives, collects their status and sends the wheel command at 250 Hz.
It has no hot path; the Cyphal node runs in a Zephyr thread
(`hardware/zephyr_common/vdc_runtime.cpp`, `src/vdc/`, `lib/vehicle/`).

Today it is a skeleton: the torque request comes from a register, the
registers live in RAM (`README.md` of the repository, "Vehicle dynamics
controller").

## Boards

| Board | Build target | Status |
|---|---|---|
| **Zuse**: the VDC board, STM32H7 (`docs/zuse_plan.md`) | planned (`unimoc_zuse`) | Draft: interfaces, supply, display link |
| NUCLEO-H743ZI (`boards/nucleo_h743zi.*`) | `nucleo_h743zi` | Builds; only FDCAN1 is used |
| Simulation (`boards/native_sim_native_64.*`) | `native_sim/native/64` | ztests and the pycyphal integration test |

Around Zuse (`docs/zuse_plan.md`): the display Gutenberg (WT32-SC01 Plus
on RS-485, Cyphal/serial), the pedal unit Drais (its own node on the bus),
the steering angle sensor (AS5048A, PWM), the IMU ICM-42688-P, GNSS, the
lights and the E-stop loop sense.

## Build

From the west workspace root:

```sh
west build -b native_sim/native/64 unimoc/apps/vdc -d unimoc/apps/vdc/build/sim   # node-ID 1
west build -b nucleo_h743zi unimoc/apps/vdc -d unimoc/apps/vdc/build/h743
```

The twister scenarios are in `testcase.yaml` (`unimoc.app.vdc.*`).

## More

- The cart, the messages and the fault reactions: `docs/system_plan.md`.
- The board and its periphery: `docs/zuse_plan.md`.
