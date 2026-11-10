# Cyphal hardware tests (manual)

pycyphal tests of a **real drive board** on a CAN FD bus: the NUCLEO-H743ZI with
the drive application today, Jacobi later. They are **not run by CI**: there is no
`testcase.yaml`, twister never finds them, and without `--hardware` every test is
skipped. They share one board and run in file order; the last ones change the
board's state.

What they check (`test_cyphal_hw.py`): plug-and-play, GetInfo, heartbeat, the port
list, the registers (list, write, read back, restore), the publish rates, the cymon
trace capture, the emergency stop, persistence of the node-ID and of the settings
across a restart, the firmware update with MCUboot (options below), the engage with the loss of the master, and the factory reset.

## Bench setup

**Adapter.** A CAN FD adapter that shows up as a SocketCAN interface (`gs_usb`). The
bench uses a MKS CANable V2.0 (STM32G431) with
[CANnectivity](https://github.com/CANnectivity/cannectivity) (USB id `1209:ca01`).
The factory slcan firmware of the CANable (`16d0:117e`, a `ttyACM` port) is classic
CAN only and does not work: the node sends CAN FD frames. mainline candleLight_fw
does not support the STM32G431 boards.

```sh
sudo ip link set can0 up type can bitrate 1000000 dbitrate 1000000 fd on
```

The setting is gone after a replug of the adapter. If the adapter stops sending
(`No buffer space available`), unplug and replug it and set it up again.

**Transceiver on the Nucleo.** A TJA1051 module on FDCAN1 (overlay
`apps/drive/boards/nucleo_h743zi.overlay`): module TXD to PD1 (CN9, D66), RXD to PD0
(CN9, D67), 5 V and GND, 120 ohm termination at both ends of the bus. Crossed TXD
and RXD give a controller that sits in "synchronizing" (`FDCAN_PSR.ACT`) and never
sends.

**Analog inputs.** The bare Nucleo has no current amplifiers: the three phase
current inputs float and latch an overcurrent trip. The per-sample check trips when
a raw count leaves the window around the zero count (2048) of the 60 A limit:
1055 to 3041 counts, 0.85 V to 2.45 V. Feed the three inputs at mid scale, about
1.65 V, for example from one 10 kohm potentiometer between 3V3 and GND:

| Signal | MCU pin | Nucleo header |
|---|---|---|
| `ia` | PA6 | D12 |
| `ib` | PB1 | A3 |
| `ic` | PC2_C | A4 |
| `vdc` | PC3_C | A2: leave open or tie to GND (a high level reads as a DC-link voltage) |

The raw counts can be read from the ADC injected data registers over OpenOCD
(`ADC1` `0x40022080`, `ADC2` `0x40022180`, `ADC3` `0x58026080`, JDR1 of each).

**Break input.** PE15 is pulled down: without a jumper to 3V3 the power stage stays
*blocked* (heartbeat status 3), and the settings cannot be saved (the save gate needs
the state *disabled*, so `STORE_PERSISTENT_STATES` returns BAD_STATE and the
allocated node-ID is not kept). Put the jumper only with **no power board attached**:
with the break released, an engage enables the TIM1 outputs (PE8 to PE13).

`clean_drive` (the tests that save or engage) checks all of this and fails with the
vendor status it saw.

## Run

From the west workspace root, with the Python environment of `scripts/requirements-cyphal-tools.txt`
(pycyphal, python-can):

```sh
python -m pytest unimoc/tests/integration/cyphal_hw --hardware -v
```

The DSDL is compiled into `<tmp>/unimoc_pycyphal` on the first run (about a minute).
A drive without a node-ID gets one from the centralized allocator of the test client
(local node-ID 127). Options:

| Option | Effect |
|---|---|
| `--hardware` | required, otherwise all tests are skipped |
| `--can-iface can0` | SocketCAN interface (default `can0`) |
| `--allow-engage` | also run the engage test: enables the power stage outputs for a few seconds, torque setpoint 0 |
| `--update-image FILE` | runs the firmware update tests: refusals (wrong board, corrupt, not newer), then the download, the swap and the confirmation. **Changes the firmware**: the board runs `FILE` afterwards (reflash to go back). `FILE` is an update image (`scripts/make_firmware_image.py`, `docs/zephyr_workflow.md`) of this board with a **higher version** than the running firmware |
| `--current-image FILE` | the update image of the running firmware: checks the refusal "not newer" |
| `--unconfirmed-image FILE` | an image newer than `--update-image`, built with `-Ddrive_CONFIG_UNIMOC_UPDATE_CONFIRM_DELAY_MS=3600000`: checks that a reset swaps it back (runs after the other update tests; the board ends on the version of `--update-image`) |
| `--no-bootloader` | the board runs a build without MCUboot (no `--sysbuild`): expects the refusal of `BEGIN_SOFTWARE_UPDATE` |
| `--allow-factory-reset` | also run FACTORY_RESET: erases the settings and the stored node-ID (the test allocates again and checks that the new ID is kept) |
| `--allow-flash-bank` | the board has a firmware bank: skip the test that expects `BEGIN_SOFTWARE_UPDATE` to be refused |

The tests restore the registers they change (`kp_d`, `pole_pairs`). The factory reset
test leaves the board at factory settings with a freshly allocated node-ID.

## Not covered

The control loop on a motor (no motor, no power stage on the bench; `estimated_torque`
shows only the ADC offsets), a trip while engaged (needs a current input driven out of
the window), the commissioning sequence (it enables the outputs once the break is
released), and the update of a board that runs a motor (the update tests run disengaged).
