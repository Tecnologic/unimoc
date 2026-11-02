# Hardware Bring-up: Nucleo Adapter and Flipsky VESC (STM32F405)

Status: 2026-11-02, proposal. Nothing here is built or verified on hardware.
The VESC facts come from the VESC firmware's hardware configuration
([vedderb/bldc](https://github.com/vedderb/bldc), `hwconf/hw.h`,
`hwconf/flipsky/hw_75_100.h` and `.c`, `motor/mcpwm_foc.c`, read
2026-11-02). Every entry marked "check" has to be measured on the board or
confirmed in the schematic before power is applied.

## 1. Situation and order

The G4 is no longer a hardware path (decision 12 in `TODO.md`): the BCC
boards at hand carry a 128 KB STM32G473CB, too small for the firmware
(194 KB), and suitable 512 KB G4 parts cannot be bought. The BCC stays a
build and SCA target and the reference of the G4 hot path, but nothing runs
on it. The hardware that is at hand:

| Hardware | MCU | Power stage | Current sense | CAN |
|---|---|---|---|---|
| NUCLEO-H743ZI (rev. Y) | STM32H743ZI, 400 MHz | none; 4850 power board through an adapter (section 2) | INA240A1 inline, three phases | FDCAN, external transceiver module |
| Flipsky FSESC 4.20 | STM32F405RG, 168 MHz | DRV8302, 60 V class (check) | two low-side shunts (check) | bxCAN, classic only |
| Flipsky FS75100 (75100V201) | STM32F405RGT6, 168 MHz | 3 x EG3112, 6 x MDP10N027 (100 V) | three low-side shunts, 0.5 mΩ, gain 20 | bxCAN, classic only |

Order:

1. **Nucleo without power** (`docs/drive4850_h7_plan.md` section 7, steps 1
   to 4): boot and log, PWM and break on the scope, ADC, Cyphal.
2. **Adapter PCB to the 4850 power board** (section 2). Order it at once:
   the board-house lead time runs in parallel with steps 1 and 3.
3. **F4 platform on the FSESC 4.20** (sections 3 to 6) while the adapter is
   being made. The DRV8302 has its own VDS overcurrent protection, and the
   stage is small: the safer board for the first current loop.
4. **4850 on the Nucleo through the adapter**: the target hardware of the
   cart, H7 hot path against the real INA240 stage.
5. **FS75100**: second F4 board, three low-side shunts.

## 2. Adapter: NUCLEO-H743ZI to the 4850 power board

The adapter carries the signals of the power-board connector
(`docs/system_plan.md` section 2.1: per phase `PwmH`, `PwmL`, `I`; shared
`Vdc`, bridge temperature, `PWM_EN`) to the Nucleo pins of
`app/boards/nucleo_h743zi.overlay`. The connector pinout comes from
`power/` in hardware_4850; the table maps by net name.

| 4850 net | Nucleo pin | Function | Adapter circuit |
|---|---|---|---|
| PwmH A, B, C | PE9, PE11, PE13 | TIM1_CH1..CH3 | series resistor (check the LM5109B input threshold against 3.3 V logic) |
| PwmL A, B, C | PE8, PE10, PE12 | TIM1_CH1N..CH3N | as above; pull-downs so the gates stay off while the MCU is in reset |
| I A | PA6 | ADC1_INP3 | divider or op-amp to 0 to 3.3 V, RC anti-alias filter |
| I B | PB1 | ADC2_INP5 | as above |
| I C | PC2_C | ADC3_INP0 | as above |
| Vdc | PC3_C | ADC3_INP1 | the power board divider plus a clamp; check the full-scale voltage |
| Bridge temperature | PA0 or PF12 (overlay: PF12, ADC1_INP6) | ADC1 | NTC divider as on the 4850 logic |
| PWM_EN | free GPIO (check PE7 on the Nucleo connectors), or the `PWM_BREAK` net | gate driver enable | pull-down: the stage is off by default |
| E-stop / break | PE15 | TIM1_BKIN, `break-gpios` | jumper or switch to 3V3, pull-down on the board |
| Hall A, B, C | PD12, PD13, PD14 | GPIO | pull-ups to 3.3 V, RC filter |
| GND, 3V3/5V | Nucleo GND, 5V | supply of the INA240 and the gate driver logic | check what the power board expects from the logic board |

Points to settle before the layout:

- **INA240 output range.** On the 4850 logic the INA240A1 (gain 20) feeds
  the GS8632 filters. Their reference and supply define the output swing;
  with a 5 V supply and a 2.5 V reference the output reaches 0 to 5 V and
  needs a divider (or the GS8632 stage reproduced on the adapter). The H7
  ADC takes 0 to 3.3 V only.
- **Phase voltages.** The power board has none on its connector. The
  identification (`docs/motor_identification_plan.md`, I3) needs them; the
  adapter can add three dividers from the phase outputs if the connector or
  a test point gives access (check).
- **`PWM_EN`.** The Nucleo overlay has no enable GPIO yet. Add an
  `enable-gpios` to `foc0` when the pin is chosen, or tie `PWM_EN` to the
  break net for the first tests.
- **Current limit for the first power-on:** laboratory supply with current
  limit, no battery.

## 3. Differences of the VESC boards to the hot path

### 3.1 Low-side shunts

The BCC, the sim and the H7 platform measure the phase current with inline
amplifiers (`kCurrentSenseIsLowSide = false`,
`kCurrentSamplesArePwmWindowed = false` in `hardware_interface_config.hpp`).
The hot path samples at every peak and valley of the center-aligned counter
and drops the phase that switches closest to the sample
(`docs/hotpath_plan.md`, "Sampling window").

A low-side shunt carries the phase current only while the low switch of
that phase conducts, around the counter valley (all low switches on, V0).
The VESC firmware itself samples at V0 only, unless a board has inline
shunts (`HW_HAS_PHASE_SHUNTS`, "allows sampling in V0 and V7"). For UNIMOC:

- **One sample per PWM period**, at the valley. At 32 kHz PWM that is
  32 kHz sampling instead of 64 kHz; the control cycle becomes two samples
  instead of four with the same 16 kHz control rate, or four samples at
  8 kHz. The samples per control cycle become a platform constant, as the
  H7 plan already asks for the control cycle on every sample
  (`docs/drive4850_h7_plan.md` section 6). Not a second hot path.
- **Phase choice inverts.** With low-side shunts the phase with the
  highest duty has the shortest low-side on-time, so the sample drops that
  phase and rebuilds it from the other two (three shunts, FS75100). With
  two shunts (FSESC 4.20) there is no choice: the measured phases need a
  minimum low-side on-time, which caps the modulation (the VESC limits the
  duty to 0.95 for this reason; check the value against the shunt
  amplifier settling time).
- **Windowed samples.** The flags `kCurrentSenseIsLowSide = true` and
  `kCurrentSamplesArePwmWindowed = true` exist already; the F4 platform sets
  them, and the sample plan (`SampleAtPeak()`, `AppliedTicks()` of the G4
  and H7 platforms) gets the low-side rule.
- **Overcurrent per sample** works the same, at half the rate. The DRV8302
  adds a VDS trip in hardware (FSESC 4.20); the FS75100 has no gate driver
  protection (EG3112) and relies on the firmware check alone.
- **Simulation.** The sim inverter gets a low-side shunt mode (current
  visible only while the low switch conducts), so the sample plan is tested
  in `tests/sim/` before it runs on a board.

### 3.2 STM32F405 instead of G4/H7

| Item | STM32F405RG | Consequence |
|---|---|---|
| Core | Cortex-M4F 168 MHz, single-precision FPU, no cache | Budget per half period at 32 kHz: about 2600 cycles, as the BCC (2656 at 170 MHz). The control cycle stays on every fourth half period (every second sample) |
| Flash | 1 MB, sectors 4 x 16 KB, 1 x 64 KB, 7 x 128 KB, one bank | The image fits (191 KB H7, 194 KB BCC). Settings in two 16 KB sectors (ZMS/NVS); no dual-bank update: the DFU slot needs a bootloader that copies (MCUboot swap or overwrite) |
| RAM | 128 KB SRAM + 64 KB CCM (no DMA access to CCM) | The 77 to 79 KB RAM fit in SRAM; the hot-path data may go to CCM, the DMA buffers must not |
| Timers | TIM1, TIM8 with CH1..CH4, CH1N..CH3N, BDTR, BKIN; no TRGO2, no CC5/CC6 | The G4/H7 scheme with two trigger compares (OC4REF before the peak, OC6REF before the valley) does not exist. One sample per period needs only one: TIM1 CC4 before the valley, or the update event at the underflow (repetition counter 1) |
| ADC | ADC1..ADC3, 12 bit, injected group, triple simultaneous injected mode, no calibration | Three currents at the same instant (FS75100: IN10, IN11, IN12 on ADC1, ADC2, ADC3 as in the VESC); no linearity or offset calibration in hardware, the offset is measured at startup as today |
| DMA | DMA1/DMA2 with fixed stream/channel requests, no DMAMUX | The duty DMA (entries 1 to 3 by DMA) is ported to a fixed TIM1 request stream of DMA2 |
| CAN | bxCAN (CAN1, CAN2), classic CAN only | Section 4 |
| Zephyr | `st,stm32f405` well supported; driver `st,stm32-bxcan` | No custom board files beyond the devicetree and the pin control |

New platform `hardware/zephyr_stm32f4/` (`CONFIG_UNIMOC_PLATFORM_STM32F4`,
default if `SOC_SERIES_STM32F4X`), binding `unimoc,foc-stm32f4`, built on
the H7 port: `stm32_dead_time.hpp` (same BDTR coding), the break logic, the
Hall inputs. Above `hardware/hot_path_interface.hpp` and
`hardware/power_stage_interface.hpp` nothing changes.

## 4. Classic CAN

The firmware supports classic CAN already: `ZephyrCanMedia` falls back to
`CAN_MODE_NORMAL` with an 8-byte MTU when the controller has no FD
(`hardware/zephyr_common/zephyr_can_media.cpp`), libcanard splits the
transfers into frames, and the simulation runs Cyphal over classic CAN on
slcan. What changes is the bus:

- **A classic controller cannot share a bus with CAN FD frames.** The bxCAN
  sees an FD frame as a form error and destroys it with an error frame. A
  VESC node therefore runs on a classic bus: all nodes, the USB adapter and
  pycyphal at 1 Mbit/s classic (the FDCAN nodes in `CAN_MODE_NORMAL`, i.e.
  `use_fd` off; a register or Kconfig switch for that is needed on the H7).
- **Multi-frame transfers.** "Every message fits in one CAN FD frame" no
  longer means one frame: a transfer above 7 bytes takes ceil((n + 2) / 7)
  frames (2 bytes transfer CRC). Bus load at 1 Mbit/s classic is in
  `docs/system_plan.md` section 6.3: about 30 % at 250 Hz, too high at
  500 Hz. A bench with one or two VESCs is fine; the cart stays on CAN FD
  with H7 drives.
- **No VESC protocol.** The VESC CAN commands are not implemented; a VESC
  with UNIMOC firmware is a Cyphal node like the others.

## 5. Board definitions

| Board | Directory | From |
|---|---|---|
| FSESC 4.20 | `boards/flipsky/fsesc_4_20/` (`fsesc_4_20`) | The VESC 4.10/4.12 hardware configuration (check its location in `hwconf/` of vedderb/bldc; the FSESC is a Flipsky derivative, check the board against it: shunt value, amplifier gain, voltage divider, NTC) |
| FS75100 | `boards/flipsky/fs75100/` (`fs75100`) | `hwconf/flipsky/hw_75_100.h` and `.c` (75100V201; the V2 board has its own `hw_75_100_V2.*`) |

FS75100 from the VESC configuration (check on the board):

| Item | Value |
|---|---|
| PWM | TIM1, high side PA8, PA9, PA10, low side PB13, PB14, PB15 |
| Dead time | 660 ns (`HW_DEAD_TIME_NSEC`) |
| Gate enable | none (EG3112, no enable or fault pin in the configuration) |
| Currents | ADC1 IN10, ADC2 IN11, ADC3 IN12 (PC0, PC1, PC2), 0.5 mΩ, gain 20, three low-side ("battery side") shunts. Range about ±165 A at 1.65 V midpoint (check the amplifier reference) |
| Phase voltages | IN0, IN1, IN2 (PA0, PA1, PA2) |
| Vin | IN13 (PC3), divider 39 kΩ / 1 kΩ (about 132 V full scale at 3.3 V) |
| Temperatures | MOSFET NTC IN3 (PA3), motor NTC IN14 (PC4) |
| Hall | PC6, PC7, PC8 (TIM3 for an encoder) |
| CAN | CAN1, PB8 (RX), PB9 (TX) by the VESC default (`hwconf/hw.h`; not overridden for the 75100) |
| LEDs | green PB5, red PB7 |
| UART | USART3 PB10/PB11 (also I2C2), UART4 PC10/PC11 |
| Limits (VESC) | 120 A phase, 160 A absolute, 6 to 120 V input range, FET temperature 110 °C |

The VESC firmware triggers the regular ADC group from TIM2 CC2 (TIM2 reset
by the TIM1 update); UNIMOC uses the injected group triggered by TIM1, as on
the other platforms.

## 6. Bring-up on the FSESC 4.20

1. **Back up the original firmware** over SWD (`st-flash read` or
   `pyocd`, full 1 MB including the VESC bootloader), and check the readout
   protection (RDP level 0 expected).
2. **Board files**: devicetree, pin control, HSE (check the crystal), RTT
   log, settings partition; boot and log, settings saved and read back.
3. **No power stage voltage**, logic from the supply of the SWD probe or a
   low DC-link voltage with the gate enable off: PWM and dead time on the
   scope, the break, the DRV8302 enable and fault input.
4. **ADC**: offsets with the stage enabled and no motor, Vin against a
   multimeter, NTCs.
5. **Cyphal over classic CAN** with a USB CAN adapter in classic mode and
   pycyphal/yakut at 1 Mbit/s (heartbeat, registers, `uavcan.diagnostic.Record`).
6. **Current loop** on a current-limited laboratory supply with a small
   motor: forced angle, then the identification (I0 to I2), then Hall or
   the observer.

## 7. Firmware work, in order

1. Sim: low-side shunt mode in the inverter, sample plan with the low-side
   rule and the samples per control cycle as a platform constant; tests in
   `tests/sim/`.
2. `hardware/zephyr_stm32f4/` and the `unimoc,foc-stm32f4` binding: TIM1
   with CC4 or update trigger, triple simultaneous injected ADC, duty DMA on
   DMA2, break.
3. `boards/flipsky/fsesc_4_20/`, a twister build scenario next to `bcc` and
   `nucleo_h743zi`; `twister_sca.sh` builds it.
4. Classic-only bus option for the FDCAN platforms (register or Kconfig),
   and pycyphal tests over classic slcan (they run on classic already).
5. `boards/flipsky/fs75100/` with the three-shunt plan.

## 8. Open points

1. Location and contents of the VESC 4.10/4.12 configuration in the current
   vedderb/bldc tree, and how far the FSESC 4.20 deviates from it.
2. The 4850 connector pinout and the INA240 output range (adapter).
3. Whether the F4 stays a bench and hobby target or becomes a cart option
   (classic bus load, section 4).
4. Firmware update on the single-bank F405 (MCUboot or USB DFU from the ROM
   bootloader only).
5. Licensing: the VESC firmware is GPL-3.0; only facts (pin numbers,
   component values) are taken from it, no code.
