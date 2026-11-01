# 4850 Logic Board on the STM32H743VIT6

Status: 2026-11-01, proposal. Nothing here is drawn, built or verified on
hardware. The pin functions are taken from the STM32 HAL pin data that Zephyr
4.4 uses (`modules/hal/stm32/dts/st/h7/stm32h743vitx-pinctrl.dtsi`) and from
the Zephyr devicetree of the STM32H743; every entry marked "check" has to be
confirmed in the data sheet (DS12110) and the reference manual (RM0433)
before the schematic is frozen.

## 1. Why the MCU changes

The system plan picked an STM32G473/G474 (LQFP64, 512 KB) for the 4850 logic
board. That choice no longer holds:

- The 512 KB G4 parts (G473CE/RE, G474CE/RE, G483, G484) are not available or
  far too expensive.
- The application does not fit into 128 KB: the BCC image is 194 KB flash and
  79 KB RAM (v0.4.0, `west build -b bcc`; with WRN log level, no asserts and
  LTO still 178 KB). About 45 KB of it is Zephyr, the rest is the
  application (drive node, settings registers, Cyphal). A port back to modm
  would not close the gap.
- The H743 gives headroom for a faster current loop: at 400 MHz a half PWM
  period at 32 kHz (15.6 µs) is about 6250 cycles, enough to run the control
  cycle on every sample instead of every fourth (target 1500 cycles,
  `docs/hotpath_plan.md`).

Alternatives that were looked at: STM32H523 (two ADCs, cheap, LQFP48/64),
STM32C593 (three ADCs, but no FDCAN node in Zephyr 4.4), STM32G491 (three
ADCs, G4 price), STM32H723 (no package below LQFP100, single flash bank).

## 2. What the H743VIT6 offers

| Item | STM32H743VIT6 | Use |
|---|---|---|
| Core | Cortex-M7, 480 MHz (silicon rev. V; rev. Y 400 MHz), double-precision FPU, I/D cache, ITCM 64 KB, DTCM 128 KB | Hot path from ITCM/DTCM, plan with 400 MHz (check: VOS0 junction temperature limit for 480 MHz) |
| Flash | 2 MB, two banks of 1 MB, 128 KB sectors, 256 bit write | Image in bank 1; DFU slot and settings in bank 2 (an erase in bank 2 does not stall the code in bank 1) |
| RAM | AXI SRAM 512 KB, SRAM1..3 288 KB (D2), SRAM4 64 KB (D3) | Duty DMA buffers in D2 SRAM, non-cacheable MPU region |
| ADC | ADC1, ADC2 (D2), ADC3 (D3), 16 bit, injected group, JEXTSEL 10 = TIM8_TRGO2 as on the G4 | Three phase currents sampled at once, one per ADC |
| Timers | TIM1, TIM8 (CH1..CH3 + CHxN, BDTR, BKIN/BKIN2), TIM2..TIM5 | TIM1 phases, TIM8 the fourth half bridge |
| DMA | DMA1/DMA2 with DMAMUX1 (as on the G4), MDMA, BDMA | Duty DMA as on the G4. DMA1/2 cannot reach DTCM. |
| CAN | FDCAN1, FDCAN2 | FDCAN1 with the TCAN334 |
| Analog extras | COMP1, COMP2, DAC1 (2 channels) | Chopper comparator (section 5) |
| USB | OTG FS (PA11/PA12) | DFU later |
| Package | LQFP100, 14 x 14 mm | The logic board grows from LQFP64 (10 x 10 mm) |

Board-level points (check in DS12110): two VCAP pins with their capacitors,
LDO supply only (no SMPS on the H743), VREF+ on its own pin (filter or
reference for the 16 bit ADC), supply current well above 100 mA at full
clock (3.3 V regulator and heat).

## 3. Signals from the current 4850 logic

From `logic/pinning/unimoc_hardware_4850_pinning.ioc` (STM32F730R8, LQFP64)
and the sheet labels of `logic/` and `power/` in hardware_4850:

- Power board, per phase: `PwmH`, `PwmL`, `I` (INA240A1 output). Shared:
  `Vdc`, bridge temperature, `PWM_EN`. No phase voltages on the connector.
- Logic: each phase current goes through the GS8632 filters into a DC path
  and an AC path (`Ix_DC`, `Ix_AC`); motor temperature, `AIN_ACC`,
  `AIN_DCC`, Hall A/B/C, four LEDs, CAN (TCAN334), I2C (24LC04, dropped),
  USB with VBUS sense, SWD with SWO, HSE.
- New: the fourth half bridge (PWM pair, its current, a temperature of the
  resistor or field winding), a connector for an angle sensor (SPI).

## 4. Pin proposal

### 4.1 Analog

Each phase current is rank 1 of the injected sequence of its own ADC, so
`Ia_DC`, `Ib_DC` and `Ic_DC` are sampled at the same instant, as on the BCC.
ADC3 only reaches PC0..PC3 in LQFP100, so phase C and the DC link sit there.

| Signal | Pin | ADC input | Group |
|---|---|---|---|
| Ia_DC | PA6 | ADC1_INP3 | injected rank 1 |
| Ia_AC | PA7 | ADC1_INP7 | injected rank 2 |
| I_aux (fourth half bridge) | PC4 | ADC1_INP4 | injected rank 3 |
| Ib_DC | PB0 | ADC2_INP9 | injected rank 1 |
| Ib_AC | PB1 | ADC2_INP5 | injected rank 2 |
| Ic_DC | PC2_C | ADC3_INP0 | injected rank 1 |
| Ic_AC | PC3_C | ADC3_INP1 | injected rank 2 |
| Vdc | PC0 | ADC3_INP10 | injected rank 3 |
| Bridge temperature | PA0 | ADC1_INP16 | regular |
| Motor temperature | PA1 | ADC1_INP17 | regular |
| AIN_ACC | PA2 | ADC2_INP14 | regular |
| AIN_DCC | PA3 | ADC2_INP15 | regular |
| Aux temperature (resistor or field) | PC5 | ADC2_INP8 | regular |
| Spare | PC1 | ADC1/2/3_INP11 | for example a later phase voltage |
| Spare / DAC | PA4, PA5 | DAC1_OUT1/OUT2, ADC1/2_INP18/19 | DAC test output or comparator threshold |

Check: PC2_C/PC3_C are the direct ADC pads in LQFP100 (analog switch
`PC2SO`/`PC3SO` in SYSCFG_PMCR); the ADC3 trigger list contains the TIM1
trigger used by ADC1/2; the GS8632 output range against the ADC range.

### 4.2 Digital

| Function | Pins | Note |
|---|---|---|
| Phase PWM high CH1..CH3 | PE9, PE11, PE13 (TIM1_CH1..CH3) | All gate signals on port E, frees PA8..PA10 and PB13..PB15 |
| Phase PWM low CH1N..CH3N | PE8, PE10, PE12 (TIM1_CH1N..CH3N) | |
| Break (E-stop, `PWM_BREAK`) | PE15 (TIM1_BKIN) and PA8 (TIM8_BKIN2), same net | Stops the phases and the fourth half bridge |
| Fourth half bridge | PC7 (TIM8_CH2), PB14 (TIM8_CH2N) | TIM8 slave of TIM1 (reset and trigger by TRGO), same period and dead time |
| Gate driver enable `PWM_EN` | PE7 | GPIO output |
| Chopper comparator input | PB2 (COMP1_INP) | Second tap of the Vdc divider; threshold from DAC1 internally |
| CAN | PB8 (FDCAN1_RX), PB9 (FDCAN1_TX) | As on the F730 board; TCAN334 |
| CAN transceiver standby | PD2 | Only if the transceiver variant has the pin |
| USB DFU | PA11, PA12 (OTG_FS DM/DP), PA9 (OTG_FS_VBUS) | |
| SWD | PA13, PA14, SWO on PB3 | |
| Hall A/B/C | PD12, PD13, PD14 (TIM4_CH1..CH3) | GPIO today; TIM4 allows capture later |
| LEDs RUN, PWM, MODE, ERROR | PD8, PD9, PD10, PD11 | GPIO outputs |
| Angle sensor (later) | PC10 (SPI3_SCK), PC11 (MISO), PC12 (MOSI), PA15 (NSS) | Connector only |
| I2C (optional) | PB6 (I2C1_SCL), PB7 (I2C1_SDA) | The EEPROM is dropped; keep the pins free |
| HSE | PH0, PH1 | Crystal; CAN FD clock from the HSE |

The power board connector stays unchanged (section 2.1 of the system plan).
Only the MCU side of the traces moves.

## 5. Fourth half bridge on the H743

- **PWM:** the H743 timers have no CH4N, so TIM1 cannot drive a fourth
  complementary pair. TIM8 CH2/CH2N takes it, started and reset by the TIM1
  trigger output so that both timers count in step. Dead time and break
  come from the TIM8 BDTR, the E-stop reaches it through BKIN2 (PA8).
- **Chopper without the CPU:** COMP1 compares the second Vdc tap (PB2) with
  a DAC1 threshold. How the comparator output turns the chopper switch on
  in hardware (TIM8 OCREF_CLR with inverted output, or a break on the
  phases) is to be confirmed in RM0433; otherwise the chopper runs in the
  sample ISR as planned in section 5.2.1 of the system plan.
- **Measurement:** `I_aux` in the ADC1 injected sequence, the aux
  temperature in a regular sequence.

## 6. Firmware work

- New platform `hardware/zephyr_stm32h7/` (`CONFIG_UNIMOC_PLATFORM_*`), next
  to `hardware/zephyr_bcc/`. Above `hardware/hot_path_interface.hpp` and
  `hardware/power_stage_interface.hpp` nothing changes.
- Reused nearly as is: the TIM programming, `dead_time.hpp` (same BDTR
  coding, other timer clock), the break logic, the DMAMUX duty DMA.
- New: the H7 ADC (clock and boost, linear and offset calibration, 16 bit
  data or the 12 bit mode for shorter conversions), ADC3 in the D3 domain,
  the cache and MPU setup (non-cacheable D2 region for the DMA buffers),
  the hot path linked into ITCM/DTCM.
- Flash layout: image in bank 1; DFU slot and the settings partition (two
  128 KB sectors, ZMS with 32 byte write blocks: check) in bank 2.
- Board definition `boards/unimoc/drive4850/`, bring-up first on a
  NUCLEO-H743ZI(2) (in Zephyr), without power stage.
- With the control cycle on every sample, `docs/hotpath_plan.md` gets the
  new cadence as a configuration of the platform, not as a second hot path.

## 7. Bring-up on a NUCLEO-H743ZI

Started 2026-11-01 on a first-revision NUCLEO-H743ZI (silicon rev. Y), so
the platform runs before the logic board is drawn. The board is Zephyr's
`nucleo_h743zi`; `app/boards/nucleo_h743zi.overlay` and `.conf` add the
UNIMOC parts. Built, not run on the board yet.

- Clock: 400 MHz core (rev. Y limit), HCLK 200 MHz, TIM1 200 MHz; ADC
  kernel clock PLL2 P 120 MHz / 4 = 30 MHz; FDCAN1 from PLL2 Q (80 MHz).
- Platform `hardware/zephyr_stm32h7/` (binding `unimoc,foc-stm32h7`): the
  G4 driver ported to the H7 ADC (asynchronous clock, boost by silicon
  revision, linearity calibration, channel preselection `PCSEL`, 12 bit),
  three injected conversions per ADC, the duty DMA on DMA1 stream 0 through
  DMAMUX1 from a `__nocache` buffer in AXI SRAM, the unlocked M7 cycle
  counter. The hot path still runs from flash with the caches on; ITCM/DTCM
  placement and the control cycle on every sample are the next steps.
- Image: 191 KB flash, 77 KB RAM (bank 1); settings (ZMS, 2 x 128 KB) and
  the DFU slot in bank 2.

Nucleo pins (the Ethernet PHY, the LEDs and the ST-LINK UART take some of
the 4850 proposal):

| Signal | Pin | ADC input / function |
|---|---|---|
| PWM CH1..CH3, CH1N..CH3N | PE9, PE11, PE13, PE8, PE10, PE12 | TIM1 |
| Break (external enable) | PE15 | TIM1_BKIN, pull-down: tie to 3V3 to release |
| ia, va, bridge temperature | PA6, PF11, PF12 | ADC1 INP3, INP2, INP6 |
| ib, vb, motor temperature | PB1, PF13, PF14 | ADC2 INP5, INP2, INP6 |
| ic, vdc, vc | PC2_C, PC3_C, PF10 | ADC3 INP0, INP1, INP6 |
| Hall A, B, C | PD12, PD13, PD14 | GPIO with interrupt |
| CAN | PD0 (RX), PD1 (TX) | FDCAN1, to an external transceiver module |
| Log | PD8, PD9 | USART3, ST-LINK virtual COM port |

All analog inputs take 0 to 3.3 V only. Wired to the 4850 power board, the
INA240 outputs and the dividers need a check of their range first.

Steps on the board: (1) boot and log, settings saved and read back after a
reset; (2) PWM and dead time on the scope with PE15 high, break by pulling
PE15 low; (3) ADC samples with potentiometers, `unimoc.timing.*` with
`CONFIG_UNIMOC_HOT_PATH_TIMING=y`; (4) Cyphal over a CAN transceiver
module; (5) the 4850 power board through an adapter.

## 8. Open points

1. Logic board size in the RND 455-00370 enclosure with an LQFP100.
2. Silicon revision of the bought parts (rev. V for 480 MHz).
3. Placement of the fourth half bridge (logic or power board) and its
   connector.
4. Whether the AC current paths stay (they need the second injected rank).
5. Hardware overcurrent: today the firmware checks every sample; a
   comparator path to BKIN needs one comparator per phase (only COMP1/2 on
   chip, COMP2 shares port E with TIM1 in this proposal).
