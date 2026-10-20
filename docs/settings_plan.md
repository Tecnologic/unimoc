# Settings Persistence: Per-Register Keys on Zephyr Settings

> **Status (2026-10-20):** implemented on `native_sim` and built for the BCC. Not
> run on the board: see "Not verified on hardware".

## Goal

Persist `NvmSettings` the Zephyr way, one entry per Cyphal register, with live
RAM tuning and an explicit, gated save to flash. `lib/settings` stays portable
and free of Zephyr headers.

## Design

```text
Cyphal register service / shell / startup FSM                 (threads)
        |  Update(fn), SetNodeId(), ...         live, RAM only, validated
        |  SaveToFlash()                        changed registers only
   SettingsOperations -> SettingsStore                        lib/settings
        |  live_ (RAM)   stored_ (shadow of flash)   SettingsSaveGate
   SettingsStorage  (key/value: LoadAll, Write, Erase)        lib/settings
        |
   ZephyrSettingsStorage                                      hardware/zephyr_common
        |
   Zephyr settings subsystem -> ZMS -> flash
   (BCC: 16 KB storage_partition, native_sim: flash simulator)
```

- **One key per register.** The key is the Cyphal register name
  (`cyphal::reg`), the Zephyr entry is `unimoc/<register name>`. The table of
  71 persistent registers is `ForEachRegister()` in
  `lib/settings/settings_registers.hpp`; adding a register is one line there
  plus the field in `NvmSettings`. The same table will drive the Cyphal
  register service.
- **Value encoding.** Units and floats are little-endian IEEE floats,
  `uint8_t` and 8 bit enums one byte, the node name its characters plus a
  terminator. No value is empty, because `settings_save_one()` deletes an
  entry on an empty value.
- **Only differences from the factory profile are stored.** A register that is
  not stored has its factory value. Adding a register in a new firmware does
  not invalidate stored settings, and a register set back to its factory value
  is erased on save.
- **Live RAM settings.** `SettingsOperations::Update()` and the typed setters
  validate against the hardware capabilities and change the RAM copy at once.
  Flash is not touched. This is the live tuning path.
- **`SaveToFlash()`** compares the live copy with the shadow of the flash
  content and writes or erases only the registers that differ. Registers saved
  before an error stay saved, a retry continues with the rest.
  `UnsavedCount()` tells how many registers a save would touch.
- **Save gate.** Saving is refused with `kPersistBlocked` unless the
  `SettingsSaveGate` allows it. The application passes
  `PowerStageSaveGate(kPowerStage)` (`src/settings_save_gate.hpp`): only
  `PowerStageState::kDisabled`. A latched fault or a missing external enable
  also blocks, until it is cleared. The gate is checked before every single
  write. A default-constructed gate never allows saving.
- **Load.** Stored registers are decoded over the factory profile. An entry
  that cannot be decoded, or a result that fails validation, leaves the factory
  profile active (`kInvalidImage`, `kInvalidSettings`, `kOutOfRange`);
  `main()` logs a warning and boots with the factory settings. The next save
  erases the undecodable entry. Storage errors stop the boot.
- **Startup calibration** (`ApplyAdcCalibration`) now commits to the live
  settings only. Saving the result is an explicit `SaveToFlash()`.
- Removed: the 326 byte image codec, the `magic` and `version` header (a layout
  version no longer exists, a missing key means "factory value"), and the RAM
  storage as a production backend. `tests/common/ram_settings_storage.hpp`
  remains as the host test double.

## Tests

| Scenario | Covers |
|----------|--------|
| `unimoc.lib.settings` (`settings_registers_test`, `settings_store_test`) | register table (71 unique names, round trip of every register, encoding, sizes), live update without flash access, write only changed registers, erase at factory value, gate, partial and failed saves, undecodable and out-of-range entries, unknown keys, empty node name |
| `unimoc.sim.settings_persistence` (native_sim) | the real settings subsystem and ZMS on the flash simulator: save and reload through a second store, the gate with the real power stage (enabled and fault block), erase at factory value, empty node name, undecodable entry healed by a save, all 71 registers rewritten 40 times (garbage collection and capacity of the partition) |

## Not verified on hardware

- ZMS on the STM32G4 flash: erase time, garbage collection and recovery after a
  power loss during a write. The BCC partition was widened from 8 KB to 16 KB
  (`0x1e000`, 8 pages of 2 KB) so that all registers plus a spare sector fit;
  this is an estimate to check on the board.
- **Flash writes stall instruction fetch** on the single-bank STM32G4 while the
  zero-latency control cycle runs from flash. The gate only guarantees the stage
  is disabled when the save starts; nothing yet prevents `Enable()` while a
  write is in progress. A write takes milliseconds, so enabling the stage must
  be held off during a save (open item).
- The settings subsystem, ZMS and flash driver cost on the 120 KB code
  partition (the BCC app builds; the size has not been compared).

## Open items

1. Hold off `PowerStage::Enable()` while a save is running (a lock in the save
   path, or a platform enable lock).
2. Serialize the settings thread: `SettingsStore` is not thread safe, it needs
   one owner thread or a lock once Cyphal and the shell call it.
3. Cyphal register service and a shell command (`settings show|save|reset`)
   built on `ForEachRegister()`.
4. Persistent registers for the hardware and software version are not stored
   (they are compile-time values); check that GetInfo reads them from the
   firmware, not from the settings.
