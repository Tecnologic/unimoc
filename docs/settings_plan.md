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
  132 persistent registers is `ForEachRegister()` in
  `lib/settings/settings_registers.hpp`; adding a register is one line there
  (with its description, below) plus the field in `NvmSettings`. The same
  table drives the Cyphal register service and the register description
  file.
- **Value encoding.** Units and floats are little-endian IEEE floats,
  `uint8_t` and 8 bit enums one byte, `uint16_t` port-IDs two bytes, the node name its characters plus a
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

## Concurrency

- **Save gate held for the whole save.** `SettingsSaveGate` brackets a save:
  `TryBegin()` before the first write, `End()` after the last. The platform
  gate (`PowerStageSaveGate()`, `src/settings_save_gate.hpp`) calls
  `PowerStageInterface::HoldOffEnable()` and then requires
  `PowerStageState::kDisabled`. `Enable()` and the hold-off share one atomic
  flag, so `Enable()` fails (returns false, the outputs stay off) while flash
  is written, and a save cannot start while an `Enable()` runs. A caller that
  wants the stage enabled retries after the save, like after a missing
  external enable. A save with nothing to write does not take the gate.
- **One lock per store.** `SettingsStore` takes its `SettingsLock` around
  every public operation (`Load`, `GetSnapshot` and all `SettingsOperations`),
  so the Cyphal register service, the shell and the startup FSM may share one
  store. `Update()` runs its operation under the lock; the operation must not
  call back into the store (the lock is not recursive). The Zephyr platforms
  provide one `k_mutex` (`ZephyrSettingsStorage::Lock()`, through
  `kSettings.GetSettingsLock()`); the default lock does nothing for single
  threaded host tests. All of this is thread context only, never the hot path.

## Tests

| Scenario | Covers |
|----------|--------|
| `unimoc.lib.settings` (`settings_registers_test`, `settings_store_test`) | register table (77 unique names, round trip of every register, encoding, sizes), live update without flash access, write only changed registers, erase at factory value, gate (held once per save, not released when refused), lock around every operation, partial and failed saves, undecodable and out-of-range entries, unknown keys, empty node name |
| `unimoc.sim.settings_persistence` (native_sim) | the real settings subsystem and ZMS on the flash simulator: save and reload through a second store, the gate with the real power stage (enabled and fault block, `Enable()` held off during a save), erase at factory value, empty node name, undecodable entry healed by a save, all 77 registers rewritten 40 times (garbage collection and capacity of the partition) |

## Not verified on hardware

- ZMS on the STM32G4 flash: erase time, garbage collection and recovery after a
  power loss during a write. The BCC partition is 16 KB (8 pages of 2 KB) so
  that all registers plus a spare sector fit; this is an estimate to check on
  the board.
- **Flash writes stall instruction fetch** in the same bank. The BCC carries
  an STM32G473CET6 (512 KB, dual bank with option bit DBANK = 1, the factory
  default). The code partition (`0x00000`, 240 KB) and the `storage_partition`
  (`0x3c000`, the end of bank 1) share bank 1, so a save stalls the
  zero-latency control cycle. The gate holds the stage disabled for the whole
  save (`Enable()` fails meanwhile, see "Concurrency"); the stall itself is
  still to be measured on the board. Bank 2 (`0x40000`, 256 KB) is kept free
  for the image of a firmware update over Cyphal, which can be written while
  bank 1 runs.
- Flash use (BCC app, Zephyr v4.4.0, size optimized): about 100 KB of the
  240 KB code partition, of which the settings subsystem, ZMS and the flash
  driver take about 6 KB.

## Cyphal register service

Implemented (2026-10-21): `src/drive/settings_register_provider.cpp` exposes
every register of `ForEachRegister()` over `uavcan.register.List/Access`.
Reads take the live settings, writes go through `Update()` and its
validation; a rejected write answers with the unchanged value.
`ExecuteCommand` 65530 (STORE_PERSISTENT_STATES) runs `SaveToFlash()`
(`BAD_STATE` when the gate refuses), 65532 (FACTORY_RESET) resets and saves.
The shell command planned here was dropped with the shell. The Cyphal ports
(`uavcan.sub.*.id`, `uavcan.pub.telemetry.id`), the telemetry rate and the
command timeout are persistent registers too. `Validate()` rejects two
enabled ports on one subject-ID or two services on one service-ID
(`HasPortIdCollision()`); the publication and subscription of the cymon
trigger may share one subject. GetInfo takes the hardware and
software versions from the firmware (`hardware_interface_config.hpp`,
`app/VERSION`), not from the settings.

## Register description file

Implemented (2026-10-24, F2 of `docs/gui_plan.md`). Each line of
`ForEachRegister()` is a `RegisterEntry`: the name and a
`cyphal::RegisterDescription` (unit, minimum, maximum, effect, help), made
with `Setting()` (applied while disengaged), `Tunable()` (live tuning while
engaged: `DriveNode` copies exactly these, `CopyTunable()`) or
`AfterRestart()` (node-ID, name, port-IDs). An empty unit is taken from the
unit type of the field (`UnitSymbol()`); plain floats and composite gains name
theirs. The limits are those of `Validate()` that hold for the register
alone; ztest `test_a_value_beyond_each_limit_is_rejected` checks that a
value just outside each limit is rejected, and
`test_every_register_is_described` that every register has help and every
dimensioned one a unit. The other providers (drive info, startup aid, timing,
simulation) describe theirs through `RegisterProvider::Describe()`.

`zephyr.exe --describe-registers=<file>` (native_sim) writes the file after
the start and exits; `scripts/package_firmware.sh` adds it to the native_sim
artifact and the release as `unimoc-registers-<version>.json`. The pytest
`test_registers_json_describes_every_register_on_the_bus` compares it with
the running drive: the same names in the order of `uavcan.register.List`,
the same value types, sizes and flags.

Schema version 1:

```json
{
  "schema": 1,
  "firmware": {"name": "unimoc", "version": "0.2.0", "vcs_revision": "<16 hex digits>", "target": "native_sim"},
  "registers": [
    {"name": "unimoc.motor.limits.i_max", "type": "real32", "size": 1, "default": [15],
     "mutable": true, "persistent": true, "unit": "A", "min": 0,
     "applies": "disengaged", "help": "Largest motor current (stator vector), up to the hardware limit"}
  ]
}
```

- `type`: the field of `uavcan.register.Value` (`string`, `unstructured`,
  `bit`, `natural8/16/32`, `real32`, `empty`); `size` its number of elements
  (bytes of a string); `default` the value of a fresh simulated node (a
  string, an array of numbers or booleans). The BCC profile differs in the
  board values (node-ID 0, the Hall sensors not fused, no speed weight of the flux
  observer; `hardware_interface.cpp` of each platform).
- `unit`, `min`, `max`: absent when none. `applies`: only for mutable
  registers. `help`: one line.
- A tool accepts the schema versions it knows and falls back to the bare
  register scan otherwise. A change of the layout raises the version
  (`kSchemaVersion` in `hardware/zephyr_sim/register_file.cpp`).

## Open items

1. Register writes that change the controller are applied by restarting the
   runtime while the drive is disengaged. Done 2026-10-22: live tuning while
   engaged for the gains (`DriveNode::TuneLive()`: current PI gains through
   `CurrentControlIsr::SetCurrentGains()`, speed and position gains through
   `OuterLoop::SetGains()`, telemetry rate, command timeout); every other
   register still waits for the disengage, reported by a diagnostic record
   and `unimoc.settings.pending`.

Done: holding off `PowerStage::Enable()` during a save and serializing the
store (see "Concurrency").
