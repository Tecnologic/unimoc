# Settings Persistence Plan: From the modm-Era Blob to Zephyr Settings

> **Status (2026-10-20):** proposal, not started. Replaces the open item "Add a
> Zephyr flash backend (NVS or ZMS) for `SettingsStorage`" in `TODO.md`.

## Goal

Persist `NvmSettings` the Zephyr way: the `settings` subsystem on top of ZMS (or
NVS) in the board's `storage_partition`, with the flash simulator on
`native_sim`. Keep `lib/settings` portable and free of Zephyr headers, keep the
safety rules of the current store, and keep the hot path untouched.

## Current State

| Part | What it is | Origin |
|------|------------|--------|
| `NvmSettings` | One struct, 68 fields, `magic` and `version` header | shared |
| `SettingsCodec` | Hand-written little-endian image, fixed 326 bytes, strict field order | modm era (raw flash page) |
| `SettingsStorage` | Three function pointers: `Load(image)`, `Save(image)` of **one blob** | modm era |
| `SettingsStore` | `Load()`, validation, `Commit()` transactions, snapshots, factory fallback | portable, keep |
| `SettingsOperations` | `Update(fn)`, `SetNodeId`, `ApplyAdcCalibration`, `ResetToFactoryDefaults` | portable, keep |
| `RamSettingsStorage` | RAM image; settings are lost at reset on both targets | placeholder |

What is good and stays: validation before save, `HardwareCapabilities` as
read-only safety limits that no register can write, atomic commit of a whole
candidate, factory fallback on an invalid image.

What is modm-shaped: a single opaque image with a custom layout and no CRC, no
wear leveling, no power-fail handling of its own, and any layout change forces
a factory reset (losing the ADC calibration). All of that is what ZMS and the
settings subsystem already provide.

Board facts: the BCC has an 8 KB `storage_partition` at `0x1e000` (4 pages of
2 KB on the STM32G473), code partition `slot0` is 120 KB. `native_sim` already
has a `zephyr,sim-flash` with a `storage_partition`.

## Target Design

```text
Cyphal register service / shell / startup FSM      (threads)
        |
   SettingsOperations -> SettingsStore (validate, snapshot)   lib/settings, portable
        |
   SettingsStorage  (portable boundary, key/value instead of one blob)
        |
   hardware/zephyr_common/zephyr_settings_storage.{hpp,cpp}   only place with Zephyr headers
        |
   Zephyr settings subsystem  ->  ZMS  ->  flash driver  (BCC: STM32G4 flash,
                                                          native_sim: sim-flash, file backed)
```

Decisions:

1. **Zephyr settings subsystem, ZMS backend** (`CONFIG_SETTINGS`,
   `CONFIG_SETTINGS_ZMS`). ZMS is the recommended backend for new designs and
   handles CRC, wear leveling and recovery after power loss. NVS is the
   fallback if the STM32G4 flash driver or the 8 KB partition does not fit
   ZMS.
2. **The portable boundary stays**, `lib/settings` does not include Zephyr
   headers (`AGENTS.md`). Only the implementation behind it changes.
3. **Keys follow the Cyphal register names**, for example
   `unimoc/control/current/kp_d` for `unimoc.control.current.kp_d`. The register
   table needed for the Cyphal register service is then also the settings
   schema (name, type, getter/setter, limits). No second list to maintain.
4. **No persistence while the motor runs.** See the flash-stall risk below.

## Phases

### Phase 0: Measure and decide (host and build only)

- Build the BCC app with `CONFIG_SETTINGS`, `CONFIG_ZMS` and `CONFIG_FLASH`
  and record the flash and RAM cost against the 120 KB code partition.
- Confirm ZMS works on the 4 x 2 KB partition (ZMS needs at least 2 sectors and
  keeps one free for garbage collection), otherwise use NVS or grow the
  partition and move `slot0` accordingly.
- Decide the blob-or-per-key question below with the numbers.

### Phase 1: Zephyr-backed storage behind the existing API

Smallest step that is "the Zephyr way" for storage, with no change in
`lib/settings`:

- `hardware/zephyr_common/zephyr_settings_storage.{hpp,cpp}` implements the
  existing `SettingsStorage` callbacks with one key, `unimoc/image`, holding
  the current 326-byte image (`settings_save_one`, `settings_load_subtree_direct`).
- `settings_subsys_init()` runs once during startup before `SettingsStore::Load()`.
- Replace `RamSettingsStorage` on `bcc` and `native_sim`
  (`CONFIG_UNIMOC_PLATFORM_*` selects the source, no `#ifdef`). Keep the RAM
  storage as the test double in `tests/lib/settings`.
- A mutex around `Load`/`Save` (settings calls block in thread context; the
  store itself has no lock, today there is only one writer).
- Tests, new scenario `tests/sim/settings_persistence` on `native_sim`
  (ztest, flash simulator): save, re-initialize the store and settings
  subsystem, load returns `kSuccess` with the same values; a corrupted entry
  falls back to `kFactoryDefaults`; a full partition returns `kStorageError`
  and leaves the running settings unchanged.
- `native_sim` option: a file-backed flash (`--flash=`) so a restarted
  simulation keeps its settings.

Acceptance: `twister_sca.sh` and `check_format.sh` pass; the settings survive a
simulated restart; no Zephyr include appears under `lib/` or `src/`.

### Phase 2: Per-key storage (only if Phase 0 shows it pays off)

Move from one blob to one key per field:

| | One blob (Phase 1) | One key per field |
|---|---|---|
| Atomic commit of several fields | yes, single write | no, needs a transaction marker |
| Add a field in a new release | layout version bump, **factory reset, calibration lost** | old keys stay valid, new key takes its default |
| Flash wear | whole 326 B per change | only changed fields |
| Cyphal register write | rewrites everything | one entry |
| Code | exists (codec) | name table, `h_set`/`h_commit` handler |

If chosen: `settings_register` handler for `unimoc/`; `h_set` fills a candidate
`NvmSettings`, `h_commit` runs `Validate()` once and falls back to the factory
profile on failure; `Commit()` writes only changed keys, then a generation
counter key last, and `h_commit` ignores a candidate whose counter does not
match the keys read (power loss during a multi-field write). `SettingsStorage`
becomes a key/value interface; the codec is only kept for export/import.

### Phase 3: Wire-up and cleanup

- `SettingsOperations` callers (Cyphal register service, shell, startup FSM)
  get `kStorageBusy` as a status when the power stage is enabled.
- Shell command `settings` (show, reset, save) once the shell exists.
- Delete `RamSettingsStorage` from the production targets; update
  `docs/zephyr_port_plan.md`, `docs/zephyr_workflow.md`, `README.md`, `CONTEXT.md`
  (new term: settings key), and tick the `TODO.md` items.

## Risks

| Risk | Mitigation |
|------|------------|
| **Flash erase and write stall instruction fetch.** The STM32G4 flash is single-bank for code and storage here (`storage_partition` sits in the same bank as `slot0`), and the zero-latency ISR runs from flash. A 2 KB page erase takes milliseconds, so a ZMS garbage collection during a write can stall the control cycle. | Persist only with the power stage disabled (`PowerStageState::kDisabled`), return busy otherwise. Check the dual-bank option and RAM-resident hot path in `docs/hotpath_plan.md` before ever relaxing this. |
| A power loss during a write | ZMS guarantees entry atomicity; Phase 2 adds a generation key for multi-field writes; factory fallback stays. |
| Flash budget (120 KB code partition) | Phase 0 measurement, NVS instead of ZMS, or `slot0` and `storage` repartitioned. |
| Layout change loses calibration | Phase 2, or an explicit migration in `SettingsCodec::Decode` per version. |
| Factory fallback hides storage faults | Keep the distinct `SettingsStatus` values and log the reason at startup. |
| Hardware behavior is not verified in the simulation | The STM32G4 flash driver, erase timing and ZMS recovery need a board run; document as an assumption until then. |

## Open Questions for the Decision

1. Phase 1 only, or go straight to per-key storage because the Cyphal register
   service needs the name table anyway?
2. Is a settings write while the motor runs ever required (for example live
   tuning from the GUI)? If yes, the hot path must run from RAM first.
3. Should the identity (`node_id`, name) and the ADC calibration live in their
   own keys even in Phase 1, so a factory reset of the tuning does not erase
   the calibration?
