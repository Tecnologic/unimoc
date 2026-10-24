# GUI contract (cypilot)

The operator GUI and CLI for UNIMOC nodes is
[cypilot](https://github.com/Tecnologic/cypilot) (`cypilot`, `cypilot-ctl`),
in its own repository since 2026-10-24. Its plan, decisions and phases are in
[docs/plan.md](https://github.com/Tecnologic/cypilot/blob/main/docs/plan.md)
there. This file keeps what the firmware owns: what cypilot relies on in
UNIMOC and the firmware work for it. A change here changes the bus contract;
cypilot follows a UNIMOC release (`DEFAULT_UNIMOC_REF` in `cypilot.dsdl`).

Section numbers of the cypilot plan: 2.1 license, 5.1 register description
file, 6 status, 7.1 readiness per node, 9 firmware support.

## 1. What each repository owns

| This repository (`unimoc`) | cypilot |
|---|---|
| DSDL namespace `dsdl/unimoc/` (cymon DSDL from cymon-lib) | No copy: downloads the DSDL of the pinned UNIMOC release at run time and compiles it |
| Registers, port defaults, vendor status bits, GetInfo names | Discovery, capabilities, the status model |
| Register description file: schema, generator, serving it | Reader of the file, editor hints |
| `native_sim` and its release artifact, `scripts/sim_cluster.sh` | Integration tests against the released `native_sim` |
| Its own pycyphal tests (`tests/integration/cyphal_sim`) for the firmware | GUI and CLI tests |


## 2. License

The firmware stays GPL-3.0-or-later. For cypilot: **Apache-2.0**.

- It matches cymon-lib (Apache-2.0), which the GUI reuses on the host side;
  pycyphal and pyqtgraph are MIT, PySide6 is LGPL-3.0 (used as an installed
  Python package; a PyInstaller bundle ships Qt as separate, replaceable
  libraries, so no one-file bundle).
- Apache-2.0 against MIT: both allow any use; Apache-2.0 adds an explicit
  patent grant and asks for notices of changes. MIT is shorter. Either works;
  Apache-2.0 keeps the two host-side projects on one license.
- For mostly AI-written code the copyright position is unclear in several
  jurisdictions. A permissive license makes that question matter little: users
  get the same rights either way. (This is no legal advice.)
- **The DSDL needs a decision too:** the files in `dsdl/unimoc/` carry
  `SPDX-License-Identifier: GPL-3.0-or-later`, and the GUI loads and compiles
  them. Proposal: relicense `dsdl/unimoc/` to Apache-2.0 (or MIT, like the
  OpenCyphal public regulated data types), so any tool, also a closed one, can
  speak the UNIMOC protocol. This needs the consent of every contributor to
  those files; today that is the repository owner.

## 3. Status in the heartbeat

The status line of each node is built from what the node publishes, in this
order of freshness:

| Source | Rate | Content |
|---|---|---|
| Heartbeat | 1 Hz | Health, mode, uptime, vendor status (below) |
| `unimoc.cart.DriveStatus` | 50 Hz | Engaged, power stage, limp-home, regen derated, overcurrent and overvoltage trips, temperatures, DC link |
| `unimoc.drive.Telemetry` | 20 Hz | Engaged, power stage, overcurrent trip, currents, voltages, speed, DC link |
| Registers, polled at about 1 Hz while the node is selected | — | `unimoc.can.bus_off_count` |
| `uavcan.diagnostic.Record` | event | Last warning or error in the row, full text in the log |

Vendor status code of the heartbeat (decided 2026-10-23; bits 0 to 3 exist
in `DriveNode::UpdateStatus()`, bits 4 to 7 are new):

| Bits | Meaning |
|---|---|
| 0 to 2 | Power stage state: 0 disabled, 1 enabled, 2 fault, 3 blocked (as in `unimoc.drive.Telemetry`) |
| 3 | Engaged |
| 4 | Trip latched (overcurrent or overvoltage), cleared by `STANDBY` |
| 5 | Emergency stop latched, cleared by a restart |
| 6 | Settings pending (`unimoc.settings.pending`) |
| 7 | Local control: the node engages by itself (e-bike, PR #29) |

With them the heartbeat alone gives the full status of any UNIMOC node, also
without `DriveStatus` or `Telemetry` enabled. A non-drive application (VDC,
BMS) that has no power stage sets bits 0 to 4 to zero and keeps 5 to 7 with the
same meaning where they apply.

States shown as colored chips: **Offline** (no heartbeat for 3 s),
**Disabled**, **Engaged**, **Blocked** (external enable missing, the E-stop
loop is open), **Tripped**, **Fault**, **E-stop latched**, **Limp-home**,
**Settings pending**, **Local control**, plus health and mode.

## 4. Register description file

A scan gives names, types and values, but no unit, range, default or help
text. The firmware owns that knowledge, so the firmware owns the file:

- **Source:** `ForEachRegister()` (`lib/settings/settings_registers.hpp`) and
  the register providers of the applications get a unit, the limits of the
  validation, when a write takes effect (`immediate`, i.e. live-tunable,
  `disengaged` or `restart`) and one line of help per register, next to the
  register itself; the default is the value of a fresh simulated node. Each application (drive, VDC, BMS,
  e-bike) describes its own registers in the same way.
- **Generator:** the simulation writes it (`--describe-registers=<file>`:
  schema version, firmware version and VCS revision, one entry per
  register). A twister pytest checks that every register on the bus has an
  entry and the other way round.
- **Delivery, step 1:** the CI attaches `registers.json` to the firmware
  artifacts and the release as `unimoc-registers-<version>.json`, next to
  `unimoc-bcc-*.elf` and `unimoc-native_sim-*.tar.gz`. The GUI picks it by the software version and
  VCS revision of `GetInfo`, from a local folder or the release.
- **Delivery, step 2:** the node serves it compressed through
  `uavcan.file.Read` (path `unimoc/registers.json.gz`, a few KB in flash) once
  the Cyphal file service exists (it comes with the firmware update). Then the
  description always matches the running firmware.
- **The schema** is documented in this repository (`docs/settings_plan.md`)
  and versioned; the GUI accepts the versions it knows and falls back to the
  bare scan otherwise.

## 5. Readiness per node

Decided 2026-10-23: every drive has its **own readiness subject**, so the GUI
(and the VDC) can enable each drive alone.

Implemented 2026-10-24 (README, "Readiness per drive and the heartbeat
status"):

- **Default:** `uavcan.sub.readiness.id` is 65534
  (`cyphal::kReadinessFollowsNodeId`): the drive subscribes
  `kSubReadinessBase + node-ID`, 1024 to 1151 for the CAN node-IDs 0 to 127,
  clear of the other default port-IDs (`cyphal::EffectiveReadinessId()`).
  Writing an ID fixes it, writing 65534 or a factory reset returns to the
  node-ID. A node-ID change moves the readiness subject with it after the
  restart, like every port; a plug-and-play drive subscribes once its ID is
  allocated.
- **The register keeps the written value (65534), not the effective ID**
  (changed from the first proposal): a settings profile copied from one drive
  to another must not carry a fixed subject, and the value reads back as it
  was written. A tool computes the subject as 1024 + `uavcan.node.id`; it also
  shows in the node's `uavcan.node.port.List`.
- **Collision check:** the check of the port registers includes the subject
  of the node-ID (for a node with a node-ID); 65534 is accepted for the
  readiness only.
- Settings saved before keep their readiness subject (106) until a factory
  reset.
- **GUI:** "Enable" publishes `ENGAGED` on the subject of each selected drive,
  "Enable all" on all of them; readiness costs one frame per drive and period
  (about 4 frames at 20 Hz for four drives).
- **Cart:** the VDC arms each drive on its subject (system plan section 6.2).
- **Setpoints** stay on shared subjects: only an engaged drive applies them,
  so per-node readiness already allows "this drive runs, the others are off".
  Different setpoints for several engaged drives at once (not the cart, which
  has one torque per wheel in `WheelCommand`) would need the same scheme for
  the setpoint subjects; open point 4 of the cypilot plan.
- **No two masters:** if any other node publishes on a drive's readiness or
  setpoint subject (seen in its `port.List`), the GUI shows it and does not
  publish there; it still offers STANDBY through an override with a
  confirmation.

## 6. Firmware work

Each its own PR with ztest and pytest coverage:

1. Done (F1, 2026-10-24): **vendor status bits 4 to 7** (section 3), README
   "Readiness per drive and the heartbeat status", pytest checks of every bit
   but 7 (the pytest build has no e-bike).
2. Done (F1, 2026-10-24): **readiness per node** (section 5), ztest
   `test_readiness_follows_the_node_id`, pytest on subject 1034.
3. Dropped (2026-10-24): a **GetInfo name convention**. The GetInfo name is
   the `uavcan.node.description` register, a role label the user sets
   (`NodeIdentity`, for example "unimoc.propulsion.left"), so it cannot also
   be a fixed product name. A tool recognizes a UNIMOC node by its registers
   (the `unimoc.` prefix) instead; the factory name stays "unimoc".
4. **`.type` registers** for every port of every application (the drive has
   them), so the GUI can decode every subject.
5. Done (F2, 2026-10-24): **register description file** (section 4):
   the description next to every register (`ForEachRegister()` entries,
   `RegisterProvider::Describe()`), `--describe-registers` of the
   simulation, ztests of the descriptions and their limits, the pytest
   against the registers on the bus, the CI and release asset
   `unimoc-registers-<version>.json`; schema in `docs/settings_plan.md`.
   Open (F3): the served file after the Cyphal file service.
6. Done (F2, 2026-10-24): **`scripts/sim_cluster.sh`**: starts N
   `native_sim` drives on `vcan0` with their own flash files, node-IDs (10,
   11, ...), unique-IDs and wheel indices (`--cart-wheel`). Not run on `vcan`
   yet (the development container has none); the multi-node checks of
   `TODO.md` ("Next") stay open. The GUI repository's tests use it with the
   released `native_sim`.
7. **License of `dsdl/unimoc/`** (section 2), if accepted.
