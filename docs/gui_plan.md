# UNIMOC GUI plan

Status: **plan** (2026-10-23). Nothing in this file is built. It replaces
the GUI items of `docs/software_in_loop_plan.md` (Phase 5) and of `TODO.md`
(SIL P3); the open decisions are in the last section.

## 1. Goal

cymon is the generic Cyphal debug tool: any register, any subject, any
service, by hand. UNIMOC also needs an operator tool for the everyday jobs on
one to many UNIMOC nodes, where the user does not write registers or publish
messages by hand:

- find all nodes of a bus and show at a glance what state each one is in
  (engaged, tripped, E-stop latched, limp-home, settings pending, bus-off);
- read, edit, compare, store and restore the settings of every node, with the
  register list scanned from the node, not taken from a fixed list;
- enable and disable one drive, a chosen set or all drives, with setpoints,
  and stop everything with one key;
- recover a node from an emergency stop, a trip or a fault by guided steps;
- plot live values of several nodes in one time base, and take triggered
  scope shots (cymon) on several nodes at once;
- work the same way against the `native_sim` instances and the real bus.

Scope of nodes: 1 to at least 4 wheel drives first, then the vehicle dynamics
controller (VDC), the BMS controller (`docs/system_plan.md`) and the e-bike
drive of PR #29 (`docs/ebike_plan.md` on that branch). Any other Cyphal node
still shows up with the generic functions (heartbeat, registers, log,
subjects).

Platforms: Linux and Windows. Transports: SocketCAN (`can0`, `vcan0`, CAN
FD), slcan (CANable and the `native_sim` pseudo terminals), the python-can
adapters on Windows, later Cyphal/UDP.

Not a goal: the GUI is no safety device. The hard-wired E-stop loop and the
mechanical brake stay the safety path (`docs/system_plan.md`, section 7). The
GUI does not replace cymon for protocol work, and it does not flash firmware
until the Cyphal firmware update exists (`TODO.md`).

## 2. Decisions proposed

| Topic | Proposal | Reason |
|---|---|---|
| Place | `tools/gui/` in this repository: Python package `unimoc_gui`, its own `pyproject.toml`, not part of the Zephyr build | The GUI follows the DSDL and the registers of this repository; one PR changes firmware and tool together. |
| Language and stack | Python 3.11+, pycyphal, PySide6 (Qt 6, LGPL), pyqtgraph, qasync | Same stack as the existing pycyphal tests and the SIL plan; pycyphal compiles the DSDL at run time, so a new message type needs no tool release; Qt runs the same on Linux and Windows. |
| Structure | A **core** without Qt (bus, discovery, registers, control, recorder, scope), a **CLI** `unimoc-ctl` and the **Qt UI** `unimoc-gui` on the same core | Quick jobs from the shell and from scripts, headless tests, and a UI that holds no protocol logic. |
| Threads | All Cyphal I/O on one asyncio loop in a bus thread; the UI only sends commands and receives snapshots | The UI never blocks the bus and a busy bus never freezes the UI (SIL plan, Phase 5). |
| Register list | Always scanned (`uavcan.register.List` until the empty name, then `Access` per name); no register name is hard-coded except the standard `uavcan.*` ones and the few the control functions need (section 5) | Requirement; firmware changes appear without a tool change. |
| Node functions | **Capabilities** found by discovery (ports, `.type` registers, register prefixes), not by node type | One drive with the cart registers and one without, the e-bike and an unknown node all get what they actually support. |
| Scope | The cymon protocol (`lib/cymon/dsdl/cymon`, services on the `uavcan.srv.cymon_*.id` registers) driven by the core, captures of several nodes joined through the shared trigger subject | cymon-lib is already in every UNIMOC node; no second trace protocol. |
| Tests | Core and CLI against the running simulation through the twister pytest harness, like `unimoc.app.cyphal_integration`; Qt smoke test offscreen in the same session | `AGENTS.md`: tests run under twister; the pytest harness is the allowed black-box exception. |

## 3. Architecture

```text
 unimoc-gui (PySide6, pyqtgraph)          unimoc-ctl (CLI)       pytest (twister)
        │  commands ▲ snapshots, events          │                       │
        └───────────┼────────────────────────────┴───────────────────────┘
                    │
   unimoc_gui.core  │  (no Qt; asyncio loop in the bus thread)
   ┌────────────────┴─────────────────────────────────────────────────────┐
   │ session    transport factory, local node, DSDL import path           │
   │ discovery  heartbeat, GetInfo, port.List, .type registers → NodeModel │
   │ registers  scan, read, write, verify, store, profiles (JSON), diff    │
   │ status     NodeStatus from heartbeat, DriveStatus, Telemetry, regs    │
   │ control    arming groups, keep-alive, setpoints, ramps, stop all,     │
   │            ExecuteCommand, recovery procedures                       │
   │ signals    any subscribed subject → named float channels with time    │
   │ recorder   ring buffer of all channels, CSV/JSON export, replay       │
   │ scope      cymon client, multi-node capture, alignment, export        │
   │ profiles   capability plugins: drive, cart, sim, ebike, vdc, bms      │
   └──────────────────────────────────────────────────────────────────────┘
                    │ pycyphal
   SocketCAN (vcan0, can0) │ slcan (pty, COM, socket://) │ python-can (PCAN, gs_usb, ...) │ later UDP
```

Repository layout (proposal):

```text
tools/gui/
  pyproject.toml            # dependencies, entry points unimoc-gui and unimoc-ctl
  README.md                 # install, transports, workflows
  unimoc_gui/
    core/                   # section 3 above; no Qt import
    profiles/               # capability plugins
    cli/                    # unimoc-ctl
    ui/                     # Qt windows, docks and widgets
  tests/                    # pytest, run by twister against native_sim
scripts/sim_cluster.sh      # starts N native_sim drives on vcan0 (section 8)
```

The DSDL comes from this repository (`dsdl/unimoc`, `lib/cymon/dsdl/cymon`)
and the west workspace (`modules/lib/public_regulated_data_types`). pycyphal
compiles it once into a cache directory. An installed GUI without the
repository gets a copy of the DSDL in its package at build time (open point
in section 11).

## 4. Discovery and the node model

Every node that publishes a heartbeat becomes a row. For each new node, or
one whose uptime went backwards (restart), the core runs:

1. `uavcan.node.GetInfo`: name, software and hardware version, unique ID,
   software CRC.
2. `uavcan.node.port.List` (published by the node every 10 s, also kept from
   the bus): subjects published and subscribed, services served.
3. Register scan (section 5) in the background, low priority.
4. Port types: the `uavcan.pub.*.type` and `uavcan.sub.*.type` registers name
   the DSDL type of each port-ID (`unimoc.drive.Telemetry.0.1` on 212). With
   them every published subject of the node can be decoded and offered for
   plotting without a list in the tool. A port without a `.type` register
   shows as "unknown type"; the user may assign one, saved per node name.
5. Capabilities from what was found:

| Capability | Found by | Enables |
|---|---|---|
| Registers | `uavcan.register.List` served | Register editor, profiles |
| Commands | `uavcan.node.ExecuteCommand` served | Restart, store, factory reset, E-stop |
| Armable drive | register `uavcan.sub.readiness.id` enabled | Arming groups, enable and disable |
| Setpoints | `uavcan.sub.{current,torque,speed,position,servo}_setpoint.id` enabled | Setpoint panel with the modes found |
| Cart drive | `uavcan.sub.wheel_command.id`, publishes `unimoc.cart.DriveStatus` | Wheel index, cart status, limp-home display |
| Scope | the five `uavcan.srv.cymon_*.id` registers enabled and served | Scope, multi-node capture |
| Simulation | registers `unimoc.sim.*` | Pause, advance, reset, virtual clock |
| Local control (e-bike, PR #29) | registers `unimoc.ebike.*` and the e-bike enabled | No engage button: the drive engages by itself; STANDBY and SLEEP stay |
| VDC, BMS | publishes `unimoc.cart.WheelCommand` or `unimoc.cart.BatteryStatus` | Cart overview, battery panel (when these nodes exist) |
| Log | publishes `uavcan.diagnostic.Record` | Log view with node filter |

The nodes show in a table and as a cart sketch when cart drives exist (wheel
index from `unimoc.cart.wheel`). Users can give a node a label (saved by unique
ID), so "FL drive" stays "FL drive" after a node-ID change.

## 5. Registers

- **Scan:** `List` with index 0, 1, ... until the empty name; then `Access`
  with an empty value to read each one's value, type, `mutable` and
  `persistent` flags. Names are shown as a tree on their dots
  (`unimoc.motor.limits.i_max`). About 130 registers per drive: one `List` and
  one `Access` each, at low priority, in about a second per node on a free bus.
- **Edit:** the editor follows the value type (`real32[1]` number, `natural8`
  integer or enum, `bit` check box, `string` text, arrays as lists). A write
  is checked against the answer: the drive answers a rejected write with the
  unchanged value (README, "Registers"), so the GUI shows "rejected" instead
  of a silent no-op. Live-tunable or not: the GUI reads
  `unimoc.settings.pending` after the write and shows "waits for disengage".
- **Store:** `ExecuteCommand STORE_PERSISTENT_STATES` per node or for all
  changed nodes; the GUI shows the `BAD_STATE` refusal while the power stage is
  enabled with its reason.
- **Profiles:** save all persistent registers of a node to JSON (with node
  name, software version and date), load it into the same or another node
  (diff first, then write, then verify, then store), compare two nodes or a
  node with a file. Copying one drive's settings to the other three drives is
  the main use. Port-ID and node-ID registers are skipped on load unless asked,
  so a profile does not create a port collision or a duplicate node-ID.
- **Search and filter:** by name, only changed, only non-default (once the
  defaults are known, see 5.1), only mutable.
- **Restart-needed hint:** port-ID and node-ID changes take effect after a
  restart; the GUI offers the restart after storing.

### 5.1 Register descriptions (optional firmware support)

A scan gives names, types and values, but no unit, range, default or help
text. Hard-coding them in the tool would bring back the fixed list. Proposal:
the firmware generates a compact description of its registers from
`ForEachRegister()` (`lib/settings/settings_registers.hpp`) at build time
(unit, minimum, maximum, default, live-tunable, one line of help) and serves
it as a file through `uavcan.file.Read` (`unimoc/registers.json.gz`, a few
KB in flash), which the firmware update needs anyway. The GUI reads it when
served and falls back to the bare scan. Until then the GUI shows units only
where the name says it (none).

## 6. Status

The status line of each node is built from what the node publishes, in this
order of freshness:

| Source | Rate | Content |
|---|---|---|
| Heartbeat | 1 Hz | Health, mode, uptime, vendor status: bits 0 to 2 power stage state, bit 3 engaged (`DriveNode::UpdateStatus()`) |
| `unimoc.cart.DriveStatus` | 50 Hz | Engaged, power stage, limp-home, regen derated, overcurrent and overvoltage trips, temperatures, DC link |
| `unimoc.drive.Telemetry` | 20 Hz | Engaged, power stage, overcurrent trip, currents, voltages, speed, DC link |
| Registers, polled at about 1 Hz while the node is selected | — | `unimoc.settings.pending`, `unimoc.can.bus_off_count` |
| `uavcan.diagnostic.Record` | event | Last warning or error in the row, full text in the log |

States shown as colored chips: **Offline** (no heartbeat for 3 s),
**Disabled**, **Engaged**, **Blocked** (external enable missing, the E-stop
loop is open), **Tripped**, **Fault**, **E-stop latched**, **Limp-home**,
**Settings pending**, **Local control** (e-bike), plus health and mode.

Gap in the firmware today: the E-stop latch only shows as health CAUTION, and
a latched trip shows as ADVISORY like a fault, so a 1 Hz heartbeat cannot tell
the user what to do. Proposal (small firmware change, section 9): use the free
bits of the vendor status, bit 4 trip latched (overcurrent or overvoltage),
bit 5 emergency stop latched, bit 6 settings pending, bit 7 local control.
Then the heartbeat alone gives the full status of any UNIMOC node, also
without `DriveStatus` or `Telemetry` enabled.

## 7. Control

### 7.1 Arming groups

Readiness (`reg.udral.service.common.Readiness`) is a subject, not addressed
to a node: every drive that subscribes the same subject-ID (default 106)
engages together. So the GUI arms **groups**: the drives that share one
readiness subject, read from their `uavcan.sub.readiness.id`.

- With the defaults all drives form one group: "Enable all" works without any
  setup; one drive cannot be enabled alone.
- To enable a single drive, the drives need their own readiness subjects. The
  GUI offers "Give each drive its own readiness subject": it writes free
  subject-IDs (checked against the port lists of all nodes), stores and
  restarts. Enabling a set of drives then publishes on their subjects. The
  same holds for the setpoint subjects.
- In the cart, the VDC will publish readiness and wheel commands. If any other
  node publishes on a group's readiness or setpoint subject (seen in its
  `port.List`), the GUI shows it and does not publish there (no two masters);
  it still offers STANDBY through an override with a confirmation.

### 7.2 Keep-alive and dead man

The drive disengages when readiness or setpoints are not refreshed within
`unimoc.control.command_timeout` (0.5 s). The GUI publishes readiness and
setpoints of every enabled group at 20 Hz from the bus thread. The UI thread
feeds a watchdog of the bus thread: if the UI stalls for longer than 0.3 s,
the bus thread stops the readiness publications, and the drives disengage by
their own timeout. Closing the GUI or losing the adapter has the same effect.

### 7.3 Enable, disable, stop

| Action | What the GUI does |
|---|---|
| Enable (group or selected) | Checks the preconditions (not blocked, no latched trip or E-stop, no foreign publisher, setpoint zero), then publishes `ENGAGED` and keeps it alive; shows the engaged bit coming back from each drive, or a timeout with the reason |
| Disable | Ramps the setpoint to zero (configurable ramp), then publishes `STANDBY` for a second, then stops publishing |
| Stop all (Esc key, always visible) | Publishes `SLEEP` on every readiness subject it knows and zero setpoints, at once, no ramp |
| Emergency stop (button with confirmation, or Shift+Esc) | `ExecuteCommand EMERGENCY_STOP` (65531) to every node that serves it; latched until a restart |

### 7.4 Setpoints

One panel per group, with the modes the drives subscribe (current d/q,
torque, speed, position, UDRAL servo). Inputs: number field, slider with
limits from the node's registers (`unimoc.motor.limits.i_max`, the speed range),
a ramp, and a step or a square wave of chosen amplitude and period for
tuning. The setpoint goes to zero on disable, on stop and when another group
is selected.

Cart: a "cart" panel publishes `unimoc.cart.WheelCommand` with one torque per
wheel (or one for all) and `unimoc.cart.PedalState` for limp-home tests, only
while no VDC or pedal unit is on the bus.

### 7.5 Recovery

A "Recover" button on a node that is not ready runs the procedure that fits
its state, step by step, with each step's result shown:

| State | Procedure |
|---|---|
| Tripped (overcurrent, overvoltage) or fault | Stop the keep-alive of its group, publish `STANDBY` (acknowledges the latched trip), wait for the trip and fault bits to clear, then "ready to enable" |
| E-stop latched (`EMERGENCY_STOP`) | Confirm, `ExecuteCommand RESTART`, wait for the heartbeat with a reset uptime, rescan the node, check that nothing is latched |
| Blocked | Cannot be cleared from the bus: "external enable missing, check the E-stop loop"; the chip clears by itself when the enable returns |
| Bus-off counted | Shows the count and the time of the last one; nothing to do on the node, it rejoins by itself |
| Offline | Shows the last known state and waits; offers a restart when the node comes back with an error |

"Recover all" runs the procedure on every node of a selection, in parallel,
and lists what is still not ready.

### 7.6 Simulation

When a node has the `unimoc.sim.*` registers: pause, run, advance N cycles,
reset, the virtual cycle count and period, and the simulation settings
(load torque, DC-link voltage) as a panel. Several simulated nodes are paused
and advanced together (one after the other; the cluster has no shared clock
yet, `TODO.md`, SIL P2).

## 8. Signals, plots, scope

### 8.1 Live plots

- Every decoded subject of every node is a source of named channels
  (`FL drive / Telemetry / i_q`), with the arrival time of the transfer as the
  time stamp (pycyphal reception time; one clock for all nodes).
- Plot windows with any number of channels from any nodes, shared or own Y
  axes, pause, cursor, auto scale. pyqtgraph draws the decimated view;
  the recorder keeps everything.
- The recorder keeps the last N minutes of all subscribed channels and of all
  register writes and commands the GUI sent (as markers). Export to CSV (one
  file per subject, wide format) and JSON (metadata: nodes, versions, register
  snapshot); replay of an export in the same plot windows.
- Rates are what the nodes publish (telemetry 20 Hz, drive status 50 Hz). The
  GUI can raise `unimoc.telemetry.rate` for a session and restores it.

### 8.2 Triggered scope shots (cymon)

For control-cycle resolution (16 kHz) the scope uses the cymon services of the
nodes (`docs/cymon_integration_plan.md`, `lib/cymon/dsdl/cymon`):

1. `GetVariableList` and `GetBufferInfo` per node: the variables (`i_q`,
   `sim.torque`, ...), the channel and sample limits, the tick period.
2. The user picks channels per node (at most `max_channels`, 8), the length,
   the sample period and the pre-trigger part, and one trigger: a variable of
   one node with rising, falling, hysteresis or immediate, and a level.
3. `SetupCapture` on every node, then `ArmTrigger`: the trigger node with the
   trigger condition; the other nodes armed so that the trigger node's
   `cymon.TriggerEvent` triggers them.
4. Poll `ReadSamples` with `max_frames = 0` until `COMPLETE`, then read the
   frames page by page at low priority, with a progress bar.
5. Show all nodes in one plot, each aligned on its own trigger sample, and
   export like the recorder.

Multi-node trigger: the nodes need the shared trigger subject
(`uavcan.pub.cymon_trigger.id` and `uavcan.sub.cymon_trigger.id` on one
subject-ID on every node, decided 2026-10-22). The GUI checks it and offers to
set it (store and restart, it is a port). The trigger reaches the other nodes
through the Cyphal thread (woken at least every 1 ms) and the bus, so their
trigger sample is late by about 1 to 2 ms; the GUI shows this as an
uncertainty band, and the value is measured in the simulation (section 10).
For a tighter alignment every node can trigger on its own copy of a shared
event (for example a setpoint step), which the GUI offers as "trigger each
node on its own signal".

Bus time: a full 8 by 1024 capture is about 0.4 s per node at 1 Mbit/s with
CAN FD frames (cymon plan); over classic slcan with 8-byte frames about eight
times as long. Several nodes are read one after the other.

Relation to the cymon host tool: the GUI implements the client side of the
same protocol in `core/scope.py`. If the cymon host package exposes its
capture client as a library with a stable API, the GUI uses it instead (open
point).

## 9. Firmware support

Small changes, each its own PR with ztest and pytest coverage:

1. Vendor status bits 4 to 7 (section 6): trip latched, E-stop latched,
   settings pending, local control. Documented in the README, "Commands".
2. GetInfo name convention per application: `org.tecnologic.unimoc.drive`,
   `.vdc`, `.bms`, `.ebike`, `.sim` suffix for the simulation. Shown in the
   GUI, also used to pick a profile icon; the capabilities still come from
   discovery.
3. `.type` registers for every port of every application (the drive has them),
   so the GUI can decode every subject.
4. Register descriptions as a file (section 5.1), after the Cyphal file
   service exists.
5. `scripts/sim_cluster.sh`: starts N `native_sim` drives on `vcan0` with
   their own flash files, node-IDs (10, 11, ...) and wheel indices; the
   multi-node simulation of `TODO.md` ("Next") is the precondition.

## 10. Transports and platforms

| Transport | Linux | Windows | Use |
|---|---|---|---|
| SocketCAN `vcan0` (CAN FD, MTU 64) | yes | no (WSL2 only) | Simulation cluster |
| SocketCAN `can0` (CAN FD adapter with a kernel driver, also CANable 2.0 candleLight) | yes | — | Bench and cart |
| slcan, serial port (CANable slcan firmware) | `/dev/ttyACM0` | `COM5` | Bench, classic CAN only (pycyphal with slcan) |
| slcan, simulation pseudo terminal | `/dev/pts/N` | — | Single simulated node (one tool per pty) |
| slcan over TCP (`socket://host:port`, pyserial URL) | yes | yes | Windows GUI against a simulation in WSL2, bridged with `socat` (to verify) |
| python-can `pcan`, `gs_usb`, `kvaser`, ... | yes | yes | CAN FD adapters on Windows (PCAN-USB FD has CAN FD under python-can) |
| Cyphal/UDP | later | later | When the simulation or the cart offers it |

The connection dialog lists the interfaces it finds (SocketCAN links, serial
ports) and remembers the last ones. Settings: local node-ID (default 127, the
tools' ID in this repository; a second tool on the bus needs another), CAN FD
or classic MTU, bit rates for the adapters that need them.

Windows and the simulation: `native_sim` runs only on Linux (WSL2 on
Windows). Options in order of preference: the GUI inside WSL2 (WSLg shows Qt
windows); the Windows GUI through slcan over TCP to a `socat` bridge in WSL2;
later Cyphal/UDP. Windows packages: `pip install` from `tools/gui` first, a
PyInstaller bundle later.

## 11. Tests

- **Core and CLI:** a pytest suite in `tools/gui/tests/`, run by a new
  twister scenario `unimoc.app.gui` (pytest harness, `app/testcase.yaml`,
  like `unimoc.app.cyphal_integration`) against the running simulation:
  discovery, the register scan against the register count of the settings,
  write, reject, store, profile round trip, enable and disable with the
  engaged bit, keep-alive timeout, stop all, the recovery procedures (trip
  through an overcurrent setting, E-stop and restart), the simulation panel,
  a cymon capture with an immediate and a level trigger. Skipped without the
  GUI dependencies, like the existing pytest scenario.
- **Qt smoke test:** with `QT_QPA_PLATFORM=offscreen` in the same session: the
  main window opens, connects, shows the node, a plot gets samples.
- **Multi-node:** once `scripts/sim_cluster.sh` exists, the same suite with
  four drives on `vcan0` (arming groups, own readiness subjects, multi-node
  capture and its trigger delay). Linux CI only; `vcan` needs the module and
  root once.
- `check_format.sh` gets a Python formatter and linter step for `tools/gui`
  (ruff, pinned), in the same way as clang-format for C++.

## 12. Phases

| Phase | Content | Done when |
|---|---|---|
| G0 | Package skeleton, transport factory, DSDL import, `unimoc-ctl nodes` and `unimoc-ctl regs <node>`; twister pytest scenario | The CLI lists the simulated drive and its registers in CI |
| G1 | Discovery, capabilities, status model, register scan, edit, store, profiles and diff in the core and the CLI | Profile copy between two simulated drives in a test |
| G2 | Qt main window: connection dialog, node table with status chips, register editor, log view | Offscreen smoke test; manual check on Linux and Windows |
| G3 | Control: arming groups, keep-alive and UI watchdog, enable, disable, stop all, E-stop, setpoint panel, recovery procedures, simulation panel | Engage, trip, recover and E-stop flows pass against the simulation |
| G4 | Live plots across nodes, recorder, CSV/JSON export and replay | Plot of four nodes from the cluster |
| G5 | Scope: cymon client, multi-node capture with the shared trigger, export | Capture of two simulated drives on one trigger, delay measured |
| G6 | Firmware support of section 9 (vendor bits, names, cluster script); multi-node CI | Four-drive suite green |
| G7 | Profiles for the cart (wheel panel, cart sketch), the e-bike (after PR #29), the VDC and the BMS as they appear | Per node type |
| G8 | Windows bundle, slcan over TCP to WSL2, README workflows | Install on a clean Windows machine |

G0 to G3 make the tool useful for the bench (one to four drives); G4 and G5
replace most cymon sessions for daily work.

## 13. Open decisions

1. **Place and name:** `tools/gui/` with `unimoc-gui` and `unimoc-ctl`, or a
   separate repository. Proposed: in this repository (section 2).
2. **Tests outside the Zephyr flow:** the plan runs every GUI test through a
   twister pytest scenario against the simulation. Pure logic tests of the core
   without a simulation would be plain pytest; `AGENTS.md` allows only the
   black-box pytest harness. Proposed: keep everything in the twister scenario
   and amend `AGENTS.md` only if that proves too slow.
3. **cymon host code:** reuse the capture client of the cymon host tool as a
   library, or own client code in `core/scope.py`. Depends on the cymon API.
4. **Vendor status bits** of section 6: accept the bit layout.
5. **Register descriptions** as a served file (section 5.1), or accept the
   bare scan without units and help.
6. **Readiness per drive:** keep one shared default subject (enable all by
   default, individual subjects set up by the GUI), or give each drive its
   own readiness subject by default (for example 106 + wheel index). The second
   changes the defaults of the firmware and the cart.
7. **License of the GUI:** GPL-3.0-or-later like the repository (PySide6 is
   LGPL, pyqtgraph MIT, pycyphal MIT).
