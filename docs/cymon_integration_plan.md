# cymon integration plan

Status: **implemented** (steps 1 to 10 below), not yet run on the board
(M0 cost of `Tick()`). `lib/cymon` points at cymon-lib 0.2 on `main`.
That version is ready for this integration: C++20, interrupt-safe `Tick()`,
UNIMOC naming and clang-tidy rules, and DSDL `cymon.*.0.1` that compiles
with Nunavut. The host side is the cymon admin tool
(<https://github.com/Tecnologic/cymon>). Its **Scope** page drives these
services, and its test suite runs a Python model of the same capture logic.

## Goal

The hot path can be recorded at the control-cycle rate, over the bus, with
no extra hardware. The tool chooses up to 8 signals and a trigger. The drive
records them into a RAM ring buffer, and the tool reads the capture back
page by page. This replaces FreeMASTER (`docs/system_plan.md`).

## What cymon-lib 0.2 provides

| Item | Where |
|---|---|
| `cymon::Device`: variable registry, capture, trigger, atomic state machine | `lib/cymon/include/cymon/device.hpp` |
| `Tick()`: no allocation, no locks, no function pointers, no libm; copies at most `CYMON_MAX_CHANNELS` floats | same |
| Service handlers `HandleSetupCapture()`, `HandleArmTrigger()`, `HandleReadSamples()`, `HandleGetBufferInfo()`, `GetVariable()` | same |
| DSDL root namespace `cymon` (5 services, `TriggerEvent`, 3 enums) | `lib/cymon/dsdl/cymon/` |
| Adapter example against the Nunavut C types, compiled in cymon-lib CI | `lib/cymon/docs/integration.md` |
| Plain CMake target `cymon` (alias `cymon::cymon`); without tests or extra flags as a subproject | `lib/cymon/CMakeLists.txt` |

cymon-lib builds clean with the UNIMOC warning set and `-Werror`, with the
UNIMOC `.clang-tidy` and with the UNIMOC `.clang-format`. Its tests run in
GoogleTest on the host in its own CI. They stay there: UNIMOC tests the
integration with ztest and pytest, as below.

## Steps

1. **Build** (`cmake/UnimocZephyr.cmake`). Make three changes:
   - Call `add_subdirectory(${UNIMOC_ROOT}/lib/cymon ...)` and set the
     `CYMON_MAX_*` cache variables from new Kconfig symbols:
     `CONFIG_UNIMOC_TRACE_BUFFER_BYTES` (default 8192),
     `CONFIG_UNIMOC_TRACE_VARS` (32) and `CONFIG_UNIMOC_TRACE_CHANNELS` (8).
   - Link `zephyr_interface`, `project_options` and
     `unimoc_strict_warnings` into `cymon`, and add `cymon` to `unimoc_lib`.
   - Remove `-*/unimoc/lib/cymon/*` from `scripts/codechecker_skip.txt`, and
     add `lib/cymon/(include/cymon|src/cymon)` to `HeaderFilterRegex`.

   Check with `twister_sca.sh` (native_sim and bcc).
2. **DSDL** (`lib/cyphal/CMakeLists.txt`): add
   `${UNIMOC_ROOT}/lib/cymon/dsdl/cymon` to `unimoc_dsdl_roots` and to the
   configure dependencies. The headers `cymon/*_0_1.h` are generated next
   to the existing ones.
3. **Service servers** (`lib/cyphal/node.*`). `CyphalNode` serves only its
   fixed standard services today. Add three things:
   - a `ServiceHandler` interface:
     `OnRequest(const Transfer&, std::span<std::byte> response) -> size`;
   - `AddServer(port_id, request_extent, ServiceHandler&)`, which also
     subscribes the request;
   - the added servers in `PublishPortList()`, which currently sends only
     `kServedServices`.

   Test it in `unimoc.lib.cyphal` with the in-memory bus.
4. **Port registers**: `uavcan.srv.cymon_{variables,buffer_info,setup,arm,read}.id`
   (natural16, persistent; defaults 120 to 124) and their read-only `.type`
   companions. Also add `uavcan.pub.cymon_trigger.id` and
   `uavcan.sub.cymon_trigger.id` (default 65535, disabled). Put them in a
   register provider next to the drive's port registers. A changed ID takes
   effect after a restart, like the other ports.
5. **Trace application** (`src/trace/trace_node.{hpp,cpp}`, Cyphal thread):
   - one `ServiceHandler` per cymon service, following
     `lib/cymon/docs/integration.md`;
   - on each `Spin()`, `TakeTriggerEvent()`, which publishes
     `cymon.TriggerEvent.0.1`;
   - when a `TriggerEvent` arrives on the subscribed trigger subject,
     `RequestTrigger()`.
6. **Signals**. The control state uses unit types (`unit::Current`, ...),
   but cymon needs `const float*`. Add a plain mirror,
   `struct TraceSignals { float i_d, i_q, i_d_ref, i_q_ref, u_d, u_q,
   electrical_angle, electrical_speed, dc_link_voltage, i_a, i_b, i_c; }`.
   The control cycle fills it with `.Value()` copies at its end, and every
   field is registered once at start-up with its SI unit. On `native_sim`,
   also add the plant truth (`unimoc.sim.Plant` fields).
7. **Sampling**: call `trace.Tick()` right after the mirror is filled, at
   the end of the control cycle (zero-latency IRQ, 16 kHz,
   `SetTickPeriod(62.5F)`). `Tick()` meets the zero-latency rules of
   `AGENTS.md`: no kernel calls, no locks, no function pointers, no libm.
   M0 has to confirm the cost (target < 1 µs at 170 MHz). The fallback is
   the observer's pended interrupt.
8. **Memory**: with the defaults above, `Device` uses about 8.9 KB of the
   G473's 128 KB SRAM. Put it in `.bss` (a static object), and in CCM SRAM
   if the M0 numbers ask for it.
9. **Tests**:
   - ztest scenario `unimoc.lib.cymon`: the library builds under the Zephyr
     toolchain, and one capture with a fake tick works.
   - pytest scenario in `tests/integration/cyphal_sim/`: against the
     running simulation, it lists the variables, captures `i_q` with an
     immediate trigger and checks the sample count and period.
10. **Docs**: README (Cyphal ports table and a "Traces" section), CONTEXT
    (glossary: trace, capture, trigger), `docs/zephyr_port_plan.md`
    (Phase 5), TODO.

## As implemented

- `CyphalNode::AddServer()` takes a `ServiceHandler` whose
  `OnRequest(const Transfer&, std::span<std::byte>)` returns
  `std::optional<std::size_t>`: `std::nullopt` sends no response (a request
  that does not deserialize). A taken, standard or invalid (> 511)
  service-ID is refused.
- One `trace::TraceNode` serves all five services and tells them apart by
  the port-ID. The persistent `.id` registers are settings
  (`NvmSettings::srv_cymon_*_id`, `pub_/sub_cymon_trigger_id`); the `.type`
  companions are in `DriveInfoRegisters`.
- `Tick()` runs in `ControlCycleHandler()` (`src/main.cpp`) right after
  `CurrentControlIsr::OnControlCycle()`, through `trace::OnControlCycle()`,
  on every cycle (also while tripped or in force-duty mode). The mirror
  `trace::TraceSignals` is filled from the controller state;
  `CurrentControlState::i_dq_last` holds the measured d/q current for it.
- The device is constructed in `.bss` (a `std::optional`) by the Cyphal
  thread before the services start: as a constant-initialized object its
  non-zero defaults would put about 8.9 KB into `.data` and flash. The
  control cycle samples it once it is published through an atomic pointer.
  On the BCC it disassembles to the copy loop plus `memset`/`memmove`, no
  other calls.
- The tick period is `dt_slow` of the controller, set at start and again
  after the drive applied new settings (another PWM frequency).
- Sim plant truth: `hardware/zephyr_sim` fills `sim::PlantTrace` at the end
  of every cycle; `CyphalPlatformTraceSignals()` registers it as `sim.*`
  (IDs from 12). The BCC registers nothing extra.

## Tool side

```sh
pip install -e <cymon>                       # once
cymon serve -i socketcan:vcan0 --dsdl unimoc/dsdl/unimoc --open      # simulation
cymon serve -i canable:/dev/ttyACM0 --dsdl unimoc/dsdl/unimoc --open # BCC, CANable 2.0
```

The BCC sends CAN FD frames at 1 Mbit/s without bit rate switch (SN65HVD230).
The CANable defaults of cymon match that. A full 8-channel capture of 1024
frames is 8192 floats, which is 128 `ReadSamples` responses of about 270
bytes each. At 1 Mbit/s that takes roughly 0.4 s of bus time.

## Open points

- `Tick()` cost on the board (M0) and its placement (control cycle or
  observer interrupt).
- Whether the trace signals should include the observer outputs, which are
  computed after the control cycle (one cycle late in the capture).
- Done: `lib/cymon` moved from the feature branch commit to cymon-lib `main`
  (the merged 0.2, which only differs in its CI workflow).
