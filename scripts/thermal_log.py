#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Logs the thermal state of a drive for a heat run (docs/motor_thermal_plan.md,
# section 6) as CSV on stdout, for scripts/thermal_fit.py:
#
#   . unimoc/scripts/sim_yakut_env.sh            # or the UAVCAN__* variables of a real bus
#   unimoc/scripts/thermal_log.py 10 > run.csv   # drive node-ID 10, one row per second
#   unimoc/scripts/thermal_fit.py run.csv --current current --winding rs --resistance 0.15 --ambient 22
#
# Columns: time [s since the start], current [A RMS of the last 0.1 s window],
# winding and stator [degC, the drive's model], rs [degC from the stator
# resistance; empty without an estimate], then each --extra register. The
# drive must carry current at low speed for rs (unimoc.motor.thermal.rs_*).
# Needs pycyphal (scripts/requirements-cyphal-tools.txt) and CYPHAL_PATH with
# the public regulated data types (sim_yakut_env.sh sets both). Stops on Ctrl-C.
from __future__ import annotations

import argparse
import asyncio
import math
import sys

import pycyphal
import pycyphal.application
import uavcan.node
import uavcan.register

REGISTERS = {
    "current": "unimoc.motor.temperature.current",
    "winding": "unimoc.motor.temperature.winding",
    "stator": "unimoc.motor.temperature.stator",
    "rs": "unimoc.motor.temperature.rs",
}


def number(value: uavcan.register.Value_1_0) -> float | None:
    """The first element of a numeric register value; None when empty or NaN."""
    for kind in ("real32", "real64", "natural8", "natural16", "natural32", "integer32", "bit"):
        field = getattr(value, kind, None)
        if field is not None and len(field.value) > 0:
            result = float(field.value[0])
            return None if math.isnan(result) else result
    return None


async def log(server: int, period: float, extra: list[str], samples: int) -> int:
    node = pycyphal.application.make_node(uavcan.node.GetInfo_1_0.Response(name="unimoc.thermal_log"))
    node.start()
    client = node.make_client(uavcan.register.Access_1_0, server)
    client.response_timeout = 1.0
    names = [*REGISTERS.values(), *extra]
    print(",".join(["time", *REGISTERS.keys(), *extra]), flush=True)
    loop = asyncio.get_running_loop()
    start = loop.time()
    count = 0
    try:
        while samples <= 0 or count < samples:
            due = start + count * period
            await asyncio.sleep(max(0.0, due - loop.time()))
            values: list[str] = []
            for name in names:
                response = await client.call(uavcan.register.Access_1_0.Request(name=uavcan.register.Name_1_0(name)))
                value = None if response is None else number(response[0].value)
                values.append("" if value is None else f"{value:.4f}")
            if not values[0]:
                print(f"no answer from node {server} ({names[0]})", file=sys.stderr)
            print(",".join([f"{loop.time() - start:.2f}", *values]), flush=True)
            count += 1
    finally:
        node.close()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("node_id", type=int, help="node-ID of the drive")
    parser.add_argument("--period", type=float, default=1.0, help="seconds between rows, default 1")
    parser.add_argument("--samples", type=int, default=0, help="rows to log; 0: until Ctrl-C")
    parser.add_argument("--extra", action="append", default=[],
                        help="further numeric register to log, e.g. a motor sensor of the bench")
    args = parser.parse_args()
    try:
        return asyncio.run(log(args.node_id, args.period, args.extra, args.samples))
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
