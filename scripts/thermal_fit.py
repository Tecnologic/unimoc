#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Fits the thermal parameters of a motor (unimoc.motor.thermal.C_winding,
# C_stator, R_winding, R_stator) to a heat run (docs/motor_thermal_plan.md,
# section 6). The model is the one of control::MotorThermalModel: two masses,
# heated by 3/2 R_s(T) |i_dq|^2, stepped with backward Euler.
#
# Input: a CSV with a header. Columns (names configurable):
#   time [s]; the current: either the RMS current [A] (the register
#   unimoc.motor.temperature.current, --current) or i_d and i_q [A]
#   (amplitude-invariant rotor frame, the cymon traces); the winding
#   temperature [degC] (unimoc.motor.temperature.rs or the trace
#   rs_temperature at a locked rotor with the dead-time compensation checked,
#   or a thermocouple in the end winding) and optionally the stator or housing
#   temperature [degC] (a motor sensor). Empty or NaN cells of a temperature
#   are left out of the fit; the current must be complete.
#
#   scripts/thermal_fit.py run.csv --resistance 0.15 --ambient 22
#   scripts/thermal_fit.py --self-test
#
# No dependencies beyond the Python standard library (Nelder-Mead in log space).
from __future__ import annotations

import argparse
import csv
import math
import random
import sys
from dataclasses import dataclass

COPPER_COEFFICIENT = 3.93e-3  # 1/K, control::kCopperTemperatureCoefficient


@dataclass
class Network:
    """The four parameters of the fit, as the registers unimoc.motor.thermal.*."""

    c_winding: float  # J/K
    c_stator: float  # J/K
    r_winding: float  # K/W, winding to stator
    r_stator: float  # K/W, stator to ambient

    def as_log(self) -> list[float]:
        return [math.log(v) for v in (self.c_winding, self.c_stator, self.r_winding, self.r_stator)]

    @staticmethod
    def from_log(values: list[float]) -> "Network":
        return Network(*(math.exp(v) for v in values))


@dataclass
class Run:
    time: list[float]
    current_squared: list[float]
    winding: list[float | None]
    stator: list[float | None]


def simulate(network: Network, run: Run, resistance: float, reference: float, ambient: float,
             start: float) -> tuple[list[float], list[float]]:
    """Winding and stator temperature over the run, as MotorThermalModel::Step()."""
    winding, stator = start, start
    windings, stators = [winding], [stator]
    for k in range(1, len(run.time)):
        dt = run.time[k] - run.time[k - 1]
        if dt <= 0.0:
            windings.append(winding)
            stators.append(stator)
            continue
        # The losses of the interval at the winding temperature at its start.
        loss = 1.5 * resistance * max(1.0 + COPPER_COEFFICIENT * (winding - reference), 0.0) * \
            run.current_squared[k - 1]
        a = network.c_winding / dt
        b = network.c_stator / dt
        g1 = 1.0 / network.r_winding
        g2 = 1.0 / network.r_stator
        source_w = a * winding + loss
        source_s = b * stator + g2 * ambient
        diag_s = b + g1 + g2
        det = a * diag_s + g1 * (b + g2)
        winding, stator = ((source_w * diag_s + g1 * source_s) / det,
                           (source_s * (a + g1) + g1 * source_w) / det)
        windings.append(winding)
        stators.append(stator)
    return windings, stators


def cost(network: Network, run: Run, resistance: float, reference: float, ambient: float,
         start: float) -> float:
    windings, stators = simulate(network, run, resistance, reference, ambient, start)
    total, count = 0.0, 0
    for k in range(len(run.time)):
        if run.winding[k] is not None:
            total += (windings[k] - run.winding[k]) ** 2
            count += 1
        if run.stator[k] is not None:
            total += (stators[k] - run.stator[k]) ** 2
            count += 1
    return total / max(count, 1)


def nelder_mead(function, start: list[float], step: float = 0.5, iterations: int = 4000,
                tolerance: float = 1e-10) -> list[float]:
    """Minimizes @p function from @p start; the simplex spans @p step in each coordinate."""
    n = len(start)
    simplex = [list(start)] + [[s + (step if i == j else 0.0) for j, s in enumerate(start)] for i in range(n)]
    values = [function(p) for p in simplex]
    for _ in range(iterations):
        order = sorted(range(n + 1), key=lambda i: values[i])
        simplex = [simplex[i] for i in order]
        values = [values[i] for i in order]
        if abs(values[-1] - values[0]) <= tolerance * (abs(values[0]) + tolerance):
            break
        centroid = [sum(p[j] for p in simplex[:-1]) / n for j in range(n)]
        worst = simplex[-1]
        reflected = [c + (c - w) for c, w in zip(centroid, worst)]
        f_reflected = function(reflected)
        if f_reflected < values[0]:
            expanded = [c + 2.0 * (c - w) for c, w in zip(centroid, worst)]
            f_expanded = function(expanded)
            simplex[-1], values[-1] = (expanded, f_expanded) if f_expanded < f_reflected else (reflected, f_reflected)
        elif f_reflected < values[-2]:
            simplex[-1], values[-1] = reflected, f_reflected
        else:
            contracted = [c + 0.5 * (w - c) for c, w in zip(centroid, worst)]
            f_contracted = function(contracted)
            if f_contracted < values[-1]:
                simplex[-1], values[-1] = contracted, f_contracted
            else:
                best = simplex[0]
                simplex = [best] + [[b + 0.5 * (p - b) for b, p in zip(best, q)] for q in simplex[1:]]
                values = [values[0]] + [function(p) for p in simplex[1:]]
    return simplex[min(range(n + 1), key=lambda i: values[i])]


def initial_guess(run: Run, resistance: float, ambient: float) -> Network:
    """A start from the data: the total rise at the mean loss, a guessed split and time constants."""
    measured = [w for w in run.winding if w is not None]
    rise = max(max(measured) - ambient, 1.0) if measured else 50.0
    mean_loss = 1.5 * resistance * sum(run.current_squared) / max(len(run.current_squared), 1)
    total = max(rise / max(mean_loss, 1e-3), 1e-3)
    duration = max(run.time[-1] - run.time[0], 1.0)
    r_winding, r_stator = total / 3.0, 2.0 * total / 3.0
    return Network(c_winding=duration / 20.0 / r_winding, c_stator=duration / 2.0 / r_stator,
                   r_winding=r_winding, r_stator=r_stator)


def fit(run: Run, resistance: float, reference: float, ambient: float, start: float) -> tuple[Network, float]:
    guess = initial_guess(run, resistance, ambient)
    objective = lambda logs: cost(Network.from_log(logs), run, resistance, reference, ambient, start)  # noqa: E731
    best = guess.as_log()
    for _ in range(3):  # restarts around the best point
        best = nelder_mead(objective, best)
    network = Network.from_log(best)
    return network, math.sqrt(objective(best))


def read_run(path: str, columns: argparse.Namespace) -> Run:
    def number(text: str | None) -> float | None:
        if text is None or text.strip() == "":
            return None
        value = float(text)
        return None if math.isnan(value) else value

    run = Run([], [], [], [])
    with open(path, newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            if columns.current:
                rms = number(row[columns.current])
                squared = None if rms is None else rms * rms
            else:
                i_d = number(row[columns.i_d])
                i_q = number(row[columns.i_q])
                squared = None if i_d is None or i_q is None else i_d * i_d + i_q * i_q
            if squared is None:
                raise SystemExit(f"{path}: the current must be complete (row {len(run.time) + 2})")
            run.time.append(float(row[columns.time]))
            run.current_squared.append(squared)
            run.winding.append(number(row.get(columns.winding)))
            run.stator.append(number(row.get(columns.stator)) if columns.stator else None)
    if len(run.time) < 10:
        raise SystemExit(f"{path}: too few rows")
    return run


def self_test() -> int:
    """Recovers the parameters of a synthetic heat run with sensor noise."""
    truth = Network(c_winding=150.0, c_stator=1500.0, r_winding=0.4, r_stator=0.8)
    rng = random.Random(1)
    time = [float(t) for t in range(0, 7201, 5)]  # two hours every 5 s
    # 15 A for 80 minutes, then 40 minutes off.
    current_squared = [225.0 if t < 4800.0 else 0.0 for t in time]
    run = Run(time, current_squared, [None] * len(time), [None] * len(time))
    windings, stators = simulate(truth, run, 0.15, 25.0, 25.0, 25.0)
    run.winding = [w + rng.gauss(0.0, 0.5) for w in windings]
    run.stator = [s + rng.gauss(0.0, 0.5) for s in stators]
    network, residual = fit(run, 0.15, 25.0, 25.0, 25.0)
    print(f"truth {truth}\nfit   {network}\nrms residual {residual:.2f} K")
    errors = [abs(f / t - 1.0) for f, t in zip(
        (network.c_winding, network.c_stator, network.r_winding, network.r_stator),
        (truth.c_winding, truth.c_stator, truth.r_winding, truth.r_stator))]
    ok = max(errors) < 0.1
    print("self-test", "passed" if ok else "FAILED", f"(largest error {100.0 * max(errors):.1f} %)")
    return 0 if ok else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="?", help="heat run with a header")
    parser.add_argument("--resistance", type=float, help="unimoc.motor.stator.R [Ohm]")
    parser.add_argument("--reference", type=float, default=25.0,
                        help="unimoc.motor.stator.R_temperature [degC], default 25")
    parser.add_argument("--ambient", type=float, help="ambient temperature of the run [degC]")
    parser.add_argument("--start", type=float, help="temperature of both masses at the start [degC]; "
                        "default the ambient (start the run with a cold motor)")
    parser.add_argument("--time", default="time", help="time column [s], default 'time'")
    parser.add_argument("--current", default=None,
                        help="RMS current column [A] (unimoc.motor.temperature.current); replaces i_d and i_q")
    parser.add_argument("--i-d", dest="i_d", default="i_d", help="d current column [A], default 'i_d'")
    parser.add_argument("--i-q", dest="i_q", default="i_q", help="q current column [A], default 'i_q'")
    parser.add_argument("--winding", default="rs_temperature",
                        help="winding temperature column [degC], default 'rs_temperature'")
    parser.add_argument("--stator", default=None, help="optional stator/housing temperature column [degC]")
    parser.add_argument("--self-test", action="store_true", help="fit a synthetic run and check the result")
    args = parser.parse_args()
    if args.self_test:
        return self_test()
    if args.csv is None or args.resistance is None or args.ambient is None:
        parser.error("csv, --resistance and --ambient are required")
    run = read_run(args.csv, args)
    start = args.ambient if args.start is None else args.start
    network, residual = fit(run, args.resistance, args.reference, args.ambient, start)
    print(f"unimoc.motor.thermal.C_winding = {network.c_winding:.1f}   # J/K")
    print(f"unimoc.motor.thermal.C_stator  = {network.c_stator:.1f}   # J/K")
    print(f"unimoc.motor.thermal.R_winding = {network.r_winding:.4f}   # K/W")
    print(f"unimoc.motor.thermal.R_stator  = {network.r_stator:.4f}   # K/W")
    print(f"time constants: winding {network.c_winding * network.r_winding:.0f} s, "
          f"stator {network.c_stator * network.r_stator:.0f} s; rms residual {residual:.2f} K")
    if not any(s is not None for s in run.stator):
        print("note: without a stator column the split of the two masses rests on the shape of the "
              "winding curve alone; a run with a cool-down phase separates them best.", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
