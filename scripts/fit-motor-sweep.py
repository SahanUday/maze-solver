#!/usr/bin/env python3
"""Turn a motor sweep (include/SweepRun.h) into numbers: where each wheel starts and
stops turning, its speed-per-duty slope, its top speed, and how well the two motors match.

  scripts/fit-motor-sweep.py CAPTURE [--counts-per-rev N] [--wheel-mm MM] [--write-csv OUT]

CAPTURE is a serial capture of a sweep run, or docs/calibration/motor-speed-curve.csv. A line
is a row when it has the shape  L|R,forward|reverse,up|down,duty,counts,dt  (6 fields: dt in
ms, from the robot) or  ...,dt,rpm,mm_per_s  (8 fields: dt in us, the older file). Everything
else, including `#` notes, is ignored, so the monitor's chatter can stay in the file.

The speed is worked out from the raw counts, with ENCODER_COUNTS_PER_REV and WHEEL_DIAMETER_MM
from include/RobotSpec.h unless given, so a corrected constant needs no new run. The slope is a
least-squares line over duty FIT_FROM..FIT_TO, both sweeps together.

--write-csv writes the rows with rpm and mm_per_s added, in the layout
scripts/plot-motor-curve.py reads.

Exit status: 0 complete; 1 the sweep is incomplete (the report still prints); 2 no rows or
unreadable input.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SPEC = ROOT / "include" / "RobotSpec.h"
SWEEP_SPEC = ROOT / "include" / "SweepRunSpec.h"

FIT_FROM, FIT_TO = 300, 700  # the linear part of the curve
MOVING_RPM = 3.0  # what counts as "turning"

ROW = re.compile(
    r"^(?P<motor>[LR]),(?P<direction>forward|reverse),(?P<sweep>up|down),"
    r"(?P<duty>\d+),(?P<counts>-?\d+),(?P<dt>\d+)(?:,-?[\d.]+,-?[\d.]+)?\s*$"
)
NOTE = re.compile(r"(\w+)=(\S*)")
COMBOS = [(m, d) for m in "LR" for d in ("forward", "reverse")]


def constant(path: Path, name: str, default: int) -> int:
    m = re.search(rf"\b{name}\s*=\s*(\d+)", path.read_text()) if path.exists() else None
    return int(m.group(1)) if m else default


def parse(text: str) -> tuple[list[dict], dict[str, str]]:
    """Capture text -> (rows, notes). Row: motor, direction, sweep, duty, counts, dt_s."""
    rows, notes = [], {}
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("#"):
            notes.update(NOTE.findall(line))
            continue
        m = ROW.match(line)
        if not m:
            continue
        # 8 fields is the older file, which timed the window in microseconds.
        per_s = 1e6 if line.count(",") == 7 else 1e3
        rows.append({
            "motor": m["motor"], "direction": m["direction"], "sweep": m["sweep"],
            "duty": int(m["duty"]), "counts": int(m["counts"]), "dt_s": int(m["dt"]) / per_s,
        })
    return rows, notes


def speeds(rows: list[dict], counts_per_rev: int, wheel_mm: float) -> None:
    """Adds rpm and mm_per_s (magnitudes) to every row."""
    for r in rows:
        r["rpm"] = abs(r["counts"]) / counts_per_rev / r["dt_s"] * 60.0
        r["mm_per_s"] = r["rpm"] * math.pi * wheel_mm / 60.0


def linear_fit(points: list[tuple[float, float]]) -> tuple[float, float, float] | None:
    """Least squares y = a + b x -> (b, a, r2), or None if the points cannot define a line."""
    n = len(points)
    if n < 3:
        return None
    sx = sum(p[0] for p in points)
    sy = sum(p[1] for p in points)
    sxx = sum(p[0] * p[0] for p in points)
    sxy = sum(p[0] * p[1] for p in points)
    den = n * sxx - sx * sx
    if den == 0:
        return None
    b = (n * sxy - sx * sy) / den
    a = (sy - b * sx) / n
    mean = sy / n
    ss_tot = sum((p[1] - mean) ** 2 for p in points)
    ss_res = sum((p[1] - (a + b * p[0])) ** 2 for p in points)
    return b, a, (1.0 - ss_res / ss_tot) if ss_tot else 1.0


def analyse(rows: list[dict], motor: str, direction: str) -> dict | None:
    mine = [r for r in rows if r["motor"] == motor and r["direction"] == direction]
    if not mine:
        return None
    up = sorted((r for r in mine if r["sweep"] == "up"), key=lambda r: r["duty"])
    down = [r for r in mine if r["sweep"] == "down"]
    out = {"rows": len(mine), "up": len(up), "down": len(down)}

    turning_up = [r["duty"] for r in up if r["rpm"] >= MOVING_RPM]
    turning_down = [r["duty"] for r in down if r["rpm"] >= MOVING_RPM]
    out["starts"] = min(turning_up) if turning_up else None
    out["keeps_turning_to"] = min(turning_down) if turning_down else None

    top_duty = max(r["duty"] for r in mine)
    top = [r for r in mine if r["duty"] == top_duty]
    out["top_duty"] = top_duty
    out["top_rpm"] = sum(r["rpm"] for r in top) / len(top)
    out["top_mm_s"] = sum(r["mm_per_s"] for r in top) / len(top)

    fit = linear_fit([(r["duty"], r["rpm"]) for r in mine if FIT_FROM <= r["duty"] <= FIT_TO])
    out["fit"] = None
    if fit and fit[0] > 0:
        slope, intercept, r2 = fit
        out["fit"] = {"slope": slope, "zero_duty": -intercept / slope, "r2": r2}
    return out


def speed_ratio(rows: list[dict], direction: str, from_duty: int = 200) -> tuple[float, float, float] | None:
    """Right over left speed at the same duty -> (mean, min, max) over the rungs both have."""
    def mean_rpm(motor: str, duty: int) -> float | None:
        v = [r["rpm"] for r in rows if r["motor"] == motor and r["direction"] == direction and r["duty"] == duty]
        return sum(v) / len(v) if v else None

    ratios = []
    for duty in sorted({r["duty"] for r in rows if r["direction"] == direction and r["duty"] >= from_duty}):
        left, right = mean_rpm("L", duty), mean_rpm("R", duty)
        if left and right:
            ratios.append(right / left)
    if not ratios:
        return None
    return sum(ratios) / len(ratios), min(ratios), max(ratios)


def expected_rows() -> int:
    """4 combos x (up + down rungs), from the run's own schedule when it can be read."""
    start = constant(SWEEP_SPEC, "SWEEP_DUTY_START", 40)
    step = constant(SWEEP_SPEC, "SWEEP_DUTY_STEP", 20)
    top = constant(SWEEP_SPEC, "SWEEP_DUTY_MAX", 760)
    return len(COMBOS) * 2 * ((top - start) // step + 1)


def report(rows: list[dict], notes: dict[str, str], cpr: int, wheel_mm: float) -> tuple[str, bool]:
    lines = []
    want = expected_rows()
    complete = len(rows) == want
    lines.append(f"rows {len(rows)} (a full sweep has {want})   counts/rev {cpr}   wheel {wheel_mm:g} mm")
    for key, used in (("counts_per_rev", cpr), ("wheel_mm", wheel_mm)):
        if key in notes and notes[key] not in ("", f"{used:g}"):
            lines.append(f"note: the capture says {key}={notes[key]}; {used:g} was used")
    for key in ("pack_v", "surface", "built"):
        if notes.get(key):
            lines.append(f"{key}: {notes[key]}")
    if not notes.get("pack_v"):
        lines.append("note: no pack voltage written in the capture; duty to speed moves with it")
    lines.append("")

    lines.append(f"{'':5}{'dir':8}{'starts':>8}{'turns to':>10}{'rpm/duty':>10}{'zero duty':>11}{'R2':>7}"
                 f"{'top rpm':>9}{'mm/s':>8}")
    results = {}
    for motor, direction in COMBOS:
        a = analyse(rows, motor, direction)
        if a is None:
            complete = False
            lines.append(f"{motor:5}{direction:8} no rows")
            continue
        results[(motor, direction)] = a
        if a["up"] != a["down"] or a["up"] == 0:
            complete = False
        fit = a["fit"] or {}
        cells = [
            a["starts"] if a["starts"] is not None else "-",
            a["keeps_turning_to"] if a["keeps_turning_to"] is not None else "-",
            f"{fit['slope']:.3f}" if fit else "-",
            f"{fit['zero_duty']:.0f}" if fit else "-",
            f"{fit['r2']:.3f}" if fit else "-",
            f"{a['top_rpm']:.1f}",
            f"{a['top_mm_s']:.0f}",
        ]
        widths = [8, 10, 10, 11, 7, 9, 8]
        lines.append(f"{motor:5}{direction:8}" + "".join(f"{str(c):>{w}}" for c, w in zip(cells, widths)))

    lines.append("")
    for direction in ("forward", "reverse"):
        ratio = speed_ratio(rows, direction)
        if ratio:
            mean, low, high = ratio
            lines.append(f"right/left speed at the same duty (200 up), {direction}: "
                         f"{mean:.3f} ({(mean - 1) * 100:+.1f} %), range {low:.3f}..{high:.3f}")
    lines.append(f"fit: duty {FIT_FROM}..{FIT_TO}; 'starts' = first rung on the way up at >= {MOVING_RPM:g} rpm;")
    lines.append("'turns to' = lowest rung on the way down still at >= that; 'zero duty' = where the line hits 0 rpm")
    if not complete:
        lines.append("")
        lines.append("INCOMPLETE: a wheel or direction is missing rungs. Re-run, or the capture was cut short.")
    return "\n".join(lines) + "\n", complete


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w") as f:
        f.write("motor,direction,sweep,duty,counts,dt_us,rpm,mm_per_s\n")
        for r in rows:
            f.write(f"{r['motor']},{r['direction']},{r['sweep']},{r['duty']},{abs(r['counts'])},"
                    f"{round(r['dt_s'] * 1e6)},{r['rpm']:.2f},{r['mm_per_s']:.1f}\n")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("capture", type=Path)
    ap.add_argument("--counts-per-rev", type=int)
    ap.add_argument("--wheel-mm", type=float)
    ap.add_argument("--write-csv", type=Path, metavar="OUT")
    args = ap.parse_args(argv)

    try:
        text = args.capture.read_text(errors="replace")
    except OSError as e:
        print(f"fit-motor-sweep: {e}", file=sys.stderr)
        return 2
    rows, notes = parse(text)
    if not rows:
        print(f"fit-motor-sweep: no sweep rows in {args.capture}", file=sys.stderr)
        return 2

    cpr = args.counts_per_rev or constant(SPEC, "ENCODER_COUNTS_PER_REV", 898)
    wheel = args.wheel_mm or float(constant(SPEC, "WHEEL_DIAMETER_MM", 65))
    speeds(rows, cpr, wheel)
    text_out, complete = report(rows, notes, cpr, wheel)
    sys.stdout.write(text_out)
    if args.write_csv:
        write_csv(args.write_csv, rows)
    return 0 if complete else 1


if __name__ == "__main__":
    sys.exit(main())
