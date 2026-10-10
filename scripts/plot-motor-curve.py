#!/usr/bin/env python3
"""Draw the motor speed curve figures from docs/calibration/motor-speed-curve.csv.

Writes motor-speed-curve.png (RPM against PWM duty, both motors, both directions, with the
linear fit) and motor-speed-detail.png (the start of the curve, and right-over-left speed)
next to the CSV. Needs matplotlib, which the firmware and CI do not: run it with any
environment that has it, e.g. `python scripts/plot-motor-curve.py`.
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
DATA = ROOT / "docs" / "calibration" / "motor-speed-curve.csv"
OUT_MAIN = DATA.with_name("motor-speed-curve.png")
OUT_DETAIL = DATA.with_name("motor-speed-detail.png")

SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e5e4df"
COLOR = {"L": "#2a78d6", "R": "#eb6834"}  # categorical slots 1 and 2, validated together
NAME = {"L": "Left motor", "R": "Right motor"}
FIT_FROM, FIT_TO = 300, 700  # the linear part of the curve
MOVING_RPM = 3.0  # what counts as "turning"

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "font.family": "sans-serif", "font.size": 10.5, "text.color": INK,
    "axes.edgecolor": INK2, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
    "axes.spines.top": False, "axes.spines.right": False, "axes.grid": True,
    "grid.color": GRID, "grid.linewidth": 1.0, "axes.axisbelow": True,
    "legend.frameon": False,
})


def load():
    rows = []
    with DATA.open() as f:
        for r in csv.DictReader(f):
            rows.append((r["motor"], r["direction"], r["sweep"], int(r["duty"]), float(r["rpm"])))
    return rows


def series(rows, motor, direction, sweep):
    pts = sorted((d, rpm) for m, di, sw, d, rpm in rows if m == motor and di == direction and sw == sweep)
    return [p[0] for p in pts], [p[1] for p in pts]


def fit(xs, ys):
    pts = [(x, y) for x, y in zip(xs, ys) if FIT_FROM <= x <= FIT_TO]
    n = len(pts)
    sx, sy = sum(p[0] for p in pts), sum(p[1] for p in pts)
    sxx, sxy = sum(p[0] ** 2 for p in pts), sum(p[0] * p[1] for p in pts)
    k = (n * sxy - sx * sy) / (n * sxx - sx * sx)
    return k, (sy - k * sx) / n


def first_moving(xs, ys):
    return next(x for x, y in zip(xs, ys) if y >= MOVING_RPM)


def main_figure(rows):
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.6), sharey=True)
    for ax, (direction, title) in zip(axes, (("forward", "Forward"), ("reverse", "Reverse"))):
        for m in "LR":
            xu, yu = series(rows, m, direction, "up")
            xd, yd = series(rows, m, direction, "down")
            ax.plot(xd, yd, color=COLOR[m], lw=2, ls=(0, (4, 3)), zorder=2)
            ax.plot(xu, yu, color=COLOR[m], lw=2, zorder=3, label=NAME[m])
            ax.plot(xu[::2], yu[::2], "o", color=COLOR[m], ms=5, mec=SURFACE, mew=1.2, zorder=4)
            k, c = fit(xu, yu)
            ax.plot([FIT_FROM, FIT_TO], [k * FIT_FROM + c, k * FIT_TO + c], color=INK2, lw=1,
                    ls=(0, (1, 2)), zorder=1)
        ax.set_title(title, loc="left", fontweight="bold", color=INK, pad=10)
        ax.set_xlim(0, 800)
        ax.set_ylim(0, 300)
        ax.set_xlabel("PWM duty (timer counts, 0-799)")
    axes[0].set_ylabel("Wheel speed (RPM)")
    kl, _ = fit(*series(rows, "L", "forward", "up"))
    axes[0].annotate("dotted: straight-line fit\nover duty 300-700", xy=(500, 500 * kl - 20),
                     xytext=(560, 90), color=INK2, fontsize=9.5,
                     arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8))
    h = [plt.Line2D([], [], color=COLOR[m], lw=2, marker="o", ms=5, mec=SURFACE, label=NAME[m]) for m in "LR"]
    h += [plt.Line2D([], [], color=INK2, lw=2, label="duty raised"),
          plt.Line2D([], [], color=INK2, lw=2, ls=(0, (4, 3)), label="duty lowered")]
    fig.legend(handles=h, loc="upper left", bbox_to_anchor=(0.062, 0.9), ncol=4, fontsize=10.5)
    fig.suptitle("Wheel speed against PWM duty", x=0.065, y=0.985, ha="left", fontsize=15, fontweight="bold")
    fig.text(0.065, 0.918, "Wheel free on the stand, 10.8 V pack, 20 kHz PWM. Each step held 400 ms, speed from "
             "its last 200 ms. 1 RPM = 3.4 mm/s with the 65 mm wheel.", color=INK2, fontsize=10, ha="left")
    fig.tight_layout(rect=(0.02, 0, 1, 0.85))
    fig.savefig(OUT_MAIN, dpi=170)
    plt.close(fig)


def detail_figure(rows):
    fig, (a, b) = plt.subplots(1, 2, figsize=(12, 5.2))
    # (a) where the wheels start turning
    for m in "LR":
        for direction, ls in (("forward", "-"), ("reverse", (0, (5, 3)))):
            x, y = series(rows, m, direction, "up")
            a.plot(x, y, color=COLOR[m], lw=2, ls=ls, marker="o", ms=4.5, mec=SURFACE, mew=1.1)
    a.axhline(MOVING_RPM, color=INK2, lw=0.8, ls=(0, (1, 2)))
    a.set_xlim(40, 200)
    a.set_ylim(0, 55)
    a.set_xlabel("PWM duty (timer counts)")
    a.set_ylabel("Wheel speed (RPM)")
    a.set_title("Where the wheels start turning", loc="left", fontweight="bold", pad=26)
    start = {m: first_moving(*series(rows, m, "forward", "up")) for m in "LR"}
    a.text(0, 1.02, f"Dotted line = {MOVING_RPM:.0f} RPM. First duty above it, forward: "
           f"left {start['L']}, right {start['R']}", transform=a.transAxes, color=INK2, fontsize=9.5)
    a.legend(handles=[plt.Line2D([], [], color=COLOR["L"], lw=2, label="Left motor"),
                      plt.Line2D([], [], color=COLOR["R"], lw=2, label="Right motor"),
                      plt.Line2D([], [], color=INK2, lw=2, label="forward"),
                      plt.Line2D([], [], color=INK2, lw=2, ls=(0, (5, 3)), label="reverse")],
             loc="upper left", ncol=1)
    # (b) how closely the two motors match
    for direction, ls, lab in (("forward", "-", "forward"), ("reverse", (0, (5, 3)), "reverse")):
        xl, yl = series(rows, "L", direction, "up")
        xr, yr = series(rows, "R", direction, "up")
        pts = [(x, r / l * 100) for x, l, r in zip(xl, yl, yr) if x >= 160 and l > 0]
        b.plot([p[0] for p in pts], [p[1] for p in pts], color=INK, lw=2, ls=ls, label=lab)
    b.axhline(100, color=INK2, lw=0.8)
    b.set_xlim(160, 800)
    b.set_ylim(90, 110)
    b.set_xlabel("PWM duty (timer counts)")
    b.set_ylabel("Right speed as % of left speed")
    b.set_title("How closely the two motors match", loc="left", fontweight="bold", pad=26)
    b.text(0, 1.02, "Right wheel speed as a percentage of the left, at the same duty (duty raised)",
           transform=b.transAxes, color=INK2, fontsize=9.5)
    b.legend(loc="upper right")
    fig.suptitle("Motor speed curve, detail", x=0.065, y=0.985, ha="left", fontsize=15, fontweight="bold")
    fig.tight_layout(rect=(0.02, 0, 1, 0.93))
    fig.savefig(OUT_DETAIL, dpi=170)
    plt.close(fig)


def main() -> int:
    if not DATA.exists():
        print(f"missing {DATA}", file=sys.stderr)
        return 2
    rows = load()
    main_figure(rows)
    detail_figure(rows)
    print(f"wrote {OUT_MAIN.name} and {OUT_DETAIL.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
