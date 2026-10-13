# Gyro and accelerometer on the compact chassis

What was measured on the compact chassis, 2026-10-13 and 14, and what it decided. The
constants these feed are in `include/RobotSpec.h`; the code that uses them is described in
[heading](../architecture/modules/heading.md) and [imu](../architecture/modules/imu.md).

**Provenance.** The raw logs were not kept: the working directory they were written to was
cleared before they could be committed. Every number below comes from the analysis output of
those logs. The figures are reproducible by repeating the runs, which are short.

**Conditions.** MPU-6050 at +-500 deg/s and +-2 g, 20 Hz filter, read at 100 Hz: by the
`env:mega` loop (the repo's `readSensors()`, with one extra print per tick) for the drift and
scale runs, and by small bench programs on the same drivers for the accelerometer and pivot
runs. Robot on a bench, USB attached, motors off unless a pivot is stated. Chip temperature
26 to 30 C.

## 1. Gyro zero and drift at rest

The firmware captures the zero for 1000 readings at boot, then integrates `(raw - zero)`.
The robot was left still and `headingDeg` was read.

| | Run A: chip warm (powered for hours) | Run B: 1-2 min after plug-in |
|---|---|---|
| Zero captured | 15.27 counts | 15.94 counts |
| Heading at 2 min | +0.52 deg | -0.01 deg |
| Heading at 5 min | +1.89 deg | +2.24 deg |
| Heading at 8 min | +2.35 deg | +3.88 deg |
| Heading at 10 min | **+2.40 deg** | **+4.81 deg** |
| Worst 8-minute window | 2.35 deg | 4.83 deg |
| Chip temperature, start to end | 26.1 to 27.3 C | 26.4 to 27.0 C |
| Raw gyro Z, per-minute mean | 15.27 to 15.80 counts | 15.8, 16.1, 16.7, 16.8, then 16.4 to 16.6 |

Run B had one bump at about 290 s (+0.2 deg of the 4.8). Both runs drift the same way. The
zero moved 0.5 counts (A) and 0.5 to 0.8 counts (B), 0.008 to 0.012 deg/s, while the chip warmed
only 0.6 to 1.3 C, so the bias does not follow the chip temperature simply (two runs).

**Re-capturing the zero at START.** The robot was assumed to start its run at 60 s or 120 s
after power-on, with the zero re-captured from the 3 s before. Heading error over the
following 8 minutes, robot still:

| | Boot capture only | Re-captured at START |
|---|---|---|
| A, START at 60 s | +2.10 deg | +0.59 deg |
| A, START at 120 s | +1.84 deg | -2.48 deg |
| B, START at 60 s | +4.50 deg | +4.96 deg |
| B, START at 120 s | +4.84 deg | +3.69 deg |

It is smaller in 2 of 4 and larger in the other 2. A 3 s window (300 readings) only fixes the
zero to about +-0.15 counts, which is +-1 deg over 8 minutes by itself, and the bias keeps moving
inside the run. **Decision: no warm-up wait and no re-zero at START.** The residual at rest is
2 to 5 deg per 10 minutes; anything tighter is a job for wall-based correction, and the motors-on
figure is not measured.

## 2. A zero captured while the robot moves

Two captures taken while the robot was being moved by hand:

| | Zero captured | True zero | What the heading did |
|---|---|---|---|
| Hand-turned scale run, partly moving | 25.30 counts | about 16.3 | ended -19.8 deg for a robot back at its start (my own integral from raw counts: +0.1 deg) |
| Turned at about 24 deg/s throughout | **1599** counts | about 15 | firmware heading +97.4 deg, exactly `wrap(real rotation 829 deg - (1599 - 15) / 65.5 x 60 s)` |

A still capture scattered 1.9 to 2.9 counts (standard deviation); the first 1.5 s of the
log taken while turning scattered 334. That gap is what `GYRO_BIAS_MAX_SPREAD_COUNTS` rejects.

## 3. Gyro scale

Raw counts integrated with their timestamps, bias from the quiet stretches (16.3 to 16.6 counts).
The robot was held against a mark, turned 5 closed laps counter-clockwise and 5 laps clockwise,
and returned to the mark.

| | Integrated | True | Ratio |
|---|---|---|---|
| 5 laps counter-clockwise | +1802.3 deg | 1800 deg | 1.0013 |
| 5 laps clockwise | -1801.5 deg | 1800 deg | 1.0008 |
| Symmetric, (CCW - CW) / 2 | 1801.9 deg | 1800 deg | **1.0011** |
| Closure after 150 s | +0.1 to +0.8 deg | 0 | |

Choosing a bias anywhere from 16.3 to 18.4 counts moves the ratio between 1.0012 and 1.0024. The hand
stopped within about +-6 deg of the mark at each lap boundary, which over 1800 deg is +-0.3%.
**Result: 1.001 to 1.002 of nominal, +-0.002. `GYRO_COUNTS_PER_DPS` stays at 65.5.**

Peak rate in the laps was 437 deg/s against a full scale of 500. The laps ran 25 to 67 deg/s
averaged over their pauses, so **linearity between slow and fast turns was not established.**

## 4. A turn the robot made itself

The robot pivoted in place closed-loop on the gyro rate (encoders not used), from its own
motors, stopping when the gyro read 360 deg.

| Turn | Mean rate | Gyro total | Coast after the stop |
|---|---|---|---|
| 90 deg | about 115 deg/s | 94.99 deg | +5.0 deg |
| 360 deg, run 1 | about 133 deg/s | 366.67 deg | +6.7 deg |
| 360 deg, run 2 | about 133 deg/s | 366.33 deg | +6.3 deg |

A photograph of run 2, the wheel against a reference strip, read about +7 +- 2 deg of extra
rotation against the +6.3 deg the gyro reported, which agrees within the photo's accuracy and
puts the scale within about 0.5% on its own. (Automatic edge-fitting of the tyre was not
reliable: its shadow and the visible rim moved its apparent edge between photos. The tape strip
fitted cleanly, to 0.01 deg.) The coast is not a gyro error: the gyro measured it.

Left and right encoders were not symmetric in these pivots: right/left counts were 1.18 (run 1)
and 1.15 (run 2), and 1.22 in the 90 deg turn. That is not explained yet.

## 5. Accelerometer, robot level

30 s still in each of two orientations 180 deg apart, chip at 29.3 and 29.5 C.

| | Position 1 | Position 2 (turned 180 deg) | Stays with the robot | Follows the table |
|---|---|---|---|---|
| X | +180.2 | -65.8 | +57 | +123 |
| Y (forward) | +32.2 | +456.7 | +245 | -212 |
| Z | 17151.7 | 17145.9 | | |
| \|g\| | 17153 | 17152 | | |

Spread within a position was 16 to 25 counts (standard deviation), so each mean is good to
about 0.3 to 0.5 counts over 3000 samples. \|g\| is 1.047 x 16384 in both positions, so the
sensitivity reads about 4.7% high on this unit. The offset that stays with the robot is
about 0.2 deg on X and 0.8 deg on Y (sensor offset plus mount tilt, not separable with two
positions); the table or the way the robot rested contributed about 0.4 deg on X and 0.7 deg on Y. A 0.8 deg mount
tilt couples about 1.5% of a pitch or roll rate into the yaw axis.

## Not measured

- Anything with the motors running: the bias, the drift and the accelerometer reading under
  vibration and motor current.
- The accelerometer's offset against temperature and with all six faces up.
- Gyro scale at different turn rates, and in each direction separately with a hard stop for the
  reference.
- The datasheet tolerances for gyro and accelerometer sensitivity: the specification table was
  not reachable while this was written, so no claim is made that a value is in or out of spec.
- Why the right encoder counts 15 to 22% more than the left in a pivot.
