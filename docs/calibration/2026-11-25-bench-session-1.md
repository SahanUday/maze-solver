# Bench session 1: every sensor and motor checked

What was checked on the robot on 2026-11-25 with the bench firmware (`env:bench`,
`tools/bench/bench.cpp`), what it showed, and what is still open. Part of issue #54. No constant in
`include/spec/` was changed: today's numbers are checks, not calibrations (no tape measure, so
every distance below was set by eye).

**Raw log:** [`raw/2026-11-25-bench.log`](raw/2026-11-25-bench.log): every line the robot sent,
prefixed with the host time in seconds, and every command (`>> ...`). The `m` labels below
(`us_front_15cm`, ...) find the matching block.

**Conditions.** Compact chassis, USB to a laptop and the battery on (except where stated), indoor
room lighting. Robot on a table, then the floor. IR calibration in EEPROM: the one stored before
the session, never overwritten (no `save` was typed).

## Summary

| Part | Result |
|---|---|
| Ultrasonic front, left, right | work; read the barrier's distance within hand placement; sd 2.3-4.2 mm |
| Encoders | work, count up forward; 895 / 905 counts for one hand turn (898 expected) |
| Gyro | works, CCW positive, scale within hand placement; **warm-up drift** after power-on (finding 2) |
| IMU power | **stays asleep after a battery power cycle while USB keeps the Arduino up** (finding 1) |
| IR array | all 8 channels work; line position and its sign correct; stored calibration usable on plain paper |
| IR calibration by hand | **fails**: tilting the robot lets room light in (finding 3) |
| Motors and drivers | both wheels, both directions, speed rises evenly with duty |
| Closed-loop straight | `s 300` in the air: done in 2.42 s, 6.4 mm short (tolerance 8) |
| Floor driving | not done (next session) |

## Ultrasonics

A barrier about 150 mm from each sensor's face, placed by eye, robot still, `m 3000`. The robot
reports the front from the axle and the sides from the centreline (`US_OFFSET_*` in
`spec/measurements/Robot.h`), so the face distance is the reading minus the offset.

| Sensor | Reading (mean, min-max) | sd | Minus offset | Barrier |
|---|---|---|---|---|
| Front | 230.7 (227-233) | 2.3 | 147 (offset 84) | ~150 |
| Left | 192.6 (186-199) | 3.5 | 158 (offset 35) | ~150 |
| Right | 193.0 (189-198) | 4.2 | 146 (offset 47) | ~150 |

Before the barrier was placed, the right sensor read exactly 161 mm for minutes; a barrier moved it.
It was a still object, not a fault, but the bench's `m` counted only readings that changed, so it
looked dead. Fixed: `m` now counts every tick with an echo.

## Encoders

`z`, then each wheel turned forward one full turn by hand: left +895, right +905 counts (898
expected, `ENCODER_COUNTS_PER_REV`); odometer 204.7 mm. Within where a hand stops.

## Gyro

**Scale and sign.** Robot on the floor, turned by hand against a straight edge after `b`:

| Turn | Expected | Read |
|---|---|---|
| 90 deg CCW | +90 | +89.28 |
| back to the start | 0 | -0.66 |
| 360 deg CCW | 0 | +2.35 (3.0 deg added) |

The 90 deg turn read 0.8% low and the 360 deg turn 0.8% high: hand placement, not scale.
`GYRO_YAW_INVERT = false` is right.

**Drift.** See finding 2.

## IR array

Robot sitting on the surface, still, `m 2000`:

| Channel | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|---|
| Off-white paper | 903 | 857 | 869 | 873 | 860 | 895 | 876 | 908 |
| Plain white paper | 816 | 736 | 775 | 801 | 797 | 858 | 841 | 874 |
| Black paper | 979 | 972 | 977 | 979 | 975 | 984 | 979 | 984 |
| Stored calibration white | 848 | 779 | 798 | 800 | 778 | 836 | 809 | 851 |
| Stored calibration black | 1016 | 1011 | 1013 | 1012 | 1009 | 1013 | 1011 | 1015 |

Noise under 0.5 counts on every channel. Against the stored calibration, black paper is 78-85%
toward black (switches at 60%), plain paper 0-16%, the off-white paper 20-35%.

**Line position.** A 2-3 cm black strip slid across the array (`v 200`): the error went smoothly
from -28 mm (under channel 0, the robot's right end) through 0 at the centre to +15 mm (channel 7,
the left end). The sign is what the follower expects and `IR_D1_ON_RIGHT = true` is right.

**Stuck channel.** On the off-white paper, after the strip had left, channel 7 read 927: 46% of
its stored range, inside the 40-60% hysteresis band, so it stayed black and the robot reported a
line at +24 mm that was not there. A calibration from another surface can fail like this.

## Motors (wheels in the air)

`o L R ms`, speed over the second half of each run:

| Duty (of 799) | Left mm/s | Right mm/s |
|---|---|---|
| 100 | 44 | 0 (did not start) |
| 150 | 152 | 126 |
| 200 (one wheel at a time) | 226 | 204 |
| 300 | 355 | 349 |
| 450 | 546 | 530 |
| 600 | 743 | 726 |
| -200 (reverse) | -221 | -199 |

Above duty 150 the two wheels run within 3% of each other. On the floor the right runs about 20%
slower (`FEEDFORWARD_RIGHT_SCALE`, `spec/tuning/Motion.h`), so that difference is load or friction
on the right side, not the motor.

## Findings

### 1. The IMU stays asleep after the sensors' power comes back

The sensors (IR array, ultrasonics, MPU-6050) are powered from the battery through the buck
converter; with USB attached the Arduino keeps running on USB when the battery is off. A battery
off and on, watched with `v 100`:

- battery off: ultrasonics no echo, IR all channels 143, IMU frozen at its last values (the
  transfers fail and the driver keeps the previous sample);
- battery on: ultrasonics and IR recover by themselves; the IMU reads **0 on every axis** and stays
  there, because the MPU-6050 powers up asleep and nothing configures it again.

The firmware does not notice: the chip answers on I2C, so `imuRead()` succeeds, and the heading
integrates the bias alone (-0.035 deg/s here). It happened once by accident, when the battery was
switched on for the motor tests, and once on purpose. A reset brought the IMU back both times.

- **On the bench:** switch the battery on first, then reset the Arduino.
- **In a run:** with the battery alone, everything powers up together and boot configures the IMU.
  A dip that resets the IMU but not the Arduino would leave the robot without a heading, unnoticed.
  Worth detecting (all axes 0, or |accel| far from 1 g) and re-configuring the chip.

### 2. Gyro drift in the first minutes after power-on

The zero captured at boot was 4.14 counts; about 6 minutes later `b` captured 2.27. Before the
re-capture the heading drifted -0.72 to -1.9 deg/min at rest; after it, -0.03 deg/min over 30 s.
That fits the October measurements (a bias that moves after plug-in), but larger. One run, so to be
repeated: power on, leave still, `m 60000` every few minutes, then decide between switching on
early and re-zeroing at the start press (#64).

### 3. IR calibration cannot be done by hand

Three captures (`k` ... `f`) with a strip moved by hand all came out wrong: "white" values of
331-742 where the same paper reads 783-899 at rest, and some channels never reaching black. With
the robot still, the readings were normal again. Bending the paper or tilting the robot changes the
ride height (7 mm) and lets room light under the array. `lineCalibrationLooksReal()` passed two of
the three bad captures: it is a check for a dead array, not for a bad sweep. None was saved.

The capture should be made with the robot on its wheels, driving itself slowly across a strip on
a flat surface (the old probe's `K`), at the arena.

## Not done

Everything needing a tape measure or the arena: ultrasonic offsets and spread at 100/300/1000 mm,
odometer over 2 m, tile pitch and wall thickness, IR on the markers, gap, ramp and finish tile, the
incline on the ramp. Floor driving: start duty on the floor, straights and turns (track width,
timings for #76).
