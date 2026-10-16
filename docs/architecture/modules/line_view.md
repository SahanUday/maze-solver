# line_view

One sweep of the IR array to where the line is. Pure logic in `include/sense/LineView.h`, tested by
`test/test_line_view/`. It takes the normalised channel values and black mask from `LineSense.h`,
and the `usable` bit mask of channels the calibration accepted.

## Interface

`lineView(norm, mask, usable, pitchMm, d1OnRight = false)` returns a `LineView`:

- `valid` - at least one usable channel. When false, the three states below are all false.
- `allBlack` - every usable channel is black: bare maze floor, or the array lifted off it.
- `allWhite` - no usable channel is black: off the line, or a white tile.
- `onLine` - some black and some white.
- `errMm` - centroid of the black channels in mm from the array centre, positive to the **left**
  (counter-clockwise, like heading). Only meaningful when `allWhite` is false.

## How it works

Black means the mask bit is set **and** the channel is usable. The centroid weights each black
channel by its darkness (`norm`); channel 0 (D1) is the leftmost, the most positive position, or
the rightmost with `d1OnRight`. On this robot D1 is the right end (`IR_D1_ON_RIGHT`, found with a
black card under one end of the array).

## Invariants

- With no usable channel (no calibration loaded) the view is not `valid`: it is neither the floor
  nor a white tile, and `errMm` is 0.
- The maze floor is black, so `allBlack` is the normal state in the maze, not a fault.
- `pitchMm` comes from `IR_SENSOR_PITCH_MM` (8 mm, measured); 0 would make `errMm` 0.

## Status

Host tests only. Not run on the robot: the array is not calibrated and `IR_SENSOR_PITCH_MM` has no
measured value yet.
