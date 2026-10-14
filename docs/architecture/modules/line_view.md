# line_view

One sweep of the IR array to where the line is. Pure logic in `include/LineView.h`, tested by
`test/test_line_view/`. It takes the normalised channel values and the black mask that
`LineSense.h` produces, and the `usable` bit mask of channels the calibration accepted.

## Interface

`lineView(norm, mask, usable, pitchMm)` returns a `LineView`:

- `allBlack` - every usable channel is black: bare maze floor (or the array lifted off it).
- `allWhite` - no usable channel is black: off the line, or a plain white tile.
- `onLine` - some black and some white.
- `errMm` - centroid of the black channels in millimetres from the array centre, positive to the **left**
  (the counter-clockwise side, like heading). Only meaningful when `allWhite` is false.

## How it works

Black means the mask bit is set **and** the channel is usable, so a bit on a dead channel is not believed. The
centroid weights each black channel by its darkness (`norm`), with channel 0 (D1) leftmost and so the most
positive position.

## Invariants

- With no usable channel, `allBlack` is never true: a missing calibration never reads as "bare floor".
- The maze floor is black, so `allBlack` is the normal state in the maze, not a fault.

## Status

8 host tests. Not yet run on the robot: the array has not been calibrated, and `IR_SENSOR_PITCH_MM` has not been
measured.
