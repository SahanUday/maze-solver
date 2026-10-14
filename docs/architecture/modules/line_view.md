# line_view

One sweep of the IR array to where the line is. Pure logic in `include/LineView.h`, tested by
`test/test_line_view/`. It takes the normalised channel values and black mask from `LineSense.h`,
and the `usable` bit mask of channels the calibration accepted.

## Interface

`lineView(norm, mask, usable, pitchMm)` returns a `LineView`:

- `allBlack` - every usable channel is black: bare maze floor, or the array lifted off it.
- `allWhite` - no usable channel is black: off the line, or a white tile.
- `onLine` - some black and some white.
- `errMm` - centroid of the black channels in mm from the array centre, positive to the **left**
  (counter-clockwise, like heading). Only meaningful when `allWhite` is false.

## How it works

Black means the mask bit is set **and** the channel is usable. The centroid weights each black
channel by its darkness (`norm`); channel 0 (D1) is leftmost, the most positive position.

## Invariants

- With no usable channel, `allBlack` is false and `allWhite` is true.
- The maze floor is black, so `allBlack` is the normal state in the maze, not a fault.

## Status

Host tests only. Not run on the robot: the array is not calibrated and `irPitchMm` has no measured
value yet.
