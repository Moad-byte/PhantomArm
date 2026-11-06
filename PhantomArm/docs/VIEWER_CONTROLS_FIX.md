# Viewer controls: previous issues and current layout

The original demo accidentally overlapped several MuJoCo viewer shortcuts:
`0` toggles geometry-group visibility; several letters alter rendering.
The orange cube is a **nonphysical target marker**, not an object being grasped.
Selected robot collision geometry was updated in earlier patches.

**Current v1.4 shortcuts:** read [ARROW_CONTROL_UPDATE.md](ARROW_CONTROL_UPDATE.md).

- Arrow keys = up/down Z, sideways left/right Y.
- 7/8 = forward/back X; 6 = gripper; 9 = software simulation stop.
- Space = reset, Home/Backspace = park, End/Delete = live, Enter = calibrate.
- F8–F12 remain optional alternatives.

The simulated stop is not an industrial safety mechanism. The robot gripper
is an animated two-finger mechanism without physical pick-and-place support.
