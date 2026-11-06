"""Keyboard actions for PhantomArm's passive MuJoCo viewer.

All codes follow GLFW, not Windows virtual-key constants. The native MuJoCo
viewer also sees key presses, so avoid digit 0-5, letters and punctuation.
The application exclusively advances the simulation in passive mode.
"""

KEYS = {
    54: 'gripper',       # 6
    55: 'forward',       # 7: +X relative to robot base
    56: 'backward',      # 8: -X relative to robot base
    57: 'estop',         # 9
    265: 'raise',        # UP arrow, +Z
    264: 'lower',        # DOWN arrow, -Z
    263: 'left',         # LEFT arrow, +Y (robot-left)
    262: 'right',        # RIGHT arrow, -Y (robot-right)
    32: 'reset',         # SPACE
    268: 'home',         # HOME
    269: 'live',         # END
    # Backspace intentionally UNBOUND: MuJoCo's native viewer uses it to reset the WHOLE scene.
    261: 'reset_cube',   # DELETE: grasp scene cube-only reset
    257: 'calibrate',    # ENTER
    335: 'calibrate',    # NUMPAD ENTER
    266: 'reset_cube',   # PAGE UP: grasp scene reset
    296: 'reset_cube',   # F7: grasp scene reset
    297: 'reset',        # F8
    298: 'home',         # F9
    299: 'live',         # F10
    300: 'calibrate',    # F11
    301: 'quit',         # F12
}


def action_for_key(keycode: int):
    return KEYS.get(keycode) if isinstance(keycode, int) else None
