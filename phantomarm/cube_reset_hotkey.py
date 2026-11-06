"""Windows-only INSERT key edge detector for PhantomArm.

The MuJoCo passive viewer receives its own keyboard events. We deliberately
avoid its GLFW key callback and poll the Windows key state instead.
"""

import sys

if sys.platform == "win32":
    import ctypes
    _get_key_state = ctypes.windll.user32.GetAsyncKeyState
    _get_key_state.argtypes = [ctypes.c_int]
    _get_key_state.restype = ctypes.c_short
else:
    _get_key_state = None

VK_INSERT = 0x2D
_was_pressed = False


def insert_just_pressed():
    """Return True once on key-down; do not repeat while the key is held.

    Not global keyboard registration: just polls once during the regular
    simulation loop. On non-Windows systems this is deliberately disabled.
    """
    global _was_pressed
    if _get_key_state is None:
        return False
    pressed = bool(_get_key_state(VK_INSERT) & 0x8000)
    rising_edge = pressed and not _was_pressed
    _was_pressed = pressed
    return rising_edge
