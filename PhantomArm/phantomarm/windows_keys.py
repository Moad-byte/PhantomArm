"""Windows-only held key detection for a MuJoCo passive viewer.

MuJoCo's key_callback supplies a key code but no reliable key-up event.
The Win32 GetAsyncKeyState API lets us poll continuously while the viewer is
foreground, so simply holding 6/Enter can increase/decrease finger pressure.
"""
import os
import sys

# Virtual key codes independent of NumLock.
VK_6 = 0x36
VK_RETURN = 0x0D


def held_gripper_keys():
    """(six_held, enter_held); ignores keys when PhantomArm is not foreground."""
    if sys.platform != 'win32':
        return False, False
    import ctypes
    from ctypes import wintypes
    u = ctypes.WinDLL('user32', use_last_error=True)
    u.GetForegroundWindow.restype = wintypes.HWND
    u.GetWindowThreadProcessId.argtypes = (wintypes.HWND, ctypes.POINTER(wintypes.DWORD))
    u.GetWindowThreadProcessId.restype = wintypes.DWORD
    u.GetAsyncKeyState.argtypes = (ctypes.c_int,)
    u.GetAsyncKeyState.restype = ctypes.c_short
    hwnd = u.GetForegroundWindow()
    if not hwnd:
        return False, False
    pid = wintypes.DWORD(0)
    u.GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
    if pid.value != os.getpid():
        return False, False
    return bool(u.GetAsyncKeyState(VK_6) & 0x8000), bool(u.GetAsyncKeyState(VK_RETURN) & 0x8000)
