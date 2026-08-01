"""Return success when this process can open the configured X11 display."""

import ctypes
import os


def main() -> int:
    display_name = os.environ.get('DISPLAY')
    if not display_name:
        return 2

    try:
        lib_x11 = ctypes.CDLL('libX11.so.6')
    except OSError:
        return 3

    lib_x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
    lib_x11.XOpenDisplay.restype = ctypes.c_void_p
    lib_x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
    lib_x11.XCloseDisplay.restype = ctypes.c_int

    display = lib_x11.XOpenDisplay(display_name.encode())
    if not display:
        return 1

    lib_x11.XCloseDisplay(display)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
