"""Load Linux controller mappings before Carbonite enumerates gamepads."""
import ctypes
import os
from pathlib import Path
import sys
import warnings


# Keep the library loaded while Kit uses it. Kit owns GLFW shutdown after startup.
_glfw = None


def _check_controller_access():
    """Explain remote-session access failures before GLFW silently skips devices."""
    for device in Path('/sys/class/input').glob('event*/device'):
        event = Path('/dev/input') / device.parent.name
        try:
            vendor = (device / 'id/vendor').read_text().strip()
            product = (device / 'id/product').read_text().strip()
            if vendor != '045e' or product not in {'0b12', '0b13'}:
                continue
            fd = os.open(event, os.O_RDONLY | os.O_NONBLOCK)
            os.close(fd)
        except PermissionError:
            warnings.warn(
                f'Xbox controller access denied: {event}. GLFW needs the event device '
                'even if /dev/input/js0 is readable. See the README Xbox controller '
                'permissions instructions and config/99-splat-xbox.rules.')
        except OSError:
            # A controller may disconnect during enumeration.
            continue


def configure_gamepad_mappings(isaac):
    """Prime Kit's own GLFW instance, not a separately installed GLFW library.

    Must run on the main thread before SimulationApp starts. Mapping an already
    ignored device after startup does not make Carbonite register it again.
    Headless machines without a display can continue without local gamepads.
    """
    global _glfw
    if sys.platform != 'linux' or _glfw is not None:
        return
    _check_controller_access()
    # Full Isaac loads this extension's copy in preference to the kernel copy.
    # They are separate files with separate GLFW state despite identical builds.
    cached_plugins = list(Path(isaac).glob(
        'extscache/carb.windowing.plugins-*/bin/libcarb.windowing-glfw.plugin.so'))
    if len(cached_plugins) > 1:
        warnings.warn('Multiple cached windowing plugins found; cannot select gamepad library safely.')
        return
    plugin = (cached_plugins[0] if cached_plugins else
              Path(isaac) / 'kit/kernel/plugins/libcarb.windowing-glfw.plugin.so')
    mappings = Path(__file__).resolve().parent.parent / 'config/gamepad_mappings.txt'
    try:
        mapping_data = mappings.read_bytes()
        glfw = ctypes.CDLL(str(plugin))
        glfw.glfwInit.argtypes = []
        glfw.glfwInit.restype = ctypes.c_int
        glfw.glfwUpdateGamepadMappings.argtypes = [ctypes.c_char_p]
        glfw.glfwUpdateGamepadMappings.restype = ctypes.c_int
        glfw.glfwTerminate.argtypes = []
        glfw.glfwTerminate.restype = None
        if not glfw.glfwInit():
            warnings.warn('GLFW unavailable; continuing without local gamepad mappings.')
            return
        if not glfw.glfwUpdateGamepadMappings(mapping_data):
            glfw.glfwTerminate()
            warnings.warn(f'GLFW rejected gamepad mappings: {mappings}')
            return
    except (OSError, AttributeError) as exc:
        warnings.warn(f'Could not configure local gamepad mappings: {exc}')
        return
    _glfw = glfw
    print(f'Loaded Linux gamepad mappings from {mappings}', flush=True)
