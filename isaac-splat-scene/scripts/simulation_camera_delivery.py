"""Camera delivery settings shared by the simulation launcher and validators."""
import json

CAMERA_QUEUE_DEPTH = 64
CAMERA_QOS = json.dumps({
    "history": "keepLast", "depth": CAMERA_QUEUE_DEPTH, "reliability": "reliable",
    "durability": "volatile", "deadline": 0.0, "lifespan": 0.0,
    "liveliness": "systemDefault", "leaseDuration": 0.0,
})


def configure_camera_delivery():
    """Apply before opening/playing a scene; Isaac caches these per publisher.

    Isaac 6.0.1's queued GPU image path shares the message/header with later
    frames. Publishing inline prevents pending sends from observing a changed
    header or image buffer. These are process settings, not persistent prefs.
    """
    import carb.settings
    settings = carb.settings.get_settings()
    settings.set("/exts/isaacsim.ros2.bridge/publish_multithreading_disabled", True)
    settings.set("/exts/isaacsim.ros2.bridge/publish_with_queue_thread", False)
    print("Simulation camera delivery: synchronous publishing; reliable image history=64.", flush=True)
