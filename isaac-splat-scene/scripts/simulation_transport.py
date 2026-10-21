"""Configure transport for this process before the first ROS participant exists."""
import os
from pathlib import Path

DEFAULT_PROFILE = Path(__file__).resolve().parent.parent / "config" / "fastdds_simulation.xml"


def configure_simulation_transport(environ=None):
    """Keep UDP interoperability, with buffers sized for two raw RGB streams.

    KAIVURI_FASTDDS_PROFILE may select another XML file, or 'inherit' to retain
    the caller's DDS configuration. Nothing writes to global ROS settings.
    """
    environ = os.environ if environ is None else environ
    if environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp") not in (
        "rmw_fastrtps_cpp", "rmw_fastrtps_dynamic_cpp",
    ):
        return None
    requested = environ.get("KAIVURI_FASTDDS_PROFILE", "").strip()
    if requested.lower() == "inherit":
        return None
    profile = Path(requested).expanduser().resolve() if requested else DEFAULT_PROFILE
    if not profile.is_file():
        raise FileNotFoundError(f"Simulation Fast DDS profile not found: {profile}")
    # Setting both aliases loads the same XML twice in this Fast DDS version,
    # producing duplicate transport/profile errors. Use only the current name.
    environ.pop("FASTRTPS_DEFAULT_PROFILES_FILE", None)
    environ["FASTDDS_DEFAULT_PROFILES_FILE"] = str(profile)
    print(f"Simulation ROS transport profile (process-local): {profile}", flush=True)
    return profile
