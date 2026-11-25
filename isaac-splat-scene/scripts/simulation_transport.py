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
    # The ROS distributions below ship Fast DDS 2, including Isaac's bundled
    # Jazzy runtime. That library does not read FASTDDS_DEFAULT_PROFILES_FILE.
    # Fast DDS 3 uses the newer name; select only one alias to avoid duplicate
    # profile loading by middleware that also supports the legacy name.
    legacy_distros = {"foxy", "galactic", "humble", "iron", "jazzy"}
    variable = ("FASTRTPS_DEFAULT_PROFILES_FILE"
                if environ.get("ROS_DISTRO", "").strip().lower() in legacy_distros
                else "FASTDDS_DEFAULT_PROFILES_FILE")
    for alias in ("FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE"):
        environ.pop(alias, None)
    environ[variable] = str(profile)
    print(f"Simulation ROS transport profile (process-local, {variable}): {profile}", flush=True)
    return profile
