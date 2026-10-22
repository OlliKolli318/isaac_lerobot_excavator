#!/usr/bin/env bash
# Launch with Isaac's own Python and bundled ROS libraries.
if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
    printf 'Execute this launcher instead of sourcing it:\n  bash %q\n' "${BASH_SOURCE[0]}" >&2
    # Keep the caller open even when it has enabled errexit (set -e).
    return 0
fi

set -e
splat_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$splat_dir/splat.local.sh" ]]; then
    source "$splat_dir/splat.local.sh"
fi
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    exec python3 "$splat_dir/scripts/launch_splat_simulation.py" "$@"
fi

export ISAAC_SIM_ROOT="${ISAAC_SIM_ROOT:-$HOME/isaacsim}"
# Select bundled libraries independently of the external ROS tools environment.
export ISAAC_ROS_DISTRO="${ISAAC_ROS_DISTRO:-jazzy}"
export ROS_DISTRO="$ISAAC_ROS_DISTRO"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [[ ! -x "$ISAAC_SIM_ROOT/python.sh" ]]; then
    echo "Isaac python.sh not found at $ISAAC_SIM_ROOT. Set ISAAC_SIM_ROOT in splat.local.sh." >&2
    exit 1
fi
if [[ -n "${AMENT_PREFIX_PATH:-}" || -n "${COLCON_PREFIX_PATH:-}" || -n "${CONDA_PREFIX:-}" ]]; then
    echo "Launch Isaac in a fresh terminal without sourced system ROS or Conda; this launcher uses Isaac's bundled ROS." >&2
    exit 1
fi
splat_ros_lib="$ISAAC_SIM_ROOT/exts/isaacsim.ros2.core/$ROS_DISTRO/lib"
if [[ ! -d "$splat_ros_lib" ]]; then
    echo "Isaac bundled ROS libraries not found: $splat_ros_lib" >&2
    exit 1
fi
export LD_LIBRARY_PATH="$splat_ros_lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
# An active policy venv must not override the interpreter used by python.sh.
unset PYTHONEXE PYTHONHOME VIRTUAL_ENV
exec "$ISAAC_SIM_ROOT/python.sh" "$splat_dir/scripts/launch_splat_simulation.py" "$@"
