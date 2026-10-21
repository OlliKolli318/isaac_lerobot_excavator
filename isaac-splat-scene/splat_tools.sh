#!/usr/bin/env bash
# Source system ROS, an optional workspace overlay, and the tools venv.
set -e
splat_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$splat_dir/splat.local.sh" ]]; then
    source "$splat_dir/splat.local.sh"
fi
# These commands need only the standard library, not a ROS installation.
case "${1:---help}" in
    -h|--help|check-policy)
        exec python3 "$splat_dir/scripts/splat_tools.py" "$@"
        ;;
esac

export ROS_DISTRO="${ROS_DISTRO:-jazzy}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/$ROS_DISTRO/setup.bash}"
ROS_LOCAL_SETUP="${ROS_LOCAL_SETUP:-}"
LEROBOT_VENV="${LEROBOT_VENV:-${VIRTUAL_ENV:-$splat_dir/../.venv}}"
if [[ ! -f "$ROS_SETUP" ]]; then
    echo "ROS setup not found: $ROS_SETUP. Install ROS 2 or set ROS_SETUP in splat.local.sh." >&2
    exit 1
fi
if [[ -n "$ROS_LOCAL_SETUP" && ! -f "$ROS_LOCAL_SETUP" ]]; then
    echo "Configured ROS_LOCAL_SETUP not found: $ROS_LOCAL_SETUP. Correct the path or unset it to use pip-installed packages." >&2
    exit 1
fi
if [[ ! -x "$LEROBOT_VENV/bin/python" || ! -f "$LEROBOT_VENV/bin/activate" ]]; then
    echo "Policy venv not found: $LEROBOT_VENV. Follow the root README setup instructions or set LEROBOT_VENV." >&2
    exit 1
fi
source "$ROS_SETUP"
if [[ -n "$ROS_LOCAL_SETUP" ]]; then
    source "$ROS_LOCAL_SETUP"
fi
source "$LEROBOT_VENV/bin/activate"
exec "$LEROBOT_VENV/bin/python" -X utf8 -B "$splat_dir/scripts/splat_tools.py" "$@"
