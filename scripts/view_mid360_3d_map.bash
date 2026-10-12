#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
WORKSPACE_DIR="$(builtin cd "${SCRIPT_DIR}/../../.." >/dev/null && pwd)"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
export QT_QPA_PLATFORM="xcb"
export FASTRTPS_DEFAULT_PROFILES_FILE="${WORKSPACE_DIR}/install/go2_mid360_config/share/go2_mid360_config/config/fastdds_udp.xml"

exec ros2 launch go2_mid360_bringup view_3d_map.launch.py "$@"
