#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
WORKSPACE_DIR="$(builtin cd "${SCRIPT_DIR}/../../.." >/dev/null && pwd)"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

export GAZEBO_MASTER_URI="http://127.0.0.1:11345"
export GAZEBO_IP="127.0.0.1"
export GAZEBO_MODEL_DATABASE_URI=""
export ALSOFT_DRIVERS="null"
export QT_QPA_PLATFORM="xcb"
export LD_LIBRARY_PATH="${WORKSPACE_DIR}/install/livox-sdk2/lib:${LD_LIBRARY_PATH:-}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${WORKSPACE_DIR}/install/go2_mid360_config/share/go2_mid360_config/config/fastdds_udp.xml"

exec ros2 launch go2_fastlio_localization mapping.launch.py "$@"
