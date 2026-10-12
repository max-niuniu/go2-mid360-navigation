#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
WORKSPACE_DIR="$(builtin cd "${SCRIPT_DIR}/../../.." >/dev/null && pwd)"

source "${SCRIPT_DIR}/setup_env.bash"

export GAZEBO_MASTER_URI="http://127.0.0.1:11345"
export GAZEBO_IP="127.0.0.1"
export GAZEBO_MODEL_DATABASE_URI=""
export ALSOFT_DRIVERS="null"
export QT_QPA_PLATFORM="xcb"
if [[ "${GO2_DDS_TRANSPORT:-default}" == "udp" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${WORKSPACE_DIR}/install/go2_mid360_config/share/go2_mid360_config/config/fastdds_udp.xml"
else
  unset FASTRTPS_DEFAULT_PROFILES_FILE
fi

exec ros2 launch go2_mid360_bringup navigation.launch.py "$@"
