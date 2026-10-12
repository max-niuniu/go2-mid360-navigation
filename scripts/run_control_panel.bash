#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/setup_env.bash"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
exec ros2 run go2_mid360_panel control_panel "$@"
