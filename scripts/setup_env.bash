# Source this file from each terminal that will run ROS commands.
GO2_WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
if [[ ! -f "${GO2_WS_DIR}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run: bash ${GO2_WS_DIR}/src/go2-mid360-navigation/scripts/build.bash" >&2
  return 1 2>/dev/null || exit 1
fi
source /opt/ros/humble/setup.bash
source "${GO2_WS_DIR}/install/setup.bash"
export LD_LIBRARY_PATH="${GO2_WS_DIR}/install/livox-sdk2/lib:${LD_LIBRARY_PATH:-}"
