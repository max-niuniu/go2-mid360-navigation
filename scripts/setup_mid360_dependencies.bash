#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "Install ROS 2 Humble first: /opt/ros/humble/setup.bash is missing." >&2
  exit 1
fi
if [[ "${EUID}" -eq 0 ]]; then
  ROOT_CMD=()
else
  ROOT_CMD=(sudo)
fi
"${ROOT_CMD[@]}" apt update
"${ROOT_CMD[@]}" apt install -y \
  git build-essential cmake python3-colcon-common-extensions \
  python3-rosdep python3-yaml python3-numpy python3-scipy python3-pyqt5 gnome-terminal \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-xacro ros-humble-robot-localization \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-pointcloud-to-laserscan ros-humble-navigation2 \
  ros-humble-nav2-bringup ros-humble-teleop-twist-keyboard

bash "${SCRIPT_DIR}/prepare_sources.bash"
set +u
source /opt/ros/humble/setup.bash
set -u
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  "${ROOT_CMD[@]}" rosdep init
fi
rosdep update
cd "${WORKSPACE_DIR}"
rosdep install --from-paths src/go2-mid360-navigation src/third_party --ignore-src -y --rosdistro humble
bash "${SCRIPT_DIR}/build.bash"
echo "Dependencies and workspace build completed."
