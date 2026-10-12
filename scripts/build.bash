#!/usr/bin/env bash
set -eo pipefail
GO2_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
source /opt/ros/humble/setup.bash
cd "${GO2_WORKSPACE}"
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
SDK_PREFIX="${GO2_WORKSPACE}/install/livox-sdk2"
cmake -S "${GO2_WORKSPACE}/src/third_party/Livox-SDK2" -B "${GO2_WORKSPACE}/build/livox_sdk2_bootstrap" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${SDK_PREFIX}"
cmake --build "${GO2_WORKSPACE}/build/livox_sdk2_bootstrap" --parallel 2
cmake --install "${GO2_WORKSPACE}/build/livox_sdk2_bootstrap"
colcon build --allow-overriding tf2 --base-paths src --symlink-install --cmake-force-configure --executor sequential --packages-skip livox_sdk2 \
  --cmake-args -DDISTRO_ROS=humble \
  -DLIVOX_LIDAR_SDK_LIBRARY="${SDK_PREFIX}/lib/liblivox_lidar_sdk_shared.so" \
  -DLIVOX_LIDAR_SDK_INCLUDE_DIR="${SDK_PREFIX}/include"
