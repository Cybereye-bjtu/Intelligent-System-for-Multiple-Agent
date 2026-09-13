#!/usr/bin/env sh
set -e

workspace_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
build_base=${AAA_BUILD_BASE:-"$workspace_dir/build.dual_path"}
install_base=${AAA_INSTALL_BASE:-"$workspace_dir/install.dual_path"}
log_base=${AAA_LOG_BASE:-"$workspace_dir/log.dual_path"}
# ROS 2 Jazzy on this host is built for the system Python 3.12.  A user
# Anaconda environment may otherwise make colcon generate unusable Python 3.13
# entry points and break rosidl_adapter imports.
PATH=/usr/bin:/bin:/opt/ros/jazzy/bin:/opt/ros/humble/bin:$PATH
export PATH
# Build only against the selected base ROS and this workspace, independent of
# any colcon overlay sourced in the caller's terminal.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
if [ -f /opt/ros/jazzy/setup.sh ]; then
  . /opt/ros/jazzy/setup.sh
elif [ -f /opt/ros/humble/setup.sh ]; then
  . /opt/ros/humble/setup.sh
else
  echo "ROS 2 Jazzy or Humble was not found" >&2
  exit 1
fi

# ROS setup scripts probe optional variables such as AMENT_TRACE_SETUP_FILES.
# Enable nounset only after the selected ROS environment has been sourced.
set -u
cd "$workspace_dir"
colcon --log-base "$log_base" build --symlink-install \
  --build-base "$build_base" --install-base "$install_base" \
  --packages-select slam_toolbox multirobot_map_merge \
  --allow-overriding slam_toolbox multirobot_map_merge \
  --cmake-args -DBUILD_TESTING=OFF \
    -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
set +u
. "$install_base/setup.sh"
set -u
colcon --log-base "$log_base" build --symlink-install \
  --build-base "$build_base" --install-base "$install_base" \
  --packages-select aaa_navigation aaa_real_multi_robot_interfaces aaa_real_multi_robot aaa_search_interfaces aaa_search_manager \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
colcon --log-base "$log_base" test --build-base "$build_base" \
  --install-base "$install_base" \
  --packages-select aaa_navigation aaa_real_multi_robot_interfaces aaa_real_multi_robot aaa_search_interfaces aaa_search_manager
colcon test-result --test-result-base "$build_base" --verbose
