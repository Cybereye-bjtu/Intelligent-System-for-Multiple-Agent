#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 --robot-name NAME [--slam-launch PATH]"
}

robot_name=""
slam_launch="${HOME}/ros2_ws/src/slam/launch/slam.launch.py"
while (($#)); do
  case "$1" in
    --robot-name)
      robot_name="${2:-}"
      shift 2
      ;;
    --slam-launch)
      slam_launch="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ! "$robot_name" =~ ^[a-z][a-z0-9_]{1,31}$ ]]; then
  echo "Robot name must match ^[a-z][a-z0-9_]{1,31}$" >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble was not found at /opt/ros/humble" >&2
  exit 1
fi
if [[ ! -f "$slam_launch" ]]; then
  echo "SLAM launch file was not found: $slam_launch" >&2
  exit 1
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$script_dir/../.." && pwd)"
base_config="$repo_root/src/aaa_navigation/config/jetson003_local_navigation.yaml"
robot_config="$repo_root/src/aaa_navigation/config/${robot_name}_local_navigation.yaml"
env_file="${HOME}/.${robot_name}_mqtt.env"

if [[ "$robot_name" != "jetson003" ]]; then
  sed \
    -e "s/aaa-jetson003-path-edge/aaa-${robot_name}-path-edge/g" \
    -e "s#edge/jetson003/path#edge/${robot_name}/path#g" \
    "$base_config" > "${robot_config}.tmp"
  mv "${robot_config}.tmp" "$robot_config"
fi

if [[ ! -f "$env_file" ]]; then
  install -m 600 "$script_dir/robot.env.example" "$env_file"
  echo "Created $env_file; fill in MQTT credentials before launch."
fi

unset COLCON_CURRENT_PREFIX
source /opt/ros/humble/setup.bash
cd "$repo_root"
colcon build --symlink-install \
  --packages-select aaa_search_interfaces aaa_navigation

if grep -q "${robot_name}_local_navigation.yaml" "$slam_launch"; then
  echo "SLAM launch is already integrated for $robot_name."
else
  expected_sha="7d5ba3ae5371f5377020da6436bbecbe8e8590e6a3ec3e5109e76ee56f240626"
  actual_sha="$(sha256sum "$slam_launch" | awk '{print $1}')"
  if [[ "$actual_sha" != "$expected_sha" ]]; then
    echo "Refusing to patch an unknown SLAM launch version." >&2
    echo "Expected: $expected_sha" >&2
    echo "Actual:   $actual_sha" >&2
    exit 1
  fi
  backup="${slam_launch}.pre_aaa_integration"
  if [[ ! -e "$backup" ]]; then
    cp -p "$slam_launch" "$backup"
  fi
  python3 "$script_dir/integrate_slam.py" \
    --file "$slam_launch" \
    --robot-name "$robot_name"
  echo "Integrated edge navigation into $slam_launch"
fi

echo "Installation complete. Gateway remains disabled by default."
echo "Next: edit $env_file, then run $script_dir/verify.sh --robot-name $robot_name"
