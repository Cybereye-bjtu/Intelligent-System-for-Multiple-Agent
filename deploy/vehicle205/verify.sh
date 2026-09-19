#!/usr/bin/env bash
set -euo pipefail

robot_name=""
while (($#)); do
  case "$1" in
    --robot-name)
      robot_name="${2:-}"
      shift 2
      ;;
    *)
      echo "Usage: $0 --robot-name NAME" >&2
      exit 2
      ;;
  esac
done
if [[ ! "$robot_name" =~ ^[a-z][a-z0-9_]{1,31}$ ]]; then
  echo "A valid --robot-name is required." >&2
  exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$script_dir/../.." && pwd)"
config="$repo_root/src/aaa_navigation/config/${robot_name}_local_navigation.yaml"
[[ "$robot_name" == "jetson003" ]] && config="$repo_root/src/aaa_navigation/config/jetson003_local_navigation.yaml"
env_file="${HOME}/.${robot_name}_mqtt.env"

unset COLCON_CURRENT_PREFIX
source /opt/ros/humble/setup.bash
source "$repo_root/install/setup.bash"

test -f "$config"
test -f "$env_file"
if grep -q 'replace_me' "$env_file"; then
  echo "MQTT credentials are still placeholders in $env_file" >&2
  exit 1
fi

python3 -m py_compile \
  "$repo_root/src/aaa_navigation/aaa_navigation/vfh_controller.py" \
  "$repo_root/src/aaa_navigation/aaa_navigation/safety_barrier.py" \
  "$repo_root/src/aaa_navigation/aaa_navigation/cmd_vel_gateway.py"
python3 -c 'import sys, yaml; yaml.safe_load(open(sys.argv[1], encoding="utf-8"))' "$config"

for executable in mqtt_path_receiver scan_preprocessor vfh_controller safety_barrier cmd_vel_gateway; do
  ros2 pkg executables aaa_navigation | grep -q "aaa_navigation ${executable}"
done

grep -q "aaa_navigation" "${HOME}/ros2_ws/src/slam/launch/slam.launch.py"
grep -q "${robot_name}_local_navigation.yaml" "${HOME}/ros2_ws/src/slam/launch/slam.launch.py"

echo "PASS: static vehicle deployment checks completed for $robot_name"
echo "No motion command was enabled or published."
