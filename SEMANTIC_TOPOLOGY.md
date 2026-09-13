# Semantic topology V1

The topology builder runs with the existing exploration launch. Each robot's
persistent detector captures one synchronized RGB-D keyframe and evaluates the
structured semantic targets against that same frame. Each structure generates
both a detector prompt and a shorter node class. Only successful results are
published on `/semantic_topology/observation/<robot>`.

Build and launch:

```sh
cd /home/jnli/.local/share/aaa_ros2_multirobot_find_target_01
source /opt/ros/jazzy/setup.sh
colcon build --packages-select aaa_search_interfaces aaa_navigation aaa_real_multi_robot aaa_search_manager --symlink-install
source install/setup.sh
ros2 launch aaa_search_manager mission_manager.launch.py
```

The per-robot navigation launch must also be running (as before); it now starts
the detector with `semantic_mapping_mode=true`. Start a search mission normally
so `/search/exploration_enabled` becomes true.

Outputs:

- `/semantic_topology/local/hyzx001` (`std_msgs/String`, JSON)
- `/semantic_topology/local/jetson003` (`std_msgs/String`, JSON)
- `/semantic_topology/fused` (`std_msgs/String`, JSON)
- `semantic_topology_output/hyzx001.json`
- `semantic_topology_output/jetson003.json`
- `semantic_topology_output/fused.json`

Confirmed nodes use compact graph IDs and ROS-like coordinate objects:

```json
{
  "id": "N1",
  "semantic_class": "school",
  "position": {"x": 2.15, "y": 3.42, "z": 0.0},
  "navigation_position": {"x": 1.88, "y": 3.29, "z": 0.0},
  "confidence": 0.93,
  "observation_count": 4
}
```

Topology graph parameters live under `semantic_topology` in
`src/aaa_search_manager/config/mission_manager.yaml`. The persistent topology
object dictionary is maintained in
`src/aaa_navigation/config/semantic_topology_objects.json` and loaded once at
startup by both robot launches. It can also be replaced at runtime by publishing
the same four-field objects to
`/semantic_topology/targets_config`:

```json
{
  "topology_objects": [
    {
      "target": "box",
      "attributes": ["white"],
      "text": "school",
      "relations": []
    },
    {
      "target": "box",
      "attributes": ["white"],
      "text": "bank",
      "relations": []
    }
  ]
}
```

In V1, `relations` must be an empty list. It never creates graph edges.

An edge is never inferred from distance or by a planner. It is emitted only
after one robot visits both valid navigation positions through a trajectory of
at least 0.30 m. Emergency stop, missing TF, and a localization jump invalidate
the current segment.

V1 is currently configured to confirm a node after one valid observation.
The topology confidence threshold is 0.50. Depth, map bounds, and
navigation-position clearance checks still apply.
