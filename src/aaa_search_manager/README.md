# aaa_search_manager — Cooperative target task layer

## Cooperative WFD preview

`cooperative_wfd` applies a centralized, multi-source Wavefront Frontier
Detection pass to `/swarm/map`.  It clusters reachable known-free boundary
cells, computes per-robot path distances through the deployed ESDF safety
gates, and assigns distinct frontier clusters with an exact one-to-one minimum
path-cost matching.

The default configuration is safe for commissioning:

- `/search/wfd/preview_goal/<robot>` publishes preview-only poses;
- `/search/wfd/markers` visualizes all frontiers and the two assignments;
- `/search/wfd/status` reports JSON state and costs;
- `publish_navigation_goals` is `false`, so the node does not write to
  `/search/exploration_goal/<robot>`.

The implementation follows the WFD structure described by Keidar and Kaminka
and the ROS `explore_lite` frontier-search conventions, while retaining AAA's
custom A*/VFH navigation and mission authority.

This ROS 2 package establishes the task-level arbitration skeleton for the cooperative search system.

## Target observation coordinate bridge

The model runtime writes a versioned `target_observation` object into
`target_pose_result.json`. The ROS-side publisher converts it into
`aaa_search_interfaces/msg/TargetObservation` on one of:

```text
/search/target_observation/hyzx001
/search/target_observation/jetson003
```

The accepted source frames are `hyzx001/base_footprint` and
`jetson003/base_link`. `target_pose_bridge` validates identity, position,
covariance, confidence, depth ratio and observation count; transforms the point
and covariance to `site_map`; publishes the complete result on
`/search/target_observation_global`; and emits the compatibility
`PoseStamped` event on `/search/target_found`.

The bridge currently uses latest central TF because MQTT telemetry is
re-stamped in the central clock domain. Phase-1 detection must therefore be
performed while the robot is stationary.

## What V1 does

- `/search/mission_query` (`std_msgs/String`) starts a mission and switches `IDLE -> EXPLORING`.
- Future frontier allocators publish exploration goals to:
  - `/search/exploration_goal/hyzx001`
  - `/search/exploration_goal/jetson003`
- `mission_manager` is the only component that forwards those goals to the existing navigation inputs:
  - `/hyzx001/goal_pose`
  - `/jetson003/goal_pose`
- `/search/target_found` (`geometry_msgs/PoseStamped`, **must be in `site_map`**) switches `EXPLORING -> TARGET_FOUND`.
- Once `TARGET_FOUND`, all later exploration goals are rejected.
- The accepted target is latched on `/search/target_pose`.
- `/search/exploration_enabled` becomes `false` at `TARGET_FOUND`.

- Once a target observation is confirmed, `approach_goal_generator` jointly
  computes two reachable, separated approach poses around the target using the
  robots' ESDF maps.
- Each approach pose is assigned to one robot and faces the detected target.
- `mission_manager` measures each robot's current straight-line distance to
  its own assigned pose. The nearer robot moves first while the other remains
  stopped.
- After the first robot reaches and dwells at its pose, the second robot is
  dispatched. The configured `navigation_order` only breaks equal-distance ties.
## Important safety boundary

This first step implements **task-level cancellation**, not physical path cancellation. The current system description documents goal input as `PoseStamped` topics, but does not document a cancel action/service for the A* / VFH navigation chain. Therefore, after `TARGET_FOUND`, the manager prevents *new* frontier goals, but an already active path may still remain in the downstream planner/controller until a later integration adds an explicit path-clear / hold mechanism.

Do not use this Step-1 package alone to test automatic stopping on moving physical robots. Keep the fleet safety gates / E-stop engaged for interface tests.

## Build

Copy both `aaa_search_interfaces` and `aaa_search_manager` into the ROS 2
workspace `src/` directory, then:

```bash
colcon build --packages-select aaa_search_interfaces aaa_search_manager
source install/setup.bash
```

## Run

Terminal 1:

```bash
ros2 launch aaa_search_manager mission_manager.launch.py
```

This launch starts both `mission_manager` and `target_pose_bridge`.

## Publish a model result

The result must contain a non-empty mission ID, a supported robot ID, and a
base-frame observation:

```bash
ros2 run aaa_search_manager publish_target_observation \
  /absolute/path/to/target_pose_result.json
```

Jetson003 must run with:

```text
CYBEREYE_MQTT_WORLD_FRAME=base_link
CYBEREYE_MQTT_ROBOT_FRAME=base_link
CYBEREYE_ROBOT_ID=jetson003
CYBEREYE_MISSION_ID=<active mission id>
```

The vision runtime takes three stationary captures by default and requires at
least two spatially consistent observations. A single-frame result does not
pass the bridge's default quality gate.

Terminal 2 — observe state:

```bash
ros2 topic echo /search/state
```

Terminal 3 — start a fake natural-language mission:

```bash
ros2 topic pub --once /search/mission_query std_msgs/msg/String "{data: 'find the black vehicle'}"
```

Expected state: `EXPLORING`.

## Verify goal ownership

Listen to the real navigation goal topic:

```bash
ros2 topic echo /hyzx001/goal_pose
```

Then publish a fake frontier goal:

```bash
ros2 run aaa_search_manager fake_frontier_goal --robot hyzx001 --x 1.5 --y 0.5
```

It should appear on `/hyzx001/goal_pose` because the mission is `EXPLORING`.

## Trigger fake target found

```bash
ros2 run aaa_search_manager fake_target_found --x 3.2 --y 1.4 --frame-id site_map
```

Expected:

- `/search/state` becomes `TARGET_FOUND`.
- `/search/exploration_enabled` becomes `false`.
- `/search/target_pose` contains `(3.2, 1.4)` in `site_map`.

Now publish another frontier goal:

```bash
ros2 run aaa_search_manager fake_frontier_goal --robot hyzx001 --x 5.0 --y 2.0
```

It must **not** be forwarded to `/hyzx001/goal_pose`.

## Reset

```bash
ros2 topic pub --once /search/reset std_msgs/msg/Empty "{}"
```

Expected state: `IDLE`.

## Acceptance criteria for Step 1

1. Startup state is `IDLE`.
2. Non-empty mission query changes state to `EXPLORING`.
3. Exploration goals are forwarded only in `EXPLORING`.
4. Goals in frames other than `site_map` are rejected.
5. A valid `/search/target_found` changes state to `TARGET_FOUND`.
6. No exploration goal is forwarded after `TARGET_FOUND`.
7. Reset returns the system to `IDLE`.
