# 实体验收清单

## 静态检查

- `colcon test-result --verbose` 无失败；
- 中央 ROS 图不存在裸 `map/odom/base_footprint/base_link/laser/laser_link/lidar_frame`；
- 每个 TF child 只有一个发布者；
- `/clock` 不参与实体系统。

## 单车检查

- 架空轮子验证正负线速和角速度；
- 停止 scan 后 Safety Barrier 和 Lease Gate 均输出零；
- 停止 odom、断中央网络、杀死 Coordinator 均在 0.5 s 内归零；
- Gate 启动和重启后保持锁定。
- Safety Manager 启动和重启后 `/swarm/emergency_stop` 均为 `true`；
- MQTT Bridge、Lease Gate、全局急停三道锁逐一验证，任一道锁定都输出零速。

## 坐标检查

- 两车同向；hyzx001 位于 Jetson003 左侧 0.30 m，且里程计均已清零；
- `/localized_scan` 同时包含 `hyzx001/...` 与 `jetson003/...` 两类 frame；
- 两车 adapter 诊断中的 `initial_odom_zeroed` 均为 `True`；
- HYZX adapter 的 `scan_timestamp_mode` 为 `latest_odom`，
  `scan_odom_synchronized=True`；
- Jetson003 adapter 的 `scan_timestamp_mode` 为 `latest_odom`，
  `scan_odom_synchronized=True`；
- HYZX MQTT 诊断中的 `telemetry_timestamp_mode` 为 `receipt`；
- 两个 `scan_mapping_filtered` 都持续发布，且无第二个同名发布者；
- `/swarm/map` 持续发布且 `header.frame_id` 为 `site_map`；
- 同一固定墙角在两车地图上的距离误差 `<0.15 m`；
- 航向误差 `<3°`；
- 静止 10 分钟坐标不持续漂移；
- TF 中不存在裸 frame 或一个 child 的多个父节点；
- `map_fuser` 必须是导航地图 `/swarm/map` 的唯一发布者；
- `multirobot_map_merge` 只能发布对照地图 `/swarm/map_mexplore`。

## 双车动态检查

依次测试远距离并行、交叉路口、窄走廊相向和一车掉线。记录最小车距、停车延迟、
定位置信度、网络延迟和任务完成时间。首轮限速保持 0.10 m/s，并安排现场断电人员。
