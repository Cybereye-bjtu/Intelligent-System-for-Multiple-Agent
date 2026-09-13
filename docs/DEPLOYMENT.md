# 部署说明

## 1. 数据与命名空间

中央机只接收带前缀的数据：`/hyzx001/scan`、`/hyzx001/odom`、
`/jetson003/scan`、`/jetson003/odom` 以及前缀化 TF。不要同时转发车端裸
`/scan`、`/odom`、`map`、`odom`、`base_link` 或 `laser` frame。

两套 `decentralized_multirobot_slam_toolbox_node` 共享 `/localized_scan`。修改过的
上游节点会保留已有 frame 前缀，并关闭 peer base TF 广播；实体底盘 TF 的唯一所有者
仍是原 frame adapter。

## 2. 已知并排初始位姿

`hyzx001` 为 `(0, 0.30 m, 0°)`；`jetson003` 为 `(0, 0, 0°)`。这里 `+X` 是
两车车头方向，`+Y` 是左侧。两车必须同向，中心间距为 0.30 m。frame adapter
把启动后第一帧 odom 自动归零，因此不再依赖底盘是否保留历史里程计；但启动瞬间车辆
必须处于上述实测位置。建议固定机械定位夹具；手工摆放时中心误差应小于 3 cm，方向
误差小于 2°。

启动后先保持静止 10 秒，再低速走过至少两个共同的非对称结构，例如相邻墙角和门框。
不要只在长直墙、空旷场地或重复货架中判断融合质量。

## 3. 地图与导航

`map_fuser` 读取 `/<robot>/map` 和统一 TF，以 OccupancyGrid 的真实 origin 和
resolution 融合并发布 `/swarm/map` (`site_map`)。`multirobot_map_merge`
发布 `/swarm/map_mexplore`，仅供 A/B 对照。A*、ESDF、VFH、Safety Barrier、
Lease Gate、MQTT Cloud Sender 和 Fleet Coordinator 均由本项目内源码提供。

控制链仍为：

```text
/swarm/map -> A* -> VFH -> Safety Barrier -> Lease Gate
           -> Cloud Sender -> MQTT Bridge -> 实体底盘
```

系统启动默认急停。地图存在不代表坐标正确；只有共同墙体重合、两车模型位置符合现场、
`./fleet.sh check` 为 PASS 后才可解除急停。

## 4. 独立环境与凭据

中央机仅需 source ROS 2 和本项目的 `install/setup.sh`。`fleet.sh` 不读取其他
导航工作空间。MQTT 凭据放在项目根目录的 `.mqtt.env`、`.hyzx_mqtt.env` 或
`.jetson003_mqtt.env`；后加载的车辆专属文件可覆盖共享文件。不要把这些文件提交
到版本库。

## 5. 时间与网络

- 所有设备运行 chrony，建议时差小于 20 ms；超过 100 ms 停止测试。
- 实体模式固定 `use_sim_time=false`。
- 激光和里程计使用 sensor-data QoS；地图使用 reliable + transient-local。
- `/localized_scan` 会传递经筛选的激光，Wi-Fi 丢包或高延迟会降低共同约束质量。
- HYZX 和 Jetson003 使用中央机接收时间，并将 scan 匹配到最近一帧 odom。
- MQTT 诊断中的 envelope age 正常应小于 1.0 s；HYZX 超过 1.5 s 会丢弃旧数据。
- Lease 仍按原项目超时归零，不因更换建图算法而放宽。

## 6. 故障回退

出现地图双影、突然旋转、机器人图标与现场不符或 TF 报多父节点时，立即执行：

```bash
./fleet.sh estop
./fleet.sh down
```

重新按已知并排位姿摆放并清零里程计。不要用 RViz 手动拖图掩盖错误，也不要在错误融合地图上
继续实体导航。生产运行应将验收通过的 `/swarm/map` 保存后，使用 `localize` 模式导航。
