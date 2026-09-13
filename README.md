# 实体双车协同 SLAM 与导航

这是一个可单独构建、单独 source、单独启动的实体双车项目。项目内已经包含：

- 两车通用的 `aaa_navigation`：MQTT 接入、扫描预处理、ESDF、A*、VFH、
  Safety Barrier、Cloud Sender 与车端安全网关；
- `aaa_real_multi_robot`：frame adapter、坐标对齐、地图融合、Lease Gate、
  车队协调、健康审计与 RViz；
- 源码版 `slam_toolbox` 和 `m-explore-ros2/multirobot_map_merge`。

运行时不再 source `aaa_ros2_navigation_hyzx` 或
`aaa_ros2_navigation_jetson003`，原项目导航能力通过项目内统一导航包保留。

## 采用的方法

- `slam_toolbox` 的 `decentralized_multirobot_slam_toolbox_node`：两车各运行一个
  命名空间隔离的 SLAM；通过全局 `/localized_scan` 交换带位姿的激光扫描。
- 项目内 `map_fuser`：严格使用同一组 `site_map -> <robot>/map` TF 和
  OccupancyGrid 原点完成坐标正确的融合，发布导航正式地图 `/swarm/map`。
- `m-explore-ros2` 的 `multirobot_map_merge`：以已知初始位姿模式生成
  `/swarm/map_mexplore`，只用于 A/B 对照，不写入导航 TF。
- 原 `alignment_manager` 以固定已知初始位姿发布
  `site_map -> <robot>/map`，不再接收 C-SLAM 在线候选。

源码固定于：

- `slam_toolbox` ros2 分支：`eee0cd5e4a161bb10f8334b5420c93876b31ca99`
- `m-explore-ros2` main 分支：`326cf8a0b487c34246bb8f3326afbcd69576dc60`

项目不读取、不构建、也不 source `/home/jyan/share/统一坐标系`。旧 Swarm-SLAM、
点云转换和 C-SLAM reference adapter 已从此副本移除。

## 当前现场初始位姿

当前约定两车同向并排，`hyzx001` 位于 `jetson003` 左侧 0.30 m：

```text
hyzx001:   x=0.00, y=0.30, yaw=0°
jetson003: x=0.00, y=0.00, yaw=0°
```

以 `hyzx001` 车头方向为 `+X`、左侧为 `+Y`。两车必须同向，中心间距量到 0.30 m。
中央 frame adapter 会把各车收到的第一帧 odom 自动归一为 `(0,0,0)`，消除底盘保留
历史 odom 原点造成的固定航向误差。去中心化 SLAM 会把 peer 的局部位姿先按这组已知
外参变换到 host 局部地图，再加入 pose graph；TF 仍保持两棵带前缀的独立子树。

## 坐标与数据流

```text
hyzx001 scan/odom  -> /hyzx001/slam_toolbox --┐
                         ↕ /localized_scan     ├-> coordinate-correct map_fuser
jetson003 scan/odom -> /jetson003/slam_toolbox┘        |
                                                     /swarm/map (site_map)

site_map -> hyzx001/map -> hyzx001/odom -> ... -> hyzx001/base_footprint
site_map -> jetson003/map -> jetson003/odom -> jetson003/base_link
```

导航仍消费 `/swarm/map` 和 `site_map`，因此原规划与控制链无需改名或改 topic。
联合 SLAM 使用独立的 `/<robot>/scan_mapping_filtered`，恢复两个单车项目已有的量程、
无效值和中值过滤；不会和导航使用的 `/<robot>/scan_filtered` 产生重复发布者。
HYZX 与 Jetson003 都使用中央机接收时间，并把 scan 配到最近一帧 odom；HYZX 建图
量程恢复为 20 m。这是本检查点保留的旧时间配对和旧量程参数。

## 构建

```bash
cd /home/jyan/share/aaa_ros2_multirobot_slam_toolbox
./fleet.sh build
```

构建脚本会先编译源码版 `slam_toolbox` 和 `multirobot_map_merge`，再编译集成包并运行
测试。无需手工 pip 安装 Open3D、CVXPY 或 Swarm-SLAM 依赖，也不需要先构建另外三个
导航工作空间。

首次接入 MQTT 时只需在本项目保存凭据：

```bash
cp .mqtt.env.example .mqtt.env
# 编辑 .mqtt.env，填入实际用户名和密码；该文件已被 .gitignore 排除。
```

## 启动

完成上述并排初始化后，最简启动为：

```bash
cd /home/jyan/share/aaa_ros2_multirobot_slam_toolbox
./fleet.sh up
```

它在 tmux 中启动两车遥测、原导航链、协同 SLAM、地图融合和 RViz；系统默认急停。
RViz 默认只显示统一地图、两色建图扫描、车辆轮廓和路径；VFH、安全屏障、里程计历史
与 TF 调试层保留在 Displays 中但默认关闭。

```bash
./fleet.sh status       # 查看窗口状态
./fleet.sh check        # 两张独立局部图、融合地图、TF、生命周期综合检查
tmux attach -t aaa_multirobot_slam
```

只有 `./fleet.sh check` 显示 PASS、RViz 中共同墙体基本重合、现场人员确认安全后，才可：

```bash
./fleet.sh enable
```

立即停车与关闭整套系统：

```bash
./fleet.sh estop
./fleet.sh down
```

也可分终端执行 `./fleet.sh hyzx`、`./fleet.sh jetson`、`./fleet.sh hyzx-nav`、
`./fleet.sh jetson-nav`、`./fleet.sh map` 和 `./fleet.sh rviz`。固定地图导航仍使用：

```bash
./fleet.sh localize /绝对路径/site_map.yaml
```

若把本项目部署到实体车本机，可使用 `hyzx-vehicle` 或 `jetson-vehicle` 启动车端
安全链；确认遥测和零速度状态正常后，再在车端执行 `./fleet.sh edge-enable`。车端
默认不解锁，`./fleet.sh edge-disable` 会立即关断并清零底盘输出。

现场部署与验收见 [部署说明](docs/DEPLOYMENT.md) 和 [验收清单](docs/VALIDATION.md)。

## 能力边界

这是适合当前两台 2D 激光实体车的“已知初始位姿协同建图”方案，不是未知初始位姿的
全局重定位器。本系统会统一地图坐标，但错误初值、里程计未清零、雷达外参错误、
时间差过大或环境无重叠仍会产生重影。首轮测试必须保持急停，先验收地图和 TF，
再测试实体运动。
