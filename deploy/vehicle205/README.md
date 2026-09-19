# 205 型小车部署

该目录把已经在 205 实车验证的边缘导航链部署到同型号车辆。服务器继续负责
ESDF、A*、任务决策和全局重规划；车端负责路径跟踪、VFH、Safety Barrier 与
底盘 Gateway。

每辆车必须使用唯一的机器人名称。以下示例使用 `jetson004`：

```bash
cd ~/Intelligent-System-for-Multiple-Agent
./deploy/vehicle205/install.sh --robot-name jetson004
nano ~/.jetson004_mqtt.env
./deploy/vehicle205/verify.sh --robot-name jetson004
```

验证通过后，仍使用厂商原启动入口：

```bash
ros2 launch slam slam.launch.py
```

安装器只构建 `aaa_search_interfaces` 和 `aaa_navigation`，不会在小车上构建服务器端
地图融合包。它只对已知 SHA-256 的原厂 `slam.launch.py` 进行受检集成；文件版本不符时
会停止，并保留原文件。新增控制网关默认禁用，必须由服务器在健康检查通过后授权。

## 新车上线前检查

- 机器人名称、MQTT client ID 与 `edge/<robot-name>/path` 主题均唯一；
- 激光话题为 `/scan`，里程计为 `/odom`，机体坐标为 `base_link`；
- `/cmd_vel` 没有第二个运动控制发布者；
- 先完成原地转向、0.5 m 直行和组合路径低速测试；
- VFH 当前约在雷达量测 0.35 m 处介入，Safety Barrier 仍保留独立近距保护。

200/HYZX 平台不再属于新部署范围。仓库中的 HYZX 文件仅作为历史兼容代码保留，
不得作为新车模板。
