# 双车目标识别与导航

中央机为两辆车各启动一个 `target_navigator`：

- `hyzx001`：订阅白车 MQTT RGB-D 快照，默认识别 `white electric kettle`。
- `jetson003`：通过现有 edge-agent RPC 获取 Aurora RGB-D，默认识别 `box`。

检测节点默认不自动触发，也不绕过车队急停、A*、VFH、安全屏障或 Lease。
检测成功后，目标位置统一转换到 `site_map`，并在目标前 `0.60 m` 发布导航目标。

## 服务和状态

```bash
# 白车
ros2 service call /hyzx001/target_navigator/detect_and_go std_srvs/srv/Trigger '{}'
ros2 topic echo /hyzx001/target_navigation/status
ros2 topic echo /hyzx001/target_pose --once

# 黑车
ros2 service call /jetson003/target_navigator/detect_and_go std_srvs/srv/Trigger '{}'
ros2 topic echo /jetson003/target_navigation/status
ros2 topic echo /jetson003/target_pose --once
```

## 修改识别目标

参数必须在触发检测前设置。SAM3 使用简短英文视觉概念通常最稳定。

```bash
ros2 param set /hyzx001/target_navigator prompt 'white electric kettle'
ros2 param set /jetson003/target_navigator prompt 'box'
```

两辆车共享一块 GPU。工作进程使用 `/tmp/aaa_sam3_gpu.lock` 串行执行 SAM3；
同时触发不会造成两个模型并发占用显存，但后触发的检测会等待前一个完成。

## 黑车部署前提

黑车 edge agent 必须能执行：

```text
/opt/edge-agent/probes/rgbd_snapshot.py
```

并发布 Aurora 对齐 RGB-D：

```text
/depth_cam/rgb0/image_raw
/depth_cam/depth0/image_raw
/depth_cam/rgb0/camera_info
```

中央机从 `.jetson003_mqtt.env` 读取 MQTT 用户名和密码，不在代码中保存凭据。
