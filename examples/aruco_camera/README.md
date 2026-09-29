# aruco_camera 示例

本示例保留原有启动入口，同时支持 ChArUco 和 AprilTag。实际采集、识别、总线发布及向 Logger 发送原始图像的代码位于 [nodes/camera](../../nodes/camera/README.md)，示例脚本调用同一实现，避免两份检测逻辑逐渐不一致。

在 [config/camera.yaml](../../config/camera.yaml) 设置 `detector.type: apriltag` 或 `charuco`。每次启动只运行选中的一种检测器。默认 AprilTag 配置对应用户提供的 `tag36h11`、ID 0、50 mm 码；根据实际打印尺寸修改 `apriltag.tag_size_m`。

在两个终端分别运行：

```bash
~/miniconda3/envs/apriltag_realsense/bin/python examples/aruco_camera/camera_publisher.py \
  --config config/camera.yaml --camera-id cockpit
~/miniconda3/envs/apriltag_realsense/bin/python examples/aruco_camera/detection_subscriber.py
```

记录图像时，先启用 `config/recording.yaml` 的 `camera.mode` 并启动 Bus、Logger，再给发布脚本添加 `--recording-config config/recording.yaml`，同时给 Logger 和相机传相同 `--session` UUID。图像经 5557 送 Logger，最终和业务 JSON 写入同一个 MCAP。具体启动命令、格式和限制见 [相机节点说明](../../nodes/camera/README.md)。
