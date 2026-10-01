# 方向盘 AprilTag 标定与验证

此工具接收 `nodes/camera` 发布的 `camera.detection`。相机消息中的位姿约定为：

```text
T_camera_tag = target_to_camera
p_camera = R * p_tag + t
```

位置单位是米，四元数顺序是 `(qx, qy, qz, qw)`。两次检测得到的相对变换为：

```text
T_relative = T_current @ inverse(T_zero)
```

标定程序从 `T_relative` 的旋转部分得到方向盘运动轴；平移向量投影到该轴得到轴向平移，
垂直分量作为标定质量诊断。若存在旋转，还会估计轴线上离相机原点最近的一点。

## 标定

先启动 `aviator_bus` 和 AprilTag 相机节点，然后运行：

```bash
python3 tools/steering_wheel_calibration/calibrate.py \
  --output config/steering_wheel_calibration.yaml \
  --camera-id cockpit --tag-id 0
```

程序会等待两次回车：

1. 方向盘处于零位时记录第一组有效帧。该组位置做分量平均，姿态使用 Markley 四元数平均，
   结果写入 `zero.pose`，并定义 `theta_rad=0`、`translation_along_axis_m=0`。
2. 将方向盘移动到第二个稳定位置后记录第二组有效帧，用相同方法得到第二个平均位姿，再计算轴方向。

默认每组采集 20 帧、帧间至少间隔 20 ms；可用 `--samples` 和 `--sample-interval-ms` 调整。
姿态不能直接平均欧拉角或旋转矩阵，程序会先转为四元数，再用 Markley 特征向量法处理
四元数的 `q/-q` 符号等价性。

默认从 `tcp://127.0.0.1:5556` 订阅；`--bind` 可用于相机 `--bind` 的直连测试。建议第二个
位置包含明确的旋转，若只做纯平移，程序会用平移方向作为轴，无法确定旋转轴上一点。

生成的配置包括：

- `zero.pose.T_camera_tag`：零位多帧平均后的 4×4 齐次矩阵；
- `second.pose.T_camera_tag`：第二个标定位姿多帧平均后的 4×4 齐次矩阵；
- `zero.pose.sample_count` / `second.pose.sample_count`：参与平均的有效帧数及采样范围；
- `motion.axis_direction`：相机坐标系中的单位轴向量；
- 两帧之间的旋转角、轴向平移、完整平移向量和垂直残差。

## 验证

标定完成后持续接收检测结果并打印相对零位的角度、完整三维平移和沿轴平移。每帧还会从
当前测试点与零位的相对变换重新估计轴向量，与 YAML 中存储的 `motion.axis_direction`
比较，并输出夹角及 `axis_match`（默认允许误差 5°，可用 `--axis-tolerance-deg` 调整）：

```bash
python3 tools/steering_wheel_calibration/validate.py \
  --config config/steering_wheel_calibration.yaml

# 输出机器可读 JSON，处理 20 帧后退出
python3 tools/steering_wheel_calibration/validate.py \
  --config config/steering_wheel_calibration.yaml --json --max-msgs 20
```

输出的 `theta_rad/theta_deg` 是绕标定轴的带符号旋转，`translation_along_axis_m` 是沿轴的
带符号平移，`translation_vector_m` 是相机坐标系下的完整平移向量。`axis_match=false` 表示
当前帧重新估计的轴与存储轴偏差超过阈值；零位没有足够运动时会显示为 `unknown`。
`translation_perpendicular_norm_m` 应较小；若持续偏大，应检查 AprilTag 是否牢固贴在方向盘
上、相机是否移动、码尺寸和相机内参是否正确。
