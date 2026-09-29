# 配置

双臂控制以 `system.yaml` 为入口，路径均相对各配置文件所在目录：

| 文件 | 内容 |
| --- | --- |
| system.yaml | 总线发布/订阅端点、Manipulator 服务、机器人配置路径、命令与任务超时、config_id |
| robot.yaml | mujoco/rokae 后端、窗口开关、双臂 IP 与阻抗、模型路径、Home/接近/轮盘速度及 Servo 参数 |
| posture.json | home、approach_seed、J2 范围；带 _deg 的字段单位为度 |
| grasp.json | 把手、轮盘、工具变换与几何；位置 m，四元数 wxyz |
| camera.yaml | 既有相机发布示例使用的采集与 ChArUco 参数 |
| recording.yaml | 全量记录配置占位，本次双臂迁移不读取 |

线上关节角为 rad、速度为 rad/s，位姿四元数使用显式 qx/qy/qz/qw 字段。模型放在 `models/control`。配置修改后重启节点；涉及关节映射、坐标或能力的改变同步更新 config_id，Core 和 Manipulator 必须加载一致配置。

`robot.yaml` 中 `collision_check_enabled: true` 控制 Core 的规划碰撞检查，省略时默认开启。设为 `false` 后不加载碰撞模型，并跳过 Home、预接近、抓取接近、MoveWheel、ServoWheel 共用的碰撞检查，包含自碰撞、双臂互碰和环境碰撞。关节限位、速度、通信超时与 SDK 故障检查继续生效，控制器自身保护不受影响。修改后重启 `aviator_core`；启动日志会打印当前开关状态。关闭后，规划器不再拦截模型中相互穿透的轨迹。

开发构建默认读取此目录；安装后读取 share/aviator/config。也可以用 --config 指向独立的 system.yaml。新节点不读取旧示例 build/bin/config，编译不会复制或覆盖根目录配置。默认 backend=mujoco；切换 rokae 前配置实际设备地址及控制器参数。
