# aviator_monitor（双 Tab Web UI）

只读系统监测：C++17 节点订阅 ZMQ，浏览器通过本地 HTTP 查看。页面包含“直观监测”和“系统消息”两个 Tab，视觉依据 [双 Tab 设计方案](../../docs/AVIATOR_Monitor双Tab界面设计方案.md)。没有控制发布接口，不发送 REQ/REP 操作。

## 构建与启动

```bash
cmake -S . -B build/communication -DAVIATOR_COMMUNICATION_ONLY=ON
cmake --build build/communication --target aviator_monitor --parallel
./build/communication/bin/aviator_monitor
```

浏览器打开 `http://127.0.0.1:8081/`。先启动 Bus 和需要观测的节点，或使用 `--subscribe` 连接隔离测试/回放总线。HTTP 固定监听本机回环地址；SIGINT/SIGTERM 正常退出。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--port` | `8081` | 本地 HTTP 端口。 |
| `--subscribe` | `tcp://127.0.0.1:5556` | Bus XPUB 出口或回放总线。 |
| `--config` | 源码或安装目录的 `monitor.json` | 监测专用 JSON 配置；部分字段覆盖默认值。 |
| `--model-root` | 源码或安装目录的 `models/` | 包含 `urdf/aviator.urdf` 和 `meshes/` 的目录。 |
| `--preview` | 配置中的 `tcp://127.0.0.1:5561` | 独立 Camera JPEG PUB 地址；`off` 禁用订阅。 |
| `--help` | — | 显示帮助。 |

源码构建默认读取 [config/monitor.json](../../config/monitor.json)；安装后优先使用可执行文件相邻的 `../share/aviator/monitor/` 资源。模型与前端文件通过精确资源清单提供，不开放任意文件读取。

## 直观监测

- 顶部显示唯一有效 `flight.state` 的运行状态、来源和当前/历史错误；来源冲突、跨时钟或过期时显示未知。
- 发布灯显示 Flight Gateway、Core、Manipulator、Inspire Hand、Camera 的消息接收新鲜度；Logger/Bus 缺少独立状态源时显示未观测。Camera 搜索目标期间发布灯仍可以绿色，检测结果单独判定。
- 完整 URDF 使用本地 Three.js + URDFLoader 加载。左右机械臂按七个实际关节反馈联动；手部实际六通道驱动条独立显示。
- 手部默认将六通道实际归一化驱动反馈映射到所加载 URDF 的关节行程，更新三维手部；提供标定曲线时优先使用曲线。显示为驱动反馈估算姿态，目标回显不充当测量。
- 相机位姿经几何标定求解驾驶盘 roll/pitch。未配置标定时不进行猜测换算，实测为 `—`，RGB 仍可使用。
- 飞控指令显示百分比、物理等价值与二维指令/视觉实测对照。指令刻度 ±100%，对照图 ±110%，反馈允许 roll ±52°、pitch −5～175 mm，映射仍为 ±50°/0～170 mm 对应 ±100%。
- 过期部件保留灰色旧姿态，当前数值及图形标记取消；HTTP 断连时保留内容明确属于旧快照。每部分的时效在浏览器本地继续推进。

模型默认显示不透明的驾驶舱和机器人、网格，不显示坐标轴和关节轴。模型允许旋转、平移、缩放和复位；左下角 gizmo 与视角同步并支持点击切换视向。“坐标轴”显示所有 link 的局部坐标系（红 X、绿 Y、蓝 Z），“关节轴”显示所有非 fixed joint 的正轴箭头与转动正方向（右手定则）。驾驶舱和机器人透明度可分别调整，不能拖动机器人产生指令。源会话变化会重置相应显示插值，存在多个近期候选来源时停止该部件更新。

## 独立 RGB 预览

Camera 的 [preview 配置](../../config/camera.yaml) 默认启用独立 `tcp://127.0.0.1:5561` PUB，640×360 上限、15 fps、JPEG 质量 80；等比缩放，保留完整视野。现有相机命令即可启动预览：

```bash
python nodes/camera/main.py --config config/camera.yaml
# Camera 与 Monitor 同时覆盖预览地址时保持一致。
python nodes/camera/main.py --preview-endpoint tcp://127.0.0.1:6561
./build/communication/bin/aviator_monitor --preview tcp://127.0.0.1:6561
```

需使用相机既有 Python 环境（OpenCV、NumPy、pyzmq、PyYAML、RealSense SDK 及所选检测器）。`--preview-endpoint off` 关闭 Camera 输出；Monitor 的 `--preview off` 关闭接收。

预览由 Camera 独立工作线程缩放/编码，待编码队列只留最新帧；与 Logger `5557` PUSH/PULL 录制链路独立，不分流录制帧，不另起相机进程。原始相机采集和检测配置不变。

传输为三个 ZMQ 帧：`camera.rgb.<camera_id>`、元数据 JSON、JPEG 字节。元数据包含 version=1、encoding=jpeg、publisher_id、session_id、camera_id、clock_id、sequence、frame_id、sample_mono_us、width/height 及预览缩放信息。图像与检测保留同一 camera/session/frame 身份；当前界面显示干净 RGB，不绘制没有逐帧几何证据的检测框。

Monitor 校验身份、序号、JPEG 头部尺寸、编码和载荷限制；最多保留 4 帧/8 MiB，单帧最大 2 MiB。图像 URL 对应不可变 token，淘汰后返回 404。浏览器最多一条未完成图像拉取链路，切换到消息页或后台时暂停图像拉取和三维渲染。

## 配置与标定

[monitor.json](../../config/monitor.json) 控制概览的发布者筛选、各 Topic 时效、预览身份、关节映射和标定。`sources` 中空字符串可用于观察任意发布者，但仍会报告多源冲突；也可将某个来源配置为 `{ "publisher_id": "aviator_core", "session_id": "实际会话 UUID" }`，显式固定会话。默认匹配项目中的实际发布者，旧 Manipulator 的手状态占位不覆盖 Inspire Hand 反馈。

默认时效：flight 100 ms，arm 50 ms，hand.state 300 ms，hand.command 100 ms，camera 200 ms，RGB 500 ms；其他流 2 s。左右臂/手还检查侧级采样时间和反馈年龄。原始 `sample_mono_us` 在匹配本机 clock_id 时才计算年龄，未知不伪造为零。JOYSTICK POSITION_HOLD 使用 checked_mono_us 判断显示有效期，同时保留原始采样年龄。

左右机械臂默认按 J1～J7 对应 `AR5-5_07L/R-W4C4A2_joint_1..7`；`arm_joints.left/right` 每项含 `name`、`sign`（±1）、`offset_rad`。实施部署前核实硬件方向和零偏。模型关节不存在、mimic 被直接赋值或值超出显示限位时停止该组的三维更新并提示，不静默裁剪。

### 驾驶盘标定

默认 `yoke_calibration=null`。配置非空对象时必须提供全部字段：

| 字段 | 要求 |
| --- | --- |
| `id` | 经验证的标定版本。 |
| `aircraft_camera` | 相机坐标系到机体的变换。 |
| `tag_yoke` | 驾驶盘坐标系到标签的变换。 |
| `aircraft_yoke_zero` | 零位驾驶盘到机体的变换。 |
| 三个变换的 `position_m` / `quaternion_xyzw` | 3 元平移米值、4 元单位四元数 `[qx,qy,qz,qw]`。 |
| `roll_axis` / `pitch_axis` | 零位驾驶盘坐标系中的单位运动轴。pitch 轴随 roll 旋转。 |
| `pitch_zero_mm` | 零位的物理俯仰位移。 |
| `min_confidence` | 检测启发式阈值，0～1，不当作概率。 |
| `max_rotation_residual_deg` / `max_translation_residual_mm` | 机构拟合残差上限，正值。 |
| `model` | `roll_sign`、`roll_offset_rad`、`pitch_sign`、`pitch_offset_m`，用于物理量到显示模型的变换。 |

检测的 camera_id 必须与 preview.camera_id 相同，避免把另一相机的位姿套用本标定。使用 `T_aircraft_camera × T_camera_tag × T_tag_yoke`，相对零位分解转动和移动；拒绝无效四元数、低置信度、过大拟合残差和超出物理容许范围的样本。显示限位仅在已配置驾驶盘标定时扩展到其对应的 ±52°、−5～175 mm；原始 URDF 文件不修改。当前 URDF 的 pitch −0.165～0 m 与 ICD 的物理行程不同，必须实测确认符号和零偏。

### 灵巧手标定

当前 [手节点消息格式](../aviator_hand/README.md#实际位置读取与发布) 的 `hands.{side}.joint_position` 为 null，实际位置使用 `drive_position_normalized[6]`（0～1），并与 `drive_position_raw[6]`（0～1000）校验一致性。`commanded_drive_position_normalized` 仅为指令回显，不驱动模型。

默认 `hand_calibration=null` 时，六通道按拇指旋转、拇指弯曲、食指、中指、无名指、小指，分别映射到 `{left,right}_thumb_1_joint`、`thumb_2_joint`、`index_1_joint`、`middle_1_joint`、`ring_1_joint`、`little_1_joint`。浏览器读取当前 URDF 各关节的 `lower/upper`，按 `q = lower + (1 - position) * (upper - lower)` 换算弧度：驱动值 1 为张开（下限），0 为闭合（上限）。mimic 从属关节由 loader 联动。概览 `pose_mapping=URDF_LIMITS` 表示此默认行程估算，不代表硬件角度标定。

提供非空 `hand_calibration` 时优先使用标定曲线（`pose_mapping=CALIBRATED`）。配置含 `id`、`left`、`right`；每侧为六项，按上述实际驱动通道顺序排列。每项含 `joint` 和 `knots`，knots 为 `[归一化驱动位置, URDF弧度]` 列表，驱动值严格递增并覆盖 0～1，采用分段线性插值。两种映射的 `pose_state` 均为 `ESTIMATED`；过期或无效反馈保留灰色旧姿态。当前软件 enabled 与 grasp_verified=false 不表示握持已验证。

## 系统消息与 REQ/REP

保留 Topic、发布者、会话、原始样本年龄、接收频率、最新接收间隔、完整序号、缺口和重复/乱序计数。两秒接收窗口在启动阶段逐渐增长，不表示源端精确频率。搜索与状态过滤只影响显示。

服务事务继续订阅 `record.service.request/reply` 副本，不连接 Core 5559 执行操作。按客户端、客户端会话和 request_id 关联，网关 QUEUED/NOT_SENT/TIMEOUT_UNKNOWN 与 Core ACCEPTED/COMPLETED/REJECTED 分列。保留 REPLY_ONLY、晚到响应、已知应答不被超时覆盖等行为。

当前 Core 长动作仅即时返回 ACCEPTED，界面不会根据后续 flight.state 自动推断某笔请求已完成。副本可能丢失，超时不能证明未执行，副本次数不是动作执行次数；观测应答间隔也不是设备执行耗时。

“暂停显示”冻结表格快照，后台继续接收，全局连接/失效状态仍更新。详情安全使用 textContent 展示，可复制；暂停期间未缓存的行详情需要恢复后查看。缓存淘汰和监控服务会话变更会使选中记录失效。

## 实现与 HTTP 接口

| 文件 | 职责 |
| --- | --- |
| main.cpp | 总线与图像独立 SUB、只读 HTTP、资源定位和退出。 |
| monitor.cpp / monitor.hpp | 原有有界流缓存、统计和服务事务关联。 |
| overview.cpp / config.cpp | 类型化概览、侧级时效、来源筛选、单位与标定映射。 |
| preview.cpp | JPEG 身份、尺寸、时效与有界帧缓存。 |
| assets.cpp | URDF/mesh 精确资源清单与原有 Cessna 路径别名。 |
| index.html / web/ | 双 Tab、布局、浏览器生命周期和 Three.js 视口。 |

只支持 GET：`/`、`/api/state`、`/api/message?id=N`、`/api/overview`、`/api/model-manifest`、`/api/camera/latest`、`/api/camera/frame/{token}`，以及清单中的 `/assets/` 和 `/models/` 文件。概览 schema_version=1，包含 Monitor 会话/快照版本，每部分保留独立来源、当前值、状态、年龄及剩余有效时间。

概览最多 50 Hz，以满足 50 ms 机械臂显示时效；消息表摘要 10 Hz；RGB 最高 15 Hz；三维目标约 30 fps，受部署硬件影响。每条轮询链路不重叠请求；本地时效持续推进，按请求耗时保守扣减有效期。视口/数值不等待图像加载。

流与服务事务各最多 64 项；每流保留最新消息及最多 512 个接收时刻。HTTP 最多 8 个同时连接，请求头 4096 字节，整个请求 2 s；慢客户端不阻塞 SUB。

前端依赖固定为 Three.js 0.186.1、URDFLoader 0.13.1、three-viewport-gizmo 2.2.0。提交的 vendor.js 可离线运行，普通 CMake 构建与运行无需 Node.js/CDN。修改依赖时使用 Node ≥18，在 `web/` 中执行 `npm ci --ignore-scripts && npm run build:vendor`；许可见 [THIRD_PARTY_NOTICES.md](web/THIRD_PARTY_NOTICES.md)。

## 验证

```bash
cmake --build build/communication --target aviator_monitor_test aviator_monitor_overview_test --parallel
ctest --test-dir build/communication -R '^monitor_' --output-on-failure
python nodes/camera/test_preview.py
python nodes/camera/test_camera.py
```

使用相机 Python 环境执行后两项，均无需真实相机。概览测试覆盖物理换算、标定缺失、范围、左右侧独立时效、跨时钟、多源、JPEG 缓存和路径白名单；原有缓存/服务及 HTTP/TCP 测试继续保留。

可选真实浏览器验证（测试环境安装 playwright-core，运行节点不依赖它）：

```bash
AVIATOR_PLAYWRIGHT=/absolute/path/to/node_modules/playwright-core \
AVIATOR_TEST_PYTHON=/absolute/path/to/camera/python \
node tests/monitor_browser_test.cjs
```

该测试使用独立临时端口和 TEST ONLY 标定，加载仓库真实 URDF，检查 RGB、消息详情、暂停、相机单独过期、恢复及 HTTP 卡顿，并把实际运行截图保存到临时目录。它不连接生产总线或真实设备，也不作为硬件标定证据。
