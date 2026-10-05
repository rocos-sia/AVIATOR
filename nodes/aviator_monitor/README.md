# aviator_monitor（监测与动态配置 Web UI）

只读系统监测：C++17 节点订阅 ZMQ，浏览器通过本地 HTTP 查看。页面包含“直观监测”“系统消息”和“配置”三个 Tab，视觉依据 [双 Tab 设计方案](../../docs/AVIATOR_Monitor双Tab界面设计方案.md)。没有控制发布接口，不发送 REQ/REP 操作。

## 构建与启动

```bash
cmake -S . -B build/communication -DAVIATOR_COMMUNICATION_ONLY=ON
cmake --build build/communication --target aviator_monitor --parallel
./build/communication/bin/aviator_monitor
```

本机浏览器打开 `http://127.0.0.1:8081/`，局域网其他计算机打开 `http://<运行 Monitor 的主机局域网 IP>:8081/`。HTTP 默认监听 `0.0.0.0`（所有 IPv4 网卡），可通过 `--bind` 指定监听地址；`0.0.0.0` 是监听地址，其他计算机访问时必须使用服务主机的实际 IP，`127.0.0.1` 始终指向浏览器所在的计算机。先启动 Bus 和需要观测的节点，或使用 `--subscribe` 连接隔离测试/回放总线；SIGINT/SIGTERM 正常退出。

GCC/Clang 的 Debug 构建保留调试符号，并对 Monitor 启用 `-O2`。`CONTROL` 下每 20 ms 的 81 点轨迹窗口需要较多 JSON 处理；未优化的接收路径可能积压队列，使仍在持续发送的臂、手反馈被判为过期。诊断时同时查看 `/api/state` 的 `age_ms` 和 `receive_age_ms`：后者很小而前者持续增长，表示收到的样本已滞后，不能通过放宽超时或刷新样本时间解决。

```bash
# 仅允许本机访问
./build/communication/bin/aviator_monitor --bind 127.0.0.1
# 仅监听指定网卡（替换为本机实际局域网 IP）
./build/communication/bin/aviator_monitor --bind 192.168.1.100
```

页面、API、模型和相机预览均通过同一 HTTP 地址提供，浏览器无需直接连接 ZMQ。远程电脑无需安装模型或复制网格文件；URDF 和 STL 从运行 Monitor 的主机 `/models/` 路径下载。首次打开需要下载网格，加载进度随网络速度变化。网格最多并发下载 3 项，为状态与图像查询保留浏览器连接；大文件传输采用 30 s 无发送进展超时，持续传输不会被请求头的 2 s 超时截断。若仍无法访问，先用 `ss -ltnp 'sport = :8081'` 确认监听地址，再检查局域网路由和防火墙是否允许 TCP 8081。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--bind` | `0.0.0.0` | HTTP 监听 IPv4 地址；`127.0.0.1` 限制为本机访问。 |
| `--port` | `8081` | HTTP 端口。 |
| `--subscribe` | `tcp://127.0.0.1:5556` | Bus XPUB 出口或回放总线。 |
| `--config` | 源码或安装目录的 `monitor.yaml` | 监测专用 YAML 配置；部分字段覆盖默认值。 |
| `--model-root` | 源码或安装目录的 `models/` | 包含 `urdf/aviator.urdf` 和 `meshes/` 的目录。 |
| `--preview` | 配置中的 `tcp://127.0.0.1:5561` | 独立 Camera JPEG PUB 地址；`off` 禁用订阅。 |
| `--log-dir` | 未配置 | 本次运行的节点 `.log` 目录；一键启动脚本自动传入。 |
| `--help` | — | 显示帮助。 |

源码构建默认读取 [config/monitor.yaml](../../config/monitor.yaml)；安装后优先使用可执行文件相邻的 `../share/aviator/monitor/` 资源。模型与前端文件通过精确资源清单提供，不开放任意文件读取。

## 节点日志

使用 `scripts/start_aviator.sh` 启动时，脚本通过 `--log-dir` 将本次运行的临时目录传给 Monitor（包括仿真模式）。单独启动可指定 `aviator_monitor --log-dir /tmp/aviator-start.XXXXXXXX`；未指定时日志页显示配置提示。

“日志”页左侧列出该目录下的普通 `.log` 文件，按文件名排序并自动发现后续启动的节点；右侧显示选中文件。页面可见时每 1 秒重新读取目录与选中文件，切换节点立即加载，离开页面或浏览器进入后台时暂停查询。支持跟随最新内容，也可取消勾选后手动滚动。

日志级别筛选为精确匹配，支持 trace、debug、info、warn / warning、error、critical。识别 AVIATOR 的 `[时间] [logger] [级别]` 及 spdlog 默认 `[时间] [级别]` 前缀，移除 ANSI 颜色码；无标记的外部软件输出、多行消息中没有前缀的续行均按 info 处理。日志以纯文本展示。

每次最多读取文件末尾 512 KiB，并显示其中最新 2000 行；超限时提示截断，级别筛选作用于此窗口。原始文件保留完整日志。文件清空、替换或删除会在后续刷新中反映；读取失败时保留当前内容并提示可能过期。Monitor 仅暴露指定目录中直接包含的普通 `.log` 文件，不跟随文件符号链接，不接受目录穿越路径。

`GET /api/logs` 返回 `{configured, directory, files: [{name, node}]}`；`GET /api/logs?file=<URL编码文件名>` 返回 `{name, entries: [{level, text}], truncated, max_bytes, max_lines}`。未配置目录时文件列表为空；无效文件名返回 400，不可读文件或目录返回 503。

## 动态配置

打开网页的“配置”页，修改后点击“保存并应用”：

- 常用参数表单支持五类概览消息的 `sources`、八类 Topic 的 `timeouts_ms`，以及 RGB 预览的 `endpoint`、`camera_id`、`publisher_id`、`timeout_ms`。
- “完整 YAML”编辑方式支持全部配置，包括 `arm_joints`、`hand_calibration` 和 `yoke_calibration`。有未保存修改时，需先保存或重新读取再切换编辑方式。
- 保存先校验并准备新资源，再以临时文件和原子替换写回当前 `--config` 文件，最后更新运行状态。无效配置、无法建立订阅或写文件失败时，保留原配置；网络端点连接为异步，保存成功不表示相机已在线。
- 来源和超时立即用于现有消息缓存；预览配置变化时清空旧图像、切换订阅；所有已打开页面会检测配置版本并更新三维映射与图像状态。
- 多个页面同时编辑时，旧版本保存返回冲突，防止覆盖另一页面的修改。“重新读取”获取服务当前生效配置并丢弃表单草稿，不从磁盘重载。

`--preview` 仅在启动时覆盖配置，网页显示该生效值，后续保存会将它写入 YAML。保存输出完整配置并统一格式，不保留原文件注释。旧 JSON 文件仍可通过 `--config` 读取，保存时写为 YAML；新部署统一使用 `monitor.yaml`。进程需要对配置文件所在目录有写权限，可用 `--config` 指定可写的部署配置副本。

HTTP 地址、端口、Bus 订阅地址及模型目录仍由启动参数指定，调整这些参数需重启。可访问 Monitor HTTP 的客户端可修改其监控配置；服务不提供用户认证，`--bind 127.0.0.1` 可限制为本机访问。配置更新只影响 Monitor，不向机器人发布控制命令。

### 配置接口

`GET /api/config` 返回 `{config, yaml, revision, path}`。`PUT /api/config` 使用 `Content-Type: application/json` 和 `X-Monitor-Config: 1`，请求体为 `{revision, config}`（完整配置对象）或 `{revision, yaml}`（YAML 文本，缺省项使用默认值），两种内容只能选其一。成功返回新的配置与版本；无效请求或保存失败返回 400，非同源浏览器写入返回 403，版本冲突返回 409。接口不接受任意保存路径。

## 直观监测

- 顶部显示唯一有效 `flight.state` 的运行状态、来源和当前/历史错误；来源冲突、跨时钟或过期时显示未知。
- 发布灯显示 Flight Gateway、Core、Manipulator、Inspire Hand、Camera 的消息接收新鲜度；Logger/Bus 缺少独立状态源时显示未观测。Camera 搜索目标期间发布灯仍可以绿色，检测结果单独判定。
- 完整 URDF 使用本地 Three.js + URDFLoader 加载。左右机械臂按七个实际关节反馈联动；手部实际六通道驱动条独立显示。
- 手部默认将六通道实际归一化驱动反馈映射到所加载 URDF 的关节行程，更新三维手部；提供标定曲线时优先使用曲线。显示为驱动反馈估算姿态，目标回显不充当测量。
- 优先使用 `camera.detection.steering_wheel` 的已标定 roll/pitch 驱动驾驶盘；不含该字段的旧消息才使用 Monitor 几何标定。无有效观测时实测为 `—`，RGB 仍可使用。
- 飞控指令显示百分比、物理等价值与二维指令/视觉实测对照。指令刻度 ±100%，对照图 ±110%，roll 反馈允许 ±52°，相机派生 pitch 行程为 0～170 mm（旧几何标定路径允许 −5～175 mm），映射仍为 ±50°/0～170 mm 对应 ±100%。
- 指令 pitch 显示与实际控制位移方向对齐：原始 `control.pitch` 的 −1、0、+1 分别显示 +100% / 170 mm、0% / 85 mm、−100% / 0 mm，百分比、进度条和二维标记使用同一显示方向。概览 `pitch_normalized` 保留原始指令，`pitch_percent` 与 `pitch_mm` 为显示换算值；不改变总线消息或控制逻辑。
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

[monitor.yaml](../../config/monitor.yaml) 控制概览的发布者筛选、各 Topic 时效、预览身份、关节映射和标定。`sources` 中空字符串可用于观察任意发布者，但仍会报告多源冲突；可使用发布者字符串或 `{ "publisher_id": "aviator_core" }` 筛选；旧配置的 `session_id` 字段兼容读取但不再用于筛选。仓库配置的 `sources["hand.state"]` 为当前 Modbus TCP 后端的 `rh56ftp_hand`；使用 `aviator_hand` CAN 后端时改为其 `node.publisher_id`（默认 `inspire_hand`）。网页保存后立即生效；手动编辑磁盘文件后需重启 Monitor。若消息页已有 `hand.state`，概览却显示“尚无样本”，先检查该筛选值是否与消息的 `publisher_id` 一致。

默认时效：flight 100 ms，arm 50 ms，hand.state 300 ms，hand.command 100 ms，camera 200 ms，RGB 500 ms；其他流 2 s。左右臂/手还检查侧级采样时间和反馈年龄。原始 `sample_mono_us` 在匹配本机 clock_id 时才计算年龄，未知不伪造为零。驾驶盘显示以本地接收 `camera.detection` 的时间判断时效，不因采样延迟或相机时钟域不同拒绝有效检测；停止接收达到 camera 超时后显示过期。JOYSTICK POSITION_HOLD 使用 checked_mono_us 判断显示有效期，同时保留原始采样年龄。

左右机械臂默认按 J1～J7 对应 `AR5-5_07L/R-W4C4A2_joint_1..7`；`arm_joints.left/right` 每项含 `name`、`sign`（±1）、`offset_rad`。实施部署前核实硬件方向和零偏。模型关节不存在、mimic 被直接赋值或值超出显示限位时停止该组的三维更新并提示，不静默裁剪；相机已标定方向盘观测不受名义显示限位约束，见下文。

### 驾驶盘标定

优先读取 `camera.detection.steering_wheel`，无需另配 Monitor 的 `yoke_calibration`：

- `roll_input_joint = -theta_rad`（rad），模型关节正方向与相机角度正方向相反。
- `pitch_input_joint = -translation_along_axis_m - 0.085`（m），将 `[-0.085, 0.085]` 映射到 `[0, -0.170]`；零位对应 `-0.085`。
- 概览的物理行程仍为 `(translation_along_axis_m + 0.085) × 1000` mm（0～170 mm），百分比为 `translation_along_axis_m / 0.085 × 100`，与飞控指令刻度一致。

显示使用相机提供的 `calibration_id`。收到消息 `valid=true`、`steering_wheel.valid=true` 且角度和轴向位移为有限数值时更新方向盘位姿，以本地接收时间判断是否断流。相机已完成标定，此路径不再用 `axis_match`、TRACKING 状态、`preview.camera_id` 或名义角度/位移范围二次否决观测。浏览器直接显示实测关节值，允许超出 URDF 名义限位，不裁剪、不修改模型文件。消息或方向盘观测无效、数值缺失或非有限、接收超时时停止更新并保留灰色旧姿态；`steering_wheel` 字段存在但无效时不回退到原始 pose。

默认 `yoke_calibration=null`。只有不含 `steering_wheel` 的旧消息使用下列几何标定路径；配置非空对象时必须提供全部字段：

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

检测的 camera_id 必须与 preview.camera_id 相同，避免把另一相机的位姿套用本标定。使用 `T_aircraft_camera × T_camera_tag × T_tag_yoke`，相对零位分解转动和移动；拒绝无效四元数、低置信度、过大拟合残差和超出物理容许范围的样本。此旧消息路径的显示限位在已配置驾驶盘标定时扩展到其对应的 ±52°、−5～175 mm；原始 URDF 文件不修改。当前 URDF 的 pitch −0.165～0 m 与 ICD 的物理行程不同，必须实测确认符号和零偏。

### 灵巧手标定

当前 [手节点消息格式](../aviator_hand/README.md#实际位置读取与发布) 的 `hands.{side}.joint_position` 为 null，实际位置使用 `drive_position_normalized[6]`（0～1），并与 `drive_position_raw[6]`（0～1000）校验一致性。`commanded_drive_position_normalized` 仅为指令回显，不驱动模型。

默认 `hand_calibration=null` 时，六通道按拇指旋转、拇指弯曲、食指、中指、无名指、小指，分别映射到 `{left,right}_thumb_1_joint`、`thumb_2_joint`、`index_1_joint`、`middle_1_joint`、`ring_1_joint`、`little_1_joint`。浏览器读取当前 URDF 各关节的 `lower/upper`，按 `q = lower + (1 - position) * (upper - lower)` 换算弧度：驱动值 1 为张开（下限），0 为闭合（上限）。mimic 从属关节由 loader 联动。概览 `pose_mapping=URDF_LIMITS` 表示此默认行程估算，不代表硬件角度标定。

提供非空 `hand_calibration` 时优先使用标定曲线（`pose_mapping=CALIBRATED`）。配置含 `id`、`left`、`right`；每侧为六项，按上述实际驱动通道顺序排列。每项含 `joint` 和 `knots`，knots 为 `[归一化驱动位置, URDF弧度]` 列表，驱动值严格递增并覆盖 0～1，采用分段线性插值。两种映射的 `pose_state` 均为 `ESTIMATED`；过期或无效反馈保留灰色旧姿态。当前软件 enabled 与 grasp_verified=false 不表示握持已验证。

## 系统消息与 REQ/REP

保留 Topic、发布者、会话、原始样本年龄、接收频率、最新接收间隔、完整序号、缺口和重复/乱序计数。两秒接收窗口在启动阶段逐渐增长，不表示源端精确频率。搜索与状态过滤只影响显示。

服务事务继续订阅 `record.service.request/reply` 副本，不连接 Core 5559 执行操作。按客户端和 request_id 关联，网关 QUEUED/NOT_SENT/TIMEOUT_UNKNOWN 与 Core ACCEPTED/COMPLETED/REJECTED 分列。保留 REPLY_ONLY、晚到响应、已知应答不被超时覆盖等行为。

当前 Core 长动作仅即时返回 ACCEPTED，界面不会根据后续 flight.state 自动推断某笔请求已完成。副本可能丢失，超时不能证明未执行，副本次数不是动作执行次数；观测应答间隔也不是设备执行耗时。

“暂停显示”冻结表格快照，后台继续接收，全局连接/失效状态仍更新。详情安全使用 textContent 展示，可复制；暂停期间未缓存的行详情需要恢复后查看。缓存淘汰和监控服务会话变更会使选中记录失效。

## 实现与 HTTP 接口

| 文件 | 职责 |
| --- | --- |
| main.cpp | 总线 SUB、HTTP 查询与配置更新、资源定位和退出。 |
| monitor.cpp / monitor.hpp | 原有有界流缓存、统计和服务事务关联。 |
| overview.cpp / config.cpp | 类型化概览、侧级时效、来源筛选、单位与标定映射。 |
| preview.cpp / preview_receiver.hpp | JPEG 身份、尺寸、时效与有界帧缓存，以及可动态切换的独立图像 SUB。 |
| assets.cpp | URDF/mesh 精确资源清单与原有 Cessna 路径别名。 |
| logs.cpp / logs.hpp | 本次运行日志列表、有界文件读取与级别识别。 |
| index.html / web/ | 四个 Tab、配置表单与 YAML 编辑、浏览器生命周期和 Three.js 视口。 |

查询接口使用 GET：`/`、`/api/state`、`/api/message?id=N`、`/api/overview`、`/api/model-manifest`、`/api/camera/latest`、`/api/camera/frame/{token}`，以及清单中的 `/assets/` 和 `/models/` 文件。概览 schema_version=1，包含 Monitor 会话/快照版本，每部分保留独立来源、当前值、状态、年龄及剩余有效时间。

概览最多 50 Hz，以满足 50 ms 机械臂显示时效；消息表摘要 10 Hz；RGB 最高 15 Hz；三维目标约 30 fps，受部署硬件影响。每条轮询链路不重叠请求；本地时效持续推进，按请求耗时保守扣减有效期。视口/数值不等待图像加载。

流与服务事务各最多 64 项；每流保留最新消息及最多 512 个接收时刻。HTTP 最多 8 个同时连接，请求头 4096 字节，配置请求体最多 128 KiB、YAML 最多 64 KiB，请求接收限时 2 s，响应发送采用 30 s 无进展超时；慢客户端不阻塞 SUB。

前端依赖固定为 Three.js 0.186.1、URDFLoader 0.13.1、three-viewport-gizmo 2.2.0。提交的 vendor.js 可离线运行，普通 CMake 构建与运行无需 Node.js/CDN。修改依赖时使用 Node ≥18，在 `web/` 中执行 `npm ci --ignore-scripts && npm run build:vendor`；许可见 [THIRD_PARTY_NOTICES.md](web/THIRD_PARTY_NOTICES.md)。

## 验证

```bash
cmake --build build/communication --target aviator_monitor_test aviator_monitor_overview_test aviator_monitor_config_test aviator_monitor_preview_receiver_test --parallel
ctest --test-dir build/communication -R '^monitor_' --output-on-failure
python nodes/camera/test_preview.py
python nodes/camera/test_camera.py
```

使用相机 Python 环境执行后两项，均无需真实相机。概览测试覆盖相机方向盘直接映射及端点、无效观测拒绝、旧标定兼容、物理换算、标定缺失、范围、左右侧独立时效、跨时钟、多源、JPEG 缓存和路径白名单；配置与 HTTP/TCP 测试还覆盖 YAML 往返、来源热切换、预览参数切换、非法更新、保存失败、并发版本冲突和重启持久化。

完整仿真构建的 `managed_gateway_process` 还会启动 Monitor，在进入 `CONTROL` 后持续轮询消息表和概览，检查臂、手反馈序号推进及新鲜度，覆盖轨迹窗口流量导致接收积压的回归场景。

可选真实浏览器验证（测试环境安装 playwright-core，运行节点不依赖它）：

```bash
AVIATOR_PLAYWRIGHT=/absolute/path/to/node_modules/playwright-core \
AVIATOR_TEST_PYTHON=/absolute/path/to/camera/python \
node tests/monitor_browser_test.cjs
```

该测试使用独立临时端口和 TEST ONLY 标定，加载仓库真实 URDF，检查 RGB、消息详情、暂停、相机单独过期、恢复及 HTTP 卡顿，并把实际运行截图保存到临时目录。它不连接生产总线或真实设备，也不作为硬件标定证据。

配置页可单独验证，无需相机 Python 环境：

```bash
AVIATOR_PLAYWRIGHT=/absolute/path/to/node_modules/playwright-core \
node tests/monitor_config_browser_test.cjs
```

该测试使用临时 YAML 和隔离端口，验证表单保存、YAML 校验、版本冲突、重新读取、键盘切页及窄屏布局，并保存配置页截图。

远程模型加载回归验证：

```bash
AVIATOR_PLAYWRIGHT=/absolute/path/to/node_modules/playwright-core \
node tests/monitor_remote_model_browser_test.cjs
```

该测试使用映射到本机的远程 HTTP 域名，在 2 MiB/s、40 ms 延迟下加载完整 URDF/STL，检查资源全部来自同一 HTTP 服务、模型几何完整和状态查询正常，并保存截图。`monitor_http` 另对大网格延迟读取超过 2 s，校验响应长度与 SHA-256，防止慢链路下载被截断。

日志页浏览器验证（无需设备）：

```bash
AVIATOR_PLAYWRIGHT=/absolute/path/to/node_modules/playwright-core \
node tests/monitor_logs_browser_test.cjs build/bin/aviator_monitor
```

测试覆盖节点切换、精确级别筛选、无级别输出、HTML 纯文本显示、每秒刷新、日志清空、后启动节点发现、文件删除、跟随开关、后台停止轮询及桌面/窄屏布局。`monitor_http` 同时覆盖日志级别解析、ANSI 颜色、文件替换、读取上限、非法 UTF-8、中文文件名和路径访问限制。
