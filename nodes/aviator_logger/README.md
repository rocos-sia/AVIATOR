# aviator_logger

独立 C++ 记录节点：订阅全部业务 JSON Topic，并根据 YAML 选择不保存图像、原始保存、压缩保存。业务 JSON 与图像分别写入两个 MCAP，图像使用独立 TCP 入口及有界队列。依赖和协议细节、实现过程见 [实现说明](../../docs/Logger配置与图像记录实现说明.md)。

`hand.state` 按原始 JSON 字节记录，仅校验公共信封，不要求旧测量别名、`enabled` 或关节量。
RH56FTP 的 `accepted_command=null`、反馈失效及 `valid=true` 但故障锁存非空的状态均可记录；
`arm_command_mode: compact/full` 不改变手部消息的内容或记录方式。

## 构建与启动

```bash
# Ubuntu 22.04；也可使用 scripts/install_dependencies.sh
sudo apt-get install libzmq3-dev nlohmann-json3-dev libyaml-cpp-dev \
  libprotobuf-dev libavcodec-dev libavutil-dev libswscale-dev libzstd-dev pkg-config
cmake -S . -B build/communication -DAVIATOR_COMMUNICATION_ONLY=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build/communication --parallel 2
ctest --test-dir build/communication --output-on-failure

./build/communication/bin/aviator_bus
# 另一个终端，工作目录为仓库根目录
./build/communication/bin/aviator_logger --config config/recording.yaml
```

仓库示例配置使用 `camera.mode: "compressed"`、`codec: h264`、`encoder: software`，适合本机无显卡环境（不传配置时仍默认禁用图像）。启用 `raw` 或 `compressed` 后，相机端也需接入记录入口。例如在已配置 RealSense/OpenCV 环境的另一个终端：

```bash
python nodes/camera/main.py \
  --config config/camera.yaml --camera-id cockpit \
  --recording-config config/recording.yaml
```

Python 相机节点不是 Logger 的运行依赖。相机在推理前提交图像记录副本，但采集和检测仍在同一循环，**不保证传感器每个 30 Hz 样本都能被应用读取**；需实机吞吐验收。先停止生产者，再停止 Logger；接收端停止时不保证排空网络/ZMQ 内尚未入应用队列的数据。

## 配置文件与优先级

仅显式传入 `--config PATH` 才读取 YAML；省略时兼容原有仅总线记录。优先级为内置默认值 → YAML → 显式命令行参数，与参数出现顺序无关。相对输出路径相对于进程工作目录；修改配置需要重启会话。

| 参数 | 默认值 / 含义 |
| --- | --- |
| `--config` | 不自动查找；读取指定 YAML。 |
| `--output` | 默认在项目根目录的 `logs/` 下自动生成 `aviator_YYYY-MM-DD_HH-MM-SS_ffffff.mcap`（本地时间，末尾为六位微秒，无 UUID），自动创建 `logs/`，不依赖启动工作目录。项目根目录为构建时的源码根目录。显式指定时覆盖 `output.path`，父目录须存在。 |
| `--image-output` | 覆盖 `output.image_path`；未设置时由数据路径派生，例如 `data.mcap` → `data.images.mcap`。仅图像启用时创建。 |
| `--subscribe` | `tcp://127.0.0.1:5556`；覆盖 `bus.subscribe_endpoint`。 |
| `--session` | 普通文本启动标记，Logger 记录会话与源会话分开，无需手动设置。 |
| `--queue-bytes` | 16777216；仅覆盖业务队列。 |
| `--receive-hwm` | 4096；仅覆盖业务 SUB HWM。 |
| `--help` / `-h` | 显示用法。 |

一次 Logger 启动对应一个记录批次。默认数据文件与图像文件共享时间前缀，例如 `aviator_2026-10-02_15-30-00_123456.mcap` 和 `aviator_2026-10-02_15-30-00_123456.images.mcap`；也可用 `--output batch_001.mcap` 指定批次名，图像文件自动命名为 `batch_001.images.mcap`（未显式设置图像路径时）。已有文件不会被覆盖。

节点消息中的 `session_id` 使用普通文本启动标记，不再生成或校验会话 UUID；Logger 可将不同节点、不同启动标记的数据记录到同一批次。`--session` 仅设置 Logger 自身的记录会话，不用于筛选来源；数据与图像 MCAP 共享此记录会话，并通过 `recording_files` 元数据保存配对路径。

完整示例见 [`recording.yaml`](../../config/recording.yaml)。可省略非版本字段，未填写部分使用内置默认值。未知/重复键、非法类型、未知模式、越界数值在启动阶段失败；数值参数不使用引号。`config_version` 必须为整数 `1`。字节预算与 Chunk 大小当前上限为 `INT_MAX`。

| `camera.mode` | RGB | 深度 | 说明 |
| --- | --- | --- | --- |
| `disabled` | 不保存 | 不保存 | 不创建图像 PULL、不分配图像队列、不初始化编码器；业务消息仍记录。 |
| `raw` | 原始 RGB8 字节 | 原始 Z16 字节 | 不缩放、不主动降采样、不压缩；这里的原始不是传感器 Bayer RAW。 |
| `compressed` | H.264 示例配置 / H.265 可选 | Zstd 无损 Z16 | RGB 为有损 YUV420P 视频，深度解压后原样恢复。 |

- `output.chunk_compression` v1 只允许 `none`；`chunk_size_bytes` 默认 4 MiB，仅为目标值。图像编码与 MCAP Chunk 压缩分开，已编码负载不再次压缩。
- `camera.sources` 的 ID 必须唯一，格式为 1–80 个字母、数字、下划线或连字符；Topic 必须为 `record.camera.<id>.rgb` / `.depth`。每个启用源要求 RGB 出现；`record_depth=true` 时还要求深度出现；缺少的流在结束元数据和 stderr 中标记 DEGRADED。
- 输入始终为 RGB8/Z16 原始帧，不接受生产者直接传入 H.264/H.265。图像通过 `5557` 的单帧 Protobuf 信封传输，禁止混入控制 JSON 总线。
- `compressed.rgb.encoder=auto` 先尝试 FFmpeg NVENC，再尝试 libx264/libx265；`hardware` 当前仅支持 NVENC，`software` 严格使用 libx264/libx265。后端不可用时失败，绝不自动切换 codec 或保存模式。编码库存在不代表硬件可用。软件分支按吞吐选预设：libx265 用 `ultrafast` 并启用 WPP，libx264 用 `veryfast`，线程数 4。
- 软件编码吞吐决定压缩模式能否成立；应与相机检测并行实测。当前配置显式使用 `h264 + software`，无需显卡。2026-09-29 本机 D436 实测 1280×720@30：约 10 秒保存并完整解码 303 帧，观察到的序号缺口、发送端和 Logger 队列丢弃均为 0；图像 MCAP 约 10.7 MB（同批原始 RGB 像素约 838 MB）。这是短时、单相机结果，文件大小依画面变化。 同机 H.265 软件编码约 5 秒的 152 帧也全部解码通过；H.264 `auto` 在 NVENC 不可用时回退到 libx264，94 帧全部解码通过。两次均未观察到丢弃或序号缺口。
- `target_bitrate_bps` 默认 8 Mbps，是平均码率目标，不是文件大小硬上限。`keyframe_interval_frames` 默认 30；B 帧固定为 0；首帧带解码参数并为关键帧。YUV420P 要求宽高均为偶数。
- 深度 `codec` 固定 `zstd`，`level` 为 1..19。帧头保留 Z16 原始 stride、字节序和 `depth_scale`，不能把伪彩色深度图冒充原始深度。
- 不修改源分辨率和帧率；RGB 实际宽高/fps/源身份/配置 ID 改变时重建编码器并开始新的可解码序列。


## 机械臂目标记录模式

仓库 `config/recording.yaml` 已选择精简记录：

```yaml
arm_command:
  mode: "compact"  # full 或 compact；省略时默认 full
```

修改后只需重启 Logger，并使用新的输出文件名。Core、Manipulator、控制总线消息和频率不变，无需重启控制节点。

- `full`：按收到的原始字节保存每条 `arm.command`，包含完整重叠窗口，适合通信排查。
- `compact`：不写入原始 `arm.command`，在每条可解析的 `arm.state` 到达时生成一条 `record.arm.target`。原始 `arm.state`、手、相机、服务及其他消息仍照常记录。

精简模块只在 Logger 写线程内工作，不发布消息。使用 `arm.state.accepted_command` 的发布者、序号、control_epoch，以及 clock_id、config_id、trajectory_id 和 tick 匹配缓存窗口；提取的目标还必须与 `execution.target` 一致。不能用窗口第一个点或最新窗口代替已接纳的对应点。

`record.arm.target` 是 MCAP 专用 JSON Topic（Schema `RecordedArmTarget/1.0`），不是控制指令，也不是实际关节测量：

| 字段 | 含义 |
| --- | --- |
| `joint_position` | 14 路目标位置，rad；顺序左臂 J1～J7、右臂 J1～J7。 |
| `joint_velocity` / `joint_acceleration` | Servo 对应点的速度、加速度，rad/s、rad/s²。 |
| `trajectory_id` / `tick` | 底层状态中的轨迹和执行游标。 |
| `source_state` / `source_command` | 原始状态身份、序号、采样时间，以及已接纳指令身份、序号、epoch；匹配成功时增加指令采样时间。 |
| `sample_mono_us` / `time_basis` | 使用 `arm.state.status_mono_us`（执行状态发布快照时间）；不是 SDK 精确执行时间，也不替代 `source_state.sample_mono_us` 的关节测量时间。无法取得执行状态时间时为 0 / unavailable。 |
| `valid` / `reason` | 是否匹配成功及原因。匹配失败时三组目标数组均为 null，不用零值冒充目标。 |
| `derivatives_available` / `streaming` | 是否含完整导数、源指令是否为 Servo。 |

普通非 Servo 窗口没有完整加速度，且其速度字段并非全部 1 ms 执行点的导数。精简模式只匹配位置（必要时按现有解码规则恢复奇数 tick 的中间位置）；速度和加速度均为 null，`reason=matched_position_only`、`derivatives_available=false`。Servo 成功匹配时三组数组完整，`reason=matched`。

记录频率随已有 `arm.state` 到达频率（当前约 100 Hz），**不保存全部 1 kHz 执行点**。保持同一游标的重复状态也保留时间记录。完整窗口、未执行的未来点和通信重传历史在 compact 文件中不可恢复。

缓存上限 64 条原始窗口（合法消息的负载总计最多 4 MiB，另有解析和对象开销）。缺失/已淘汰窗口、状态先于对应指令到达、时钟/会话不符、执行目标不一致、故障/停止及重复或乱序状态都会生成 `valid=false` 和明确原因；不延迟控制、不等待补包、不事后改写已记录样本。不同会话的指令不混用。

结束时 MCAP 元数据 `arm_target_recording` 记录省略的指令数、匹配/未匹配目标数、非法输入及缓存淘汰数；它们不等同于网络丢包数。MCAP 原有消息数统计只计算实际写入的消息。精简模式用于减少存储，网络流量、接收队列负载和图像编码不变。

## 队列、状态与退出

接收线程独占 SUB/PULL，写盘线程独占编码器和两个 MCAP Writer。业务队列默认 16 MiB，图像队列默认 256 MiB，分别计费，交替取出保证一侧不会无限抢占另一侧；单次编码仍可能延迟业务写盘。队列按负载和条目结构计费，不含分配器、ZMQ 缓冲和编码器工作区。软件编码线程数设为 4，但 x265 的 `frame-threads` 固定为 1——编码要求每帧立即出包，帧级线程会引入延迟，因此并行度主要来自 WPP，通过 `pools=4:wpp=1` 启用 4 个线程池工作线程。总内存仍需按实际分辨率与来源数量实测。

图像接收 HWM 默认 16 条，单条 Protobuf 信封上限默认 16 MiB。队列满丢新帧并计数，不等待控制线程。丢弃发生在编码前，因此后续视频帧不依赖未落盘的编码参考帧。编码器错误、编码后超限、磁盘错误均失败退出并保留 `.partial`，不继续输出可能损坏的视频序列。

启动横幅 `LISTENING` 仅表示本地入口和 Writer/编码器初始化成功，不代表生产者已连接或所有源 READY。启动日志 `Camera encoder (startup probe)` 显示初始化探测选中的编码器与预设（如 `libx264 (veryfast)`）；首帧日志与帧元数据记录实际编码器，NVENC 静默回退到软件编码时可立即发现；同一结果写入 `camera_recording` 元数据的 `encoder_backend` / `encoder_preset`。异常输出本地 DEGRADED/ERROR；结束时 MCAP `camera_recording` 元数据保存缺失流、非法信封、multipart 拒绝及队列丢弃计数。目前尚未通过业务总线发布完整状态机。显式禁用图像不算异常。

SIGINT/SIGTERM 停止接收并排空已入队数据，然后写索引与尾部、`fsync` 文件、读回检查结构及计数、原子更名、`fsync` 父目录。两个文件分别收尾，不是跨文件原子事务：若第二个文件收尾失败，第一个文件可能已经发布，仍以非零退出状态为准。拒绝覆盖最终文件和 `.partial`。失败返回非零；父目录同步失败时最终文件可能已存在，但仍报告失败。磁盘阻塞和关闭读回没有硬性时间上限；运行中尚无周期性持久化承诺。

## MCAP 映射与完整性

除 compact 模式下转换的 `arm.command` 外，业务 JSON 保留原始 Frame1 字节、空白、扩展字段和 `valid=false`，Schema 为 `类型/版本` 的公共头部 jsonschema；Channel 按 Topic、版本、发布者、源会话和时钟区分。未知 Topic/非法业务 JSON 仍只计数跳过，尚无 `record.invalid` 存档。

图像使用 `aviator.record.v1.CameraPacket`，`message_encoding=protobuf`；嵌入完整 `FileDescriptorSet`。该信封是独立的相机协议，不等同于 ICD 中尚未冻结的通用 `RecordEnvelope`。帧头、像素格式、编码器、标定信息随帧保存。现有 Foxglove 通用视频面板不能自动识别此自定义 Schema，需适配或转换。

两类消息的 `log_time` 为接收 UTC（微秒接口换算纳秒，保留时钟回退），缺少真实发布时间时 `publish_time=log_time`；源完整序号保留于负载，MCAP sequence 取低 32 位。按 Topic、发布者、源会话统计观察到的前向缺口及重复/乱序。图像额外保存接收单调时间和 Logger clock_id。

MCAP 启用 Chunk/Data/Summary CRC 与默认索引；关闭检查不是独立全文件 CRC 审计。`aviator` Metadata 保存最终有效配置，`recording_files` 保存两个输出路径；两个文件使用相同 Logger session。各文件的 `summary` 分别保存本文件的消息数、图像数和统计，`camera_recording` 保存图像专项结果。

PUB/SUB 和 PUSH/PULL 都没有持久化交付确认；ZMQ 超限拒绝、生产者/网络丢帧可能无法由 Logger 逐条计数。会话仍标记 `completeness=unverified_pubsub`；缺口统计不是最终丢失量，有损属性也不等于缺帧属性。

## 验证与剩余工作

CTest 覆盖配置校验与 CLI 覆盖、三模式 TCP 接入、原始字节、H.264/H.265 连续解码、Z16 解压一致、坏帧/multipart、队列溢出、Python/C++ 信封互通、信号退出和写盘失败。

尚未实现分卷、带哈希的外部会话清单、生产者开始/结束握手、通用设备/伺服记录、`record.invalid`、周期性持久化与崩溃恢复。D436 实机、硬件编码和长时间峰值吞吐需在目标设备验收。MCAP 固定为 `third_party/mcap-2.1.3`；示例仍保留自己的依赖副本。

## 分文件启动示例

以下从仓库根目录启动，logger 和相机读取同一份配置。输出父目录须存在，已有 MCAP 不会被覆盖。

```bash
./build/bin/aviator_logger --config config/recording.yaml \
  --output output/data.mcap --image-output output/images.mcap

# 另一个终端（使用安装了相机依赖的 Python 环境）
python nodes/camera/main.py --config config/camera.yaml \
  --recording-config config/recording.yaml --camera-id cockpit
```

`data.mcap` 包含业务 JSON（包括 `camera.detection`），`images.mcap` 包含 RGB 和按配置启用的深度帧。首帧被 Writer 接收后输出 `first image recorded`，正常停止时分别打印数据和图像条数。先停止相机，再停止 logger，使已入队图像完成写入。注意 `build/bin/config/recording.yaml` 是另一份配置；修改仓库配置后应同步，或两个进程都使用仓库配置的绝对路径。

## 状态切换请求与响应记录

Logger 自动识别总线上的 `record.service.request` 和 `record.service.reply`，分别按 common/service.hpp 的 ServiceRequest/ServiceReply 校验，写入数据 MCAP，原始收到的 JSON 字节保持不变。MCAP 嵌入独立的服务信封 Schema，按请求客户端或响应服务端身份/会话分 Channel；不要求连续消息的 sequence/valid，也不据服务记录推算连续序号缺口。错误类型或不匹配 Topic 计入 invalid。

这些消息是 Gateway 主动发布的记录副本，Logger 不直接订阅 5559 服务 socket。request_id/client_session_id 将请求、超时观察与响应关联起来；请求记录中的 gateway_observation 区分 QUEUED、NOT_SENT 和 TIMEOUT_UNKNOWN。QUEUED 不是 Core 受理；无服务端时没有虚构的 ServiceReply。数据文件的完整性仍标记 unverified_pubsub，未实现设计中的独立持久化补传通道；重启、慢订阅或队列溢出可能丢失记录。
