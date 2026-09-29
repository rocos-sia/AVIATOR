# Logger 配置与图像记录实现说明

## 目标与交付范围

将 `config/recording.yaml` 从格式草案接入实际 C++ Logger，支持三种保存模式：不保存图像、保存未压缩 RGB8/Z16、保存 H.264/H.265 RGB 与无损 Zstd 深度。业务消息与图像分别写入独立 MCAP（`--output` / `--image-output`），图像不经过控制总线。实现保留原来的命令行启动、信号退出、文件覆盖保护及 `.partial` 收尾方式。

RealSense 图像记录适配器位于 `nodes/camera`，以 Python 节点运行。其采集和推理仍共用主循环，不能据此宣称全量 30 Hz 采集保证。

## 相比配置草案的实施调整

1. **编码位置改到 Logger 非实时写盘线程。** 采集端始终发送 RGB8/Z16，Logger 独占模式选择和编码参数。避免采集端编码配置与文件中声明不一致，也避免为了切换 codec 修改相机逻辑。代价是本机/网络入口仍承载未压缩图像带宽，编码与业务写盘共享工作线程。
2. **实现相机专用 `CameraPacket`，不提前冻结通用设备信封。** ICD 的 `RecordEnvelope` 仍为草案。本次增加独立名称和 `.proto`，通过 JSON 元数据加二进制 bytes 承载图像；它与通用信封不兼容，不能混用。
3. **硬件编码范围明确为 NVIDIA NVENC。** `auto` 先探测 NVENC，再使用同 codec 的 libx264/libx265；没有隐式切换 codec。VAAPI/QSV 等后端未实现，`hardware` 不代表“任意 GPU”。软件分支按吞吐选预设（libx265 用 `ultrafast` 并启用 WPP，libx264 用 `veryfast`，线程数 4），`frame-threads` 固定为 1 以满足“每帧立即出包”。
4. **启动成功不等于相机就绪。** 初始化后显示 LISTENING；记录结束时逐源检查 RGB/深度是否出现，将缺失流存入 `camera_recording` Metadata。尚未实现生产者注册和完整 READY 状态机。
5. **编码失败采用失败退出。** 队列丢弃发生在编码之前；编码器错误或编码后超限直接保留 `.partial` 并返回非零，避免静默留下不可解码的依赖帧。

## 模块与数据路径

| 位置 | 职责 |
| --- | --- |
| `common/recording_config.hpp/.cpp` | 默认值、强类型选项、严格 YAML 解析、范围校验、有效配置快照。 |
| `common/camera_recording.hpp/.cpp` | CameraPacket 序列化/反序列化、原始帧校验、FFmpeg RGB 编码、Zstd 深度编码。 |
| `common/recording.hpp/.cpp` | JSON/Protobuf Schema 与 Channel、消息和元数据、持久化/读回/更名。 |
| `nodes/aviator_logger/main.cpp` | `--config`、显式 CLI 覆盖、信号和启动/结束输出。 |
| `nodes/aviator_logger/logger.cpp` | SUB/PULL 接收、有界双队列、编码/写盘线程、专项统计。 |
| `schemas/recording/camera_packet.proto` | 相机入口与 MCAP 图像消息的 Protobuf 结构。 |
| `nodes/camera/recording_client.py` | 相机节点的有界发送队列、单线程 PUSH 所有权、发送失败统计。 |

```text
业务生产者 → Bus XPUB :5556 → SUB → 业务队列 ──────────┐
                                                       ├→ Writer → MCAP.partial → MCAP
相机适配器 → 原始 CameraPacket → PULL :5557 → 图像队列 → 编码/直通 ┘
```

接收线程不做图像转换、视频编码或磁盘写入。两条应用队列采用独立预算和 drop-newest，工作线程交替取出两类消息。相机模式关闭时没有图像 socket、图像队列负载和编码器初始化。

编码器按 camera_id 维护一个 RGB 上下文；源发布者/会话/时钟、宽高、fps 或 config_id 改变时重建。软件编码使用低延迟配置，不使用 B 帧；要求每次输入立即产生一个完整视频帧包，否则失败。因而停止时不需要依赖延迟编码帧回填先前元数据。

## 配置读取约定

```bash
./build/communication/bin/aviator_logger \
  --config config/recording.yaml --output run.mcap
```

不传 `--config` 时保留内置仅总线默认值，不自动发现配置文件。读取顺序是内置默认值 → YAML → 显式 CLI；`--output` 放在 `--config` 之前也同样覆盖 YAML。最终输出路径、总线参数和完整图像设置存入 MCAP `aviator.effective_config`。

`config_version: 1` 必填，其他部分可省略并使用默认值。未知/重复映射键、错误类型、未知枚举、源 ID 冲突、Topic 不匹配、无效数值均失败。整数字段不接受带引号的字符串；字节预算、Chunk 大小上限为 `INT_MAX`。即使图像禁用，也校验已有图像字段，但不探测编码器。

路径相对于进程工作目录；不创建不存在的父目录。部署样例安装到 `<prefix>/share/aviator/config/recording.yaml`，必须显式传入其路径，不自动覆盖 `/etc` 配置。

## 相机二进制协议 v1

每次 PUSH 发送恰好一个 ZMQ frame，内容是 `CameraPacket` 的 Protobuf 序列化字节：

```proto
syntax = "proto3";
package aviator.record.v1;
message CameraPacket {
  string metadata_json = 1;
  bytes data = 2;
}
```

`metadata_json` 最多 64 KiB，须为无重复键的 JSON 对象，嵌套深度最多 32；`data` 是原始或编码后的图像字节。总信封长度受 `camera.max_record_bytes` 限制。MCAP 内嵌与 `.proto` 对应的完整 `FileDescriptorSet`，schema encoding 和 message encoding 均为 `protobuf`。C++ 使用 Protobuf 动态描述符，构建无需 protoc 可执行程序；修改 `.proto` 时必须同步描述符构造代码与互通测试。

入口必需元数据：

| 字段 | 类型 / 约束 |
| --- | --- |
| `version` | 正整数，当前为 1。 |
| `camera_id`、`stream` | 已配置 ID；stream 为 `rgb` 或 `depth`。Topic 由 Logger 配置映射，不信任发送端自由指定 Topic。 |
| `publisher_id`、`session_id`、`clock_id`、`config_id` | 非空字符串，最多 256 字节；session_id 为 UUID 文本。 |
| `sequence` | 正 uint64，逐源流序号；保留完整值。 |
| `timestamp_us`、`sample_mono_us` | 正整数，源快照 UTC 与源采样单调时刻；UTC 微秒检查转纳秒溢出。 |
| `width`、`height` | 正整数，上限 16384；实际帧字节数还受单条大小上限约束。 |
| `stride_bytes` | 正整数，至少 width × 每像素字节数；data 长度须严格等于 stride × height。 |
| `fps` | 1..240 的整数源帧率。当前不支持有理数帧率。 |
| `pixel_format`、`byte_order`、`encoding` | RGB 为 RGB8，深度为 Z16；byte_order 为 little，入口 encoding 固定 raw。 |
| `calibration` | 非空 JSON 对象，包含生产者提供的内参、畸变、外参快照。Logger 检查对象存在，不校验标定物理正确性。 |
| `depth_scale` | 深度必填，有限正数，单位为米/数值单位。 |

可添加 `frame_id`、设备时间戳、曝光等扩展字段，Logger 原样保留。Logger 新增 `receive_mono_us`、`receive_clock_id`；压缩模式新增 `original_size_bytes`、`encoder`、`lossless`，并更新 `encoding`。RGB 另外保存 `encoded_pixel_format=yuv420p`、`bitstream_format=annexb`、`keyframe`。原始 stride 表示输入布局，不是压缩负载布局。结束元数据 `camera_recording` 另记 `encoder_backend` / `encoder_preset`，说明实际产出该录像的编码器与预设。

RGB 解码结果是 YUV420P，允许有损；深度解压目标大小为 `original_size_bytes`，恢复原始 Z16 帧。读取外部文件时应在分配前校验解码尺寸/长度，不能无条件信任这些值。该自定义 Schema 不能直接获得 Foxglove 的通用视频面板支持，需要转换到相应视频 Schema 或实现专用 Reader。

## RealSense 相机节点接入

把 `recording.yaml` 的 `camera.mode` 设置为 `raw` 或 `compressed`，先启动 Logger，再启动：

```bash
python nodes/camera/main.py \
  --config config/camera.yaml --camera-id cockpit \
  --recording-config config/recording.yaml
```

相机逻辑 ID 必须与 `camera.sources` 一致。节点同时启用彩色流和 Z16，深度尺寸取 `camera.yaml` 的 `camera.depth.width/height`，fps 沿用彩色设置。无该选项或模式禁用时保留仅彩色检测行为。

节点在热身结束后、推理前提交 RGB/深度帧副本，RGB 从已有 BGR8 转换为 RGB8。硬件帧号作为记录源序号，检测关联帧号另存为 frame_id；深度保存比例尺与标定快照。`sample_mono_us` 是主机收到 frameset 的时间，元数据明确标记 `sample_time_basis=host_frameset_receive`，同时保留传感器 timestamp/domain；不能把主机收帧时刻冒充曝光时刻。

后台线程独占 PUSH，启用 IMMEDIATE 和非阻塞发送；应用队列预算采用 recording.yaml 图像队列大小，另有 ZMQ HWM。输出 queued-to-zmq 和 dropped，前者仅表示入 ZMQ 队列，不表示已落盘。发送器与 Logger 的配置必须一致，目前没有自动握手。若要保证读取每个传感器样本，仍需实现独立高频采集适配器。

## 失败处理与统计

- 配置错误、PULL 端口占用、请求编码器不可用、Writer 创建失败：启动失败，不输出监听成功横幅。
- 非单帧消息、非法信封、未注册源、数据布局不一致：拒绝计数，保留合法消息处理。坏帧本身暂不存档。
- 图像队列满：编码前丢弃新帧并计数，首个溢出输出 DEGRADED。其他源的独立业务队列不被图像占用。
- 编码器错误、编码后的消息超限、写盘/同步失败：返回非零，保留 partial；不静默切换原始模式或继续写不可解码序列。
- 缺少配置流：结束时 `missing_streams` 非空并标记 DEGRADED。成功关闭可读文件仍返回 0，不能以退出码 0 判断数据完整。
- final `summary.message_count` 包含 JSON 与图像；`camera_message_count` 单列图像；`rejected` 包含传输拒绝与无效图像，`invalid` 保持业务 JSON 校验失败语义。图像专项原因/数量保存在 `camera_recording` 元数据。

应用不能逐条统计 ZMQ 在 maxmsgsize/HWM 层拒绝的所有消息，首尾缺失也需生产者清单才能判定。完整性仍未获确认，不把 MCAP 可读或深度无损当作端到端无丢帧保证。

## 验证过程

验证构建使用 `AVIATOR_COMMUNICATION_ONLY=ON`，不要求机器人 SDK、D436 或 GPU。增加了以下检查：

1. YAML 默认/部分配置、重复键/未知键、枚举/范围/数值类型、CLI 优先级。
2. disabled 不占用图像端口；raw 双流逐字节读回一致。
3. H.264/H.265 软件编码各 8 帧，通过 TCP 入站、MCAP 保存/读取、FFmpeg 连续解码，核对尺寸与帧数；深度逐帧 Zstd 解压并比对字节。
4. 相机队列预算不足、非法 Protobuf、multipart、布局错误、未知源等路径。
5. Python CameraPacket 与 C++ 解析器互通，验证包含零字节/高位字节的深度负载和超过 32 位的源序号。
6. 保留现有总线、MCAP、SIGINT/SIGTERM、输出冲突及模拟写盘失败测试。

本次通信构建的 16 项 CTest 全部通过；Python 源码语法检查、`git diff --check` 和临时安装目录检查也通过。验证环境使用 yaml-cpp 0.7.0、Protobuf 3.12.4、libavcodec 58.134.100、libavutil 56.70.100、libswscale 5.9.100、Zstd 1.4.8。

测试不能替代 D436 实机验收。当前开发机无 NVIDIA GPU（仅 Intel 核显），NVENC 后端无法在此验证；软件编码路径已按 1280x720@30 实测选定（libx264 约 4.6–6.5 倍实时，libx265 约 2.5 倍），仍需在目标机器复验实际流组合、长时间编码吞吐、峰值队列/内存、慢盘和源端丢帧。当前仍为单文件，不含分卷、周期持久化、通用原始设备信封、恢复工具和完整状态发布。
