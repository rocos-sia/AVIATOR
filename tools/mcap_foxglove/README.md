# mcap_foxglove — 把录制图像 MCAP 转成 Foxglove 能显示的 MCAP

`aviator_logger` 把相机帧记成 `aviator.record.v1.CameraPacket`（`metadata_json` +
裸 `data`）。Foxglove 能**解码**这个 schema，但它的 Image panel 只渲染一份固定的
image schema 白名单，所以在 Foxglove 里直接打开录制文件只能看到 JSON 元数据，
**看不到画面**。

本工具把同一批字节重新打包成 Image panel 认得的 schema（裸帧→`foxglove.RawImage`，
h264→`foxglove.CompressedVideo`），画面就能渲染，播放/暂停/拖动进度条/调速全部由
Foxglove 自带的时间轴提供。

## 快速开始

```sh
conda activate mcap
cd tools/mcap_foxglove

# 先看一眼文件里有什么（不开 GUI），尤其看 source 那一行是 raw 还是 h264
python mcap_to_foxglove.py ~/Downloads/data.images.mcap --info

# 转换：裸帧用默认模式
python mcap_to_foxglove.py ~/Downloads/data.images.mcap

# 转换：h264 录制用 --video（无损换壳，不重编码）
python mcap_to_foxglove.py --video ~/Downloads/data.images.mcap
# -> ~/Downloads/data.images.foxglove.mcap

# 在 Foxglove 里打开
foxglove-studio ~/Downloads/data.images.foxglove.mcap
```

打开后：`Add panel` → `Image`，topic 选 `record.camera.cockpit.rgb`，用底部播放条重播。

> 从 VSCode 内置终端启动时，环境里带着 `ELECTRON_RUN_AS_NODE=1`，Electron 会退化成
> 纯 Node 去"执行"这个 mcap 而报 `SyntaxError: Invalid or unexpected token`。用
> `env -u ELECTRON_RUN_AS_NODE foxglove-studio ...` 启动，或从普通终端启动。

## 三种输出

| 模式 | 输入 | 输出 schema | 大小（样本 / 1280×720） | 说明 |
|---|---|---|---|---|
| 默认 | 裸帧 | `foxglove.RawImage` | ≈ 源文件 | 像素**逐字节**等同录制数据，无损 |
| `--jpeg` | 裸帧 | `foxglove.CompressedImage` | 小两个量级 | JPEG q90 重编码，方便传阅；有损 |
| `--video` | h264 | `foxglove.CompressedVideo` | 比源文件略小 | 原码流**逐字节**换壳，不解码不重编码 |

`--jpeg-quality N` 可调质量（默认 90）。`--jpeg` 与 `--video` 互斥。`--info` 只打印
摘要不转换。

模式必须对上录制里的 `encoding`，对不上就报错而不是猜：把 h264 码流写进 `RawImage`
的 `rgb8` 会渲染成满屏噪声且不报错，把裸像素塞进 `CompressedVideo` 则根本解不出来。

### `--video` 为什么是无损的

`CameraPacket.data` 本来就是一条完整的 Annex B 码流（logger 侧 `libx264` 编码，
`veryfast`，`yuv420p`），而 `foxglove.CompressedVideo` 要的正是这个——工具只把它从
`CameraPacket.data` 搬到 `CompressedVideo.data`，`format` 填 `h264`，一个字节都不动。
实测本机样本 4362 帧搬完后整条流的 sha256 与源完全一致，`ffprobe` 能完整解出
4362 帧、1280×720、30 fps、`has_b_frames=0`。

Foxglove 的 h264 路径有三条硬要求，工具会在转换时校验前两条，第三条由录制端保证：

1. **Annex B**：`bitstream_format` 必须是 `annexb`，且每帧以起始码开头，否则 Image
   panel 直接报 "not in Annex B format"。长度前缀（AVCC）的流得先重新封装。
2. **一条消息一帧**：工具一帧一条消息地搬，天然满足。
3. **不能有 B 帧**（B 帧需要 lookahead），且每个关键帧消息里要带 SPS。这两条取决于
   录制时的编码器设置，换壳改不了，所以靠上面的 `ffprobe` 测试盯着。

## 时序

输出消息的 `log_time` 原样沿用录制时间戳，不做任何栅格化，因此重播保留录制时
真实的帧间距（该 h264 样本 145.407 s / 4362 帧，均值 33.34 ms，29.99 fps）。

## 为什么需要这一步

Foxglove 2.21 的 Image panel 按 schema **名字**匹配，白名单是：

```
foxglove.RawImage         (以及 foxglove_msgs/RawImage 等四种别名)
foxglove.CompressedImage  (同上)
foxglove.CompressedVideo  (同上)
sensor_msgs/Image         (以及 sensor_msgs 的其他图像类型)
```

`aviator.record.v1.CameraPacket` 不在其中，面板不会把它当图像渲染。

内嵌 schema 的做法和仓库既有约定一致（见 `schemas/README.md`：C++ 运行时构造
FileDescriptorSet，不需要 protoc）——本工具同样在 Python 里用 `descriptor_pb2`
程序化构造，不依赖 protoc。

注意字段的**线格式**必须与上游
[foxglove/schemas](https://github.com/foxglove/schemas/tree/main/schemas/proto/foxglove)
一致：`RawImage.width/height/step` 是 `fixed32` 而非 varint；`CompressedVideo` 的
字段号是 `timestamp=1 / frame_id=2 / data=3 / format=4`（和 `CompressedImage` 的
`timestamp/data/format/frame_id` 顺序不同，别照抄）。写错线格式仍然能对自己内嵌的
描述符解码，但生成的文件任何真正的 foxglove 读取器都读不对，所以有专门的契约测试
盯着（见下）。

## 依赖

专用 conda 环境 `mcap`：

```sh
conda create -n mcap python=3.11 mcap protobuf pillow pytest -y
```

运行前若 `PYTHONPATH` 里带 ROS humble 的 python3.10 路径，pytest 会去加载 ROS 的
插件而报 `No module named 'yaml'`，跑测试时用 `PYTHONPATH= python -m pytest` 清掉。

## 测试

```sh
cd tools/mcap_foxglove
PYTHONPATH= python -m pytest test_mcap_to_foxglove.py -q
```

测试刻意复刻 Foxglove 自己的解码路径：把输出的 `schema.data` 当 FileDescriptorSet、
建一个**全新的** pool、`lookupType(schema.name)`、再用结果解包——和 Foxglove 的
MCAP protobuf 读取器同一套逻辑。

对真实录制样本的端到端用例按样本实际的 `encoding` 自动选择线上跑哪个（裸帧样本跑
像素逐字节对比，h264 样本跑码流逐字节对比），样本不在本机时整体 skip。h264 那组还会
把换出来的码流喂给 `ffprobe`，断言消息数 == 解出的帧数、`has_b_frames=0`——这两个
性质光看字节看不出来，但 Foxglove 的 h264 路径要求它们。

## 限制

- `--video` 只认 `encoding: "h264"`。logger 若用 h265 或 zstd，得先在这里接解码器
  再重新编码；把编码流当裸 RGB 写出满屏乱码是更糟的结果，所以对不上的编码一律报错。
- 只处理图像 topic。通用业务 topic 的 schema 尚未冻结，`data.mcap`（状态/遥测）
  不在本工具范围内。
- 只做文件转换，**不**向总线重新发布，也不做虚拟时钟——那是 `nodes/aviator_replay`
  的目标（该目录目前只有 README 占位）。
- 录制里带完整相机内参，目前**不**写入 `foxglove.CameraCalibration` topic；3D panel
  里放相机或去畸变需要它，属于后续可加项。
