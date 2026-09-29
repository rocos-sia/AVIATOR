# mcap_foxglove — 把录制图像 MCAP 转成 Foxglove 能显示的 MCAP

`aviator_logger` 把相机帧记成 `aviator.record.v1.CameraPacket`（`metadata_json` +
裸 `data`）。Foxglove 能**解码**这个 schema，但它的 Image panel 只渲染一份固定的
image schema 白名单，所以在 Foxglove 里直接打开录制文件只能看到 JSON 元数据，
**看不到画面**。

本工具把同一批像素字节重新打包成 `foxglove.RawImage`，Image panel 就能渲染，
播放/暂停/拖动进度条/调速全部由 Foxglove 自带的时间轴提供。

## 快速开始

```sh
conda activate mcap
cd tools/mcap_foxglove

# 先看一眼文件里有什么（不开 GUI）
python mcap_to_foxglove.py ~/Downloads/data.images.mcap --info

# 转换
python mcap_to_foxglove.py ~/Downloads/data.images.mcap
# -> ~/Downloads/data.images.foxglove.mcap

# 在 Foxglove 里打开
foxglove-studio ~/Downloads/data.images.foxglove.mcap
```

打开后：`Add panel` → `Image`，topic 选 `record.camera.cockpit.rgb`，用底部播放条重播。

> 从 VSCode 内置终端启动时，环境里带着 `ELECTRON_RUN_AS_NODE=1`，Electron 会退化成
> 纯 Node 去"执行"这个 mcap 而报 `SyntaxError: Invalid or unexpected token`。用
> `env -u ELECTRON_RUN_AS_NODE foxglove-studio ...` 启动，或从普通终端启动。

## 两种输出

| 模式 | 输出 schema | 大小（64 帧 / 1280×720） | 说明 |
|---|---|---|---|
| 默认 | `foxglove.RawImage` | ≈ 源文件（177 MB） | 像素**逐字节**等同录制数据，无损 |
| `--jpeg` | `foxglove.CompressedImage` | ≈ 7 MB | JPEG q90 重编码，方便传阅；有损 |

`--jpeg-quality N` 可调质量（默认 90）。`--info` 只打印摘要不转换。

## 时序

输出消息的 `log_time` 原样沿用录制时间戳，不做任何栅格化，因此重播保留录制时
真实的帧间距（实测该样本 2.088 s / 64 帧，均值 33.14 ms，抖动 22.4–35.7 ms）。

## 为什么需要这一步

Foxglove 2.21 的 Image panel 按 schema **名字**匹配，白名单是：

```
foxglove.RawImage        (以及 foxglove_msgs/RawImage、foxglove_msgs/msg/RawImage、foxglove::RawImage)
foxglove.CompressedImage (同上四种别名)
sensor_msgs/Image        (以及 sensor_msgs 的其他图像类型)
```

`aviator.record.v1.CameraPacket` 不在其中，面板不会把它当图像渲染。

内嵌 schema 的做法和仓库既有约定一致（见 `schemas/README.md`：C++ 运行时构造
FileDescriptorSet，不需要 protoc）——本工具同样在 Python 里用 `descriptor_pb2`
程序化构造，不依赖 protoc。

注意字段的**线格式**必须与上游
[foxglove/schemas](https://github.com/foxglove/schemas/tree/main/schemas/proto/foxglove)
一致：`RawImage.width/height/step` 是 `fixed32` 而非 varint。写错线格式仍然能对
自己内嵌的描述符解码，但生成的文件任何真正的 foxglove.RawImage 读取器都读不对，
所以有专门的契约测试盯着（见下）。

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
MCAP protobuf 读取器同一套逻辑。其中包含一个对真实录制样本的端到端用例（样本不在
本机时自动 skip），断言 64 帧像素逐字节相等、时间戳与源 `log_time` 一致。

## 限制

- 只接受 `encoding: "raw"` 的帧。logger 若切成 `camera.mode: compressed`（h265/zstd），
  metadata 里的 `pixel_format` 仍报 `RGB8` 但payload 已是编码流——工具会明确报错而不是
  把编码流当裸 RGB 写出满屏乱码。要支持得先在这里接解码器。
- 只处理图像 topic。通用业务 topic 的 schema 尚未冻结，`data.mcap`（状态/遥测）
  不在本工具范围内。
- 只做文件转换，**不**向总线重新发布，也不做虚拟时钟——那是 `nodes/aviator_replay`
  的目标（该目录目前只有 README 占位）。
- 录制里带完整相机内参，目前**不**写入 `foxglove.CameraCalibration` topic；3D panel
  里放相机或去畸变需要它，属于后续可加项。
