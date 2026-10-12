# 离线语音生成模型

本目录统一保存 `aviator_tts` 的中文语音生成资源，重新生成或修改文案时使用。
播放 `assets/tts/zh_CN/` 中已经生成的 WAV 不需要这些模型。

```text
models/tts/
├── matcha-icefall-zh-baker/   # 声学模型、词典、tokens 和文本规范化规则
├── vocos-22khz-univ.onnx     # 将声学特征转换成波形的声码器
├── sources.json             # 上游下载地址和来源说明
└── SHA256SUMS               # 全部 19 个上游文件的 SHA256
```

资源约 139 MiB。应保留完整的 `matcha-icefall-zh-baker` 目录，不能只保留其中的 ONNX。
上游 README 原样保留；它明确说明训练数据仅限非商业用途，当前资源用于非商业开发测试。
原始压缩包不重复存放，上游下载地址及压缩包校验值记录在 `sources.json`。

在项目根目录安装独立的生成环境并生成全部文案：

```bash
python3 -m venv .cache/tts-venv
.cache/tts-venv/bin/pip install -r tools/tts/requirements.txt
.cache/tts-venv/bin/python tools/tts/generate.py \
  --config config/tts.yaml \
  --model-dir models/tts/matcha-icefall-zh-baker \
  --vocoder models/tts/vocos-22khz-univ.onnx \
  --model-source '官方 Matcha Baker 模型，仅用于非商业开发测试；来源见 models/tts/sources.json' \
  --threads 2 --speed 1.0
.cache/tts-venv/bin/python tools/tts/generate.py --config config/tts.yaml --check
```

模型与依赖准备完成后，生成过程完全离线，输出到 `assets/tts/zh_CN/`。
核对模型文件完整性：

```bash
(cd models/tts && sha256sum --check SHA256SUMS)
```

模型存放在项目内，默认 CMake install/release 只打包生成的音频，不将本目录加入运行包。
更新模型时同步更新 `sources.json` 和 `SHA256SUMS`，然后重新生成音频并试听。
节点和设备配置见 [aviator_tts 说明](../../nodes/aviator_tts/README.md)。
