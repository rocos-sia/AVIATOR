# aviator_tts

独立中文语音提醒节点：订阅同机 `aviator_core_managed` 的 `flight.state`，按状态变化
播放预生成 WAV。运行时无需模型、GPU、Python 或网络；使用普通用户的 `aplay` 音频设备。
完整方案见 [语音提醒设计](../../docs/AVIATOR语音提醒设计.md)。

## 配置语音提醒

编辑 [config/tts.yaml](../../config/tts.yaml)，配置项如下：

| 配置项 | 默认值 | 含义 |
| --- | --- | --- |
| `subscribe` | `tcp://127.0.0.1:5556` | Bus 的 SUB 端点，须与 Core 同机。 |
| `publisher` | `aviator_core` | 接受的状态发布者。 |
| `device` | `default` | `aplay -L` 中的 ALSA 播放设备名称。 |
| `audio_dir` | `../assets/tts/zh_CN` | 音频包目录，相对于本 YAML 所在目录。 |
| `freshness_ms` | `200` | 超过此采样/接收年龄即停止使用旧状态。 |
| `lost_after_ms` | `2000` | 状态失效后持续多久播报一次通信中断，须不小于 `freshness_ms`。 |
| `startup_grace_ms` | `15000` | 首次有效状态前的等待时间，避免正常启动时误报。 |
| `queue_ttl_ms` | `5000` | 尚未开始播放的任务有效期。 |
| `announce_initial` | `false` | 是否播报首次收到的普通状态；默认高优先级告警仍会提醒。 |
| `cues.<ID>.text` | 中文文案 | 提醒文本，最多 256 个 UTF-8 字节。 |
| `cues.<ID>.priority` | `0`–`3` | 数字越大优先级越高；默认急停为 3、故障/保护为 2。 |

例如只修改 READY 文案和设备，在原配置中修改对应项，保留其他必需项和全部 cue ID：

```yaml
device: default
announce_initial: true
cues:
  # 其余 cue 保留，此处仅展示要修改的一项
  READY: {text: "初始化完成，请进入待机状态", priority: 0}
```

`text` 修改后必须重新生成音频；只修改设备、优先级或超时无需重新合成。
配置不热加载，修改配置或替换音频包后重启 `aviator_tts`。
用 `--system-config` 时，Bus 地址覆盖 `subscribe`；显式 `--subscribe` 的优先级最高。

## 构建和日志联调

```bash
cmake -S . -B build/communication -DAVIATOR_COMMUNICATION_ONLY=ON -DAVIATOR_BUILD_EXAMPLES=OFF
cmake --build build/communication --target aviator_tts aviator_tts_test -j2
./build/communication/bin/aviator_tts --config config/tts.yaml --dry-run
```

`--dry-run` 会连接 Bus 并记录应播报的 cue，不需要音频或模型，不创建播放器。
节点默认使用配置的 `tcp://127.0.0.1:5556`；可用 `--system-config config/system.yaml`
读取现有 Bus 地址，或用 `--subscribe tcp://127.0.0.1:6556` 覆盖。
回放需隔离端点与锁，且消息应具备有效的当前时钟语义，不能直接把历史单调时间当新状态。

## 准备离线音频

当前非商业开发测试模型统一存放在 [models/tts](../../models/tts/README.md)：
`matcha-icefall-zh-baker/` 包含声学模型、词典、tokens 和规范化规则，
`vocos-22khz-univ.onnx` 为匹配的声码器。完整保留模型目录，重新生成时不再使用 `/tmp`。
模型来源与 SHA256 校验文件也位于该目录，运行安装包只携带生成的音频。
[官方模型说明](https://k2-fsa.github.io/sherpa/onnx/tts/pretrained_models/matcha.html)
中的中文 Baker 数据限非商业用途，不应直接当作商业部署的默认声音。

以下命令从项目根目录执行。首次创建独立生成环境并安装依赖，这一步需要可访问包源：

```bash
python3 -m venv .cache/tts-venv
.cache/tts-venv/bin/pip install -r tools/tts/requirements.txt
```

随后使用 [scripts/generate_tts.sh](../../scripts/generate_tts.sh) 预生成语音：

```bash
./scripts/generate_tts.sh --dry-run      # 预览命令，不生成或写文件
./scripts/generate_tts.sh                # 生成全部文案，并自动校验音频包
./scripts/generate_tts.sh --check        # 仅校验已有音频，不加载模型
./scripts/generate_tts.sh --speed 0.9     # 调整语速后重新生成
```

脚本默认使用项目内的模型、`config/tts.yaml` 和 `.cache/tts-venv/bin/python`。
若虚拟环境不存在，则使用 PATH 中的 `python3`；也可用 `TTS_PYTHON=/path/to/python`
指定已安装依赖的解释器。脚本不自动安装依赖或下载模型，资源准备完成后生成过程完全离线。
可从任意工作目录调用；默认路径始终定位到项目，显式传入的相对路径相对于调用目录。

```bash
# 自定义配置，输出位置仍由该配置的 audio_dir 决定
./scripts/generate_tts.sh --config /path/to/tts.yaml --threads 2 --speed 1.0

# 独立试听包，不覆盖当前使用的 manifest
./scripts/generate_tts.sh --output /tmp/aviator-tts-preview
./scripts/generate_tts.sh --output /tmp/aviator-tts-preview --check

# 更换模型时同时记录实际来源
./scripts/generate_tts.sh \
  --model-dir /path/to/matcha-model \
  --vocoder /path/to/vocos.onnx \
  --model-source '所选模型的来源和许可说明'
```

等价的 Python 生成命令如下，便于单独调试：

```bash
.cache/tts-venv/bin/python tools/tts/generate.py \
  --config config/tts.yaml \
  --model-dir models/tts/matcha-icefall-zh-baker \
  --vocoder models/tts/vocos-22khz-univ.onnx \
  --model-source '官方 Matcha Baker 模型，仅用于非商业开发测试；来源见 models/tts/sources.json' \
  --threads 2 --speed 1.0

# 检查文案、格式和 SHA256，不加载模型
.cache/tts-venv/bin/python tools/tts/generate.py --config config/tts.yaml --check
./build/communication/bin/aviator_tts --config config/tts.yaml --check
```

默认输出 `assets/tts/zh_CN/`，路径相对于配置文件。模型不存入该目录。
成功后目录内包含 `manifest.json` 和 16 条 WAV；WAV 文件名是内容哈希，文案对应关系在 manifest 中。
生成器 `--output` 可用于独立测试目录；部署节点需要在配置的 `audio_dir` 指向相同目录。
文案、模型、音色或语速变化会生成新的缓存文件；失败时不替换原有 manifest。
发布前试听并保存 manifest 中的来源信息。生成器不清理旧文件，可在全新目录生成最终音频包。

## 播放和启动

先用 `aplay -L` 查看设备，在 `config/tts.yaml` 设置 `device`，再测试单条文案。
`default` 使用当前用户的 ALSA 默认输出；桌面会话或后台服务的设备访问需在实际运行用户下验证。

```bash
# 1. 验证全部文案与音频文件一致；不出声、不连接 Bus
./build/communication/bin/aviator_tts --config config/tts.yaml --check
# 2. 单条试听；不需要 Bus 或机器人，测试结束自动退出
./build/communication/bin/aviator_tts --config config/tts.yaml --test-cue READY
./build/communication/bin/aviator_tts --config config/tts.yaml --test-cue EMERGENCY_STOP
# 3. 持续订阅 Managed Core 状态并播报；需要已有 Bus 和 Managed Core
./build/communication/bin/aviator_tts --config config/tts.yaml --system-config config/system.yaml
```

常驻节点按 Ctrl+C 停止。上述 `build/communication/bin` 是本页独立构建目录，
实际工程可替换为 `build/bin`、`build/debug/bin` 或 `build/release/bin`。
`--test-cue EMERGENCY_STOP` 仅播放配置文案，不发送急停命令。

```bash
# 完整系统；按实际构建目录设置 AVIATOR_BIN
AVIATOR_BIN="$PWD/build/debug/bin" ./scripts/start_aviator.sh --tts --dry-run
AVIATOR_BIN="$PWD/build/debug/bin" ./scripts/start_aviator.sh --tts
# 仿真也支持；需先生成音频包
AVIATOR_BIN="$PWD/build/debug/bin" ./scripts/start_aviator.sh --simulation --tts
```

`--check` 仅检查音频资源，不连接 Bus、不证明扬声器可用。`--test-cue ID` 播放一条后退出，
播放失败返回非零。生产模式设备播放失败只记录日志，等待下一条新提醒。
脚本 `TTS_CONFIG=/path/to/tts.yaml` 可覆盖配置，TTS 日志保存在本次目录的 `tts.log`。
缺少音频会在启动设备前报错；运行中 TTS 退出不会停止其他控制节点。
`--lock-file` 默认为 `/tmp/aviator_tts-<uid>.lock`，测试和隔离回放可另设路径。

首次普通快照默认不播报；高优先级告警立即播报。首次状态等待 15 秒，之后按 200 ms
时效撤销旧提示，再等待 2 秒播报通信中断。状态持续不变不重复提醒。状态改变会取消
旧语音，急停优先；错误内容改变也会触发当前状态提示。详细参数见配置与设计文档。
不支持 Direct Core、跨主机时钟或“每次转换必达”；不朗读原始异常文本。

## 安装与测试

运行依赖 `alsa-utils`，生成/清单检查需要 PyYAML，推理额外需要 sherpa-onnx 和 numpy。
项目依赖安装脚本包含 `alsa-utils` 和 `python3-yaml`，不会自动安装模型或推理环境。
生成 WAV 后执行项目原有 release/install 流程，节点、配置、生成工具和 `assets/tts/`
会一起安装。已安装节点优先寻找自身安装前缀内的配置，不依赖原始源码路径。

```bash
ctest --test-dir build/communication -R '^tts_' --output-on-failure
```

`tts_rules` 检查状态规则；`tts_process` 使用系统 libzmq、Python/PyYAML、ALSA null 设备
和测试播放器验证真实进程行为，无需 pyzmq、模型或扬声器。合成音频夹具仅在临时目录中，
不作为提醒声音安装。目标扬声器音量、中文发音、控制负载下的延迟仍需现场验证。
