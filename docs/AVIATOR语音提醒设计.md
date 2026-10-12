# AVIATOR 语音提醒设计

`aviator_tts` 是独立的、只读的整机状态提醒进程。节点订阅 Managed Core 的
`flight.state`，检测有效状态变化，按优先级播放本地 WAV。固定文案由 sherpa-onnx
提前离线合成；运行节点不加载模型、不访问网络，也不参与动作执行或控制授权。

## 接入现有系统

```text
aviator_core_managed → aviator_bus → aviator_tts → aplay → 扬声器
                                       ↑
tts.yaml → 本地 sherpa-onnx 生成工具 → WAV 和 manifest.json
```

Core 在 `RemoteLink.cpp` 中约每 20 ms 发布一次 `flight.state`。状态机报告位于
`StateMachineRuntime.cpp`，消息公共头包含发布者、启动实例、序号、采样单调时钟和
`valid`；整机状态位于消息的 `system` 对象。公共库的 `Message.body` 是解码后的表示，
线上 JSON 沿用现有扁平公共头格式，不额外增加 `body` 包装。

语音节点只订阅 `flight.state`，按 `system.state`、`state_code`、`generation`、
`current_error` 识别当前 Managed 状态。状态码遵循 `RobotStateMachine::stateCode()`：
INIT=0、STANDBY=1、GRASPING=2、FOLLOWING=3、CONTROL=4、SAFE=5、ERROR=6、
EMERGENCY_STOP=7、RELEASING=8、INITIALIZING=10、READY=11、HOMING=12。
Direct Core 没有这些完整字段，不能按整机状态播报。以后协议可增加显式状态模型标识。

首版要求与 Core 同一主机、同一单调时钟域；不以跨主机 UTC 或接收时刻伪造采样时效。
`system.state` Topic 的节点生命周期心跳不作为业务状态替代来源。

## 离线语音方案

| 方案 | 项目取舍 |
| --- | --- |
| sherpa-onnx 加 Matcha 和 Vocos | 首选离线生成器。中文短句提前合成，C++ 节点无需携带推理依赖。 |
| Piper | 本地神经网络 TTS 备选，具体中文音色需要试听。当前维护仓库标示 GPL-3.0。 |
| eSpeak NG | 紧凑、可输出 WAV，适合调试及资源有限设备；自然度较低。 |

sherpa-onnx 主体采用 Apache-2.0，提供 C/C++ 和 Python 接口及 Linux 平台支持；
模型、声码器和附带组件需分别检查许可。[官方仓库](https://github.com/k2-fsa/sherpa-onnx)

Matcha 中文 Baker 示例使用约 72 MB 声学模型和约 51 MB Vocos 声码器，文件大小不能
代替内存或延迟测量。**其官方文档明确标注训练数据仅限非商业用途**，当前模型及完整
词典统一保存在 `models/tts/`，用于非商业开发测试；来源记录和逐文件 SHA256 一并维护。
实际部署选择已获适当许可的本地兼容模型，或自行
录制符合音频格式的文案。[官方模型说明](https://k2-fsa.github.io/sherpa/onnx/tts/pretrained_models/matcha.html)

Piper 与 eSpeak 资料：[Piper](https://github.com/OHF-Voice/piper1-gpl)、
[eSpeak NG](https://github.com/espeak-ng/espeak-ng)。首版代码只实现 sherpa-onnx Matcha
生成路径，不把备选引擎描述为已经支持的配置项。

生成工具从本地目录读取声学模型、词典、tokens、文本规范化 FST 和声码器，不隐式下载。
模型只加载一次，再生成全部 16 条文案。音频采用 8–48 kHz、单声道 PCM16 WAV，
每条最长 30 秒。运行节点也允许同规格双声道录音。模型、词典、引擎版本、文案、音色、
语速共同决定缓存文件名；manifest 保存来源说明和 SHA256。音频先写入临时目录，校验
通过后最后原子替换 manifest，合成失败保留原有可用清单。旧音频不自动删除。

部署前用生成器 `--check` 检查内容 SHA256；节点启动时检查所有配置文案与清单一致、
WAV 结构和时长有效。运行节点不重复计算音频 SHA256。修改文案后需要重新生成，
配置和音频更新后重启节点。发布包依靠项目现有 SHA256SUMS 记录完整安装资源。

## 提醒规则

默认文案和优先级以 [tts.yaml](../config/tts.yaml) 为唯一配置源，优先级范围为 0–3。

| 状态或事件 | 文案 | 默认优先级 |
| --- | --- | --- |
| INIT | 系统尚未初始化 | 0 |
| INITIALIZING | 系统正在初始化 | 0 |
| READY | 初始化完成 | 0 |
| HOMING | 正在返回待机位置 | 0 |
| STANDBY | 已进入待机状态 | 0 |
| GRASPING | 正在抓握方向盘 | 0 |
| FOLLOWING | 进入跟随模式 | 0 |
| CONTROL 到 FOLLOWING | 退出操控，进入跟随模式 | 1 |
| CONTROL | 进入操控模式 | 1 |
| RELEASING | 正在释放方向盘 | 0 |
| SAFE | 进入保护状态 | 2 |
| ERROR | 系统故障，请检查 | 2 |
| EMERGENCY_STOP | 软件急停已触发 | 3 |
| 通信中断 | 控制状态通信中断 | 2 |
| 通信恢复 | 控制状态通信恢复 | 1 |
| 来源冲突 | 检测到多个控制状态来源，请检查 | 2 |

CONTROL 可能先于默认刚度恢复和输入接纳，因此不播报“操控已就绪”。SAFE 不代表物理
安全已经确认；软件急停不代表物理手刹已确认。FOLLOWING 也不被描述为已实现柔顺控制。
`current_error` 变化会触发当前状态的提醒；历史 `last_error` 不参与触发，不直接朗读原始
英文异常、路径和堆栈。当前相同错误码可对应不同错误，不能仅以错误码去重。

1. 只接受配置发布者、同一时钟域、有效且新鲜的已支持 Managed 状态。按启动实例检查
   序号和采样时间，拒绝重复、倒退、未来和过期消息。公共消息解码成功不代替业务字段验证。
2. 首次有效普通状态默认只建立基线，`announce_initial: true` 可播报配置中的当前状态文案。
   默认高优先级告警即使是首份快照也会提醒。重启和短暂重连重新建立基线，不虚构历史转换。
3. 首次有效状态前默认等待 15 秒，避免正常启动顺序触发失联告警。获得基线后，200 ms
   不新鲜即撤销当前播报资格，再持续不可用 2 秒播报一次通信中断。合法 `valid=false`
   立即撤销资格。通信恢复播报一次，并在已有基线的情况下提示当前状态。
4. 同一配置发布者的两个启动实例在时效窗口内同时活跃时，报告来源冲突并暂停业务播报。
   仅剩一个新鲜来源、且再次收到有效状态后恢复。实例跟踪最多 8 槽，提醒队列最多 4 条。
5. 状态、当前错误或可用性改变时，使旧提醒失效并取消正在播放的旧文案；新高优先级任务
   可抢占低优先级任务。队列任务默认 5 秒过期，已开始且仍相关的音频不受队列期限截断。
   不重放过时普通状态，不循环重复同一告警；新的有效变化可再次触发。

## 执行与故障隔离

节点复用 `aviator_transport`、`aviator_protocol`、`aviator_runtime` 和 YAML。
单个非实时循环拥有 SUB socket、规则和调度器，每轮有界排空最多 128 条消息，轮询
周期 10 ms。播放由 `posix_spawnp` 创建的 `aplay` 子进程执行，不经过 shell，不把
消息文本拼进命令。主循环用 `waitpid(WNOHANG)` 收割退出状态，不等待音频播放完成。

取消时终止并回收直接子进程；单条播放最多 31 秒。SIGINT/SIGTERM 在启动线程前阻塞，
主循环同步收取信号，退出时回收播放器。用户级单实例 flock 避免重复播报，测试可使用
独立锁路径。缺失/过时音频和配置错误导致节点启动失败；设备或播放器故障记录日志，
节点继续观察状态，后续新提醒会再次尝试播放，不累积重试队列。

`start_aviator.sh --tts` 在 Core 前启动语音节点，使用同一 system 配置的 Bus 订阅地址。
启动前验证可执行文件、aplay、配置和全部音频。运行中 TTS 退出只报告一次，不触发
控制节点退出；整体停止仍会清理 TTS。默认不启用 TTS，便于无音频设备的仿真/CI。
相机随动脚本运行 Direct Core，不接入整机语音提醒。

安装目录为 `bin/aviator_tts`、`share/aviator/config/tts.yaml`、
`share/aviator/assets/tts/` 和 `share/aviator/tools/tts/generate.py`。
模型统一存放在项目的 `models/tts/`，重新生成时需要；默认不随运行节点安装。
应先生成音频，再生成和校验 release 包，避免修改已封装资源。

## 验证与后续扩展

自动测试覆盖规则去重、状态码校验、首次告警、当前错误变化、时效、重启、冲突、队列
过期；进程测试使用真实 loopback ZMQ，验证协议、抢占、信号退出、实例锁、音频包更新
与损坏检测。ALSA null 设备和可控假播放器用于测试，不驱动机器人或实际扬声器。

现场验收需断网运行、逐条试听文案、检查音频设备权限，并在控制负载下测量从接收有效
状态到出声的延迟。预生成语音 P95 小于 200 ms 是目标，不是已验证的设备性能保证。

首版基于 50 Hz 快照，可能遗漏短暂状态；ZMQ PUB/SUB 不保证送达，语音也不作为安全
确认。若需要完整转换观察，后续在状态机转换处产生 `system.event/STATE_CHANGED`，
由通信线程发布，事件和快照共享转换序号以去重。完整补发还需要事件缓存及查询机制。
任务 `generation` 不能当作转换序号。动态错误文本合成、常驻模型、错误分类码和
显式 `accepts_control` 播报属于后续范围。
