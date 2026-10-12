# 离线提醒音频

`zh_CN/manifest.json` 和 WAV 由 [生成工具](../../tools/tts/generate.py) 从本地模型生成，
生成模型统一存放在 [models/tts](../../models/tts/README.md)，不再依赖 `/tmp`。
重新生成时需要模型；播放已有 WAV 不需要模型。CMake install 会安装音频包，不安装生成模型。
部署前运行 `aviator_tts --check`；没有音频包时可用 `--dry-run` 验证消息规则。

生成环境准备完成后，在项目根目录执行 `./scripts/generate_tts.sh` 即可预生成全部文案；
`./scripts/generate_tts.sh --check` 只校验已有音频包。首次环境准备见节点说明。

文案来源是 [tts.yaml](../../config/tts.yaml)。修改文案后必须重新生成；manifest 保存文案、
音频 SHA256、模型与词典 SHA256、模型来源、引擎版本、音色和语速。
发布音频包前应试听全部文案并核对模型及数据许可，使用方法见
[节点说明](../../nodes/aviator_tts/README.md)。

当前 Matcha Baker 模型及音频用于非商业开发测试，来源与模型校验值见
[sources.json](../../models/tts/sources.json) 和 [SHA256SUMS](../../models/tts/SHA256SUMS)。
