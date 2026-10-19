# AVIATOR 版本管理、发布校验与台架测试方案

日期：2026-10-19。依据：[软件版本号规范](AVIATOR软件版本号规范.md)、[软件校验码规范](AVIATOR软件校验码规范.md)。

## 1. 实现范围

本次接入 CMake 安装、版本记录、SHA-256 清单生成与验证、提交版本维护 skill，以及无安装元数据时显式启用的台架测试身份。保持业务控制逻辑不变。

Gateway 已接入双路 RS422 台架模式，启动时调用 `identity` 验证当前运行目录并核对完整编译版本字符串及线上编码，成功后缓存身份并周期上报。其余组件混装检测、全系统启动门控和正式发布审批仍需独立完善。详见 Gateway README 和 RS422 规范第 10 节。

## 2. 版本唯一来源与提交维护

唯一来源为根 `CMakeLists.txt` 中的 `project(AVIATOR VERSION ...)`。显示时可以加 `v`，机器字段不带前缀。配置阶段拒绝前导零、后缀、不完整版本及MAJOR/MINOR 超过 255 的版本；PATCH 为非负整数并保留于完整发布版本，不占串口位。

```text
version_u16 = (MAJOR << 8) | MINOR
```

线上版本不能区分同一 MAJOR.MINOR 的不同 PATCH；Gateway 启动校验必须同时比较完整版本字符串与 `version_u16`。

构建生成 `build/<配置>/generated/aviator/version.hpp` 和 `release-build.json`。版本头文件通过 `aviator_logging` 的接口依赖提供给节点，也安装到 `include/aviator/version.hpp`。发布记录包含版本、编码、发布配置、构建类型、实际 Git 提交、dirty 状态、工具链、已发现的依赖版本、构建开关及仓库内第三方源码目录标识；Git 提交用于定位这些源码。目录标识列举仓库源码，不表示全部依赖均被选择编译。系统动态库和 Python 环境仍属于外部部署依赖。

项目 skill 位于 `.agents/skills/aviator-version/SKILL.md`，根 `AGENTS.md` 要求助手在创建提交前读取：不兼容变更升 MAJOR，兼容功能升 MINOR，兼容修复和交付修订升 PATCH；纯文档、测试和工具维护也升 PATCH。按待提交内容判断最高级别，不按行数判断，重试或 amend 不重复递增。溢出时停止，不回绕。

普通编辑或构建不升级版本。本功能首次提交按兼容性功能增加，将版本从 `0.1.0` 升至 `0.2.0`。skill 不会拦截人工 Git 操作，不新增 Git hook，也不自动打标签或推送。

## 3. 安装与构建流程

默认安装前缀为 `${CMAKE_BINARY_DIR}/dist`，例如 release preset 使用 `build/release/dist/`，不同构建目录各自保存安装产物；现有 `/build/` Git 忽略规则覆盖这些目录。显式 `-DCMAKE_INSTALL_PREFIX=...` 优先。已有 CMakeCache 的旧路径不会被静默覆盖，需重新配置：

```bash
cmake --preset release -DCMAKE_INSTALL_PREFIX="$PWD/build/release/dist"
cmake --build --preset release
```

默认完整构建的 `aviator_release` 目标依赖项目构建目标，在编译成功后先安装到构建目录内的临时目录，再生成和验证发布记录，最后同步到安装前缀。上次清单列出的过期文件会被移除；日志、缓存等排除项保留。发现清单之外的未知文件时拒绝覆盖，要求选择干净前缀。不会递归删除任意安装目录。共享前缀通过目录锁拒绝同时发布。

更新安装目录时先移除旧摘要文件，避免复制中断后保留旧身份；仍应停止运行节点再替换软件。这不是热更新机制。损坏或中断的安装请使用新的空前缀重新构建发布。

发布更新允许替换使用旧版 `4+4+8` 版本编码的安装包（如 `6.0.0` 的 `version_u16=24576`），并生成当前 `8+8` 编码的记录；旧记录仍需满足原编码范围、发布配置和校验范围要求，未知文件保护保持有效。此兼容仅用于识别待替换的旧安装包，`verify`、`identity` 和新包生成仍严格使用当前编码。

仅编译某个 target 不代表整套发布完成；需要时随后执行：

```bash
cmake --build build/release --target aviator_release
```

完整 `cmake --install` 也会在安装及 RPATH/strip 处理后生成校验文件，支持 `--prefix` 和 `DESTDIR`；推荐向空目录安装，它本身不清理旧组件。`--component` 部分安装不生成完整系统身份。不要并发执行显式安装和发布目标。

安装布局：

```text
build/release/dist/
├── bin/                         # 已选择构建的程序
├── lib/                         # 随包库
├── include/                     # 现有开发头文件、生成的版本头
├── share/aviator/
│   ├── config/                  # 交付配置
│   ├── models/                  # 所选组件的模型资源
│   ├── nodes/camera/            # 相机运行 Python 模块
│   ├── rh56ftp_hand/            # 手节点及 pendant 依赖
│   ├── common/                  # Python 日志桥接
│   ├── tools/steering_wheel_calibration/
│   └── scripts/                 # 校验工具、适用构建的启动脚本
├── release-version.json
├── SHA256SUMS
└── release-checksum.json
```

沿用已有模型、库和文档安装规则，补齐相机模块、其标定模块及启动脚本；安装后的启动脚本从安装包中取程序和配置。无 Core 的 communication 构建不安装完整设备启动脚本。现有开发头文件、静态库和文档也纳入校验，故其交付内容变化会改变摘要。

默认发布配置按实际构建开关区分 `communication`、`simulation`、`hardware`、`hardware-simulation`，可以显式设置 `AVIATOR_RELEASE_PROFILE`。Preset 名称本身不能证明已启用或禁用某组件；实际开关写入记录。Debug 和 dirty 构建可供开发核对，元数据不代表已经批准正式交付。

## 4. 文件范围、校验文件与验证

采用 Python 3.9+ 标准库 `hashlib.sha256`，当前发布驱动使用 Linux 的目录锁。范围为安装根目录下所有普通文件及解析到包内普通文件的文件符号链接。禁止目录符号链接、越界链接、非法相对路径、仅大小写不同的路径；文件链接按目标实际内容哈希。

排除根目录的 `SHA256SUMS`、`release-checksum.json`，以及目录 `logs/`、`cache/`、`tmp/`、`run/`、`test-output/`、`share/aviator/logs/` 和任意层级 `__pycache__/`。不按 `.yaml`、`.json` 等扩展名排除。规则写入参与校验的 `release-version.json`，验证工具拒绝不认识的范围定义。

`SHA256SUMS` 为 UTF-8、无 BOM、LF、末行带 LF，每行是小写 64 位十六进制摘要、两个 ASCII 空格、相对路径。路径按 UTF-8 字节序排序；不包含时间戳、绝对路径或注释。

```text
release_digest = SHA256(SHA256SUMS 原始字节)
checksum_u16 = int(release_digest[0:4], 16)
```

`release-checksum.json` 保存 `algorithm: SHA256SUMS-v1`、完整 `release_digest`、十进制 `checksum_u16`、对应版本和发布配置。该文件不参与自身校验；验证时重新计算清单摘要并核对所有字段。校验值不回写已参与哈希的程序，避免自引用。

```bash
python3 scripts/aviator_release.py verify build/release/dist
# 验收时通过独立可信交付记录提供完整摘要：
python3 scripts/aviator_release.py verify build/release/dist --expected-digest <64位小写摘要>
# GNU 工具可逐项校验；它不检查多出的文件：
(cd build/release/dist && sha256sum --check --strict SHA256SUMS)
```

工具会同时比对实际文件集合，拒绝修改、删除、改名、新增、清单格式错误及元数据不一致。`verified: true` 表示文件与清单一致；没有可信摘要或签名时，不证明发布者身份。相同最终文件和元数据产生相同摘要，不保证编译器每次生成相同二进制。

## 5. 无安装元数据时的台架测试校验码

固定测试输入为 ASCII/UTF-8 字节串 `AVIATOR-RS422-BENCH-TEST-v1\n`，最后一个字节为 LF：

```text
SHA-256 = 9ca1285afa2aace1e038abf092699dce8838bc3f29b51c3d866fdae689f60b1b
测试短码 = 0x9CA1 = 40097
字节24～25 = A1 9C
```

该摘要只属于公开测试字符串，不是安装文件摘要。测试码不占用协议保留值，也可能与某个真实发布短码碰撞，飞控及台架必须通过测试配置/界面区分运行模式，不能只看短码识别测试版。

读取身份：

```bash
# 正常模式：必须存在完整、通过验证的安装记录
python3 scripts/aviator_release.py identity build/release/dist

# 台架模式：三个发布元数据文件均不存在时才使用固定测试码
python3 scripts/aviator_release.py identity build/release/dist \
  --allow-test-fallback --version 0.2.0
```

`--version` 必须与当前 CMake 项目版本一致，不能用测试码替代版本；后续 C++ 调用方可直接使用生成头文件的 `version_u16` 和 `bench_checksum_u16`。有效安装存在时，即使允许 fallback 也优先验证并使用真实发布身份。任意发布元数据存在但缺失其他文件、校验失败、数据格式错误或与可信摘要不符，必须报错，不能回退。指定可信摘要时不允许 fallback。

测试输出显式带 `mode: bench-test`、`verified: false`、`release_profile: bench-test`、`test_vector_sha256`，不提供伪造的 `release_digest`。版本 `0.2.0` 对应上行字节 22～25 为 `02 00 A1 9C`；`identity` 输出 `rs422_bytes_22_25` 供台架对照。测试身份只用于飞控联调，不作为交付验收通过或进入正式可操控状态的依据。

**软件短码与整帧累加校验必须同时存在。** 对 64 字节上行帧，先填入偏移 22～23 的版本和 24～25 的短码，将偏移 26～62 的备用位填零，再计算偏移 63：

```python
frame[26:63] = bytes(37)
frame[63] = sum(frame[2:63]) & 0xFF
# 排除帧头，直接累加；无帧尾；短码不能替代 frame[63]。
```

工具模块提供 `identity()` 和 `uplink_frame_checksum()`，供测试程序复用。正式 Gateway 后续应在启动时完成实际文件验证、缓存身份，在 80 Hz 周期中仅编码缓存值和计算帧累加校验，不扫描磁盘。

## 6. 验证与改动边界

`python3 tests/release_test.py`（也注册为 CTest 的 `release_metadata`）覆盖版本边界/非法值、规范 SHA-256 和小端向量、路径排序、GNU 格式、文件集合变化、运行数据排除、符号链接、可信摘要核对、元数据篡改、显式测试 fallback、完整身份优先以及安装遗留清理。构建验证采用不连接硬件的 communication 配置。

本次不实现串口驱动、控制门控、数字签名、压缩包封装、系统依赖自动打包或热更新。正式发布仍需冻结版本、依赖及配置，并独立保管完整摘要。
