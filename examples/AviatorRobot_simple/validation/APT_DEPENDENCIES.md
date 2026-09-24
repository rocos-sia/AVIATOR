# Ubuntu 22.04 系统依赖切换验证

日期：2026-09-24。环境：Ubuntu 22.04.5 x86_64、GCC 11.4.0、CMake 3.22.1，Release。

本次删除 13 个顶层依赖源码目录和 4 个 MuJoCo 内部依赖目录，改用 apt 开发包；第三方目录从约 1.2 GiB 缩减到 198 MiB。保留源码与 apt 包映射见 `../third_party/manifest.json`。

已完成的检查：

- 安装脚本 `bash -n`、`--dry-run` 通过；apt 模拟安装确认依赖包列表可解析。
- 默认仿真、GLFW 窗口与 xCore 后端开启的全新目录配置、完整构建和安装通过。
- 关闭仿真时的独立 CMake 配置通过，不要求 TinyObjLoader 或 GLFW。
- MuJoCo 实际加载 `model/aviator.xml` 与网格资源通过。
- CTest 14/14 全部通过，耗时 79.84 秒；包括位姿、PIN-IK、基础仿真、验收、后端选择、实时接口、方向盘运动、伺服、运动学、无头演示、开环、控制安全和交互测试。
- 安装后的 `aviator` 经 `ldd` 检查，无缺失共享库；通用库解析到系统目录，保留的机器人库解析到安装目录。

本机原有绝大多数 apt 开发包；缺少的 `libtinyobjloader-dev` 与 `libtinyobjloader1` 使用 `apt-get download` 下载并解压到 `/tmp/aviator-apt-check/root`，验证时通过 CMAKE_PREFIX_PATH 传入。没有在系统上执行实际安装，也未连接真实机器人；真机后端仅完成编译和链接检查。安装脚本的实际 apt 安装流程需由使用者执行。

复现步骤（从仓库根目录执行，需使用新的构建目录）：

```bash
./examples/AviatorRobot_simple/scripts/install_dependencies.sh
cmake -S examples/AviatorRobot_simple -B build/aviator-simple-apt -DCMAKE_BUILD_TYPE=Release
cmake --build build/aviator-simple-apt --parallel 2
ctest --test-dir build/aviator-simple-apt --output-on-failure
cmake --install build/aviator-simple-apt --prefix /tmp/aviator-simple-install
```

这是本次依赖替换的构建与仿真回归结果，不替代目标硬件的实时性或真机验收。
