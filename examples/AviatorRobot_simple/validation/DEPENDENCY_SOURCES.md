# 第三方源码集成验证

> 历史记录：当前已切换为 Ubuntu 22.04 系统包与保留源码混合构建；以下全源码依赖与断网验证结论不直接适用于当前版本。

日期：2026-09-21。环境：Ubuntu 22.04 x86_64、GCC 11、CMake 3.22.1、Ninja 1.10.1，Release。

## 变更范围

- 开源依赖源码放入 `third_party/`，包括固定提交的 MuJoCo 间接依赖和 Pinocchio/coal 的 CMake 模块。
- 删除 `linux-x86_64.tar.gz`、对应校验文件和旧打包脚本。
- CMake 从本地源码编译到构建目录，再安装到 `build/dependencies/`；无下载步骤。
- xCore SDK 0.7.1 保留厂商头文件和 Linux x86_64 静态库，路径为 `third_party/xcore/`。
- 版本、来源、授权与本地适配分别记录在 `third_party/manifest.json`、`licenses/`、`PATCHES.md`。
- KDL/TRAC-IK、Pinocchio 的职责及控制代码不变；保留用户设置的 `wheel_angular_speed: 0.8`。

## 断网构建

原构建目录保留为 `build-before-source-20260921/`，新建空的 `build/`。
以下命令在独立的 Linux 网络命名空间内运行，构建进程无法访问外网：

```bash
unshare --user --map-root-user --net cmake -S AviatorRobot_simple -B AviatorRobot_simple/build \
    -G Ninja -DCMAKE_BUILD_TYPE=Release
unshare --user --map-root-user --net cmake --build AviatorRobot_simple/build -j4
unshare --user --map-root-user --net env -u LD_LIBRARY_PATH LD_BIND_NOW=1 \
    ctest --test-dir AviatorRobot_simple/build --output-on-failure
```

全新配置、全部依赖编译、SDK 链接和控制程序编译通过。每个第三方项目使用 2 个编译任务。
后续相对 RPATH 适配和增量构建也在断网环境中通过。
`unshare` 只用于本次验证，普通用户不需要使用它，执行 README 中的 CMake 命令即可。

检查各依赖的 CMakeCache 与最终 ELF 动态库解析：机器人库全部来自本构建目录，
不引用 `AviatorRobot`、`/opt/rocos`、旧构建目录或系统安装的同名库。
X11/OpenGL 及 C/C++ 标准运行库仍由系统提供。
源码内容快照对比未发现构建过程改写或生成源码文件，手工适配均记录在 PATCHES.md。

## 运行结果

| 验证 | 结果 |
|---|---|
| 默认 CTest | **11/11 通过，76.57 秒** |
| 启动姿态、使能、接近、锁定、现有完整目标表、解锁与失能 | 通过 |
| 双臂 J2 全程范围 | 85.4956° ～ 94.5001°，符合 [85°, 95°] |
| MoveWheel 速度倍率、下发速度限制、关节约束延时 | 通过 |
| ServoWheel 连续目标、停止、超时、错误与恢复 | 通过 |
| 实时节拍、后端配置拒绝、开环逻辑、使能失败回滚、交互停止 | 通过 |
| 实际 GLFW 窗口的相机旋转、平移、缩放、复位与退出 | 通过 |
| 安装版 `aviator --servo-demo` 带窗口运行 | 通过，Demo completed |
| 安装目录改名后，从项目外清除 LD_LIBRARY_PATH 运行完整无头演示 | 通过，Demo completed |
| 安装版动态库查找及 RPATH | 无缺失库，无本机构建目录绝对路径 |

相机与安装验证先在独立的 `build-source/` 构建中完成；随后常用 `build/` 完成断网全新构建和完整 CTest。
真机未连接、未尝试连接；xCore 后端仅验证编译链接与连接前参数检查。

日志：[全新构建](dependencies-build.txt)、[增量构建](dependencies-incremental.txt)、
[CTest](dependencies-ctest.txt)、[测试详细输出](dependencies-ctest-details.txt)、
[独立安装演示](dependencies-installed-demo.txt)、[窗口 Servo](dependencies-servo-gui.txt)、
[安装版库解析](dependencies-installed-ldd.txt)。
