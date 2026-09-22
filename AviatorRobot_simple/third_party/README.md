# 第三方源码

开源依赖直接以源码目录保存，不使用压缩依赖包或构建时下载。
CMake 按依赖顺序编译，产物位于 `build/third_party/`，头文件和库安装到
`build/dependencies/`。不依赖原 `AviatorRobot`、ROS、`/opt/rocos` 或系统安装的同名机器人库。

唯一的预编译例外是厂商未提供实现源码的 **xCore SDK**：

```text
third_party/
  eigen/ boost/ pin_ik/ nlopt/ ...  # 开源源码
  pinocchio/ coal/                                 # 包含本地 cmake 模块
  mujoco/ mujoco_deps/ glfw/                       # 仿真及其源码依赖
  xcore/
    include/rokae/                                # 真机头文件
    lib/linux-x86_64/libxCoreSDK.a
    lib/linux-x86_64/libxMateModel.a
  licenses/                                       # 安装时携带的授权说明
  manifest.json                                   # 固定版本、来源和下载原件校验值
  PATCHES.md                                      # 对上游源码的必要适配
```

| 目录 | 版本 / 来源 |
|---|---|
| eigen | 3.4.0 |
| boost | 1.74.0；编译 filesystem、system、thread、date_time、serialization 及其依赖 |
| pin_ik | 2.2.0 源码快照，来自工作区 pin_ik-main；正式 FK/IK 使用 Pinocchio/PIN-IK |
| pinocchio、coal | 3.9.0、3.0.4；保留原项目版本及内含的 jrl-cmakemodules |
| yaml-cpp、nlopt | 0.7.0、2.7.0 |
| console_bridge、urdfdom_headers | 1.0.1、1.0.5 |
| urdfdom | 上游标签 3.0.1（该标签内部 CMake 版本仍写作 3.0.0） |
| tinyxml、tinyxml2 | 2.6.2（STL 接口）、9.0.0 |
| assimp、octomap、zlib | 5.2.2、1.9.7、1.3.1 |
| mujoco、glfw | 3.4.0、3.3.6 |
| mujoco_deps | 按 MuJoCo 3.4.0 固定提交保存 lodepng、marchingcubecpp、qhull、tinyxml2、tinyobjloader、trianglemeshdistance、ccd |
| xcore | SDK 0.7.1，Linux x86_64；厂商头文件与静态库 |

MuJoCo 内部使用的 TinyXML2 和控制代码使用的 TinyXML2 分开放置，避免改变上游版本配套。
Assimp 只启用当前模型需要的 STL、OBJ 导入器，使用其源码内附的 unzip 和本地 zlib；
关闭未使用的导出器、Draco 等功能。新增模型格式时在 `cmake/Dependencies.cmake` 中启用对应导入器。
MuJoCo 的示例、Python、测试、Studio 等未使用组件不参与构建，其额外依赖也不需要下载。

## 构建与平台

从项目根目录执行 README 中的普通 CMake 命令即可，无需先逐个安装这些库。
首次构建需要较长时间；后续构建会检查第三方源码变更并增量编译。
可用 `-DAVIATOR_DEPENDENCY_JOBS=2` 调整每个依赖的编译并发数。

已验证 Ubuntu 22.04 x86_64、GCC 11、CMake 3.22。系统仍需提供编译器、标准运行库、
pthread；窗口构建还需要 X11/OpenGL 开发包及显示环境。源码集成不代表这些系统组件也被打包。
当前链接规则面向 Linux；其他平台需单独移植和验证。xCore 静态库仅支持 Linux x86_64，
其他平台必须关闭 `AVIATOR_WITH_ROKAE` 或换用对应厂商 SDK。

SDK 独立 DSO 使用静态库及局部符号绑定，其内嵌 KDL 保持私有。
PIN-IK 求逆解，Pinocchio 做正运动学、位姿表示和碰撞检测，Eigen 处理旋转与插值。
项目不再构建或链接独立 KDL、kdl_parser、TRAC-IK；对应源码已移除。
`licenses/orocos-kdl/` 的既有授权说明保留，因闭源 xCore SDK 内部仍包含 KDL；
PIN-IK 源码自身的上游版权与来源声明也保持原样。
PIN-IK 复用现有依赖，不另行引入 Pinocchio、Eigen 或 NLopt 的其他版本。

## 来源与授权

每个新增源码目录的 `.aviator-source.json` 和总表 `manifest.json` 记录版本、来源及
下载原件 SHA256（如适用）。这些 URL 只用于追溯，构建过程不访问它们。
上游版权和授权文件保留在源码树，`licenses/` 随安装复制到 `share/aviator/licenses/`。
xCore 属于厂商 SDK，不受本项目 MIT 授权覆盖；其厂商说明见 `xcore/README.vendor.md`。

更新库时替换对应源码、同步来源记录及必要适配，并重新执行完整仿真测试。
无需重建任何第三方压缩包。
