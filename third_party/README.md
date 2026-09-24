# 第三方依赖（Ubuntu 22.04）

通用依赖使用 Ubuntu jammy 的 apt 开发包；保留无法通过该发行版官方仓库安装的源码和厂商 SDK。
安装入口为 [AviatorRobot_simple 安装脚本](../examples/AviatorRobot_simple/scripts/install_dependencies.sh)，支持 `--dry-run`。
脚本安装默认仿真、窗口和真机后端构建所需的开发包，不修改软件源、不添加 ROS/PPA 源。

| 已移除的源码目录 | apt 包 |
| --- | --- |
| eigen | libeigen3-dev |
| boost | libboost-filesystem-dev、libboost-system-dev、libboost-thread-dev、libboost-date-time-dev、libboost-serialization-dev |
| assimp、octomap | libassimp-dev、liboctomap-dev |
| console_bridge | libconsole-bridge-dev |
| urdfdom、urdfdom_headers | liburdfdom-dev、liburdfdom-headers-dev |
| yaml-cpp | libyaml-cpp-dev |
| nlopt | libnlopt-dev、libnlopt-cxx-dev |
| tinyxml、tinyxml2 | libtinyxml-dev、libtinyxml2-dev |
| zlib、glfw | zlib1g-dev、libglfw3-dev |
| ccd、qhull | libccd-dev、libqhull-dev |
| tinyxml2、tinyobjloader | libtinyxml2-dev、libtinyobjloader-dev |

包名已依据 Ubuntu 22.04 apt 元数据核对；例如 [TinyXML2 官方包信息](https://packages.ubuntu.com/jammy/libtinyxml2-dev)。
使用发行版维护的包版本及安全更新，不再要求与已删除源码的版本完全相同。
这也意味着安装产物依赖系统共享库：部署目标机须安装相应软件包，不再宣称整个安装目录自带全部依赖。

保留目录：

- `pinocchio-3.9.0/`（3.9.0）、`coal-3.0.4/`（3.0.4）、`pin_ik-2.2.0/`（2.2.0，包含本地适配）。
- `mujoco-3.4.0/`（3.4.0），以及 同级的 `lodepng-17d08dd26cac/`、`marchingcubecpp-f03a1b3ec29b/`、`trianglemeshdistance-2cb643de1436/`；Jammy 官方仓库没有对应可替换开发包。
- `xcore-0.7.1/`：0.7.1 厂商头文件和 Linux x86_64 静态库。
- `licenses/`：保留既有授权记录，包括闭源 SDK 内嵌 KDL 的说明。

CMake 先查找系统依赖，再构建剩余源码。产物仅写入构建目录；Pinocchio、Coal、PIN-IK 和应用共享系统 Eigen、Boost，避免混用 ABI。MuJoCo 使用系统共享 Qhull、CCD、TinyXML2 和 TinyObjLoader，并检查 CCD 的 double 精度配置。默认不构建上游示例、测试、Python 或 Studio。

来源与源码版本见 `manifest.json` 的 `packages`，apt 替换映射见 `system_packages`；本地改动见 `PATCHES.md`。历史许可证记录不表示对应源码仍在构建。更换依赖后应使用全新构建目录并重新验证，不能复用旧版 `dependencies/` 中的自编译通用库。

## 仓库统一管理

依赖源码统一存放在仓库根目录 `third_party/`。`examples/AviatorRobot_simple` 通过 CMake 引用此目录，构建产物仍写入各自构建目录；根工程当前不构建这些依赖。单独构建示例也需保留仓库目录布局。

`pin_ik-2.2.0/` 统一采用 AviatorRobot_simple 已适配的 2.2.0 副本，包含限位、数值梯度和命名空间修复；`pinocchio-3.9.0/` 已引入 3.9.0 源码，替代原预留目录。版本及校验信息见 `manifest.json`。

其他依赖：`cppzmq-4.11.0/` 保持原有带版本号的名称；上游提交及校验值仍待核验，尚未纳入 `manifest.json` 的来源锁定。保留上游许可证，不在业务节点中复制依赖。

所有依赖库目录采用 `名称-版本` 命名。未记录发布版本的 MuJoCo 间接依赖使用已锁定提交的前 12 位作为版本后缀，完整提交及归档校验值见 `manifest.json`。`licenses/` 是许可证汇总目录，不是依赖库。原 `mujoco_deps/` 层级已移除。更名后请重新创建构建目录，避免旧 CMake 缓存引用已移除的源码路径。
