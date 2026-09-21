# 离线第三方依赖

`linux-x86_64.tar.gz` 是本项目直接使用的开发头文件与运行库，约 32 MiB。
正常配置时 CMake 校验 `linux-x86_64.sha256` 后解压到构建目录；不下载、不搜索兄弟工程，
也不调用系统的 Boost/KDL/Pinocchio/ROS CMake 包。

平台：**Ubuntu 22.04 x86_64 / GCC 11 / glibc 2.35**。这是预编译依赖包，
不承诺 Windows、ARM 或旧版 glibc 的二进制兼容性。仍需要系统编译器、CMake、pthread、
C/C++ 标准运行库；可视化还需要系统显示服务和 OpenGL 驱动。

| 依赖 | 集成方式 / 来源 |
|---|---|
| TRAC-IK、kdl_parser | `trac_ik/`、`kdl_parser/` 源码，来自原 AviatorRobot 的 rocos_app/3rdparty；不编译 ROS 支持 |
| Pinocchio 3.9.0、coal 3.0.4 | 原 AviatorRobot 的已构建头文件和动态库 |
| MuJoCo 3.4.0 | 原 reference/rocos-mujoco 所用发行版 |
| xCoreSDK 0.7.1 | xCoreSDK-CPP-main 的头文件及 0.7.1 Linux 包的静态 SDK/模型库 |
| NLopt 2.7.0 | 本机原 rocos 使用的配套头文件和动态库 |
| Boost 1.74、Eigen 3.4、KDL 1.5.1、yaml-cpp、urdfdom、TinyXML/TinyXML2、GLFW | Ubuntu 22.04 开发头文件及对应动态库 |
| Assimp、OctoMap、Draco、zlib、minizip、X11 等 | 上述动态库的运行时依赖闭包 |

`manifest.json` 记录包版本与每个文件的 SHA256；授权/版权说明在压缩包的 `licenses/` 内，
安装时复制到 `share/aviator/licenses/`。xCoreSDK 保留厂商版权声明，不属于本项目的 MIT 授权范围。
TRAC-IK 和 kdl_parser 的授权说明保留在各源码文件头。

维护者可用 `python3 tools/bundle_dependencies.py` 从原工作区重建包；脚本说明了输入来源。
**普通用户构建不需要这些原路径**。要跨平台移植，需为目标平台重新构建相应依赖及 SDK。
SDK 独立 DSO 使用静态库及局部符号绑定，避免其内嵌 KDL 覆盖控制库的 KDL；
不使用 `--allow-shlib-undefined` 或运行时延迟解析来绕过链接错误。
