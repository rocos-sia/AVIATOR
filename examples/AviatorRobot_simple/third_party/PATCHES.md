# 上游源码适配

`pin_ik/` 的角度回绕、数值梯度、内部命名空间及构建适配见
[pin_ik/README.vendor.md](pin_ik/README.vendor.md)。

当前保留的适配：

- `mujoco/cmake/MujocoDependencies.cmake`：Qhull、CCD、TinyXML2、TinyObjLoader 改用 Ubuntu 22.04 开发包导入目标；Qhull 添加 `libqhull_r` 头文件路径，CCD 检查 double 精度。TinyXML2 使用 Jammy 提供的 pkg-config 元数据。
- `mujoco_deps/trianglemeshdistance/TriangleMeshDistance/include/tmd/TriangleMeshDistance.h`：预先应用上游 range-loop 引用修复；配置时不再改写源码。
- 保留部分 `.gitignore` 的源码文件例外，避免 CMake 模块等被宽泛忽略规则漏掉。
- `pinocchio/`、`coal/` 包含既有 jrl-cmakemodules，避免构建时下载子模块。

原 TinyXML、OctoMap、NLopt、zlib 及 Boost 构建适配已随对应源码删除；这些库现在使用 Ubuntu 包。旧版本验证记录保留在 `../validation/`，不代表新的系统包构建已经通过全部验收。

SDK 内嵌 KDL 的许可证保留；应用不构建独立 KDL、kdl_parser 或 TRAC-IK。
