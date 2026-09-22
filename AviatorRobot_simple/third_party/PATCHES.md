# 上游源码适配

除以下适配外，不修改第三方算法实现。构建选项集中在 `cmake/Dependencies.cmake`。

- `tinyxml/CMakeLists.txt`：上游 2.6.2 只有 Makefile，补充最小共享库构建与安装规则。
- `tinyxml/tinyxml.h`：默认启用 `TIXML_USE_STL`，使库和调用方使用一致的 STL ABI，
  满足 urdfdom 的 `std::string` 接口需求。
- `octomap/octomap/CMakeLists.txt`：输出根目录从源码目录改为构建目录，避免将 lib/bin 写入源码树。
- `nlopt/CMakeLists.txt`、`octomap/octomap/CMakeModules/CompilerSettings.cmake`：
  保留调用方指定的相对 RPATH，避免将本机构建目录写入安装库的运行时搜索路径。
- `zlib/zconf.h.included`：预先完成上游 CMake 原本会执行的 `zconf.h` 重命名。
  实际使用构建目录按平台生成的 `zconf.h`，后续配置不再改写源码目录。
- `mujoco_deps/trianglemeshdistance/TriangleMeshDistance/include/tmd/TriangleMeshDistance.h`：
  预先应用 MuJoCo 自带的 range-loop 引用修复；相应删除
  `mujoco/cmake/MujocoDependencies.cmake` 中配置时改写该源码的步骤。
- 移除上游包中与本项目构建无关的两个预编译文件：
  `boost/libs/serialization/test/config_test.o`、`mujoco_deps/tinyobjloader/tools/windows/premake5.exe`。
- 为部分 `.gitignore` 添加源码文件例外，确保上游 Makefile、CMake 模块、样例数据等
  不被工作区或上游的宽泛忽略规则漏掉。

`trac_ik/`、`kdl_parser/` 保留项目已有的无 ROS 适配，本次不变。
`pinocchio/`、`coal/` 包含原 AviatorRobot 中已有的 jrl-cmakemodules 源码，避免构建时下载子模块。
