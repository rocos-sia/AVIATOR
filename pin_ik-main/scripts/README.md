# 维护脚本

`migrations/` 保存一次性的历史目录及命名迁移脚本，不参与构建或安装。

`migrations/rename_to_pin_ik.sh` 用于旧 TRAC-IK 源码布局向 PIN-IK 迁移，会移动 Git 跟踪文件并批量替换文本。当前仓库已完成迁移，无需执行；脚本会拒绝在当前布局下运行。仅在处理旧布局时审阅并按需使用，CMake 配置仍需手动合并。
