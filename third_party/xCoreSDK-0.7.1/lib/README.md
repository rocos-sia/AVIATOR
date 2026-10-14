# AR SDK 预编译库

当前目录使用用户提供的 `librokae-v0.7.1.ar_10`（2026-08-05）完整分发包。
来源：`/home/rocos/Downloads/xCoreSDK-0.7.1.ar_10/librokae-v0.7.1.ar_10`。
目录名和共享库文件名仍保留 `0.7.1`，实际版本以 `CHANGELOG.md` 和 `sdkVersion()` 为准。
头文件、SDK 库及模型库必须使用同一 AR 包，不要混入普通版 0.7.1 库。

本地已替换分发包内所有文件，包括 Linux/Windows 库。仓库忽略规则可能不跟踪二进制文件；其他工作站需要复制相同 AR 分发包。
保留原 LICENSE。`lib/CMakeLists.txt` 的 `Rokae::Rokae` 别名使用 `PROJECT_NAME`，维持 AVIATOR 通过 add_subdirectory 集成时的兼容性；这是相对于分发包的本地修正。
