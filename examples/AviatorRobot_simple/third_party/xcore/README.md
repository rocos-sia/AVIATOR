# xCore SDK 0.7.1

本目录是源码集成的唯一预编译例外：厂商提供了 API 头文件和库，没有提供 SDK 实现源码。

- `include/rokae/`：来自工作区 `xCoreSDK-CPP-main/include/rokae`。
- `lib/linux-x86_64/libxCoreSDK.a`、`libxMateModel.a`：原项目使用的 0.7.1 Linux x86_64 SDK 静态库。
- `README.vendor.md`：原厂说明。头文件保留厂商版权声明。

文件 SHA256 记录在上层 `manifest.json`。更新 SDK 时必须同时更换配套头文件和库。
项目通过 `libaviator_rokae_sdk.so` 隔离 SDK 内嵌的 KDL 符号。
这些文件不受 AviatorRobot_simple 的 MIT 授权覆盖，使用和分发遵循厂商条款。
