# 协议定义

用于业务 Topic 的消息 Schema、公共头部及记录信封定义。`record_image.proto` 是当前相机直接写入 PNG 图像 MCAP 使用的 Protobuf Schema；通用 RecordEnvelope 和完整业务 Schema 尚未实现。相机使用的 Python 类型位于 `examples/aruco_camera/record_image_pb2.py`。
`recording/camera_packet.proto` 是已实现的相机专用二进制入口和 MCAP Schema，完整元数据契约见 [实现说明](../docs/Logger配置与图像记录实现说明.md)。C++ 在运行时构造对应 FileDescriptorSet，不需要 protoc；字段变更须同步描述符代码和跨语言测试。

该 Schema 不等同于 ICD 草案通用 RecordEnvelope。业务 Topic 的完整 JSON Schema 尚未冻结，当前 Logger 仍使用公共头部 Schema。
