# 协议定义

用于业务 Topic 的消息 Schema、公共头部及记录信封定义。`record_image.proto` 是当前相机直接写入 PNG 图像 MCAP 使用的 Protobuf Schema；通用 RecordEnvelope 和完整业务 Schema 尚未实现。相机使用的 Python 类型位于 `examples/aruco_camera/record_image_pb2.py`。
