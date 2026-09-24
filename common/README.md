# 公共代码

头文件与源文件就近存放，由本目录 CMakeLists.txt 维护按需链接的小型库：

| 文件（待实现） | 职责 |
| --- | --- |
| protocol.hpp / protocol.cpp | 强类型消息、Topic、JSON 编解码与校验 |
| transport.hpp / transport.cpp | ZMQ context、完整消息收发与 TCP 端点 |
| runtime.hpp / runtime.cpp | 时钟、有界队列、快照、watchdog 与非实时诊断 |
| recording.hpp / recording.cpp | 非实时采集适配、MCAP 读写与会话管理 |

只提取实际共用的代码。控制算法与设备适配留在节点内；伺服线程不调用 JSON、ZMQ、MCAP 或文本日志接口。当前不提供空壳 API 或库 target。
