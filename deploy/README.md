# 部署

后续放置与 nodes 中可执行程序同名的 systemd 服务、aviator.target 及必要的设备权限规则。程序真正实现就绪通知和健康检查后才配置 Type=notify 与 WatchdogSec。当前没有可部署的节点，不提供可启用的占位服务。
