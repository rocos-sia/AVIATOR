# AVIATOR 机器人状态机设计

本文描述 `aviator_core` Managed 模式的整机业务状态机。实现入口为
`include/aviator/RobotStateMachine.hpp`、`RobotStateMachine.cpp` 和 `Aviator.cpp`。
Direct 模式的小写动作接口不经过整机业务状态机；设备执行器阶段与整机状态分别报告。

状态机采用单个平面 boost::sml 转换表，由唯一拥有线程调用。网络和 SDK 回调不直接执行转换；
耗时动作交给唯一工作任务，拥有线程持续处理反馈、请求、心跳、故障和急停。

## 1. 状态与正常流程

| 状态 | 进入后的行为 |
| --- | --- |
| INIT | 初始状态；检查急停证据，再由内部 Boot 启动初始化。 |
| INITIALIZING | 初始化资源、使能双臂、配置阻抗，保持当前位置。 |
| READY | 已准备完成或故障复位完成，保持当前位置，等待明确操作；不表示已回 home，也不表示已脱离方向盘。 |
| HOMING | 执行回 home，等待实际关节到位和停稳。 |
| STANDBY | 双臂已回到配置的 home 姿态并停稳，等待抓握。 |
| GRASPING | 张开确认、接近、抓握。 |
| FOLLOWING | 保持当前参考，异步切换到跟随刚度；不接收主动飞控目标。 |
| CONTROL | 恢复默认刚度后，接纳授权且新鲜的飞控目标。 |
| RELEASING | 必要时先结束 Servo/阻抗切换，再恢复默认刚度、松手、撤离、回 home。 |
| ERROR | 停止并撤销普通运动输出，记录当前错误和历史错误，等待明确复位。 |
| EMERGENCY_STOP | 锁存软件急停、取消任务、请求独立手刹通道；本实例无普通退出路径。 |

可导出的状态图：[SVG](assets/aviator_state_machine/aviator_state_machine.svg)、[PNG](assets/aviator_state_machine/aviator_state_machine.png)，源文件为同目录 `.dot`。

共十一个业务状态。SAFE 已取消，不再作为错误恢复或输入失效的中间状态。
“回零”指双臂回到配置的 home 姿态，不是把方向盘角度/位移归零。

```mermaid
stateDiagram-v2
    [*] --> INIT
    INIT --> INITIALIZING: Boot
    INITIALIZING --> READY: 初始化完成
    READY --> HOMING: ENTER_STANDBY / 已脱离
    HOMING --> STANDBY: 到位停稳
    STANDBY --> GRASPING: GRASP_WHEEL
    GRASPING --> FOLLOWING: 抓握完成
    FOLLOWING --> CONTROL: START_CONTROL
    CONTROL --> FOLLOWING: EXIT_CONTROL
    FOLLOWING --> RELEASING: LEAVE_WHEEL / ENTER_STANDBY
    CONTROL --> RELEASING: LEAVE_WHEEL / ENTER_STANDBY
    READY --> RELEASING: LEAVE_WHEEL / 允许撤离
    RELEASING --> STANDBY: 撤离并回 home 完成
    ERROR --> READY: RESET_ERROR / 故障清除且可恢复
    note right of ERROR
        执行失败、阻断故障或超时直接进入
        输入恢复不自动退出 ERROR
    end note
    note right of EMERGENCY_STOP
        任一其他状态可进入
        普通请求不能解除锁存
    end note
```

## 2. 六操作与守卫

| 当前状态 | 操作 | 条件及结果 |
| --- | --- | --- |
| READY | ENTER_STANDBY | 资源就绪、停稳、执行器空闲且已脱离方向盘，登记 home 任务，进入 HOMING。 |
| STANDBY | ENTER_STANDBY | 幂等完成，不重复回 home。 |
| STANDBY | GRASP_WHEEL | 资源就绪、停稳、跟随授权及执行器空闲，进入 GRASPING。 |
| FOLLOWING | START_CONTROL | 资源就绪、停稳、任务空闲、跟随/输入来源授权及输入有效，直接进入 CONTROL。 |
| CONTROL | EXIT_CONTROL | 跟随获授权，关闭主动目标接纳，进入 FOLLOWING 并登记跟随刚度任务。 |
| FOLLOWING | EXIT_CONTROL | 空闲时幂等完成；已有刚度任务时返回 BUSY。 |
| FOLLOWING / CONTROL | LEAVE_WHEEL / ENTER_STANDBY | 资源就绪且释放获授权，直接进入 RELEASING；一次请求包含减速、撤离和回 home。 |
| READY | LEAVE_WHEEL | 仍在握盘、资源就绪、停稳、执行器空闲且释放获授权，进入 RELEASING。 |
| ERROR | RESET_ERROR | 原因已排除、资源可用、已停稳、执行器空闲且无急停锁存，进入 READY；清当前错误，保留历史错误。 |

FOLLOWING / CONTROL 的直接撤离不要求调用方先发 EXIT_CONTROL，也不要求请求时 Servo 已停稳。
正在进行的跟随刚度或 CONTROL 默认刚度恢复可以被撤离请求接管：取消旧任务代号，等待旧工作线程退出，
再执行新的释放任务。不会同时使用两个工作槽，也不会先重新进入 FOLLOWING。
RELEASING 中重复操作返回 BUSY。实际张开、撤离前必须确认 Servo 已退出，不能把逻辑切换当成物理停稳。
释放完成包含 `releaseHandles()` 和 `moveHome()`；仅张开手或仅撤离不报告 STANDBY。

START_CONTROL 的 COMPLETED 表示业务模式已切换；默认刚度恢复期间仍关闭运动目标接纳。
恢复后只接纳本次 CONTROL 和刚度恢复完成之后的新鲜输入，不能回放旧目标。
EXIT_CONTROL 保持当前参考并减速，不回到抓握起点，不自动回 home。

复位本身不启动运动、不松手、不重新抓握、不恢复 CONTROL。READY 下仍握盘时用 LEAVE_WHEEL；
已脱离时用 ENTER_STANDBY。Managed 执行器保留“可恢复位置关系”的证据：使能完成、完整抓握、
撤离完成后可恢复；接近或张开撤离中途失败且位置关系尚不明确时拒绝复位。
初始化未完成或设备仍不可用也不能仅清除错误后宣告 READY；须先排除原因并完成必要维护。
软件阶段/锁只作为程序证据，不代表物理接触传感确认。

## 3. 可恢复问题、超时与故障

可恢复问题保留原业务状态、任务代号及任务执行，不执行取消/停止副作用，也不重新提交已有动作。
当前任务完成但所需反馈暂未满足时，保留完成结果，后续周期重试完成条件；原任务期限不延期。

| 情况 | 处理 |
| --- | --- |
| 必需资源短暂不可用、跟随授权短暂丢失、CONTROL 输入/来源暂时无效 | 保持原状态；每类原因独立计时，恢复后清除对应计时。 |
| 上述原因持续达到恢复窗口 | 直接 ERROR，保留具体超时原因。默认窗口 100 ms，由 `Timeouts::recovery` 定义。 |
| CONTROL 自开始接纳或最后一条有效目标起满 100 ms 未获得有效目标 | 输入租约超时，直接 ERROR；不会再叠加恢复窗口。 |
| 设备明确阻断故障、执行器异常、任务提交失败 | 立即 ERROR。 |
| 初始化、回 home、抓握、释放或刚度任务超过原截止时间 | ERROR，取消旧代号，忽略迟到结果。 |
| 非 CONTROL 下无飞控输入 | 不触发控制输入超时。 |
| 急停锁存或急停证据不可用 | EMERGENCY_STOP，优先于普通故障、完成及请求。 |

初始化及跟随刚度任务默认 30 s；回 home、抓握、释放默认 180 s。
设备层保留更具体的停止、通信和动作超时。恢复窗口不延长设备 watchdog，也不覆盖明确的故障。
CONTROL 默认刚度恢复期间不启动“首个目标”租约，但输入/来源监督仍持续，阻抗 RPC 自身有期限。

保持 CONTROL 状态不等于接受无效数据：只有资源、跟随授权、输入来源和输入时效均有效时才能提交新的 Servo 目标。
短暂输入失效时保持现有参考；达到超时后停止并进入 ERROR。ERROR 不随输入恢复而自动退出。

## 4. 执行与取消

任务类型为 initialize、home、grasp、release、following_impedance。每个任务持有 generation 和单调时钟 deadline，
动作只登记一次，耗时函数不在 SML action 内阻塞执行。其他互斥任务返回 BUSY。
拥有线程每轮先监督急停、阻断故障及超时，再消费工作线程结果和普通请求。

取消时关闭控制接纳并推进 generation，旧完成/失败结果不得改变新状态。ERROR/急停撤销运动授权；
正常 EXIT_CONTROL 和直接撤离由既有 Servo 流程减速，保留授权以便执行后续获准动作。
工作线程未退出前不提交新任务，设备和执行器未停稳前不执行释放轨迹。

回 home 要求实际关节到达配置 home，速度低于阈值并保持稳定窗口；不以轨迹提交或软件阶段替代到位反馈。
失败原始原因在执行器暴露 FAULT 前记录，避免后续资源错误覆盖诊断。

## 5. 协议与界面

内部状态码维持既有数值：INIT=0、STANDBY=1、GRASPING=2、FOLLOWING=3、CONTROL=4、
ERROR=6、EMERGENCY_STOP=7、RELEASING=8、INITIALIZING=10、READY=11、HOMING=12。
旧 SAFE=5 和旧 STOPPING=9 保留不用，其他状态不重编号。
RS422 使用独立映射，旧 SAFE 的 0x09 保留不用，详见 [RS422 协议](AVIATOR_RS422通信协议规范.md)。

Gateway、状态播报、操作提示及测试使用这十一个整机状态。设备 `arms.*.status` 的 SAFE 是设备停止状态，
不属于本次删除的整机业务状态；设备本地停止和手节点 safe_pose 策略保留。
六操作名称不变，串口操作码仍为 0x10..0x60。ACCEPTED 表示受理，最终动作进度由状态反馈观察。

## 6. 验证与实现边界

离线状态机覆盖正常流程、两种状态直接撤离、刚度切换接管、复位 READY、恢复守卫、
短暂失效恢复、持续失效超时、任务完成等待、迟到结果和急停无出口。
Managed 测试覆盖真实工作线程与模拟设备、阻抗切换期间撤离、输入失效、失败原始诊断和恢复。
隔离 MuJoCo 进程测试覆盖运动中 CONTROL 直接释放、输入超时停稳、ERROR 显式复位及 READY 撤离。
具体命令见 [Core 说明](../nodes/aviator_core/README.md) 和 [测试说明](../tests/README.md)。

当前本地运行适配授权跟随和释放，按设备反馈及软件阶段判断就绪、停稳和脱离。
软件急停在当前进程锁存；独立硬件急停检测、跨重启锁存和物理手刹尚未接入，brake_confirmed 始终为 false。
这些软件/仿真校验不代表实机验收，也不代表已实现物理接触验证或完整柔顺跟随控制律。
