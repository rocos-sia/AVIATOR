# AVIATOR 机器人状态机设计

版本：V1.0 设计稿。范围：aviator_core 整机业务状态机及其与 Gateway、Manipulator 的接口。本文件细化状态、事件、异步动作和最小 boost::sml 实现方式，不代表运行代码或手刹控制已经实现。

## 1. 依据与设计边界

参考文件：

- [软件架构设计](AVIATOR机器人驾驶飞机控制系统软件架构设计方案.md)：Core 唯一业务仲裁、设备本地保护、运动执行与通信隔离。
- [内部 ZMQ 协议](AVIATOR_ZMQ协议格式说明.md)：强类型数据、可靠服务、控制来源、反馈有效性和错误域。
- [RS422 通信协议](AVIATOR_RS422通信协议规范.md)：六类操作、REQUEST/REPLY 一问一答、50 Hz CONTROL/STATUS。
- [本地 SML 1.2.0 头文件](../third_party/sml-1.2.0/include/boost/sml.hpp)与 [CMake 定义](../third_party/sml-1.2.0/CMakeLists.txt)：实际编译依据，头文件版本为 `BOOST_SML_VERSION == 1'2'0`。

采用单个平面 `boost::sml::sm`、一个拥有状态机的 Core 线程、一个活动耗时任务。网络、串口、SDK 回调不直接调用状态机。状态机只作守卫判定、状态转换与非阻塞动作登记，不承担规划、插值或硬件闭环。不要为每个状态建立继承层次，不引入通用流程引擎、并行状态区或重复的可写业务状态变量。

SML 的转换表由源状态、事件、守卫、动作和目标状态组成；无目标的内部转换适合幂等确认。本设计只用这些基础能力，参见 [SML 官方教程](https://boost-ext.github.io/sml/tutorial.html)。工程以本地固定版本为准，不依赖在线最新版。

本次设计按用户指定的十个状态执行。旧协议的 STOPPING 不作为本设计状态；INITIALIZING 是新增过渡状态。协议差异集中列于第 9 节，不静默更改已经形成的 RS422 Word 文件或现有协议枚举。

## 2. 状态定义

| 状态 | 分类 | 进入后的行为 | 允许主动执行飞控目标 |
| --- | --- | --- | --- |
| INIT | 起始状态 | 创建状态机；检查急停锁存；无锁存时自动投递一次内部 Boot。 | 否 |
| INITIALIZING | 过渡状态 | 异步连接、自检、校准配置检查和初始状态确认。 | 否 |
| STANDBY | 稳定状态 | 已完成初始化、已脱离方向盘，等待操作；不等于断电或自动回零。 | 否 |
| GRASPING | 过渡状态 | 执行既有接近/抓握程序；只允许一个活动任务。 | 否 |
| FOLLOWING | 稳定状态 | 经授权的柔顺或目标随动；执行批准的跟随控制律。 | 否 |
| CONTROL | 稳定状态 | 接纳合法、授权、新鲜的飞控目标，受本地输入 watchdog 限制。 | 是，仍须通过执行门控 |
| RELEASING | 过渡状态 | 执行松开并离开方向盘的程序。 | 否 |
| SAFE | 保护状态 | 中止当前普通任务、关闭主动操控、按批准策略保持/减速；待显式恢复。 | 否 |
| ERROR | 故障状态 | 记录当前及上次错误，中止任务并执行本地保护；不得自动恢复操控。 | 否 |
| EMERGENCY_STOP | 不可逆锁存状态 | 关闭操控、中止普通任务、请求拉动手刹并锁存急停。 | 否，永久禁止本状态机实例恢复 |

INITIALIZING、GRASPING、RELEASING 都是“命令已受理、任务尚未完成”的正式状态，不用布尔 `busy` 替代线上状态。状态名称定义业务模式，不保证物理动作已到位。

### 2.1 初始化边界

正常启动：`INIT → INITIALIZING → STANDBY`；初始化提交失败、自检失败或超时进入 ERROR。Boot 由 Core 启动流程自动产生，不增加串口消息或外部指令。

初始化默认不包含隐式机械臂回零、松手或其他未授权运动。成功须确认设备及配置满足待机要求、机器人已脱离方向盘且当前执行已稳定；若开机时位置或接触关系未知，进入 ERROR，由维护流程确认，不能仅因设备连接成功报告 STANDBY。若部署需要运动初始化，必须明确该任务轨迹、授权及可中止方式。

### 2.2 FOLLOWING 与退出操控

FOLLOWING 不消费主动飞控 roll/pitch，即使串口仍有 50 Hz CONTROL 输入也不得缓存为未来目标。跟随来源和控制律须预先授权。

`EXIT_CONTROL` 同步关闭主动飞控接纳并切换至 FOLLOWING。该转换的“完成”指控制模式和目标所有权已切换，不表示机械运动立即停止。执行器从当前参考连续接管、平滑减速或柔顺跟随，不回中、不跳到新目标；接管未稳定时 `settled=false`，拒绝新的 START_CONTROL 和释放请求，STATUS.phase 可报告 DECELERATE。

这一方案不增加 STOPPING。前提是现有执行器能够非阻塞地切换目标所有权并保证指令连续性；若做不到，实施前须重新评审过渡状态，不能把等待 SDK 停止的阻塞调用藏在 EXIT_CONTROL 动作里。FOLLOWING 授权丢失时优先进入 SAFE，不直接切进无授权随动。

## 3. 事件与守卫

### 3.1 六个外部事件

外部事件仅为 ENTER_STANDBY、GRASP_WHEEL、START_CONTROL、EXIT_CONTROL、LEAVE_WHEEL、RESET_ERROR。Gateway 验证协议和请求去重后转换为强类型事件，Core 是唯一决策者。RESET_ERROR 不重启进程，不清除急停锁存。

### 3.2 必要的内部事件

| 事件 | 产生者 | 用途 |
| --- | --- | --- |
| Boot | Core 启动流程 | INIT 自动开始初始化。 |
| Done{generation} | 异步执行适配器 | 当前初始化、抓握或释放任务确实完成。 |
| Fault | 故障监督/执行适配器 | 初始化或任务失败、超时、设备阻断错误、任务提交失败。错误详情进入 Context/诊断。 |
| SafetyLost | 安全监督 | 输入超时、授权撤销、双路冲突、当前模式必要反馈失效等，进入 SAFE。 |
| Emergency | 独立急停输入/监督器 | 最高优先级，进入 EMERGENCY_STOP 并请求手刹。 |

内部事件不是新增 RS422 消息。异步任务失败回调同样携带 generation，经适配器核对仍属当前任务后才转成 Fault；与任务无关的实时设备故障不受任务编号过滤。

### 3.3 最小守卫数据

| 条件 | 含义与来源 |
| --- | --- |
| ready | 当前操作所需设备、配置、标定及执行资源可用，无阻断故障；包括任务提交槽位可预留。 |
| settled | 执行器报告当前切换/停止已达到批准的稳定条件；不是 TCP/关节跟踪偏差判据。 |
| clear_of_wheel | 已完成离开程序或维护流程确认已脱离方向盘；不能由“当前状态非 CONTROL”推断。 |
| following_authorized | 跟随模式及来源获得授权，控制律可运行。 |
| source_authorized / input_ready | 飞控来源及链路授权有效，必要输入和反馈满足模式时效；不要求已收到开始操控后的首条 valid=1 目标。 |
| fault_cleared | 原故障原因已排除，复位流程获准；不等于仅删除错误码。 |
| job == none | 无活动任务，且此前任务的取消/停止已经得到确认。 |

守卫只读取 Core 本周期一致快照，不阻塞、不发命令、不自行更改状态。失败时请求拒绝，保持当前状态。对模式必需条件的持续检查由监督器负责，不只在转换瞬间检查一次。

当前双臂示例采用本地开环程序：不恢复已经废除的 TCP/关节跟踪误差或 grasp.ready 判据。软件 lock 成功只证明程序阶段完成，不能表示真实接触已验证。若飞控实装要求物理抓握证据，需要单独增加已实现的传感器能力和验收约定；缺失手部、视觉反馈仍如实标无效。

## 4. 转换表

### 4.1 正常转换与恢复

| 当前状态 | 事件 | 守卫/前提 | 下一状态 | 动作及应答 |
| --- | --- | --- | --- | --- |
| INIT | Boot | 无急停锁存、任务空闲 | INITIALIZING | 登记初始化任务；内部事件无 REPLY。 |
| INITIALIZING | Done | 当前 generation，初始化完成，ready/settled/clear_of_wheel | STANDBY | 清活动任务。 |
| STANDBY | ENTER_STANDBY | 正常状态 | STANDBY | 内部幂等转换，COMPLETED；不重复初始化。 |
| STANDBY | GRASP_WHEEL | ready、settled、跟随已授权、任务空闲 | GRASPING | 登记抓握任务，ACCEPTED。 |
| GRASPING | Done | 当前 generation，动作完成，ready、跟随仍授权 | FOLLOWING | 清任务，执行器启用跟随模式。 |
| FOLLOWING | START_CONTROL | ready、settled、跟随/飞控来源授权、input_ready、任务空闲 | CONTROL | 建立本次输入时间边界，开启目标接纳；同步模式切换成功可 COMPLETED。 |
| CONTROL | EXIT_CONTROL | 随动仍获授权；否则先处理 SafetyLost | FOLLOWING | 关闭主动目标、清目标缓存、连续接管；COMPLETED 的含义见第 2.2 节。 |
| FOLLOWING | EXIT_CONTROL | 无更高优先级安全事件 | FOLLOWING | 幂等 COMPLETED，不重新触发执行器。 |
| FOLLOWING | LEAVE_WHEEL 或 ENTER_STANDBY | ready、settled、任务空闲 | RELEASING | 登记释放任务，ACCEPTED。 |
| RELEASING | Done | 当前 generation，settled、clear_of_wheel | STANDBY | 清任务；不隐含回零。 |
| ERROR | RESET_ERROR | fault_cleared、settled、无急停锁存 | SAFE | 确认清当前故障，保留 last_error；COMPLETED，不自动回操控。 |
| SAFE | ENTER_STANDBY | ready、settled、clear_of_wheel、任务空闲 | INITIALIZING | 显式重新初始化检查，ACCEPTED。 |
| SAFE | LEAVE_WHEEL | ready、settled、任务空闲，释放动作已获准 | RELEASING | 显式受控脱离，ACCEPTED；不是安全状态自动松手。 |

ERROR 恢复统一经 SAFE：已脱离方向盘则 ENTER_STANDBY 重新检查；仍处于接触关系则按批准流程 LEAVE_WHEEL。RESET_ERROR 不直接执行释放动作。

CONTROL 下 ENTER_STANDBY、LEAVE_WHEEL 拒绝，须先 EXIT_CONTROL。所有未列出的外部事件均拒绝：过渡状态中的普通请求返回 BUSY，其他非法组合返回 INVALID_STATE；守卫失败按原因返回 CAPABILITY_UNAVAILABLE、INVALID_STATE 或 BUSY。不把未知事件自动当故障，也不重复执行耗时任务。

### 4.2 全局保护转换

| 当前状态集合 | 事件 | 下一状态 | 规则 |
| --- | --- | --- | --- |
| 除 EMERGENCY_STOP 外的全部状态 | Emergency | EMERGENCY_STOP | 不受普通任务占用、守卫或队列容量限制。 |
| INIT、INITIALIZING、STANDBY、GRASPING、FOLLOWING、CONTROL、RELEASING、SAFE | Fault | ERROR | 关闭操控，中止任务，记录错误。 |
| ERROR | Fault | ERROR | 仅更新诊断，不重复启动停止动作。 |
| GRASPING、FOLLOWING、CONTROL、RELEASING | SafetyLost | SAFE | 中止本次动作，关闭主动操控；不自动释放。 |
| INIT、INITIALIZING | 必需资源失效 | ERROR | 分类为初始化 Fault。 |
| STANDBY、SAFE | 非当前模式必需输入断流 | 原状态 | 更新健康标志，不因待机没有飞控目标而反复报控制超时。 |
| EMERGENCY_STOP | 任何事件 | EMERGENCY_STOP | 无出边；只允许诊断及手刹执行反馈更新。 |

```mermaid
stateDiagram-v2
    [*] --> INIT
    INIT --> INITIALIZING: 自动 Boot
    INITIALIZING --> STANDBY: 初始化完成
    INITIALIZING --> ERROR: 失败或超时
    STANDBY --> GRASPING: GRASP_WHEEL
    GRASPING --> FOLLOWING: 抓握程序完成
    FOLLOWING --> CONTROL: START_CONTROL
    CONTROL --> FOLLOWING: EXIT_CONTROL
    FOLLOWING --> RELEASING: LEAVE_WHEEL / ENTER_STANDBY
    RELEASING --> STANDBY: 释放完成
    ERROR --> SAFE: RESET_ERROR
    SAFE --> INITIALIZING: ENTER_STANDBY / 已脱离
    SAFE --> RELEASING: LEAVE_WHEEL / 允许释放
    CONTROL --> SAFE: SafetyLost
    GRASPING --> ERROR: Fault
    RELEASING --> ERROR: Fault
    note right of EMERGENCY_STOP
        任一其他状态可由 Emergency 进入
        拉动手刹；无任何退出转换
    end note
```

图示正常路径；故障、急停的完整适用状态以转换表为准。EMERGENCY_STOP 不画向终点的转换，因为诊断和手刹监督仍持续运行。

## 5. 耗时操作：一个任务槽、一个代号

Core 只持有一个活动任务 `Job + generation + deadline`。Job 仅有 initialize、grasp、release；generation 为进程内 uint64 递增代号，接近上限时维护，不复用。它不增加串口字段，也不替代 Gateway 的请求去重。

一次处理顺序：

1. 刷新一致输入快照；先处理急停、故障和 SafetyLost。
2. 对外部请求做守卫判定，预留唯一执行槽；状态机转换到过渡状态并登记 Job/generation。
3. `process_event()` 返回后发布新状态，再非阻塞提交已登记任务，最后形成 ACCEPTED。不能在 SML action 内执行轨迹、等待 SDK、休眠或递归调用 process_event。
4. 任务完成回调只向 Core 入队 Done{generation}。Core 核对代号和当前 Job，转换至下一稳定状态。过期、取消后或重复完成事件直接丢弃并限速记录。
5. 当前任务失败、提交失败或截止时间到达时形成 Fault，进入 ERROR。完成事件所需后置条件不成立时形成 Fault 或 SafetyLost，不静默丢弃后永久卡在过渡状态。

任务提交成功前失败可返回 REJECTED 并进入 ERROR；已经接受/开始后失败，通过内部服务结果、错误码及 STATUS 状态报告，不追加第二条 RS422 REPLY。deadline 由本地单调时钟计算，各任务最大时长按既有程序配置冻结，不拿 100 ms 应答超时当机械动作超时。

取消任务时立即推进 generation，阻止迟到回调恢复旧状态；执行器必须同步撤销旧 generation 的运动输出，清排队目标并报告取消/停止确认。在确认之前保持 settled=false、ready 或任务槽可用条件为 false，禁止开启新任务。仅在 Core 丢弃回调而设备仍继续旧轨迹不算取消。

每个登记任务只提交一次，可用一次性 pending_effect/待提交标志；不能每个控制周期看到 job 非空就重发。骨架中的 Context 只演示转换数据，不包含实际提交器。

## 6. 急停与手刹：不可退出

Emergency 的入口可以是独立硬件急停、监护进程或经过授权的本地紧急输入；它不是新增的六类远程普通操作之一。

进入 EMERGENCY_STOP 时立即完成逻辑锁存、关闭普通目标接纳、作废任务代号，并向独立安全执行通道发出“拉动手刹”请求。手刹是安全动作，不能因取消普通任务而一并取消，也不能依赖正在被停止的普通规划队列。

本状态没有通向 INIT、SAFE、ERROR 或任一运行状态的出边，RESET_ERROR、ENTER_STANDBY、任务完成及普通故障均不能使其退出。重复急停不反复启动一套普通轨迹。手刹动作由独立执行器幂等接受，持续监督确认、超时和失败；拉动成功、失败或反馈未知都保持 EMERGENCY_STOP，并分别记录诊断。不能把发出请求等同手刹已拉到位。

“不可退出”还要求禁止进程重启绕过锁存：急停状态由独立硬件/安全控制器锁存，并保留重启可读的证据。Core 启动先检查该证据，已锁存或证据不可确认时直接处理 Emergency，不执行 Boot 初始化。普通 RS422/ZMQ 服务不提供解锁；机械检修、物理解除和新任务启动条件属于本状态机之外的维护规程。

手刹执行机构、供电域、允许动作、反馈条件和最大拉动时间需与硬件方确认。若急停切断的电源也使手刹失去动力，纯软件顺序不能保证拉动，必须由独立硬件/安全通道解决。Core 失效时不能依赖这份软件状态机才触发紧急制动。

## 7. 最小代码组织与 SML 骨架

建议只新增 `nodes/aviator_core/include/aviator/RobotStateMachine.hpp` 和 `nodes/aviator_core/RobotStateMachine.cpp`，复用既有操作执行器和 Core 调度循环。纯事件/转换定义放头文件，协议映射、任务提交和状态导出放 cpp；不创建十个 State 派生类。下述为可编译的转换骨架，尚不含 SDK、消息收发、日志、队列或持久化。

Context 的 accepts_control 是执行门控，不是第二份状态枚举；每次真正提交目标仍检查 `sm.is(state<CONTROL>)`、门控和输入有效性。状态对外导出使用 `is()` 或 `visit_current_states()` 映射，不能再维护一个可独立修改的 current_state。

```cpp
#pragma once
#include <boost/sml.hpp>
#include <cstdint>
namespace aviator::fsm {
namespace sml = boost::sml;
struct INIT {}; struct INITIALIZING {}; struct STANDBY {};
struct GRASPING {}; struct FOLLOWING {}; struct CONTROL {};
struct RELEASING {}; struct SAFE {}; struct ERROR {};
struct EMERGENCY_STOP {};
struct Boot {};
struct ENTER_STANDBY {}; struct GRASP_WHEEL {}; struct START_CONTROL {};
struct EXIT_CONTROL {}; struct LEAVE_WHEEL {}; struct RESET_ERROR {};
struct Done { std::uint64_t generation; };
struct Fault {}; struct SafetyLost {}; struct Emergency {};
enum class Job { none, initialize, grasp, release };
struct Context {
  bool ready{}, following_authorized{}, source_authorized{}, input_ready{};
  bool settled{}, clear_of_wheel{}, fault_cleared{}, emergency_latched{};
  bool accepts_control{}, brake_requested{};
  Job job{Job::none};
  std::uint64_t generation{};
  void cancel() { accepts_control = false; job = Job::none; ++generation; }
  void begin(Job value) { cancel(); job = value; }
};
struct RobotMachine {
  auto operator()() const {
    using namespace sml;
    const auto free = [](const Context& c) { return c.job == Job::none; };
    const auto grasp_ok = [](const Context& c) {
      return c.ready && c.settled && c.following_authorized && c.job == Job::none;
    };
    const auto control_ok = [](const Context& c) {
      return c.ready && c.settled && c.following_authorized &&
             c.source_authorized && c.input_ready && c.job == Job::none;
    };
    const auto release_ok = [](const Context& c) {
      return c.ready && c.settled && c.job == Job::none;
    };
    const auto recover_ok = [](const Context& c) {
      return c.ready && c.settled && c.clear_of_wheel && c.job == Job::none;
    };
    const auto reset_ok = [](const Context& c) {
      return c.fault_cleared && c.settled && !c.emergency_latched;
    };
    const auto init_done = [](const Done& e, const Context& c) {
      return e.generation == c.generation && c.job == Job::initialize &&
             c.ready && c.settled && c.clear_of_wheel;
    };
    const auto grasp_done = [](const Done& e, const Context& c) {
      return e.generation == c.generation && c.job == Job::grasp &&
             c.ready && c.following_authorized;
    };
    const auto release_done = [](const Done& e, const Context& c) {
      return e.generation == c.generation && c.job == Job::release &&
             c.settled && c.clear_of_wheel;
    };
    const auto initialize = [](Context& c) { c.begin(Job::initialize); };
    const auto grasp = [](Context& c) { c.begin(Job::grasp); };
    const auto release = [](Context& c) { c.begin(Job::release); };
    const auto finished = [](Context& c) { c.job = Job::none; };
    const auto enable = [](Context& c) { c.accepts_control = true; };
    const auto disable = [](Context& c) {
      c.accepts_control = false; c.settled = false;
    };
    const auto stop = [](Context& c) { c.cancel(); };
    const auto emergency = [](Context& c) {
      c.cancel(); c.emergency_latched = true; c.brake_requested = true;
    };
    return make_transition_table(
      *state<INIT> + event<Boot>[free] / initialize = state<INITIALIZING>,
      state<INITIALIZING> + event<Done>[init_done] / finished = state<STANDBY>,
      state<STANDBY> + event<ENTER_STANDBY> / [] {},
      state<STANDBY> + event<GRASP_WHEEL>[grasp_ok] / grasp = state<GRASPING>,
      state<GRASPING> + event<Done>[grasp_done] / finished = state<FOLLOWING>,
      state<FOLLOWING> + event<START_CONTROL>[control_ok] / enable = state<CONTROL>,
      state<CONTROL> + event<EXIT_CONTROL> / disable = state<FOLLOWING>,
      state<FOLLOWING> + event<EXIT_CONTROL> / [] {},
      state<FOLLOWING> + event<LEAVE_WHEEL>[release_ok] / release = state<RELEASING>,
      state<FOLLOWING> + event<ENTER_STANDBY>[release_ok] / release = state<RELEASING>,
      state<RELEASING> + event<Done>[release_done] / finished = state<STANDBY>,
      state<SAFE> + event<ENTER_STANDBY>[recover_ok] / initialize = state<INITIALIZING>,
      state<SAFE> + event<LEAVE_WHEEL>[release_ok] / release = state<RELEASING>,
      state<ERROR> + event<RESET_ERROR>[reset_ok] / stop = state<SAFE>,
      state<GRASPING> + event<SafetyLost> / stop = state<SAFE>,
      state<FOLLOWING> + event<SafetyLost> / stop = state<SAFE>,
      state<CONTROL> + event<SafetyLost> / stop = state<SAFE>,
      state<RELEASING> + event<SafetyLost> / stop = state<SAFE>,
      state<INIT> + event<Fault> / stop = state<ERROR>,
      state<INITIALIZING> + event<Fault> / stop = state<ERROR>,
      state<STANDBY> + event<Fault> / stop = state<ERROR>,
      state<GRASPING> + event<Fault> / stop = state<ERROR>,
      state<FOLLOWING> + event<Fault> / stop = state<ERROR>,
      state<CONTROL> + event<Fault> / stop = state<ERROR>,
      state<RELEASING> + event<Fault> / stop = state<ERROR>,
      state<SAFE> + event<Fault> / stop = state<ERROR>,
      state<INIT> + event<Emergency> / emergency = state<EMERGENCY_STOP>,
      state<INITIALIZING> + event<Emergency> / emergency = state<EMERGENCY_STOP>,
      state<STANDBY> + event<Emergency> / emergency = state<EMERGENCY_STOP>,
      state<GRASPING> + event<Emergency> / emergency = state<EMERGENCY_STOP>,
      state<FOLLOWING> + event<Emergency> / emergency = state<EMERGENCY_STOP>,
      state<CONTROL> + event<Emergency> / emergency = state<EMERGENCY_STOP>,
      state<RELEASING> + event<Emergency> / emergency = state<EMERGENCY_STOP>,
      state<SAFE> + event<Emergency> / emergency = state<EMERGENCY_STOP>,
      state<ERROR> + event<Emergency> / emergency = state<EMERGENCY_STOP>
    );
  }
};
} // namespace aviator::fsm
```

Core 构造 `Context context; sml::sm<RobotMachine> machine{context};` 后，machine 初始确为 INIT。先读外部急停锁存：已锁存则处理 Emergency，否则由启动流程自动 `process_event(Boot{})`。采用显式内部 Boot 便于初始化依赖和验证初态；对飞控而言仍是自动初始化，无需发指令。

骨架的 cancel 只更新任务身份和门控；实际封装必须按第 5、6 节补齐取消、停止确认、手刹通道和执行副作用。Context 中 ready 等布尔量是守卫输入的简写，不允许业务调用者随意置真跳过检查。Fault 错误码、来源和任务失败归属在封装层记录，不需要为了示例把诊断对象塞入每条转换。

### 7.1 单线程分发与优先级

采用一个 Core 拥有线程；其他线程仅发布强类型事件。每轮先检查不可丢失的急停锁存，再处理 Fault、SafetyLost，然后是任务完成、普通请求。普通有界队列满时拒绝新请求；急停/阻断故障使用独立锁存位和监督快照，不能排在满队列后面等待。

同一轮同时出现 Done 和 Fault，以故障优先，取消后的 Done 因 generation 失效被丢弃。process_event 不跨线程调用、不在 action 内重入；本设计不依靠额外 SML 线程锁掩盖共享资源竞态。发出每个普通执行副作用前再次检查急停锁存，实际驱动通道也独立执行安全门控。

`process_event()` 的 bool 只表示事件是否被状态机处理，不是动作物理成功。返回 false 后由请求适配器区分 BUSY、非法状态与守卫失败；完成事件返回 false 不产生外部拒绝应答。已处于 ERROR 的再次 Fault 在封装层更新诊断，不要求 SML 自迁移。

## 8. CMake 引用本地 sml-1.2.0

库已经提供 INTERFACE 目标 `sml` 和别名 `sml::sml`，无需编译库文件、find_package(Boost) 或从网络下载。项目采用 C++20，与该库要求兼容。

以下片段拟放在 `nodes/aviator_core/CMakeLists.txt` 现有提前 return 判断之后、创建/链接业务目标的位置；只在 Core 实际启用时引入，避免破坏 AVIATOR_COMMUNICATION_ONLY 构建：

```cmake
# 保留文件开头原有 AVIATOR_COMMUNICATION_ONLY / robotics 判断。
if(NOT TARGET sml::sml)
  set(SML_BUILD_EXAMPLES OFF CACHE BOOL "Build SML examples" FORCE)
  set(SML_BUILD_TESTS OFF CACHE BOOL "Build SML tests" FORCE)
  set(SML_BUILD_BENCHMARKS OFF CACHE BOOL "Build SML benchmarks" FORCE)
  add_subdirectory(
    "${PROJECT_SOURCE_DIR}/third_party/sml-1.2.0"
    "${CMAKE_CURRENT_BINARY_DIR}/sml"
    EXCLUDE_FROM_ALL
  )
endif()

# 以下两行放在已有 add_library(aviator_core_control ...) 之后。
target_sources(aviator_core_control PRIVATE RobotStateMachine.cpp)
target_link_libraries(aviator_core_control PUBLIC sml::sml)
```

这里采用 PUBLIC 是因为建议的公开 RobotStateMachine.hpp 包含 `<boost/sml.hpp>`；如果后续把 SML 完全隐藏到 cpp，可改为 PRIVATE，但不要仅为隐藏依赖增加不必要的 Pimpl 层。现有 `aviator_core`、`aviator_core_servo` 已链接 aviator_core_control，无需分别重复添加头文件路径。

本文件只提供接入方案，没有提前添加不存在的 cpp 或修改实际构建入口。将来实现时再应用上述片段。

## 9. 协议映射与必须同步的差异

| 接口项 | 当前文档 | 本设计及实施时的同步要求 |
| --- | --- | --- |
| INIT / INITIALIZING | RS422 仅有 INIT | INIT 保持 0；建议新增 INITIALIZING=10，不挪用现有状态值。启动 INIT 可非常短，外部可能首次只观察到 INITIALIZING。 |
| 其他状态值 | STANDBY=1、GRASPING=2、FOLLOWING=3、CONTROL=4、SAFE=5、ERROR=6、EMERGENCY_STOP=7、RELEASING=8、STOPPING=9 | 保留 1..8；9 退役保留，不发送；本设计不含 STOPPING。 |
| EXIT_CONTROL | RS422 经 STOPPING 到 FOLLOWING | 改为 FOLLOWING 直接接管，phase 可为 DECELERATE；该协议变更须同时更新 Markdown/Word 并确认。 |
| ENTER_STANDBY | RS422 主要是 STANDBY 幂等确认 | 增加 FOLLOWING→RELEASING 和 SAFE→INITIALIZING 的明确路径；仍拒绝 CONTROL 直接待机。 |
| RESET_ERROR | RS422 可恢复 SAFE/STANDBY | 收敛为 ERROR→SAFE，不直接恢复 STANDBY 或 CONTROL。 |
| EMERGENCY_STOP | 现有协议描述为硬件急停/驱动保护 | 本设计明确手刹动作和无出口锁存。一般可恢复驱动故障归 ERROR，不一律映射成不可逆急停。 |
| ZMQ system.state | 旧枚举缺 INITIALIZING、RELEASING | 增加两个字符串枚举及接收端校验；保留内部服务 request_id/session/epoch 机制。 |
| STATUS.operation | 最近受理操作类别 | 初始化 Boot 非外部操作，启动时为 0；正常外部事件受理后更新；拒绝不覆盖。 |
| STATUS.phase | 已有阶段 0..7 | INITIALIZING 默认 NONE，详细自检子阶段写内部诊断；无需为最小设计扩展线上字段。 |

六个 REQUEST 操作码仍为 0x01..0x06，不增加线上消息类型。外部运行毫秒时间戳、双路去重和回绕处理仍在 Gateway；异步任务 generation 只用于 Core 内部。

RS422 REQUEST：Core 实际受理后返回一次 REPLY，过渡操作为 ACCEPTED，短模式转换或幂等确认可 COMPLETED；后续仅经 STATUS 的 state/phase/error 观察进度。最新协议没有逐请求最终结果字段，不能重新偷偷加入。ZMQ 内部可靠服务可使用已有结果查询契约；串口不因此增加 get_result 消息。

CONTROL 只在 CONTROL 状态及门控有效时转换为执行目标；首条有效目标 100 ms 和连续输入超时采用协议既有预算。状态机周期不能用“收到任何串口字节”刷新 watchdog。错误和无效反馈规则保留，状态导出为 50 Hz 聚合，不宣称状态机或动作本身只有 50 Hz 响应能力。

## 10. 验证与实施顺序

先验证纯状态机，再接入异步执行器，最后接协议和实机；不要在串口回调中直接调用阻塞的抓握/初始化函数。

| 验证项 | 预期 |
| --- | --- |
| 正常启动 | 构造为 INIT；自动 Boot 后 INITIALIZING；成功后 STANDBY。 |
| 初始化失败/超时/提交失败 | ERROR，不假报 STANDBY。 |
| 正常循环 | STANDBY→GRASPING→FOLLOWING→CONTROL→FOLLOWING→RELEASING→STANDBY。 |
| 守卫失败/非法事件 | 状态不变，明确拒绝；STANDBY 不能直接 START_CONTROL。 |
| 重复请求/过渡中再请求 | 同请求不重复执行，其他互斥请求 BUSY。 |
| EXIT_CONTROL 接管 | 立即不接纳飞控目标；未稳定时拒绝释放/重新操控，指令连续。 |
| 故障复位 | 故障未消除时拒绝；清除后 ERROR→SAFE；仍需显式恢复。 |
| 迟到 Done/失败回调 | generation 不匹配则不改变状态；不让旧动作覆盖新状态。 |
| 同轮故障与完成 | 先处理故障，取消代号后完成无效。 |
| 任一状态急停 | 均进入 EMERGENCY_STOP，普通任务中止，手刹请求发出。 |
| 急停重复/手刹失败/任务完成/RESET_ERROR | 保持 EMERGENCY_STOP，不恢复运动。 |
| 急停后重启 | 读取外部锁存，直接急停，不先初始化再检查。 |
| 队列满/执行器无法取消 | 不丢急停，拒绝新任务；未确认停止不得报告 settled。 |
| 缺少手部/视觉或物理抓握证据 | 如实无效，不把软件阶段结束当物理验证。 |

本文转换骨架已在临时独立工程中通过本地 `third_party/sml-1.2.0` 的 `add_subdirectory` / `sml::sml` 目标，以 C++20 Debug 构建并运行断言测试。覆盖启动、正常循环、守卫拒绝、释放、SafetyLost、故障复位、初始化失败、迟到完成、全部九个非急停状态进入急停，以及急停后六种外部事件均不可退出；不代表异步执行器、手刹硬件或全部故障注入已经通过实机验收。

实施前需冻结：FOLLOWING 连续接管接口、三类任务最大时长、ready/settled 的实际来源、取消确认机制、SAFE 释放授权、急停独立执行及跨重启锁存、INITIALIZING 状态码和旧 STOPPING 的协议迁移。

