# aviator_core

当前保留直接执行、受状态机管理、纯离线测试三类入口：

| 程序 | 用途 | 执行方式 |
| --- | --- | --- |
| `aviator_core` | 原交互、`--demo`、`--servo-demo` | 直接顺序调用 Aviator 功能函数，保留动作检查和设备 watchdog |
| `aviator_core_servo` | 原自动抓握和摇杆控制 | 直接调用功能函数，Servo 周期更新目标 |
| `aviator_core_managed` | 六操作管理设备 | 调用 Aviator 大写接口，异步执行原有功能；使用设备反馈、输入时效及本地测试策略，不依赖安全文件 |
| `aviator_core_sml` | **纯软件整机状态机测试** | 真实 SML 转换逻辑 + 模拟守卫、执行器、时钟；不连接设备 |

两个原 Core 使用小写接口绕过整机业务状态机，不能把它们的执行器阶段当作十二状态整机状态。
真实手开合与 HandLink 独立通信线程保留。设备入口均不依赖 safety.json；同一 Manipulator 只运行一个设备控制 Core。

Core 负责 PIN-IK、Pinocchio 碰撞检查和 Ruckig 规划，通过 RemoteLink 与设备节点通信，不直接加载 xCore SDK 或创建 MuJoCo 窗口。

## 离线状态机测试入口

参考 `SurgicalRobotArm/src/robot.cpp` 的“事件 → SML 转换表 → 动作 → 完成/失败事件”结构，
保留 [整机状态机设计](../../docs/AVIATOR机器人状态机设计.md) 的十二状态和六操作。

- `include/aviator/RobotStateMachine.hpp`：事件、守卫、转换表及动作登记。
- `RobotStateMachine.cpp`：请求结果、任务 generation / 超时、完成和保护事件。
- `main_sml.cpp`：终端测试及模拟执行器；不实例化 Aviator / RemoteLink，只链接状态机库。
- `Aviator.cpp` 的 `Aviator::Managed`：拥有实际状态机和唯一任务槽，执行器完成/异常由拥有线程处理。
- `StateMachineRuntime.cpp/.hpp` 和 `ManagedGateway.cpp/.hpp`：`aviator_core_managed` 的设备反馈、终端和 Gateway 适配，调用 Aviator 大写 API，不再另建状态机。

状态机动作只登记任务，耗时执行与转换表分开。测试执行器通过 `done` / `fail` 回传结果，
所有状态机调用在单线程进行，不照搬参考文件的全局 Robot 指针和多线程直接修改状态机方式。
功能动作仍留在 Aviator / RemoteLink；Managed 路径已复用这些功能，不重复规划或手控制代码。

从仓库根目录运行，不需要 Bus、Manipulator、手节点、相机、配置文件或安全文件：

```bash
cmake -S . -B build
cmake --build build --target aviator_core_sml aviator_robot_state_machine_test -j2
# 自动演示完整正常状态流程，完成后退出
./build/bin/aviator_core_sml --demo
# 交互测试
./build/bin/aviator_core_sml
```

也可在不构建机器人依赖的配置下编译：

```bash
cmake -S . -B build-sml -DAVIATOR_COMMUNICATION_ONLY=ON -DAVIATOR_BUILD_EXAMPLES=OFF
cmake --build build-sml --target aviator_core_sml aviator_robot_state_machine_test -j2
```

启动后模拟 Boot 进入 INITIALIZING，普通守卫默认 true、emergency_latched 默认 false。
执行中 executor_idle / settled 为 false；`done` 模拟任务结束，恢复两者为 true；抓握开始后
clear_of_wheel 为 false，释放完成后为 true。手工设置的其他证据仍须满足状态机守卫。
这些值只存在测试进程内，不是真实设备反馈。

正常流程逐条输入（第一个 done 完成初始化/使能；ENTER_STANDBY 后的 done 模拟回 home 完成）：

```text
done
ENTER_STANDBY
done
GRASP_WHEEL
done
START_CONTROL
EXIT_CONTROL
LEAVE_WHEEL
done
quit
```

对应 `INIT → INITIALIZING → READY → HOMING → STANDBY → GRASPING → FOLLOWING → CONTROL → FOLLOWING → RELEASING → STANDBY`。
`done` 表示模拟动作完成，不会发送张开、闭合、使能或撤离指令。

| 指令 | 作用 |
| --- | --- |
| 六操作名称 | ENTER_STANDBY、GRASP_WHEEL、START_CONTROL、EXIT_CONTROL、LEAVE_WHEEL、RESET_ERROR |
| `status` / `help` | 查看当前状态、控制接纳、错误、任务编号及全部模拟守卫 / 帮助 |
| `set <guard> <0\|1>` | 修改守卫并立即执行监督；例如 `set source_authorized 0`、`set input_ready 0` |
| `done` / `fail` | 模拟当前任务完成 / 失败 |
| `late_done` | 重放上次任务的完成事件，测试取消后的迟到回调 |
| `advance <毫秒>` | 推进模拟时间，不实际等待，范围 0～3600000 |
| `fault` / `safety_lost` / `emergency` | 注入故障 / 安全条件丢失 / 急停 |
| `quit` | 退出测试 |

例如 GRASPING 中输入 `advance 180000` 触发抓握任务超时；INITIALIZING 与 RELEASING 的预算分别为
30000 / 180000 ms，独立回 home 任务也是 180000 ms。CONTROL 中 `set input_ready 0` 模拟输入失效进入 SAFE，恢复为 1 不自动回 CONTROL。
`fail` 进入 ERROR 后可用 RESET_ERROR 到 SAFE；若仍未脱离方向盘，ENTER_STANDBY 会拒绝，
可模拟获准的 LEAVE_WHEEL，再 done 返回 STANDBY。

输出包含事件、转换前后状态、请求前守卫快照、接纳/拒绝结果和模拟动作。
模拟 stop 立即确认稳定，不模拟实际制动时间。时间仅由指令推进，交互等待不会触发真实输入时效；
测试输入超时用 input_ready=false，不宣称验证了 ZMQ watchdog。
急停只在当前进程锁存，普通请求不能解除；不写急停文件、不请求实际手刹，重启创建新的模拟实例。
独立安全监督器、物理手刹、真实抓握及真实 FOLLOWING 控制律均不属于此入口的验证范围。

离线回归（Python 只使用标准库）：

```bash
ctest --test-dir build -R '^(robot_state_machine|core_sml_demo|core_sml_cli)$' --output-on-failure
```

覆盖正常流程、非法状态、授权守卫、故障恢复、输入失效、任务超时、迟到事件与急停无出口。
`robot_state_machine_process_test.py` 单独验证 Managed 路径与隔离 MuJoCo 的集成，见下节；它不属于纯离线入口。

## Aviator 的最小状态机接入

构造时不传 `ManagedOptions` 为 Direct 模式，两个原 main 只把动作调用改为小写，流程不变。
传入 `ManagedOptions` 为 Managed 模式，外部小写动作会抛错，避免绕过正在工作的状态机。
两个模式共用同一组 `Impl` 功能实现；原接近和撤离轨迹不重写，只增加组合任务以及再次抓握前的张开确认。

释放失败会在执行器进入 FAULT 前记录 `ReleaseHandles [阶段]: 原因`，包含规划起点、
正/逆解、撤离轨迹检查、张开与撤离执行阶段。状态机据此进入 ERROR 并保留原始错误，
避免先因 `ready=false` 进入 SAFE、更新任务编号后忽略工作线程的失败结果。

| Managed API | 事件 / 行为 |
| --- | --- |
| `Init()` | 内部 Boot → initialize 任务；初始化资源、双臂使能及阻抗配置完成后 READY，保持当前位置，不自动回 home |
| `EnterStandby()` | ENTER_STANDBY；READY/已脱离的 SAFE → HOMING，实际到 home 并停稳 → STANDBY；STANDBY 幂等；FOLLOWING 先释放撤离再回 home |
| `GraspWheel()` | GRASP_WHEEL；确认手张开 → approachHandles(from_home=true) → lockHandles；不重复使能或回 home |
| `StartControl()` | START_CONTROL；检查守卫并建立新输入时间边界 |
| `ExitControl()` | EXIT_CONTROL；关闭接纳并非阻塞减速，等 settled 后才能释放/重启操控 |
| `LeaveWheel()` | LEAVE_WHEEL；releaseHandles 张开撤离 → moveHome，实际回 home 并停稳后 STANDBY |
| `ResetError()` | RESET_ERROR；故障已排除后确认本地错误，ERROR → SAFE，不调用会隐式解锁/失能的 resetFault |
| `ServoWheel(a,d,v,sample_mono_us)` | 目标接口，不是第七个操作；仅 CONTROL 接受本次操控后的有效时间戳，拒绝过期、未来、重复/倒退目标 |
| `Update()` | 拥有线程每约 5 ms 调用，刷新证据/心跳，监督、收取任务结果和执行转换 |
| `EmergencyStop(reason)` | 内部急停输入，撤销普通输出并请求独立手刹通道，不是普通六操作 |

直接接口为 `init/enable/disable/approachHandles/lockHandles/moveWheel/servoWheel/unlockHandles/releaseHandles/resetFault/acknowledgeFault/stop`。
原来的单步大写 Enable、ApproachHandles、LockHandles 等不再作为公共 API；调用方使用直接接口或六个业务操作。
`GetState()` 仍为执行器阶段，`GetSystemState()` / `GetSystemStatus()` 仅用于 Managed 整机状态，二者不混用。

`ManagedOptions` 必须提供 snapshot、allow_motion、heartbeat、request_brake 回调，可另提供 report。
回调来自可信运行适配器，不能让普通业务消息直接填写任意守卫。所有大写管理接口在创建 Aviator 的同一线程执行，
耗时动作由一个工作线程完成；等待结果时仍须持续调用 Update，不能在拥有线程上阻塞等待整个抓握过程。
此设备测试入口以本地策略授权跟随和释放，ready/settled/fault_cleared 使用设备反馈和执行器情况。调用 ServoWheel 前由输入适配器验证来源、会话和数值，
sample_mono_us 必须来自该有效输入；POSITION_HOLD 使用通过校验的 checked_mono_us 作为目标有效期依据，原始轴事件时间保留在输入报文中。不能用 Core 当前时间伪造续期。

独立设备控制入口（不要与两个原 Core 同时控制一个 Manipulator）：

```bash
cmake --build build --target aviator_core_managed -j2
# 设备、Bus、手节点准备好后；--console 为终端目标测试模式
./build/bin/aviator_core_managed --config config/system.yaml --console
# MuJoCo 使用同一套设备反馈守卫，通过启动 simulation 替代真机设备节点
./build/bin/aviator_core_managed --config /path/to/mujoco/system.yaml --console
```

初始化/使能完成后停在 READY。先输入 ENTER_STANDBY，等待 HOMING → STANDBY，再输入 GRASP_WHEEL，等待 FOLLOWING；输入 START_CONTROL 后需要程序每小于 100 ms 更新
`servo <angle_rad> <displacement_m> [v]`，普通手工打字不适合持续控制。EXIT_CONTROL 后等待 settled，
再 LEAVE_WHEEL。Managed 入口没有 `done`，完成事件来自真实执行函数返回和设备执行确认。
`quit` / Ctrl+C 撤销输出并等待任务退出，不自动执行正常撤离；正常离开须先走 EXIT_CONTROL / LEAVE_WHEEL。

此入口不再读取 safety.json，也不再支持 `--safety-file` / `--fsm-simulation`，system.yaml 无需 managed_core.safety_file。
真机和 MuJoCo 使用同一套运行适配；通过运行 manipulator + rh56ftp_hand 或 simulation 选择设备，robot.yaml 不再包含 backend。各条件来源为：

- ready：通常要求实时反馈新鲜且无故障。设备的 UNINITIALIZED/INITIALIZED/DISABLED 阶段在无轨迹、未停止中时，允许用新鲜设备状态进入待机/使能流程；使能完成、接近及操控仍要求实时反馈。fault_cleared 仍要求实时反馈新鲜且无故障。Aviator 还检查本地执行器故障。
- settled：设备停止确认、撤销输出后的轨迹清空、工作任务结束及执行器阶段。
- input_ready / source_authorized：Gateway 输入有效期及绑定会话；终端测试仍受 ServoWheel 的 100 ms 目标时效限制。
- clear_of_wheel：设备软件锁已解除，且执行器处于初始化/已使能/已失能等非接触流程阶段；这只是程序判据，不是物理脱离检测。
- following_authorized / release_authorized：启动这个本地测试入口即允许这两类业务操作，具体执行仍由状态机守卫决定。

终端 emergency 和 Aviator::EmergencyStop 仍进入不可用普通请求退出的 EMERGENCY_STOP。
锁存仅在当前进程内保存，不再读写 `.emergency` / `.brake-request` 文件；进程重启不恢复该软件锁存。
硬件急停状态未独立接入，不能将软件急停状态解释为硬件急停检测。

**当前接入限制**：未接入独立安全监督器和物理手刹驱动，brake_confirmed 始终为 false，急停时 brake_status=UNAVAILABLE；FOLLOWING
仅保持现有参考，需要相应策略授权，不是柔顺控制。保护/退出撤销手目标后仍沿用手节点的默认张开 safe_pose，
不能宣称满足设计文档的保护时抓握保持。真实硬件策略需另行落实。本次实现和验证没有驱动真机。

```bash
ctest --test-dir build -R '^(aviator_managed_api|robot_state_machine|core_sml_demo|core_sml_cli|core_hand)$' --output-on-failure
# 需要 pyzmq / PyYAML；测试自动使用随机 loopback 端口和临时 MuJoCo 配置
~/miniconda3/envs/apriltag_realsense/bin/python tests/robot_state_machine_process_test.py \
  "$PWD/build/bin/aviator_bus" "$PWD/build/bin/simulation" "$PWD/build/bin/aviator_core_managed" "$PWD"
```

## Managed Core：摇杆按钮与连续目标驱动真机

默认启用 Gateway 模式，无需输入 `--flight-gateway`。`ManagedGateway` 在拥有线程中同时处理：

- `flight.command`：SUB 连接 system.yaml 的 Bus 输出，校验生产者、固定 Gateway 会话、时钟、序号、有效位和时效。
- 六操作服务：ROUTER 默认 bind `tcp://127.0.0.1:5559`，调用 Aviator 六个大写接口，由同一状态机决定是否执行。
- 仅 CONTROL 向大写 `ServoWheel()` 传目标，沿用当前反向 roll 配置 `angle = roll * -0.87266` rad、`displacement = 0.085 * (pitch - 1.0)` m、`v = 1`。最大约 50 Hz，不补发错过周期。

目标仍是绝对目标；进入 CONTROL 后，下一份通过校验的输入可能立即要求运动到当前摇杆对应位置。
静止摇杆通过新鲜设备检查保持位置；重复检查时间不能续期。原始轴事件可早于本次 START_CONTROL，
但用于放行的设备检查时间必须晚于本次操控开始。普通非 POSITION_HOLD 输入仍使用原始采样时间。
有效操控时 `flight.state.system.control_source=JOYSTICK`，否则为 NONE；额外导出 input_ready、source_authorized。

启动即自动使能双臂：Rokae 复用 `prepare()` 的上电、状态订阅及 `setJointImpedance`，随后 `startMove(jointImpedance)` 保持当前位置。
现有 RemoteLink 使能流程会先张开手并等待确认，因此需先启动手节点。任一准备步骤失败进入 ERROR，不进入 READY。
新增 READY=11、HOMING=12，原状态编号保留。STANDBY 要求 home 轨迹完成后，每个关节实际位置误差不超过
`robot.yaml: home_position_tolerance`（当前配置 0.15 rad，约 8.59°；省略时默认 0.02 rad，允许范围 (0, 0.15]），且速度连续 0.15 s 低于 0.02 rad/s；到位等待最多 5 s。
若报 `Home settling timeout`，错误中会列出左右各关节的 `target_rad`、`actual_rad`、`error_rad`、
`velocity_rad_s`、`position_ok` 和 `speed_ok`，以及最长连续达标时间 `longest_stable_ms`。
轨迹执行结束不代表实测到位；依据上述反馈区分位置残差和速度未稳定，超时仍进入 ERROR。
Direct Demo 原先只检查 home 停稳的逻辑保持不变；此新位置判据仅用于 Managed 回 home。

### 启动顺序

从仓库根目录，在不同终端执行。以下是实际设备入口，不与其他 Core 同时运行。
CAN 接口按手节点 README 预先配置，`config/robot.yaml` 选择实际后端。

```bash
cmake --build build --target aviator_bus aviator_hand manipulator aviator_core_managed flight_gateway -j2

# 1. Bus
./build/bin/aviator_bus
# 2. 真实手节点
./build/bin/aviator_hand --config config/inspire_hand.yaml
# 3. 双臂设备节点；sudo 用于真机实时调度权限
sudo ./build/bin/manipulator --config config/system.yaml
# 4. 启动 Managed Core，无需安全证据文件
./build/bin/aviator_core_managed --config config/system.yaml
```

不需要创建 `/run/aviator/safety.json`，也不需要启动安全文件监督程序。Gateway 模式默认开启；
`--console` 可切换到原终端目标测试模式。设备故障、反馈丢失、输入超时、任务超时及非法状态转换检查仍保留。

Gateway 默认自动识别 Core 会话，无需复制 UUID。保持 [config/flight.yaml](../../config/flight.yaml) 默认值即可：

```yaml
core_session: ""
service: tcp://127.0.0.1:5559
```

核对 device 指向实际 evdev 摇杆后，启动 Gateway：

```bash
# 5. Gateway；所有参数从 config/flight.yaml 读取
./build/bin/flight_gateway
```

Gateway 会用首次成功设备查询建立初始位置，摇杆静止也可提供有效输入；Core 应打印 `Received valid flight_gateway input` 和 `Managed flight input: valid`。会话不再参与授权，`--gateway-session` 与配置的 `core_session` 仅兼容旧调用并忽略。Gateway 收到新鲜 Core 状态及 source_authorized 确认后显示 `service_ready=1`，此时可按按钮；反馈过期时按钮仍显示 NOT_SENT，不缓存。

可用 `--operation-service tcp://127.0.0.1:<端口>` 更改 Core 服务地址，同时修改 flight.yaml 的 service；
该端点不能与 Bus 或 Manipulator 服务相同。Gateway 模式在 stdin 关闭时继续工作，可用 Ctrl+C 退出。
终端六操作、status、emergency、quit 保留；终端 servo 在此模式下拒绝，避免混用目标来源。

### 按钮验证

按钮编号由 Gateway 打印的 evdev 映射决定，不保证等于外壳标号。默认流程：

| 步骤 | 默认按钮 | 预期行为 / 状态 |
| --- | --- | --- |
| 初始化 | 无需按钮 | INITIALIZING → READY；自动使能并设置阻抗，保持当前位置 |
| 回 home 待机 | 1 / enter_standby | READY → HOMING → STANDBY；已在 STANDBY 则幂等确认 |
| 抓握 | 2 / grasp_wheel | ACCEPTED，GRASPING → FOLLOWING |
| 操控 | 3 / start_control | 输入有效且守卫满足后 CONTROL，连续轴值驱动机器人 |
| 退出操控 | 4 / exit_control | FOLLOWING；等待 `system.settled=true` |
| 释放撤离 | 5 / leave_wheel | ACCEPTED，RELEASING 内完成松手、撤离及回 home → STANDBY |
| 故障确认 | 6 / reset_error | 仅 ERROR 且故障已消除等条件满足时 → SAFE，不自动回操控 |

CONTROL 中按钮 1/5 不直接释放，先按钮 4。FOLLOWING 中按钮 1 同样执行释放、撤离及回 home。
新鲜输入恢复不会自动退出 SAFE/ERROR；按实际状态和证据走恢复流程。
**现有保护未更改**：输入/设备检查失效触发 CONTROL → SAFE 并撤销普通输出，后续设备故障可能进入 ERROR；
手节点仍可能因目标超时执行默认张开 safe_pose。它不同于按钮 4 的正常减速保持，实际测试前须理解此行为。

服务即时回复 ACCEPTED/COMPLETED/REJECTED/EXPIRED；REJECTED 的 `result.reason` 给出 BUSY、INVALID_STATE、
CAPABILITY_UNAVAILABLE 或协议拒绝原因。ACCEPTED 仅表示登记任务，动作完成看 flight.state 的最终状态和错误。
此最小适配不实现 get_result 或异步最终服务应答；相同请求重发返回原始应答，不作为完成查询。
请求在 Core 会话内去重，最多保存 1024 项，不淘汰后重复执行；满后拒绝新请求，需在安全停止后重启。
服务期限检查范围 1–10000 ms；Gateway 默认等待 100 ms，该期限不是机械动作时限。
超时显示 UNKNOWN，不自动重试或用新请求猜测执行结果。记录副本不执行；身份白名单仅适用于受控本机，不是密码学认证。

### 启动与请求拒绝排查

Rokae 未使能时尚无 RT TCP/速度采样，不能把这一条件当成初始化前的掉线。
Manipulator 的 arm.state 新增 `status_mono_us`，仅代表设备状态发布时间；Core 仍保留原始关节采样时间及 valid。
本次更新后需**同时重启 Manipulator 和 Managed Core**；旧设备节点没有此状态时间字段，不能用于新待机判据。

Gateway 按钮请求和连续轴值走不同通道，首次按钮可能先于首个有效轴值抵达。
先看到 `Received valid flight_gateway input` 和 `system.state=READY`，按按钮 1 回 home；等 `system.state=STANDBY` 再按按钮 2 抓握，输入有效后按按钮 3 操控。
请求拒绝的 result.reason 包括 GATEWAY_NOT_BOUND（尚无通过校验的输入）、CLOCK_DOMAIN_MISMATCH（时钟域不符）和 INVALID_PARAMETERS；应答保留 expected_clock_id。已取消 Gateway/Core 会话匹配及 server_session_id 必填要求，旧客户端提供此参数时忽略。
ERROR 状态不会因摇杆输入恢复而自动退出；故障排除后按 RESET_ERROR → SAFE，再 ENTER_STANDBY。

### 隔离联调

```bash
# 需要 pyzmq / PyYAML；临时配置固定 MuJoCo，四个随机 loopback 端口，不使用实际摇杆/CAN/机器人
~/miniconda3/envs/apriltag_realsense/bin/python tests/managed_gateway_process_test.py \
  "$PWD/build/bin/aviator_bus" "$PWD/build/bin/simulation" "$PWD/build/bin/aviator_core_managed" "$PWD"
# 若 CMake 选用的 Python 具备依赖，也会注册为 CTest：
ctest --test-dir build -R '^managed_gateway_process$' --output-on-failure
```

覆盖 Gateway 同格式请求/输入、来源/时钟/期限校验及跨启动实例接收、去重及冲突、非法状态和忙碌拒绝、静止位置保持到达目标、
完整抓握/操控/释放、设备检查冻结触发保护、恢复输入不自动恢复操控及 stdin EOF。测试不验证实际 USB 按钮标号或真实抓握。

## 原 Core 构建与使用

从 AVIATOR 根目录构建：

```bash
cmake -S . -B build
cmake --build build --target aviator_bus aviator_core aviator_core_servo simulation -j3
```

三个终端分别启动：

```bash
./build/bin/aviator_bus --config config/system.yaml
./build/bin/simulation --config config/system.yaml
./build/bin/aviator_core --config config/system.yaml --demo
```

`simulation --headless` 关闭仿真窗口。Core 的 `--servo-demo` 演示每 20 ms 更新目标；不加 Demo 选项进入交互模式：

```text
enable
approach
lock
wheel 0.1 -0.01 0.5
servo 0.02 -0.002 0.5
status
stop
unlock
disable
reset
quit
```

阻塞动作在工作线程执行，过程中可输入 status 或 stop；其他动作需等待 `[ok]`。Servo 超过 robot.yaml 的 servo_timeout 没有新目标则停止并保持。v 是速度倍率 `(0,1]`，不是时间。主线程保持任务监督心跳，机械臂与手部各自的通信线程独占自己的 PUB/SUB，规划线程只操作强类型接口。

`main.cpp` 保留易读的顺序 Demo 和目标数组。`Aviator.cpp` 实现业务与规划；`RemoteLink` 将已检查的轨迹分成有界窗口发布，等待执行进度，不以实际关节/TCP 到位误差判断完成。初次使能从设备返回的实际保持目标开始，后续从上一条已下发目标衔接。仿真与真机共用上述代码。

`ServoPlanner` 用持久的二维 Ruckig 状态在线规划轮盘角度/推拉位移。新目标继承上一段末端的位置、速度、加速度；不是每 20 ms 从静止重新规划。双臂在同一轮盘路径上用 Pinocchio 微分运动学延续接近阶段选定的逆解分支，并抑制冗余 J2 漂移。开始 Servo 时保留上一条关节指令 FK 的微小数值残差（不超过 2 µm / 2 µrad），避免在首个短周期内突然修正残差；不使用实测 TCP 做此处理。关节段采用匹配两端 q/dq/ddq 的五次曲线，逐毫秒检查关节位置限位、有限数值和规划几何。Servo 规划、传输和执行层不再按 joint_speed/joint_acceleration/joint_jerk 拦截。这里的几何检查针对**规划指令**，没有恢复实际 TCP/抓取偏差到位判断。

Ruckig 0.15.3 的 `-111` 表示时间同步求解失败。目标保持或减速接近目标时，极小的浮点残差
可能使同步时间落在数值退化边界。Servo 仅对该错误重试一次：保留原始位置、速度、加速度、
目标、运动约束和 `Time` 同步，将最短轨迹时长设为失败求解的时长加一个 `servo_period`。
重试成功记录 `Servo Ruckig synchronization recovered` 及完整输入；其他错误或重试失败仍
进入原有 FAULT 处理，并输出错误码、周期、停止标志和完整规划输入，供复现排查。

Servo 预填充至少 80 ms，随后持续追加不可改写的样本，Core 队列上限 250 ms；正常提前量约 80～100 ms（周期大于 20 ms 时可到 130 ms）。ZMQ 每帧最多 81 个原始 1 ms 样本，附带 q/dq/ddq，公共 JSON 报文上限为 128 KiB；Manipulator 不再对 Servo 做 2 ms 线性插值，两臂共用执行游标。窗口包含反馈游标前的 4 个历史点，实际未来余量还需扣除反馈与传输延迟；50 ms 命令 watchdog 保持不变。缓冲耗尽、通信超时、限位或 SDK 故障仍会停止并报错，不能重复旧点伪装成正常执行。RT 回调不做 IK、Ruckig 或 JSON 编解码。

持续 Servo 的 `first_tick` / `total_ticks` 是累计的 1 ms 索引，不受普通预规划轨迹的 3,600,000 tick（一小时）长度预算限制，因此不会在连续运行一小时或 24 小时时因累计索引被拒收。内存仍由滚动窗口和 Core 队列限制，线上的整数仍受 uint53 精确范围约束。`motion_protocol` 回归覆盖 1、24、48 小时边界前后的窗口及结束条件；`control_nodes_servo_window` 验证大累计长度经实际总线进入仿真执行器后仍能执行、停止并触发原有看门狗。这些加速边界测试不等同于真机连续 24 小时稳定性测试。

Managed 进入 SAFE 等保护状态时，RemoteLink 撤销普通轨迹发布并异步请求设备本地停止，期间继续接收反馈和更新 Core 心跳。Servo 的取消会退出补窗/排空等待，设备确认停止后保持当前指令；输入恢复不会自动重新授权。正常 EXIT_CONTROL 仍使用规划器减速。更新窗口协议后需同时重编译并重启 Core、Manipulator、Bus，以及读取 arm.command 的相关节点。

`robot.yaml` 新增 `wheel_angular_acceleration`、`wheel_linear_acceleration`、`wheel_angular_jerk`、`wheel_linear_jerk`。Servo 继续按 `wheel_*_speed` 及这些轮盘加速度/jerk 参数规划；移除关节动态上限检查不代表生成的关节轨迹满足原动态上限。Rokae 后端按 URDF 速度限制的单周期防跳变检查和控制器自身保护仍保留。正常 Stop/输入超时在已提交的短缓冲之后按 Ruckig 速度模式减速至零，再保持最终位置；完成制动需要时间。通信或规划故障走后端独立制动，此时不承诺正常规划的 C2 衔接。C2 指规划曲线，真实机械臂仍受离散采样、RT 调度、阻抗刚度和负载影响。

设备轨迹采用 `local.task`：已显式使能的本地任务，由 Core 主线程持续监督。它与 flight.command 分开授权，不伪造 FLIGHT/JOYSTICK 输入。普通 Demo/交互入口的 flight.state 中 control_source=NONE，task_phase 表达本地任务阶段，缺失手部和视觉反馈报告无效。

## 摇杆实时控制入口

`main_servo.cpp` 生成 `aviator_core_servo`。先 Enable → ApproachHandles → LockHandles，再订阅 `flight.command`，以 `robot.yaml` 的 `servo_period` 更新 ServoWheel。它与原 `aviator_core` 是二选一的控制入口，同一 Manipulator 不要同时启动两个 Core。

### 相机方向盘随动测试

`main_camera_servo.cpp` 生成独立入口 `aviator_core_camera_servo`，启动后自动执行
Enable → ApproachHandles → LockHandles，然后订阅 Bus 的 `camera.detection`。
真机仍由 Manipulator 使用关节阻抗执行，刚度读取 `robot.yaml` 的
`rokae.joint_stiffness`。此入口无需 flight_gateway；同一 Manipulator 只运行一个 Core。

```bash
cmake --build build/release --target aviator_core_camera_servo -j2
# 先启动 Bus、Manipulator、RH56FTP（或 fake 手）及相机节点。
./build/release/bin/aviator_core_camera_servo --config config/system.yaml --camera-id cockpit
```

只接受 `publisher_id=camera`、指定 `camera_id` 的观测，并要求消息与
`steering_wheel.valid` 有效、坐标为有限数。按 Monitor 的模型映射调用小写直接接口：
`servoWheel(clamp(-theta_rad, -0.87266, 0.87266),
clamp(-translation_along_axis_m - 0.085, -0.170, 0), 1.0)`。
角度单位 rad、位移单位 m；相机轴向位移 -0.085/0/+0.085 分别对应
0/-0.085/-0.170，目标为绝对位置，不叠加 `wheel_initial`。越界时限幅并打印提示。

更新周期沿用 `servo_period`，复用现有 IK、碰撞检查及 Ruckig Servo 规划。
`--camera-timeout-ms` 默认 200（允许 20..1000），检查本机 `clock_id` 和原始
`sample_mono_us`，同时拒绝未来时间、重复/倒退序号及倒退采样时间。
这比 Monitor 按接收时间显示更严格，积压帧不能持续刷新控制目标。
接近完成前的帧不参与随动；相机重启后可用新 session 和新鲜采样恢复。
若固定复用 `--session` 且序号从头开始，应同时重启此测试 Core，或为相机换一个 session。
不使用 `axis_match` 作为额外门槛，也不回退到原始 `pose`。

本相机的无效观测立即停止刷新目标，断流则在相机超时后停止刷新；Servo 随后按
`servo_timeout` 减速并保持锁定（不是收到无效帧就瞬时停住）。新鲜有效观测恢复后，
等待减速完成即可自动继续；运动故障则退出。Ctrl+C 停止 Servo、解锁并下使能。

先启动 Bus、Manipulator 和 flight_gateway，再直接启动：

```bash
./build/bin/flight_gateway
./build/bin/aviator_core_servo
```

真机的 `manipulator` 需要实时调度权限，可用 `sudo ./build/bin/manipulator --config config/system.yaml` 启动；Core 不需要 sudo。网关自定义总线时，修改 config/flight.yaml 的 publish / subscribe，与 system.yaml 保持一致。

默认配置路径与原 Core 相同，开发构建读取源码根目录 `config/system.yaml`，可用 `--config` 指定。输入按 flight_gateway 发布者、时钟、有效性、时效和序号检查；`--gateway-session` 已忽略，不再固定启动会话。

两个摇杆入口共用 `joystickWheelDisplacement()`，`main_servo.cpp` 的映射为：`angle = roll * 0.87266` rad，`displacement = 0.085 * (pitch - 1.0)` m，`v = 1.0`。pitch 的 -1、0、+1 分别对应 -0.170、-0.085、0 m；轮盘速度上限由 robot.yaml 的 wheel_*_speed 决定，目标仍经 Servo 规划执行；不再按应用层关节动态上限拦截。

仅接收同一时钟域、publisher_id=flight_gateway、source=JOYSTICK 的有限且处于 [-1,1] 的数据。沿用 InputGuard 检查序号与 valid。POSITION_HOLD 输入显式按 input_state.checked_mono_us 和消息接收时刻检查时效，同时保留原始轴事件时间并拒绝时间倒退；没有扩展的普通输入仍按原始采样时间检查。输入时效取 system.yaml 的 origin_timeout_ms（默认 100 ms）。失效后不再更新 Servo 目标，现有 servo_timeout（默认 250 ms）触发减速并保持软件锁定，因此停更到触发停止的上限约为两项超时之和，实际停止还需制动时间。有效数据恢复后可恢复跟随；网关重启后按新启动标记重新计数，不需要手动绑定会话。Ctrl+C 停止、调用 UnlockHandles（启用真实手时等待张开）、释放软件锁定并失能。

新入口有效跟随时 flight.state 的 control_source=JOYSTICK，等待或失效时为 NONE。Manipulator 仍验证 Core 的 local.task 心跳；网关输入时效在本入口检查，不宣称已实现端到端 flight.command origin 传播。位置保持输入在设备检查正常时允许静止持杆，详见 [flight_gateway 位置保持与设备检查](../flight_gateway/README.md#位置保持与设备检查)。网关与 Core 需一起更新并重启。

`control_nodes_flight_hold` 验证长时间无新轴事件的阶跃跟随、设备查询过期、网关静默和设备断开；`control_nodes_flight` / `control_nodes_flight_auto` 分别验证普通输入的旧参数兼容与发布者接收，通过 ZMQ 注入同格式输入，验证完整接近/锁定流程、正负映射、最大速度参数、无效消息不能抢先绑定、其他启动实例仍须满足来源和时效检查、旧采样重发不能维持运动、输入恢复和 Ctrl+C 退出；使用无头 MuJoCo，不连接真机或 USB 摇杆。

已废除的 TCP 对齐、grasp.ready、跟踪误差及锁定丢失判据没有迁移。软件锁定只表示操作阶段。限位、IK 失败、碰撞规划、来源/时效、指令连续性、设备故障和取消仍独立生效。Home 完成后保留关节速度停稳检查。MuJoCo 实测轮盘跟踪只作为测试断言，不能反向成为运行准入条件。

协议能力、窗口与时序见 [ZMQ 文档第 19 节](../../docs/AVIATOR_ZMQ协议格式说明.md)。配置入口为根目录 `config/system.yaml`，不是旧示例的 `build/bin/config/aviator.yaml`。安装后默认查找 `share/aviator/config/system.yaml`；开发构建默认使用源码根目录配置，不依赖工作目录。

## Core 控制真实机械手

`aviator_core`（交互、Demo）与 `aviator_core_servo` 共用 `RemoteLink` 的手部控制。
配置位于 [`config/system.yaml`](../../config/system.yaml) 的 `core_hand`，手节点仍读取
[`config/inspire_hand.yaml`](../../config/inspire_hand.yaml)。两者需连接同一个 Bus，运行在同一主机。
当前源码配置启用真实手控制，双手张开为 `[0.5,1,1,1,1,1]`，闭合为 `[0.5,0,0,0,0,0]`。
初始化、复位和松开共用 `core_hand.open`；其中侧摆 `0.5` 对应寄存器 `500`。
真机、仿真配置以及未显式配置 `open` 时的程序默认值均使用此张开姿态。
六路顺序为拇指旋转、拇指弯曲、食指、中指、无名指、小指。配置使用 0～1，手节点转换为 CAN 0～1000。

```yaml
core_hand:
  enabled: true
  publisher_id: inspire_hand
  completion_timeout_ms: 5000
  feedback_timeout_ms: 500
  open_tolerance: 0.03
  open:
    left: [0.5, 1, 1, 1, 1, 1]
    right: [0.5, 1, 1, 1, 1, 1]
  close:
    left: [0, 0, 0, 0, 0, 0]
    right: [0, 0, 0, 0, 0, 0]
```

`close` 缺失时跳过手部闭合并警告；超范围/非六元素等无效手部配置会禁用该手连接并警告。
未配置 `core_hand` 或 `enabled: false` 保留旧的软件锁定行为。正常情况下，真机和 simulation
均按 core_hand 配置发送手命令并等待反馈/ACK；手部失败或等待超时后继续 Core 流程，不代表手已到位或抓稳。
同一总线只启动一组设备节点。

从仓库根目录，在独立终端依次启动（CAN 接口应已配置并启用）：

```bash
# 终端 1，已有 Bus 可复用
./build/bin/aviator_bus --config config/system.yaml
# 终端 2，不加 --feedback-only
./build/bin/aviator_hand --config config/inspire_hand.yaml
# 终端 3，真机机械臂使用实时调度权限
sudo ./build/bin/manipulator --config config/system.yaml
# 终端 4，分步控制
./build/bin/aviator_core --config config/system.yaml
```

Core 终端逐条输入，等待每条阻塞动作打印 `[ok]`：

```text
enable
approach
lock
status
```

- `enable`：先尝试发送张开目标并等待双手实际位置到位；手部失败/超时仅警告，继续使能机械臂。
- `approach`：从 home 连续到达最终抓握位姿；各侧 tool 到目标进入 `grasp.json.hand_closing_distance`（默认 0.07 m）后，手指从 `core_hand.open` 平滑闭合到 `core_hand.close`，跟随机械臂执行游标，与臂轨迹共用终点。没有预接近停靠。
- `lock`：同步接近已提交闭合终点并收到本 Core 发布者、当前目标对应的 `accepted_command` 且实际反馈有效后，执行机械臂软件 lock，不再启动一段闭合动作。不会等待闭合位置全为 0，也不宣称已接触或抓稳方向盘。
- `unlock`：持续发送张开目标，等待两手各六路实际位置距目标不超过 `open_tolerance`，再执行软件 unlock；手部失败/超时警告后仍执行软件 unlock。
- `stop`：只停止机械臂运动，保持当前手目标；等运动停止后可输入 `unlock`、`disable`、`quit`。普通 `unlock` 不带机械臂撤离。
- `disable`：若软件仍锁定，或同步接近已闭合手指但尚未 lock，先张开，再失能。正常退出也保持监督心跳直到清理动作结束。

使用摇杆时，前三个节点保持运行，改为启动：

```bash
./build/bin/flight_gateway
./build/bin/aviator_core_servo --config config/system.yaml
```

Servo 入口会自动张开、使能、接近、闭合，然后接收摇杆输入。两个 Core 入口只能运行一个。
`aviator_core_sml` 仅测试状态转换，不执行上述开合，也不提供摇杆/RS422 接入。

Core 的独立手部通信线程 `HandLink` 以 50 Hz 发布 `hand.command`，并独立接收 `hand.state`。
兼容 RH56FTP 精简协议：实际位置使用 `drive_position_normalized`，不依赖每侧 `enabled`、
关节量占位或旧测量别名。启动时 `accepted_command=null` 且 `command_valid=false` 的状态
正常更新反馈与消息在线检查，不作为动作 ACK。`hold_control_error` 是设备锁存的历史诊断，
保留在接收报文中，但不参与反馈有效性、续发或完成判断；恢复后的有效位置与新鲜 ACK
可以确认张开完成，即使该诊断字段始终非空。
反馈有效性与命令续发分开判断：消息及命令 ACK 持续新鲜时，顶层或单手 `valid=false`
以及 `hold_control_error` 非空均不会中止 50 Hz 的目标发布，新的张开请求也可正常下发。
无效反馈不能确认动作完成；等待达到完成期限后仅警告并返回未完成，目标继续发布。
反馈恢复有效后重新开始完成观察窗口，张开仍需按实际位置确认到位。
它拥有单独的 PUB/SUB socket、目标/反馈锁和条件变量，不使用机械臂 IO 线程的锁。`lock/unlock`
只提交完整的双手目标快照；目标版本防止旧等待被新目标的应答完成，需要确认的操作在释放机械臂锁后等待。
`flight.state.hand_control.target_version` 可用于诊断当前目标版本。目标长期不变仍持续发送，
包括规划、等待及普通 Stop 阶段；这是普通 Linux 周期线程，不承诺硬实时调度。

origin 保留主线程的真实监督心跳。发送线程在取得手部锁后先读取心跳快照，再读取当前单调时间，
避免“先读取旧 now、再读取更新后的 heartbeat”造成误判。真正超时会打印 `age_us` 和 `limit_us`，
时钟顺序异常单独报告 `ahead_us`，发送线程不会自行刷新主线程心跳。
主线程心跳超过 100 ms、发布授权撤销或 Core 退出后，不再续发旧目标，手节点按自身
`timeout_ms`（默认 100 ms）回到 `safe_pose`（默认全张开）。因此通信保护/退出不会保证继续抓握；
软件锁定状态也不表示手仍闭合。这里沿用手节点既有超时策略，没有实现物理制动。

手部消息持续在线时，Core 将无效反馈导致的完成未确认、动作/ACK 超时、动作未到位、只读模式、发布者冲突、缺少闭合目标及手部 IO 异常作为
`Core hand warning` 输出，同一原因最多每 10 秒重复一次。等待返回未完成，继续后续流程；
不会将手部原因写入机械臂故障快照、触发 FSM ERROR 或停止机械臂轨迹/Servo 发布。
本策略适用于 managed FSM、交互和 Servo 等共用 RemoteLink 的入口。

启用 `core_hand` 后，Core 独立监测配置发布者的 `hand.state` 消息：从手部连接启动起，
或最近一条通过校验的消息接收起，连续超过 **1000 ms** 未收到新消息，报 ERROR 级别错误
并将状态机切换到 `ERROR`，撤销运动授权。该检查在空闲、动作失败和已撤销授权时也有效；
持续到达的无效手部反馈仍属于消息在线，原有设备错误、动作及 ACK 超时继续作为警告。
消息恢复后，需手动 `RESET_ERROR`；消息未恢复时不能复位。`core_hand.enabled: false` 不监测。
重复序号、其他发布者、错误时钟、过期或无法解析的消息不刷新计时。

机械臂、Core 总线、任务总期限、授权撤销、急停及控制输入保护仍有效，不按错误文本包含 hand 来屏蔽异常。
真实反馈过期、ACK 过期或缺失、有效反馈下目标未到位的超时、只读模式和发布者冲突仍会停止手部目标续发。
手部自身故障状态仍保留，失败的手命令停止续发，手节点继续执行自身超时和安全张开保护；
软件锁定或 FSM 继续运行不代表手部正常或抓握成功。通信线程退出后需重启 Core。
不要同时运行 `hand_command.py` 或旧 `inspire_hand_node`；手节点仍绑定首个有效发布者和 control_epoch，Core 重启导致 epoch 变化时仍需
重启手节点重新绑定。`flight.state.hands` 现在来自真实手节点，`freshness.hand` 报告实际反馈时效，
`hand_control.error` 报告控制错误；`grasp_verified` 仍由手节点保持 false。

离线验证（模拟设备和 SocketCAN 写入，不驱动硬件）：

```bash
cmake --build build --target aviator_core aviator_core_servo aviator_core_hand_test aviator_core_hand_link_driver aviator_hand_test -j2
ctest --test-dir build -R '^(core_hand|aviator_hand_controller|robot_state_machine|motion_protocol)$' --output-on-failure
# Python 需安装 pyzmq；亦可使用已有 apriltag_realsense 环境
python tests/core_hand_process_test.py "$PWD/build/tests/aviator_core_hand_link_driver" "$PWD/build/bin/aviator_bus"
```

手部线程回归还覆盖：人为占用机械臂 IO 锁 400 ms 时，手部发送和反馈保持连续；停止 Core 心跳后仍停止
续发旧目标并报告实际超时时间；目标替换、撤销授权和线程退出能结束相应的等待；机械臂 IO 异常会停止
手目标续发，构造失败能清理已启动的线程。测试使用随机本机端口
和模拟设备，不打开 CAN，也不连接实际机械臂。

## 轮盘初始位形与摇杆中位

`config/robot.yaml` 配置：

```yaml
wheel_initial:
  angle: 0.0           # rad，范围 [-0.87266, 0.87266]
  displacement: -0.085 # m，范围 [-0.170, 0]
```

该值表示启动时轮盘实际所在位形。Manipulator 初始化软件参考并通过 `arm.state.wheel_reference` 传给 Core；Rokae 不会因为这项配置自动移动轮盘。MuJoCo 同时设置实体轮盘关节初值。省略整个配置块时保持旧版 `(0, 0)` 初值；给出配置块时必须包含两个有限数值。

使能保持、回 home 和连续抓取接近沿用这一参考。抓取完成后 Servo 从该位形衔接；松开/重新抓取使用运行中的最新参考，不在每次使能时重置初值。`grasp.json` 的几何零位及抓取点不变，位移沿轮盘自身推拉轴定义。

`aviator_core_servo` 和 `aviator_core_managed` 的 pitch 均按 `0.085 * (pitch - 1)` 映射：-1 → -0.170 m，0 → -0.085 m，+1 → 0 m。该映射固定为绝对位移，不额外叠加 wheel_initial。配置其它初值时，摇杆中位仍对应 -0.085 m。原有 roll 方向各自保持不变。

直接调用 `moveWheel/servoWheel` 的位移参数仍是绝对位置。`main.cpp --demo/--servo-demo` 中写明的目标值保持原样（目标 0 仍会移到零位）；改变的是抓取起始位形。

修改初值后需重启 Manipulator 和 Core；使用手节点/Gateway 时按控制 epoch 授权流程一起重启。启动日志显示 `Manipulator initial wheel`，READY/STANDBY/抓取前可检查 `wheel_reference.displacement=-0.085`。启动前真机轮盘应处于所声明的位形，这不是轮盘位置测量。

### 统一 URDF 的碰撞检查

Core 的运动学与碰撞检查均读取 `robot.yaml` 的 `urdf`（默认 `models/urdf/aviator.urdf`），
不再读取 `collision_urdf`、旧碰撞 URDF 或 SRDF，也不再要求法兰抓握圆柱。
碰撞几何使用完整 URDF 的 `<collision>`，排除固定连接及直接相邻连杆的碰撞对；
旧模型对轮盘轴承、腕部壳体和抓握圆柱的额外排除不再沿用。
当前检查接口只传入双臂与轮盘位置，其余关节（包括手指）按 URDF 零位检查，
尚不能检查实际手指开合过程。`collision_check_enabled` 控制是否启用此检查。
