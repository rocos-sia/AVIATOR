# AVIATOR MuJoCo 仿真

独立运行 MuJoCo，通过 POSIX 共享内存模拟 EtherCAT 主站，为 ROCOS
控制程序提供 CiA 402 驱动和 PDO 数据。默认加载 AVIATOR：固定飞机机体、
左右各七轴机械臂，以及旋转/推拉两轴被动操纵盘。

## 构建与运行

在本目录执行（依赖 C++17、CMake、yaml-cpp；可视化另需 GLFW/OpenGL）：

```bash
cmake -S . -B build/aviator-headless -DCMAKE_BUILD_TYPE=Release -DROCOS_MUJOCO_ENABLE_VISUALIZATION=OFF
cmake --build build/aviator-headless -j4
./build/aviator-headless/bin/rocos_mujoco_sim --duration 5
ctest --test-dir build/aviator-headless --output-on-failure
```

可视化构建：

```bash
cmake -S . -B build/aviator-visual -DCMAKE_BUILD_TYPE=Release -DROCOS_MUJOCO_ENABLE_VISUALIZATION=ON
cmake --build build/aviator-visual -j4
./build/aviator-visual/bin/rocos_mujoco_sim -v
```

不传 `--model` 时默认使用 `aviator`，即 `model/aviator.xml` 和
`config/hardware_aviator_config.yaml`；也可显式指定 `--model aviator`。
不传 `-v` 时仍为无头模式。保留 `sparc`、`sparc_urdf`、`charge_arm`、`talon` 预置；
`--urdf <MJCF/URDF路径>` 和 `--config <YAML路径>` 可覆盖预置，且不受
参数顺序影响。可执行文件支持从仓库根目录、构建目录或其他工作目录启动。

## 模型与配置

- `aviator` 来源于仓库根目录的 `urdf/aviator.urdf`，场景参考 `model/scene.xml`。
  保留 URDF 的关节名、轴向、安装位姿、限位、质量、完整惯量和网格缩放；
  机体固定，双臂启动时加载 `aviator_home` keyframe，采用肘部下沉姿态、两侧第 2 关节均为 +90°；
  初始角度来自 `AviatorRobot/config/posture.json`，操纵盘两轴仍从 0 开始。
  地面降至 z=-0.3 m，给操纵盘留出运动空间；
  不移动 URDF 模型坐标系。相机默认对准整个座舱和双臂。
- `config/hardware_aviator_config.yaml` 仅配置 14 个机械臂驱动：左臂
  `AR5-5_07L-W4C4A2_joint_1..7` 对应从站 `0..6`，右臂
  `AR5-5_07R-W4C4A2_joint_1..7` 对应从站 `7..13`。
  `lower/upper/vel/effort` 来自 URDF；`acc/jerk`、编码器及力矩换算为仿真默认值。
  MJCF 提供位置限位；当前仿真器不会根据 YAML 自动强制执行速度/力矩上限。
- 操纵盘 `roll_input_joint`（±0.87266 rad）与 `pitch_input_joint`
  （-0.165..0 m）没有执行器，也不占用从站。无质量的中间 `dummy_link`
  合并为操纵盘刚体上的两个同轴关节，顺序保持旋转后平移，保留原运动学和惯量。
  两轴添加仿真阻尼 0.1 N·m·s/rad 和 1 N·s/m，没有回中弹簧，可由外力或接触力推动。
  双臂沿用仿真器的关节力控制与被动阻尼，无需在 XML 中添加 actuator。
- `model/aviator_meshes/` 是独立运行所需的网格副本：机械臂二进制 STL 保持原样，
  飞机和操纵盘由 ASCII STL 转为二进制 STL，三角形和尺度保持不变。
  补充 URDF 引用但未定义的白色材质。原始 URDF 和 `meshes/` 不修改。
- 飞机和操纵盘的碰撞几何按 CAD 分离部件拆开，再逐部件使用 MuJoCo 凸包近似，
  避免整块凸包填满座舱内部空隙；仍未做精细凹面分解。排除五对 CAD 装配重叠：
  飞机与操纵盘、左右臂各自的 base/link1 和 link5/link7。
- 左右末端增加直径 56 mm、长度 60 mm 的圆柱抓握代理，各有 0.15 kg 质量和对应惯量。
  圆柱横向安装在 100 mm 工具偏移处，连接杆止于圆柱表面；尺寸和完整工具变换由
  `AviatorRobot/config/grasp.json` 配置。圆柱通过碰撞位掩码允许与各自握持外壳及按钮
  重叠（左 31/34/35、右 33/36/37），与其他部件、另一圆柱及地面的碰撞仍启用。
- `left_tcp/right_tcp` 对应圆柱中心，`left_handle/right_handle` 对应操纵盘握持段目标。
  两组 `left_grasp/right_grasp` weld 默认关闭。`aviator_grasp` 配置启用
  `/aviatorN` 抓取命令/反馈通道，由 `AviatorRobot` 控制器发起对齐后的锁定和解锁。
  操纵盘仍然是被动关节；锁定不直接改写关节位置，也不增加操纵盘执行器。
  详细运行流程见 [AviatorRobot README](../../AviatorRobot/README.md)。

修改源 URDF 或 `AviatorRobot/config/grasp.json` 后，可用仅依赖 Python 标准库的脚本
重新生成 XML、YAML、网格及 `AviatorRobot/config/aviator_control.urdf`，
然后重新构建以同步可执行文件旁的资源：

```bash
python3 scripts/generate_aviator.py
cmake --build build/aviator-headless -j4
```

`aviator_control` 测试验证 16 个自由度与 14 个驱动从站、操纵盘被动外力响应、
双臂使能及正反向不同目标的反馈映射，要求所有驱动的跟踪误差小于 0.01 rad。

## 可选 SPARC 模型

- `sparc` 直接加载仓库 `Model/urdf/scene.xml`，由场景包含 `SPARC.xml`，
  网格来自 `Model/meshes`。模型有一个浮动基座和七个连续旋转关节。
  场景原有的零重力及两个未激活的 weld 保持原定义；启动不执行爬行或锚点切换。
- `sparc_urdf` 直接加载 `Model/urdf/SPARC.urdf`。它是固定基座模型，
  不包含场景的地面、锚点或 weld，并使用 MuJoCo 默认重力。
  原始 URDF 的底座与第一轴碰撞网格约有 16 mm 重叠，在 MuJoCo 3.4
  下会产生接触阻力，影响第一轴跟踪；需要完整控制仿真时使用 `sparc` 场景。
  加载器为 URDF 自动设置相邻的 `../meshes` 目录，解析 CAD 导出的网格引用。
- `config/hardware_SPARC_config.yaml` 参考 Talon 的 PDO 布局，配置从站
  `0..6` 对应 `joint_1..joint_7`。第二轴通过 `joint_alias: joint2`
  兼容原始 URDF，浮动基座不占用驱动从站。

原模型未定义速度、力矩限值或编码器参数。YAML 中的 ±π 位置软限制及
速度、加速度、jerk、力矩、换算参数是仿真控制器默认值；零位偏置全部为 0。
仿真器不将 YAML 的软限制转换为 MuJoCo 物理限位。SPARC 无内置 actuator，
位置控制通过关节力和附加被动阻尼实现；关节位置、速度和执行器分别按各自
地址映射，避免浮动基座造成索引错位。

CMake 的可选 `SPARC_MODEL_DIR` 默认指向本目录的 `../../../Model`，可用
`-DSPARC_MODEL_DIR=/绝对路径/Model` 覆盖。构建时会把 `urdf/`、`meshes/`
复制到 `build/.../bin/model/SPARC/`；目录缺失时跳过 SPARC 复制和测试，
不影响 AVIATOR 构建。运行时优先使用原模型，原路径不存在时
回退到可执行文件旁的模型副本。模型副本及配置会在重新构建仿真器时同步。

测试通过共享内存依次使能七轴并下发不同位置目标，分别检查场景及 URDF 的
关节映射、状态字和位置反馈。URDF 第一轴受上述碰撞影响，测试检查运动响应；
场景全部七轴及 URDF 其余六轴检查目标跟踪误差小于 0.01 rad。
