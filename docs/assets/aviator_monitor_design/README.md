# AVIATOR Monitor 效果参考图

日期：2026-10-01。生成方式：内置 `image_gen` 工具（imagegen 技能），先生成 UI 概念图，再针对单位和状态归属进行定向修改；未使用 CLI/API fallback。

## 资产

| 文件 | 内容 |
| --- | --- |
| [01_overview.png](01_overview.png) | 直观监测正常观测：模型、发布灯、RGB、指令与实测。 |
| [02_messages.png](02_messages.png) | 系统消息：总线、REQ/REP、只读详情。 |
| [03_camera_stale.png](03_camera_stale.png) | 相机过期：冻结图像、旧姿态、有效指令继续显示。 |

完整方案见 [双 Tab 界面设计方案](../../AVIATOR_Monitor双Tab界面设计方案.md)。所有图片均为概念效果参考，并非实现截图、仓库模型渲染或现场视频。数值、JSON、发布者和日期是示意内容，不得从图片反推消息协议。

## 使用边界与审阅说明

- 配色、面板层级和信息分区供视觉参考；三个画面的栏宽与局部控件有生成差异，最终统一采用方案第 2、8 节。
- 正常页图中指令表盘显示了 ±110% 刻度；正式指令刻度应为 ±100%，仅指令/实测二维对照图扩展到 ±110%。实测条的标称 ±100% 两侧还需容纳反馈越界区。
- 异常页的“正常”指示灯文案正式实现应采用“发布新鲜”，且应保留设备反馈摘要与未标定提示；不能把发布新鲜解读为硬件健康。
- 图中的“已发送”只是概念文案。正式 REQ/REP 列严格展示当前网关观测枚举，TIMEOUT_UNKNOWN 属于网关记录，ACCEPTED/COMPLETED/REJECTED 属于 Core 应答。
- 最终运行界面不显示“设计示意 · 非实时数据”；开发演示/模拟模式则必须保留清晰的数据来源标记。

## 初始生成内容约束

三张图均采用 `ui-mockup`：16:9 桌面工程监测界面，深灰面板、中灰场景、蓝色选中、绿色新鲜、琥珀色过期；仅有“直观监测”和“系统消息”两个一级 Tab；只读，不出现发送、启动、停止或机器人控制按钮。图片内固定标明“设计示意 · 非实时数据”。

正常页强调双臂、灵巧手与驾驶盘三维场景，RGB 预览，消息发布指示灯，实际驱动反馈与未标定手姿态。指令示例 roll +35.0%、pitch −12.0%，物理等价值 +17.5°、74.8 mm；实测示例 +17.0°/+34.0%、73.1 mm/−14.0%。

消息页包含总线数据流和 REQ/REP 两张表及共享 JSON 详情，标明接收频率不等于源端发布率、受理不等于完成、超时结果未知。相机过期页保持指令继续显示，将冻结 RGB 和驾驶盘旧姿态明确置灰并标龄，失效实测使用破折号而非零。

## 最终修改提示词（完整）

以下分别以对应初始概念图为编辑输入，仅修正指定内容。

### 01_overview.png

```text
Edit this AVIATOR Monitor read-only Chinese UI mockup. Preserve all layout, neutral charcoal color palette, CAD concept, exact two tabs, and existing physical values. In the left 消息发布 card add a separate green publication row 'Inspire Hand' before Camera, keeping Logger 未观测 gray. In the flight XY chart use axis extents -110% and +110%, retaining command +35.0% / -12.0% and measured +34.0% / -14.0%. Replace cyan measurement accents with soft teal #67D6A0 so measured values and legend match. Keep 设计示意 · 非实时数据 label. Only these corrections; this is conceptual artwork, not actual hardware screenshot.
```

### 02_messages.png

```text
Edit only the diagnostic data semantics in this AVIATOR Monitor Chinese read-only UI concept. Preserve dark neutral palette, layout, two tabs and typography. In lower REQ/REP table the row grasp_wheel must place amber TIMEOUT_UNKNOWN under 网关记录, put an em dash under Core 应答 and error code, and keep >30 s observation interval. ACCEPTED, COMPLETED, REJECTED stay under Core 应答 on other rows. Gateway entries on other rows may read 已发送. The hand.state row publisher should be inspire_hand, Hz 10.0, sample age 40 ms, receive interval 100 ms; keep it fresh green. Remove fabricated temperature field from JSON. Ensure joint_position arrays each contain 7 values, add seventh value 0.0 if needed. Change fictional dates to 2026-10-01. Keep label 设计示意 · 非实时数据 and read-only status, no send buttons. No new panels.
```

### 03_camera_stale.png

```text
Edit only semantic errors in this AVIATOR Monitor camera-stale UI concept. Preserve composition, dark gray theme, CAD robots, frozen camera, exact two tabs, and all command values. All labels PITCH 俯仰角 must become PITCH 俯仰位移; under 驾驶盘姿态 (视觉测量), pitch unavailable unit must be mm rather than degrees, displayed — mm. Roll remains — °. Publication lamp fresh row ages: Flight Gateway 20 ms, Core 10 ms, Manipulator 8 ms. Add green row Inspire Hand 80 ms above Camera; Camera stays amber 消息超时 1.2 s; Logger gray 未观测. Blue command markers remain live while actual visual measurement stays unavailable; never show zero measured values. Keep 设计示意 · 非实时数据. Bottom message state only 相机消息超时，原因待排查, avoid claiming a cause without evidence.
```
