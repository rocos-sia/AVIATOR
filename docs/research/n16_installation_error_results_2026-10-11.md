# N16 安装误差鲁棒性结果（2026-10-11）

## 证据边界

本节点在真实 MuJoCo 模型中注入夹具/工件安装误差和双臂基座相对安装误差，比较冻结的 endpoint-aware online governor 与未重建的 stale safe-scaled LUT。两者使用同一初始状态、任务速度、dt、swept checker 和 d_safe。结果是测试误差包络内的经验鲁棒性证据；不构成全局安全保证、全局 IK branch preservation 或 RL 泛化证据。

## 原始数据

- `/home/rocos/sia/AVIATOR/outputs/differential_lift_feasibility_20261011/n16_installation_error_formal/n16_installation_error_raw.json`

## 总体结果

- online governor：475/480 = 99.0% candidate-step completion；task-progress median=0.9997；clearance P05=5.153 mm。
- stale LUT：315/480 = 65.6% candidate-step completion；task-progress median=1.0000；clearance P05=4.859 mm。
- online governor 的 `governor_exhausted` 次数为 5；stale LUT 的 `lut_endpoint_validation` 次数为 163。
- online P95 step latency 的 trial 中位数为 75.36 ms；stale LUT 为 11.65 ms。安全检查没有被省略，因此 online 的安全代价必须单独报告。

## 解释

online governor 在扰动模型上使用当前 FK/Jacobian 和独立 endpoint/swept checker，因此它体现的是在线适应与安全监管组合的表现；stale LUT 保留名义安装条件下的查询，体现模型失配下的查表退化。该设计可以支持“在所测安装误差范围内，在线监管保持更高的候选步接受率/任务进度，但计算延迟更高”的条件性结论。它不能单独证明在线方法在所有安装误差、所有任务或真实硬件上都更好。

## 逐 cell 统计

详见输出目录中的 `n16_cell_summary.csv`、`n16_method_summary.csv` 和 `n16_stop_reason_summary.csv`。

按误差家族汇总：

| family | method | trials | complete rate | progress median | progress P05 | clearance P05 (mm) | trial-median P95 latency (ms) |
|---|---|---:|---:|---:|---:|---:|---:|
| base_rotation | online_endpoint_governor | 120 | 100.0% | 0.9999 | 0.9970 | 5.159 | 76.28 |
| base_rotation | stale_lut_safe_scaled | 120 | 50.0% | 0.9450 | 0.1100 | 4.811 | 11.64 |
| base_translation | online_endpoint_governor | 120 | 95.8% | 0.9998 | 0.9965 | 5.140 | 74.31 |
| base_translation | stale_lut_safe_scaled | 120 | 75.8% | 1.0000 | 0.3570 | 4.957 | 11.59 |
| fixture_rotation | online_endpoint_governor | 120 | 100.0% | 0.9997 | 0.9774 | 5.165 | 77.60 |
| fixture_rotation | stale_lut_safe_scaled | 120 | 55.8% | 1.0000 | 0.1585 | 4.720 | 11.78 |
| fixture_translation | online_endpoint_governor | 120 | 100.0% | 0.9997 | 0.9976 | 5.153 | 74.79 |
| fixture_translation | stale_lut_safe_scaled | 120 | 80.8% | 1.0000 | 0.3385 | 4.961 | 11.62 |

## 后续

若需要 rebuilt LUT，应将其作为另一个信息条件运行：必须明确给出重新标定/重建成本，并保持同一 checker。RL 应留在独立节点，不纳入本节点主比较。
