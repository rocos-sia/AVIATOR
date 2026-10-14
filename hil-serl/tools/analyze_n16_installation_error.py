#!/usr/bin/env python3
"""Aggregate immutable N16 raw JSON into auditable tables and a report."""
from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np


def stat(values):
    a = np.asarray(values, dtype=float)
    return {"n": int(a.size), "median": float(np.median(a)),
            "p05": float(np.quantile(a, .05)), "p95": float(np.quantile(a, .95)),
            "mean": float(np.mean(a))}


def load_rows(raw_paths):
    rows = []
    for path in raw_paths:
        data = json.loads(Path(path).read_text())
        rows.extend(data.get("trial_rows", []))
    if not rows:
        raise RuntimeError("no trial_rows found")
    return rows


def aggregate(rows):
    groups = defaultdict(list)
    for row in rows:
        groups[(row["family"], float(row["error_level"]), row["error_unit"], row["speed"], row["method"])].append(row)
    out = []
    for (family, level, unit, speed, method), rs in sorted(groups.items()):
        out.append({
            "family": family, "error_level": level, "error_unit": unit,
            "speed": speed, "method": method, "trials": len(rs),
            "complete": int(sum(bool(r["complete"]) for r in rs)),
            "complete_rate": float(np.mean([bool(r["complete"]) for r in rs])),
            "progress_median": float(np.median([r["task_progress_fraction"] for r in rs])),
            "progress_p05": float(np.quantile([r["task_progress_fraction"] for r in rs], .05)),
            "clearance_median_m": float(np.median([r["min_clearance_m"] for r in rs])),
            "swept_clearance_median_m": float(np.median([r["min_swept_clearance_m"] for r in rs])),
            "closure_p95": float(np.quantile([r["max_closure_error_mixed_m_rad"] for r in rs], .95)),
            "latency_p95_median_ms": float(np.median([r["p95_step_latency_ms"] for r in rs])),
            "latency_p99_median_ms": float(np.median([r["p99_step_latency_ms"] for r in rs])),
            "governor_exhausted": int(sum(r["stop_reason"] == "governor_exhausted" for r in rs)),
            "lut_endpoint_validation": int(sum(r["stop_reason"] == "lut_endpoint_validation" for r in rs)),
            "other_stops": int(sum(r["stop_reason"] not in {"complete", "governor_exhausted", "lut_endpoint_validation"} for r in rs)),
        })
    return out


def method_summary(rows):
    groups = defaultdict(list)
    for row in rows:
        groups[(row["family"], row["speed"], row["method"])].append(row)
    out = []
    for (family, speed, method), rs in sorted(groups.items()):
        out.append({
            "family": family, "speed": speed, "method": method, "trials": len(rs),
            "complete_rate": float(np.mean([bool(r["complete"]) for r in rs])),
            "progress_median": float(np.median([r["task_progress_fraction"] for r in rs])),
            "progress_p05": float(np.quantile([r["task_progress_fraction"] for r in rs], .05)),
            "clearance_p05_m": float(np.quantile([r["min_clearance_m"] for r in rs], .05)),
            "closure_p95": float(np.quantile([r["max_closure_error_mixed_m_rad"] for r in rs], .95)),
            "latency_p95_median_ms": float(np.median([r["p95_step_latency_ms"] for r in rs])),
            "governor_exhausted": int(sum(r["stop_reason"] == "governor_exhausted" for r in rs)),
            "lut_endpoint_validation": int(sum(r["stop_reason"] == "lut_endpoint_validation" for r in rs)),
        })
    return out


def family_summary(rows):
    groups = defaultdict(list)
    for row in rows:
        groups[(row["family"], row["method"])].append(row)
    out = []
    for (family, method), rs in sorted(groups.items()):
        out.append({
            "family": family, "method": method, "trials": len(rs),
            "complete_rate": float(np.mean([bool(r["complete"]) for r in rs])),
            "progress_median": float(np.median([r["task_progress_fraction"] for r in rs])),
            "progress_p05": float(np.quantile([r["task_progress_fraction"] for r in rs], .05)),
            "clearance_p05_m": float(np.quantile([r["min_clearance_m"] for r in rs], .05)),
            "latency_p95_median_ms": float(np.median([r["p95_step_latency_ms"] for r in rs])),
            "governor_exhausted": int(sum(r["stop_reason"] == "governor_exhausted" for r in rs)),
            "lut_endpoint_validation": int(sum(r["stop_reason"] == "lut_endpoint_validation" for r in rs)),
        })
    return out


def write_csv(path, rows):
    if not rows:
        return
    with Path(path).open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--raw", nargs="+", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--report", required=True)
    p.add_argument("--pilot-raw", default=None)
    args = p.parse_args()
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=True)
    rows = load_rows(args.raw)
    cells = aggregate(rows)
    methods = method_summary(rows)
    families = family_summary(rows)
    write_csv(out / "n16_cell_summary.csv", cells)
    write_csv(out / "n16_method_summary.csv", methods)
    write_csv(out / "n16_family_summary.csv", families)
    stop = []
    for (family, method, reason), rs in sorted({
        (r["family"], r["method"], r["stop_reason"]): [] for r in rows
    }.items()):
        matching = [r for r in rows if r["family"] == family and r["method"] == method and r["stop_reason"] == reason]
        stop.append({"family": family, "method": method, "stop_reason": reason,
                     "count": len(matching), "fraction_within_method_family": float(len(matching) / max(1, sum(r["family"] == family and r["method"] == method for r in rows))),
                     "progress_median": float(np.median([r["task_progress_fraction"] for r in matching])) if matching else None})
    write_csv(out / "n16_stop_reason_summary.csv", stop)

    online = [r for r in rows if r["method"] == "online_endpoint_governor"]
    lut = [r for r in rows if r["method"] == "stale_lut_safe_scaled"]
    report = Path(args.report)
    report.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# N16 安装误差鲁棒性结果（2026-10-11）", "",
        "## 证据边界", "",
        "本节点在真实 MuJoCo 模型中注入夹具/工件安装误差和双臂基座相对安装误差，比较冻结的 endpoint-aware online governor 与未重建的 stale safe-scaled LUT。两者使用同一初始状态、任务速度、dt、swept checker 和 d_safe。结果是测试误差包络内的经验鲁棒性证据；不构成全局安全保证、全局 IK branch preservation 或 RL 泛化证据。", "",
        "## 原始数据", "",
        *[f"- `{Path(x).resolve()}`" for x in args.raw], "",
        "## 总体结果", "",
        f"- online governor：{sum(bool(r['complete']) for r in online)}/{len(online)} = {np.mean([bool(r['complete']) for r in online]):.1%} candidate-step completion；task-progress median={np.median([r['task_progress_fraction'] for r in online]):.4f}；clearance P05={np.quantile([r['min_clearance_m'] for r in online], .05)*1e3:.3f} mm。",
        f"- stale LUT：{sum(bool(r['complete']) for r in lut)}/{len(lut)} = {np.mean([bool(r['complete']) for r in lut]):.1%} candidate-step completion；task-progress median={np.median([r['task_progress_fraction'] for r in lut]):.4f}；clearance P05={np.quantile([r['min_clearance_m'] for r in lut], .05)*1e3:.3f} mm。",
        f"- online governor 的 `governor_exhausted` 次数为 {sum(r['stop_reason']=='governor_exhausted' for r in online)}；stale LUT 的 `lut_endpoint_validation` 次数为 {sum(r['stop_reason']=='lut_endpoint_validation' for r in lut)}。",
        f"- online P95 step latency 的 trial 中位数为 {np.median([r['p95_step_latency_ms'] for r in online]):.2f} ms；stale LUT 为 {np.median([r['p95_step_latency_ms'] for r in lut]):.2f} ms。安全检查没有被省略，因此 online 的安全代价必须单独报告。", "",
        "## 解释", "",
        "online governor 在扰动模型上使用当前 FK/Jacobian 和独立 endpoint/swept checker，因此它体现的是在线适应与安全监管组合的表现；stale LUT 保留名义安装条件下的查询，体现模型失配下的查表退化。该设计可以支持“在所测安装误差范围内，在线监管保持更高的候选步接受率/任务进度，但计算延迟更高”的条件性结论。它不能单独证明在线方法在所有安装误差、所有任务或真实硬件上都更好。", "",
        "## 逐 cell 统计", "",
        "详见输出目录中的 `n16_cell_summary.csv`、`n16_method_summary.csv` 和 `n16_stop_reason_summary.csv`。", "",
        "按误差家族汇总：", "",
        "| family | method | trials | complete rate | progress median | progress P05 | clearance P05 (mm) | trial-median P95 latency (ms) |", "|---|---|---:|---:|---:|---:|---:|---:|",
        *[f"| {r['family']} | {r['method']} | {r['trials']} | {r['complete_rate']:.1%} | {r['progress_median']:.4f} | {r['progress_p05']:.4f} | {r['clearance_p05_m']*1e3:.3f} | {r['latency_p95_median_ms']:.2f} |" for r in families], "",
        "## 后续", "",
        "若需要 rebuilt LUT，应将其作为另一个信息条件运行：必须明确给出重新标定/重建成本，并保持同一 checker。RL 应留在独立节点，不纳入本节点主比较。",
    ]
    report.write_text("\n".join(lines) + "\n")
    print(json.dumps({"rows": len(rows), "online_complete_rate": float(np.mean([bool(r['complete']) for r in online])),
                      "lut_complete_rate": float(np.mean([bool(r['complete']) for r in lut]))}, indent=2))


if __name__ == "__main__":
    main()
