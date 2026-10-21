#!/usr/bin/env python3
"""Fit each measured frequency using original acquisition times, including irregular sampling."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


PHASE_THRESHOLDS = (-30, -45, -90)


def fit_sine(t, values, frequency):
    phase = 2 * np.pi * frequency * np.asarray(t)
    design = np.column_stack((np.sin(phase), np.cos(phase), np.ones_like(phase)))
    coefficients, _, rank, _ = np.linalg.lstsq(design, values, rcond=None)
    if rank != 3:
        raise ValueError("Insufficient phase coverage")
    amplitude = float(np.hypot(*coefficients[:2]))
    residual = float(np.sqrt(np.mean((values - design @ coefficients) ** 2)))
    return amplitude, float(np.arctan2(coefficients[1], coefficients[0])), residual


def axis_fields(axis):
    if axis == "rotation":
        return ("command_rad", "reference_rad", "estimated_rad", "arm_difference_rad",
                180 / np.pi, "deg", "Wheel angle")
    if axis == "translation":
        return ("command_displacement_m", "reference_displacement_m", "estimated_displacement_m",
                "arm_difference_m", 1000, "mm", "Wheel displacement")
    raise ValueError(f"Unknown sweep axis: {axis}")


def frequency_result(rows, frequency, cycles, axis="rotation"):
    command, _, estimate, _, scale, unit, _ = axis_fields(axis)
    t = np.array([float(r["segment_t_s"]) for r in rows])
    target = np.array([float(r[command]) for r in rows])
    actual = np.array([float(r[estimate]) for r in rows])
    warnings = []
    result = {"frequency_hz": frequency, "samples": len(rows), "valid": False}
    if len(t) < 10 or not np.isfinite(np.r_[t, target, actual]).all() or np.any(np.diff(t) <= 0):
        return dict(result, warnings=["insufficient_or_invalid_samples"])
    span = t[-1] - t[0]
    rate = (len(t) - 1) / span
    if span * frequency < cycles - .15:
        warnings.append("incomplete_measurement_cycles")
    if rate / frequency < 20:
        warnings.append("fewer_than_20_samples_per_cycle")
    if np.max(np.diff(t)) * frequency > .15:
        warnings.append("measurement_gap")
    a_in, p_in, residual_in = fit_sine(t, target, frequency)
    a_out, p_out, residual_out = fit_sine(t, actual, frequency)
    if a_in < 1e-6:
        return dict(result, warnings=warnings + ["missing_excitation"])
    if residual_in / a_in > .25 or residual_out / max(a_out, 1e-6) > .25:
        warnings.append("large_non_sinusoidal_residual")
    arm_difference = max(abs(float(r["arm_difference_rad"])) for r in rows) * 180 / np.pi
    displacement_difference = (max(abs(float(r["arm_difference_m"])) for r in rows) * 1000
                               if axis == "translation" else 0)
    transverse = max(float(r[k]) for r in rows for k in ("left_transverse_m", "right_transverse_m"))
    tilt = max(float(r[k]) for r in rows for k in ("left_tilt_rad", "right_tilt_rad")) * 180 / np.pi
    if arm_difference > 3 or displacement_difference > 20 or transverse > .02 or tilt > 5:
        warnings.append("inconsistent_grasp_geometry")
    phase = (p_out - p_in + np.pi) % (2 * np.pi) - np.pi
    result.update(valid=not warnings, warnings=warnings, sample_rate_hz=rate,
                gain=a_out/a_in, gain_db=float(20*np.log10(max(a_out/a_in, 1e-15))),
                phase_deg=float(phase * 180 / np.pi),
                max_arm_difference_deg=arm_difference, max_transverse_m=transverse, max_tilt_deg=tilt)
    result.update({f"input_amplitude_{unit}": a_in * scale, f"output_amplitude_{unit}": a_out * scale,
                   f"residual_rms_{unit}": residual_out * scale})
    if axis == "translation":
        result["max_arm_difference_mm"] = displacement_difference
    return result


def usable_for_bandwidth(result, ignore_geometry=False):
    return result["valid"] or (ignore_geometry and
                               result.get("warnings") == ["inconsistent_grasp_geometry"])


def measurement_rejection(results, completed, ignore_geometry=False):
    if not completed:
        return "run_incomplete"
    if len(results) < 2:
        return "need_multiple_frequencies"
    if not all(usable_for_bandwidth(r, ignore_geometry) for r in results):
        return "invalid_frequency_measurements"
    return None


def downward_crossing(results, values, threshold):
    """Interpolate the first downward crossing on the logarithmic frequency axis."""
    for low, high, y0, y1 in zip(results, results[1:], values, values[1:]):
        if y0 > threshold >= y1:
            fraction = (threshold - y0) / (y1 - y0)
            return float(np.exp(np.log(low["frequency_hz"]) + fraction *
                         np.log(high["frequency_hz"] / low["frequency_hz"])))
    return None


def bandwidth(results, completed, ignore_geometry=False):
    rejection = measurement_rejection(results, completed, ignore_geometry)
    if rejection:
        return None, rejection
    baseline = results[0]["gain_db"]
    value = downward_crossing(results, [r["gain_db"] - baseline for r in results], -3)
    return value, "crossing_estimated" if value is not None else "minus_3db_crossing_not_observed"


def unwrapped_phases(results, ignore_geometry=False):
    """Unwrap contiguous usable measurements; never bridge an invalid frequency."""
    phases = [None] * len(results)
    start = 0
    for end in range(len(results) + 1):
        if (end < len(results) and usable_for_bandwidth(results[end], ignore_geometry)
                and np.isfinite(results[end].get("phase_deg", np.nan))):
            continue
        if start < end:
            phases[start:end] = np.rad2deg(np.unwrap(np.deg2rad(
                [r["phase_deg"] for r in results[start:end]]))).tolist()
        start = end + 1
    return phases


def phase_crossings(results, completed, ignore_geometry=False):
    rejection = measurement_rejection(results, completed, ignore_geometry)
    phases = unwrapped_phases(results, ignore_geometry)
    if not rejection and any(p is None for p in phases):
        rejection = "invalid_frequency_measurements"
    crossings = {}
    for threshold in PHASE_THRESHOLDS:
        value = None
        if rejection:
            status = rejection
        elif phases[0] <= threshold:
            # No earlier sample brackets the first crossing; do not extrapolate or
            # report a later recrossing as the first one.
            status = "phase_threshold_already_reached_at_first_frequency"
        else:
            value = downward_crossing(results, phases, threshold)
            status = "crossing_estimated" if value is not None else "phase_crossing_not_observed"
        crossings[f"minus_{abs(threshold)}_deg"] = {
            "threshold_deg": threshold, "frequency_hz": value, "status": status}
    return crossings


def status_text(status, english=False):
    return {
        "crossing_estimated": ("已估计交叉频率", "Crossing estimated"),
        "minus_3db_crossing_not_observed": ("未观测到交叉", "Crossing not observed"),
        "phase_crossing_not_observed": ("未观测到交叉", "Crossing not observed"),
        "phase_threshold_already_reached_at_first_frequency":
            ("首频点已达到相位阈值，无法定位首次交叉", "Threshold reached at first frequency"),
        "run_incomplete": ("测试未完成", "Run incomplete"),
        "need_multiple_frequencies": ("至少需要两个频点", "Need multiple frequencies"),
        "invalid_frequency_measurements": ("存在无效频点", "Invalid frequency measurements"),
    }[status][int(english)]


def analyze(directory, plots=True, ignore_geometry=False):
    directory = Path(directory)
    suffix = "_geometry_ignored" if ignore_geometry else ""
    with (directory / "samples.csv").open() as file:
        rows = list(csv.DictReader(file))
    metadata = json.loads((directory / "run.json").read_text()) if (directory / "run.json").exists() else {}
    launch = directory / "launch.json"
    options = json.loads(launch.read_text())["options"] if launch.exists() else {}
    axis = metadata.get("axis", options.get("axis", "rotation"))
    command, reference, estimate, difference, scale, unit, label = axis_fields(axis)
    # Planned frequencies remain present even if a run ends before producing samples.
    frequencies = metadata.get("frequencies_hz")
    if frequencies is None:
        if launch.exists():
            frequencies = [float(f) for f in options["frequencies"].split(",")]
            metadata["measure_cycles"] = options["measure_cycles"]
        else:
            frequencies = sorted({float(r["frequency_hz"]) for r in rows if float(r["frequency_hz"]) > 0})
    results = [frequency_result([r for r in rows if r["measure"] == "1" and
                                abs(float(r["frequency_hz"])-f) < 1e-10], f,
                               metadata.get("measure_cycles", 5), axis) for f in frequencies]
    for result in results:
        result["usable_for_bandwidth"] = usable_for_bandwidth(result, ignore_geometry)
    for result, phase in zip(results, unwrapped_phases(results, ignore_geometry)):
        result["phase_unwrapped_deg"] = phase
    completed = metadata.get("completed", False)
    value, status = bandwidth(results, completed, ignore_geometry)
    phases = phase_crossings(results, completed, ignore_geometry)
    metrics = [{"metric": "minus_3db", "threshold": -3, "threshold_unit": "dB",
                "frequency_hz": value, "status": status}]
    metrics.extend({"metric": f"phase_{key}", "threshold": crossing["threshold_deg"],
                    "threshold_unit": "deg", "frequency_hz": crossing["frequency_hz"],
                    "status": crossing["status"]} for key, crossing in phases.items())
    summary = {"measurement": "joint_fk_estimate", "axis": axis,
               "completed": completed,
               "bandwidth_hz": value, "bandwidth_status": status,
               "phase_crossings": phases,
               "has_estimated_crossing": any(m["frequency_hz"] is not None for m in metrics),
               "ignore_geometry": ignore_geometry,
               "baseline": "first tested frequency; -3 dB relative to its gain, not independently verified DC gain",
               "phase_convention": "output minus input; negative means lag",
               "phase_unwrap": "contiguous usable frequencies; assumes low-frequency phase near zero "
                               "and phase change below 180 degrees between adjacent frequencies",
               "crossing_method": "first downward crossing, interpolated in log frequency; no extrapolation",
               "includes": "existing Servo/Ruckig/IK and mechanical arm tracking",
               "frequencies": results}
    (directory / f"summary{suffix}.json").write_text(json.dumps(summary, indent=2, allow_nan=False))
    with (directory / f"crossings{suffix}.csv").open("w") as file:
        writer = csv.DictWriter(file, fieldnames=list(metrics[0]))
        writer.writeheader()
        writer.writerows(metrics)
    if results:
        keys = sorted(set().union(*(r.keys() for r in results)))
        with (directory / f"frequency_response{suffix}.csv").open("w") as file:
            writer = csv.DictWriter(file, fieldnames=keys)
            writer.writeheader()
            writer.writerows(results)
    if plots:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 1, figsize=(11, 7), constrained_layout=True)
        if rows:
            t = [float(r["t_s"]) for r in rows]
            for key, legend in ((command, "Sine command"), (reference, "Device reference (diagnostic)"),
                                (estimate, "Joint FK estimate")):
                axes[0].plot(t, [float(r[key])*scale for r in rows], label=legend, linewidth=1)
            axes[1].plot(t, [float(r[difference])*scale for r in rows], linewidth=1)
            axes[0].legend()
        axes[0].set(ylabel=f"{label} ({unit})", title="Wheel bandwidth test — joint feedback estimate")
        axes[1].set(xlabel="Time (s)", ylabel=f"Left - right estimate ({unit})")
        for ax in axes:
            ax.grid(True, alpha=.3)
        fig.savefig(directory / f"tracking{suffix}.png", dpi=160)
        plt.close(fig)
        fig, axes = plt.subplots(2, 1, figsize=(9, 7), constrained_layout=True)
        fitted = [r for r in results if "gain_db" in r]
        if fitted:
            f = [r["frequency_hz"] for r in fitted]
            axes[0].semilogx(f, [r["gain_db"] for r in fitted], "o-")
            axes[0].axhline(fitted[0]["gain_db"]-3, color="gray", linestyle="--", label="First frequency gain - 3 dB")
            if value is not None:
                axes[0].axvline(value, color="tab:orange", linestyle=":",
                               label=f"-3 dB: {value:.3f} Hz")
            axes[0].legend()
            axes[1].semilogx([r["frequency_hz"] for r in results],
                            [r["phase_unwrapped_deg"] if r["phase_unwrapped_deg"] is not None
                             else np.nan for r in results], "o-")
            for r in fitted:
                if not r["valid"]:
                    label = "geometry ignored" if r["usable_for_bandwidth"] else "invalid"
                    axes[0].annotate(label, (r["frequency_hz"], r["gain_db"]), color="red")
                if r["phase_unwrapped_deg"] is None:
                    axes[1].plot(r["frequency_hz"], r["phase_deg"], "rx")
        for crossing, color in zip(phases.values(), ("tab:green", "tab:orange", "tab:red")):
            phase_value = crossing["frequency_hz"]
            result_label = (f"{phase_value:.3f} Hz" if phase_value is not None
                            else status_text(crossing["status"], english=True))
            axes[1].axhline(crossing["threshold_deg"], color=color, linestyle="--",
                           label=f"{crossing['threshold_deg']} deg: {result_label}")
            if phase_value is not None:
                axes[1].plot(phase_value, crossing["threshold_deg"], "s", color=color)
        axes[1].legend(fontsize=8)
        note = " (geometry checks ignored)" if ignore_geometry else ""
        bandwidth_label = f"{value:.3f} Hz" if value is not None else status_text(status, english=True)
        axes[0].set(ylabel="Gain (dB)", title=f"Joint feedback frequency response{note}\n-3 dB: {bandwidth_label}")
        axes[1].set(xlabel="Frequency (Hz)", ylabel="Phase (deg; negative = lag)")
        for ax in axes:
            ax.grid(True, which="both", alpha=.3)
        fig.savefig(directory / f"frequency_response{suffix}.png", dpi=160)
        plt.close(fig)
    note = "（忽略几何超限，仅作关节反馈链路估计）" if ignore_geometry else ""
    print(f"频响分析报告{note}：", flush=True)
    for metric in metrics:
        label = "−3 dB 带宽" if metric["metric"] == "minus_3db" else f"{metric['threshold']}°相位交叉频率"
        value_text = (f"，约 {metric['frequency_hz']:.3f} Hz"
                      if metric["frequency_hz"] is not None else "")
        print(f"  {label}：{status_text(metric['status'])}{value_text}", flush=True)
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--ignore-geometry", action="store_true",
                        help="仅忽略几何超限对带宽计算的否决；保留告警，另存带 _geometry_ignored 后缀的结果")
    args = parser.parse_args()
    analyze(args.directory, ignore_geometry=args.ignore_geometry)
