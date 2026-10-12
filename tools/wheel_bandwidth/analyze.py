#!/usr/bin/env python3
"""Fit each measured frequency using original acquisition times, including irregular sampling."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def fit_sine(t, values, frequency):
    phase = 2 * np.pi * frequency * np.asarray(t)
    design = np.column_stack((np.sin(phase), np.cos(phase), np.ones_like(phase)))
    coefficients, _, rank, _ = np.linalg.lstsq(design, values, rcond=None)
    if rank != 3:
        raise ValueError("Insufficient phase coverage")
    amplitude = float(np.hypot(*coefficients[:2]))
    residual = float(np.sqrt(np.mean((values - design @ coefficients) ** 2)))
    return amplitude, float(np.arctan2(coefficients[1], coefficients[0])), residual


def frequency_result(rows, frequency, cycles):
    t = np.array([float(r["segment_t_s"]) for r in rows])
    target = np.array([float(r["command_rad"]) for r in rows])
    actual = np.array([float(r["estimated_rad"]) for r in rows])
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
    transverse = max(float(r[k]) for r in rows for k in ("left_transverse_m", "right_transverse_m"))
    tilt = max(float(r[k]) for r in rows for k in ("left_tilt_rad", "right_tilt_rad")) * 180 / np.pi
    if arm_difference > 3 or transverse > .02 or tilt > 5:
        warnings.append("inconsistent_grasp_geometry")
    phase = (p_out - p_in + np.pi) % (2 * np.pi) - np.pi
    return dict(result, valid=not warnings, warnings=warnings, sample_rate_hz=rate,
                input_amplitude_deg=a_in * 180 / np.pi, output_amplitude_deg=a_out * 180 / np.pi,
                gain=a_out/a_in, gain_db=float(20*np.log10(max(a_out/a_in, 1e-15))),
                phase_deg=float(phase * 180 / np.pi), residual_rms_deg=residual_out * 180 / np.pi,
                max_arm_difference_deg=arm_difference, max_transverse_m=transverse, max_tilt_deg=tilt)


def bandwidth(results, completed):
    if not completed:
        return None, "run_incomplete"
    if len(results) < 2:
        return None, "need_multiple_frequencies"
    if not all(r["valid"] for r in results):
        return None, "invalid_frequency_measurements"
    baseline = results[0]["gain_db"]
    for low, high in zip(results, results[1:]):
        y0, y1 = low["gain_db"] - baseline, high["gain_db"] - baseline
        if y0 > -3 >= y1:
            fraction = (-3 - y0)/(y1-y0)
            return float(np.exp(np.log(low["frequency_hz"]) + fraction *
                         np.log(high["frequency_hz"]/low["frequency_hz"]))), "crossing_estimated"
    return None, "minus_3db_crossing_not_observed"


def analyze(directory, plots=True):
    directory = Path(directory)
    with (directory / "samples.csv").open() as file:
        rows = list(csv.DictReader(file))
    metadata = json.loads((directory / "run.json").read_text()) if (directory / "run.json").exists() else {}
    # Planned frequencies remain present even if a run ends before producing samples.
    frequencies = metadata.get("frequencies_hz")
    if frequencies is None:
        launch = directory / "launch.json"
        if launch.exists():
            options = json.loads(launch.read_text())["options"]
            frequencies = [float(f) for f in options["frequencies"].split(",")]
            metadata["measure_cycles"] = options["measure_cycles"]
        else:
            frequencies = sorted({float(r["frequency_hz"]) for r in rows if float(r["frequency_hz"]) > 0})
    results = [frequency_result([r for r in rows if r["measure"] == "1" and
                                abs(float(r["frequency_hz"])-f) < 1e-10], f,
                               metadata.get("measure_cycles", 5)) for f in frequencies]
    value, status = bandwidth(results, metadata.get("completed", False))
    summary = {"measurement": "joint_fk_estimate", "completed": metadata.get("completed", False),
               "bandwidth_hz": value, "bandwidth_status": status,
               "baseline": "first tested frequency; -3 dB relative to its gain, not independently verified DC gain",
               "phase_convention": "output minus input; negative means lag",
               "includes": "existing Servo/Ruckig/IK and mechanical arm tracking",
               "frequencies": results}
    (directory / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False))
    if results:
        keys = sorted(set().union(*(r.keys() for r in results)))
        with (directory / "frequency_response.csv").open("w") as file:
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
            for key, label in (("command_rad", "Sine command"), ("reference_rad", "Device reference (diagnostic)"),
                               ("estimated_rad", "Joint FK estimate")):
                axes[0].plot(t, [float(r[key])*180/np.pi for r in rows], label=label, linewidth=1)
            axes[1].plot(t, [float(r["arm_difference_rad"])*180/np.pi for r in rows], linewidth=1)
            axes[0].legend()
        axes[0].set(ylabel="Wheel angle (deg)", title="Wheel bandwidth test — joint feedback estimate")
        axes[1].set(xlabel="Time (s)", ylabel="Left - right estimate (deg)")
        for ax in axes:
            ax.grid(True, alpha=.3)
        fig.savefig(directory / "tracking.png", dpi=160)
        plt.close(fig)
        fig, axes = plt.subplots(2, 1, figsize=(9, 7), constrained_layout=True)
        fitted = [r for r in results if "gain_db" in r]
        if fitted:
            f = [r["frequency_hz"] for r in fitted]
            axes[0].semilogx(f, [r["gain_db"] for r in fitted], "o-")
            axes[0].axhline(fitted[0]["gain_db"]-3, color="gray", linestyle="--", label="First frequency gain - 3 dB")
            axes[0].legend()
            axes[1].semilogx(f, np.unwrap([r["phase_deg"]*np.pi/180 for r in fitted])*180/np.pi, "o-")
            for r in fitted:
                if not r["valid"]:
                    axes[0].annotate("invalid", (r["frequency_hz"], r["gain_db"]), color="red")
        axes[0].set(ylabel="Gain (dB)", title=f"Bandwidth: {value if value is not None else status}")
        axes[1].set(xlabel="Frequency (Hz)", ylabel="Phase (deg; negative = lag)")
        for ax in axes:
            ax.grid(True, which="both", alpha=.3)
        fig.savefig(directory / "frequency_response.png", dpi=160)
        plt.close(fig)
    print(f"带宽分析：{status}" + (f"，约 {value:.3f} Hz" if value is not None else ""), flush=True)
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    analyze(parser.parse_args().directory)
