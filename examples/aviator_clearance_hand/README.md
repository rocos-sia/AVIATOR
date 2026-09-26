# aviator_clearance_hand — B0 baseline clearance on the hand + wall model

Runs the PIN-IK port of `build_manifold_phi` (dev's `tools/aviator_clearance/`)
against the **hand model** (`RH56E2` gripper, `models/mjcf/aviator.xml`) with the
left/right walls added, to reproduce the previous clearance exploration on the
sine trajectory (θ = 0.87266·sin(2πf·t) ≈ ±50°, s = 0).

## Files

- `config.yaml` — points `build_manifold_phi` at the hand URDF + hand MJCF (with
  `aviator_wall_*` geoms) and the grasp/posture calibration.
- `grasp.json` / `posture.json` — copied verbatim from the `grasp-conditioned`
  branch's `AviatorRobot/config/` (wheel + handle geometry is identical across
  the two models, so the grasp frame and approach seeds transfer unchanged).
- `run_b0.sh` — builds-check + runs `TASKS=online_b0`.

## Build (once)

```sh
cmake -S tools/aviator_clearance -B build/aviator_clearance \
      -DAVIATOR_DEPENDENCY_JOBS=$(nproc)
cmake --build build/aviator_clearance -j$(nproc)
```

## Run

```sh
bash examples/aviator_clearance_hand/run_b0.sh
```

The binary lands at `build/aviator_clearance/build_manifold_phi`.

Outputs land in `examples/aviator_clearance_hand/out/`:

- `online_summary_b0.csv` — one row per (profile, frequency) with `d_min`, `n_coll`,
  `safe_frac`, `max_dq`, `max_ddq`, `rate_sat`.
- `trajectory_online_b0_*.csv` — per-knot `d_min` / `coll` / `dq` / `ddq` / `interv`.

## Notes

- `TASKS=online_b0` is method 0 (B0): one-step continuation IK from the previous
  config, no reshaping. It runs three profiles — `step`, `sine` (f = 0.1…1.0 Hz),
  `roll_pull` — at amplitude 0.87266 rad; the `sine` rows are the requested
  ±50° exploration.
- The hand (RH56E2) geoms are **not** in the clearance cloud: `owner()` in the
  port matches arm bodies (`07L-`/`07R-`) only, so the fingers/palm are excluded
  from the wall-clearance measurement (they sit at the wheel, far from the walls).

## Upright thumbs and grasp closure

The MJCF mounts the left hand at +90° and the right at −90° about flange-local
z. The renderer uses `thumb_1=0` for upright thumbs; `1.3` turns them sideways.
The TCP counter-rotates the mounting and includes the hand's −18 mm base offset:
its position in the hand base is 136 mm, giving the calibrated 118 mm from the
flange. Arm IK targets in `grasp.json` remain unchanged.

The recorded T0 baseline freezes an arm's joints if both continuation and home
seed IK fail, while the wheel continues moving. This produces relative slipping
in the video. Changing the mounting or activating a weld cannot correct those
kinematic recordings (`mj_forward` does not project `qpos`).

For a separate offline grasp projection, run from the repository root:

```sh
MUJOCO_GL=egl /home/rocos/miniconda3/envs/mujo/bin/python \
  examples/aviator_clearance_hand/project_grasp.py
```

This preserves the original CSVs and wheel commands, solves failed arm poses
within joint limits, and writes `keyframes_projected_*.csv` and closure reports
to `outputs/hand_mount_check/projected/`. It fails explicitly if grasp closure
cannot be achieved. These are offline projected trajectories, not T0 baseline
results or a validation of physical friction, controller tracking, or dynamic
grasp stability. Clearance values from the original trajectory must be
recomputed before using them with the projected joint configurations.
