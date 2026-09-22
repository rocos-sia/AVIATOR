# Aviator v0.1 — Learned Redundancy Policy Training Pipeline (v3)

> **v3 修正说明（相对于 v2，标注 GO 前的最后 4 项 blocker）**

1. ⚠️ **Deterministic wavefront continuation** (Task 0.1): φ 的 continuation 有严格数据依赖。同一 wavefront 层内的 grid point 可并行；层之间必须串行传播。取消 v2 中 "每个 grid point launch thread" 的非确定性描述，改为 propagation tree: anchor → ±θ rows → ±s columns。
2. ⚠️ **Per-arm clearance files** (Task 0.2): 将 `d.bin` 拆分为 `dL.bin` + `dR.bin`，query 时取 `d_min = min(dL, dR)`。`Q_x` 改为垂直拼接 `[Q_x,L; Q_x,R] ∈ ℝ^{14×2}`（θ,s 是共享任务变量，不是 block-diag）。
3. ⚠️ **Closed-loop phase observation encoding** (Task 0.1/1.3/2.2): 双臂 self-motion 是闭环 `S¹`，observation 固定为 `[sin φ_L, cos φ_L, sin φ_R, cos φ_R]`（4-D，无 cut artifact）。观测维度从 38-D 改为 **40-D**。Unwrapped φ̃ 仅在内部用于求 φ̇ 和 safe-arc 检查，不进入 MLP。Open chart (`branch_id ≠ 0`) 使用 branch_id + local normalized coordinate，不混用。
4. ⚠️ **Post-Newton finite-step safety verification** (Task 1.3): Safety filter 的线性化约束仅保证投影后的 `q̇` 安全。实际执行链 `φ_{t+1} → Q(x_{t+1}, φ_{t+1}) → Newton → q_ref^{proj}` 的有限步结果必须二次验证：`max|(q_ref^{proj} - q_t)/dt| ≤ 1.5` 且 `d(q_ref^{proj}) ≥ 5mm`。失败则 2-D backtracking 收缩 φ̇ 至零（最多 5 步），仍失败 → terminate。这是 v0.1 的 "hard shield"。

---

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans. Steps use checkbox syntax.

**Goal:** Close the full data chain `trajectory → manifold Q(x,φ) → full-horizon DP demo → Gym env → BC → 8-env SAC/RLPD` and produce a redundancy-only policy `(x, ẋ, history, φ, local geometry) → (φ̇_L, φ̇_R)` that is collision-free and speed-feasible on held-out trajectories.

**Architecture:** The offline C++ engine (`AviatorRobot/tools/clearance_trajectory.cpp`) builds a phase-consistent `Q(x,φ)` manifold with a transported φ coordinate, then serves as the full-horizon DP teacher. A thin Python/HIL-SERL layer consumes those artifacts: a Gymnasium MuJoCo env, a manifold lookup, a 2-D safety filter, and a BC→RLPD training loop. C++ and Python communicate only through files under `hil-serl/data/aviator/`.

**HIL-SERL pinning:** `c32939b` (current checked-out commit). Record this SHA in `run_actor.sh`/`run_learner.sh`. Do not reference line numbers from upstream main; use the function/class names listed in each task.

**Tech Stack:** C++17 (MuJoCo 3.4, TRAC-IK, KDL, Eigen, YAML-cpp) for the engine/teacher; Python 3.10 + JAX/Flax for HIL-SERL; Gymnasium for the env.

**Spec:** The design brief from the conversation. Supporting project memory: `clearance-study-findings`, `t5-safe-manifold-tracking`, `dg-static-feasibility-certificate`, `random-stream-rate-limit`.

## Global Constraints (FROZEN for v0.1)

- Gap inner width `g = 0.51 m`.
- Policy control period `dt = 0.01 s` (100 Hz); MuJoCo physics `dt_phys = 0.001 s` → 10 substeps per action.
- Hard joint speed `|q̇_i| ≤ 1.5 rad/s`; hard clearance `d_safe = 5 mm = 0.005 m`.
- Task state `x = [θ, s]`, ranges `θ ∈ [-0.87266, +0.87266] rad`, `s ∈ [-0.16, 0.0] m`.
- Redundancy coordinate: **transported phase φ** (per-arm, cut at anchor config, continuous across x via continuation).
- Policy action `a ∈ [-1,1]²` maps to nominal `φ̇_nom = a ⊙ φ̇_scale`. **φ̇_scale is deferred** — calibrate after 100 DP trajectories are dumped (see Phase-3 note at end).
- **φ observation encoding:** Since each arm's self-motion is a closed `S¹` loop, the policy observes phase as `[sin φ_L, cos φ_L, sin φ_R, cos φ_R]` (4-D, no cut artifact). The unwrapped `φ̃` is used **only internally** for computing `φ̇` and for the safety filter's arc-length check. Raw unwrapped φ never reaches the MLP.
- Discount `γ = 0.999`.
- Data split: single trajectory source → 100 DP + 400 RL + 50 val + N test.
- 8 MuJoCo env workers; RLPD demo/RL sampling 50/50.
- Policy network: MLP(256,256,256), tanh-squashed Gaussian actor; critic MLP(256,256,256), ensemble 2.
- The network never outputs 14-D joint commands; it outputs only `(φ̇_L, φ̇_R)`.
- Observation dimension: **40-D** (sin/cos phase encoding for closed-loop `S¹`).

## Single Trajectory Source (Task 0 — Pre-requisite, BLOCKER #7)

Before any Phase 0 or Phase 1 work, create the trajectory source. This is the dependency for both C++ DP teacher and Python env.

**Files:**
- Create: `hil-serl/data/aviator/trajectory_source/generate.py`
- Output: `hil-serl/data/aviator/trajectory_source/trajs/` with subdirs `dp_train/`, `rl_train/`, `val/`, `test/`, plus `split_manifest.json`.

**Interfaces:**
```python
def generate_split(rng, seed: int, n_dp=100, n_rl=400, n_val=50, n_test=100,
                   T_min=8.0, T_max=12.0, dt=0.01) -> dict:
    """Writes trajectory files and returns split_manifest."""
```

- [ ] **Step 1: Implement `generate.py`** using the C² quintic waypoint generator from Task 1.1 (implement Task 1.1 first, then extract the generator here). Each trajectory is a `.npz` with `t, x, xdot, xddot`.
- [ ] **Step 2: Write `split_manifest.json`** with keys `{split, index, seed, T, dt, path}` for every trajectory. Include velocity/acceleration statistics (max |ẋ|, max |ẍ|) and file SHA256.
- [ ] **Step 3: Generate the full split** (`n_dp=100, n_rl=400, n_val=50, n_test=100`). Verify counts and statistics.
- [ ] **Step 4: Commit** `feat(aviator): unified trajectory source with split manifest`.

C++ DP teacher (Task 0.3) and Python env (Task 1.3) both read from this directory. No other generator is used anywhere.

---

## Phase 0 — Gate 0: Phase-consistent φ coordinate + Q(x,φ) manifold + DP teacher (C++)

**Critical reading order:** Tasks 0.1 → 0.2 → 0.3 are strictly sequential. Task 0.0 (trajectory source) must be done first.

### Task 0.0: Read existing infrastructure

Before writing new code, read and understand:
- `self_motion_trace_ex` (~line 993): bidirectional walk, `TraceResult` struct, `StopReason` enum
- `existence_test` (~line 4146–4360): `collect_boundary_candidates`, `minimax` lambda, candidate struct fields
- `build_manifold` (~line 3580): worker-pool pattern, bilinear interpolation helper
- `build_atlas` (~line 3205): file I/O pattern (`std::ofstream` binary, `manifest.json`)
- `Manifold::lookup`: the existing bilinear interpolator signature

Do not modify any of these existing functions. New code calls them.

### Task 0.1: Transported phase φ on the self-motion loop (BLOCKER #1)

**Files:**
- Modify: `AviatorRobot/tools/clearance_trajectory.cpp`

**Key change from v1:** φ is defined at one anchor grid point, then transported along the `(θ,s)` grid by continuation. Each grid point's j-th phase point is IK-seeded from the previous grid point's j-th q. This guarantees `φ` has the same physical meaning at every `x`.

**Interfaces:**
- Consumes: `self_motion(int side)` for the nullspace direction, `ik_multi_seed` for IK, `set_config` + `clearance` for d-computation.
- Produces (used by Task 0.2):
  ```cpp
  // Walk the 1-D self-motion loop of `side` from q0 (the anchor config),
  // accumulating transported phase φ using the nullspace direction.
  // q0 carries the physical phase from the previous grid point's continuation.
  // Fills qs/ds at uniform φ steps. Returns L (loop length), safe interval
  // [phi_safe_lo, phi_safe_hi], and branch id.
  bool self_motion_arc(int side, const Q &q0, double th, double s,
                       std::vector<double> &phi, std::vector<Q> &qs, std::vector<double> &ds,
                       double &L, double &phi_safe_lo, double &phi_safe_hi, int &branch);
  ```

- [ ] **Step 1: Implement `self_motion_arc` at the anchor.** Pick `(θ=0, s=0)` as the anchor. Call `ik_multi_seed` at the anchor to get `q0`, then `self_motion(side)` to get the nullspace direction `n`. Walk bidirectionally: `q_{k+1} = q_k + ε·n`, accumulating `φ += ε` (W=I). Record `(phi, q, d)` at every `trace_step`. Detect loop closure by `distW(q_forward_end, q_backward_end) < ε`. Set `phi_safe_lo/hi` to the maximal connected safe interval containing `φ=0`. Set `branch=0` for closed loop; non-zero for open chart segments.

- [ ] **Step 2: Deterministic wavefront continuation along the grid.** Continuation has strict data dependencies: the j-th phase at grid point `(i_theta, i_s)` requires the j-th q from the **previous** grid point. Parallelism across grid points is only safe within a single wavefront layer.

  **Wavefront order (deterministic propagation tree):**
  - Layer 0: anchor `(0, 0)` — run `self_motion_arc` from scratch.
  - Layer 1: all `(±i_theta, 0)` for `i_theta = 1 … n_theta//2` — each seeds from `(i_theta−1, 0)`. These are independent of each other → parallelize within the layer.
  - Layer 2: for each `(i_theta, 0)`, expand to `(i_theta, ±i_s)` for `i_s = 1 … n_s//2` — each seeds from `(i_theta, i_s−1)`. Parallelize within each column.
  - Repeat for all `i_s` until the full grid is covered.

  Implementation: a serial outer loop over wavefront layers. Within each layer, a thread pool processes all independent grid points. The dependency is: `q_seed = qs_from_previous_grid_point[j]` for each `j`. Never spawn threads that depend on each other's output. This guarantees deterministic phase correspondence and reproducible `manifest.json`.

- [ ] **Step 3: Cycle-consistency gate.** After the full grid is built, verify: for each closed loop, traverse all grid points in a cycle and check `‖Q(x_{cycle_end}, φ_j) − Q(x_{anchor}, φ_j)‖ < ε` for all `j`. If any grid point fails, mark `branch` non-zero and exclude from the safe manifold. This is the **gate** — the manifold is only valid where cycle-consistency holds.

- [ ] **Step 4: Smoke test.** `TASKS=rho_arc` at anchor prints `L`, `phi_safe_lo`, `phi_safe_hi`, `branch`, and sample `(phi, d)` values. Cross-check loop extent against `grip-roll-resingularity-scan` memory (reachable θ range, loop collapse).

- [ ] **Step 5: Commit** `feat(clearance): phase-consistent transported φ on self-motion loop (Gate 0)`.

### Task 0.2: Build the Q(x,φ) manifold database (BLOCKER #1, #3)

**Files:**
- Modify: `AviatorRobot/tools/clearance_trajectory.cpp`

**Key changes from v1:** φ replaces raw ρ as the atlas coordinate. Per-arm derivative storage (7-D, not 14-D). No per-point normalization to u ∈ [0,1]. Cycle-consistency check.

**Interfaces:**
- Consumes: `self_motion_arc` (Task 0.1).
- Produces (files at `outdir/manifold_phi/`):
  - `manifest.json`: `{"n_theta":101,"n_s":65,"n_phi":128,"L_phi_L":<L_L>,"L_phi_R":<L_R>,"anchor_theta":0.0,"anchor_s":0.0,"th_min":...,"th_max":...,"s_min":...,"s_max":...}`
  - `phi.bin`: float32 `[n_phi]` — the uniform φ sample grid in **raw transported phase units** (not normalized). Same φ grid for all `(θ,s)`.
  - `qL.bin` / `qR.bin`: float32 `[n_theta*n_s*n_phi*7]` — per-arm joint configs.
  - `dL.bin` / `dR.bin`: float32 `[n_theta*n_s*n_phi]` — **per-arm** clearance. `d_min` is computed at query time as `min(dL, dR)`.
  - `safe.bin`: float32 `[n_theta*n_s*2*2]` — per-arm `[phi_safe_lo_L, phi_safe_hi_L, phi_safe_lo_R, phi_safe_hi_R]` in raw phase units.
  - `branch.bin`: uint8 `[n_theta*n_s]` — 0 = cycle-consistent, non-zero = excluded.
  - `QxL.bin` / `QxR.bin`: float32 `[n_theta*n_s*n_phi*7*2]` — **per-arm** `Q_x = ∂q/∂x ∈ ℝ^{7×2}`.
  - `QphiL.bin` / `QphiR.bin`: float32 `[n_theta*n_s*n_phi*7*1]` — **per-arm** `Q_φ = ∂q/∂φ ∈ ℝ^{7×1}`.

**Storage layout correction (BLOCKER #3):** Per-arm derivatives are `7×2` and `7×1`, not `14×2`. Total derivative storage: `2 arms × (n_theta×n_s×n_phi × (7×2+7×1) × 4 bytes)` ≈ 2 × 101×65×128 × 21 × 4 ≈ **135 MiB**. Acceptable.

At query time, `ManifoldLookup` assembles:
```
Q_x   = [Q_x,L ]   ∈ ℝ^{14×2}    (vertical stack; θ,s are shared task variables)
        [Q_x,R]
```

- [ ] **Step 1: Implement `build_manifold_phi`.** For each grid point `(i_theta, i_s)`, continuation-seed from the previous grid point's `qs` array (same `j` index → same physical phase). Walk the loop, record `(phi, q, d)` at uniform `n_phi=128` samples spanning `[phi_safe_lo, phi_safe_hi]`. Store in the binary formats above.

- [ ] **Step 2: Precompute per-arm derivatives.** Using the stored `(phi, q)` tables:
  - `Q_x ≈ [q(θ+δθ, φ) − q(θ−δθ, φ)] / (2δθ)` stacked with `[q(θ, s+δs, φ) − q(θ, s−δs, φ)] / (2δs)`, evaluated at each φ sample. Use the existing bilinear interpolator for off-grid neighbors.
  - `Q_φ ≈ [q(φ_{j+1}) − q(φ_{j−1})] / (2Δφ)` for interior j; forward/backward at endpoints.
  - Store as `QxL.bin` etc. in `[n_theta*n_s*n_phi*7*2]` and `[n_theta*n_s*n_phi*7*1]` layouts.

- [ ] **Step 3: Cycle-consistency validation.** After the full grid is written, run a verification pass: for each `(i_theta, i_s)` with `branch==0`, check `‖Q(x_{i+1, j}) − IK_continuation(Q(x_{i,j}))‖ < ε` for all `j`. If any fail, set `branch` to non-zero. The Python `validate_manifold.py` tool (Task 1.2) re-checks this on load.

- [ ] **Step 4: CLI + smoke test.** `TASKS=manifold_phi 16`. Verify file sizes: `qL.bin` = `101×65×128×7×4` bytes, `dL.bin` = `101×65×128×4` bytes, `QxL.bin` = `101×65×128×7×2×4` bytes.

- [ ] **Step 5: Commit** `feat(clearance): phase-consistent Q(x,phi) manifold with per-arm derivatives (Gate 0)`.

### Task 0.3: Full-horizon DP teacher dump — `record_dp` (BLOCKER #4, #10)

**Files:**
- Modify: `AviatorRobot/tools/clearance_trajectory.cpp`

**Key changes from v1:** Candidate table carries φ directly (no nearest-q lookup). Teacher action = forward difference of φ. Lexicographic DP objective.

**Interfaces:**
- Consumes: `existence_test`'s `collect_boundary_candidates` + `minimax`, `self_motion_arc` (Task 0.1). Reads trajectories from `trajectory_source/trajs/dp_train/`.
- Produces: `outdir/dp_demo/traj_<i>.csv` with metadata comment `# L_phi_L=<val>,L_phi_R=<val>` followed by columns:
  ```
  t,theta,s,theta_dot,s_dot,
  sin_phi_L,cos_phi_L,sin_phi_R,cos_phi_R,
  phi_dot_L,phi_dot_R,
  qL1..qL7,qR1..qR7,qdotL1..qdotL7,qdotR1..qdotR7,
  dL,dR,d_min,m_phi_minus_L,m_phi_minus_R,m_phi_plus_L,m_phi_plus_R,m_q
  ```
  Unwrapped φ̃ is tracked internally for continuous `φ̇` computation and safe-arc checks. The CSV stores `(sin φ, cos φ)` for the 40-D observation and raw `φ̇` for the action label.

**Lexicographic DP objective (BLOCKER #10):**
```
1. min  max_t  ‖q̇_t / q̇_max‖_∞
2. min  Σ_t  ‖q̇_t‖²
3. min  Σ_t  ‖Δq̇_t‖²
4. max  Σ_t  R_reserve(t)
```
All clearance/speed/joint-limit constraints are hard.

**Candidate → teacher action (BLOCKER #4):** The candidate struct stores `(q, phi, d, branch_id)`. After minimax selects the best candidate sequence, the teacher's action at step `t` is:
```
phi_dot_L_t = (phiL_{t+1} − phiL_t) / dt
phi_dot_R_t = (phiR_{t+1} − phiR_t) / dt
```
No nearest-neighbor lookup. No centered difference for the action label (centered difference can be computed for diagnostic purposes but is not stored as the transition action).

- [ ] **Step 1: Modify candidate collection.** Extend the candidate struct to include `phi_L`, `phi_R` (the transported phase from `self_motion_arc`). These are written alongside `q` during `collect_boundary_candidates`.

- [ ] **Step 2: Implement `record_dp`.** Read trajectories from `trajectory_source/trajs/dp_train/`. For each trajectory: run candidate collection + minimax DP with the lexicographic objective. Extract `phi*_t` directly from the winning candidate sequence. Compute `phi_dot*_t` as forward difference. Apply continuous unwrap to each arm's φ̃ series. Emit CSV.

- [ ] **Step 3: Add CLI branch.** `TASKS=record_dp`. Build and smoke-test on one trajectory. Expected: `dmin >= 0.005` in every row, `max|qdot| <= 1.5`.

- [ ] **Step 4: Commit** `feat(clearance): full-horizon DP teacher with transported φ (record_dp)`.

**Gate-0 exit check:** (1) Cycle-consistency passes for all `(θ,s)` in the operating range `θ∈[−50°,+50°]`, `s∈[−0.16, 0]`. (2) All DP CSV rows are clearance-safe (`dmin >= 0.005`). (3) `validate_manifold.py` passes on the loaded `.bin` files. If any grid point has `branch != 0` in the operating range, surface it — it changes the env's valid region.

---

## Phase 1 — Python data layer

### Task 1.1: Trajectory loader + trajectory generator (BLOCKER #7, #8)

**Files:**
- Create: `hil-serl/examples/experiments/aviator_manifold/trajectory_generator.py`
- Create: `hil-serl/data/aviator/trajectory_source/generate.py` (moves the generator here from Task 0)
- Test: `hil-serl/examples/experiments/aviator_manifold/test_trajectory_generator.py`

**Key change from v1:** This module serves double duty: (a) generate the unified trajectory source, (b) provide the generator for the Python env's `reset()`. The C++ DP teacher reads `.npz` files produced by this module, not its own generator.

**Interfaces:**
```python
def generate_task_trajectory(rng, T: float, dt: float = 0.01,
                             theta_range=(-0.87266, 0.87266),
                             s_range=(-0.16, 0.0),
                             n_waypoints: int = 5,
                             v_max=(1.486, 0.167),
                             a_max=(5.0, 2.0),
                             j_max=(20.0, 10.0)) -> dict:
    """C2 quintic B-spline trajectory. Returns {"t":(N,), "x":(N,2), "xdot":(N,2),
    "xddot":(N,2), "xdddot":(N,2)}. Waypoints carry continuous velocity/acceleration
    (not zero at knots). Velocity/acceleration/jerk bounded by v_max/a_max/j_max."""
def load_trajectory(path: str) -> dict:
    """Load a .npz trajectory file from trajectory_source/trajs/."""
def generate_split(...) -> dict:
    """Generate the full train/val/test split, write .npz files + split_manifest.json."""
```

**C² tolerance (BLOCKER #8):** Test segment-boundary C¹/C² continuity analytically (exact polynomial match at knots, not numerical gradient). For finite-difference checks, use `atol = 1e-6 + 1e-3 * dt²` (velocity tolerance scales with `dt²` for central differences).

**Velocity/acceleration/jerk bounds (BLOCKER #8):** The quintic B-spline between waypoints carries the endpoint velocity and acceleration from the previous segment, ensuring C² continuity. Time-scale each segment so that `|ẋ| ≤ v̄`, `|ẍ| ≤ ā`, `|x⃛| ≤ j̄`. This produces smooth, non-stop motion (no dwell at waypoints).

- [ ] **Step 1: Write failing tests** — bounds, C¹/C² at segment boundaries (analytical), velocity/acceleration/jerk bounds.
- [ ] **Step 2: Run → FAIL.**
- [ ] **Step 3: Implement.** Quintic B-spline with endpoint velocity/acceleration continuity. Time-scaling to respect v_max/a_max/j_max.
- [ ] **Step 4: Run → PASS.**
- [ ] **Step 5: Commit** `feat(aviator): C2 quintic trajectory generator with jerk bounds`.

### Task 1.2: `manifold_lookup.py` (BLOCKER #3)

**Files:**
- Create: `hil-serl/examples/experiments/aviator_manifold/manifold_lookup.py`
- Create: `hil-serl/tools/validate_manifold.py`
- Test: `hil-serl/examples/experiments/aviator_manifold/test_manifold_lookup.py`

**Key changes from v1:** Per-arm `Q_x` (7×2) and `Q_φ` (7×1) storage. Assemble 14-D at query time. Vectorized batch query. Cycle-consistency validation tool.

**Interfaces:**
```python
class ManifoldLookup:
    def __init__(self, manifold_dir: str): ...
    def query(self, x: np.ndarray, phi: np.ndarray) -> dict:
        """x=(theta,s) shape (B,2); phi=(phi_L, phi_R) shape (B,2).
        Returns dict with:
          qL (B,7), qR (B,7), dL (B,), dR (B,), d_min (B,),
          m_phi_minus (B,2), m_phi_plus (B,2), m_q (B,),
          phi_safe_lo (B,2), phi_safe_hi (B,2),
          Q_x (B,14,2), Q_phi (B,14,2),  # assembled per-arm
          m_phi_minus (B,2), m_phi_plus (B,2), m_q (B,),
          branch (B,)
        Raise ValueError if any (x,phi) is outside grid or outside safe interval."""
```

**Assembly at query time:**
```python
# Per-arm derivatives from stored bins:
Qx_L = ...  # (B, 7, 2)
Qx_R = ...  # (B, 7, 2)
Qphi_L = ...  # (B, 7, 1)
Qphi_R = ...  # (B, 7, 1)

# Assemble Q_x: vertical stack (θ,s are shared task variables)
Qx = np.zeros((B, 14, 2))
Qx[:, 0:7, :] = Qx_L
Qx[:, 7:14, :] = Qx_R

# Assemble Q_phi: block-diagonal (phi_dot_L affects left arm only, phi_dot_R affects right arm only)
Qphi = np.zeros((B, 14, 2))
Qphi[:, 0:7, 0:1] = Qphi_L
Qphi[:, 7:14, 1:2] = Qphi_R
```

**Cycle-consistency validation (`validate_manifold.py`):** Load the `.bin` files, for each grid point with `branch==0`, verify that `‖Q(x_{i+1}, φ_j) − IK_forward(Q(x_i, φ_j))‖ < ε` for all `j`. Report max violation. This runs as a CI check after `TASKS=manifold_phi`.

- [ ] **Step 1: Write failing tests** — synthetic manifold (3×3×5), exact interpolation at nodes, bilinear in (θ,s) + linear in φ, assembled Q_x/Q_φ dimensions, batch query shape, out-of-grid raises.
- [ ] **Step 2: Run → FAIL.**
- [ ] **Step 3: Implement.** Numpy-only, fully vectorized. Load bins with `np.fromfile` (or `np.memmap` if memory-constrained). `query` handles `(B,2)` → `(B,...)` in one shot.
- [ ] **Step 4: Run → PASS.**
- [ ] **Step 5: Commit** `feat(aviator): Q(x,phi) manifold lookup + batch query + validate tool`.

### Task 1.3: `env.py` + `safety_filter.py` + `reward.py` (BLOCKER #5, #6, #9)

**Files:**
- Create: `hil-serl/examples/experiments/aviator_manifold/env.py`
- Create: `hil-serl/examples/experiments/aviator_manifold/safety_filter.py`
- Create: `hil-serl/examples/experiments/aviator_manifold/reward.py`
- Test: `hil-serl/examples/experiments/aviator_manifold/test_env.py`, `test_safety_filter.py`

**Interfaces:**

```python
def project_phi_dot(phi_dot_nom: np.ndarray, x: np.ndarray, x_next: np.ndarray,
                    phi: np.ndarray, lookup: ManifoldLookup,
                    qdot_max: float = 1.5, dt: float = 0.01) -> tuple[np.ndarray, dict]:
    """Returns (phi_dot_safe, info). Solves min ‖r − phi_dot_nom‖² s.t.
    |Q_x·ẋ + Q_φ·r| ≤ q̇_max (14D, evaluated at x_next)
    and phi_{t+1} = phi_t + r·dt ∈ [phi_safe_lo(x_next), phi_safe_hi(x_next)].
    If infeasible: returns (zeros, {'feasible': False, 'intervened': True}) and the
    env terminates the episode. No 'least-infeasible' fallback."""
```

```python
def reward_fn(x, phi_dot_nom, phi_dot_safe, d_min, m_phi_minus, m_phi_plus, m_q, lookup, *,
              w_R=1.0, w_v=1.0, w_a=0.1, w_j=0.05, w_f=0.5) -> tuple[float, dict]:
    """Per-arm R_i = 2·min(m_phi_i^-, m_phi_i^+) / (hi_i − lo_i).
    R_reserve = soft-min(R_L, R_R) (not average).
    C_v, C_a, C_da, C_filter as before.
    Hard terminate: d_min < 0.005 OR max|qdot| > 1.5."""
```

**Observation layout (40-D):**
```
[0:2]  x                    [2:4]  xdot
[4:6]  sin(phi_L), cos(phi_L)   [6:8]  sin(phi_R), cos(phi_R)
[8:10] m_phi_minus (L,R)   [10:12] m_phi_plus (L,R)   [12] d_min   [13] m_q
[14:16] a_prev              [16:24] hist_x        [24:32] hist_xdot   [32:40] hist_a
```
`hist_*` = lags `{t-2, t-5, t-10, t-20}` initialized from `t=0`.

> Note: unwrapped `φ̃` is maintained internally (per-arm scalar) for computing `φ̇` and for the safe-arc check. The observation uses only `(sin φ, cos φ)` to avoid the cut-point discontinuity. **Internal state:** `phi_unwrapped_L, phi_unwrapped_R` (not part of the 40-D obs).

**Safety filter: x_next constraint (BLOCKER #5):** The constraint uses `x_next` (known from the trajectory), not `x_current`. Query `phi_safe_lo/hi` at `x_next`. The safe-arc box is `phi_safe_lo(x_next) ≤ phi_t + phi_dot·dt ≤ phi_safe_hi(x_next)`. This prevents the "safe arc moves while the arm is mid-step" leak.

**Empty polygon handling (BLOCKER #5):** When the half-plane intersection is empty, return `feasible=False`. The env terminates with `done=True`. For logging, optionally compute the least-infeasible point but never expose it as `phi_dot_safe`.

**Hard safety verification after Newton projection (BLOCKER #4):** The linearized safety filter constrains the projected joint velocity. But the actual execution path `Q(x_{t+1}, φ_{t+1}) → Newton → q_ref^{proj}` can violate constraints due to (a) the linearization error in `Q_x` and `Q_φ`, and (b) Newton projection moving away from the manifold. Therefore, after computing `q_ref^{proj}`, verify:

```
q_dot_actual = (q_ref^{proj} - q_t) / dt
d_actual     = clearance(q_ref^{proj})
```

and require:
```
max_i |q_dot_actual,i| ≤ 1.5 rad/s
d_actual ≥ 0.005 m
```

If either check fails, the `φ̇` that produced it is **infeasible** — not just the nominal projected velocity, but the actual finite-step result. Implementation: in `project_phi_dot`, after projecting `phi_dot_safe`, compute `q_ref = Q(x_next, phi_t + phi_dot_safe*dt)`, run Newton (1-2 iters, match the env's execution), clamp to joint limits, then check the two conditions. If fail, shrink `phi_dot` toward zero (2-D backtracking: `phi_dot ← 0.5·phi_dot`, recompute, repeat up to 5 steps). If still fail after 5 steps, return `feasible=False` and terminate. This is v0.1's "hard shield" — not a formal CBF, but a verified finite-step guard.

**Closed-loop phase encoding:** Since each arm's self-motion is a closed `S¹` loop, the policy observes phase as `[sin φ_L, cos φ_L, sin φ_R, cos φ_R]` (4-D, no cut artifact). The unwrapped `φ̃` is used **only internally** for computing `φ̇` and for the safety filter's arc-length check. Raw unwrapped φ never reaches the MLP. If an open chart is encountered (`branch_id ≠ 0`), use the branch_id + local normalized coordinate instead — do not mix the two encodings.

**Reward R_reserve fix (BLOCKER #6):** Per-arm: `R_i = 2·min(m_phi_i^-, m_phi_i^+) / (hi_i - lo_i)`. Bilateral: `R_reserve = min(R_L, R_R)` (or `−exp(−β·min(R_L,R_R))` for soft-min). This is 1.0 at center, 0.0 at the safe boundary.

**Hard collision threshold (BLOCKER #6):** `d_min < 0.005` (not `d <= 0`). The `d_safe = 5mm` is the hard limit.

**MuJoCo execution chain (BLOCKER #9):** After computing `phi_dot_safe`:
```
phi_{t+1} = phi_t + phi_dot_safe * dt          # (2,) per-arm unwrapped
q_ref = Q(x_{t+1}, phi_{t+1})                   # (14,) from manifold lookup
q_ref_proj = newton_task_projection(q_ref, ...)  # 1-2 Newton iters, clamp to joint limits
mj_step × 10 substeps with q_ref_proj as position reference
```
The `q_ref_proj → mj_step` interface must match the existing joint position servo (check the MuJoCo actuator mode in the model). Document the exact mj_set_actuator_gainprm / mj_set_actuator_biasprm settings used.

- [ ] **Step 1: Write `safety_filter` tests.** Cases: (a) feasible → unchanged; (b) joint-vel violation → projected; (c) safe-arc violation → clipped using x_next; (d) empty intersection → `feasible=False`.
- [ ] **Step 2: Implement `safety_filter.project_phi_dot`.** 2-D polygon clipping (Sutherland-Hodgman). Constraints: up to 32 joint-vel half-planes (14 joints × 2 sides) + 4 box edges (2 per arm) = 36 edges. Polygon clipping is O(E²) in 2D, ~1300 edge-crossing tests — fast in numpy. Target < 50 μs.
- [ ] **Step 3: Implement `reward.py`** with the corrected R_reserve and d<0.005 threshold.
- [ ] **Step 4: Implement `AviatorManifoldEnv`.** `reset()` loads a trajectory from `trajectory_source/trajs/`, seeds arms at φ=0 via lookup, initializes history deque. `step(a)` executes the full chain: `phi_dot_nom → project_phi_dot(x, x_next) → phi_{t+1} → q_ref → Newton projection → mj_step × 10 → recompute x, d, qdot, phi → obs → reward`. Apply continuous unwrap of φ̃: if `|phi_new − phi_old| > L_phi/2`, adjust by ±L_phi (per-arm L from manifest). History: `deque(maxlen=21)` of `(x, xdot, a)`; extract lags `{t-2, t-5, t-10, t-20}`.
- [ ] **Step 5: Write `test_env.py`** — 200-step rollout, `obs["state"].shape == (40,)`, action bounds, info keys, speed-violation termination. Use the small synthetic manifold (no 100 MB data needed).
- [ ] **Step 6: Run all tests → PASS.**
- [ ] **Step 7: Commit** `feat(aviator): Gym env + 2-D safety filter + reward (x_next aware)`.

### Task 1.4: `build_dp_dataset.py` (BLOCKER #11)

**Files:**
- Create: `hil-serl/tools/build_dp_dataset.py`

**Key change from v1:** φ̇_scale is NOT hardcoded. Read all DP CSVs first, compute `phi_dot_scale = max(max|phi_dot*|_all_trajs) × 1.1` (10% headroom), then normalize actions.

**Interfaces:**
- Consumes: `record_dp` CSVs + `trajectory_source/trajs/dp_train/` trajectories.
- Produces: `hil-serl/data/aviator/dp_demo/traj_<i>.pkl` with transition dicts matching HIL-SERL's replay buffer format.

- [ ] **Step 1: Write failing test** — 3-row CSV → 3 dicts with correct keys, shapes, masks=1.0, dones=False for all-but-last.
- [ ] **Step 2: Run → FAIL.**
- [ ] **Step 3: Implement.** Read CSVs → compute `phi_dot_scale` from all trajectories → build obs40 (θ, s, θ̇, ṡ, sinφ_L, cosφ_L, sinφ_R, cosφ_R, m_φ^−, m_φ^+, d_min, m_q, a_prev, hist_x, hist_xdot, hist_a) → normalize actions → call `reward_fn`. Handle terminal transitions. Internal state tracks `phi_unwrapped_L/R` (not part of obs).
- [ ] **Step 4: Run → PASS.**
- [ ] **Step 5: Write `phi_dot_scale.json`** alongside the pkls: `{"phi_dot_scale": <value>, "n_trajs": 100, "max_abs_phi_dot": [...]}`. This file is read by `config.py` to set the action scale in the env.
- [ ] **Step 6: Commit** `feat(aviator): DP CSV → HIL-SERL demo pkl with calibrated phi_dot_scale`.

**Phase-1 exit check:** `pytest` green. `build_dp_dataset.py` produces pkls that load correctly. `validate_manifold.py` passes. Data chain closed: `trajectory_source → manifold → DP CSV → pkl → env → demo buffer`.

---

## Phase 2 — HIL-SERL integration

### Task 2.1: Experiment config + wiring

**Files:**
- Create: `hil-serl/examples/experiments/aviator_manifold/config.py`
- Create: `hil-serl/examples/experiments/aviator_manifold/wrappers.py`
- Create: `hil-serl/examples/experiments/aviator_manifold/run_actor.sh`, `run_learner.sh`
- Modify: `hil-serl/examples/experiments/mappings.py`

**Interfaces:**
- `config.py`: `TrainConfig(DefaultTrainingConfig)` with `image_keys=[]`, `proprio_keys=["state"]`, `discount=0.999`, `max_traj_length=1200`, `replay_buffer_capacity=1_000_000`. Reads `phi_dot_scale.json` from `data/aviator/dp_demo/` to set the env's action scale.

- [ ] **Step 1: `wrappers.py`.** Tiny adapter: our env emits `Dict({"state": flat Box(40,)})`. SERLObsWrapper expects `Dict({"state": Dict(...), "images": Dict(...)})`. Adapter adds empty `images` dict. Verify against the HIL-SERL commit currently checked out — the exact `observation_space` shape required by `MemoryEfficientReplayBuffer._init_replay_dict` may differ between versions.

- [ ] **Step 2: `config.py` `get_environment`.** `AviatorManifoldEnv` → `ChunkingWrapper(env, obs_horizon=1)`. No Spacemouse, no RelativeFrame, no Quat2Euler.

- [ ] **Step 3: Add to `mappings.py`.** `"aviator_manifold": AviatorTrainConfig`.

- [ ] **Step 4: `run_actor.sh` / `run_learner.sh`.** Mirror `usb_pickup_insertion`. Add `XLA_PYTHON_CLIENT_PREALLOCATE=false`, pin HIL-SERL SHA in comments.

- [ ] **Step 5: Smoke test.** `python -c "from experiments.aviator_manifold.config import TrainConfig; e = TrainConfig().get_environment(); print(e.observation_space, e.action_space)"` → `Box(40,)` and `Box(2,)`.

- [ ] **Step 6: Commit** `feat(aviator): HIL-SERL experiment config + mappings entry`.

### Task 2.2: BC pretraining + sanity evaluation (BLOCKER #12)

**Files:**
- Create: `hil-serl/tools/evaluate_policy.py`
- Modify: `hil-serl/examples/experiments/aviator_manifold/config.py` (agent factory overrides)

**Key changes from v1:** BC gate criteria corrected. Action variance diagnostic added. `rho_dot_scale` → `phi_dot_scale` loaded from JSON.

**BC/SAC architecture alignment:** Using the currently checked-out HIL-SERL commit, verify `make_bc_agent` and `make_sac_agent` (or equivalent) have matching `hidden_dims` and `tanh_squash_distribution`. If they differ, add a factory override in `config.py` that returns agents with `hidden_dims=[256,256,256]`, `tanh_squash_distribution=True` for both BC and SAC. Do not modify HIL-SERL core files.

**BC gate criteria (BLOCKER #12):**
- Must pass: `collision=0`, `max|q̇|≤1.5` on held-out trajectories.
- Should improve over: static T5 baseline and greedy baseline (statistical test on e_task and intervention rate).
- Record but do not enforce: `oracle_regret = J_BC − J_DP` (expect non-zero due to partial observability).
- **Do NOT require** `J_BC ≈ J_DP` as a hard gate.

**Action variance diagnostic:** After BC training, for every demo transition, find the K nearest observations in the demo set (by L2 in obs space). Compute `Var[a_DP | obs]` from those K actions. Report the distribution of this variance. High variance → the teacher uses information (future) the student doesn't have; this explains BC's clairvoyance gap and is expected.

- [ ] **Step 1: Run BC.**
  ```bash
  python examples/train_bc.py --exp_name=aviator_manifold \
      --bc_checkpoint_path=... --train_steps=20000 --eval_n_trajs=0
  ```

- [ ] **Step 2: Verify BC→SAC checkpoint compatibility.** Try loading the BC checkpoint into the SAC agent's actor. If the architectures don't match, fix in `config.py`.

- [ ] **Step 3: Write `evaluate_policy.py`.** Baselines: DP Oracle (re-read CSV), static T5, greedy. Metrics: collision, max|q̇|, e_task, oracle_regret, intervention_rate, action_variance.

- [ ] **Step 4: Run eval.** Check collision=0, |q̇|≤1.5, improvement over static/greedy. Record oracle_regret distribution and action variance.

- [ ] **Step 5: Commit** `feat(aviator): BC pretraining + evaluation harness`.

### Task 2.3: 8-env SAC/RLPD

**Files:**
- Create: `hil-serl/examples/experiments/aviator_manifold/vectorized_actor.py`
- Modify: `hil-serl/examples/train_rlpd.py` (only to register the 8-way env)

- [ ] **Step 1: Write the 8-way actor.** Multiprocessing: 8 `AviatorManifoldEnv` in subprocesses. Each loads its own `mjModel` + `ManifoldLookup`. Batch `sample_actions` over `(8, 40)` once per step. Single JAX device, no GPU per env.

- [ ] **Step 2: Wire the learner.** Replace single `env.step` with vectorized actor. Replay buffer (RL) + demo buffer (DP) at 50/50. `discount=0.999`.

- [ ] **Step 3: Launch and monitor.** WandB metrics: `max|q̇|`, `P95/P99 |q̇|`, `min d`, `max e_task`, `∫‖φ̇‖dt`, `∫‖φ̈‖dt`, `R_reserve`, `intervention_rate`. Success: `collision=0`, `max|q̇|≤1.5`, `e_task<0.1mm`, intervention rate single-digit %, `T_actor,P99 < 2ms`.

- [ ] **Step 4: Deterministic eval at checkpoints.** Mean action, held-out test set. Record spec §22 criteria.

- [ ] **Step 5: Commit** `feat(aviator): 8-env SAC/RLPD actor + learner wiring`.

---

## Phase 3 — φ̇_scale calibration (Post-DP, Pre-BC)

After `record_dp` finishes all 100 trajectories and before `build_dp_dataset.py`:

1. Read all DP CSVs, compute `P95, P99, max|φ̇*|` per arm.
2. Set `phi_dot_scale = max(max|φ̇*|) × 1.1` (10% headroom).
3. If `max|φ̇*| > 2.0 rad/s` for either arm, flag as "teacher exceeds nominal action range" — this is not an error but means the calibrated scale will be >1.0 and the action space `[-1,1]²` needs to be treated as a unit ball, not a hard speed limit.
4. Write `phi_dot_scale.json` to `data/aviator/dp_demo/`.

This replaces the v1 hardcoded `rho_dot_scale = 1.0`.

---

## Validation Scripts (run as CI after each Phase)

```bash
# After Phase 0:
python hil-serl/tools/validate_manifold.py --dir data/aviator/manifold_phi/
# → cycle_consistency_max_violation, n_excluded_points, file_sizes

# After Phase 1:
pytest hil-serl/examples/experiments/aviator_manifold/ -v
# → all green

# After Phase 1 + Phase 0:
python hil-serl/tools/build_dp_dataset.py --check-only
# → validates CSV → pkl round-trip without building
```

---

## Self-Review

**Spec coverage:** Wavefront continuation (#1) → Task 0.1. Per-arm d files (#2) → Task 0.2. Closed-loop phase encoding (#3) → Task 0.1/1.3/2.2. Post-Newton safety (#4) → Task 1.3. Phase-consistent φ (#5) → Task 0.1/0.2. Per-arm Q dimensions (#6) → Task 0.2. Direct φ from candidates (#7) → Task 0.3. x_next safety (#8) → Task 1.3. Reward fix (#9) → Task 1.3. Unified trajectory source (#10) → Task 0 (pre-requisite). Jerk bounds (#11) → Task 1.1. MuJoCo execution chain (#12) → Task 1.3. Lexicographic DP (#13) → Task 0.3. Calibrated φ̇_scale (#14) → Phase 3 + Task 1.4. BC gate fix (#15) → Task 2.2.

**Remaining risk:** The continuation seeding strategy (Task 0.1 Step 2) assumes the self-motion loop doesn't bifurcate between adjacent grid points. If bifurcation occurs at some `(θ,s)`, the continuation seed lands on a different branch → φ jumps. The cycle-consistency gate catches this but the fix (re-seeding from the anchor for that grid point) may create a phase discontinuity in the atlas. This is acceptable in v0.1 (mark `branch != 0`), but the fallback strategy should be decided before Phase 0 implementation.

**HIL-SERL pinning:** SHA `c32939b` (recorded in `run_actor.sh`/`run_learner.sh`). If HIL-SERL is updated, re-verify the agent factory interfaces and replay buffer dict shape.
