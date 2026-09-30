"""Recheck a frozen 2-D joint table with the AVIATOR MuJoCo geometry.

This reproduces the wall point-cloud/box clearance calculation in
``AviatorRobot/tools/clearance_trajectory.cpp`` and checks actual MuJoCo
contacts at every trajectory sample and every joint-space segment midpoint.
"""
from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

import mujoco
import numpy as np

from examples.experiments.aviator_manifold.manifold_lookup import ManifoldLookup
from tools.audit_static_field import interpolate_q


class CollisionChecker:
    def __init__(self, model_path: str):
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.joint_adr = []
        for side in ("L", "R"):
            prefix = f"AR5-5_07{side}-W4C4A2_joint_"
            for j in range(1, 8):
                jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, prefix + str(j))
                if jid < 0:
                    raise ValueError(f"missing joint {prefix}{j}")
                self.joint_adr.append(int(m.jnt_qposadr[jid]))
        self.wheel_adr = [int(m.jnt_qposadr[mujoco.mj_name2id(
            m, mujoco.mjtObj.mjOBJ_JOINT, name)])
            for name in ("roll_input_joint", "pitch_input_joint")]
        mujoco.mj_forward(m, self.data)

        self.walls = []
        for g in range(m.ngeom):
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(m.geom_bodyid[g])) or ""
            if "aviator_wall" in name:
                self.walls.append((self.data.geom_xpos[g].copy(),
                                   m.geom_size[g].copy(),
                                   self.data.geom_xmat[g].reshape(3, 3).copy()))
        if len(self.walls) != 2:
            raise ValueError(f"expected 2 wall boxes, found {len(self.walls)}")
        middle = (self.walls[0][0] + self.walls[1][0]) / 2
        self.wall_sign = [1 if np.dot(middle - center, rotation[:, 2]) >= 0 else -1
                          for center, _, rotation in self.walls]

        self.clouds = []
        for g in range(m.ngeom):
            owner = self.owner(g)
            if owner < 0 or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            mid = int(m.geom_dataid[g])
            nv = int(m.mesh_vertnum[mid])
            if nv <= 0:
                continue
            step = max(1, nv // 400)
            start = int(m.mesh_vertadr[mid])
            points = np.asarray(m.mesh_vert[start:start + nv:step], dtype=np.float64)
            rot = np.empty(9)
            mujoco.mju_quat2Mat(rot, m.geom_quat[g])
            points = points @ rot.reshape(3, 3).T + m.geom_pos[g]
            self.clouds.append((owner, int(m.geom_bodyid[g]), points.astype(np.float32)))

    def owner(self, geom: int) -> int:
        name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY,
                                 int(self.model.geom_bodyid[geom])) or ""
        if "07L-" in name or name == "left_gripper":
            return 0
        if "07R-" in name or name == "right_gripper":
            return 1
        return -1

    def check(self, x: np.ndarray, q: np.ndarray) -> tuple[float, bool]:
        d = self.data
        d.qpos[self.joint_adr] = q
        d.qpos[self.wheel_adr] = x
        mujoco.mj_forward(self.model, d)
        contact = any(c.dist < -0.001 and
                      (self.owner(c.geom1) >= 0 or self.owner(c.geom2) >= 0)
                      for c in d.contact[:d.ncon])
        clearance = np.inf
        for _, body, points in self.clouds:
            xyz = points @ d.xmat[body].reshape(3, 3).T + d.xpos[body]
            for (center, half, rotation), sign in zip(self.walls, self.wall_sign):
                local = (xyz - center) @ rotation
                excess = np.abs(local) - half
                outside = np.linalg.norm(np.maximum(excess, 0), axis=1)
                dist = np.where(outside > 0, outside, sign * local[:, 2] - half[2])
                clearance = min(clearance, float(np.min(dist)))
        return clearance, contact


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    source = ap.add_mutually_exclusive_group(required=True)
    source.add_argument("--field", type=Path)
    source.add_argument("--rollout-dir", type=Path,
                        help="use saved x/q rollout NPZ files instead of a static field")
    ap.add_argument("--manifold-dir", default="data/aviator/manifold_phi_stale_pitch-10deg")
    ap.add_argument("--model", default="../reference/rocos-mujoco/model/aviator.xml")
    ap.add_argument("--trajectory-dir", default="data/aviator/trajectory_source")
    ap.add_argument("--split", default="val")
    ap.add_argument("--limit", type=int)
    ap.add_argument("--output", type=Path)
    args = ap.parse_args()
    lookup = ManifoldLookup(args.manifold_dir)
    if args.field:
        with np.load(args.field) as field:
            q_grid = field["q"]
            if not (np.array_equal(field["theta"], lookup.theta_axis) and
                    np.array_equal(field["s"], lookup.s_axis)):
                ap.error("field axes do not match manifold")
    else:
        terminations = json.loads((args.rollout_dir / "termination.json").read_text())
    checker = CollisionChecker(args.model)
    paths = sorted(args.rollout_dir.glob("*.npz")) if args.rollout_dir else sorted(
        glob.glob(f"{args.trajectory_dir}/trajs/{args.split}/*.npz"))
    if args.limit:
        paths = paths[:args.limit]
    if not paths:
        ap.error("no trajectories found")
    episodes = {}
    for path in paths:
        with np.load(path) as f:
            x = np.asarray(f["x"], dtype=np.float64)
            if args.rollout_dir:
                q = np.asarray(f["q"], dtype=np.float64)
        if args.field:
            q = interpolate_q(lookup, q_grid, x)
        qdot = float(np.max(np.abs(np.diff(q, axis=0) / 0.01))) if len(q) > 1 else 0.0
        lower, upper = lookup.joint_lower.reshape(14), lookup.joint_upper.reshape(14)
        joint_failures = int(np.sum(np.any((q < lower) | (q > upper), axis=1)))
        min_clearance = np.inf
        contacts = wall_failures = 0
        for xx, qq in zip(x, q):
            clearance, contact = checker.check(xx, qq)
            min_clearance = min(min_clearance, clearance)
            contacts += int(contact)
            wall_failures += int(clearance < 0.005)
        mid_contacts = mid_wall_failures = 0
        for xx, qq in zip((x[1:] + x[:-1]) / 2, (q[1:] + q[:-1]) / 2):
            clearance, contact = checker.check(xx, qq)
            min_clearance = min(min_clearance, clearance)
            mid_contacts += int(contact)
            mid_wall_failures += int(clearance < 0.005)
        terminal = "end" if args.field else terminations[Path(path).stem]
        episodes[Path(path).stem] = {
            "complete": bool(terminal == "end" and qdot <= 1.5 and not joint_failures and
                             not (contacts or wall_failures or mid_contacts or mid_wall_failures)),
            "termination": terminal,
            "joint_limit_samples": joint_failures,
            "max_qdot": float(qdot),
            "min_wall_clearance_m": float(min_clearance),
            "contact_samples": contacts,
            "wall_failure_samples": wall_failures,
            "mid_contact_samples": mid_contacts,
            "mid_wall_failure_samples": mid_wall_failures,
        }
    report = {
        "n_episodes": len(episodes),
        "completed": sum(e["complete"] for e in episodes.values()),
        "contact_episodes": sum(e["contact_samples"] + e["mid_contact_samples"] > 0
                                for e in episodes.values()),
        "wall_failure_episodes": sum(e["wall_failure_samples"] + e["mid_wall_failure_samples"] > 0
                                     for e in episodes.values()),
        "speed_failure_episodes": sum(e["max_qdot"] > 1.5 for e in episodes.values()),
        "joint_limit_episodes": sum(e["joint_limit_samples"] > 0 for e in episodes.values()),
        "min_clearance_m": min(e["min_wall_clearance_m"] for e in episodes.values()),
        "episodes": episodes,
    }
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k != "episodes"}, indent=2))


if __name__ == "__main__":
    main()
