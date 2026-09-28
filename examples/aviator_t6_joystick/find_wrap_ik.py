#!/usr/bin/env python3
"""Search palm-wrap IK solutions seeded from the sine-trajectory branches.

Seeds: k=0 and k=125 keyframes of
examples/aviator_clearance_hand/out/keyframes_t0_sine.csv — same
(theta,s)=(0,0) but different left-arm branches. The physical hand mount
(+/−108.26 deg about flange Z, models/mjcf/aviator_physical_grasp.xml) is
kept untouched; only the 7 arm joints per side are solved.

Per seed branch, per side, per wrap gap: the palm anchor is IK'd to `gap` mm
off the tube surface with the palm plane tangent to the tube. The remaining
freedom — palm rotation about its normal (alpha) — is grid-searched over
[-90, +90] deg so the fingers can curl across the tube instead of sliding
along it. Each candidate is finger-tuned (independent joints + mimic ratios)
and audited for penetration of every hand collision geom against every
steering-wheel segment; fingertip contact must land on the side's own handle
segment.

Run with the MuJoCo Python environment (mujo, mujoco 3.3.6) and MUJOCO_GL=egl.
"""
import argparse
import csv
import json
from pathlib import Path

import cv2
import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

CSV = "examples/aviator_clearance_hand/out/keyframes_t0_sine.csv"
HANDLE_GEOM = {"left": 31, "right": 33}
PALM_GEOM = {"left": "l_base_link_collision_0", "right": "r_base_link_collision_0"}
PALM_BODY = {"left": "l_base_link", "right": "r_base_link"}
FINGERS = ("index", "middle", "ring", "little")
TIPS = ("thumb_4", "index_2", "middle_2", "ring_2", "little_2")

# Hand grip from the verified physical pinch (GRASP_POSE.md) — base flexion
# values the finger tuning starts from.
HAND = {
    "left": [0.52, 0.57, 0.50, 0.03991098, 0.77604755, 0.10718285],
    "right": [0.52599652, 0.55878870, 0.50, 0.04051597, 0.77281815, 0.10931270],
}


def joint_addr(m, name):
    return int(m.joint(name).qposadr[0])


def set_arm(m, d, side, q):
    letter = "L" if side == "left" else "R"
    for j, value in enumerate(q, 1):
        d.qpos[joint_addr(m, f"AR5-5_07{letter}-W4C4A2_joint_{j}")] = value


def set_hand(m, d, side, values):
    def put(name, value):
        d.qpos[joint_addr(m, f"{side}_{name}_joint")] = value
    for i, finger in enumerate(FINGERS):
        put(f"{finger}_1", values[i])
        put(f"{finger}_2", 1.0843 * values[i])
    put("thumb_1", values[4])
    put("thumb_2", values[5])
    put("thumb_3", 0.8392 * values[5])
    put("thumb_4", 0.891 * 0.8392 * values[5])


def geom_distance(m, d, a_id, b_id):
    endpoints = np.zeros(6)
    value = mujoco.mj_geomDistance(m, d, a_id, b_id, 1.0, endpoints)
    return float(value), endpoints


def hand_subtree_geoms(m, side):
    root = m.body(PALM_BODY[side]).id
    out = []
    for b in range(m.nbody):
        p = b
        while p != 0 and p != root:
            p = m.body_parentid[p]
        if p == root:
            for g in range(m.body_geomnum[b]):
                name = m.geom(m.body_geomadr[b] + g).name
                if name and name.endswith("_collision_0"):
                    out.append(m.body_geomadr[b] + g)
    return out


def wheel_geoms(m):
    return [i for i in range(m.ngeom)
            if m.geom(i).name and m.geom(i).name.startswith("steering_wheel_collision_")]


def geom_sphere(m, d, g):
    """Conservative world-space bounding sphere of geom g."""
    aabb = m.geom_aabb[g]  # center + half extents in geom frame
    xpos = d.geom_xpos[g]
    xmat = d.geom_xmat[g].reshape(3, 3)
    c = xpos + xmat @ aabb[:3]
    r = float(np.linalg.norm(aabb[3:]))
    return c, r


def audit(m, d, side):
    """Distances (mm) of palm + fingertips to the side's handle, the nearest
    wheel segment per fingertip (exact, over all 34), and the worst
    penetration of any hand collision geom against any wheel segment
    (sphere-culled full scan)."""
    handle = m.geom(f"steering_wheel_collision_{HANDLE_GEOM[side]}").id
    palm = m.geom(PALM_GEOM[side]).id
    result = {"palm_mm": round(1000 * geom_distance(m, d, handle, palm)[0], 3)}
    wheels = wheel_geoms(m)
    wheel_spheres = [geom_sphere(m, d, w) for w in wheels]
    for tip in TIPS:
        tip_geom = m.geom(f"{side}_{tip}_collision_0").id
        result[f"{tip}_mm"] = round(1000 * geom_distance(m, d, handle, tip_geom)[0], 3)
        dists = [geom_distance(m, d, w, tip_geom)[0] for w in wheels]
        result[f"{tip}_nearest_seg"] = int(m.geom(wheels[int(np.argmin(dists))]).name
                                           .split("_")[-1])
    worst_any = 0.0
    worst_nt = 0.0
    pairs = []
    for hg in hand_subtree_geoms(m, side):
        hname = m.geom(hg).name
        is_tip = any(hname == f"{side}_{tip}_collision_0" for tip in TIPS)
        hc, hr = geom_sphere(m, d, hg)
        for w, (wc, wr) in zip(wheels, wheel_spheres):
            if np.linalg.norm(hc - wc) > hr + wr + 0.02:
                continue
            dist, _ = geom_distance(m, d, w, hg)
            worst_any = min(worst_any, dist)
            if not is_tip:
                worst_nt = min(worst_nt, dist)
                if dist < -0.0003:
                    pairs.append((hname, m.geom(w).name.split("_")[-1],
                                  round(1000 * dist, 2)))
    result["worst_penetration_mm"] = round(1000 * worst_any, 3)
    result["worst_non_tip_penetration_mm"] = round(1000 * worst_nt, 3)
    result["penetrating_pairs"] = sorted(pairs, key=lambda p: p[2])[:8]
    result["phys_tcp_to_handle_mm"] = round(
        1000 * np.linalg.norm(
            d.site_xpos[m.site(f"{side}_physical_tcp").id]
            - d.site_xpos[m.site(f"{side}_handle").id]), 3)
    return result


def wrap_target(m, d, side, gap_mm, slide_mm=0.0):
    """Anchor point, palm frame at the seed and palm-facing normal for the wrap.
    slide_mm moves the contact point along the tube (wheel-circle tangent)."""
    handle = m.geom(f"steering_wheel_collision_{HANDLE_GEOM[side]}").id
    palm = m.geom(PALM_GEOM[side]).id
    dist, ep = geom_distance(m, d, handle, palm)
    if abs(dist) < 1e-6:
        raise RuntimeError(f"{side}: palm and handle touch at seed")
    u = (ep[3:] - ep[:3]) / dist  # handle surface -> palm surface
    pos = ep[:3] + (gap_mm / 1000.0) * u
    # Tube tangent: torus major axis = wheel roll axis.
    axis = np.array(m.joint("roll_input_joint").axis)
    wheel_center = d.xpos[m.body("steering_wheel").id]
    r = (ep[:3] - wheel_center)
    r -= (r @ axis) * axis
    t = np.cross(axis, r)
    t /= max(np.linalg.norm(t), 1e-9)
    pos = pos + (slide_mm / 1000.0) * t
    body = m.body(PALM_BODY[side]).id
    xmat = d.xmat[body].reshape(3, 3).copy()
    anchor_local = xmat.T @ (ep[3:] - d.xpos[body])  # anchor in palm frame
    n = -u                                            # palm faces the handle
    return pos, xmat, anchor_local, n, t


def solve_wrap_arm(m, d, side, seed_q, gap_mm, alpha_deg, slide_mm=0.0,
                   beta_deg=0.0, gamma_deg=0.0):
    letter = "L" if side == "left" else "R"
    names = [f"AR5-5_07{letter}-W4C4A2_joint_{j}" for j in range(1, 8)]
    joints = [m.joint(name).id for name in names]
    addr = np.array([joint_addr(m, name) for name in names])
    limits = m.jnt_range[joints]

    set_arm(m, d, side, seed_q)
    mujoco.mj_forward(m, d)
    pos_target, xmat0, anchor_local, n, t = wrap_target(m, d, side, gap_mm,
                                                        slide_mm)
    # Tilt the pad normal about the tube tangent (beta, bar-grasp tilt) and the
    # binormal n x t (gamma, tangential lean) -- the two tilt DOFs of n -- then
    # rotate the palm in-plane by alpha about the tilted normal.
    b = np.cross(n, t)
    nt = Rotation.from_rotvec(np.deg2rad(beta_deg) * t
                              + np.deg2rad(gamma_deg) * b).as_matrix() @ n
    n_local = xmat0.T @ n
    base = Rotation.align_vectors([nt], [xmat0 @ n_local])[0].as_matrix() @ xmat0
    rot_target = Rotation.from_rotvec(np.deg2rad(alpha_deg) * nt).as_matrix() @ base
    body = m.body(PALM_BODY[side]).id

    def residual(q):
        d.qpos[addr] = q
        mujoco.mj_kinematics(m, d)
        xmat = d.xmat[body].reshape(3, 3)
        position = (d.xpos[body] + xmat @ anchor_local) - pos_target
        rotation = Rotation.from_matrix(rot_target @ xmat.T).as_rotvec()
        return np.r_[20 * position, rotation]

    seed = np.clip(d.qpos[addr].copy(), limits[:, 0] + 1e-8, limits[:, 1] - 1e-8)
    result = least_squares(residual, seed, bounds=(limits[:, 0], limits[:, 1]),
                           max_nfev=600, ftol=1e-11, xtol=1e-11, gtol=1e-11)
    error = residual(result.x)
    pos_err_mm = np.linalg.norm(error[:3]) / 20 * 1000
    rot_err_deg = np.linalg.norm(error[3:]) * 180 / np.pi
    d.qpos[addr] = result.x
    mujoco.mj_forward(m, d)
    return list(result.x), pos_err_mm, rot_err_deg


def tune_fingers(m, d, side, base, pen=0.0):
    """Greedy flexion search per finger then thumb grid, contact cost.
    pen adds a cost for driving the tip deep through the surface (m)."""
    values = list(base)
    handle = m.geom(f"steering_wheel_collision_{HANDLE_GEOM[side]}").id
    for finger in FINGERS:
        idx = FINGERS.index(finger)
        best = None
        for flex in np.linspace(0.0, 1.37, 46):
            values[idx] = float(flex)
            set_hand(m, d, side, values)
            mujoco.mj_forward(m, d)
            gap = geom_distance(m, d, handle,
                                m.geom(f"{side}_{finger}_2_collision_0").id)[0]
            cost = (abs(gap) + pen * max(0.0, -0.0004 - gap)
                    + 0.001 * abs(flex - base[idx]))
            if best is None or cost < best[0]:
                best = (cost, flex)
        values[idx] = float(best[1])
    best = None
    for thumb1 in np.linspace(0.0, 1.65, 45):
        for thumb2 in np.linspace(0.0, 0.619, 18):
            values[4], values[5] = float(thumb1), float(thumb2)
            set_hand(m, d, side, values)
            mujoco.mj_forward(m, d)
            gap = geom_distance(m, d, handle,
                                m.geom(f"{side}_thumb_4_collision_0").id)[0]
            cost = (abs(gap) + pen * max(0.0, -0.0004 - gap)
                    + 0.0005 * (abs(thumb1 - base[4]) + abs(thumb2 - base[5])))
            if best is None or cost < best[0]:
                best = (cost, thumb1, thumb2)
    values[4], values[5] = float(best[1]), float(best[2])
    set_hand(m, d, side, values)
    mujoco.mj_forward(m, d)
    return values


def render(m, d, path, azimuth, elevation, distance, lookat):
    m.vis.global_.offwidth = 1000
    m.vis.global_.offheight = 700
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.azimuth, cam.elevation, cam.distance = azimuth, elevation, distance
    cam.lookat[:] = lookat
    with mujoco.Renderer(m, 700, 1000) as renderer:
        renderer.update_scene(d, camera=cam)
        cv2.imwrite(str(path), cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR))


def load_seed(csv_path, k):
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            if int(row["k"]) == k:
                return ([float(row[f"qL{i}"]) for i in range(7)],
                        [float(row[f"qR{i}"]) for i in range(7)])
    raise RuntimeError(f"k={k} not found in {csv_path}")


def passes(metrics, gap_mm, side):
    palm_ok = 0.0 <= metrics["palm_mm"] <= gap_mm + 3.0
    tips_ok = all(-0.5 <= metrics[f"{tip}_mm"] <= 0.3 for tip in TIPS)
    # A tip in contact must touch the own handle tube, not a distant rim arc
    # (the segment tiling is a mesh artifact, so +/-2 segments is allowed).
    allowed = {31: {28, 29, 30, 31, 32}, 33: {0, 1, 31, 32, 33}}
    own_segment = all(
        metrics[f"{tip}_mm"] > 0.3 or
        metrics.get(f"{tip}_nearest_seg") in allowed[HANDLE_GEOM[side]]
        for tip in TIPS)
    clean = metrics["worst_penetration_mm"] >= -0.3
    return palm_ok and tips_ok and clean and own_segment


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="models/mjcf/aviator_physical_grasp.xml")
    parser.add_argument("--csv", default=CSV)
    parser.add_argument("--out", default="outputs/hand_mount_check/wrap_ik")
    parser.add_argument("--gaps-mm", default="3,6,10")
    parser.add_argument("--seeds", default="0,125")
    parser.add_argument("--sides", default="left,right")
    parser.add_argument("--alpha-deg-span", type=float, default=180.0)
    parser.add_argument("--alpha-steps", type=int, default=25)
    parser.add_argument("--slide-mm", default="0")
    parser.add_argument("--beta-deg", default="0")
    parser.add_argument("--gamma-deg", default="0")
    parser.add_argument("--alpha-center", type=float, default=0.0)
    parser.add_argument("--tune-pen", type=float, default=0.0)
    args = parser.parse_args()
    gaps = [float(x) for x in args.gaps_mm.split(",")]
    seeds = [int(x) for x in args.seeds.split(",")]
    sides = [s for s in args.sides.split(",")]
    slides = [float(x) for x in args.slide_mm.split(",")]
    betas = [float(x) for x in args.beta_deg.split(",")]
    gammas = [float(x) for x in args.gamma_deg.split(",")]
    alphas = args.alpha_center + np.linspace(-args.alpha_deg_span / 2,
                                             args.alpha_deg_span / 2,
                                             args.alpha_steps)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    m = mujoco.MjModel.from_xml_path(args.model)
    d = mujoco.MjData(m)
    report = {"gaps_mm": gaps, "alpha_degs": [round(float(a), 2) for a in alphas],
              "slides_mm": slides, "betas_deg": betas, "gammas_deg": gammas,
              "seeds": {}}

    for k in seeds:
        qL0, qR0 = load_seed(args.csv, k)
        seed_report = {}
        for side, seed_q in (("left", qL0), ("right", qR0)):
            if side not in sides:
                continue
            entry = {"seed_k": k, "side": side, "results": {}}
            for gap in gaps:
                best = None
                for slide in slides:
                    for beta in betas:
                        for gamma in gammas:
                            for alpha in alphas:
                                q, perr, rerr = solve_wrap_arm(m, d, side, seed_q,
                                                               gap, alpha, slide,
                                                               beta, gamma)
                                if perr > 2.0 or rerr > 3.0:
                                    continue
                                hand_q = tune_fingers(m, d, side, HAND[side],
                                                      args.tune_pen)
                                metrics = audit(m, d, side)
                                ok = passes(metrics, gap, side)
                                key = (ok, metrics["worst_penetration_mm"])
                                if best is None or key > best[0]:
                                    best = (key, alpha, slide, beta, gamma, q,
                                            perr, rerr, hand_q, metrics)
                if best is not None:
                    _, alpha, slide, beta, gamma, q, perr, rerr, hand_q, metrics = best
                    entry["results"][str(int(gap))] = {
                        "alpha_deg": round(float(alpha), 1),
                        "slide_mm": round(float(slide), 1),
                        "beta_deg": round(float(beta), 1),
                        "gamma_deg": round(float(gamma), 1),
                        "arm_q": q, "ik_pos_err_mm": perr, "ik_rot_err_deg": rerr,
                        "hand_q": hand_q, "metrics": metrics,
                        "pass": passes(metrics, gap, side),
                    }
                else:
                    entry["results"][str(int(gap))] = {"error": "no converged candidate"}
            seed_report[side] = entry
        report["seeds"][str(k)] = seed_report
        for s in seed_report:
            for g in (str(int(x)) for x in gaps):
                e = seed_report[s]["results"].get(g, {})
                if "metrics" in e:
                    print(f"seed k={k} {s} gap={g} alpha={e['alpha_deg']} "
                          f"slide={e['slide_mm']} beta={e['beta_deg']} "
                          f"gamma={e['gamma_deg']} "
                          f"ik_err={e['ik_pos_err_mm']:.2f}mm/{e['ik_rot_err_deg']:.2f}deg "
                          f"pass={e['pass']} | " +
                          " ".join(f"{kk}={vv}" for kk, vv in e["metrics"].items()))

    # Baseline: the verified physical pinch (GRASP_POSE.md) for comparison.
    for side, q in (("left", [0.33500136, 1.62446586, -1.59836666, 2.01742108,
                              -0.07085989, -0.10542336, 0.08459278]),
                    ("right", [-0.31181121, 1.86843677, 1.79887493, 1.98169084,
                               -0.09492177, -0.05794433, 0.19528163])):
        set_arm(m, d, side, q)
    for side in ("left", "right"):
        set_hand(m, d, side, HAND[side])
    mujoco.mj_forward(m, d)
    report["baseline_pinch"] = {side: audit(m, d, side)
                                for side in ("left", "right")}

    (out / "wrap_ik_report.json").write_text(json.dumps(report, indent=2) + "\n")
    print("report ->", out / "wrap_ik_report.json")

    # Renders: one per seed at the best gap-3 candidate (both arms).
    for k in seeds:
        qL0, qR0 = load_seed(args.csv, k)
        for side, seed_q in (("left", qL0), ("right", qR0)):
            set_arm(m, d, side, seed_q)
        for side in sides:
            entry = report["seeds"][str(k)][side]["results"]
            pick = next((g for g in gaps if str(int(g)) in entry), None)
            if pick is not None and "arm_q" in entry[str(int(pick))]:
                set_arm(m, d, side, entry[str(int(pick))]["arm_q"])
                set_hand(m, d, side, entry[str(int(pick))]["hand_q"])
        mujoco.mj_forward(m, d)
        sub = out / f"seed_k{k}"
        sub.mkdir(parents=True, exist_ok=True)
        render(m, d, sub / "front.png", 0, 20, 1.1, (-0.44, 0, -0.01))
        render(m, d, sub / "side.png", 90, 15, 1.2, (-0.44, 0, -0.01))
        print("rendered", sub)


if __name__ == "__main__":
    main()
