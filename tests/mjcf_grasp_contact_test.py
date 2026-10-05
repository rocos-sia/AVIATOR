"""Headless physical grasp regression; requires mujoco, numpy and PyYAML.

Uses the configured flange/tool/grasp transforms and hand targets, a six-second
quintic joint approach, and the simulation node's torque gains. This isolates
contact geometry: it is not a replay of Core's Ruckig trajectory or bus timing.
The wheel stays free and grasp welds stay inactive throughout.
"""
import argparse
import json
from pathlib import Path

import mujoco as mj
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
SIDES = ("left", "right")
DRIVES = ("thumb_1", "thumb_2", "index_1", "middle_1", "ring_1", "little_1")


def rotation(quat):
    matrix = np.empty(9)
    mj.mju_quat2Mat(matrix, np.asarray(quat, dtype=float))
    return matrix.reshape(3, 3)


def smooth(t):
    t = np.clip(t, 0, 1)
    return t**3 * (10 - 15*t + 6*t*t)


def solve_pose(model, data, joints, body, position, orientation, seed):
    """Damped Jacobian IK; no extra optimizer dependency or saved joint fixture."""
    adr = model.jnt_qposadr[joints]
    data.qpos[adr] = seed
    jp = np.zeros((3, model.nv))
    jr = np.zeros_like(jp)
    for _ in range(300):
        mj.mj_kinematics(model, data)
        mj.mj_comPos(model, data)
        current = data.xmat[body].reshape(3, 3)
        error = np.r_[position - data.xpos[body], .5 * sum(
            np.cross(current[:, k], orientation[:, k]) for k in range(3))]
        if np.linalg.norm(error) < 1e-8:
            # The cross-product residual also vanishes at 180 degrees.
            assert np.linalg.norm(current - orientation) < 1e-6
            return data.qpos[adr].copy()
        mj.mj_jacBody(model, data, jp, jr, body)
        jac = np.vstack((jp, jr))[:, model.jnt_dofadr[joints]]
        delta = jac.T @ np.linalg.solve(jac @ jac.T + 1e-6*np.eye(6), error)
        data.qpos[adr] = np.clip(data.qpos[adr] + np.clip(delta, -.1, .1),
                                 model.jnt_range[joints, 0], model.jnt_range[joints, 1])
    raise AssertionError(f"Grasp IK did not converge: {error}")


def run(path, report_only=False, grip_inset=0.0):
    print(f"MuJoCo {mj.__version__}: {path}")
    grasp = json.loads((ROOT / "config/grasp.json").read_text())
    posture = json.loads((ROOT / "config/posture.json").read_text())
    hand = yaml.safe_load((ROOT / "config/system.yaml").read_text())["core_hand"]
    robot = yaml.safe_load((ROOT / "config/robot.yaml").read_text())
    initial = robot["wheel_initial"]
    stiffness = np.tile(robot["rokae"]["joint_stiffness"], 2)
    model = mj.MjModel.from_xml_path(str(path.resolve()))
    # Move only physical grip proxies inward; IK/calibrated grasp targets stay fixed.
    assert np.isfinite(grip_inset) and 0 <= grip_inset < .02
    for side in SIDES:
        grip = model.geom(f"steering_wheel_{side}_grip").id
        model.geom_pos[grip, 0] += grip_inset if side == "left" else -grip_inset
    if grip_inset:
        print(f"Grip spacing reduced by {2 * grip_inset * 1000:.1f} mm")
    data = mj.MjData(model)
    mj.mj_resetDataKeyframe(model, data, model.key("aviator_home").id)
    data.qpos[model.joint("roll_input_joint").qposadr] = initial["angle"]
    data.qpos[model.joint("pitch_input_joint").qposadr] = initial["displacement"]
    start_qpos = data.qpos.copy()
    mj.mj_forward(model, data)
    wheel = model.body("steering_wheel").id
    wheel_r = data.xmat[wheel].reshape(3, 3).copy()
    wheel_p = data.xpos[wheel].copy()
    arms, hands, targets, flanges, goals, tools = [], [], [], [], [], []
    for side, letter in zip(SIDES, ("L", "R")):
        joints = np.array([model.joint(f"AR5-5_07{letter}-W4C4A2_joint_{j}").id
                           for j in range(1, 8)])
        flange = model.body(f"AR5-5_07{letter}-W4C4A2_flan_link").id
        tool = grasp["tool"][side]
        goal = wheel_p + wheel_r @ np.array(grasp[side]["position"])
        orient = wheel_r @ rotation(grasp[side]["quaternion"]) @ rotation(tool["quaternion"]).T
        target = solve_pose(model, data, joints, flange,
                            goal - orient @ np.array(tool["position"]), orient,
                            np.deg2rad(posture[side + "_approach_seed_deg"]))
        arms.extend(joints)
        hands.extend(model.joint(f"{side}_{drive}_joint").id for drive in DRIVES)
        targets.extend(target)
        flanges.append(flange)
        goals.append(goal)
        tools.append(np.array(tool["position"]))
    arms, hands, targets = np.array(arms), np.array(hands), np.array(targets)
    aq, av = model.jnt_qposadr[arms], model.jnt_dofadr[arms]
    hq, hv = model.jnt_qposadr[hands], model.jnt_dofadr[hands]
    home = start_qpos[aq]
    triggers = [None, None]
    for k in range(601):
        t = k / 600
        data.qpos[aq] = home + smooth(t) * (targets - home)
        mj.mj_kinematics(model, data)
        for s, flange in enumerate(flanges):
            tcp = data.xpos[flange] + data.xmat[flange].reshape(3, 3) @ tools[s]
            if triggers[s] is None and np.linalg.norm(tcp - goals[s]) <= grasp["hand_closing_distance"]:
                triggers[s] = t
    assert all(t is not None and t < 1 for t in triggers)
    data.qpos[:] = start_qpos
    mj.mj_forward(model, data)
    model.dof_damping[av] += 80 * np.sqrt(stiffness / 1000)
    model.dof_damping[hv] += .15
    low = model.jnt_range[hands, 0]
    span = np.diff(model.jnt_range[hands], axis=1).ravel()
    opened = np.array([hand["open"][s] for s in SIDES])
    closed = np.array([hand["close"][s] for s in SIDES])
    # One second to open, six seconds to approach, then 2.5 seconds to settle.
    for step in range(round(9.5 / model.opt.timestep)):
        t = np.clip((step * model.opt.timestep - 1) / 6, 0, 1)
        arm_target = home + smooth(t) * (targets - home)
        progress = np.array([smooth((t - start) / (1 - start)) for start in triggers])
        normalized = opened + progress[:, None] * (closed - opened)
        hand_target = low + (1 - normalized.ravel()) * span
        data.qfrc_applied[av] = data.qfrc_bias[av] + stiffness * (arm_target - data.qpos[aq])
        data.qfrc_applied[hv] = np.clip(data.qfrc_bias[hv] + 3 * (hand_target - data.qpos[hq]), -1, 1)
        mj.mj_step(model, data)
    mj.mj_forward(model, data)
    assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
    assert not data.warning.number.any(), data.warning.number
    assert not data.eq_active[model.eq_type == mj.mjtEq.mjEQ_WELD].any()
    closure = ((data.qpos[hq] - low) / span).reshape(2, 6)
    failures = []
    for s, side in enumerate(SIDES):
        thumbs, fingers, depths = [], [], []
        for contact_id, contact in enumerate(data.contact):
            a, b = contact.geom
            if model.geom_bodyid[a] == wheel:
                other, normal = b, contact.frame[:3]
            elif model.geom_bodyid[b] == wheel:
                other, normal = a, -contact.frame[:3]
            else:
                continue
            name = model.geom(other).name
            if not name.startswith(side + "_"):
                continue
            force = np.zeros(6)
            mj.mj_contactForce(model, data, contact_id, force)
            if force[0] <= .01:
                continue
            depth_mm = max(0, -contact.dist) * 1000
            depths.append(depth_mm)
            if depth_mm > 8:
                print(f"Deep contact: {model.geom(a).name} / {model.geom(b).name}, "
                      f"depth={depth_mm:.2f} mm, normal_force={force[0]:.2f} N")
            if "thumb" in name:
                thumbs.append(normal.copy())
            elif any(f"_{finger}_" in name for finger in ("index", "middle", "ring", "little")):
                fingers.append(normal.copy())
        opposition = min((a @ b for a in thumbs for b in fingers), default=1.)
        depth = max(depths, default=0.)
        print(f"{side}: flexion={closure[s, 1:].round(3).tolist()}, "
              f"opposition={opposition:.3f}, max_penetration={depth:.2f} mm")
        if closure[s, 1] < .3 or np.min(closure[s, 2:]) < .7:
            failures.append(f"{side}: fingers blocked before wrapping")
        if opposition >= -.2:
            failures.append(f"{side}: missing opposing thumb/finger force contacts")
        if depth > 8:
            failures.append(f"{side}: excessive contact penetration")
    if failures and not report_only:
        raise AssertionError("; ".join(failures))
    print("FAIL " + "; ".join(failures) if failures else "PASS bilateral contact grasp without welds")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=ROOT / "models/mjcf/aviator.xml")
    parser.add_argument("--report-only", action="store_true", help="Report an old model without failing closure checks")
    parser.add_argument("--grip-inset", type=float, default=0.0,
                        help="Move each physical handle inward by this many metres; keep calibration unchanged")
    args = parser.parse_args()
    run(args.model, args.report_only, args.grip_inset)
