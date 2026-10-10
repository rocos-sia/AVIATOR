"""Headless site-weld grasp regression; requires mujoco, numpy and PyYAML.

Uses the configured flange/tool/grasp transforms and hand targets, a six-second
quintic joint approach, and the simulation node's torque gains. This isolates
contact geometry: it is not a replay of Core's Ruckig trajectory or bus timing.
The wheel is passive; TCP/handle welds lock after approach, without hand contacts.
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


def run(path):
    print(f"MuJoCo {mj.__version__}: {path}")
    grasp = json.loads((ROOT / "config/grasp.json").read_text())
    posture = json.loads((ROOT / "config/posture.json").read_text())
    hand = yaml.safe_load((ROOT / "config/system.yaml").read_text())["core_hand"]
    robot = yaml.safe_load((ROOT / "config/robot.yaml").read_text())
    initial = robot["wheel_initial"]
    stiffness = np.tile(robot["rokae"]["joint_stiffness"], 2)
    model = mj.MjModel.from_xml_path(str(path.resolve()))
    # No hand collision geometry may compete with the six-DOF grasp welds.
    hand_geoms = [g for g in range(model.ngeom)
                  if model.body(model.geom_bodyid[g]).name.startswith(
                      ("left_", "right_", "l_base_link", "r_base_link"))]
    assert hand_geoms
    assert not model.geom_contype[hand_geoms].any()
    assert not model.geom_conaffinity[hand_geoms].any()
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
        # Match DataLink_direct's configured flange->TCP pose.
        gripper = model.body(side + "_gripper").id
        parent = model.body_parentid[gripper]
        parent_r = data.xmat[parent].reshape(3, 3)
        flange_r = data.xmat[flange].reshape(3, 3)
        model.body_pos[gripper] = parent_r.T @ (
            data.xpos[flange] + flange_r @ np.array(tool["position"]) - data.xpos[parent])
        tcp_r = parent_r.T @ flange_r @ rotation(tool["quaternion"])
        mj.mju_mat2Quat(model.body_quat[gripper], tcp_r.ravel())
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
    mj.mj_setConst(model, data)
    data.qpos[:] = start_qpos
    mj.mj_forward(model, data)
    model.dof_damping[av] += 80 * np.sqrt(stiffness / 1000)
    model.dof_damping[hv] += .15
    low = model.jnt_range[hands, 0]
    span = np.diff(model.jnt_range[hands], axis=1).ravel()
    opened = np.array([hand["open"][s] for s in SIDES])
    closed = np.array([hand["close"][s] for s in SIDES])
    welds = [model.equality(side + "_grasp").id for side in SIDES]
    assert not data.eq_active[welds].any()
    # One second to open, six seconds to approach, then 2.5 seconds welded.
    for step in range(round(9.5 / model.opt.timestep)):
        t = np.clip((step * model.opt.timestep - 1) / 6, 0, 1)
        arm_target = home + smooth(t) * (targets - home)
        progress = np.array([smooth((t - start) / (1 - start)) for start in triggers])
        normalized = opened + progress[:, None] * (closed - opened)
        hand_target = low + (1 - normalized.ravel()) * span
        data.qfrc_applied[av] = data.qfrc_bias[av] + stiffness * (arm_target - data.qpos[aq])
        data.qfrc_applied[hv] = np.clip(data.qfrc_bias[hv] + 3 * (hand_target - data.qpos[hq]), -1, 1)
        if step * model.opt.timestep >= 7:
            data.eq_active[welds] = 1
        mj.mj_step(model, data)
        assert not any(c.geom[0] in hand_geoms or c.geom[1] in hand_geoms
                       for c in data.contact), "Unexpected hand contact"
    mj.mj_forward(model, data)
    assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
    assert not data.warning.number.any(), data.warning.number
    assert data.eq_active[welds].all()
    closure = ((data.qpos[hq] - low) / span).reshape(2, 6)
    for s, side in enumerate(SIDES):
        tcp, handle = model.site(side + "_tcp").id, model.site(side + "_handle").id
        error = np.linalg.norm(data.site_xpos[tcp] - data.site_xpos[handle])
        rotation_error = np.linalg.norm(data.site_xmat[tcp] - data.site_xmat[handle])
        print(f"{side}: flexion={closure[s, 1:].round(3).tolist()}, "
              f"weld_position_error={error * 1000:.3f} mm")
        assert closure[s, 1] >= .3 and np.min(closure[s, 2:]) >= .7
        assert error < .001 and rotation_error < .05, (side, error, rotation_error)
    data.eq_active[welds] = 0
    mj.mj_forward(model, data)
    assert not data.eq_active[welds].any()
    print("PASS bilateral site-weld grasp without hand contacts")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=ROOT / "models/mjcf/aviator.xml")
    args = parser.parse_args()
    run(args.model)
