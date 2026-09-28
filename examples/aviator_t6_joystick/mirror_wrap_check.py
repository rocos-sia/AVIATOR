"""Mirror the passing RIGHT-hand wrap solution to the LEFT hand.

Fits the mirror plane from the two handle sites, reflects the right palm
frame, IK's the left arm to it, reuses the (mirror-identical) hand q, and
audits. Answers: is the left difficulty real asymmetry or a search gap?
"""
import sys
import numpy as np
import mujoco
sys.path.insert(0, "examples/aviator_t6_joystick")
import find_wrap_ik as fw
from scipy.optimize import least_squares

m = mujoco.MjModel.from_xml_path("models/mjcf/aviator_physical_grasp.xml")
d = mujoco.MjData(m)

arm_q_r = [-0.08013674488228205, 1.690765542303265, 1.0965538386115117,
           1.526158886661439, -0.22821954727170718, 0.6331749495654962,
           -0.7563863967444805]
hand_q_r = [1.2482222222222223, 0.09133333333333335, 0.7915555555555557,
            0.030444444444444448, 0.975, 0.10923529411764706]

fw.set_arm(m, d, "right", arm_q_r)
fw.set_hand(m, d, "right", hand_q_r)
mujoco.mj_forward(m, d)
body_r = m.body("r_base_link").id
p_r = d.xpos[body_r].copy()
R_r = d.xmat[body_r].reshape(3, 3).copy()

# Mirror plane from the two handle sites.
p31 = d.site_xpos[m.site("left_handle").id]
p33 = d.site_xpos[m.site("right_handle").id]
nh = p33 - p31
nh /= np.linalg.norm(nh)
o = 0.5 * (p31 + p33)
H = np.eye(3) - 2 * np.outer(nh, nh)
p_L_tgt = H @ (p_r - o) + o
R_L_tgt = H @ R_r @ H
print("handle sites: p31=", np.round(p31, 4), " p33=", np.round(p33, 4))
print("mirror normal:", np.round(nh, 4))

# Left arm IK from the k=125 seed.
qL0, qR0 = fw.load_seed(fw.CSV, 125)
names = ["AR5-5_07L-W4C4A2_joint_%d" % j for j in range(1, 8)]
addr = np.array([fw.joint_addr(m, n) for n in names])
limits = m.jnt_range[[m.joint(n).id for n in names]]
fw.set_arm(m, d, "left", qL0)
mujoco.mj_forward(m, d)
body_l = m.body("l_base_link").id


def residual(q):
    d.qpos[addr] = q
    mujoco.mj_kinematics(m, d)
    pos = d.xpos[body_l] - p_L_tgt
    rot = fw.Rotation.from_matrix(
        R_L_tgt @ d.xmat[body_l].reshape(3, 3).T).as_rotvec()
    return np.r_[20 * pos, rot]


seed = np.clip(d.qpos[addr].copy(), limits[:, 0] + 1e-8, limits[:, 1] - 1e-8)
res = least_squares(residual, seed, bounds=(limits[:, 0], limits[:, 1]),
                    max_nfev=600, ftol=1e-11, xtol=1e-11, gtol=1e-11)
err = residual(res.x)
print("left IK: pos_err %.3f mm, rot_err %.3f deg"
      % (np.linalg.norm(err[:3]) / 20 * 1000,
         np.linalg.norm(err[3:]) * 180 / np.pi))
d.qpos[addr] = res.x
arm_q_l = list(res.x)
print("arm_q left: ", [round(v, 4) for v in arm_q_l])
diff = [arm_q_l[j] - (-arm_q_r[j] if j in (0, 2, 4, 6) else arm_q_r[j])
        for j in range(7)]
print("vs mirror of right arm_q (per joint, rad):", [round(v, 3) for v in diff])

# Mirrored hand q: identical values.
fw.set_hand(m, d, "left", hand_q_r)
mujoco.mj_forward(m, d)
metrics = fw.audit(m, d, "left")
print("LEFT audit of mirrored right solution:")
for k, v in metrics.items():
    print("  %s = %s" % (k, v))
print("pass:", fw.passes(metrics, 10.0, "left"))
