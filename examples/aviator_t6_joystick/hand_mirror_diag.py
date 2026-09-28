"""Diagnose left/right hand mirror symmetry.

1. Identical hand q on both sides: how well do palm-relative fingertip
   positions mirror across the handle-site mirror plane?
2. Sign-flipped q (left = -right): does the left hand then wrap tube 31?
"""
import sys
import numpy as np
import mujoco
sys.path.insert(0, "examples/aviator_t6_joystick")
import find_wrap_ik as fw

m = mujoco.MjModel.from_xml_path("models/mjcf/aviator_physical_grasp.xml")
d = mujoco.MjData(m)

arm_q_r = [-0.08013674488228205, 1.690765542303265, 1.0965538386115117,
           1.526158886661439, -0.22821954727170718, 0.6331749495654962,
           -0.7563863967444805]
hand_q_r = [1.2482222222222223, 0.09133333333333335, 0.7915555555555557,
            0.030444444444444448, 0.975, 0.10923529411764706]

# --- mirror plane from handle sites (same as mirror_wrap_check.py) ---
fw.set_arm(m, d, "right", arm_q_r)
fw.set_hand(m, d, "right", hand_q_r)
mujoco.mj_forward(m, d)
p31 = d.site_xpos[m.site("left_handle").id]
p33 = d.site_xpos[m.site("right_handle").id]
nh = p33 - p31
nh /= np.linalg.norm(nh)
o = 0.5 * (p31 + p33)
H = np.eye(3) - 2 * np.outer(nh, nh)

body_r = m.body("r_base_link").id
body_l = m.body("l_base_link").id
p_r = d.xpos[body_r].copy()
R_r = d.xmat[body_r].reshape(3, 3).copy()
p_L_tgt = H @ (p_r - o) + o
R_L_tgt = H @ R_r @ H

# --- put the left palm on the reflected pose (arm IK from mirror_wrap result) ---
qL0, qR0 = fw.load_seed(fw.CSV, 125)
names = ["AR5-5_07L-W4C4A2_joint_%d" % j for j in range(1, 8)]
addr = np.array([fw.joint_addr(m, n) for n in names])
limits = m.jnt_range[[m.joint(n).id for n in names]]
fw.set_arm(m, d, "left", qL0)
mujoco.mj_forward(m, d)
from scipy.optimize import least_squares


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
d.qpos[addr] = res.x

R_l = d.xmat[body_l].reshape(3, 3)
p_l = d.xpos[body_l].copy()

TIPS = ("thumb_4", "index_2", "middle_2", "ring_2", "little_2")

# --- test 1: identical q, palm-relative tip positions vs mirror ---
fw.set_hand(m, d, "left", hand_q_r)
mujoco.mj_forward(m, d)
print("test 1: left hand q = right hand q")
for tip in TIPS:
    tl = d.xpos[m.body("left_%s" % tip).id] - p_l
    tr = d.xpos[m.body("right_%s" % tip).id] - p_r
    tr_mir = H @ (R_r @ (R_r.T @ tr))  # mirror of right palm-relative vector
    tr_mir = H @ tr                    # world-relative mirror
    err_world = np.linalg.norm(tl - tr_mir) * 1000
    print("  %-9s palm-rel left=%s  mirror(right)=%s  err %.1f mm"
          % (tip, np.round(tl, 3), np.round(tr_mir, 3), err_world))

# --- test 2: sign-flipped q ---
hand_q_flip = [-v for v in hand_q_r]
fw.set_hand(m, d, "left", hand_q_flip)
mujoco.mj_forward(m, d)
metrics = fw.audit(m, d, "left")
print("test 2: left hand q = -right hand q; audit:")
for k, v in metrics.items():
    print("  %s = %s" % (k, v))
print("  pass:", fw.passes(metrics, 10.0, "left"))

# --- test 3: per-joint mirror-consistency at zero pose ---
fw.set_hand(m, d, "left", [0.0] * 6)
fw.set_hand(m, d, "right", [0.0] * 6)
mujoco.mj_forward(m, d)
print("test 3: zero-pose fingertip mirror errors (mm):")
for tip in TIPS:
    tl = d.xpos[m.body("left_%s" % tip).id] - p_l
    tr = d.xpos[m.body("right_%s" % tip).id] - p_r
    err = np.linalg.norm(tl - H @ tr) * 1000
    print("  %-9s %.1f" % (tip, err))
