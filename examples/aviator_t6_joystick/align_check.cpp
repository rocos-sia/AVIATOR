// Milestone 1: LUT start point vs current MJCF TCP alignment check.
//
// Verifies that the 14 arm joints the LUT returns for a start task state
// (theta, s) place the flange TCP site exactly on the wheel handle site in the
// FULL-HAND model (models/mjcf/aviator.xml, with the hand mounted below the
// flange). This is a pure kinematic check: set wheel qpos, set arm qpos from the
// LUT, mj_forward, then measure TCP-vs-handle position/orientation error.
//
// The hand (24 joints) is left at the aviator_home pose (open) — it sits below
// the flange and does not affect the TCP/handle relationship, which is what the
// LUT (built for the arm chain) must reproduce.
//
// Exit code 0 = aligned (both sides within the MuJoCoDirectDataLink tolerances:
// 3 mm / 0.035 rad), non-zero = misaligned.
#include "t6.hpp"
#include <mujoco/mujoco.h>

#include <array>
#include <cstdio>
#include <cstdlib>
#include <string>

using aviator::t6::Lookup;
using aviator::t6::LookupResult;
using aviator::t6::Vec2;

static constexpr double kPositionTol = 0.003;  // m   (matches MuJoCoDirectDataLink)
static constexpr double kRotationTol = 0.035;  // rad (matches MuJoCoDirectDataLink)

static int jid(mjModel* m, const char* name) { return mj_name2id(m, mjOBJ_JOINT, name); }
static int sid(mjModel* m, const char* name) { return mj_name2id(m, mjOBJ_SITE, name); }

int main(int argc, char** argv) {
    const std::string model = argc > 1 ? argv[1] : "models/mjcf/aviator.xml";
    const std::string lut_dir = argc > 2 ? argv[2] : "hil-serl/data/aviator/manifold_phi";
    const double theta0 = argc > 3 ? std::atof(argv[3]) : 0.0;
    const double s0 = argc > 4 ? std::atof(argv[4]) : -0.08;

    char err[1024];
    mjModel* m = mj_loadXML(model.c_str(), nullptr, err, sizeof(err));
    if (!m) { std::fprintf(stderr, "load failed: %s\n", err); return 2; }
    mjData* d = mj_makeData(m);

    const int home = mj_name2id(m, mjOBJ_KEY, "aviator_home");
    if (home < 0) { std::fprintf(stderr, "missing aviator_home keyframe\n"); return 2; }
    mj_resetDataKeyframe(m, d, home);
    mj_forward(m, d);

    // 14 arm joints (L/R × 7), mapped by name.
    const char* sides[2] = {"L", "R"};
    std::array<int, 14> arm_qadr{};
    for (int s = 0; s < 2; ++s)
        for (int j = 0; j < 7; ++j) {
            char name[64];
            std::snprintf(name, sizeof(name), "AR5-5_07%s-W4C4A2_joint_%d", sides[s], j + 1);
            const int id = jid(m, name);
            if (id < 0) { std::fprintf(stderr, "missing joint %s\n", name); return 2; }
            arm_qadr[s * 7 + j] = m->jnt_qposadr[id];
        }

    // Wheel passive joints.
    const int roll = jid(m, "roll_input_joint");
    const int pitch = jid(m, "pitch_input_joint");
    if (roll < 0 || pitch < 0) { std::fprintf(stderr, "missing wheel joint\n"); return 2; }

    // TCP / handle sites.
    const int tcp[2] = {sid(m, "left_tcp"), sid(m, "right_tcp")};
    const int handle[2] = {sid(m, "left_handle"), sid(m, "right_handle")};
    if (tcp[0] < 0 || tcp[1] < 0 || handle[0] < 0 || handle[1] < 0) {
        std::fprintf(stderr, "missing tcp/handle site\n"); return 2;
    }

    // LUT start point.
    Lookup lut(lut_dir);
    const Vec2 x0{theta0, s0};
    const Vec2 phi0 = lut.initial_phase(x0);
    const LookupResult r = lut.query(x0, phi0);
    std::printf("LUT start x=(theta %.4f, s %.4f), phi=(%.4f, %.4f), branch=%u, d=%.5f\n",
                x0[0], x0[1], phi0[0], phi0[1], r.branch, r.clearance);

    // Place wheel at the task state and arms at the LUT solution.
    d->qpos[m->jnt_qposadr[roll]] = theta0;
    d->qpos[m->jnt_qposadr[pitch]] = s0;
    for (int i = 0; i < 14; ++i) d->qpos[arm_qadr[i]] = r.q[i];
    mj_forward(m, d);

    bool ok = true;
    for (int side = 0; side < 2; ++side) {
        mjtNum delta[3], qa[4], qb[4], qdiff[3];
        mju_sub3(delta, d->site_xpos + 3 * tcp[side], d->site_xpos + 3 * handle[side]);
        mju_mat2Quat(qa, d->site_xmat + 9 * tcp[side]);
        mju_mat2Quat(qb, d->site_xmat + 9 * handle[side]);
        mju_subQuat(qdiff, qa, qb);
        const double perr = mju_norm3(delta);
        const double rerr = mju_norm3(qdiff);
        const bool good = perr < kPositionTol && rerr < kRotationTol;
        ok = ok && good;
        std::printf("%s  |pos_err|=%.5f m  |rot_err|=%.5f rad  -> %s\n",
                    side ? "right" : "left ", perr, rerr, good ? "OK" : "MISALIGNED");
    }

    mj_deleteData(d);
    mj_deleteModel(m);
    std::printf("%s\n", ok ? "ALIGNED" : "MISALIGNED");
    return ok ? 0 : 1;
}
