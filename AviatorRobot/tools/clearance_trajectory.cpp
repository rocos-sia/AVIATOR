// Offline clearance-aware redundancy controller along a continuous task trajectory.
//
// Three regimes are measured knot-by-knot along the same task path, at a fixed
// rigid weld grasp (no handle-axis rotation is exploited):
//
//   B0  continuation IK            q_k = TRAC_IK(seed = q_{k-1})      (baseline: no reshaping)
//   B1  d*_static(x_k)             max clearance over the WHOLE arm self-motion manifold
//                                  (the pointwise geometric upper bound: what clearance is
//                                   statically achievable if continuity/rates are ignored)
//   B3  least-intervention reshaping  q_k = q_base + alpha_k n, with n the arm's nullspace
//                                  direction, alpha chosen so that clearance >= d_safe with
//                                  the SMALLEST |alpha| (reshape only when required), subject
//                                  to joint-limit margin q_margin, |qdot|<=qdot_max,
//                                  |qddot|<=qddot_max. Starts from a safe pre-shaped posture
//                                  q_safe,0 (closest-to-home self-motion config with d>=d_safe)
//                                  to remove the startup touch artifact.
//
// Per knot we report d_base, d_static, d_b3, collision flags, joint-limit margin,
// max |qdot|/|qddot|, grasp/closure error, the per-knot intervention norm, the d_safe
// slack, and the ACTIVE constraint (none / clearance / dynamic / joint-limit) so the
// failure mode can be read off a constraint-activation strip.
//
// The scientific question: does d_base <= d_executable <= d*_static hold with
// d_executable >= d_safe (continuous executable recovery), and where does B3 fail —
// startup, dynamic rate limits, or joint-limit-induced redundancy exhaustion?
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <numeric>
#include <random>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <kdl_parser/kdl_parser.hpp>
#include <mujoco/mujoco.h>
#include <trac_ik/trac_ik.hpp>
#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;
constexpr double rad = 3.14159265358979323846 / 180.;
using Q = KDL::JntArray;

static KDL::Vector vector(const YAML::Node &n) {
    return {n[0].as<double>(), n[1].as<double>(), n[2].as<double>()};
}
static KDL::Frame frame(const YAML::Node &n) {
    auto q = n["quaternion"];
    return {KDL::Rotation::Quaternion(q[1].as<double>(), q[2].as<double>(), q[3].as<double>(), q[0].as<double>()),
            vector(n["position"])};
}
static double attr(const std::string &tag, const char *name) {
    std::string key = std::string(name) + "=\"";
    size_t p = tag.find(key);
    if (p == std::string::npos)
        return 0.0;
    p += key.size();
    return std::atof(tag.c_str() + p);
}
static std::string fmt(double v) {
    char b[32];
    std::snprintf(b, sizeof(b), "%.2f", v);
    return b;
}

struct TaskSummary {
    std::string tag;
    double d_base_min = 1e9, d_static_min = 1e9, d_b3_min = 1e9;
    int coll_base = 0, coll_b3 = 0, n_safe = 0, N = 0;
    double max_dq = 0, max_ddq = 0, max_e = 0, min_qm = 1e9, J_interv = 0;
    double mean_static_minus_b3 = 0;
    int n_none = 0, n_clearance = 0, n_dynamic = 0, n_joint = 0;
};

// Why a self-motion trace terminated, for the atlas trace-completeness audit.
enum class StopReason { NONE = 0, LOOP_CLOSED = 1, JOINT_LIMIT = 2, IK_FAIL = 3, MAX_STEPS = 4 };

static int stop_code(StopReason r) { return static_cast<int>(r); }

// Per-arm result of a bidirectional self-motion-component trace (atlas diagnostics).
struct TraceResult {
    bool ik_reachable = false;        // deterministic multi-seed baseline IK found a config
    bool component_complete = false;  // full connected self-motion component traversed
    double d_max = -1.0;              // max wall clearance over the component (m)
    double q_margin_at_dmax = 0.0;    // joint-limit margin at the argmax config
    int n_steps_pos = 0, n_steps_neg = 0;
    StopReason stop_pos = StopReason::NONE, stop_neg = StopReason::NONE;
    double closure_error = 1e9;       // weighted ||q_k - q0|| at loop closure
    Q q_max{7};                       // argmax config
    // safe+margin section support: the config on the loop with the LARGEST joint-limit margin
    // among those that are simultaneously safe (d>=d_safe) and margin-feasible (margin>=m_q).
    bool has_safe = false;
    double m_safe = -1.0;
    double d_safe_at = -1.0;
    Q q_safe{7};
};

// Deterministic per-(grid-point, arm, trial) seed for TRAC-IK random restarts.
// Reproducible regardless of thread scheduling (see ik_rng.hpp).
static unsigned int atlas_seed(size_t idx, int side, unsigned int trial) {
    unsigned int h = (unsigned int)idx * 0x9E3779B1u ^ (unsigned int)side * 0x85EBCA6Bu ^ trial * 0xC2B2AE35u;
    h ^= h >> 16;
    h *= 0x7FEB352Du;
    h ^= h >> 15;
    return h ? h : 1u;
}

// Prebuilt 2-D feasibility atlas (module A output → module C input). Holds the raw
// D_g float32 grid and the feasibility mask, with a bilinear lookup matching the
// s-major row-major layout written by build_atlas (index = i_s*n_theta + i_theta).
struct Atlas {
    int n_theta = 101, n_s = 65;
    double th_min = -0.8726646259971648, th_max = 0.8726646259971648;  // ±50°
    double s_min = -0.16, s_max = 0.0;                                  // m
    std::vector<float> Dg;
    std::vector<uint8_t> feasible;
    bool ok = false;

    bool load(const fs::path &dir) {
        size_t N = size_t(n_theta) * n_s;
        auto rd = [&](const char *name, size_t nbytes, std::vector<char> &buf) {
            std::ifstream f(dir / name, std::ios::binary);
            if (!f)
                return false;
            f.read(buf.data(), (std::streamsize)nbytes);
            return f.gcount() == (std::streamsize)nbytes;
        };
        Dg.resize(N);
        feasible.resize(N);
        std::vector<char> b1(N * sizeof(float)), b2(N);
        if (!rd("atlas_Dg.bin", N * sizeof(float), b1) || !rd("atlas_feasible.bin", N, b2))
            return false;
        std::memcpy(Dg.data(), b1.data(), N * sizeof(float));
        std::memcpy(feasible.data(), b2.data(), N);
        ok = true;
        return true;
    }

    // Bilinear D_g at (θ, s); returns -1 outside the grid (or if not loaded).
    double lookup(double th, double s) const {
        if (!ok)
            return -1.0;
        double x = (th - th_min) / (th_max - th_min) * (n_theta - 1);
        double y = (s - s_min) / (s_max - s_min) * (n_s - 1);
        if (x < 0.0 || x > double(n_theta - 1) || y < 0.0 || y > double(n_s - 1))
            return -1.0;
        int ix = std::min((int)x, n_theta - 2), iy = std::min((int)y, n_s - 2);
        double fx = x - ix, fy = y - iy;
        int i0 = iy * n_theta + ix;
        return Dg[i0] * (1 - fx) * (1 - fy) + Dg[i0 + 1] * fx * (1 - fy) + Dg[i0 + n_theta] * (1 - fx) * fy +
               Dg[i0 + n_theta + 1] * fx * fy;
    }
};

// T5 safe-manifold SECTION: Q_g(θ,s) = (qL*, qR*) ∈ R^14 at each grid point, with bilinear
// interpolation. Unlike Atlas (which stores only the scalar D_g), this stores the actual
// 14-D config chosen for each point, so online is a lookup + small projection instead of an
// IK search. Layout matches build_manifold: index = i_s*n_theta + i_theta; qL/qR are N*7
// row-major (point-major, joint inner).
struct Manifold {
    int n_theta = 101, n_s = 65;
    double th_min = -0.8726646259971648, th_max = 0.8726646259971648;  // ±50°
    double s_min = -0.16, s_max = 0.0;                                  // m
    std::vector<float> qL, qR, dcont;  // N*7, N*7, N (min clearance of the section)
    bool ok = false;

    bool load(const fs::path &dir) {
        size_t N = size_t(n_theta) * n_s;
        qL.resize(N * 7);
        qR.resize(N * 7);
        dcont.resize(N);
        auto rd = [&](const char *name, void *buf, size_t nbytes) {
            std::ifstream f(dir / name, std::ios::binary);
            if (!f)
                return false;
            f.read((char *)buf, (std::streamsize)nbytes);
            return f.gcount() == (std::streamsize)nbytes;
        };
        if (!rd("manifold_qL.bin", qL.data(), N * 7 * sizeof(float)) ||
            !rd("manifold_qR.bin", qR.data(), N * 7 * sizeof(float)) ||
            !rd("manifold_dcont.bin", dcont.data(), N * sizeof(float)))
            return false;
        ok = true;
        return true;
    }

    // Bilinear 14-D config at (θ,s); returns false outside the grid. q14 = [qL; qR].
    bool lookup(double th, double s, std::array<double, 14> &q14) const {
        if (!ok)
            return false;
        double x = (th - th_min) / (th_max - th_min) * (n_theta - 1);
        double y = (s - s_min) / (s_max - s_min) * (n_s - 1);
        if (x < 0.0 || x > double(n_theta - 1) || y < 0.0 || y > double(n_s - 1))
            return false;
        int ix = std::min((int)x, n_theta - 2), iy = std::min((int)y, n_s - 2);
        double fx = x - ix, fy = y - iy;
        size_t i00 = size_t(iy) * n_theta + ix, i10 = i00 + 1, i01 = i00 + n_theta, i11 = i01 + 1;
        double w00 = (1 - fx) * (1 - fy), w10 = fx * (1 - fy), w01 = (1 - fx) * fy, w11 = fx * fy;
        for (int j = 0; j < 7; j++) {
            q14[j] = w00 * qL[i00 * 7 + j] + w10 * qL[i10 * 7 + j] + w01 * qL[i01 * 7 + j] +
                     w11 * qL[i11 * 7 + j];
            q14[7 + j] = w00 * qR[i00 * 7 + j] + w10 * qR[i10 * 7 + j] + w01 * qR[i01 * 7 + j] +
                         w11 * qR[i11 * 7 + j];
        }
        return true;
    }
};

class ClearanceTrajectory {
  public:
    YAML::Node config, grasp, posture;
    fs::path cfg_path;  // absolute path of the config YAML (for per-thread instances)
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> m{nullptr, mj_deleteModel};
    std::unique_ptr<mjData, decltype(&mj_deleteData)> d{nullptr, mj_deleteData};
    std::unique_ptr<TRAC_IK::TRAC_IK> ik[2];
    KDL::Frame origin, tool;
    KDL::Frame handle[2];
    KDL::Vector handle_axis[2];    // grip cylinder axis (wheel frame, unit), for handle-axis rotation φ
    KDL::Vector handle_radial[2];  // wheel-center → handle radial (wheel frame, unit), for R2 tilt β_r
    KDL::Vector wheel_normal;      // steering axis (wheel frame) = (0,0,1), for R2 tilt β_n
    Q seeds[2]{Q(7), Q(7)}, lower[2]{Q(7), Q(7)}, upper[2]{Q(7), Q(7)};
    int joint[2][7]{}, wheel[2]{}, tcp_site[2]{}, handle_site[2]{};
    double qdot_max[2][7]{};
    double qddot_max = 10.0;
    double d_safe = 0.005;
    double q_margin_target = 0.03;
    int trace_half = 10;
    double trace_step = 0.1;
    double dt = 0.02;  // knot spacing (seconds); coarsen for fast static-envelope scans
    int alpha_grid = 201;
    double wall_thickness = 0.03;
    std::vector<int> wall_geoms;
    // Decimated surface point cloud per arm side (body-local coords) + cached wall
    // boxes, so clearance is an exact point-to-box signed distance (monotonic in wall
    // position). mj_geomDistance on concave mesh geoms returns spurious 0 near contact,
    // which made d*_static non-monotonic across the gap sweep.
    struct Cloud { int body; std::vector<std::array<float, 3>> pts; };
    std::vector<Cloud> cloud[2];
    std::array<double, 3> wall_center[2]{}, wall_half[2]{};
    std::array<double, 9> wall_xmat[2]{};
    std::array<double, 3> wall_thick_dir[2]{};   // wall thickness axis (local Z) in world
    double wall_inner_sign[2]{1.0, 1.0};         // +1 if workspace is on +Z side, else -1

    explicit ClearanceTrajectory(const fs::path &path, double d_safe, double q_margin, int trace_half,
                                 double trace_step)
        : d_safe(d_safe), q_margin_target(q_margin), trace_half(trace_half), trace_step(trace_step),
          cfg_path(path) {
        config = YAML::LoadFile(path.string());
        auto resolve = [&](const char *key) { return path.parent_path() / config[key].as<std::string>(); };
        grasp = YAML::LoadFile(resolve("grasp").string());
        posture = YAML::LoadFile(resolve("posture").string());
        tool = frame(grasp["tool"]);
        for (int s = 0; s < 2; s++) {
            handle[s] = frame(grasp[s ? "right" : "left"]);
            auto ax = grasp["handle_geometry"][s ? "right" : "left"]["axis"];
            handle_axis[s] = KDL::Vector(ax[0].as<double>(), ax[1].as<double>(), ax[2].as<double>());
            handle_axis[s].Normalize();
        }
        // Radial (wheel center → handle) and steering-normal axes for the R2/R3
        // orientation-release upper bounds. The wheel center is the midpoint of the two
        // handle centers (the handles sit symmetrically at ±X of the yoke).
        {
            KDL::Vector c[2];
            for (int s = 0; s < 2; s++) {
                auto hg = grasp["handle_geometry"][s ? "right" : "left"]["center"];
                c[s] = KDL::Vector(hg[0].as<double>(), hg[1].as<double>(), hg[2].as<double>());
            }
            KDL::Vector wc = 0.5 * (c[0] + c[1]);
            for (int s = 0; s < 2; s++) {
                handle_radial[s] = c[s] - wc;
                handle_radial[s].Normalize();
            }
            wheel_normal = KDL::Vector(0.0, 0.0, 1.0);  // roll/pitch joints are both axis="0 0 1"
        }

        char error[1024]{};
        m.reset(mj_loadXML(resolve("model").c_str(), nullptr, error, sizeof(error)));
        if (!m)
            throw std::runtime_error(error);
        d.reset(mj_makeData(m.get()));
        mj_forward(m.get(), d.get());
        int b = mj_name2id(m.get(), mjOBJ_BODY, "steering_wheel");
        if (b < 0)
            throw std::runtime_error("Missing steering_wheel body");
        auto *r = d->xmat + 9 * b;
        auto *p = d->xpos + 3 * b;
        origin = {KDL::Rotation(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]),
                  KDL::Vector(p[0], p[1], p[2])};
        wheel[0] = mj_name2id(m.get(), mjOBJ_JOINT, "roll_input_joint");
        wheel[1] = mj_name2id(m.get(), mjOBJ_JOINT, "pitch_input_joint");
        if (wheel[0] < 0 || wheel[1] < 0)
            throw std::runtime_error("Missing passive wheel joints");
        for (int g = 0; g < m->ngeom; g++) {
            const char *bn = mj_id2name(m.get(), mjOBJ_BODY, m->geom_bodyid[g]);
            std::string name = bn ? bn : "";
            if (name.find("aviator_wall") != std::string::npos) {
                wall_geoms.push_back(g);
                if (m->geom_type[g] == mjGEOM_BOX)
                    wall_thickness = std::max(wall_thickness, 2.0 * m->geom_size[3 * g + 2]);
            }
        }
        // Cache the (static) wall boxes once: world center, half-extents, orientation.
        for (int wi = 0; wi < (int)wall_geoms.size() && wi < 2; wi++) {
            int g = wall_geoms[wi];
            for (int r = 0; r < 3; r++) {
                wall_center[wi][r] = d->geom_xpos[3 * g + r];
                wall_half[wi][r] = m->geom_size[3 * g + r];
                for (int c = 0; c < 3; c++)
                    wall_xmat[wi][3 * r + c] = d->geom_xmat[9 * g + 3 * r + c];
            }
        }
        // For deep penetration the box signed-distance saturates at the half-thickness
        // (it reports distance to the *nearest* face, which flips past the wall center).
        // Measure penetration against the workspace-facing (inner) face plane instead,
        // which is monotonic and unbounded. Inner face normal = wall thickness axis (local
        // Z), signed so it points toward the workspace (the other wall / origin).
        {
            int nw = (int)wall_geoms.size();
            std::array<double, 3> ws_center{0.0, 0.0, 0.0};
            if (nw == 2)
                for (int r = 0; r < 3; r++)
                    ws_center[r] = 0.5 * (wall_center[0][r] + wall_center[1][r]);
            for (int wi = 0; wi < nw && wi < 2; wi++) {
                const double *R = wall_xmat[wi].data();
                wall_thick_dir[wi] = {R[2], R[5], R[8]};  // third column = local Z in world
                double dot = 0.0;
                for (int r = 0; r < 3; r++)
                    dot += (ws_center[r] - wall_center[wi][r]) * wall_thick_dir[wi][r];
                wall_inner_sign[wi] = dot >= 0.0 ? 1.0 : -1.0;
            }
        }
        // Build the arm surface point cloud (decimated mesh vertices, body-local).
        for (int g = 0; g < m->ngeom; g++) {
            int side = owner(g);
            if (side < 0 || m->geom_type[g] != mjGEOM_MESH)
                continue;
            int mid = m->geom_dataid[g];
            int nv = m->mesh_vertnum[mid];
            if (nv <= 0)
                continue;
            int stride = std::max(1, nv / 400);
            Cloud cl;
            cl.body = m->geom_bodyid[g];
            cl.pts.reserve((nv + stride - 1) / stride);
            const float *v = m->mesh_vert + 3 * m->mesh_vertadr[mid];
            const double *gp = m->geom_pos + 3 * g;
            const double *gq = m->geom_quat + 4 * g;
            for (int i = 0; i < nv; i += stride) {
                double vec[3] = {v[3 * i], v[3 * i + 1], v[3 * i + 2]}, rot[3];
                mju_rotVecQuat(rot, vec, gq);
                cl.pts.push_back({(float)(gp[0] + rot[0]), (float)(gp[1] + rot[1]), (float)(gp[2] + rot[2])});
            }
            cloud[side].push_back(std::move(cl));
        }

        KDL::Tree tree;
        if (!kdl_parser::treeFromFile(resolve("urdf").string(), tree))
            throw std::runtime_error("Cannot parse URDF");
        for (int s = 0; s < 2; s++) {
            auto side = std::string(s ? "right" : "left"),
                 prefix = std::string("AR5-5_07") + (s ? "R" : "L") + "-W4C4A2_";
            KDL::Chain chain;
            if (!tree.getChain("aircraft", prefix + "flan_link", chain) || chain.getNrOfJoints() != 7)
                throw std::runtime_error("Expected seven-joint arm chain");
            for (int j = 0; j < 7; j++) {
                joint[s][j] = mj_name2id(m.get(), mjOBJ_JOINT, (prefix + "joint_" + std::to_string(j + 1)).c_str());
                lower[s](j) = m->jnt_range[2 * joint[s][j]];
                upper[s](j) = m->jnt_range[2 * joint[s][j] + 1];
                seeds[s](j) = posture[side + "_approach_seed_deg"][j].as<double>() * rad;
            }
            const double margin = posture["joint2_planning_margin_deg"].as<double>();
            lower[s](1) = (posture["joint2_limits_deg"][0].as<double>() + margin) * rad;
            upper[s](1) = (posture["joint2_limits_deg"][1].as<double>() - margin) * rad;
            ik[s] = std::make_unique<TRAC_IK::TRAC_IK>(chain, lower[s], upper[s], .005, 7e-7, TRAC_IK::Speed);
            tcp_site[s] = mj_name2id(m.get(), mjOBJ_SITE, (side + "_tcp").c_str());
            handle_site[s] = mj_name2id(m.get(), mjOBJ_SITE, (side + "_handle").c_str());
            if (tcp_site[s] < 0 || handle_site[s] < 0)
                throw std::runtime_error("Missing TCP/handle site");
        }
        load_velocity_limits(resolve("urdf"));
    }

    void load_velocity_limits(const fs::path &urdf) {
        std::ifstream in(urdf);
        std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        for (int s = 0; s < 2; s++) {
            auto prefix = std::string("AR5-5_07") + (s ? "R" : "L") + "-W4C4A2_";
            for (int j = 0; j < 7; j++) {
                std::string jn = prefix + "joint_" + std::to_string(j + 1);
                size_t jp = text.find("<joint name=\"" + jn + "\"");
                qdot_max[s][j] = 1.0;
                if (jp == std::string::npos)
                    continue;
                size_t je = text.find("</joint>", jp);
                size_t lp = text.find("<limit", jp);
                if (lp != std::string::npos && lp < je) {
                    size_t le = text.find("/>", lp);
                    double v = attr(text.substr(lp, le - lp), "velocity");
                    if (v > 0)
                        qdot_max[s][j] = v;
                }
                size_t hp = text.find("<hardware", jp);
                if (hp != std::string::npos && hp < je) {
                    size_t hl = text.find("<limit", hp);
                    if (hl != std::string::npos && hl < je) {
                        size_t he = text.find("/>", hl);
                        double a = attr(text.substr(hl, he - hl), "acc");
                        if (a > 0)
                            qddot_max = std::min(qddot_max, a);
                    }
                }
            }
        }
    }

    // Handle orientation under the constraint-release ladder. φ spins about the grip axis
    // (R1); β tilts about the radial (beta_axis=0) or the steering normal (beta_axis=1) —
    // these are the R2 kinematic-upper-bound relaxations that are NOT coaxial with the
    // handle (they model palm-tilt, not a rigid weld).
    KDL::Rotation handle_rot(int side, double phi, double beta, int beta_axis) {
        KDL::Rotation Rb = beta_axis == 0 ? KDL::Rotation::Rot(handle_radial[side], beta)
                                          : KDL::Rotation::Rot(wheel_normal, beta);
        return KDL::Rotation::Rot(handle_axis[side], phi) * Rb * handle[side].M;
    }
    KDL::Frame target_Rh(int side, double theta, double s, const KDL::Rotation &Rh) {
        return origin * KDL::Frame(KDL::Rotation::RotZ(theta), KDL::Vector(0, 0, s)) *
               KDL::Frame(Rh, handle[side].p) * tool.Inverse();
    }
    KDL::Frame target(int side, double theta, double s, double phi = 0.0, double beta = 0.0,
                      int beta_axis = 0) {
        // φ rotates the grasp about the handle (grip) axis through the weld point: the grasp
        // cylinder stays coaxial with the handle, so the weld position is invariant and only
        // the flange orientation spins. Rigid weld is φ = 0.
        return target_Rh(side, theta, s, handle_rot(side, phi, beta, beta_axis));
    }
    void set(int side, const Q &q) {
        for (int j = 0; j < 7; j++)
            d->qpos[m->jnt_qposadr[joint[side][j]]] = q(j);
    }
    void set_config(const Q &ql, const Q &qr, double theta, double s) {
        set(0, ql);
        set(1, qr);
        d->qpos[m->jnt_qposadr[wheel[0]]] = theta;
        d->qpos[m->jnt_qposadr[wheel[1]]] = s;
        mj_forward(m.get(), d.get());
    }
    int owner(int geom) {
        const char *raw = mj_id2name(m.get(), mjOBJ_BODY, m->geom_bodyid[geom]);
        std::string name = raw ? raw : "";
        if (name.find("07L-") != std::string::npos || name == "left_gripper")
            return 0;
        if (name.find("07R-") != std::string::npos || name == "right_gripper")
            return 1;
        return -1;
    }
    bool collision(int side = -1) {
        for (int k = 0; k < d->ncon; k++) {
            auto &c = d->contact[k];
            if (c.dist >= -.001)
                continue;
            int a = owner(c.geom1), b = owner(c.geom2);
            if (a < 0 && b < 0)
                continue;
            if (side >= 0 && ((a != side && b != side) || a == 1 - side || b == 1 - side))
                continue;
            return true;
        }
        return false;
    }
    double joint_margin() {
        double margin = 1e9;
        for (int s = 0; s < 2; s++)
            for (int j = 0; j < 7; j++) {
                int jid = joint[s][j];
                double q = d->qpos[m->jnt_qposadr[jid]];
                margin = std::min(margin, std::min(q - m->jnt_range[2 * jid], m->jnt_range[2 * jid + 1] - q));
            }
        return margin;
    }
    double wall_clearance(int side) {
        double dmin = 1e9;
        for (const auto &cl : cloud[side]) {
            const double *bp = d->xpos + 3 * cl.body;
            const double *bm = d->xmat + 9 * cl.body;
            for (const auto &pt : cl.pts) {
                double wx = bp[0] + bm[0] * pt[0] + bm[1] * pt[1] + bm[2] * pt[2];
                double wy = bp[1] + bm[3] * pt[0] + bm[4] * pt[1] + bm[5] * pt[2];
                double wz = bp[2] + bm[6] * pt[0] + bm[7] * pt[1] + bm[8] * pt[2];
                for (int wi = 0; wi < 2 && wi < (int)wall_geoms.size(); wi++) {
                    const double *c = wall_center[wi].data(), *h = wall_half[wi].data(),
                                 *R = wall_xmat[wi].data();
                    double dx = wx - c[0], dy = wy - c[1], dz = wz - c[2];
                    double lx = R[0] * dx + R[3] * dy + R[6] * dz;  // R^T (p - c)
                    double ly = R[1] * dx + R[4] * dy + R[7] * dz;
                    double lz = R[2] * dx + R[5] * dy + R[8] * dz;
                    double qx = std::fabs(lx) - h[0], qy = std::fabs(ly) - h[1], qz = std::fabs(lz) - h[2];
                    double ax = std::max(0.0, qx), ay = std::max(0.0, qy), az = std::max(0.0, qz);
                    double outside = std::sqrt(ax * ax + ay * ay + az * az);
                    // inside the box: signed distance to the inner face plane (monotonic
                    // penetration), not to the nearest face (which saturates past center).
                    double dist = outside > 0 ? outside : wall_inner_sign[wi] * lz - h[2];
                    dmin = std::min(dmin, dist);
                }
            }
        }
        return dmin == 1e9 ? 1.0 : dmin;
    }
    Eigen::Matrix<double, 7, 1> self_motion(int side) {
        int nv = m->nv;
        Eigen::Matrix<double, 6, 7> J;
        std::vector<mjtNum> jacp(3 * nv), jacr(3 * nv);
        mj_jacSite(m.get(), d.get(), jacp.data(), jacr.data(), tcp_site[side]);
        for (int j = 0; j < 7; j++) {
            int col = m->jnt_dofadr[joint[side][j]];
            for (int r = 0; r < 3; r++) {
                J(r, j) = jacp[r * nv + col];
                J(3 + r, j) = jacr[r * nv + col];
            }
        }
        Eigen::JacobiSVD<Eigen::Matrix<double, 6, 7>> svd(J, Eigen::ComputeFullV);
        return svd.matrixV().col(6);
    }
    // Range-normalized config distance (each joint scaled by 1/range so mixed-range
    // arms compare evenly). Used for loop-closure detection and nearest-to-home seed
    // selection. Joints here are all limited (RotJoint), so no angle wrapping.
    double distW(int side, const Q &a, const Q &b) {
        double d = 0.0;
        for (int j = 0; j < 7; j++) {
            double r = upper[side](j) - lower[side](j);
            double w = r > 1e-9 ? 1.0 / r : 1.0;
            double e = (a(j) - b(j)) * w;
            d += e * e;
        }
        return std::sqrt(d);
    }
    // Joint-limit margin of one arm config (min over its 7 joints).
    double arm_margin(int side, const Q &q) {
        double m = 1e9;
        for (int j = 0; j < 7; j++)
            m = std::min(m, std::min(q(j) - lower[side](j), upper[side](j) - q(j)));
        return m;
    }
    double task_error() {
        double e = 0;
        for (int side = 0; side < 2; side++) {
            double *hp = d->site_xpos + 3 * handle_site[side];
            double *tp = d->site_xpos + 3 * tcp_site[side];
            double dx = tp[0] - hp[0], dy = tp[1] - hp[1], dz = tp[2] - hp[2];
            e = std::max(e, std::sqrt(dx * dx + dy * dy + dz * dz));
        }
        return e;
    }
    double wall_gap_inner() {
        int a = mj_name2id(m.get(), mjOBJ_BODY, "aviator_wall_left");
        int b = mj_name2id(m.get(), mjOBJ_BODY, "aviator_wall_right");
        if (a < 0 || b < 0)
            return -1;
        mjtNum delta[3];
        mju_sub3(delta, d->xpos + 3 * a, d->xpos + 3 * b);
        return mju_norm3(delta) - wall_thickness;  // centre distance minus thickness = inner clearance
    }
    // Step 4: verify the handle-axis rotation φ is physically legal — IK-reachable and
    // collision-free over its full 2π range at representative wheel poses. The grasp is a
    // cylinder-on-cylinder, so φ is a pure spin with no slip by construction; this checks the
    // flange/tool-mount swept volume for self-collision.
    void phi_sweep(const fs::path &outdir) {
        std::ofstream out(outdir / "phi_sweep.csv");
        out << "theta,s,side,phi,ik_ok,coll,clearance\n";
        out.precision(8);
        const double pts[][2] = {{0.0, 0.0}, {0.87266, 0.0}, {-0.87266, 0.0}, {0.0, -0.16}};
        const int nphi = 64;
        for (auto &p : pts) {
            double th = p[0], s = p[1];
            Q q0[2]{Q(7), Q(7)};
            for (int side = 0; side < 2; side++) {
                Q seed = seeds[side], out7(7);
                if (ik[side]->CartToJnt(seed, target(side, th, s, 0.0), out7) < 0)
                    out7 = seeds[side];
                q0[side] = out7;
            }
            for (int side = 0; side < 2; side++) {
                Q prev = q0[side];  // continuation seed: previous φ's solution (connected branch)
                for (int i = 0; i < nphi; i++) {
                    double phi = 2.0 * M_PI * i / nphi;
                    Q out7(7);
                    int ok = ik[side]->CartToJnt(prev, target(side, th, s, phi), out7);
                    bool coll = false;
                    double clr = 0.0;
                    if (ok >= 0) {
                        prev = out7;
                        Q ql = side == 0 ? out7 : q0[0], qr = side == 1 ? out7 : q0[1];
                        set_config(ql, qr, th, s);
                        coll = collision(side);
                        clr = wall_clearance(side);
                    }
                    out << th << ',' << s << ',' << side << ',' << phi << ',' << ok << ',' << coll
                        << ',' << clr << '\n';
                }
            }
        }
        out.close();
        std::cout << "Wrote " << outdir / "phi_sweep.csv" << "\n";
    }

    // Step 5: static φ ablation — d*_ρ (φ=0) vs d*_{ρ+φ} (self-motion ρ × handle-axis φ),
    // at the hardest task endpoints. This is the offline test that decides whether φ
    // carries value BEFORE any B3 upgrade. For each arm (other frozen at baseline) we
    // report the full-loop d*_ρ, the 2-D d*_{ρ+φ}, the gain, and the optimal φ.
    void phi_ablation(const fs::path &outdir) {
        std::ofstream out(outdir / "phi_ablation.csv");
        out << "task,side,d_rho,d_rho_phi,gain,phimax\n";
        out.precision(8);
        const double pts[][2] = {{-0.87266, 0.0}, {0.87266, 0.0}, {0.0, -0.16}};  // roll_neg, roll_pos, pull
        const char *tags[] = {"roll_neg", "roll_pos", "pull"};
        for (int t = 0; t < 3; t++) {
            double th = pts[t][0], s = pts[t][1];
            Q q0[2]{Q(7), Q(7)};
            bool ok = true;
            for (int side = 0; side < 2; side++) {
                Q out7(7);
                if (ik[side]->CartToJnt(seeds[side], target(side, th, s, 0.0), out7) < 0) {
                    ok = false;
                    break;
                }
                q0[side] = out7;
            }
            if (!ok)
                continue;
            for (int side = 0; side < 2; side++) {
                double d_rho = -1.0, d_rho_phi = -1.0, phimax = 0.0;
                Q qmax(7);
                self_motion_trace(side, q0[side], q0[1 - side], th, s, handle[side].M, d_rho, qmax);
                self_motion_trace_phi(side, q0[side], q0[1 - side], th, s, d_rho_phi, phimax, qmax);
                out << tags[t] << ',' << side << ',' << d_rho << ',' << d_rho_phi << ','
                    << (d_rho_phi - d_rho) << ',' << phimax << '\n';
                std::cout << tags[t] << " side " << side << ": d_rho=" << d_rho
                          << " d_rho_phi=" << d_rho_phi << " gain=" << (d_rho_phi - d_rho)
                          << " phimax=" << phimax << "\n";
            }
        }
        out.close();
        std::cout << "Wrote " << outdir / "phi_ablation.csv" << "\n";
    }

    // Constraint-release ladder (R0→R1→R2a/R2b→R3): d* at each release level, for each
    // task/arm, at the current wall gap. R0 = rigid weld (3P+3R, ρ only); R1 = +φ about the
    // grip axis; R2a = +β about the radial; R2b = +β about the steering normal; R3 =
    // position-only (orientation free). One CSV row per (task, side, level).
    void release_ablation(const fs::path &outdir) {
        std::ofstream out(outdir / "release_ablation.csv");
        out << "task,side,level,d_star,phi,beta_r,beta_n\n";
        out.precision(8);
        // Focused ladder: roll_neg is the binding task (side 0 hardest). roll_pos/pull are
        // included side-0-only at R0/R1 to check whether β flips the binding task; side 1 is
        // non-binding by ~32 mm and skipped. R3 (full 3-param orientation) runs only for the
        // single binding arm (roll_neg side 0) — it is the expensive 175-sample sweep.
        struct Spec { std::string tag; double th, s; int nsides; };
        const Spec specs[] = {
            {"roll_neg", -0.87266, 0.0, 2},
            {"roll_pos",  0.87266, 0.0, 1},
            {"pull",      0.0, -0.16, 1},
        };
        for (const auto &sp : specs) {
            Q q0[2]{Q(7), Q(7)};
            bool ok = true;
            for (int side = 0; side < 2; side++) {
                Q out7(7);
                if (ik[side]->CartToJnt(seeds[side], target(side, sp.th, sp.s, 0.0), out7) < 0) {
                    ok = false;
                    break;
                }
                q0[side] = out7;
            }
            if (!ok)
                continue;
            for (int side = 0; side < sp.nsides; side++) {
                double d, phi, br, bn;
                Q qmax(7);
                auto emit = [&](const char *lvl) {
                    out << sp.tag << ',' << side << ',' << lvl << ',' << d << ',' << phi << ',' << br << ',' << bn
                        << '\n';
                    out.flush();
                    std::cout << sp.tag << " side " << side << ' ' << lvl << " d*=" << d << "\n";
                };
                d = -1.0; phi = br = bn = 0.0;
                self_motion_trace(side, q0[side], q0[1 - side], sp.th, sp.s, handle[side].M, d, qmax);
                emit("R0");
                d = -1.0; phi = br = bn = 0.0;
                self_motion_trace_phi(side, q0[side], q0[1 - side], sp.th, sp.s, d, phi, qmax);
                emit("R1");
                d = -1.0; phi = br = bn = 0.0;
                self_motion_trace_r2(side, q0[side], q0[1 - side], sp.th, sp.s, 0, d, phi, br, qmax);
                emit("R2a");
                d = -1.0; phi = br = bn = 0.0;
                self_motion_trace_r2(side, q0[side], q0[1 - side], sp.th, sp.s, 1, d, phi, bn, qmax);
                emit("R2b");
                if (sp.tag == "roll_neg" && side == 0) {
                    d = -1.0; phi = br = bn = 0.0;
                    self_motion_trace_r3(side, q0[side], q0[1 - side], sp.th, sp.s, d, phi, br, bn, qmax);
                    emit("R3");
                }
            }
        }
        out.close();
        std::cout << "Wrote " << outdir / "release_ablation.csv" << "\n";
    }

    // Deterministic baseline IK. The home approach seed is tried FIRST: with SolveType
    // Speed the solver does local continuation (Newton from the seed, first convergence),
    // so it stays on the home self-motion branch and keeps the left/right arms on
    // mirror-consistent branches. (A global Distance search would instead land the arms on
    // whichever *disconnected* component is joint-space-closest to home, breaking the
    // dL(θ,s)≈dR(-θ,s) symmetry with a very different max clearance.) Only when home fails
    // — a false negative at 1° grid resolution — fall back to a deterministic bank and
    // keep the solution closest to home. Each call is seeded via setSeed() (reproducible).
    bool ik_multi_seed(int side, double theta, double s, const Q &ref, unsigned int seed_base, Q &out) {
        Q q(7);
        ik[side]->setSeed(seed_base);
        if (ik[side]->CartToJnt(seeds[side], target(side, theta, s), q) >= 0) {
            out = q;
            return true;
        }
        Q mid(7);
        for (int j = 0; j < 7; j++)
            mid(j) = 0.5 * (lower[side](j) + upper[side](j));
        Q bank[4]{Q(7), Q(7), Q(7), Q(7)};
        bank[0] = mid;
        for (int j = 0; j < 7; j++) {
            bank[1](j) = lower[side](j) + 0.25 * (upper[side](j) - lower[side](j));
            bank[2](j) = lower[side](j) + 0.75 * (upper[side](j) - lower[side](j));
            bank[3](j) = 2.0 * mid(j) - seeds[side](j);
        }
        Q best;
        double best_d = 1e18;
        bool any = false;
        for (int t = 0; t < 4; t++) {
            Q seed(7);
            for (int j = 0; j < 7; j++)
                seed(j) = std::clamp(bank[t](j), lower[side](j), upper[side](j));
            ik[side]->setSeed(seed_base + 1u + (unsigned int)t);  // per-trial deterministic stream
            Q qq(7);
            if (ik[side]->CartToJnt(seed, target(side, theta, s), qq) >= 0) {
                double d = distW(side, qq, ref);
                if (!any || d < best_d) {
                    any = true;
                    best_d = d;
                    best = qq;
                }
            }
        }
        if (!any)
            return false;
        out = best;
        return true;
    }

    // Walk the 1-D self-motion loop of one arm (unit-nullspace continuation with
    // re-projection) until it closes back on q0 or the arc terminates at a joint limit,
    // invoking `visit(q)` at every valid manifold point. Returns true once a closed
    // orbit has been traversed (the opposite direction is then redundant and skipped).
    // This replaces the old +-trace_half segment, which under-traced the ~2pi elbow
    // orbit and could miss the global clearance max near the geometric boundary.
    bool walk_loop(int side, const Q &q0, const Q &other0, double theta, double s, const KDL::Rotation &Rh,
                   const std::function<void(const Q &)> &visit) {
        visit(q0);
        const int min_steps = std::max(10, int(1.5 / trace_step));  // don't "close" before one full arc
        const int max_steps = int(4.0 * M_PI / trace_step) + 4;     // generous cap (~2 orbits)
        for (int dir = 0; dir < 2; dir++) {
            double sgn = dir ? -1.0 : 1.0;
            Q cur = q0, out(7);
            bool closed = false;
            for (int k = 1; k <= max_steps; k++) {
                set_config(side ? other0 : cur, side ? cur : other0, theta, s);
                auto n = self_motion(side);
                Q seed(7);
                for (int j = 0; j < 7; j++)
                    seed(j) = std::clamp(cur(j) + sgn * trace_step * n(j), lower[side](j), upper[side](j));
                if (ik[side]->CartToJnt(seed, target_Rh(side, theta, s, Rh), out) < 0)
                    break;  // manifold ends here (joint limit / singular branch)
                visit(out);
                double dev = 0;
                for (int j = 0; j < 7; j++)
                    dev = std::max(dev, std::fabs(out(j) - q0(j)));
                if (k >= min_steps && dev < 0.08) {
                    closed = true;  // returned to the start: full orbit covered
                    break;
                }
                cur = out;
            }
            if (closed)
                return true;
        }
        return false;
    }

    // Sharpen the clearance peak: local resampling around the coarse-grid best point.
    void refine_peak(int side, const Q &q0, const Q &other0, double theta, double s,
                     const KDL::Rotation &Rh, double &dmax, Q &qmax) {
        for (int dir = 0; dir < 2; dir++) {
            double sgn = dir ? -1.0 : 1.0;
            Q cur = q0, out(7);
            for (int k = 1; k <= 5; k++) {
                set_config(side ? other0 : cur, side ? cur : other0, theta, s);
                auto n = self_motion(side);
                Q seed(7);
                for (int j = 0; j < 7; j++)
                    seed(j) = std::clamp(cur(j) + sgn * 0.02 * n(j), lower[side](j), upper[side](j));
                if (ik[side]->CartToJnt(seed, target_Rh(side, theta, s, Rh), out) < 0)
                    break;
                set_config(side ? other0 : out, side ? out : other0, theta, s);
                double dd = wall_clearance(side);
                if (dd > dmax) {
                    dmax = dd;
                    qmax = out;
                }
                cur = out;
            }
        }
    }

    // Walk ONE direction (sgn = ±1) of the 1-D self-motion component from q0, invoking
    // visit(q) at each on-manifold point, until the loop closes back on q0, the arc hits
    // a joint limit, IK fails, or the step cap is reached. Reports stop reason + count.
    // Both directions are always traced (unlike the early-return walk_loop) so that
    // d* = max(d*_+, d*_-) is well-defined.
    StopReason walk_dir(int side, const Q &q0, const Q &other0, double theta, double s,
                        const KDL::Rotation &Rh, double sgn, int &n_steps, double &closure_error,
                        const std::function<void(const Q &)> &visit) {
        constexpr double eps_q = 0.12;      // ||q_k - q0||_∞ closure tolerance (rad)
        constexpr double delta_away = 0.25; // must have left the start meaningfully (rad)
        const int min_steps = std::max(10, int(1.5 / trace_step));
        const int max_steps = int(4.0 * M_PI / trace_step) + 4;
        Q cur = q0, out(7);
        double max_dev = 0.0;
        closure_error = 1e9;
        for (int k = 1; k <= max_steps; k++) {
            set_config(side ? other0 : cur, side ? cur : other0, theta, s);
            auto n = self_motion(side);
            Q seed(7);
            bool clamped = false;
            for (int j = 0; j < 7; j++) {
                double v = cur(j) + sgn * trace_step * n(j);
                if (v < lower[side](j) || v > upper[side](j)) {
                    v = std::clamp(v, lower[side](j), upper[side](j));
                    clamped = true;
                }
                seed(j) = v;
            }
            if (ik[side]->CartToJnt(seed, target_Rh(side, theta, s, Rh), out) < 0) {
                n_steps = k - 1;
                // Clamped seed at a limit -> genuine manifold boundary; otherwise a
                // transient numeric failure (incomplete trace).
                return clamped ? StopReason::JOINT_LIMIT : StopReason::IK_FAIL;
            }
            visit(out);
            double dev = 0.0;
            for (int j = 0; j < 7; j++)
                dev = std::max(dev, std::fabs(out(j) - q0(j)));  // raw max-abs, rad
            max_dev = std::max(max_dev, dev);
            if (k >= min_steps && dev < eps_q && max_dev > delta_away) {
                closure_error = dev;
                n_steps = k;
                return StopReason::LOOP_CLOSED;
            }
            if (clamped) {
                // Clamped step that re-projected with no meaningful progress: the manifold
                // has reached a genuine joint-limit boundary (stall), not a transient clamp.
                // Terminate as JOINT_LIMIT instead of walking to MAX_STEPS.
                double prog = 0.0;
                for (int j = 0; j < 7; j++)
                    prog = std::max(prog, std::fabs(out(j) - cur(j)));
                if (prog < 0.02) {
                    n_steps = k;
                    return StopReason::JOINT_LIMIT;
                }
            }
            cur = out;
        }
        n_steps = max_steps;
        return StopReason::MAX_STEPS;
    }

    // B1 diagnostics: trace the FULL bidirectional self-motion component of one arm
    // (other arm frozen), returning d*_static plus coverage state (per-direction stop
    // reasons, closure error, joint margin at the argmax).
    void self_motion_trace_ex(int side, const Q &q0, const Q &other0, double theta, double s,
                              const KDL::Rotation &Rh, TraceResult &tr) {
        tr = TraceResult{};
        tr.q_max = q0;
        auto visit = [&](const Q &q) {
            set_config(side ? other0 : q, side ? q : other0, theta, s);
            double dd = wall_clearance(side);
            if (dd > tr.d_max) {
                tr.d_max = dd;
                tr.q_max = q;
            }
            // safe+margin tracking: keep the config with the largest joint margin among those
            // that satisfy d>=d_safe AND margin>=m_q (the margin-respecting safe section).
            if (dd >= d_safe) {
                double m = arm_margin(side, q);
                if (m >= q_margin_target && m > tr.m_safe) {
                    tr.has_safe = true;
                    tr.m_safe = m;
                    tr.d_safe_at = dd;
                    tr.q_safe = q;
                }
            }
        };
        visit(q0);  // the baseline config belongs to the component
        double ce_pos = 1e9, ce_neg = 1e9;
        tr.stop_pos = walk_dir(side, q0, other0, theta, s, Rh, +1.0, tr.n_steps_pos, ce_pos, visit);
        tr.stop_neg = walk_dir(side, q0, other0, theta, s, Rh, -1.0, tr.n_steps_neg, ce_neg, visit);
        tr.closure_error = std::min(ce_pos, ce_neg);
        // Full component covered iff the orbit closed OR both ends terminated at a real
        // joint-limit boundary (an open interval [ρ_min, ρ_max] is also a complete trace).
        tr.component_complete =
            tr.stop_pos == StopReason::LOOP_CLOSED || tr.stop_neg == StopReason::LOOP_CLOSED ||
            (tr.stop_pos == StopReason::JOINT_LIMIT && tr.stop_neg == StopReason::JOINT_LIMIT);
        refine_peak(side, tr.q_max, other0, theta, s, Rh, tr.d_max, tr.q_max);
        tr.q_margin_at_dmax = arm_margin(side, tr.q_max);
    }

    // B1: max wall clearance over the FULL self-motion loop of one arm (d*_static).
    bool self_motion_trace(int side, const Q &q0, const Q &other0, double theta, double s,
                           const KDL::Rotation &Rh, double &dmax, Q &qmax) {
        TraceResult tr;
        self_motion_trace_ex(side, q0, other0, theta, s, Rh, tr);
        dmax = tr.d_max;
        qmax = tr.q_max;
        return tr.d_max > -0.5;
    }

    // Step 5: static φ ablation — d*_{ρ+φ} = max clearance over the 2-D null space
    // (self-motion ρ × handle-axis rotation φ). Traces the full ρ loop at each reachable φ
    // (continuation from the φ=0 baseline branch) and keeps the global max, to compare
    // against d*_ρ = self_motion_trace(φ=0). phimax is the φ that attains the max.
    bool self_motion_trace_phi(int side, const Q &q0, const Q &other0, double theta, double s,
                               double &dmax, double &phimax, Q &qmax) {
        dmax = -1.0;
        phimax = 0.0;
        qmax = q0;
        if (!self_motion_trace(side, q0, other0, theta, s, handle[side].M, dmax, qmax))
            return false;
        const double phi_hi = 1.0;   // generous bound; reachability pruned by IK
        const int nphi = 9;          // φ ∈ {-phi_hi … +phi_hi}, φ=0 skipped (already covered)
        for (int i = 0; i < nphi; i++) {
            double phi = phi_hi * (2.0 * i / (nphi - 1.0) - 1.0);
            if (std::fabs(phi) < 1e-6)
                continue;
            Q qphi(7);
            if (ik[side]->CartToJnt(q0, target(side, theta, s, phi), qphi) < 0)
                continue;  // φ not reachable from the baseline branch
            double dphi = -1.0;
            Q qphi_best = qphi;
            self_motion_trace(side, qphi, other0, theta, s, handle_rot(side, phi, 0.0, 0), dphi, qphi_best);
            if (dphi > dmax) {
                dmax = dphi;
                phimax = phi;
                qmax = qphi_best;
            }
        }
        return dmax > -0.5;
    }

    // R2: max clearance over the 3-D null space (ρ × φ × β), where β tilts the grasp about
    // the radial (beta_axis=0) or steering normal (beta_axis=1). This is a KINEMATIC UPPER
    // BOUND — β breaks the coaxial weld (models palm-tilt), so it is not a rigid grasp.
    // Sweeps the (φ, β) grid with continuation seeding, tracing the full ρ loop at each
    // reachable orientation. betamax is the β that attains the max.
    bool self_motion_trace_r2(int side, const Q &q0, const Q &other0, double theta, double s, int beta_axis,
                              double &dmax, double &phimax, double &betamax, Q &qmax) {
        dmax = -1.0;
        phimax = 0.0;
        betamax = 0.0;
        qmax = q0;
        if (!self_motion_trace(side, q0, other0, theta, s, handle[side].M, dmax, qmax))
            return false;
        const double hi = 1.0;   // ±1 rad orientation range; reachability pruned by IK
        const int n = 7;         // 7×7 φ-β grid
        Q qseed = q0;
        for (int i = 0; i < n; i++) {
            double phi = hi * (2.0 * i / (n - 1.0) - 1.0);
            for (int j = 0; j < n; j++) {
                double beta = hi * (2.0 * j / (n - 1.0) - 1.0);
                if (std::fabs(phi) < 1e-6 && std::fabs(beta) < 1e-6)
                    continue;
                Q qo(7);
                if (ik[side]->CartToJnt(qseed, target(side, theta, s, phi, beta, beta_axis), qo) < 0 &&
                    ik[side]->CartToJnt(q0, target(side, theta, s, phi, beta, beta_axis), qo) < 0)
                    continue;  // orientation unreachable from the connected branch
                qseed = qo;
                double dd = -1.0;
                Q qbest = qo;
                self_motion_trace(side, qo, other0, theta, s, handle_rot(side, phi, beta, beta_axis), dd, qbest);
                if (dd > dmax) {
                    dmax = dd;
                    phimax = phi;
                    betamax = beta;
                    qmax = qbest;
                }
            }
        }
        return dmax > -0.5;
    }

    // R3: position-only upper bound — orientation free up to joint-limit reachability.
    // Full 3-param orientation family Rot(t,φ)·Rot(r,βr)·Rot(n,βn), coarse grid.
    bool self_motion_trace_r3(int side, const Q &q0, const Q &other0, double theta, double s,
                              double &dmax, double &phimax, double &brmax, double &bnmax, Q &qmax) {
        dmax = -1.0;
        phimax = brmax = bnmax = 0.0;
        qmax = q0;
        if (!self_motion_trace(side, q0, other0, theta, s, handle[side].M, dmax, qmax))
            return false;
        const double hi = 1.0;
        const int nphi = 7, nb = 5;   // 7×5×5 = 175 orientation samples
        Q qseed = q0;
        for (int i = 0; i < nphi; i++) {
            double phi = hi * (2.0 * i / (nphi - 1.0) - 1.0);
            for (int j = 0; j < nb; j++) {
                double br = hi * (2.0 * j / (nb - 1.0) - 1.0);
                for (int k = 0; k < nb; k++) {
                    double bn = hi * (2.0 * k / (nb - 1.0) - 1.0);
                    if (std::fabs(phi) < 1e-6 && std::fabs(br) < 1e-6 && std::fabs(bn) < 1e-6)
                        continue;
                    KDL::Rotation Rh = KDL::Rotation::Rot(handle_axis[side], phi) *
                                       KDL::Rotation::Rot(handle_radial[side], br) *
                                       KDL::Rotation::Rot(wheel_normal, bn) * handle[side].M;
                    KDL::Frame tgt = target_Rh(side, theta, s, Rh);
                    Q qo(7);
                    if (ik[side]->CartToJnt(qseed, tgt, qo) < 0 && ik[side]->CartToJnt(q0, tgt, qo) < 0)
                        continue;
                    qseed = qo;
                    double dd = -1.0;
                    Q qbest = qo;
                    self_motion_trace(side, qo, other0, theta, s, Rh, dd, qbest);
                    if (dd > dmax) {
                        dmax = dd;
                        phimax = phi;
                        brmax = br;
                        bnmax = bn;
                        qmax = qbest;
                    }
                }
            }
        }
        return dmax > -0.5;
    }

    // Safe pre-shaped start: the self-motion config at x_0 with d >= d_safe that is
    // CLOSEST to q_home (least reshaping), falling back to max-d if none reaches d_safe.
    bool safe_start(int side, const Q &q_home, const Q &other_home, double theta, double s, Q &q_out, double &d_out) {
        Q best = q_home;
        double best_d = -1.0, best_dev = 1e18;
        bool found_safe = false;
        walk_loop(side, q_home, other_home, theta, s, handle[side].M, [&](const Q &q) {
            set_config(side ? other_home : q, side ? q : other_home, theta, s);
            if (collision(side))
                return;
            double dd = wall_clearance(side);
            double dev = 0;
            for (int j = 0; j < 7; j++) {
                double e = q(j) - q_home(j);
                dev += e * e;
            }
            if (dd >= d_safe - 1e-6) {
                if (!found_safe || dev < best_dev) {
                    found_safe = true;
                    best_dev = dev;
                    best = q;
                    best_d = dd;
                }
            } else if (!found_safe && dd > best_d) {
                best_d = dd;
                best = q;
            }
        });
        q_out = best;
        set_config(side ? other_home : q_out, side ? q_out : other_home, theta, s);
        d_out = wall_clearance(side);
        return found_safe;
    }

    // B3 least-intervention solve for one arm. Returns true iff clearance >= d_safe.
    // active: 0 none, 1 clearance-bound, 2 dynamic-bound (failed, not joint), 3 joint-limit-bound.
    bool solve_b3(int side, const Q &q_base, const Q &other_cur, const Eigen::Matrix<double, 7, 1> &n,
                  const Q &q_prev, const Q &delta_prev, double theta, double s, Q &q_out, double &d_out,
                  double &alpha_out, int &active) {
        const double dt = 0.02;
        double a_lo = -1e9, a_hi = 1e9;
        bool lin_ok = true;
        auto tighten = [&](double lo, double hi) {
            a_lo = std::max(a_lo, lo);
            a_hi = std::min(a_hi, hi);
        };
        for (int j = 0; j < 7; j++) {
            double dqb = q_base(j) - q_prev(j);
            double nj = n(j);
            if (std::fabs(nj) < 1e-9) {
                if (std::fabs(dqb) > qdot_max[side][j] * dt)
                    lin_ok = false;
                if (std::fabs(dqb - delta_prev(j)) > qddot_max * dt * dt)
                    lin_ok = false;
                continue;
            }
            double v = qdot_max[side][j] * dt;
            double hi = (v - dqb) / nj, lo = (-v - dqb) / nj;
            nj > 0 ? tighten(lo, hi) : tighten(hi, lo);
            double a = qddot_max * dt * dt;
            hi = (a + delta_prev(j) - dqb) / nj;
            lo = (-a + delta_prev(j) - dqb) / nj;
            nj > 0 ? tighten(lo, hi) : tighten(hi, lo);
            double lo_q = (lower[side](j) + q_margin_target - q_base(j)) / nj;
            double hi_q = (upper[side](j) - q_margin_target - q_base(j)) / nj;
            nj > 0 ? tighten(lo_q, hi_q) : tighten(hi_q, lo_q);
        }
        if (!lin_ok || a_lo > a_hi) {
            a_lo = std::min(a_lo, 0.0);
            a_hi = std::max(a_hi, 0.0);
            if (a_lo > a_hi)
                a_lo = a_hi = 0.0;
        }

        auto eval = [&](double al, double &dd, bool &coll) {
            Q cand(7);
            for (int j = 0; j < 7; j++)
                cand(j) = std::clamp(q_base(j) + al * n(j), lower[side](j), upper[side](j));
            set_config(side ? other_cur : cand, side ? cand : other_cur, theta, s);
            coll = collision(side);
            dd = wall_clearance(side);
        };
        double best_alpha = 0.0, best_d = -1e9, best_abs = 1e18;
        bool feasible = false;
        double d0;
        bool c0;
        eval(0.0, d0, c0);
        if (!c0 && d0 >= d_safe) {
            feasible = true;
            best_alpha = 0.0;
            best_d = d0;
        } else {
            if (!c0 && d0 > best_d)
                best_d = d0;
            for (int i = 1; i <= alpha_grid; i++) {
                double frac = double(i) / alpha_grid;
                for (int sg = 0; sg < 2; sg++) {
                    double al = sg ? a_lo + (0.0 - a_lo) * frac : a_hi * frac;
                    double dd;
                    bool coll;
                    eval(al, dd, coll);
                    if (coll)
                        continue;
                    if (dd >= d_safe) {
                        if (std::fabs(al) < best_abs) {
                            best_abs = std::fabs(al);
                            best_alpha = al;
                            best_d = dd;
                            feasible = true;
                        }
                    } else if (!feasible && dd > best_d) {
                        best_d = dd;
                        best_alpha = al;
                    }
                }
            }
        }

        Q cand(7), out(7);
        for (int j = 0; j < 7; j++)
            cand(j) = std::clamp(q_base(j) + best_alpha * n(j), lower[side](j), upper[side](j));
        if (ik[side]->CartToJnt(cand, target(side, theta, s), out) >= 0) {
            set_config(side ? other_cur : out, side ? out : other_cur, theta, s);
            if (!collision(side) && wall_clearance(side) >= d_safe - 1e-6)
                cand = out;
        }
        set_config(side ? other_cur : cand, side ? cand : other_cur, theta, s);
        d_out = wall_clearance(side);
        q_out = cand;
        alpha_out = best_alpha;

        // classify the active constraint
        bool safe = !collision(side) && d_out >= d_safe - 1e-6;
        if (safe) {
            active = (std::fabs(best_alpha) < 1e-9) ? 0 : 1;
        } else {
            double jslack = 1e9;
            for (int j = 0; j < 7; j++)
                jslack = std::min({jslack, q_out(j) - lower[side](j), upper[side](j) - q_out(j)});
            active = (jslack <= q_margin_target + 1e-3) ? 3 : 2;
        }
        return safe;
    }

    // B4 predictive solve for one arm. Same rate/accel/joint-limit box as solve_b3, but the
    // reshaping TRIGGER is a short-horizon lookahead: predict the wheel θ_{k+h}=θ+hΔtθ̇ and the
    // arm's baseline clearance there (continuation IK from q_base, no reshaping). If the minimum
    // predicted clearance over the horizon dips below d_safe, reshape NOW toward the max-clearance
    // self-motion branch (rate-constrained) so the arm has time to reach the safe branch before
    // the tight pose arrives — the fix for B3's "reactive too late" failure at high f.
    bool solve_b4(int side, const Q &q_base, const Q &other_cur, const Eigen::Matrix<double, 7, 1> &n,
                  const Q &q_prev, const Q &delta_prev, double theta, double s, double thd, double sd,
                  int H, Q &q_out, double &d_out, double &alpha_out, int &active) {
        double a_lo = -1e9, a_hi = 1e9;
        bool lin_ok = true;
        auto tighten = [&](double lo, double hi) {
            a_lo = std::max(a_lo, lo);
            a_hi = std::min(a_hi, hi);
        };
        for (int j = 0; j < 7; j++) {
            double dqb = q_base(j) - q_prev(j);
            double nj = n(j);
            if (std::fabs(nj) < 1e-9) {
                if (std::fabs(dqb) > qdot_max[side][j] * dt)
                    lin_ok = false;
                if (std::fabs(dqb - delta_prev(j)) > qddot_max * dt * dt)
                    lin_ok = false;
                continue;
            }
            double v = qdot_max[side][j] * dt;
            double hi = (v - dqb) / nj, lo = (-v - dqb) / nj;
            nj > 0 ? tighten(lo, hi) : tighten(hi, lo);
            double a = qddot_max * dt * dt;
            hi = (a + delta_prev(j) - dqb) / nj;
            lo = (-a + delta_prev(j) - dqb) / nj;
            nj > 0 ? tighten(lo, hi) : tighten(hi, lo);
            double lo_q = (lower[side](j) + q_margin_target - q_base(j)) / nj;
            double hi_q = (upper[side](j) - q_margin_target - q_base(j)) / nj;
            nj > 0 ? tighten(lo_q, hi_q) : tighten(hi_q, lo_q);
        }
        if (!lin_ok || a_lo > a_hi) {
            a_lo = std::min(a_lo, 0.0);
            a_hi = std::max(a_hi, 0.0);
            if (a_lo > a_hi)
                a_lo = a_hi = 0.0;
        }

        // Predict the future baseline clearance over the horizon (constant-velocity wheel).
        double d_future = 1e9;
        Q qh(7);
        for (int h = 1; h <= H; h++) {
            double thh = theta + h * dt * thd, sh = s + h * dt * sd;
            if (ik[side]->CartToJnt(q_base, target(side, thh, sh), qh) < 0)
                continue;
            set_config(side ? other_cur : qh, side ? qh : other_cur, thh, sh);
            d_future = std::min(d_future, wall_clearance(side));
        }
        set_config(side ? other_cur : q_base, side ? q_base : other_cur, theta, s);

        auto eval = [&](double al, double &dd, bool &coll) {
            Q cand(7);
            for (int j = 0; j < 7; j++)
                cand(j) = std::clamp(q_base(j) + al * n(j), lower[side](j), upper[side](j));
            set_config(side ? other_cur : cand, side ? cand : other_cur, theta, s);
            coll = collision(side);
            dd = wall_clearance(side);
        };

        double d0;
        bool c0;
        eval(0.0, d0, c0);

        // If the future is safe, stay (least intervention). Otherwise reshape toward max
        // current clearance (build headroom so the continuation follows the high branch).
        double best_alpha = 0.0, best_d = d0, best_abs = 1e18;
        bool any = false;
        if (d_future >= d_safe && !c0) {
            best_alpha = 0.0;
            best_d = d0;
        } else {
            if (!c0)
                best_d = d0;
            for (int i = 1; i <= alpha_grid; i++) {
                double frac = double(i) / alpha_grid;
                for (int sg = 0; sg < 2; sg++) {
                    double al = sg ? a_lo + (0.0 - a_lo) * frac : a_hi * frac;
                    double dd;
                    bool coll;
                    eval(al, dd, coll);
                    if (coll)
                        continue;
                    any = true;
                    // max-d objective (build headroom), tie-break on least |α|
                    if (dd > best_d + 1e-9 || (std::fabs(dd - best_d) <= 1e-9 && std::fabs(al) < best_abs)) {
                        best_d = dd;
                        best_alpha = al;
                        best_abs = std::fabs(al);
                    }
                }
            }
        }

        Q cand(7), out(7);
        for (int j = 0; j < 7; j++)
            cand(j) = std::clamp(q_base(j) + best_alpha * n(j), lower[side](j), upper[side](j));
        if (ik[side]->CartToJnt(cand, target(side, theta, s), out) >= 0) {
            set_config(side ? other_cur : out, side ? out : other_cur, theta, s);
            if (!collision(side) && wall_clearance(side) >= d_safe - 1e-6)
                cand = out;
        }
        set_config(side ? other_cur : cand, side ? cand : other_cur, theta, s);
        d_out = wall_clearance(side);
        q_out = cand;
        alpha_out = best_alpha;

        bool safe = !collision(side) && d_out >= d_safe - 1e-6;
        if (safe) {
            active = (std::fabs(best_alpha) < 1e-9) ? 0 : 1;
        } else {
            double jslack = 1e9;
            for (int j = 0; j < 7; j++)
                jslack = std::min({jslack, q_out(j) - lower[side](j), upper[side](j) - q_out(j)});
            active = (jslack <= q_margin_target + 1e-3) ? 3 : 2;
        }
        return safe;
    }

    // =====================================================================
    // ONLINE phase: real-time centralized bimanual redundancy reshaping.
    // Each control cycle reads ONLY the current state (x_t, ẋ_t, q_t, q̇_t, d_t)
    // and a pure-time command profile (no future-trajectory lookahead). The
    // task-tracking constraint is velocity-level (priority 1); the 2-D redundancy
    // z=(α_L, α_R) is reshaped in the nullspace (priority 2) to keep clearance.
    // =====================================================================

    Eigen::Matrix<double, 6, 7> arm_jac(int side) {
        int nv = m->nv;
        Eigen::Matrix<double, 6, 7> J;
        std::vector<mjtNum> jacp(3 * nv), jacr(3 * nv);
        mj_jacSite(m.get(), d.get(), jacp.data(), jacr.data(), tcp_site[side]);
        for (int j = 0; j < 7; j++) {
            int col = m->jnt_dofadr[joint[side][j]];
            for (int r = 0; r < 3; r++) {
                J(r, j) = jacp[r * nv + col];
                J(3 + r, j) = jacr[r * nv + col];
            }
        }
        return J;
    }
    KDL::Vector site_pos(int site) {
        const double *p = d->site_xpos + 3 * site;
        return {p[0], p[1], p[2]};
    }
    KDL::Rotation site_rot(int site) {
        const double *r = d->site_xmat + 9 * site;
        return KDL::Rotation(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]);
    }
    Q get_q(int side) {
        Q q(7);
        for (int j = 0; j < 7; j++)
            q(j) = d->qpos[m->jnt_qposadr[joint[side][j]]];
        return q;
    }

    // One fixed T5 Newton projection step: q ← clamp(q + J^# e, [q_min+m_q, q_max−m_q]).
    // e = grasp-closure error (handle site − TCP site, position + axis-angle orientation) in
    // the task_error() convention, J = 6×7 TCP-site Jacobian (mj_jacSite), solved min-norm via
    // completeOrthogonalDecomposition. Δq = J^# e drives the TCP onto the handle (NOT −e).
    void manifold_newton_step(const Q &ql, const Q &qr, double th, double s, Q &ql_out, Q &qr_out) {
        set_config(ql, qr, th, s);
        Eigen::Matrix<double, 6, 1> eL, eR;
        {
            KDL::Vector pc = site_pos(tcp_site[0]);
            KDL::Rotation Rc = site_rot(tcp_site[0]);
            KDL::Vector hp = site_pos(handle_site[0]);
            KDL::Rotation Rh = site_rot(handle_site[0]);
            KDL::Vector pe = hp - pc, oe = (Rh * Rc.Inverse()).GetRot();
            eL << pe.x(), pe.y(), pe.z(), oe.x(), oe.y(), oe.z();
            pc = site_pos(tcp_site[1]);
            Rc = site_rot(tcp_site[1]);
            hp = site_pos(handle_site[1]);
            Rh = site_rot(handle_site[1]);
            pe = hp - pc;
            oe = (Rh * Rc.Inverse()).GetRot();
            eR << pe.x(), pe.y(), pe.z(), oe.x(), oe.y(), oe.z();
        }
        Eigen::Matrix<double, 6, 7> JL = arm_jac(0), JR = arm_jac(1);
        // Damped least-squares (Levenberg–Marquardt) step Δq = Jᵀ(JJᵀ+λ²I)⁻¹ e. λ bounds the
        // step near arm singularities so the projection stays a LOCAL correction (robustness; the
        // actual divergence in the first T5 cut was the joint-margin clamp, fixed below with a
        // true-limit clamp).
        const double lambda = 1e-2;
        auto dls = [&](const Eigen::Matrix<double, 6, 7> &J,
                       const Eigen::Matrix<double, 6, 1> &e) -> Eigen::VectorXd {
            Eigen::Matrix<double, 6, 6> A = J * J.transpose();
            A.diagonal().array() += lambda * lambda;
            Eigen::VectorXd z = A.ldlt().solve(e);
            return J.transpose() * z;
        };
        Eigen::VectorXd dL = dls(JL, eL);
        Eigen::VectorXd dR = dls(JR, eR);
        ql_out = Q(7);
        qr_out = Q(7);
        for (int k = 0; k < 7; k++) {
            // Clamp to TRUE joint limits (not +margin). The offline section already guarantees
            // margin ≥ m_q on the certified 97.76%; clamping to [lower+m_q, upper−m_q] here would
            // snap the margin-infeasible fallback configs (q_max sits at the limit) inward and
            // blow up the grasp closure. The margin is a soft preference enforced offline, not a
            // hard online bound.
            ql_out(k) = std::clamp(ql(k) + dL(k), lower[0](k), upper[0](k));
            qr_out(k) = std::clamp(qr(k) + dR(k), lower[1](k), upper[1](k));
        }
    }

    // Desired world-frame 6-D TCP twist to track the wheel command (θ_d, s_d). The feedforward
    // is the DISCRETE target velocity (target moves from (θ_prev,s_prev) to (θ_d,s_d) in one
    // dt, since the wheel position is commanded exactly each cycle), plus proportional
    // feedback on the 6-D pose error. (An analytic-velocity feedforward double-counts the
    // exactly-set wheel and made the resolved-rate loop diverge.)
    Eigen::Matrix<double, 6, 1> task_twist(int side, double th_d, double s_d, double th_prev, double s_prev,
                                           double Kp, double Ko) {
        KDL::Frame T0 = target(side, th_d, s_d);
        KDL::Frame Tp = target(side, th_prev, s_prev);
        KDL::Vector vff = (T0.p - Tp.p) / dt;
        KDL::Vector wff = (T0.M * Tp.M.Inverse()).GetRot() / dt;
        KDL::Vector pc = site_pos(tcp_site[side]);
        KDL::Rotation Rc = site_rot(tcp_site[side]);
        KDL::Vector pe = T0.p - pc;
        KDL::Vector oe = (T0.M * Rc.Inverse()).GetRot();
        Eigen::Matrix<double, 6, 1> v;
        v << vff.x() + Kp * pe.x(), vff.y() + Kp * pe.y(), vff.z() + Kp * pe.z(), wff.x() + Ko * oe.x(),
            wff.y() + Ko * oe.y(), wff.z() + Ko * oe.z();
        return v;
    }

    // Clearance gradient J_d = ∂d/∂q via central finite difference (restores config).
    Eigen::Matrix<double, 1, 7> clearance_grad(int side, const Q &ql, const Q &qr, double theta, double s,
                                               double eps = 1e-3) {
        Eigen::Matrix<double, 1, 7> g;
        g.setZero();
        const Q &qcur = side ? qr : ql;
        for (int j = 0; j < 7; j++) {
            Q qp = qcur, qm = qcur;
            qp(j) = std::clamp(qp(j) + eps, lower[side](j), upper[side](j));
            qm(j) = std::clamp(qm(j) - eps, lower[side](j), upper[side](j));
            double den = qp(j) - qm(j);
            if (den < 1e-12) {
                g(0, j) = 0.0;
                continue;
            }
            set_config(side ? ql : qp, side ? qp : qr, theta, s);
            double dp = wall_clearance(side);
            set_config(side ? ql : qm, side ? qm : qr, theta, s);
            double dm = wall_clearance(side);
            g(0, j) = (dp - dm) / den;
        }
        set_config(ql, qr, theta, s);
        return g;
    }

    // Velocity-level least-intervention nullspace QP (one arm). q̇ = q̇_nom + α n.
    // Hard constraints: |q̇_j| ≤ q̇max, |q̇_j − q̇_prev,j| ≤ q̈max Δt, and joint-limit margin.
    // The clearance requirement d_next ≥ d_safe is enforced by DIRECTLY evaluating the true
    // point-cloud clearance at each candidate q_next = q + Δt(q̇_nom + α n) over a 1-D grid
    // (the linearized J_d constraint is too noisy at the point-cloud level; the grid is the
    // same mechanism the offline solve_b3 uses). Least |α| when feasible, max-d fallback
    // otherwise. active: 0 none, 1 clearance-bound, 2 dynamic, 3 joint-limit.
    double solve_qp_online(int side, const Q &other_cur, const Eigen::Matrix<double, 7, 1> &qdot_nom,
                           const Eigen::Matrix<double, 7, 1> &n, const Q &q_cur,
                           const Eigen::Matrix<double, 7, 1> &qdot_prev, double theta, double s,
                           int &active) {
        double a_lo = -1e9, a_hi = 1e9;
        bool rate_ok = true;
        auto tighten = [&](double lo, double hi) {
            a_lo = std::max(a_lo, lo);
            a_hi = std::min(a_hi, hi);
        };
        for (int j = 0; j < 7; j++) {
            double nj = n(j), v0 = qdot_nom(j);
            double v = qdot_max[side][j];
            if (std::fabs(nj) < 1e-9) {
                if (std::fabs(v0) > v)
                    rate_ok = false;
                if (std::fabs(v0 - qdot_prev(j)) > qddot_max * dt)
                    rate_ok = false;
            } else {
                double hi = (v - v0) / nj, lo = (-v - v0) / nj;
                nj > 0 ? tighten(lo, hi) : tighten(hi, lo);
                double a = qddot_max * dt;
                hi = (a + qdot_prev(j) - v0) / nj;
                lo = (-a + qdot_prev(j) - v0) / nj;
                nj > 0 ? tighten(lo, hi) : tighten(hi, lo);
            }
            if (std::fabs(nj) < 1e-9) {
                double qn = q_cur(j) + dt * v0;
                if (qn < lower[side](j) + q_margin_target || qn > upper[side](j) - q_margin_target)
                    rate_ok = false;
            } else {
                double lo_q = (lower[side](j) + q_margin_target - q_cur(j) - dt * v0) / (dt * nj);
                double hi_q = (upper[side](j) - q_margin_target - q_cur(j) - dt * v0) / (dt * nj);
                nj > 0 ? tighten(lo_q, hi_q) : tighten(hi_q, lo_q);
            }
        }
        if (!rate_ok || a_lo > a_hi) {
            a_lo = std::min(a_lo, 0.0);
            a_hi = std::max(a_hi, 0.0);
            if (a_lo > a_hi)
                a_lo = a_hi = 0.0;
        }

        auto eval = [&](double al, double &dd, bool &coll) {
            Q cand(7);
            for (int j = 0; j < 7; j++)
                cand(j) = std::clamp(q_cur(j) + dt * (qdot_nom(j) + al * n(j)), lower[side](j), upper[side](j));
            set_config(side ? other_cur : cand, side ? cand : other_cur, theta, s);
            coll = collision(side);
            dd = wall_clearance(side);
        };
        double best_a = 0.0, best_d = -1e9, best_abs = 1e18;
        bool any = false;
        double d0;
        bool c0;
        eval(0.0, d0, c0);
        if (!c0 && d0 >= d_safe - 1e-9) {
            best_a = 0.0;
            best_d = d0;
            best_abs = 0.0;
            any = true;
        } else if (!c0 && d0 > best_d) {
            best_d = d0;
            best_a = 0.0;
        }
        const int ng = 41;
        for (int i = 1; i <= ng; i++) {
            double frac = double(i) / ng;
            for (int sg = 0; sg < 2; sg++) {
                double al = sg ? a_lo + (0.0 - a_lo) * frac : a_hi * frac;
                if (std::fabs(al) > 1e9)
                    continue;
                double dd;
                bool coll;
                eval(al, dd, coll);
                if (coll)
                    continue;
                if (dd >= d_safe - 1e-9) {
                    if (std::fabs(al) < best_abs) {
                        best_abs = std::fabs(al);
                        best_a = al;
                        best_d = dd;
                        any = true;
                    }
                } else if (!any && dd > best_d) {
                    best_d = dd;
                    best_a = al;
                }
            }
        }

        if (any) {
            active = (std::fabs(best_a) < 1e-9) ? 0 : 1;
        } else {
            Q cand(7);
            for (int j = 0; j < 7; j++)
                cand(j) = std::clamp(q_cur(j) + dt * (qdot_nom(j) + best_a * n(j)), lower[side](j), upper[side](j));
            double jslack = 1e9;
            for (int j = 0; j < 7; j++)
                jslack = std::min({jslack, cand(j) - lower[side](j), upper[side](j) - cand(j)});
            active = (jslack <= q_margin_target + 1e-3) ? 3 : 2;
        }
        return best_a;
    }

    // Pure-time command profile (no future lookahead): (θ, s) and their derivatives.
    void command(const std::string &profile, double t, double f, double th_amp, double s_amp, double duration,
                 double &th, double &s, double &thd, double &sd) {
        th = s = thd = sd = 0.0;
        if (profile == "step") {
            double u = t / duration;
            double tri = u < 0.5 ? 2.0 * u : 2.0 * (1.0 - u);
            th = th_amp * tri;
            thd = (u < 0.5 ? 1.0 : -1.0) * 2.0 * th_amp / duration;
        } else if (profile == "sine") {
            double w = 2.0 * M_PI * f;
            th = th_amp * std::sin(w * t);
            thd = th_amp * w * std::cos(w * t);
        } else if (profile == "roll_pull") {
            double w = 2.0 * M_PI * f;
            th = th_amp * std::sin(w * t);
            // pull-only: pitch joint is limited to [−0.165, 0], so keep s ≤ 0
            s = -s_amp * 0.5 * (1.0 - std::cos(w * t));
            thd = th_amp * w * std::cos(w * t);
            sd = -0.5 * s_amp * w * std::sin(w * t);
        }
    }

    struct OnlineResult {
        std::string tag;
        double d_min = 1e9, max_dq = 0, max_ddq = 0, e_task_max = 0, J_interv = 0;
        double e_theta_rms = 0, e_s_rms = 0;
        int n_safe = 0, N = 0, n_coll = 0, rate_sat = 0;
    };

    OnlineResult online_run(const fs::path &outdir, int method, const std::string &profile, double f,
                            double th_amp, double s_amp, double duration) {
        const double Kp = 8.0, Ko = 8.0;
        const int N = std::max(2, int(std::lround(duration / dt)));
        std::string tag = std::string(method == 0 ? "b0" : (method == 2 ? "b4" : "b3")) + "_" + profile +
                          (profile != "step" ? ("_f" + fmt(f)) : "");
        std::ofstream csv(outdir / ("trajectory_online_" + tag + ".csv"));
        csv.precision(10);
        csv << "k,t,theta,theta_d,s,s_d,d_min,coll,e_task,dq,ddq,interv,active,alpha_L,alpha_R\n";

        Q ql = seeds[0], qr = seeds[1], out(7);
        double th0, s0, thd0, sd0;
        command(profile, 0.0, f, th_amp, s_amp, duration, th0, s0, thd0, sd0);
        // Land on the exact grasp manifold at θ0 first (the raw home seed is off-manifold by
        // ~76 mm; starting there produces a spurious first-step jump and would diverge the
        // resolved-rate loop), so B0 and B3/B4 share the same on-manifold baseline.
        ik[0]->CartToJnt(seeds[0], target(0, th0, s0), out);
        ql = out;
        ik[1]->CartToJnt(seeds[1], target(1, th0, s0), out);
        qr = out;
        if (method == 1 || method == 2) {  // B3/B4: additionally pre-shape to a safe config (d >= d_safe)
            double dstart;
            Q qs(7);
            safe_start(0, ql, qr, th0, s0, qs, dstart);
            ql = qs;
            safe_start(1, qr, ql, th0, s0, qs, dstart);
            qr = qs;
        }
        set_config(ql, qr, th0, s0);

        OnlineResult R;
        R.tag = tag;
        R.N = N;
        Eigen::Matrix<double, 7, 1> qdot_prevL = Eigen::Matrix<double, 7, 1>::Zero();
        Eigen::Matrix<double, 7, 1> qdot_prevR = Eigen::Matrix<double, 7, 1>::Zero();
        double th_prev = th0, s_prev = s0;
        Q dl_prev(7), dr_prev(7);
        for (int j = 0; j < 7; j++)
            dl_prev(j) = dr_prev(j) = 0.0;

        for (int k = 0; k < N; k++) {
            double t = k * dt;
            double th, s, thd, sd;
            command(profile, t, f, th_amp, s_amp, duration, th, s, thd, sd);

            Q ql_new = ql, qr_new = qr;
            double aL = 0.0, aR = 0.0;
            int acL = 0, acR = 0;

            if (method == 0) {
                // B0: one-step continuation IK from the previous config (no reshaping)
                Q out(7);
                bool ok = true;
                for (int side = 0; side < 2; side++) {
                    Q &cur = side ? qr_new : ql_new;
                    if (ik[side]->CartToJnt(cur, target(side, th, s), out) < 0 &&
                        ik[side]->CartToJnt(seeds[side], target(side, th, s), out) < 0) {
                        ok = false;
                        break;
                    }
                    cur = out;
                }
                if (!ok) {
                    ql_new = ql;
                    qr_new = qr;
                }
            } else {
                // B3 / B4: per-cycle nullspace reshaping (causal, position-level). Both compute
                // the on-manifold baseline q_base via continuation IK, then search the 1-D
                // self-motion nullspace (rate/accel/joint-limit constrained). B3 reshapes
                // least-intervention to hold d >= d_safe NOW; B4 looks ahead a short horizon and
                // reshapes EARLY toward the max-clearance branch so the arm beats the tight pose.
                // (The velocity-level formulation task_twist + solve_qp_online was abandoned: its
                // J^+ resolved-rate step is ill-conditioned near reshaped near-singular postures.)
                const int H = 10;  // prediction horizon (0.2 s)
                Q ql_base(7), qr_base(7), ql_out(7), qr_out(7), out(7);
                bool ok = true;
                for (int side = 0; side < 2; side++) {
                    Q &cur = side ? qr : ql;
                    Q &base = side ? qr_base : ql_base;
                    if (ik[side]->CartToJnt(cur, target(side, th, s), out) < 0 &&
                        ik[side]->CartToJnt(seeds[side], target(side, th, s), out) < 0) {
                        ok = false;
                        break;
                    }
                    base = out;
                }
                if (!ok) {
                    ql_new = ql;
                    qr_new = qr;
                } else {
                    double dL, dR;
                    set_config(ql_base, qr_base, th, s);
                    auto nL = self_motion(0);
                    if (method == 2)
                        solve_b4(0, ql_base, qr_base, nL, ql, dl_prev, th, s, thd, sd, H, ql_out, dL, aL, acL);
                    else
                        solve_b3(0, ql_base, qr_base, nL, ql, dl_prev, th, s, ql_out, dL, aL, acL);
                    set_config(ql_out, qr_base, th, s);
                    auto nR = self_motion(1);
                    if (method == 2)
                        solve_b4(1, qr_base, ql_out, nR, qr, dr_prev, th, s, thd, sd, H, qr_out, dR, aR, acR);
                    else
                        solve_b3(1, qr_base, ql_out, nR, qr, dr_prev, th, s, qr_out, dR, aR, acR);
                    ql_new = ql_out;
                    qr_new = qr_out;
                }
            }

            set_config(ql_new, qr_new, th, s);

            double dmin = std::min(wall_clearance(0), wall_clearance(1));
            int coll = (collision(0) || collision(1)) ? 1 : 0;
            double et = task_error();
            double mdq = 0, mddq = 0;
            Eigen::Matrix<double, 7, 1> dqL, dqR;
            for (int j = 0; j < 7; j++) {
                dqL(j) = (ql_new(j) - ql(j)) / dt;
                dqR(j) = (qr_new(j) - qr(j)) / dt;
                mdq = std::max({mdq, std::fabs(dqL(j)), std::fabs(dqR(j))});
            }
            for (int j = 0; j < 7; j++)
                mddq = std::max({mddq, std::fabs((dqL(j) - qdot_prevL(j)) / dt),
                                 std::fabs((dqR(j) - qdot_prevR(j)) / dt)});
            double interv = std::sqrt(aL * aL + aR * aR);
            int act = std::max(acL, acR);

            R.d_min = std::min(R.d_min, dmin);
            R.max_dq = std::max(R.max_dq, mdq);
            R.max_ddq = std::max(R.max_ddq, mddq);
            R.e_task_max = std::max(R.e_task_max, et);
            R.J_interv += interv * interv;
            R.n_coll += coll;
            if (dmin >= d_safe - 1e-9 && !coll)
                R.n_safe++;
            for (int j = 0; j < 7; j++)
                if (std::fabs(dqL(j)) > qdot_max[0][j] || std::fabs(dqR(j)) > qdot_max[1][j])
                    R.rate_sat = 1;
            if (mddq > qddot_max)
                R.rate_sat = 1;

            csv << k << ',' << t << ',' << th << ',' << th << ',' << s << ',' << s << ',' << dmin << ',' << coll
                << ',' << et << ',' << mdq << ',' << mddq << ',' << interv << ',' << act << ',' << aL << ',' << aR
                << '\n';

            for (int j = 0; j < 7; j++) {
                dl_prev(j) = ql_new(j) - ql(j);
                dr_prev(j) = qr_new(j) - qr(j);
            }
            ql = ql_new;
            qr = qr_new;
            qdot_prevL = dqL;
            qdot_prevR = dqR;
            th_prev = th;
            s_prev = s;
        }
        csv.close();
        R.J_interv = std::sqrt(R.J_interv);
        R.e_theta_rms = R.e_s_rms = 0.0;  // wheel is driven exactly (kinematic command)
        return R;
    }

    void online(int method, const fs::path &outdir) {
        struct P {
            std::string name;
            std::vector<double> fs;
            double th_amp, s_amp;
        };
        const P profiles[] = {
            {"step", {0.0}, 0.87266, 0.0},
            {"sine", {0.1, 0.2, 0.3, 0.5, 0.7, 1.0}, 0.87266, 0.0},
            {"roll_pull", {0.1, 0.2, 0.3, 0.5, 0.7, 1.0}, 0.6, 0.08},
        };
        const char *mname = method == 0 ? "b0" : (method == 2 ? "b4" : "b3");
        std::ofstream sum(outdir / (std::string("online_summary_") + mname + ".csv"));
        sum.precision(10);
        sum << "method,profile,f,duration,d_min,max_dq,max_ddq,e_task_max,J_interv,n_safe,N,n_coll,rate_sat,"
               "safe_frac\n";
        for (const auto &p : profiles) {
            for (double f : p.fs) {
                double duration = p.name == "step" ? 6.0 : std::max(6.0, 3.0 / f);
                auto R = online_run(outdir, method, p.name, f, p.th_amp, p.s_amp, duration);
                double safe_frac = R.N ? double(R.n_safe) / R.N : 0.0;
                std::cout << "  " << R.tag << ": d_min=" << R.d_min << " safe=" << R.n_safe << "/" << R.N
                          << " (" << safe_frac << ") max_dq=" << R.max_dq << " max_ddq=" << R.max_ddq
                          << " rate_sat=" << R.rate_sat << " J_interv=" << R.J_interv << "\n";
                sum << mname << ',' << p.name << ',' << (p.name == "step" ? 0.0 : f) << ',' << duration << ','
                    << R.d_min << ',' << R.max_dq << ',' << R.max_ddq << ',' << R.e_task_max << ',' << R.J_interv
                    << ',' << R.n_safe << ',' << R.N << ',' << R.n_coll << ',' << R.rate_sat << ',' << safe_frac
                    << '\n';
            }
        }
        sum.close();
    }

    // Diagnostic: validate the online clearance linearization J_d·n against a direct
    // nullspace step, at a mid-roll config where the continuation branch has d < d_safe.
    void debug_qp(const fs::path &outdir) {
        const double th = 0.5, s = 0.0;
        Q ql = seeds[0], qr = seeds[1], out(7);
        for (int side = 0; side < 2; side++) {
            Q &cur = side ? qr : ql;
            if (ik[side]->CartToJnt(cur, target(side, th, s), out) < 0)
                out = cur;
            cur = out;
        }
        set_config(ql, qr, th, s);
        std::cout << "--- debug_qp @ th=" << th << " ---\n";
        for (int side = 0; side < 2; side++) {
            double dcur = wall_clearance(side);
            auto n = self_motion(side);
            auto Jd = clearance_grad(side, ql, qr, th, s);
            double dn = 0;
            for (int j = 0; j < 7; j++)
                dn += Jd(0, j) * n(j);
            // direct nullspace step (no IK re-projection: raw q + 0.05·n)
            Q qraw(7);
            for (int j = 0; j < 7; j++)
                qraw(j) = std::clamp(ql(j) + 0.05 * n(j), lower[side](j), upper[side](j));
            set_config(side ? ql : qraw, side ? qraw : qr, th, s);
            double draw = wall_clearance(side);
            // reprojected nullspace step (on-manifold)
            Q qo(7);
            int ikok = ik[side]->CartToJnt(qraw, target(side, th, s), qo);
            set_config(side ? ql : qo, side ? qo : qr, th, s);
            double dproj = wall_clearance(side);
            std::cout << "side " << side << ": d_cur=" << dcur << "  Jd·n=" << dn
                      << "  raw +0.05n d=" << draw << " (Δ=" << (draw - dcur) << ")"
                      << "  proj +0.05n d=" << dproj << " (Δ=" << (dproj - dcur) << ", ik=" << ikok << ")\n";
            set_config(ql, qr, th, s);
        }
    }

    TaskSummary run_task(const fs::path &outdir, const std::string &tag, double th0, double s0, double th1,
                         double s1, double duration) {
        const double dt = this->dt;
        const int N = std::max(2, int(std::lround(duration / dt)));
        std::ofstream csv(outdir / ("trajectory_" + tag + ".csv"));
        csv.precision(10);
        csv << "k,u,theta,s,d_base,d_static,d_b3,coll_base,coll_b3,q_margin,dq,ddq,e_task,interv,slack,active\n";

        std::vector<double> thv(N), sv(N), uv(N);
        for (int k = 0; k < N; k++) {
            double u = N > 1 ? double(k) / (N - 1) : 0.5;
            thv[k] = th0 + (th1 - th0) * u;
            sv[k] = s0 + (s1 - s0) * u;
            uv[k] = u;
        }

        TaskSummary S;
        S.tag = tag;
        S.N = N;

        // Full joint keyframes for the B0 (baseline) and B3 (null-space) regimes, so a
        // renderer can replay the exact trajectories the scalars above describe.
        // Layout per knot: {theta, s, qL[0..6], qR[0..6]}.
        std::vector<std::array<double, 16>> kb(N), kv(N);

        // ---- B0 pass: continuation IK from home, no reshaping ----
        std::vector<double> d_base(N), coll_base(N);
        {
            Q ql = seeds[0], qr = seeds[1], out(7);
            for (int k = 0; k < N; k++) {
                bool ok = true;
                for (int side = 0; side < 2; side++) {
                    Q &cur = side ? qr : ql;
                    if (ik[side]->CartToJnt(cur, target(side, thv[k], sv[k]), out) < 0 &&
                        ik[side]->CartToJnt(seeds[side], target(side, thv[k], sv[k]), out) < 0) {
                        ok = false;
                        break;
                    }
                    cur = out;
                }
                if (!ok) {
                    d_base[k] = NAN;
                    coll_base[k] = 1;
                    continue;
                }
                set_config(ql, qr, thv[k], sv[k]);
                coll_base[k] = collision(0) || collision(1) ? 1 : 0;
                d_base[k] = std::min(wall_clearance(0), wall_clearance(1));
                S.coll_base += (int)coll_base[k];
                S.d_base_min = std::min(S.d_base_min, d_base[k]);
                kb[k][0] = thv[k];
                kb[k][1] = sv[k];
                for (int j = 0; j < 7; j++) {
                    kb[k][2 + j] = ql(j);
                    kb[k][9 + j] = qr(j);
                }
            }
        }

        // ---- B1 (d*_static) + B3 (least-intervention from safe start) ----
        std::vector<double> d_static(N), d_b3(N), coll_b3(N), qm(N), dq(N), ddq(N), et(N), interv(N), slack(N);
        std::vector<int> act(N);
        {
            // safe pre-shaped start at x_0 = (th0, s0), closest-to-home with d>=d_safe
            Q ql = seeds[0], qr = seeds[1];
            double dstart;
            Q qs(7);
            safe_start(0, seeds[0], seeds[1], th0, s0, qs, dstart);
            ql = qs;
            safe_start(1, seeds[1], ql, th0, s0, qs, dstart);
            qr = qs;

            Q dl_prev(7), dr_prev(7), out(7);
            std::array<double, 14> prev_q{}, prev_dq{};
            bool have_prev = false, have_prev_dq = false;
            Q qstatL = seeds[0], qstatR = seeds[1];  // previous knot's static-argmax config (manifold continuation)
            bool have_stat = false;
            for (int k = 0; k < N; k++) {
                double th = thv[k], s = sv[k];

                Q ql_base, qr_base;
                bool ok = true;
                for (int side = 0; side < 2; side++) {
                    Q &cur = side ? qr : ql;
                    Q &base = side ? qr_base : ql_base;
                    if (ik[side]->CartToJnt(cur, target(side, th, s), out) < 0 &&
                        ik[side]->CartToJnt(seeds[side], target(side, th, s), out) < 0) {
                        ok = false;
                        break;
                    }
                    base = out;
                }

                if (ok) {
                    Q ql_s, qr_s;
                    double dLs, dRs;
                    set_config(ql_base, qr_base, th, s);
                    // Seed from the previous knot's argmax config so the trace follows the
                    // max-clearance branch continuously (avoids isolated off-branch dips).
                    self_motion_trace(0, have_stat ? qstatL : ql_base, qr_base, th, s, handle[0].M, dLs, ql_s);
                    self_motion_trace(1, have_stat ? qstatR : qr_base, ql_base, th, s, handle[1].M, dRs, qr_s);
                    qstatL = ql_s;
                    qstatR = qr_s;
                    have_stat = true;
                    d_static[k] = std::min(dLs, dRs);
                    S.d_static_min = std::min(S.d_static_min, d_static[k]);
                } else {
                    d_static[k] = NAN;
                }

                if (!ok) {
                    d_b3[k] = NAN;
                    coll_b3[k] = 1;
                    qm[k] = NAN;
                    dq[k] = NAN;
                    ddq[k] = NAN;
                    et[k] = NAN;
                    interv[k] = NAN;
                    slack[k] = NAN;
                    act[k] = 0;
                    have_prev = false;
                    have_prev_dq = false;
                    continue;
                }

                Q ql_out, qr_out;
                double dL, dR, aL, aR;
                int acL, acR;
                set_config(ql_base, qr_base, th, s);
                auto nL = self_motion(0);
                bool safeL = solve_b3(0, ql_base, qr_base, nL, ql, dl_prev, th, s, ql_out, dL, aL, acL);
                set_config(ql_out, qr_base, th, s);
                auto nR = self_motion(1);
                bool safeR = solve_b3(1, qr_base, ql_out, nR, qr, dr_prev, th, s, qr_out, dR, aR, acR);

                set_config(ql_out, qr_out, th, s);
                d_b3[k] = std::min(wall_clearance(0), wall_clearance(1));
                coll_b3[k] = collision(0) || collision(1) ? 1 : 0;
                qm[k] = joint_margin();
                et[k] = task_error();
                slack[k] = d_safe - d_b3[k];
                act[k] = std::max(acL, acR);
                if (safeL && safeR && !coll_b3[k])
                    S.n_safe++;

                std::array<double, 14> cur_q{};
                for (int j = 0; j < 7; j++) {
                    cur_q[j] = ql_out(j);
                    cur_q[7 + j] = qr_out(j);
                }
                double intrv = 0;
                for (int j = 0; j < 7; j++) {
                    intrv += std::pow((ql_out(j) - ql_base(j)) / dt, 2);
                    intrv += std::pow((qr_out(j) - qr_base(j)) / dt, 2);
                }
                interv[k] = std::sqrt(intrv);
                S.J_interv += intrv;
                S.mean_static_minus_b3 += d_static[k] - d_b3[k];

                double mdq = 0, mddq = 0;
                if (have_prev) {
                    std::array<double, 14> cur_dq{};
                    for (int j = 0; j < 14; j++) {
                        cur_dq[j] = (cur_q[j] - prev_q[j]) / dt;
                        mdq = std::max(mdq, std::fabs(cur_dq[j]));
                    }
                    if (have_prev_dq)
                        for (int j = 0; j < 14; j++)
                            mddq = std::max(mddq, std::fabs((cur_dq[j] - prev_dq[j]) / dt));
                    prev_dq = cur_dq;
                    have_prev_dq = true;
                }
                dq[k] = mdq;
                ddq[k] = mddq;
                prev_q = cur_q;
                have_prev = true;

                S.coll_b3 += (int)coll_b3[k];
                S.d_b3_min = std::min(S.d_b3_min, d_b3[k]);
                S.max_dq = std::max(S.max_dq, dq[k]);
                S.max_ddq = std::max(S.max_ddq, ddq[k]);
                S.max_e = std::max(S.max_e, et[k]);
                S.min_qm = std::min(S.min_qm, qm[k]);
                switch (act[k]) {
                    case 0: S.n_none++; break;
                    case 1: S.n_clearance++; break;
                    case 2: S.n_dynamic++; break;
                    default: S.n_joint++; break;
                }

                for (int j = 0; j < 7; j++) {
                    dl_prev(j) = ql_out(j) - ql(j);
                    dr_prev(j) = qr_out(j) - qr(j);
                }
                ql = ql_out;
                qr = qr_out;
                kv[k][0] = th;
                kv[k][1] = s;
                for (int j = 0; j < 7; j++) {
                    kv[k][2 + j] = ql_out(j);
                    kv[k][9 + j] = qr_out(j);
                }
            }
        }
        S.mean_static_minus_b3 /= N;

        for (int k = 0; k < N; k++)
            csv << k << ',' << uv[k] << ',' << thv[k] << ',' << sv[k] << ',' << d_base[k] << ',' << d_static[k] << ','
                << d_b3[k] << ',' << coll_base[k] << ',' << coll_b3[k] << ',' << qm[k] << ',' << dq[k] << ','
                << ddq[k] << ',' << et[k] << ',' << interv[k] << ',' << slack[k] << ',' << act[k] << '\n';
        csv.close();

        auto write_keyframes = [&](const char *suffix, const std::vector<std::array<double, 16>> &kf) {
            std::ofstream kf_csv(outdir / ("keyframes_" + tag + "_" + suffix + ".csv"));
            kf_csv.precision(10);
            kf_csv << "k,theta,s,qL0,qL1,qL2,qL3,qL4,qL5,qL6,qR0,qR1,qR2,qR3,qR4,qR5,qR6\n";
            for (int k = 0; k < N; k++) {
                kf_csv << k;
                for (double v : kf[k])
                    kf_csv << ',' << v;
                kf_csv << '\n';
            }
        };
        write_keyframes("base", kb);
        write_keyframes("b3", kv);
        return S;
    }

    std::string summarize(const TaskSummary &S) const {
        char buf[640];
        std::snprintf(buf, sizeof(buf),
                      "%s: B0 min d=%.4f colls=%d | d*_static min=%.4f | B3 min d=%.4f colls=%d safe=%d/%d | "
                      "max|qdot|=%.2f max|qddot|=%.2f min q_margin=%.4f | J_interv=%.3f mean(d*-d)=%.4f | "
                      "act none/c/clear/dyn/joint=%d/%d/%d/%d",
                      S.tag.c_str(), S.d_base_min, S.coll_base, S.d_static_min, S.d_b3_min, S.coll_b3, S.n_safe, S.N,
                      S.max_dq, S.max_ddq, S.min_qm, S.J_interv, S.mean_static_minus_b3, S.n_none, S.n_clearance,
                      S.n_dynamic, S.n_joint);
        return buf;
    }

    // Micro-benchmark: time the atomic ops that make up the offline atlas build and the
    // online B4 cycle, at N random valid task states (θ∈[-50°,50°], s∈[-160,0]mm). Reports
    // mean/P50/P95/P99/max so the offline build cost and the online latency budget can be
    // predicted BEFORE committing to a 2-D vs branch-aware atlas.
    void benchmark(const fs::path &outdir, int N) {
        using clk = std::chrono::steady_clock;
        auto ms = [](clk::time_point a, clk::time_point b) {
            return std::chrono::duration<double, std::milli>(b - a).count();
        };
        std::vector<double> t_ik, t_fwd, t_clr, t_aeval, t_trace, t_alpha, t_bilin;

        std::mt19937 rng(12345);
        std::uniform_real_distribution<double> thd(-0.87266, 0.87266);  // ±50°
        std::uniform_real_distribution<double> sd(-0.16, 0.0);          // [-160,0] mm

        std::vector<std::array<double, 2>> pts(N);
        for (auto &p : pts)
            p = {thd(rng), sd(rng)};

        // land on the grasp manifold at the first random state
        Q ql = seeds[0], qr = seeds[1], out(7);
        ik[0]->CartToJnt(ql, target(0, pts[0][0], pts[0][1]), out);
        ql = out;
        ik[1]->CartToJnt(qr, target(1, pts[0][0], pts[0][1]), out);
        qr = out;

        int n_ik_fail = 0, n_trace_ok = 0;
        for (int k = 0; k < N; k++) {
            double th = pts[k][0], s = pts[k][1];
            Q ql_prev = ql, qr_prev = qr;

            // 1. continuation TRAC-IK (seed = previous config), one per arm
            auto t0 = clk::now();
            int okL = ik[0]->CartToJnt(ql_prev, target(0, th, s), out);
            t_ik.push_back(ms(t0, clk::now()));
            if (okL < 0) { out = ql_prev; n_ik_fail++; }
            ql = out;
            t0 = clk::now();
            int okR = ik[1]->CartToJnt(qr_prev, target(1, th, s), out);
            t_ik.push_back(ms(t0, clk::now()));
            if (okR < 0) { out = qr_prev; n_ik_fail++; }
            qr = out;

            // 2. mj_forward (via set_config)
            t0 = clk::now();
            set_config(ql, qr, th, s);
            t_fwd.push_back(ms(t0, clk::now()));

            // 3. wall clearance, one side (point cloud vs box)
            t0 = clk::now();
            volatile double d0 = wall_clearance(0);
            (void)d0;
            t_clr.push_back(ms(t0, clk::now()));

            // 4. one alpha-grid eval = forward + contact + clearance
            t0 = clk::now();
            {
                set_config(ql, qr, th, s);
                bool c = collision(0);
                double dd = wall_clearance(0);
                (void)c; (void)dd;
            }
            t_aeval.push_back(ms(t0, clk::now()));

            // 5. full self-motion trace (one arm) = the per-grid-point atlas cost
            double dmax = -1.0;
            Q qmax(7);
            t0 = clk::now();
            bool ok = self_motion_trace(0, ql, qr, th, s, handle[0].M, dmax, qmax);
            t_trace.push_back(ms(t0, clk::now()));
            if (ok)
                n_trace_ok++;

            // 6. full alpha-grid reshape scan (2*alpha_grid evals) = worst-case online reshape
            t0 = clk::now();
            {
                double d_best = -1e9;
                for (int i = 1; i <= alpha_grid; i++) {
                    double frac = double(i) / alpha_grid;
                    for (int sg = 0; sg < 2; sg++) {
                        double al = sg ? -0.02 * frac : 0.02 * frac;  // nominal ±nullspace step
                        Q cand(7);
                        for (int j = 0; j < 7; j++)
                            cand(j) = std::clamp(ql(j) + al * 0.0, lower[0](j), upper[0](j));
                        set_config(cand, qr, th, s);
                        double dd = wall_clearance(0);
                        if (dd > d_best)
                            d_best = dd;
                    }
                }
            }
            t_alpha.push_back(ms(t0, clk::now()));

            // 7. bilinear lookup, amortized over reps (a single lookup is ~ns)
            {
                static std::vector<float> g(101 * 65, 0.5f);
                static volatile double sink = 0.0;
                t0 = clk::now();
                double acc = 0.0;
                const int R = 200000;
                for (int r = 0; r < R; r++) {
                    double x = (th + 0.87266) / (2 * 0.87266) * 100.0;
                    double y = (s + 0.16) / 0.16 * 64.0;
                    int ix = std::clamp((int)x, 0, 99);
                    int iy = std::clamp((int)y, 0, 63);
                    double fx = x - ix, fy = y - iy;
                    int i0 = iy * 101 + ix;
                    double v = g[i0] * (1 - fx) * (1 - fy) + g[i0 + 1] * fx * (1 - fy) +
                               g[i0 + 101] * (1 - fx) * fy + g[i0 + 102] * fx * fy;
                    acc += v;
                }
                sink += acc;  // keep the loop from being dead-code eliminated
                t_bilin.push_back(ms(t0, clk::now()) / R);  // ms per lookup
            }
        }

        auto report = [](const char *name, const std::vector<double> &v, double scale, const char *unit,
                         std::ofstream &csv) {
            if (v.empty())
                return;
            auto w = v;
            std::sort(w.begin(), w.end());
            auto pct = [&](double p) {
                return w[std::min((size_t)std::llround(p * (w.size() - 1)), w.size() - 1)];
            };
            double mean = std::accumulate(w.begin(), w.end(), 0.0) / w.size();
            double mn = mean * scale, p50 = pct(0.50) * scale, p95 = pct(0.95) * scale,
                   p99 = pct(0.99) * scale, mx = w.back() * scale;
            std::cout << "  " << std::left << std::setw(22) << name << std::right << std::fixed << std::setprecision(3)
                      << " mean=" << std::setw(8) << mn << " " << unit << "  P50=" << std::setw(8) << p50
                      << "  P95=" << std::setw(8) << p95 << "  P99=" << std::setw(8) << p99
                      << "  max=" << std::setw(8) << mx << "  n=" << w.size() << "\n";
            csv << name << ',' << mn << ',' << p50 << ',' << p95 << ',' << p99 << ',' << mx << ',' << w.size() << '\n';
        };

        std::cout << "benchmark N=" << N << " (random θ∈[-50°,50°], s∈[-160,0]mm), IK-fails=" << n_ik_fail
                  << ", self-motion-trace ok=" << n_trace_ok << "/" << N << "\n";
        std::ofstream csv(outdir / "benchmark_timing.csv");
        csv << "op,mean,p50,p95,p99,max,n\n";
        report("T_TRACIK", t_ik, 1.0, "ms", csv);
        report("T_forward", t_fwd, 1.0, "ms", csv);
        report("T_clearance", t_clr, 1.0, "ms", csv);
        report("T_alpha_eval", t_aeval, 1.0, "ms", csv);
        report("T_selfmotion_trace", t_trace, 1.0, "ms", csv);
        report("T_alpha_scan_402", t_alpha, 1.0, "ms", csv);
        report("T_bilinear", t_bilin, 1e6, "ns", csv);
        csv.close();
        std::cout << "wrote " << outdir / "benchmark_timing.csv" << "\n";
    }

    // =====================================================================
    // Module C: T0 / T1 / T4 realtime comparison. ONE precomputed task stream
    // (θ_k, s_k) is replayed through the three controllers, timing each cycle's
    // algorithmic sub-steps separately, so the offline-atlas question is answered
    // with measured numbers (P50/P95/P99/max, 20-ms deadline-miss rate, T4
    // activation ratio) plus safety metrics — not estimates.
    //
    //   T0  pure continuation TRAC-IK            T_total = T_IK
    //   T1  TRAC-IK + collision detection        T_total = T_IK + T_collision
    //   T4  TRAC-IK + atlas + collision + reshape
    //                                       T_total = T_IK + T_atlas + T_collision + [T_reshape]
    //   T5  safe-manifold lookup + fixed 2-step Newton (NO per-frame TRAC-IK)
    //                                       T_total = T_lookup + T_newton
    //
    // T4 is least-intervention (solve_b3): reshape only when the current baseline
    // branch is unsafe (d_base < d_safe) AND the atlas says a safe self-motion
    // config exists (D_g >= d_safe). Atlas-infeasible (D_g < d_safe) → declare
    // infeasible, no reshape. At g=0.51 m the atlas is 100% feasible, so the atlas
    // gate never fires and T4's active ratio isolates the branch-recovery cost.
    //
    // T5 moves safety OFFLINE: the section Q_g(θ,s) is built so every stored config
    // satisfies d ≥ d_safe AND joint margin ≥ m_q (build_manifold_safe), so online is
    // a bilinear lookup + 1–2 fixed Newton projections and no TRAC-IK branch search or
    // α-grid. Safety (d_min/coll) is still MEASURED each cycle but is not in its runtime.
    // =====================================================================
    void compare(const fs::path &outdir, int N, const fs::path &atlas_dir) {
        using clk = std::chrono::steady_clock;
        auto ms = [](clk::time_point a, clk::time_point b) {
            return std::chrono::duration<double, std::milli>(b - a).count();
        };
        const double deadline = 20.0;  // 50 Hz control cycle
        const double dt = 0.02;        // fixed 50 Hz cycle (argv[8] is repurposed as N here, not DT)

        Atlas atlas;
        if (!atlas.load(atlas_dir)) {
            std::cerr << "compare: failed to load atlas from " << atlas_dir << "\n";
            return;
        }
        std::cout << "atlas loaded: n_theta=" << atlas.n_theta << " n_s=" << atlas.n_s << "\n";

        Manifold manifold;
        bool have_manifold = manifold.load(atlas_dir);
        std::cout << (have_manifold ? "manifold loaded: " : "WARNING: no manifold in ")
                  << atlas_dir << (have_manifold ? " (T5 enabled)\n" : " (T5 disabled)\n");

        struct Profile { const char *name; double f, th_amp, s_amp; };
        const Profile profiles[] = {
            {"sine", 0.2, 0.87266, 0.0},
            {"roll_pull", 0.2, 0.6, 0.08},
            {"random", 0.0, 0.0, 0.0},
        };

        struct MS {
            std::vector<double> t_ik, t_atlas, t_coll, t_reshape, t_lookup, t_newton, t_total;
            std::vector<double> t_total_inactive, t_total_active;
            std::vector<double> dq_all, ddq_all, margin_all;  // per-cycle max|q̇|, max|q̈|, min joint margin
            int n_active = 0, n_infeasible = 0, n_lookup_fail = 0;
            double d_min = 1e9, max_dq = 0, max_ddq = 0, e_task_max = 0;
            int n_coll = 0, n_safe = 0;
        };

        auto run_method = [&](int method, const std::vector<std::array<double, 4>> &stream) {
            MS S;
            const size_t M = stream.size();
            Q ql = seeds[0], qr = seeds[1], out(7);
            // land on the grasp manifold at the first state (shared by all methods)
            double th0 = stream[0][0], s0 = stream[0][1];
            if (method == 3 && have_manifold) {
                // T5: manifold lookup + 2 Newton steps (no IK seed)
                std::array<double, 14> q14;
                if (manifold.lookup(th0, s0, q14)) {
                    for (int j = 0; j < 7; j++) {
                        ql(j) = q14[j];
                        qr(j) = q14[7 + j];
                    }
                    Q ql1(7), qr1(7);
                    manifold_newton_step(ql, qr, th0, s0, ql1, qr1);
                    manifold_newton_step(ql1, qr1, th0, s0, ql, qr);
                }
            } else {
                ik[0]->CartToJnt(seeds[0], target(0, th0, s0), out);
                ql = out;
                ik[1]->CartToJnt(seeds[1], target(1, th0, s0), out);
                qr = out;
            }
            set_config(ql, qr, th0, s0);
            Q dl_prev(7), dr_prev(7);
            for (int j = 0; j < 7; j++) dl_prev(j) = dr_prev(j) = 0.0;
            Eigen::Matrix<double, 7, 1> qdotL = Eigen::Matrix<double, 7, 1>::Zero();
            Eigen::Matrix<double, 7, 1> qdotR = Eigen::Matrix<double, 7, 1>::Zero();

            for (size_t k = 0; k < M; k++) {
                double th = stream[k][0], s = stream[k][1];
                double t_ik = 0, t_atlas = 0, t_coll = 0, t_reshape = 0, t_lookup = 0, t_newton = 0;

                Q ql_base(7), qr_base(7);
                if (method == 3 && have_manifold) {
                    // T5: safe-manifold bilinear lookup + fixed 2-step Newton (no TRAC-IK).
                    auto t0 = clk::now();
                    std::array<double, 14> q14;
                    bool inside = manifold.lookup(th, s, q14);
                    t_lookup = ms(t0, clk::now());
                    if (inside) {
                        Q ql0(7), qr0(7);
                        for (int j = 0; j < 7; j++) {
                            ql0(j) = q14[j];
                            qr0(j) = q14[7 + j];
                        }
                        t0 = clk::now();
                        Q ql1(7), qr1(7);
                        manifold_newton_step(ql0, qr0, th, s, ql1, qr1);
                        manifold_newton_step(ql1, qr1, th, s, ql_base, qr_base);
                        t_newton = ms(t0, clk::now());
                    } else {
                        S.n_lookup_fail++;  // outside the certified grid: hold previous config
                        ql_base = ql;
                        qr_base = qr;
                    }
                } else {
                    // 1. continuation IK (both arms) — shared by T0/T1/T4
                    auto t0 = clk::now();
                    bool okL = ik[0]->CartToJnt(ql, target(0, th, s), out) >= 0;
                    if (!okL)
                        okL = ik[0]->CartToJnt(seeds[0], target(0, th, s), out) >= 0;
                    ql_base = okL ? out : ql;
                    bool okR = ik[1]->CartToJnt(qr, target(1, th, s), out) >= 0;
                    if (!okR)
                        okR = ik[1]->CartToJnt(seeds[1], target(1, th, s), out) >= 0;
                    qr_base = okR ? out : qr;
                    t_ik = ms(t0, clk::now());
                }

                double d_base = 1e9, D_g = -1.0;
                int coll_base = 0;
                bool active = false, infeasible = false;

                if (method == 1) {
                    // T1: detect only — measure d_min and collision, never intervene
                    auto t0 = clk::now();
                    set_config(ql_base, qr_base, th, s);
                    d_base = std::min(wall_clearance(0), wall_clearance(1));
                    coll_base = (collision(0) || collision(1)) ? 1 : 0;
                    t_coll = ms(t0, clk::now());
                } else if (method == 2) {
                    // T4: atlas feasibility gate + collision detection + conditional reshape
                    auto t0 = clk::now();
                    D_g = atlas.lookup(th, s);
                    t_atlas = ms(t0, clk::now());

                    t0 = clk::now();
                    set_config(ql_base, qr_base, th, s);
                    d_base = std::min(wall_clearance(0), wall_clearance(1));
                    coll_base = (collision(0) || collision(1)) ? 1 : 0;
                    t_coll = ms(t0, clk::now());

                    if (d_base < d_safe) {
                        if (D_g >= d_safe) {
                            // redundancy-recoverable: local least-intervention reshape
                            active = true;
                            t0 = clk::now();
                            Q ql_out(7), qr_out(7);
                            double dL, dR, aL, aR;
                            int acL, acR;
                            set_config(ql_base, qr_base, th, s);
                            auto nL = self_motion(0);
                            solve_b3(0, ql_base, qr_base, nL, ql, dl_prev, th, s, ql_out, dL, aL, acL);
                            set_config(ql_out, qr_base, th, s);
                            auto nR = self_motion(1);
                            solve_b3(1, qr_base, ql_out, nR, qr, dr_prev, th, s, qr_out, dR, aR, acR);
                            ql_base = ql_out;
                            qr_base = qr_out;
                            t_reshape = ms(t0, clk::now());
                        } else {
                            infeasible = true;  // atlas says no self-motion config is safe
                        }
                    }
                }
                // (method 0: T0 does no collision checking; baseline config is final)

                // final config: baseline (T0/T1) or reshaped (T4)
                set_config(ql_base, qr_base, th, s);

                // safety metrics — measured for every method (for T0 this is a
                // measurement, not part of its runtime)
                double dmin = std::min(wall_clearance(0), wall_clearance(1));
                int coll = (collision(0) || collision(1)) ? 1 : 0;
                double et = task_error();
                double mdq = 0, mddq = 0;
                Eigen::Matrix<double, 7, 1> dqL, dqR;
                for (int j = 0; j < 7; j++) {
                    dqL(j) = (ql_base(j) - ql(j)) / dt;
                    dqR(j) = (qr_base(j) - qr(j)) / dt;
                    mdq = std::max({mdq, std::fabs(dqL(j)), std::fabs(dqR(j))});
                }
                for (int j = 0; j < 7; j++)
                    mddq = std::max({mddq, std::fabs((dqL(j) - qdotL(j)) / dt),
                                     std::fabs((dqR(j) - qdotR(j)) / dt)});

                S.d_min = std::min(S.d_min, dmin);
                S.max_dq = std::max(S.max_dq, mdq);
                S.max_ddq = std::max(S.max_ddq, mddq);
                S.e_task_max = std::max(S.e_task_max, et);
                S.n_coll += coll;
                if (dmin >= d_safe - 1e-9 && !coll)
                    S.n_safe++;
                S.dq_all.push_back(mdq);
                S.ddq_all.push_back(mddq);
                S.margin_all.push_back(std::min(arm_margin(0, ql_base), arm_margin(1, qr_base)));

                double t_total = (method == 3) ? (t_lookup + t_newton)
                                              : (t_ik + t_atlas + t_coll + t_reshape);
                S.t_ik.push_back(t_ik);
                S.t_atlas.push_back(t_atlas);
                S.t_coll.push_back(t_coll);
                S.t_reshape.push_back(t_reshape);
                S.t_lookup.push_back(t_lookup);
                S.t_newton.push_back(t_newton);
                S.t_total.push_back(t_total);
                if (active) {
                    S.n_active++;
                    S.t_total_active.push_back(t_total);
                } else {
                    S.t_total_inactive.push_back(t_total);
                }
                if (infeasible)
                    S.n_infeasible++;

                for (int j = 0; j < 7; j++) {
                    dl_prev(j) = ql_base(j) - ql(j);
                    dr_prev(j) = qr_base(j) - qr(j);
                }
                ql = ql_base;
                qr = qr_base;
                qdotL = dqL;
                qdotR = dqR;
            }
            return S;
        };

        auto pct = [](const std::vector<double> &v, double p) {
            if (v.empty())
                return 0.0;
            auto w = v;
            std::sort(w.begin(), w.end());
            return w[std::min((size_t)std::llround(p * (w.size() - 1)), w.size() - 1)];
        };
        auto mean = [](const std::vector<double> &v) {
            return v.empty() ? 0.0 : std::accumulate(v.begin(), v.end(), 0.0) / v.size();
        };
        auto mx = [](const std::vector<double> &v) {
            return v.empty() ? 0.0 : *std::max_element(v.begin(), v.end());
        };
        auto miss_rate = [&](const std::vector<double> &v) {
            if (v.empty())
                return 0.0;
            size_t n = 0;
            for (double x : v)
                if (x > deadline)
                    n++;
            return 100.0 * n / v.size();
        };

        std::ofstream tim(outdir / "compare_timing.csv");
        tim << "profile,method,P50_ms,P95_ms,P99_ms,max_ms,mean_ms,deadline_miss_pct,n\n";
        tim.precision(4);
        std::ofstream saf(outdir / "compare_safety.csv");
        saf << "profile,method,n_coll,d_min_mm,max_dq,max_ddq,e_task_max_mm,n_safe,n_infeasible,active_pct\n";
        saf.precision(4);
        std::ofstream sub(outdir / "compare_substeps.csv");
        sub << "profile,method,mean_t_ik_ms,mean_t_atlas_ms,mean_t_coll_ms,mean_t_reshape_ms,mean_t_lookup_ms,mean_t_newton_ms\n";
        sub.precision(4);
        std::ofstream dyn(outdir / "compare_dynamics.csv");
        dyn << "profile,method,dq_p50,dq_p95,dq_p99,dq_max,ddq_p50,ddq_p95,ddq_p99,ddq_max,pct_low_margin\n";
        dyn.precision(4);

        for (const auto &p : profiles) {
            // precompute ONE task stream, replayed by all three methods
            std::vector<std::array<double, 4>> stream(N);
            std::mt19937 rng(987654321u);
            std::uniform_real_distribution<double> U01(0.0, 1.0);
            std::uniform_real_distribution<double> Uph(0.0, 2.0 * M_PI);
            const int ncomp = 5;
            double thA[ncomp], thPh[ncomp], sA[ncomp], sPh[ncomp];
            const double thF[ncomp] = {0.10, 0.25, 0.40, 0.70, 1.10};
            const double sF[ncomp] = {0.08, 0.20, 0.35, 0.60, 0.90};
            for (int i = 0; i < ncomp; i++) {
                thA[i] = 0.35 + 0.65 * U01(rng);
                thPh[i] = Uph(rng);
                sA[i] = 0.35 + 0.65 * U01(rng);
                sPh[i] = Uph(rng);
            }
            for (int k = 0; k < N; k++) {
                double t = k * dt;
                double th, s;
                if (std::string(p.name) == "random") {
                    th = 0.0;
                    s = -0.08;
                    for (int i = 0; i < ncomp; i++) {
                        th += thA[i] * std::sin(2.0 * M_PI * thF[i] * t + thPh[i]);
                        s += 0.04 * sA[i] * std::sin(2.0 * M_PI * sF[i] * t + sPh[i]);
                    }
                    th = std::clamp(0.5 * th, -0.87266, 0.87266);
                    s = std::clamp(s, -0.16, 0.0);
                } else {
                    double thd, sd;
                    command(p.name, t, p.f, p.th_amp, p.s_amp, dt * N, th, s, thd, sd);
                }
                stream[k] = {th, s, 0.0, 0.0};
            }

            std::cout << "\n=== profile " << p.name << " (N=" << N << ") ===\n";
            std::cout << std::left << std::setw(20) << "method" << std::right << std::setw(9) << "P50"
                      << std::setw(9) << "P95" << std::setw(9) << "P99" << std::setw(9) << "max"
                      << std::setw(10) << "miss%"
                      << "   (total ms)\n";

            for (int method = 0; method < (have_manifold ? 4 : 3); method++) {
                const char *mname = method == 0   ? "T0_TRACIK"
                                    : method == 1 ? "T1_TRACIK+coll"
                                    : method == 2 ? "T4_atlas+reshape"
                                                  : "T5_manifold+Newton";
                MS S = run_method(method, stream);
                double active_pct = 100.0 * S.n_active / std::max(1, (int)stream.size());

                auto emit = [&](const std::string &label, const std::vector<double> &v) {
                    std::cout << std::left << std::setw(20) << label << std::right << std::fixed
                              << std::setprecision(3) << std::setw(9) << pct(v, 0.50) << std::setw(9)
                              << pct(v, 0.95) << std::setw(9) << pct(v, 0.99) << std::setw(9)
                              << mx(v) << std::setw(9) << std::setprecision(1)
                              << miss_rate(v) << "\n";
                    tim << p.name << ',' << label << ',' << pct(v, 0.50) << ',' << pct(v, 0.95) << ','
                        << pct(v, 0.99) << ',' << mx(v) << ',' << mean(v) << ','
                        << miss_rate(v) << ',' << v.size() << '\n';
                };

                if (method == 2) {
                    emit(std::string(mname) + "_inactive", S.t_total_inactive);
                    emit(std::string(mname) + "_active", S.t_total_active);
                } else {
                    emit(mname, S.t_total);
                }

                std::cout << "    d_min=" << std::fixed << std::setprecision(2) << S.d_min * 1e3
                          << "mm  coll=" << S.n_coll << "  max_dq=" << std::setprecision(2) << S.max_dq
                          << "  max_ddq=" << S.max_ddq << "  e_task_max=" << S.e_task_max * 1e3 << "mm"
                          << "  active=" << std::setprecision(1) << active_pct << "%"
                          << "  infeasible=" << S.n_infeasible
                          << (method == 3 ? ("  lookup_fail=" + std::to_string(S.n_lookup_fail)) : "")
                          << "\n";
                saf << p.name << ',' << mname << ',' << S.n_coll << ',' << S.d_min * 1e3 << ',' << S.max_dq
                    << ',' << S.max_ddq << ',' << S.e_task_max * 1e3 << ',' << S.n_safe << ',' << S.n_infeasible
                    << ',' << active_pct << '\n';
                sub << p.name << ',' << mname << ',' << mean(S.t_ik) << ',' << mean(S.t_atlas) << ','
                    << mean(S.t_coll) << ',' << mean(S.t_reshape) << ',' << mean(S.t_lookup) << ','
                    << mean(S.t_newton) << '\n';
                double pct_low_margin =
                    S.margin_all.empty()
                        ? 0.0
                        : 100.0 * std::count_if(S.margin_all.begin(), S.margin_all.end(),
                                                [&](double m) { return m < q_margin_target - 1e-6; }) /
                              S.margin_all.size();
                dyn << p.name << ',' << mname << ',' << pct(S.dq_all, 0.5) << ',' << pct(S.dq_all, 0.95) << ','
                    << pct(S.dq_all, 0.99) << ',' << mx(S.dq_all) << ',' << pct(S.ddq_all, 0.5) << ','
                    << pct(S.ddq_all, 0.95) << ',' << pct(S.ddq_all, 0.99) << ',' << mx(S.ddq_all) << ','
                    << pct_low_margin << '\n';
                std::cout << "    |qdot| P50/P99/max=" << std::fixed << std::setprecision(2)
                          << pct(S.dq_all, 0.50) << "/" << pct(S.dq_all, 0.99) << "/" << mx(S.dq_all)
                          << " rad/s  |qddot| P50/P99/max=" << pct(S.ddq_all, 0.50) << "/"
                          << pct(S.ddq_all, 0.99) << "/" << mx(S.ddq_all) << " rad/s^2"
                          << "  low_margin=" << std::setprecision(1) << pct_low_margin << "%\n";
            }
        }
        tim.close();
        saf.close();
        sub.close();
        dyn.close();
        std::cout << "\nwrote " << outdir / "compare_timing.csv" << ", " << outdir / "compare_safety.csv"
                  << ", " << outdir / "compare_substeps.csv" << ", " << outdir / "compare_dynamics.csv"
                  << "\n";
    }

    // Record the T0 (pure continuation TRAC-IK, no collision awareness) joint trajectory for
    // the three compare profiles, as keyframes + clearance scalars, so a renderer can show the
    // arm penetrating the wall. Writes keyframes_t0_<profile>.csv (16 cols:
    // theta,s,qL0..6,qR0..6) and trajectory_t0_<profile>.csv (d_base,coll per knot).
    void record_t0(const fs::path &outdir, int N) {
        const double dt = 0.02;  // 50 Hz
        struct Profile { const char *name; double f, th_amp, s_amp; };
        const Profile profiles[] = {
            {"sine", 0.2, 0.87266, 0.0},
            {"roll_pull", 0.2, 0.6, 0.08},
            {"random", 0.0, 0.0, 0.0},
        };

        for (const auto &p : profiles) {
            // identical task stream to compare()
            std::vector<std::array<double, 2>> stream(N);
            std::mt19937 rng(987654321u);
            std::uniform_real_distribution<double> U01(0.0, 1.0);
            std::uniform_real_distribution<double> Uph(0.0, 2.0 * M_PI);
            const int ncomp = 5;
            double thA[ncomp], thPh[ncomp], sA[ncomp], sPh[ncomp];
            const double thF[ncomp] = {0.10, 0.25, 0.40, 0.70, 1.10};
            const double sF[ncomp] = {0.08, 0.20, 0.35, 0.60, 0.90};
            for (int i = 0; i < ncomp; i++) {
                thA[i] = 0.35 + 0.65 * U01(rng);
                thPh[i] = Uph(rng);
                sA[i] = 0.35 + 0.65 * U01(rng);
                sPh[i] = Uph(rng);
            }
            for (int k = 0; k < N; k++) {
                double t = k * dt;
                double th, s;
                if (std::string(p.name) == "random") {
                    th = 0.0; s = -0.08;
                    for (int i = 0; i < ncomp; i++) {
                        th += thA[i] * std::sin(2.0 * M_PI * thF[i] * t + thPh[i]);
                        s += 0.04 * sA[i] * std::sin(2.0 * M_PI * sF[i] * t + sPh[i]);
                    }
                    th = std::clamp(0.5 * th, -0.87266, 0.87266);
                    s = std::clamp(s, -0.16, 0.0);
                } else {
                    double thd, sd;
                    command(p.name, t, p.f, p.th_amp, p.s_amp, dt * N, th, s, thd, sd);
                }
                stream[k] = {th, s};
            }

            // pure continuation IK (T0), landing on the grasp manifold at the first knot
            Q ql = seeds[0], qr = seeds[1], out(7);
            ik[0]->CartToJnt(seeds[0], target(0, stream[0][0], stream[0][1]), out); ql = out;
            ik[1]->CartToJnt(seeds[1], target(1, stream[0][0], stream[0][1]), out); qr = out;

            std::ofstream kf(outdir / (std::string("keyframes_t0_") + p.name + ".csv"));
            kf.precision(10);
            kf << "k,theta,s,qL0,qL1,qL2,qL3,qL4,qL5,qL6,qR0,qR1,qR2,qR3,qR4,qR5,qR6\n";
            std::ofstream tr(outdir / (std::string("trajectory_t0_") + p.name + ".csv"));
            tr.precision(10);
            tr << "k,t,theta,s,d_base,coll\n";

            double dmin = 1e9, dmax = -1e9; int ncoll = 0;
            for (int k = 0; k < N; k++) {
                double th = stream[k][0], s = stream[k][1];
                Q ql_base(7), qr_base(7);
                bool okL = ik[0]->CartToJnt(ql, target(0, th, s), out) >= 0;
                if (!okL) okL = ik[0]->CartToJnt(seeds[0], target(0, th, s), out) >= 0;
                ql_base = okL ? out : ql;
                bool okR = ik[1]->CartToJnt(qr, target(1, th, s), out) >= 0;
                if (!okR) okR = ik[1]->CartToJnt(seeds[1], target(1, th, s), out) >= 0;
                qr_base = okR ? out : qr;
                set_config(ql_base, qr_base, th, s);
                double d = std::min(wall_clearance(0), wall_clearance(1));
                int coll = (collision(0) || collision(1)) ? 1 : 0;
                dmin = std::min(dmin, d); dmax = std::max(dmax, d); ncoll += coll;

                kf << k << ',' << th << ',' << s;
                for (int j = 0; j < 7; j++) kf << ',' << ql_base(j);
                for (int j = 0; j < 7; j++) kf << ',' << qr_base(j);
                kf << '\n';
                tr << k << ',' << k * dt << ',' << th << ',' << s << ',' << d << ',' << coll << '\n';
                ql = ql_base; qr = qr_base;
            }
            kf.close(); tr.close();
            std::cout << "t0 " << p.name << ": N=" << N << "  d_min=" << dmin * 1e3
                      << " mm  d_max=" << dmax * 1e3 << " mm  coll=" << ncoll << "/" << N << "\n";
        }
    }

    // =====================================================================
    // ATLAS builder (module A): 2-D static-feasibility atlas over the task
    // space x=(θ,s). At each grid point we compute, per arm, the max wall
    // clearance over the full 1-D self-motion loop (other arm frozen at the
    // baseline continuation-IK config) and record the binding-arm value:
    //
    //   D_g(θ,s) = min_{side} d*_static(side)   (m; -1 if baseline IK fails)
    //   Feasible(θ,s) = 1  iff  D_g >= d_safe   (static feasibility flag)
    //
    // D_g is a static feasibility certificate, NOT a current-branch risk
    // predictor: since D_g >= d_branch always, an online trigger on
    // D_g < d_safe fires LATER than one on the branch clearance would.
    //
    // Layout: s-major row-major, index = i_s*n_theta + i_theta, matching the
    // bilinear lookup in the online path. Outputs raw float32 D_g, uint8 feasibility
    // flag, a JSON manifest, and a human-readable CSV. Each worker owns its own
    // ClearanceTrajectory (mjData + TRAC-IK hold mutable per-call state and
    // are not thread-safe) and pulls grid indices from an atomic counter.
    // =====================================================================
    void build_atlas(const fs::path &outdir, int n_threads) {
        const double th_min = -0.8726646259971648, th_max = 0.8726646259971648;  // ±50°
        const double s_min = -0.16, s_max = 0.0;                                  // m
        const int n_theta = 101;  // 1° step
        const int n_s = 65;       // 2.5 mm step
        const size_t N = size_t(n_theta) * n_s;  // 6565

        std::vector<float> Dg(N), dL(N), dR(N), qLmax(N * 7), qRmax(N * 7);
        std::vector<uint8_t> Feasible(N, 0), reachable(N, 0), trace_complete(N, 0);
        std::vector<uint8_t> stopL(N, 0), stopR(N, 0), closedL(N, 0), closedR(N, 0);
        std::vector<uint8_t> ikfail(N, 0);

        std::atomic<size_t> next{0}, done{0}, n_ik_fail{0}, n_feasible{0}, n_complete{0};
        std::mutex cout_mu;

        auto th_at = [&](int i) { return th_min + double(i) * (th_max - th_min) / (n_theta - 1); };
        auto s_at = [&](int j) { return s_min + double(j) * (s_max - s_min) / (n_s - 1); };

        auto t0 = std::chrono::steady_clock::now();
        auto worker = [&]() {
            ClearanceTrajectory run(cfg_path, d_safe, q_margin_target, trace_half, trace_step);
            while (true) {
                size_t idx = next.fetch_add(1, std::memory_order_relaxed);
                if (idx >= N)
                    break;
                int i = int(idx % n_theta), j = int(idx / n_theta);
                double theta = th_at(i), s = s_at(j);

                Q q0[2]{Q(7), Q(7)};
                bool reach = true;
                for (int side = 0; side < 2; side++) {
                    if (!run.ik_multi_seed(side, theta, s, run.seeds[side], atlas_seed(idx, side, 0), q0[side])) {
                        reach = false;
                        break;
                    }
                }
                if (!reach) {
                    Dg[idx] = dL[idx] = dR[idx] = -1.0f;
                    reachable[idx] = trace_complete[idx] = Feasible[idx] = 0;
                    ikfail[idx] = 1;
                    n_ik_fail.fetch_add(1, std::memory_order_relaxed);
                } else {
                    reachable[idx] = 1;
                    TraceResult tr[2];
                    for (int side = 0; side < 2; side++) {
                        run.ik[side]->setSeed(atlas_seed(idx, side, 1000));  // fixed trace stream
                        run.self_motion_trace_ex(side, q0[side], q0[1 - side], theta, s, run.handle[side].M, tr[side]);
                    }
                    dL[idx] = float(tr[0].d_max);
                    dR[idx] = float(tr[1].d_max);
                    for (int k = 0; k < 7; k++) {
                        qLmax[idx * 7 + k] = float(tr[0].q_max(k));
                        qRmax[idx * 7 + k] = float(tr[1].q_max(k));
                    }
                    stopL[idx] = uint8_t(stop_code(tr[0].stop_pos));
                    stopR[idx] = uint8_t(stop_code(tr[1].stop_pos));
                    closedL[idx] = (tr[0].stop_pos == StopReason::LOOP_CLOSED || tr[0].stop_neg == StopReason::LOOP_CLOSED);
                    closedR[idx] = (tr[1].stop_pos == StopReason::LOOP_CLOSED || tr[1].stop_neg == StopReason::LOOP_CLOSED);
                    trace_complete[idx] = (tr[0].component_complete && tr[1].component_complete);
                    double dg = std::min(tr[0].d_max, tr[1].d_max);
                    Dg[idx] = float(dg);
                    Feasible[idx] = (dg >= run.d_safe);
                    if (Feasible[idx])
                        n_feasible.fetch_add(1, std::memory_order_relaxed);
                    if (trace_complete[idx])
                        n_complete.fetch_add(1, std::memory_order_relaxed);
                }

                size_t k = done.fetch_add(1, std::memory_order_relaxed) + 1;
                if (k % 500 == 0 || k == N) {
                    std::lock_guard<std::mutex> lk(cout_mu);
                    double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                    std::cout << "  atlas " << k << "/" << N << "  (" << std::fixed << std::setprecision(0)
                              << el << " s)" << std::endl;
                }
            }
        };

        std::cout << "building atlas: θ∈[" << th_min / rad << "°, " << th_max / rad << "°] × " << n_theta
                  << ", s∈[" << s_min * 1e3 << ", " << s_max * 1e3 << "] mm × " << n_s << " = " << N
                  << " points on " << n_threads << " threads" << std::endl;
        std::vector<std::thread> pool;
        for (int t = 0; t < n_threads; t++)
            pool.emplace_back(worker);
        for (auto &t : pool)
            t.join();

        double wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

        // --- mirror symmetry audit: dL(θ,s) ≈ dR(-θ,s) (exact for the mirrored arms) ---
        std::vector<double> mirr;
        for (int jj = 0; jj < n_s; jj++)
            for (int ii = 0; ii < n_theta; ii++) {
                size_t a = size_t(jj) * n_theta + ii;
                size_t b = size_t(jj) * n_theta + (n_theta - 1 - ii);
                if (dL[a] >= -0.5 && dR[b] >= -0.5)
                    mirr.push_back(std::fabs(dL[a] - dR[b]));
                if (dR[a] >= -0.5 && dL[b] >= -0.5)
                    mirr.push_back(std::fabs(dR[a] - dL[b]));
            }
        std::sort(mirr.begin(), mirr.end());
        double mirr_mae = 0, mirr_p95 = 0, mirr_max = 0;
        if (!mirr.empty()) {
            for (double v : mirr)
                mirr_mae += v;
            mirr_mae /= mirr.size();
            mirr_p95 = mirr[size_t(0.95 * (mirr.size() - 1))];
            mirr_max = mirr.back();
        }

        // --- write outputs ---
        auto write_f32 = [&](const char *name, const std::vector<float> &v) {
            std::ofstream f(outdir / name, std::ios::binary);
            f.write(reinterpret_cast<const char *>(v.data()), std::streamsize(v.size() * sizeof(float)));
        };
        auto write_u8 = [&](const char *name, const std::vector<uint8_t> &v) {
            std::ofstream f(outdir / name, std::ios::binary);
            f.write(reinterpret_cast<const char *>(v.data()), std::streamsize(v.size() * sizeof(uint8_t)));
        };
        write_f32("atlas_Dg.bin", Dg);
        write_f32("atlas_dL.bin", dL);
        write_f32("atlas_dR.bin", dR);
        write_f32("atlas_qL_max.bin", qLmax);
        write_f32("atlas_qR_max.bin", qRmax);
        write_u8("atlas_feasible.bin", Feasible);
        write_u8("atlas_reachable.bin", reachable);
        write_u8("atlas_trace_complete.bin", trace_complete);
        {
            std::ofstream f(outdir / "atlas.csv");
            f << "theta_deg,s_mm,dL,dR,D_g,reachable,closed_L,closed_R,stop_L,stop_R,trace_complete,feasible\n";
            f.precision(8);
            for (int jj = 0; jj < n_s; jj++)
                for (int ii = 0; ii < n_theta; ii++) {
                    size_t idx = size_t(jj) * n_theta + ii;
                    f << th_at(ii) / rad << ',' << s_at(jj) * 1e3 << ',' << dL[idx] << ',' << dR[idx] << ','
                      << Dg[idx] << ',' << int(reachable[idx]) << ',' << int(closedL[idx]) << ',' << int(closedR[idx])
                      << ',' << int(stopL[idx]) << ',' << int(stopR[idx]) << ',' << int(trace_complete[idx]) << ','
                      << int(Feasible[idx]) << '\n';
                }
        }
        {
            std::ofstream f(outdir / "atlas.json");
            f << "{\n"
              << "  \"name\": \"aviator_feasibility_atlas_v2\",\n"
              << "  \"grid\": {\n"
              << "    \"theta\": {\"min\": " << th_min << ", \"max\": " << th_max << ", \"n\": " << n_theta
              << ", \"unit\": \"rad\"},\n"
              << "    \"s\":     {\"min\": " << s_min << ", \"max\": " << s_max << ", \"n\": " << n_s
              << ", \"unit\": \"m\"}\n"
              << "  },\n"
              << "  \"layout\": \"s-major row-major; index = i_s * n_theta + i_theta\",\n"
              << "  \"n_theta\": " << n_theta << ", \"n_s\": " << n_s << ", \"n_total\": " << N << ",\n"
              << "  \"d_safe\": " << d_safe << ",\n"
              << "  \"gap_inner\": " << wall_gap_inner() << ",\n"
              << "  \"sentinel_Dg_infeasible\": -1.0,\n"
              << "  \"files\": {\"Dg\": \"atlas_Dg.bin\", \"dL\": \"atlas_dL.bin\", \"dR\": \"atlas_dR.bin\", "
                 "\"feasible\": \"atlas_feasible.bin\", \"reachable\": \"atlas_reachable.bin\", "
                 "\"trace_complete\": \"atlas_trace_complete.bin\", \"csv\": \"atlas.csv\"},\n"
              << "  \"dtype\": {\"Dg\": \"float32\", \"dL\": \"float32\", \"dR\": \"float32\", "
                 "\"feasible\": \"uint8\", \"reachable\": \"uint8\", \"trace_complete\": \"uint8\"},\n"
              << "  \"build\": {\"threads\": " << n_threads << ", \"n_ik_fail\": " << n_ik_fail.load()
              << ", \"n_feasible\": " << n_feasible.load() << ", \"n_complete\": " << n_complete.load()
              << ", \"wall_seconds\": " << wall_s << "},\n"
              << "  \"audit\": {\"mirror_mae_mm\": " << mirr_mae * 1e3 << ", \"mirror_p95_mm\": " << mirr_p95 * 1e3
              << ", \"mirror_max_mm\": " << mirr_max * 1e3 << "}\n"
              << "}\n";
        }

        double dmin = 1e9, dmax = -1e9, dsum = 0;
        int nd = 0;
        for (double v : Dg)
            if (v >= -0.5) {
                dmin = std::min(dmin, v);
                dmax = std::max(dmax, v);
                dsum += v;
                nd++;
            }
        std::cout << std::defaultfloat << std::setprecision(5);  // undo the progress print's fixed/precision(0)
        std::cout << "atlas done: " << N << " points in " << wall_s << " s (" << n_threads << " threads)\n"
                  << "  IK-fail " << n_ik_fail.load() << " (" << 100.0 * n_ik_fail.load() / N << "%), feasible "
                  << n_feasible.load() << "/" << N << " (" << 100.0 * n_feasible.load() / N << "% estimate), "
                  << "trace-complete " << n_complete.load() << "/" << N << "\n"
                  << "  D_g∈[" << dmin * 1e3 << ", " << dmax * 1e3 << "] mm (mean "
                  << (nd ? dsum / nd : 0.0) * 1e3 << " mm over " << nd << " traced)\n"
                  << "  mirror symmetry: MAE " << mirr_mae * 1e3 << " mm, P95 " << mirr_p95 * 1e3
                  << " mm, max " << mirr_max * 1e3 << " mm\n";
        {  // Q_max section continuity (T5 diagnostic): is the independent max-d argmax a
           // continuous section, or does it branch-jump between adjacent grid points?
            auto q_at = [&](const std::vector<float> &g, size_t idx) {
                Q q(7);
                for (int k = 0; k < 7; k++)
                    q(k) = g[idx * 7 + k];
                return q;
            };
            auto dq_abs = [](const Q &a, const Q &b) {
                double d = 0;
                for (int k = 0; k < 7; k++)
                    d = std::max(d, std::fabs(a(k) - b(k)));
                return d;
            };
            double mq_th = 0, mq_s = 0, sq_th = 0, sq_s = 0;
            size_t n_th = 0, n_sn = 0;
            for (int j = 0; j < n_s; j++)
                for (int i = 0; i < n_theta; i++) {
                    size_t idx = size_t(j) * n_theta + i;
                    if (!reachable[idx])
                        continue;
                    if (i > 0 && reachable[idx - 1]) {
                        double d = std::max(dq_abs(q_at(qLmax, idx), q_at(qLmax, idx - 1)),
                                            dq_abs(q_at(qRmax, idx), q_at(qRmax, idx - 1)));
                        mq_th = std::max(mq_th, d);
                        sq_th += d;
                        n_th++;
                    }
                    if (j > 0 && reachable[idx - n_theta]) {
                        double d = std::max(dq_abs(q_at(qLmax, idx), q_at(qLmax, idx - n_theta)),
                                            dq_abs(q_at(qRmax, idx), q_at(qRmax, idx - n_theta)));
                        mq_s = std::max(mq_s, d);
                        sq_s += d;
                        n_sn++;
                    }
                }
            std::cout << "  Q_max section continuity  Δq_θ-neighbor: max " << mq_th << " rad, mean "
                      << (n_th ? sq_th / n_th : 0.0) << "  |  Δq_s-neighbor: max " << mq_s << " rad, mean "
                      << (n_sn ? sq_s / n_sn : 0.0) << "\n";
        }
        std::cout << "wrote " << outdir / "atlas_*.bin" << ", " << outdir / "atlas.csv" << ", "
                  << outdir / "atlas.json" << "\n";
    }

    // T5 offline build (margin-respecting): construct the safe-manifold section Q_safe(θ,s) by,
    // at each grid point, tracing the full self-motion loop and selecting the config with the
    // LARGEST joint-limit margin among those that satisfy d>=d_safe AND margin>=m_q. Unlike the
    // max-d argmax (which sits AT the joint-limit boundary and therefore violates the user's
    // q∈[q_min+m_q, q_max-m_q] constraint), this section is interior to the limits by m_q, so an
    // online projection can apply a joint-limit clamp without snapping the config off the manifold.
    // Selection is per-point independent (parallelizable); continuity is verified by Q1.
    void build_manifold_safe(const fs::path &outdir, int n_threads) {
        const double th_min = -0.8726646259971648, th_max = 0.8726646259971648;  // ±50°
        const double s_min = -0.16, s_max = 0.0;                                  // m
        const int n_theta = 101, n_s = 65;
        const size_t N = size_t(n_theta) * n_s;

        std::vector<float> qL(N * 7), qR(N * 7), dcont(N);
        std::vector<uint8_t> has(N, 0), reachable(N, 0);

        std::atomic<size_t> next{0}, done{0}, n_has{0}, n_reach{0};
        std::mutex cout_mu;

        auto th_at = [&](int i) { return th_min + double(i) * (th_max - th_min) / (n_theta - 1); };
        auto s_at = [&](int j) { return s_min + double(j) * (s_max - s_min) / (n_s - 1); };

        auto t0 = std::chrono::steady_clock::now();
        auto worker = [&]() {
            ClearanceTrajectory run(cfg_path, d_safe, q_margin_target, trace_half, trace_step);
            while (true) {
                size_t idx = next.fetch_add(1, std::memory_order_relaxed);
                if (idx >= N)
                    break;
                int i = int(idx % n_theta), j = int(idx / n_theta);
                double theta = th_at(i), s = s_at(j);

                Q q0[2]{Q(7), Q(7)};
                bool reach = true;
                for (int side = 0; side < 2; side++)
                    if (!run.ik_multi_seed(side, theta, s, run.seeds[side], atlas_seed(idx, side, 0), q0[side])) {
                        reach = false;
                        break;
                    }
                if (!reach) {
                    reachable[idx] = 0;
                } else {
                    reachable[idx] = 1;
                    n_reach.fetch_add(1, std::memory_order_relaxed);
                    TraceResult tr[2];
                    for (int side = 0; side < 2; side++) {
                        run.ik[side]->setSeed(atlas_seed(idx, side, 1000));
                        run.self_motion_trace_ex(side, q0[side], q0[1 - side], theta, s, run.handle[side].M, tr[side]);
                    }
                    if (tr[0].has_safe && tr[1].has_safe) {
                        has[idx] = 1;
                        n_has.fetch_add(1, std::memory_order_relaxed);
                        for (int k = 0; k < 7; k++) {
                            qL[idx * 7 + k] = float(tr[0].q_safe(k));
                            qR[idx * 7 + k] = float(tr[1].q_safe(k));
                        }
                        dcont[idx] = float(std::min(tr[0].d_safe_at, tr[1].d_safe_at));
                    } else {
                        // no safe+margin config on this loop: fall back to the max-d config so the
                        // section stays defined (flagged by has[idx]=0 for the feasibility report)
                        for (int k = 0; k < 7; k++) {
                            qL[idx * 7 + k] = float(tr[0].q_max(k));
                            qR[idx * 7 + k] = float(tr[1].q_max(k));
                        }
                        dcont[idx] = float(std::min(tr[0].d_max, tr[1].d_max));
                    }
                }

                size_t k = done.fetch_add(1, std::memory_order_relaxed) + 1;
                if (k % 500 == 0 || k == N) {
                    std::lock_guard<std::mutex> lk(cout_mu);
                    double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                    std::cout << "  manifold_safe " << k << "/" << N << "  (" << std::fixed << std::setprecision(0)
                              << el << " s)" << std::endl;
                }
            }
        };

        std::cout << "building margin-respecting safe manifold: " << N << " points on " << n_threads << " threads"
                  << std::endl;
        std::vector<std::thread> pool;
        for (int t = 0; t < n_threads; t++)
            pool.emplace_back(worker);
        for (auto &t : pool)
            t.join();
        double wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

        auto write_f32 = [&](const char *name, const std::vector<float> &v) {
            std::ofstream f(outdir / name, std::ios::binary);
            f.write(reinterpret_cast<const char *>(v.data()), std::streamsize(v.size() * sizeof(float)));
        };
        write_f32("manifold_qL.bin", qL);
        write_f32("manifold_qR.bin", qR);
        write_f32("manifold_dcont.bin", dcont);

        // Q1 continuity of the safe section (θ- and s-neighbors), plus the safety fraction.
        auto q_at = [&](const std::vector<float> &g, size_t idx) {
            Q q(7);
            for (int k = 0; k < 7; k++)
                q(k) = g[idx * 7 + k];
            return q;
        };
        auto dq_abs = [](const Q &a, const Q &b) {
            double d = 0;
            for (int k = 0; k < 7; k++)
                d = std::max(d, std::fabs(a(k) - b(k)));
            return d;
        };
        double mq_th = 0, mq_s = 0, sq_th = 0, sq_s = 0;
        size_t n_th = 0, n_sn = 0;
        for (int j = 0; j < n_s; j++)
            for (int i = 0; i < n_theta; i++) {
                size_t idx = size_t(j) * n_theta + i;
                if (!reachable[idx])
                    continue;
                if (i > 0 && reachable[idx - 1]) {
                    double d = std::max(dq_abs(q_at(qL, idx), q_at(qL, idx - 1)), dq_abs(q_at(qR, idx), q_at(qR, idx - 1)));
                    mq_th = std::max(mq_th, d);
                    sq_th += d;
                    n_th++;
                }
                if (j > 0 && reachable[idx - n_theta]) {
                    double d = std::max(dq_abs(q_at(qL, idx), q_at(qL, idx - n_theta)),
                                        dq_abs(q_at(qR, idx), q_at(qR, idx - n_theta)));
                    mq_s = std::max(mq_s, d);
                    sq_s += d;
                    n_sn++;
                }
            }
        std::cout << std::defaultfloat << std::setprecision(5);
        std::cout << "manifold_safe done: " << N << " points in " << wall_s << " s (" << n_threads << " threads)\n"
                  << "  reachable " << n_reach.load() << "/" << N << ", safe+margin config exists " << n_has.load()
                  << "/" << N << " (" << 100.0 * n_has.load() / N << "%)\n"
                  << "  Q1 continuity  Δq_θ-neighbor: max " << mq_th << " rad, mean " << (n_th ? sq_th / n_th : 0.0)
                  << "  |  Δq_s-neighbor: max " << mq_s << " rad, mean " << (n_sn ? sq_s / n_sn : 0.0) << "\n";
        std::cout << "wrote " << outdir / "manifold_*.bin" << "\n";
    }

    // T5 offline build: construct the CONTINUOUS safe-manifold SECTION Q_cont(θ,s) ∈ R^14 by a
    // deterministic raster sweep (s outer, θ inner) with continuation IK — each point is seeded
    // from its already-built θ-neighbor (or the θ_min column of the previous s-row), so the
    // section follows ONE branch continuously instead of an independent per-point argmax. This
    // is the antithesis of the atlas's max-d argmax and reveals (a) how continuous a single
    // branch is (Q1) and (b) where that continuous branch drops below d_safe — a branch switch
    // would be needed there (the deferred smooth-selection step, not this first cut).
    void build_manifold(const fs::path &outdir) {
        const double th_min = -0.8726646259971648, th_max = 0.8726646259971648;  // ±50°
        const double s_min = -0.16, s_max = 0.0;                                  // m
        const int n_theta = 101, n_s = 65;
        const size_t N = size_t(n_theta) * n_s;

        std::vector<float> qL(N * 7), qR(N * 7), dL(N), dR(N), dcont(N);
        std::vector<uint8_t> safe(N, 0);

        auto th_at = [&](int i) { return th_min + double(i) * (th_max - th_min) / (n_theta - 1); };
        auto s_at = [&](int j) { return s_min + double(j) * (s_max - s_min) / (n_s - 1); };

        Q col0L = seeds[0], col0R = seeds[1], out(7);
        int n_safe = 0, n_ikfallback = 0;
        auto t0 = std::chrono::steady_clock::now();

        for (int j = 0; j < n_s; j++) {
            double s = s_at(j);
            Q refL = col0L, refR = col0R;  // start-of-row reference = previous row's θ_min config
            for (int i = 0; i < n_theta; i++) {
                double th = th_at(i);
                size_t idx = size_t(j) * n_theta + i;
                Q ql(7), qr(7);
                bool okL = ik[0]->CartToJnt(refL, target(0, th, s), out) >= 0;
                if (!okL)
                    okL = ik[0]->CartToJnt(seeds[0], target(0, th, s), out) >= 0;
                if (!okL) {
                    okL = ik_multi_seed(0, th, s, refL, atlas_seed(idx, 0, 0), out);
                    n_ikfallback++;
                }
                ql = okL ? out : refL;
                bool okR = ik[1]->CartToJnt(refR, target(1, th, s), out) >= 0;
                if (!okR)
                    okR = ik[1]->CartToJnt(seeds[1], target(1, th, s), out) >= 0;
                if (!okR) {
                    okR = ik_multi_seed(1, th, s, refR, atlas_seed(idx, 1, 0), out);
                    n_ikfallback++;
                }
                qr = okR ? out : refR;

                set_config(ql, qr, th, s);
                double d_0 = wall_clearance(0), d_1 = wall_clearance(1);
                double d = std::min(d_0, d_1);
                dL[idx] = float(d_0);
                dR[idx] = float(d_1);
                dcont[idx] = float(d);
                safe[idx] = (d >= d_safe) ? 1 : 0;
                if (safe[idx])
                    n_safe++;
                for (int k = 0; k < 7; k++) {
                    qL[idx * 7 + k] = float(ql(k));
                    qR[idx * 7 + k] = float(qr(k));
                }
                if (i == 0) {
                    col0L = ql;
                    col0R = qr;
                }
                refL = ql;
                refR = qr;
            }
            if (j % 16 == 0)
                std::cout << "  manifold s-row " << j << "/" << n_s << "\n";
        }
        double wall_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

        // --- Q1: grid continuity, both neighbor directions, both arms (raw max-abs, rad) ---
        auto q_at = [&](const std::vector<float> &g, size_t idx) {
            Q q(7);
            for (int k = 0; k < 7; k++)
                q(k) = g[idx * 7 + k];
            return q;
        };
        auto dq_abs = [](const Q &a, const Q &b) {
            double d = 0;
            for (int k = 0; k < 7; k++)
                d = std::max(d, std::fabs(a(k) - b(k)));
            return d;
        };
        double max_dq_th = 0, max_dq_s = 0, sum_th = 0, sum_s = 0;
        size_t n_th = 0, n_sn = 0;
        for (int j = 0; j < n_s; j++)
            for (int i = 0; i < n_theta; i++) {
                size_t idx = size_t(j) * n_theta + i;
                if (i > 0) {
                    double d = std::max(dq_abs(q_at(qL, idx), q_at(qL, idx - 1)),
                                        dq_abs(q_at(qR, idx), q_at(qR, idx - 1)));
                    max_dq_th = std::max(max_dq_th, d);
                    sum_th += d;
                    n_th++;
                }
                if (j > 0) {
                    size_t u = idx - n_theta;
                    double d = std::max(dq_abs(q_at(qL, idx), q_at(qL, u)),
                                        dq_abs(q_at(qR, idx), q_at(qR, u)));
                    max_dq_s = std::max(max_dq_s, d);
                    sum_s += d;
                    n_sn++;
                }
            }

        auto write_f32 = [&](const char *name, const std::vector<float> &v) {
            std::ofstream f(outdir / name, std::ios::binary);
            f.write(reinterpret_cast<const char *>(v.data()), std::streamsize(v.size() * sizeof(float)));
        };
        auto write_u8 = [&](const char *name, const std::vector<uint8_t> &v) {
            std::ofstream f(outdir / name, std::ios::binary);
            f.write(reinterpret_cast<const char *>(v.data()), std::streamsize(v.size() * sizeof(uint8_t)));
        };
        write_f32("manifold_qL.bin", qL);
        write_f32("manifold_qR.bin", qR);
        write_f32("manifold_dL.bin", dL);
        write_f32("manifold_dR.bin", dR);
        write_f32("manifold_dcont.bin", dcont);
        write_u8("manifold_safe.bin", safe);
        {
            std::ofstream f(outdir / "manifold.csv");
            f << "theta_deg,s_mm,dL,dR,dcont,safe\n";
            f.precision(8);
            for (int j = 0; j < n_s; j++)
                for (int i = 0; i < n_theta; i++) {
                    size_t idx = size_t(j) * n_theta + i;
                    f << th_at(i) / rad << ',' << s_at(j) * 1e3 << ',' << dL[idx] * 1e3 << ',' << dR[idx] * 1e3 << ','
                      << dcont[idx] * 1e3 << ',' << int(safe[idx]) << '\n';
                }
        }

        std::cout << "manifold done: " << N << " points in " << wall_s << " s\n"
                  << "  safe (continuous branch) " << n_safe << "/" << N << " (" << 100.0 * n_safe / N
                  << "%), IK-fallback " << n_ikfallback << "\n"
                  << "  Q1 continuity  Δq_θ-neighbor: max " << max_dq_th << " rad, mean "
                  << (n_th ? sum_th / n_th : 0.0) << "\n"
                  << "                 Δq_s-neighbor: max " << max_dq_s << " rad, mean "
                  << (n_sn ? sum_s / n_sn : 0.0) << "\n";
        std::cout << "wrote " << outdir / "manifold_*.bin" << ", " << outdir / "manifold.csv" << "\n";
    }

    // T5 validation (the user's three questions, answered on the built section):
    //   Q1 continuity is re-derived here from the section files (and reported by build_manifold);
    //   Q2 how far is the bilinear lookup from the task manifold? (e_0 = grasp closure error);
    //   Q3 does a FIXED 1-step / 2-step Newton projection recover e_T<ε, d≥d_safe, q∈limits?
    void check_manifold(const fs::path &outdir, int K) {
        Manifold M;
        if (!M.load(outdir)) {
            std::cerr << "check_manifold: failed to load manifold from " << outdir << "\n";
            return;
        }
        const int n_theta = M.n_theta, n_s = M.n_s;

        auto q_at = [&](const std::vector<float> &g, size_t idx) {
            Q q(7);
            for (int k = 0; k < 7; k++)
                q(k) = g[idx * 7 + k];
            return q;
        };
        auto dq_abs = [](const Q &a, const Q &b) {
            double d = 0;
            for (int k = 0; k < 7; k++)
                d = std::max(d, std::fabs(a(k) - b(k)));
            return d;
        };
        double max_dq_th = 0, max_dq_s = 0;
        for (int j = 0; j < n_s; j++)
            for (int i = 0; i < n_theta; i++) {
                size_t idx = size_t(j) * n_theta + i;
                if (i > 0)
                    max_dq_th = std::max(max_dq_th, std::max(dq_abs(q_at(M.qL, idx), q_at(M.qL, idx - 1)),
                                                             dq_abs(q_at(M.qR, idx), q_at(M.qR, idx - 1))));
                if (j > 0)
                    max_dq_s = std::max(max_dq_s, std::max(dq_abs(q_at(M.qL, idx), q_at(M.qL, idx - n_theta)),
                                                           dq_abs(q_at(M.qR, idx), q_at(M.qR, idx - n_theta))));
            }

        // grasp closure error (position max-abs [m], orientation max-abs [rad]) per arm, in the
        // task_error() convention: TCP site vs handle site. (NOT the flange target: target() is
        // the flange frame, offset from the gripper-tip tcp_site by the tool transform ~100 mm.)
        auto closure = [&](int side) {
            KDL::Vector pc = site_pos(tcp_site[side]);
            KDL::Rotation Rc = site_rot(tcp_site[side]);
            KDL::Vector hp = site_pos(handle_site[side]);
            KDL::Rotation Rh = site_rot(handle_site[side]);
            KDL::Vector pe = pc - hp;
            KDL::Vector oe = (Rc * Rh.Inverse()).GetRot();
            return std::array<double, 2>{std::max({std::fabs(pe.x()), std::fabs(pe.y()), std::fabs(pe.z())}),
                                         std::max({std::fabs(oe.x()), std::fabs(oe.y()), std::fabs(oe.z())})};
        };
        // one fixed Newton step (damped least-squares Δq, joint-limit clamped), with
        // e = [handle_pos − tcp_pos; (R_handle · R_tcp⁻¹).rot] so Δq drives the TCP onto the handle.
        auto newton = [&](const Q &ql, const Q &qr, double th, double s, Q &ql_out, Q &qr_out) {
            manifold_newton_step(ql, qr, th, s, ql_out, qr_out);
        };

        std::mt19937_64 rng(123456789u);
        std::uniform_real_distribution<double> U(0.0, 1.0);
        std::vector<double> e0, e1, e2, d0, d1, d2;
        int n1_ok = 0, n2_ok = 0, n_d_safe = 0;
        const double eps_T = 1e-4, eps_T2 = 1e-5;

        std::ofstream csv(outdir / "manifold_check.csv");
        csv << "th,s,e0,e1,e2,d0,d1,d2\n";
        csv.precision(10);

        for (int k = 0; k < K; k++) {
            double th = M.th_min + U(rng) * (M.th_max - M.th_min);
            double s = M.s_min + U(rng) * (M.s_max - M.s_min);
            std::array<double, 14> q14;
            if (!M.lookup(th, s, q14))
                continue;
            Q ql(7), qr(7);
            for (int j = 0; j < 7; j++) {
                ql(j) = q14[j];
                qr(j) = q14[7 + j];
            }
            set_config(ql, qr, th, s);
            double e_0 = std::max(closure(0)[0], closure(1)[0]);
            double d_0 = std::min(wall_clearance(0), wall_clearance(1));
            Q ql1(7), qr1(7), ql2(7), qr2(7);
            newton(ql, qr, th, s, ql1, qr1);
            set_config(ql1, qr1, th, s);
            double e_1 = std::max(closure(0)[0], closure(1)[0]);
            double d_1 = std::min(wall_clearance(0), wall_clearance(1));
            newton(ql1, qr1, th, s, ql2, qr2);
            set_config(ql2, qr2, th, s);
            double e_2 = std::max(closure(0)[0], closure(1)[0]);
            double d_2 = std::min(wall_clearance(0), wall_clearance(1));

            e0.push_back(e_0);
            e1.push_back(e_1);
            e2.push_back(e_2);
            d0.push_back(d_0);
            d1.push_back(d_1);
            d2.push_back(d_2);
            if (e_1 < eps_T)
                n1_ok++;
            if (e_2 < eps_T2)
                n2_ok++;
            if (d_2 >= d_safe)
                n_d_safe++;
            csv << th << ',' << s << ',' << e_0 << ',' << e_1 << ',' << e_2 << ',' << d_0 << ',' << d_1 << ',' << d_2
                << '\n';
        }
        csv.close();

        auto pct = [](std::vector<double> v, double p) {
            if (v.empty())
                return 0.0;
            std::sort(v.begin(), v.end());
            return v[std::min(size_t(p * (v.size() - 1)), v.size() - 1)];
        };
        int n = (int)e0.size();
        std::cout << "manifold check: " << n << " off-grid samples\n"
                  << "  Q1 continuity  max Δq θ-neighbor " << max_dq_th << " rad, s-neighbor " << max_dq_s << " rad\n"
                  << "  Q2 interpolation  e_0 [m]: median " << pct(e0, 0.5) << ", P95 " << pct(e0, 0.95) << ", max "
                  << pct(e0, 1.0) << "\n"
                  << "  Q3 1-step e_1 [m]: median " << pct(e1, 0.5) << ", P95 " << pct(e1, 0.95) << ", max "
                  << pct(e1, 1.0) << "  -> P(e_1<" << eps_T << ")=" << (n ? 100.0 * n1_ok / n : 0.0) << "%\n"
                  << "     2-step e_2 [m]: median " << pct(e2, 0.5) << ", P95 " << pct(e2, 0.95) << ", max "
                  << pct(e2, 1.0) << "  -> P(e_2<" << eps_T2 << ")=" << (n ? 100.0 * n2_ok / n : 0.0) << "%\n"
                  << "     clearance: d_0 median " << pct(d0, 0.5) * 1e3 << " mm, d_2 median " << pct(d2, 0.5) * 1e3
                  << " mm, P(d_2>=d_safe)=" << (n ? 100.0 * n_d_safe / n : 0.0) << "%\n";
        std::cout << "wrote " << outdir / "manifold_check.csv" << "\n";
    }
};

int main(int argc, char **argv) {
    try {
        if (argc < 3 || argc > 9) {
            std::cerr << "Usage: aviator_clearance_trajectory CONFIG OUTPUT_DIR [D_SAFE=0.005] [Q_MARGIN=0.03] "
                         "[TRACE_HALF=10] [TRACE_STEP=0.1] [TASKS=roll_pos,roll_neg,pull,combined] [DT=0.02]\n";
            return 2;
        }
        double d_safe = argc > 3 ? std::atof(argv[3]) : 0.005;
        double q_margin = argc > 4 ? std::atof(argv[4]) : 0.03;
        int trace_half = argc > 5 ? std::atoi(argv[5]) : 10;
        double trace_step = argc > 6 ? std::atof(argv[6]) : 0.1;
        ClearanceTrajectory run(fs::absolute(argv[1]), d_safe, q_margin, trace_half, trace_step);
        if (argc > 8)
            run.dt = std::atof(argv[8]);
        fs::path outdir = fs::absolute(argv[2]);
        fs::create_directories(outdir);

        std::cout << "gap_inner=" << run.wall_gap_inner() << " m, d_safe=" << d_safe << ", q_margin=" << q_margin
                  << ", trace_half=" << trace_half << ", qddot_max=" << run.qddot_max << "\n";

        struct TaskSpec { const char *tag; double th0, s0, th1, s1, dur; };
        const TaskSpec specs[] = {
            {"roll_pos", 0.0, 0.0, 0.87266, 0.0, 3.0},
            {"roll_neg", 0.0, 0.0, -0.87266, 0.0, 3.0},
            {"pull", 0.0, 0.0, 0.0, -0.16, 3.0},
            {"combined", 0.0, 0.0, 0.87266, -0.16, 8.0},
        };
        std::set<std::string> only;
        if (argc > 7) {
            std::string list = argv[7];
            size_t p = 0;
            while (p < list.size()) {
                size_t c = list.find(',', p);
                only.insert(list.substr(p, c == std::string::npos ? std::string::npos : c - p));
                if (c == std::string::npos)
                    break;
                p = c + 1;
            }
        }
        if (only.count("phi_sweep")) {
            run.phi_sweep(outdir);
            return 0;
        }
        if (only.count("phi_ablation")) {
            run.phi_ablation(outdir);
            return 0;
        }
        if (only.count("release_ablation")) {
            run.release_ablation(outdir);
            return 0;
        }
        if (only.count("online_b0")) {
            run.online(0, outdir);
            return 0;
        }
        if (only.count("online_b3")) {
            run.online(1, outdir);
            return 0;
        }
        if (only.count("online_b4")) {
            run.online(2, outdir);
            return 0;
        }
        if (only.count("debug_qp")) {
            run.debug_qp(outdir);
            return 0;
        }
        if (only.count("benchmark")) {
            // argv[8] doubles as the sample count here (DT is unused in this mode).
            run.benchmark(outdir, argc > 8 ? std::atoi(argv[8]) : 100);
            return 0;
        }
        if (only.count("atlas")) {
            // argv[8] doubles as the thread count here (DT is unused in this mode).
            run.build_atlas(outdir, argc > 8 ? std::atoi(argv[8]) : 16);
            return 0;
        }
        if (only.count("compare")) {
            // argv[8] doubles as the cycle count here; the atlas is read from outdir
            // (build it first with TASKS=atlas into the same OUTPUT_DIR).
            run.compare(outdir, argc > 8 ? std::atoi(argv[8]) : 10000, outdir);
            return 0;
        }
        if (only.count("record_t0")) {
            // argv[8] doubles as the per-profile knot count here (50 Hz).
            run.record_t0(outdir, argc > 8 ? std::atoi(argv[8]) : 250);
            return 0;
        }
        if (only.count("manifold")) {
            // T5 offline: build the continuous safe-manifold section Q_g(θ,s) ∈ R^14.
            run.build_manifold(outdir);
            return 0;
        }
        if (only.count("manifold_safe")) {
            // T5 offline: margin-respecting safe-manifold section (max-margin among d>=d_safe).
            // argv[8] doubles as the thread count here.
            run.build_manifold_safe(outdir, argc > 8 ? std::atoi(argv[8]) : 16);
            return 0;
        }
        if (only.count("check_manifold")) {
            // argv[8] doubles as the off-grid sample count here (default 5000).
            run.check_manifold(outdir, argc > 8 ? std::atoi(argv[8]) : 5000);
            return 0;
        }
        std::vector<TaskSummary> all;
        for (auto &sp : specs)
            if (only.empty() || only.count(sp.tag))
                all.push_back(run.run_task(outdir, sp.tag, sp.th0, sp.s0, sp.th1, sp.s1, sp.dur));

        std::ofstream sum(outdir / "summary.csv");
        sum.precision(10);
        sum << "task,gap_inner,d_safe,q_margin,P_succ,d_base_min,d_static_min,d_b3_min,mean_static_minus_b3,"
               "max_dq,max_ddq,min_q_margin,J_interv,n_none,n_clearance,n_dynamic,n_joint\n";
        for (auto &S : all) {
            std::cout << run.summarize(S) << "\n";
            sum << S.tag << ',' << run.wall_gap_inner() << ',' << d_safe << ',' << q_margin << ','
                << double(S.n_safe) / S.N << ',' << S.d_base_min << ',' << S.d_static_min << ',' << S.d_b3_min << ','
                << S.mean_static_minus_b3 << ',' << S.max_dq << ',' << S.max_ddq << ',' << S.min_qm << ',' << S.J_interv
                << ',' << S.n_none << ',' << S.n_clearance << ',' << S.n_dynamic << ',' << S.n_joint << '\n';
        }
        sum.close();
        std::cout << "Wrote " << outdir / "trajectory_*.csv" << " and " << outdir / "summary.csv" << "\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
