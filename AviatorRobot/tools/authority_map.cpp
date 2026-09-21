// Offline failure-mode map: no shared memory, drive commands or live-config edits.
//
// Sweeps the passive-wheel task space (theta, s) on a grid. At every point it
// measures the KINEMATIC directional task authority (wall-agnostic, see below)
//     A_kin(q,x;v) = max lambda s.t. lambda*v in U(q,x),
//     U(q,x) = { xdot : exists qdot, J_arm qdot = J_h xdot, |qdot_i| <= qmax_i }
// for the four unit directions v in {+theta,-theta,+s,-s}. A_kin depends only on
// the Jacobian and joint velocity limits, so it is insensitive to the cockpit
// walls by construction -- it answers "can the arm physically produce this task
// velocity", NOT "can it do so without hitting anything".
//
// Because the real failure mode here is collision (a wide pinned elbow), the
// primary diagnostics are clearance-based rather than authority-based:
//   d_base : true signed clearance to the nearest wall at the baseline (IK
//            continuation) posture, per-arm min
//   d_best : best (max) wall clearance reachable over the nullspace self-motion
//   R_d    : d_best - d_base  (redundancy recovery margin)
//   C      : 0 = baseline collision-free, 1 = baseline collides but redundancy
//            recovers, 2 = no collision-free posture exists (geometric limit)
// Authority is still reported as A_exec (baseline) and A_max / A_min (best /
// worst over the self-motion sweep), with A_exec masked to NaN wherever the
// baseline posture already penetrates a wall.
//
// Reuses the self-contained offline pattern of tools/search_grasp.cpp.
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
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

// Four task directions: +theta, -theta, +s, -s.
static const double DIRS[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

class AuthorityMap {
  public:
    YAML::Node config, grasp, posture;
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> m{nullptr, mj_deleteModel};
    std::unique_ptr<mjData, decltype(&mj_deleteData)> d{nullptr, mj_deleteData};
    std::unique_ptr<TRAC_IK::TRAC_IK> ik[2];
    KDL::Frame origin, tool;
    KDL::Frame handle[2];
    Q seeds[2]{Q(7), Q(7)}, lower[2]{Q(7), Q(7)}, upper[2]{Q(7), Q(7)};
    int joint[2][7]{}, wheel[2]{}, tcp_site[2]{}, handle_site[2]{};
    double qmax[2][7]{};
    int n_theta = 40, n_s = 20, redundancy_half = 10;
    double redundancy_step = 0.1;
    std::vector<int> wall_geoms; // geom ids of the two virtual cockpit walls

    explicit AuthorityMap(const fs::path &path, int n_theta, int n_s, int redundancy_half)
        : n_theta(n_theta), n_s(n_s), redundancy_half(redundancy_half) {
        config = YAML::LoadFile(path.string());
        auto resolve = [&](const char *key) { return path.parent_path() / config[key].as<std::string>(); };
        grasp = YAML::LoadFile(resolve("grasp").string());
        posture = YAML::LoadFile(resolve("posture").string());
        tool = frame(grasp["tool"]);
        for (int s = 0; s < 2; s++)
            handle[s] = frame(grasp[s ? "right" : "left"]);

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
            if (name.find("aviator_wall") != std::string::npos)
                wall_geoms.push_back(g);
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
                joint[s][j] =
                    mj_name2id(m.get(), mjOBJ_JOINT, (prefix + "joint_" + std::to_string(j + 1)).c_str());
                lower[s](j) = m->jnt_range[2 * joint[s][j]];
                upper[s](j) = m->jnt_range[2 * joint[s][j] + 1];
                seeds[s](j) = posture[side + "_approach_seed_deg"][j].as<double>() * rad;
            }
            const double margin = posture["joint2_planning_margin_deg"].as<double>();
            lower[s](1) = (posture["joint2_limits_deg"][0].as<double>() + margin) * rad;
            upper[s](1) = (posture["joint2_limits_deg"][1].as<double>() - margin) * rad;
            ik[s] = std::make_unique<TRAC_IK::TRAC_IK>(chain, lower[s], upper[s], .005, 7e-7, TRAC_IK::Distance);
            tcp_site[s] = mj_name2id(m.get(), mjOBJ_SITE, (side + "_tcp").c_str());
            handle_site[s] = mj_name2id(m.get(), mjOBJ_SITE, (side + "_handle").c_str());
            if (tcp_site[s] < 0 || handle_site[s] < 0)
                throw std::runtime_error("Missing TCP/handle site");
            read_velocity_limits(resolve("urdf").string(), prefix, qmax[s]);
        }
    }

    // Velocity limits live in the URDF <limit velocity="..."/> (rad/s). Both arms share them.
    static void read_velocity_limits(const std::string &urdf_path, const std::string &prefix, double out[7]) {
        std::ifstream in(urdf_path);
        std::string txt((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        for (int j = 0; j < 7; j++) {
            auto name = prefix + "joint_" + std::to_string(j + 1);
            auto pos = txt.find("<joint name=\"" + name + "\"");
            auto vel = txt.find("velocity=\"", pos == std::string::npos ? 0 : pos);
            if (vel == std::string::npos)
                throw std::runtime_error("Missing velocity limit for " + name);
            out[j] = std::atof(txt.c_str() + vel + 10);
            if (!std::isfinite(out[j]) || out[j] <= 0)
                out[j] = 1.0;
        }
    }

    KDL::Frame target(int side, double theta, double s) {
        return origin * KDL::Frame(KDL::Rotation::RotZ(theta), KDL::Vector(0, 0, s)) * handle[side] *
               tool.Inverse();
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
    // -1 = not an arm geom, 0 = left arm, 1 = right arm (same convention as search_grasp.cpp).
    int owner(int geom) {
        const char *raw = mj_id2name(m.get(), mjOBJ_BODY, m->geom_bodyid[geom]);
        std::string name = raw ? raw : "";
        if (name.find("07L-") != std::string::npos || name == "left_gripper")
            return 0;
        if (name.find("07R-") != std::string::npos || name == "right_gripper")
            return 1;
        return -1;
    }
    // Collision of the given arm (side<0 => any arm) against the world, including the virtual walls.
    bool collision(int side = -1) {
        for (int k = 0; k < d->ncon; k++) {
            auto &c = d->contact[k];
            if (c.dist >= -.001)
                continue;
            int a = owner(c.geom1), b = owner(c.geom2);
            if (a < 0 && b < 0)
                continue; // world-vs-world (walls are static, arms are the moving part)
            if (side >= 0 && ((a != side && b != side) || a == 1 - side || b == 1 - side))
                continue;
            return true;
        }
        return false;
    }
    // Joint-limit margin (rad) of the current arm configuration, min over both arms.
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
    // True signed clearance (m) from one arm's collision geoms to the nearest virtual
    // wall. Positive = separated, negative = penetration. Capped at distmax=1.0, so a
    // control run with no walls returns +1.0 (no constraint). The two arms flank their
    // own wall independently (left->+y wall, right->-y wall), so clearance is per arm.
    double wall_clearance(int side) {
        double dmin = 1.0;
        for (int g = 0; g < m->ngeom; g++) {
            if (owner(g) != side)
                continue;
            for (int w : wall_geoms)
                dmin = std::min(dmin, (double)mj_geomDistance(m.get(), d.get(), g, w, 1.0, nullptr));
        }
        return dmin;
    }

    struct SideResult {
        double sigma_min = 0;
        double a[4]{};
        Eigen::Matrix<double, 7, 1> n; // nullspace basis (self-motion direction) at this config
    };
    // Directional authority for one arm from the CURRENT d state (already mj_forward'ed).
    SideResult evaluate_side(int side) {
        SideResult out;
        int nv = m->nv;
        Eigen::Matrix<double, 6, 7> J;
        Eigen::Matrix<double, 6, 2> Jh;
        std::array<mjtNum, 3 * 16> jacp_tcp{}, jacr_tcp{}, jacp_h{}, jacr_h{};
        mj_jacSite(m.get(), d.get(), jacp_tcp.data(), jacr_tcp.data(), tcp_site[side]);
        mj_jacSite(m.get(), d.get(), jacp_h.data(), jacr_h.data(), handle_site[side]);
        for (int j = 0; j < 7; j++) {
            int col = m->jnt_dofadr[joint[side][j]];
            for (int r = 0; r < 3; r++) {
                J(r, j) = jacp_tcp[r * nv + col];
                J(3 + r, j) = jacr_tcp[r * nv + col];
            }
        }
        for (int j = 0; j < 2; j++) {
            int col = m->jnt_dofadr[wheel[j]];
            for (int r = 0; r < 3; r++) {
                Jh(r, j) = jacp_h[r * nv + col];
                Jh(3 + r, j) = jacr_h[r * nv + col];
            }
        }
        Eigen::JacobiSVD<Eigen::Matrix<double, 6, 7>> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
        out.sigma_min = svd.singularValues()(5);
        double sigma0 = svd.singularValues()(0);
        out.n = svd.matrixV().col(6);
        bool singular = out.sigma_min < 1e-5 * std::max(sigma0, 1e-9);

        for (int dir = 0; dir < 4; dir++) {
            Eigen::Matrix<double, 6, 1> rhs = Jh * Eigen::Vector2d(DIRS[dir][0], DIRS[dir][1]);
            Eigen::Matrix<double, 7, 1> b0 = svd.solve(rhs);
            if (singular) {
                // Rank-deficient fallback: min-norm solution is a valid lower bound.
                double a = 1e9;
                for (int i = 0; i < 7; i++)
                    a = std::min(a, qmax[side][i] / std::max(std::fabs(b0(i)), 1e-12));
                out.a[dir] = a;
                continue;
            }
            auto feasible = [&](double lam) {
                double lo = -1e9, hi = 1e9;
                for (int i = 0; i < 7; i++) {
                    double bi = lam * b0(i), ni = out.n(i), qm = qmax[side][i];
                    if (std::fabs(ni) < 1e-12) {
                        if (std::fabs(bi) > qm + 1e-12)
                            return false;
                        continue;
                    }
                    double loi, hii;
                    if (ni > 0) {
                        loi = (-qm - bi) / ni;
                        hii = (qm - bi) / ni;
                    } else {
                        loi = (qm - bi) / ni;
                        hii = (-qm - bi) / ni;
                    }
                    lo = std::max(lo, loi);
                    hi = std::min(hi, hii);
                }
                return lo <= hi;
            };
            double lam_hi = 1.0;
            while (feasible(lam_hi) && lam_hi < 1e6)
                lam_hi *= 2;
            double lam_lo = 0.0;
            for (int it = 0; it < 50; it++) {
                double mid = 0.5 * (lam_lo + lam_hi);
                if (feasible(mid))
                    lam_lo = mid;
                else
                    lam_hi = mid;
            }
            out.a[dir] = lam_lo;
        }
        return out;
    }

    // Sweep one arm's self-motion around a baseline config; report per-direction
    // max/min authority over collision-free candidates, the candidate count, and the
    // best (max) wall clearance this arm can reach while the other arm stays frozen.
    void sweep_arm(int side, const Q &baseline, const Q &other, const Eigen::Matrix<double, 7, 1> &n,
                   double theta, double s, double amax[4], double amin[4], int &count, double &d_best) {
        Q seed(7), out(7);
        for (int k = -redundancy_half; k <= redundancy_half; k++) {
            for (int j = 0; j < 7; j++)
                seed(j) = std::clamp(baseline(j) + redundancy_step * k * n(j), lower[side](j), upper[side](j));
            if (ik[side]->CartToJnt(seed, target(side, theta, s), out) < 0)
                continue;
            set_config(side ? other : out, side ? out : other, theta, s);
            if (collision(side))
                continue;
            d_best = std::max(d_best, wall_clearance(side));
            auto r = evaluate_side(side);
            for (int dir = 0; dir < 4; dir++) {
                amax[dir] = std::max(amax[dir], r.a[dir]);
                amin[dir] = std::min(amin[dir], r.a[dir]);
            }
            count++;
        }
    }

    void run(const fs::path &dir) {
        fs::create_directories(dir);
        std::ofstream csv(dir / "authority_grid.csv");
        csv.precision(12);
        csv << "theta,s,ik_ok,n_valid,C,"
               "a_max_theta_p,a_max_theta_m,a_max_s_p,a_max_s_m,"
               "a_min_theta_p,a_min_theta_m,a_min_s_p,a_min_s_m,"
               "a_exec_theta_p,a_exec_theta_m,a_exec_s_p,a_exec_s_m,"
               "d_base,d_best,R_d,q_margin,sigma_min\n";

        const double theta_min = m->jnt_range[2 * wheel[0]], theta_max = m->jnt_range[2 * wheel[0] + 1];
        const double s_min = m->jnt_range[2 * wheel[1]], s_max = m->jnt_range[2 * wheel[1] + 1];
        Q prev[2]{seeds[0], seeds[1]};
        bool have_prev = false;

        for (int is = 0; is < n_s; is++) {
            double s = s_min + (s_max - s_min) * (n_s > 1 ? double(is) / (n_s - 1) : 0.5);
            for (int it = 0; it < n_theta; it++) {
                double theta = theta_min + (theta_max - theta_min) * (n_theta > 1 ? double(it) / (n_theta - 1) : 0.5);
                Q ql(7), qr(7), out(7);
                bool ok = true;
                for (int side = 0; side < 2; side++) {
                    Q seed = have_prev ? prev[side] : seeds[side];
                    if (ik[side]->CartToJnt(seed, target(side, theta, s), out) < 0 &&
                        ik[side]->CartToJnt(seeds[side], target(side, theta, s), out) < 0) {
                        ok = false;
                        break;
                    }
                    (side ? qr : ql) = out;
                }
                if (!ok) {
                    csv << theta << ',' << s << ",0,0,2,"
                           "nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,nan\n";
                    have_prev = false;
                    std::cout << "IK miss theta=" << theta << " s=" << s << "\n";
                    continue;
                }

                set_config(ql, qr, theta, s);
                bool base_collide_L = collision(0), base_collide_R = collision(1);
                bool base_collide = base_collide_L || base_collide_R;
                double d_baseL = wall_clearance(0), d_baseR = wall_clearance(1);
                double d_base = std::min(d_baseL, d_baseR);
                double q_margin = joint_margin();

                double a_exec[4]{}, a_max[4]{1e9, 1e9, 1e9, 1e9}, a_min[4]{1e9, 1e9, 1e9, 1e9};
                double sigma_min = 1e9;
                for (int dir = 0; dir < 4; dir++) {
                    a_max[dir] = -1e9;
                    a_min[dir] = 1e9;
                }
                auto resL = evaluate_side(0), resR = evaluate_side(1);
                for (int dir = 0; dir < 4; dir++)
                    a_exec[dir] = std::min(resL.a[dir], resR.a[dir]);
                sigma_min = std::min(resL.sigma_min, resR.sigma_min);

                int countL = 0, countR = 0;
                double d_bestL = -1e9, d_bestR = -1e9;
                double amaxL[4]{}, aminL[4]{}, amaxR[4]{}, aminR[4]{};
                for (int dir = 0; dir < 4; dir++) {
                    amaxL[dir] = amaxR[dir] = -1e9;
                    aminL[dir] = aminR[dir] = 1e9;
                }
                // Include the baseline config itself as a candidate when collision-free.
                if (!base_collide_L) {
                    for (int dir = 0; dir < 4; dir++) {
                        amaxL[dir] = std::max(amaxL[dir], resL.a[dir]);
                        aminL[dir] = std::min(aminL[dir], resL.a[dir]);
                    }
                    countL = 1;
                    d_bestL = d_baseL;
                }
                if (!base_collide_R) {
                    for (int dir = 0; dir < 4; dir++) {
                        amaxR[dir] = std::max(amaxR[dir], resR.a[dir]);
                        aminR[dir] = std::min(aminR[dir], resR.a[dir]);
                    }
                    countR = 1;
                    d_bestR = d_baseR;
                }
                sweep_arm(0, ql, qr, resL.n, theta, s, amaxL, aminL, countL, d_bestL);
                sweep_arm(1, qr, ql, resR.n, theta, s, amaxR, aminR, countR, d_bestR);
                int n_valid = std::min(countL, countR);
                for (int dir = 0; dir < 4; dir++) {
                    a_max[dir] = std::min(amaxL[dir], amaxR[dir]);
                    a_min[dir] = std::min(aminL[dir], aminR[dir]);
                }
                double d_best = n_valid > 0 ? std::min(d_bestL, d_bestR) : NAN;
                double R_d = n_valid > 0 ? d_best - d_base : NAN;
                // 0 = baseline collision-free, 1 = baseline collides but some redundancy
                // recovers, 2 = no collision-free posture exists (geometric infeasibility).
                int C = !base_collide ? 0 : (n_valid > 0 ? 1 : 2);

                csv << theta << ',' << s << ",1," << n_valid << ',' << C;
                for (int g = 0; g < 3; g++)
                    for (int dir = 0; dir < 4; dir++) {
                        double v = g == 0 ? a_max[dir] : (g == 1 ? a_min[dir] : a_exec[dir]);
                        // Mask the executed authority when the baseline config already
                        // collides (A_kin at a penetrating posture is not a usable number).
                        bool masked = (n_valid == 0 && g != 2) || (g == 2 && base_collide);
                        if (masked)
                            csv << ",nan";
                        else
                            csv << ',' << v;
                    }
                csv << ',' << d_base << ',' << d_best << ',' << R_d << ',' << q_margin << ',' << sigma_min << '\n';
                prev[0] = ql;
                prev[1] = qr;
                have_prev = true;
            }
        }
        write_metadata(dir, theta_min, theta_max, s_min, s_max);
        std::cout << "Wrote " << dir / "authority_grid.csv" << "\n";
    }

    void write_metadata(const fs::path &dir, double tmin, double tmax, double smin, double smax) {
        std::ofstream out(dir / "metadata.json");
        out.precision(15);
        out << "{\n"
            << "  \"grid\": {\"n_theta\": " << n_theta << ", \"n_s\": " << n_s
            << ", \"theta_range\": [" << tmin << ", " << tmax << "], \"s_range\": [" << smin << ", " << smax
            << "]},\n"
            << "  \"redundancy\": {\"half_range\": " << redundancy_half << ", \"step_rad\": " << redundancy_step
            << "},\n"
            << "  \"velocity_limits_rad_s\": [";
        for (int j = 0; j < 7; j++)
            out << (j ? ", " : "") << qmax[0][j];
        out << "],\n  \"walls\": ";
        double gap = wall_gap();
        if (gap >= 0)
            out << "{\"gap_m\": " << gap << "}\n";
        else
            out << "null\n";
        out << "}\n";
    }
    // Distance between the two virtual wall geoms (world frame), or -1 when absent.
    double wall_gap() {
        int a = mj_name2id(m.get(), mjOBJ_GEOM, "aviator_wall_left");
        int b = mj_name2id(m.get(), mjOBJ_GEOM, "aviator_wall_right");
        if (a < 0 || b < 0)
            return -1;
        mjtNum delta[3];
        mju_sub3(delta, d->geom_xpos + 3 * a, d->geom_xpos + 3 * b);
        return mju_norm3(delta);
    }
};

int main(int argc, char **argv) {
    try {
        if (argc < 3 || argc > 6) {
            std::cerr << "Usage: aviator_authority_map CONFIG OUTPUT_DIRECTORY [N_THETA=40] [N_S=20] [K=10]\n";
            return 2;
        }
        int n_theta = argc > 3 ? std::atoi(argv[3]) : 40;
        int n_s = argc > 4 ? std::atoi(argv[4]) : 20;
        int k = argc > 5 ? std::atoi(argv[5]) : 10;
        AuthorityMap map(fs::absolute(argv[1]), n_theta, n_s, k);
        map.run(fs::absolute(argv[2]));
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
