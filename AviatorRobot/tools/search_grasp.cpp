// Offline search: no shared memory, drive commands or edits to live configuration.
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <kdl_parser/kdl_parser.hpp>
#include <memory>
#include <mujoco/mujoco.h>
#include <trac_ik/trac_ik.hpp>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;
constexpr double rad = 3.14159265358979323846 / 180.;
using Q = KDL::JntArray;
KDL::Vector vector(const YAML::Node &n) { return {n[0].as<double>(), n[1].as<double>(), n[2].as<double>()}; }
KDL::Frame frame(const YAML::Node &n) {
    auto q = n["quaternion"];
    return {
        KDL::Rotation::Quaternion(q[1].as<double>(), q[2].as<double>(), q[3].as<double>(), q[0].as<double>()),
        vector(n["position"])};
}
struct Sample {
    double angle, slide, time;
};
struct Candidate {
    KDL::Frame handle;
    double offset = 0, spin = 0, score = 0;
    Q home{7}, approach{7};
    std::vector<Q> path;
    std::vector<Q> approach_path;
};
class Search {
  public:
    YAML::Node config, grasp, posture;
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> m{nullptr, mj_deleteModel};
    std::unique_ptr<mjData, decltype(&mj_deleteData)> d{nullptr, mj_deleteData};
    std::unique_ptr<TRAC_IK::TRAC_IK> ik[2], home_ik[2];
    KDL::Frame origin, tool;
    KDL::Vector centers[2], axes[2];
    KDL::Rotation aligned[2];
    Q seeds[2]{Q(7), Q(7)}, lower[2]{Q(7), Q(7)}, upper[2]{Q(7), Q(7)};
    int joint[2][7]{}, wheel[2]{};
    std::vector<Sample> samples;
    std::vector<Candidate> candidates[2];
    double approach_distance = .06;
    double grip_half_length[2]{};
    std::string failure;
    explicit Search(const fs::path &path) {
        config = YAML::LoadFile(path.string());
        auto resolve = [&](const char *key) { return path.parent_path() / config[key].as<std::string>(); };
        grasp = YAML::LoadFile(resolve("grasp").string());
        posture = YAML::LoadFile(resolve("posture").string());
        tool = frame(grasp["tool"]);
        approach_distance = grasp["approach_distance"].as<double>();
        char error[1024]{};
        m.reset(mj_loadXML(resolve("model").c_str(), nullptr, error, sizeof(error)));
        if (!m)
            throw std::runtime_error(error);
        d.reset(mj_makeData(m.get()));
        mj_forward(m.get(), d.get());
        int b = mj_name2id(m.get(), mjOBJ_BODY, "steering_wheel");
        auto *r = d->xmat + 9 * b;
        auto *p = d->xpos + 3 * b;
        origin = {KDL::Rotation(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]),
                  KDL::Vector(p[0], p[1], p[2])};
        wheel[0] = mj_name2id(m.get(), mjOBJ_JOINT, "roll_input_joint");
        wheel[1] = mj_name2id(m.get(), mjOBJ_JOINT, "pitch_input_joint");
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
            ik[s] =
                std::make_unique<TRAC_IK::TRAC_IK>(chain, lower[s], upper[s], .005, 7e-7, TRAC_IK::Distance);
            auto lo = lower[s], hi = upper[s];
            lo(1) = 90 * rad - 1e-7;
            hi(1) = 90 * rad + 1e-7;
            home_ik[s] = std::make_unique<TRAC_IK::TRAC_IK>(chain, lo, hi, .03, 7e-7, TRAC_IK::Distance);
            auto geometry = grasp["handle_geometry"][side];
            grip_half_length[s] = geometry["half_length"].as<double>();
            centers[s] = vector(geometry["center"]);
            axes[s] = vector(geometry["axis"]);
            if (std::abs(axes[s].Norm() - 1) > 1e-6)
                throw std::runtime_error("Handle axis must be normalized");
            axes[s].Normalize();
            auto x = KDL::Vector(0, 0, 1) - KDL::dot(KDL::Vector(0, 0, 1), axes[s]) * axes[s];
            x.Normalize();
            aligned[s] = KDL::Rotation(x, axes[s] * x, axes[s]);
            // A changed tool config must be regenerated before searching,
            // otherwise collision geometry and the IK tool frame would differ.
            const int tcp = mj_name2id(m.get(), mjOBJ_SITE, (side + "_tcp").c_str());
            const int flange = mj_name2id(m.get(), mjOBJ_BODY, (prefix + "flan_link").c_str());
            const int geom = mj_name2id(m.get(), mjOBJ_GEOM, (side + "_grasp_cylinder").c_str());
            if (tcp < 0 || flange < 0 || geom < 0)
                throw std::runtime_error("Missing cylinder tool");
            auto pose = [](const mjtNum *r, const mjtNum *p) {
                return KDL::Frame(KDL::Rotation(r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8]),
                                  KDL::Vector(p[0], p[1], p[2]));
            };
            auto actual = pose(d->xmat + 9 * flange, d->xpos + 3 * flange).Inverse() *
                          pose(d->site_xmat + 9 * tcp, d->site_xpos + 3 * tcp);
            auto difference = KDL::diff(actual, tool);
            if (difference.vel.Norm() > 1e-8 || difference.rot.Norm() > 1e-8 ||
                std::abs(m->geom_size[3 * geom] - grasp["tool"]["radius"].as<double>()) > 1e-9 ||
                std::abs(m->geom_size[3 * geom + 1] - grasp["tool"]["length"].as<double>() / 2) > 1e-9)
                throw std::runtime_error("Tool config differs from MJCF; regenerate assets first");
        }
        // Time-parametrized complete task; the runtime planner validates again at 20 ms.
        samples.push_back({0, 0, 0});
        Sample from = samples.back();
        for (auto to :
             std::vector<Sample>{{.872, 0, 8}, {-.872, 0, 16}, {0, 0, 8}, {0, -.16, 8}, {0, 0, 8}}) {
            int steps = static_cast<int>(std::ceil(to.time / .2));
            for (int k = 1; k <= steps; k++) {
                double u = double(k) / steps, s = u * u * u * (10 + u * (-15 + 6 * u));
                samples.push_back({from.angle + s * (to.angle - from.angle),
                                   from.slide + s * (to.slide - from.slide), from.time + u * to.time});
            }
            from = samples.back();
        }
    }
    KDL::Frame target(const Candidate &c, const Sample &s, double retreat = 0) {
        auto f = origin * KDL::Frame(KDL::Rotation::RotZ(s.angle), KDL::Vector(0, 0, s.slide)) * c.handle *
                 tool.Inverse();
        f.p = f.p - f.M * KDL::Vector(0, 0, retreat);
        return f;
    }
    void set(int side, const Q &q) {
        for (int j = 0; j < 7; j++)
            d->qpos[m->jnt_qposadr[joint[side][j]]] = q(j);
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
    bool collision(const Sample &s, int side = -1) {
        d->qpos[m->jnt_qposadr[wheel[0]]] = s.angle;
        d->qpos[m->jnt_qposadr[wheel[1]]] = s.slide;
        mj_forward(m.get(), d.get());
        for (int k = 0; k < d->ncon; k++) {
            auto &c = d->contact[k];
            if (c.dist >= -.001)
                continue;
            int a = owner(c.geom1), b = owner(c.geom2);
            if (side >= 0 && ((a != side && b != side) || a == 1 - side || b == 1 - side))
                continue;
            failure = std::string(mj_id2name(m.get(), mjOBJ_GEOM, c.geom1)) + " / " +
                      mj_id2name(m.get(), mjOBJ_GEOM, c.geom2);
            return true;
        }
        return false;
    }
    bool trace(int side, Candidate &c) {
        if (std::abs(c.offset) + grasp["tool"]["length"].as<double>() / 2 > grip_half_length[side]) {
            failure = "cylinder extends past the grasp segment";
            return false;
        }
        Q q = seeds[side], out(7);
        if (ik[side]->CartToJnt(q, target(c, samples[0]), out) < 0) {
            failure = "initial IK";
            return false;
        }
        q = out;
        Q fixed = q;
        fixed(1) = 90 * rad;
        if (home_ik[side]->CartToJnt(fixed, target(c, samples[0], approach_distance + .03), c.home) < 0) {
            failure = "home IK";
            return false;
        }
        c.home(1) = 90 * rad;
        if (ik[side]->CartToJnt(c.home, target(c, samples[0], approach_distance), c.approach) < 0) {
            failure = "approach IK";
            return false;
        }
        // Validate the same joint pre-approach and Cartesian final approach used by Aviator.
        for (int k = 0; k <= 30; k++) {
            Q p(7);
            p.data = c.home.data + (double(k) / 30) * (c.approach.data - c.home.data);
            set(side, p);
            if (collision(samples[0], side))
                return false;
            c.approach_path.push_back(p);
        }
        q = c.approach;
        for (int k = 1; k <= 30; k++) {
            if (ik[side]->CartToJnt(q, target(c, samples[0], approach_distance * (1 - double(k) / 30)), out) <
                0) {
                failure = "final approach IK";
                return false;
            }
            if ((out.data - q.data).cwiseAbs().maxCoeff() > .07) {
                failure = "approach discontinuity";
                return false;
            }
            q = out;
            set(side, q);
            if (collision(samples[0], side))
                return false;
            c.approach_path.push_back(q);
        }
        c.path.push_back(q);
        for (size_t k = 1; k < samples.size(); k++) {
            if (ik[side]->CartToJnt(q, target(c, samples[k]), out) < 0) {
                failure = "path IK";
                return false;
            }
            double speed =
                (out.data - q.data).cwiseAbs().maxCoeff() / (samples[k].time - samples[k - 1].time);
            if (speed > .7) {
                failure = "path speed/discontinuity";
                return false;
            }
            Q mid(7);
            mid.data = .5 * (q.data + out.data);
            set(side, mid);
            Sample midpoint{.5 * (samples[k].angle + samples[k - 1].angle),
                            .5 * (samples[k].slide + samples[k - 1].slide), 0};
            if (collision(midpoint, side))
                return false;
            q = out;
            set(side, q);
            if (collision(samples[k], side))
                return false;
            c.path.push_back(q);
            c.score = std::max(c.score, speed);
        }
        return true;
    }
    void search() {
        for (int s = 0; s < 2; s++) {
            int tried = 0;
            for (double offset : {0., -.02, .02})
                for (int spin = 0; spin < 360; spin += 15) {
                    Candidate c;
                    c.offset = offset;
                    c.spin = spin;
                    c.handle = {aligned[s] * KDL::Rotation::RotZ(spin * rad), centers[s] + offset * axes[s]};
                    tried++;
                    if (trace(s, c)) {
                        candidates[s].push_back(c);
                        std::cout << "PASS " << s << " offset=" << offset << " spin=" << spin
                                  << " speed=" << c.score << std::endl;
                    } else if (spin == 0 || spin == 15 || spin == 345)
                        std::cout << "REJECT " << s << " offset=" << offset << " spin=" << spin << " "
                                  << failure << std::endl;
                }
            std::sort(candidates[s].begin(), candidates[s].end(),
                      [](auto &a, auto &b) { return a.score < b.score; });
            std::cout << "Candidates side=" << s << " accepted=" << candidates[s].size() << "/" << tried
                      << std::endl;
        }
    }
    bool pair(const Candidate &a, const Candidate &b) {
        for (size_t k = 0; k < a.approach_path.size(); k++) {
            set(0, a.approach_path[k]);
            set(1, b.approach_path[k]);
            if (collision(samples[0]))
                return false;
            if (k) {
                Q l(7), r(7);
                l.data = .5 * (a.approach_path[k].data + a.approach_path[k - 1].data);
                r.data = .5 * (b.approach_path[k].data + b.approach_path[k - 1].data);
                set(0, l);
                set(1, r);
                if (collision(samples[0]))
                    return false;
            }
        }
        for (size_t k = 0; k < samples.size(); k++) {
            set(0, a.path[k]);
            set(1, b.path[k]);
            if (collision(samples[k]))
                return false;
            if (k) {
                Q l(7), r(7);
                l.data = .5 * (a.path[k].data + a.path[k - 1].data);
                r.data = .5 * (b.path[k].data + b.path[k - 1].data);
                set(0, l);
                set(1, r);
                if (collision({.5 * (samples[k].angle + samples[k - 1].angle),
                               .5 * (samples[k].slide + samples[k - 1].slide), 0}))
                    return false;
            }
        }
        return true;
    }
    void save(const fs::path &dir, const Candidate &a, const Candidate &b) {
        fs::create_directories(dir);
        std::ofstream out(dir / "selection.json");
        out.precision(15);
        out << "{\n";
        for (int s = 0; s < 2; s++) {
            const auto &c = s ? b : a;
            double x, y, z, w;
            c.handle.M.GetQuaternion(x, y, z, w);
            out << '"' << (s ? "right" : "left") << "\": {\"position\": [" << c.handle.p.x() << ','
                << c.handle.p.y() << ',' << c.handle.p.z() << "], \"quaternion\": [" << w << ',' << x << ','
                << y << ',' << z << "], \"offset\": " << c.offset << ", \"spin_deg\": " << c.spin
                << ", \"home_deg\": [";
            for (int j = 0; j < 7; j++)
                out << (j ? "," : "") << c.home(j) / rad;
            out << "], \"approach_seed_deg\": [";
            for (int j = 0; j < 7; j++)
                out << (j ? "," : "") << c.approach(j) / rad;
            out << "]},\n";
        }
        out << "\"validation\": {\"sample_period_s\": 0.2, \"midpoint_checks\": true, \"max_joint_speed\": "
            << std::max(a.score, b.score) << ", \"wheel_angle_rad\": 0.872, \"translation_m\": -0.16}}\n";
        std::ofstream csv(dir / "trajectory.csv");
        csv << "time,angle,slide";
        for (int j = 0; j < 14; j++)
            csv << ",q" << j;
        csv << '\n';
        for (size_t k = 0; k < samples.size(); k++) {
            auto &s = samples[k];
            csv << s.time << ',' << s.angle << ',' << s.slide;
            for (int j = 0; j < 7; j++)
                csv << ',' << a.path[k](j);
            for (int j = 0; j < 7; j++)
                csv << ',' << b.path[k](j);
            csv << '\n';
        }
    }
};
int main(int argc, char **argv) {
    try {
        if (argc != 3) {
            std::cerr << "Usage: aviator_grasp_search CONFIG OUTPUT_DIRECTORY\n";
            return 2;
        }
        Search search(fs::absolute(argv[1]));
        search.search();
        for (auto &a : search.candidates[0])
            for (auto &b : search.candidates[1])
                if (search.pair(a, b)) {
                    search.save(argv[2], a, b);
                    std::cout << "SEARCH PASS: " << argv[2] << std::endl;
                    return 0;
                }
        std::cerr << "No validated pair found; last rejection: " << search.failure << std::endl;
        return 1;
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}
