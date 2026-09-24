// 实际双臂：MuJoCo 独立核对 FK、IK 残差、限位、连续性和耗时。
#include "aviator/backend.hpp"
#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>

void check(bool ok, const char *message) { if (!ok) throw std::runtime_error(message); }

pinocchio::SE3 bodyPose(const mjData *data, int body) {
    Eigen::Matrix3d rotation;
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 3; ++col) rotation(row,col) = data->xmat[9*body+3*row+col];
    return {rotation, Eigen::Vector3d(data->xpos[3*body], data->xpos[3*body+1], data->xpos[3*body+2])};
}

int main() {
    try {
        char error[1024]{};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(AVIATOR_MODEL_DIR "/aviator.xml", nullptr, error, sizeof(error)), mj_deleteModel);
        check(bool(model), error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
        check(bool(data), "Cannot allocate simulation data");
        const int root = mj_name2id(model.get(), mjOBJ_BODY, "aircraft");
        check(root >= 0, "Missing aircraft body");
        const auto posture = YAML::LoadFile(AVIATOR_CONFIG_DIR "/posture.json");
        const auto limits = posture["joint2_limits_deg"].as<std::vector<double>>();
        const double margin = posture["joint2_planning_margin_deg"].as<double>();
        const double lo = (limits[0] + margin) * M_PI / 180;
        const double hi = (limits[1] - margin) * M_PI / 180;
        auto pin = aviator::makePinIkKinematics(AVIATOR_CONFIG_DIR "/aviator_control.urdf", lo, hi);
        std::mt19937 random(20260922);
        std::uniform_real_distribution<double> unit(0., 1.);
        double max_fk_position = 0, max_fk_rotation = 0;
        double max_ik_position = 0, max_ik_rotation = 0, max_seed_step = 0;
        std::vector<double> timings;
        int solved = 0;
        for (int arm = 0; arm < 2; ++arm) {
            const auto side = static_cast<aviator::Side>(arm);
            const auto home = posture[arm == 0 ? "left_home_deg" : "right_home_deg"].as<std::vector<double>>();
            const std::string prefix = std::string("AR5-5_07") + (arm == 0 ? "L" : "R") + "-W4C4A2";
            const int tip = mj_name2id(model.get(), mjOBJ_BODY, (prefix + "_flan_link").c_str());
            check(tip >= 0, "Missing flange body");
            int address[7]{};
            for (int j = 0; j < 7; ++j) {
                const int id = mj_name2id(model.get(), mjOBJ_JOINT, (prefix + "_joint_" + std::to_string(j+1)).c_str());
                check(id >= 0, "Missing joint");
                address[j] = model->jnt_qposadr[id];
                check(std::abs(pin->jointLower(side,j)-model->jnt_range[2*id]) < 1e-9, "Lower physical limit mismatch");
                check(std::abs(pin->jointUpper(side,j)-model->jnt_range[2*id+1]) < 1e-9, "Upper physical limit mismatch");
            }
            const auto referenceFk = [&](const std::array<double,7> &q) {
                for (int j = 0; j < 7; ++j) data->qpos[address[j]] = q[j];
                mj_forward(model.get(), data.get());
                return bodyPose(data.get(), root).inverse() * bodyPose(data.get(), tip);
            };
            for (int sample = 0; sample < 100; ++sample) {
                std::array<double,7> q{};
                for (int j = 0; j < 7; ++j)
                    q[j] = pin->jointLower(side,j) + unit(random) * (pin->jointUpper(side,j)-pin->jointLower(side,j));
                pinocchio::SE3 actual;
                check(pin->solveFk(side,q,actual), "FK failed");
                const auto expected = referenceFk(q);
                max_fk_position = std::max(max_fk_position,(actual.translation()-expected.translation()).norm());
                max_fk_rotation = std::max(max_fk_rotation,aviator::rotationError(actual,expected));
                check(max_fk_position < 1e-8 && max_fk_rotation < 1e-8, "FK mapping/transform mismatch");
            }
            for (int sample = 0; sample < 40; ++sample) {
                std::array<double,7> q{}, seed{}, result{};
                for (int j = 0; j < 7; ++j) {
                    q[j] = home[j]*M_PI/180 + .08*std::sin(.1*sample+j);
                    seed[j] = q[j] + .003*std::cos(.3*sample+j);
                }
                q[1] = sample == 0 ? lo : sample == 1 ? hi : lo+(hi-lo)*unit(random);
                seed[1] = std::clamp(q[1] + .002,lo,hi);
                const auto goal = referenceFk(q);
                const auto begin = std::chrono::steady_clock::now();
                const bool ok = pin->solveIk(side,seed,goal,result);
                timings.push_back(std::chrono::duration<double,std::milli>(
                    std::chrono::steady_clock::now()-begin).count());
                if (!ok) continue;
                ++solved;
                check(result[1] >= lo && result[1] <= hi, "J2 planning limit violation");
                const auto actual = referenceFk(result);
                max_ik_position = std::max(max_ik_position,(actual.translation()-goal.translation()).norm());
                max_ik_rotation = std::max(max_ik_rotation,aviator::rotationError(actual,goal));
                check(max_ik_position < 2e-6 && max_ik_rotation < 2e-6, "IK residual too large");
                for (int j = 0; j < 7; ++j) {
                    check(result[j]>=pin->jointLower(side,j) && result[j]<=pin->jointUpper(side,j), "Physical limit violation");
                    max_seed_step = std::max(max_seed_step,std::abs(result[j]-seed[j]));
                }
            }
            std::array<double,7> seed{}, untouched{};
            for (int j = 0; j < 7; ++j) seed[j] = home[j]*M_PI/180;
            untouched.fill(123.);
            const pinocchio::SE3 unreachable(Eigen::Matrix3d::Identity(), Eigen::Vector3d(10,10,10));
            check(!pin->solveIk(side,seed,unreachable,untouched), "Unreachable target accepted");
            check(std::all_of(untouched.begin(),untouched.end(),[](double v) { return v == 123.; }), "Failed IK overwrote output");
            seed[0] = std::numeric_limits<double>::quiet_NaN();
            check(!pin->solveIk(side,seed,pinocchio::SE3::Identity(),untouched), "Nonfinite seed accepted");
        }
        std::sort(timings.begin(),timings.end());
        std::cout << "PIN-IK solved=" << solved << "/80 p50_ms=" << timings[timings.size()/2]
                  << " p95_ms=" << timings[timings.size()*95/100] << " max_ms=" << timings.back() << '\n'
                  << "MuJoCo FK difference: " << max_fk_position << " m, " << max_fk_rotation << " rad\n"
                  << "MuJoCo IK residual: " << max_ik_position << " m, " << max_ik_rotation << " rad\n"
                  << "Largest seed step: " << max_seed_step << " rad\n";
        check(solved >= 78, "PIN-IK success rate regressed");
        check(max_seed_step < .15, "Nearby targets caused a joint branch jump");
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
