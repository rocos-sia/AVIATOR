#include "WheelEstimate.hpp"
#include "aviator/backend.hpp"
#include "JointWheelFeedback.hpp"
#include <filesystem>
#include <iostream>
using namespace wheel_bandwidth;
using namespace aviator;
void check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
int main(int argc, char** argv) {
    try {
        check(argc == 2, "Expected repository path");
        const std::filesystem::path root = argv[1];
        const auto grasp = YAML::LoadFile((root/"config/grasp.json").string());
        const auto posture = YAML::LoadFile((root/"config/posture.json").string());
        const auto lim = posture["joint2_limits_deg"].as<std::vector<double>>();
        auto fk = makePinIkKinematics((root/"models/urdf/aviator.urdf").string(),lim[0]*pi/180,lim[1]*pi/180);
        WheelEstimate estimator(grasp);
        ArmFeedback measured;
        measured.valid = measured.enabled = {true, true};
        measured.sample_time = {1.0, 1.001};
        for (int side = 0; side < 2; ++side) {
            std::array<double, 7> seed{}, q{};
            const auto values = posture[side ? "right_approach_seed_deg" : "left_approach_seed_deg"].as<std::vector<double>>();
            for (int j = 0; j < 7; ++j) seed[j] = values[j] * pi / 180;
            const double angle = side ? .12 : .10, displacement = side ? -.09 : -.08;
            const pinocchio::SE3 wheel(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
                                      Eigen::Vector3d(0, 0, displacement));
            const auto target = frame(grasp["wheel_origin"]) * wheel * frame(grasp[side ? "right" : "left"]) *
                                frame(toolFrameConfig(grasp, side)).inverse();
            check(fk->solveIk(Side(side), seed, target, q), "Feedback fixture IK failed");
            std::copy(q.begin(), q.end(), measured.q.begin() + 7 * side);
        }
        JointWheelFeedback feedback(makePinIkKinematics((root/"models/urdf/aviator.urdf").string(),
                                   lim[0]*pi/180, lim[1]*pi/180), grasp);
        auto result = feedback.measure(measured, true, 1002000);
        check(result["valid"] == true && result["source"] == "joint_fk" && result["sample_mono_us"] == 1000000,
              "Joint FK validity and oldest source timestamp");
        check(std::abs(result["angle"].get<double>() - .11) < 1e-5 &&
              std::abs(result["displacement"].get<double>() + .085) < 1e-5, "Dual-arm FK fusion");
        measured.target.fill(0); // Commands must never enter the measurement path.
        check(feedback.measure(measured, true, 1003000)["angle"] == result["angle"], "Targets cannot alter feedback");
        check(feedback.measure(measured, true, 1100001)["valid"] == false, "Republish does not refresh joint time");
        check(feedback.measure(measured, false, 1002000)["valid"] == false, "Unlocked FK is invalid");
        check(feedback.measure(measured, true, 999000)["valid"] == false, "Future joint sample is invalid");
        auto bad = measured;
        bad.sample_time[1] = 1.010;
        check(feedback.measure(bad, true, 1011000)["valid"] == false, "Unsynchronized arms invalid");
        bad = measured; bad.valid[0] = false;
        check(feedback.measure(bad, true, 1002000)["valid"] == false, "Missing arm invalid");
        bad = measured; bad.q[7] = std::numeric_limits<double>::quiet_NaN();
        check(feedback.measure(bad, true, 1002000)["valid"] == false, "Nonfinite joint invalid");
        for (int side=0; side<2; ++side) {
            const auto seed = posture[side ? "right_approach_seed_deg" : "left_approach_seed_deg"].as<std::vector<double>>();
            std::array<double,7> q{}, solution{};
            for(int j=0;j<7;++j) q[j]=seed[j]*pi/180;
            for (double angle : {-10*pi/180, 0., 10*pi/180}) {
                for (double displacement : {-.12, -.085, -.04}) {
                    const pinocchio::SE3 wheel(Eigen::AngleAxisd(angle,Eigen::Vector3d::UnitZ()).toRotationMatrix(),
                                               Eigen::Vector3d(0,0,displacement));
                    const auto target = frame(grasp["wheel_origin"])*wheel*frame(grasp[side ? "right" : "left"])*
                                        frame(toolFrameConfig(grasp,side)).inverse();
                    auto est = estimator.fromFlange(side,target);
                    check(std::abs(difference(est.angle,angle))<1e-10, "Geometry angle inversion");
                    check(std::abs(est.displacement-displacement)<1e-10, "Geometry displacement inversion");
                    check(est.transverse_error<1e-10 && est.tilt_error<1e-10,"Unexpected geometry residual");
                    check(fk->solveIk(static_cast<Side>(side),q,target,solution),"Test pose IK failed");
                    pinocchio::SE3 actual;
                    check(fk->solveFk(static_cast<Side>(side),solution,actual),"FK failed");
                    est=estimator.fromFlange(side,actual);
                    check(std::abs(difference(est.angle,angle))<1e-5,"Joint feedback angle estimate");
                    check(std::abs(est.displacement-displacement)<1e-5,"Joint feedback displacement estimate");
                    q=solution;
                }
            }
        }
        check(std::abs(difference(meanAngle(179*pi/180,-179*pi/180),pi))<1e-10,"Circular fusion");
        Sweep sweep;
        double boundary=0;
        for (double f:sweep.frequencies) {
            check(std::abs(sweep.at(boundary).offset)<1e-8,"Segment starts at zero");
            boundary+=(2*sweep.ramp+sweep.settle+sweep.measure)/f;
            check(std::abs(sweep.at(boundary-1e-6).offset)<1e-8,"Segment smoothly ends at zero");
        }
        for(double t=0;t<sweep.duration();t+=.001)
            check(std::abs(sweep.at(t).offset)<=10*pi/180+1e-12,"Amplitude exceeds 10 degrees");
        sweep.amplitude = 3*pi/180;
        sweep.validateMotion(-.085);
        auto target = sweep.target(sweep.at(32.5).offset, -.085); // 3.25 cycles at 0.1 Hz
        check(std::abs(target.angle-3*pi/180)<1e-10 && target.displacement == -.085,
              "Rotation must hold displacement and reach 3 degrees");
        sweep.translation = true;
        sweep.amplitude = .020;
        sweep.validateMotion(-.085);
        for (double t : {32.5, 37.5}) {
            target = sweep.target(sweep.at(t).offset, -.085);
            check(target.angle == 0, "Translation must hold angle at zero");
            check(std::abs(target.displacement-(t == 32.5 ? -.065 : -.105))<1e-10,
                  "Translation must reach +/-20 mm about center");
        }
        target = sweep.target(sweep.at(sweep.duration()).offset, -.085);
        check(target.angle == 0 && target.displacement == -.085, "Sweep must return to center");
        for (double center : {-.160, -.010}) {
            bool rejected = false;
            try { sweep.validateMotion(center); } catch (const std::runtime_error&) { rejected = true; }
            check(rejected, "Out-of-range translation must be rejected");
        }
        std::cout<<"Wheel estimator: geometry, real URDF IK/FK, angle fusion and sweep passed\n";
        return 0;
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
