#include "RemoteLink.hpp"
#include <atomic>
#include <iostream>
using namespace aviator;
namespace aviator {
struct RemoteLinkTestAccess {
    static void stallArmIO(RemoteLink& link) {
        std::lock_guard<std::mutex> lock(link.mutex_);
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
    }
};
}
int main(int argc, char** argv) {
    try {
        RemoteLink link(loadMotionConfig(argv[1]));
        std::atomic<bool> done{false};
        std::thread heartbeat([&] { while (!done) { link.heartbeat(); std::this_thread::sleep_for(std::chrono::microseconds(100)); } });
        auto cleanup = [&] { done = true; heartbeat.join(); };
        try {
            const std::string mode = argv[2];
            if (mode == "synchronized") {
                link.enable(Side::Left);
                std::vector<JointFrame> frames(1001);
                for (size_t k = 0; k < frames.size(); ++k) {
                    for (int side = 0; side < 2; ++side) {
                        const size_t start = side ? 100 : 300;
                        const double u = k <= start ? 0 : double(k-start)/(1000-start);
                        frames[k].hand_closure[side] = u*u*u*(10+u*(-15+6*u));
                    }
                }
                std::atomic<bool> cancel{false};
                link.runTrajectory(frames, cancel);
                link.sendGraspCommand(GraspCommand::Lock);
                link.sendGraspCommand(GraspCommand::Unlock);
                link.disable(Side::Left);
                cleanup();
            } else if (mode == "normal" || mode == "isolation") {
                link.enable(Side::Left);
                link.sendGraspCommand(GraspCommand::Lock);
                if (mode == "isolation") RemoteLinkTestAccess::stallArmIO(link);
                std::this_thread::sleep_for(std::chrono::milliseconds(600));
                link.stopTrajectory();
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                link.sendGraspCommand(GraspCommand::Unlock);
                link.disable(Side::Left);
                cleanup();
            } else if (mode == "dropped" || mode == "revoke" || mode == "heartbeat" || mode == "arm_failure") {
                link.enable(Side::Left);
                link.sendGraspCommand(GraspCommand::Lock);
                if (mode == "revoke") link.allowMotion(false);
                if (mode == "heartbeat") cleanup();
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                bool fresh;
                const auto snapshot = link.snapshot(fresh);
                if ((mode == "dropped" || mode == "heartbeat" || mode == "arm_failure") && (!link.graspState().fault || !snapshot.fault))
                    throw std::runtime_error("Lost hand feedback/heartbeat not propagated");
                if (mode == "heartbeat") {
                    const auto error = link.graspState().fault_reason;
                    if (error.find("age_us=") == std::string::npos || error.find("limit_us=100000") == std::string::npos)
                        throw std::runtime_error("Missing actual heartbeat age diagnostic: " + error);
                } else cleanup();
            } else {
                bool rejected = false;
                try { link.enable(Side::Left); } catch (const std::exception& e) {
                    rejected = true; std::cout << "Expected rejection: " << e.what() << '\n';
                }
                cleanup();
                if (!rejected) throw std::runtime_error("Bad hand state allowed arm enable");
            }
        } catch (...) { if (!done.exchange(true)) heartbeat.join(); throw; }
        std::cout << "RemoteLink hand integration passed\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
