#include <algorithm>
#include <array>
#include <aviator/Aviator.hpp>
#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <limits>
#include <rocos_mujoco/shared_memory_config.hpp>
#include <thread>

using namespace std::chrono_literals;
namespace ipc = rocos_mujoco::aviator;
void require(bool ok, const char *message) {
    if (!ok)
        throw std::runtime_error(message);
}
template <class Operation> void rejected(Operation operation) {
    bool failed = false;
    try {
        operation();
    } catch (const std::runtime_error &error) {
        failed = true;
        std::cout << "Expected rejection: " << error.what() << std::endl;
    }
    require(failed, "Expected command rejection");
}
int main(int argc, char **argv) {
    if (argc != 3)
        return 2;
    const int bus = std::stoi(argv[2]);
    try {
        aviator::Aviator robot(argv[1], bus);
        robot.Init();
        auto *hw = rocos::SharedMemoryConfig::getInstance(bus);
        constexpr double degrees = 180.0 / 3.14159265358979323846;
        std::array<double, 2> minimum{95, 95}, maximum{85, 85};
        auto check_elbows = [&](bool check_commands) {
            for (int side = 0; side < 2; ++side) {
                const int axis = 7 * side + 1;
                const double actual =
                    hw->getSlaveInputVarValueByName<int32_t>(axis, "Position actual value") / 156455.678 *
                    degrees;
                minimum[side] = std::min(minimum[side], actual);
                maximum[side] = std::max(maximum[side], actual);
                require(actual >= 85 && actual <= 95, "Measured joint 2 left [85, 95] deg");
                if (check_commands) {
                    const double target = hw->getSlaveOutputVarValueByName<int32_t>(axis, "Target Position") /
                                          156455.678 * degrees;
                    require(target >= 85 && target <= 95, "Commanded joint 2 left [85, 95] deg");
                } else {
                    // Disabled-drive PD holding has gravity deflection. The
                    // simulator model test checks the exact +90 deg keyframe.
                    require(std::abs(actual - 90) < 1, "Simulator did not start near joint 2 +90 deg");
                }
            }
        };
        auto monitored = [&](auto operation) {
            auto motion = std::async(std::launch::async, operation);
            try {
                do {
                    check_elbows(true);
                } while (motion.wait_for(1ms) != std::future_status::ready);
                motion.get();
                check_elbows(true);
            } catch (...) {
                robot.Stop();
                throw;
            }
        };
        check_elbows(false);
        rejected([&] { robot.MoveWheel(.1, -.02); });
        robot.Enable();
        rejected([&] { robot.LockHandles(); });
        // Also exercise simulator-side gating, independently of the controller.
        ipc::Channel channel(bus);
        uint64_t request;
        {
            ipc::Channel::Guard guard(channel);
            request = ++channel.data().request;
            channel.data().command = ipc::Command::Lock;
        }
        for (int i = 0; i < 100 && channel.read().ack != request; ++i)
            std::this_thread::sleep_for(10ms);
        auto f = channel.read();
        require(f.ack == request && f.result == ipc::Result::NotAligned && f.locked == 0,
                "Simulator must refuse distant locking");
        auto cancelled = std::async(std::launch::async, [&] {
            try {
                robot.ApproachHandles();
                return false;
            } catch (const std::runtime_error &) {
                return true;
            }
        });
        for (int i = 0; i < 100 && robot.GetState() != "approaching"; ++i)
            std::this_thread::sleep_for(10ms);
        std::this_thread::sleep_for(500ms);
        robot.Stop();
        require(cancelled.wait_for(2s) == std::future_status::ready && cancelled.get(),
                "Stop failed to interrupt approach");
        require(robot.GetStatus().locked == 0, "Cancelled approach unexpectedly locked");
        monitored([&] { robot.ApproachHandles(); });
        robot.LockHandles();
        require(robot.GetStatus().locked == 3, "Both handles must be locked");
        auto targets = [&] {
            std::array<int32_t, 14> result{};
            for (int axis = 0; axis < 14; ++axis)
                result[axis] = hw->getSlaveOutputVarValueByName<int32_t>(axis, "Target Position");
            return result;
        };
        const auto before = targets();
        rejected([&] { robot.MoveWheel(.9, -.02); });
        rejected([&] { robot.MoveWheel(0, .01); });
        rejected([&] { robot.MoveWheel(0, -.17); });
        rejected([&] { robot.MoveWheel(std::numeric_limits<double>::quiet_NaN(), -.02); });
        rejected([&] { robot.MoveWheel(0, -.02, 0); });
        require(targets() == before, "Invalid commands changed arm targets");
        for (const auto target :
             {std::array<double, 3>{.87266, 0, 8}, {-.87266, 0, 16}, {0, 0, 8}, {0, -.16, 8}, {0, 0, 8}}) {
            monitored([&] { robot.MoveWheel(target[0], target[1], target[2]); });
            f = robot.GetStatus();
            std::cout << "Verified wheel: target=" << target[0] << ',' << target[1] << " actual=" << f.angle
                      << ',' << f.displacement << std::endl;
            require(f.locked == 3 && !f.fault, "Lost grasp while moving");
            require(std::abs(f.angle - target[0]) < .005 && std::abs(f.displacement - target[1]) < .001,
                    "Wheel target tracking error");
            for (int side = 0; side < 2; ++side)
                require(f.position_error[side] < .002 && f.rotation_error[side] < .01, "Grasp drift");
        }
        // A full turn requested too quickly must be rejected before execution.
        const auto constrained_before = targets();
        monitored([&] { rejected([&] { robot.MoveWheel(.87266, 0, .5); }); });
        require(targets() == constrained_before, "Rejected constrained path changed arm targets");
        require(robot.GetStatus().locked == 3 && !robot.GetStatus().fault,
                "Rejected constrained path disturbed the grasp");
        robot.UnlockHandles();
        require(robot.GetStatus().locked == 0, "Unlock failed");
        robot.Disable();
        for (int side = 0; side < 2; ++side)
            std::cout << "Verified joint 2 " << (side == 0 ? "left" : "right")
                      << " range (deg): " << minimum[side] << " .. " << maximum[side] << '\n';
        std::cout << "INTEGRATION PASS\n";
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
