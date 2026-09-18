#include "aviator/backend.hpp"

#include <rokae/robot.h>
#include <urdf_parser/urdf_parser.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <system_error>
#include <time.h>
#include <vector>

namespace aviator {

namespace {
constexpr std::chrono::microseconds kTick{1000};

double monotonic() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}
[[noreturn]] void fail(const std::string &message) { throw std::runtime_error(message); }
void require(bool ok, const std::string &message) {
    if (!ok)
        fail(message);
}
} // namespace

// Rokae 真机后端:持有两个 7 轴 xMateErProRobot 实例(左/右臂),臂 IO 走 SDK 的周期调度
// setControlLoop(RT 线程按 1ms 节拍回调下发 JointPosition),关节速度限位从 URDF 读入。
// 轮盘/夹爪为外部硬件、不在 Rokae SDK 内,相关接口留 TODO 桩。
// 本文件只在 CMake 检测到 xCoreSDK 头文件与预编译库时才编译(定义 AVIATOR_HAVE_ROKAE)。
class RokaeDataLink final : public DataLink {
  public:
    RokaeDataLink(const std::string &urdf, const std::string &left_ip,
                  const std::string &right_ip, const std::string &local_ip) {
        load_velocity_limits(urdf);
        const char *ip[2] = {left_ip.c_str(), right_ip.c_str()};
        for (int side = 0; side < 2; ++side) {
            arms_[side] = std::make_unique<rokae::xMateErProRobot>();
            try {
                arms_[side]->connectToRobot(ip[side], local_ip);
            } catch (const std::exception &e) {
                throw std::runtime_error(std::string("Rokae connectToRobot(") + ip[side] +
                                         ") failed: " + e.what());
            }
            std::error_code ec;
            arms_[side]->setOperateMode(rokae::OperateMode::automatic, ec);
            check(ec, "Rokae setOperateMode(automatic) failed");
            arms_[side]->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
            check(ec, "Rokae setMotionControlMode(RtCommand) failed");
            rt_[side] = arms_[side]->getRtMotionController().lock();
            require(rt_[side] != nullptr, "Rokae getRtMotionController() returned null");
            // 上电前读一次关节角,供 Init/Enable 阶段的 measured() 使用。
            joint_pos_[side] = arms_[side]->jointPos(ec);
            check(ec, "Rokae jointPos() initial read failed");
        }
    }

    ~RokaeDataLink() override {
        for (int side = 1; side >= 0; --side) {
            if (!arms_[side])
                continue;
            // 回调捕获了 this,必须在其存活期内先停周期调度,再停运动、下电、断开。
            try {
                if (servoing_[side])
                    rt_[side]->stopLoop();
            } catch (...) {
            }
            try {
                if (servoing_[side])
                    rt_[side]->stopMove();
            } catch (...) {
            }
            std::error_code ec;
            try {
                if (powered_[side])
                    arms_[side]->setPowerState(false, ec);
            } catch (...) {
            }
            try {
                arms_[side]->disconnectFromRobot(ec);
            } catch (...) {
            }
        }
    }

    double getJointPosition(Side side, int axis) const override {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return joint_pos_[static_cast<int>(side)][axis];
    }
    void setJointPositions(const std::array<double, 14> &q) override {
        // 目标位形只在此处更新,由 setControlLoop 的回调(SDK RT 线程)按 1ms 节拍读取下发。
        std::lock_guard<std::mutex> lock(state_mutex_);
        target_ = q;
    }
    double jointVelLimit(Side side, int axis) const override {
        return vel_limit_[static_cast<int>(side)][axis];
    }
    bool isEnabled(Side side) const override { return powered_[static_cast<int>(side)]; }

    void enable(Side side) override {
        const int i = static_cast<int>(side);
        if (powered_[i])
            return;
        try {
            std::error_code ec;
            arms_[i]->setPowerState(true, ec);
            check(ec, "Rokae setPowerState(true) failed");
            arms_[i]->startReceiveRobotState(std::chrono::milliseconds(1),
                                             {rokae::RtSupportedFields::jointPos_m});
            // 用当前实测位形初始化目标,保证 enable 后立即"保持当前位置",而非跳向零位。
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                for (int j = 0; j < 7; ++j)
                    target_[7 * i + j] = joint_pos_[i][j];
            }
            // 命令下发交给 SDK 的周期调度(RT 线程按 1ms 节拍调用回调),替代普通线程忙等
            // sendCommand 的节拍抖动(发送间隔过短/过长会触发伺服报错/通信丢包)。
            // useStateDataInLoop=true 时 SDK 会在回调前更新实时状态,回调内可直接 getStateData。
            rt_[i]->setControlLoop(
                std::function<rokae::JointPosition()>([this, i]() {
                    std::array<double, 7> q{};
                    if (arms_[i]->getStateData(rokae::RtSupportedFields::jointPos_m, q) == 0) {
                        std::lock_guard<std::mutex> lock(state_mutex_);
                        joint_pos_[i] = q;
                    }
                    std::array<double, 7> cmd{};
                    {
                        std::lock_guard<std::mutex> lock(state_mutex_);
                        std::copy(target_.begin() + 7 * i, target_.begin() + 7 * i + 7, cmd.begin());
                    }
                    return rokae::JointPosition(std::vector<double>(cmd.begin(), cmd.end()));
                }),
                0, true);
            rt_[i]->startMove(rokae::RtControllerMode::jointPosition);
            rt_[i]->startLoop(false); // 非阻塞:由 SDK 内部 RT 线程驱动回调,算法侧继续走 waitTick 节拍。
        } catch (const std::runtime_error &) {
            throw;
        } catch (const std::exception &e) {
            throw std::runtime_error(std::string("Rokae enable failed: ") + e.what());
        }
        powered_[i] = true;
        servoing_[i] = true;
    }

    void disable(Side side) override {
        const int i = static_cast<int>(side);
        if (!powered_[i])
            return;
        try {
            if (servoing_[i])
                rt_[i]->stopLoop();
        } catch (...) {
        }
        try {
            if (servoing_[i])
                rt_[i]->stopMove();
        } catch (...) {
        }
        servoing_[i] = false;
        std::error_code ec;
        arms_[i]->setPowerState(false, ec);
        check(ec, "Rokae setPowerState(false) failed");
        powered_[i] = false;
    }

    GraspState graspState() const override {
        // TODO(用户): 轮盘/夹爪为外部硬件,不在 Rokae SDK 内。此处仅返回存活心跳,
        // 使 status()/heartbeat 检查不立即失败;locked/ready/fault/误差等字段待接入
        // 真实反馈后填充。
        GraspState state;
        state.heartbeat = monotonic();
        return state;
    }

    uint64_t sendGraspCommand(GraspCommand command) override {
        // TODO(用户): 轮盘锁定/解锁/复位为外部硬件,不在 Rokae SDK 内。
        (void)command;
        throw std::runtime_error("RokaeDataLink: wheel/grasp I/O not implemented");
    }

    void waitTick() override {
        if (next_tick_ == std::chrono::steady_clock::time_point{})
            next_tick_ = std::chrono::steady_clock::now();
        next_tick_ += kTick;
        while (std::chrono::steady_clock::now() < next_tick_) {
        }
        // 命令下发与实测位形读取已移入 setControlLoop 的回调(SDK RT 线程按 1ms 节拍执行),
        // 此处仅为算法侧提供 1ms 节拍,不再直接操作 SDK 状态或下发指令。
    }

  private:
    void check(std::error_code ec, const std::string &what) const {
        if (ec)
            throw std::runtime_error(what + ": " + ec.message());
    }
    void load_velocity_limits(const std::string &urdf) {
        urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(urdf);
        require(model != nullptr, "Cannot parse URDF: " + urdf);
        for (int side = 0; side < 2; ++side)
            for (int axis = 0; axis < 7; ++axis) {
                const std::string name = std::string("AR5-5_07") + (side == 0 ? "L" : "R") +
                                         "-W4C4A2_joint_" + std::to_string(axis + 1);
                const auto joint = model->getJoint(name);
                require(joint && joint->limits, "Missing joint/velocity limit: " + name);
                vel_limit_[side][axis] = joint->limits->velocity;
            }
    }

    std::unique_ptr<rokae::xMateErProRobot> arms_[2];
    std::shared_ptr<rokae::RtMotionControlCobot<7>> rt_[2];
    std::array<double, 7> vel_limit_[2]{};
    std::array<double, 7> joint_pos_[2]{};
    std::array<double, 14> target_{};
    bool powered_[2] = {false, false};
    bool servoing_[2] = {false, false};
    std::chrono::steady_clock::time_point next_tick_{};
    // 保护 joint_pos_ / target_ 跨线程访问:算法线程写入 target_、读取 joint_pos_;
    // SDK RT 线程(回调)读取 target_、写入 joint_pos_。
    mutable std::mutex state_mutex_;
};

std::unique_ptr<DataLink> makeRokaeDataLink(const std::string &urdf_path,
                                            const std::string &left_ip,
                                            const std::string &right_ip,
                                            const std::string &local_ip) {
    return std::make_unique<RokaeDataLink>(urdf_path, left_ip, right_ip, local_ip);
}

} // namespace aviator
