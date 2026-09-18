#include "aviator/backend.hpp"

#include <rocos_app/ethercat/hardware.h>
#include <rocos_app/robot.h>
#include <rocos_mujoco/aviator_protocol.hpp>
#include <rocos_mujoco/shared_memory_config.hpp>

#include <atomic>
#include <chrono>
#include <semaphore.h>
#include <signal.h>
#include <stdexcept>
#include <thread>
#include <time.h>
#include <unistd.h>

namespace aviator {
namespace ipc = rocos_mujoco::aviator;

namespace {
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
ipc::Command map(GraspCommand command) {
    switch (command) {
    case GraspCommand::Lock: return ipc::Command::Lock;
    case GraspCommand::Unlock: return ipc::Command::Unlock;
    case GraspCommand::ResetFault: return ipc::Command::ResetFault;
    }
    fail("Unknown grasp command");
}
GraspResult map(ipc::Result result) {
    switch (result) {
    case ipc::Result::Ok: return GraspResult::Ok;
    case ipc::Result::NotAligned: return GraspResult::NotAligned;
    case ipc::Result::NotEnabled: return GraspResult::NotEnabled;
    case ipc::Result::Fault: return GraspResult::Fault;
    }
    fail("Unknown grasp result");
}
} // namespace

// MuJoCo 后端:持有 shm 通道、rocos 驱动(臂)与周期信号量,把原始符号限制在本文件内。
class MuJoCoDataLink final : public DataLink {
  public:
    MuJoCoDataLink(const std::string &urdf, int bus) : bus_(bus) {
        channel_ = std::make_unique<ipc::Channel>(bus);
        {
            ipc::Channel::Guard guard(*channel_);
            auto &shared = channel_->data();
            require(shared.controller_pid == 0 || (kill(shared.controller_pid, 0) != 0 && errno == ESRCH),
                    "Another AVIATOR controller owns this simulator");
            shared.controller_pid = getpid();
            shared.controller_heartbeat = monotonic();
            claimed_ = true;
        }
        auto *bus_config = rocos::SharedMemoryConfig::getInstance(bus);
        require(bus_config->getSlaveNum() == 14, "Expected 14 drive slaves");
        require(bus_config->getDt() == 1000, "Controller expects a 1000 us simulation cycle");
        hardware_ = std::make_unique<rocos::Hardware>(urdf, bus);
        for (int side = 0; side < 2; ++side) {
            const std::string tip =
                std::string("AR5-5_07") + (side == 0 ? "L" : "R") + "-W4C4A2_flan_link";
            arms_[side] = std::make_unique<rocos::Robot>(hardware_.get(), urdf, "aircraft", tip, true);
            require(arms_[side]->getJointNum() == 7 && arms_[side]->GetRobotState() != "ERROR_STATE",
                    "Robot initialization failed");
        }
        tick_ = sem_open(("/sync" + std::to_string(bus) + "_8").c_str(), 0);
        require(tick_ != SEM_FAILED, "Cannot open simulation cycle semaphore");
        running_ = true;
        heartbeat_ = std::thread([this] {
            while (running_) {
                try {
                    ipc::Channel::Guard guard(*channel_);
                    channel_->data().controller_heartbeat = monotonic();
                } catch (...) {
                    cancel_ = true;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        });
    }

    ~MuJoCoDataLink() override {
        running_ = false;
        if (heartbeat_.joinable())
            heartbeat_.join();
        if (channel_ && claimed_) {
            try {
                ipc::Channel::Guard guard(*channel_);
                channel_->data().controller_pid = 0;
            } catch (...) {
            }
        }
        if (tick_ != SEM_FAILED)
            sem_close(tick_);
    }

    double getJointPosition(Side side, int axis) const override {
        return arms_[static_cast<int>(side)]->getJointPosition(axis);
    }
    void setJointPositions(const std::array<double, 14> &q) override {
        ipc::Channel::Guard guard(*channel_);
        for (int i = 0; i < 14; ++i)
            arms_[i / 7]->setJointPosition(i % 7, q[i]);
    }
    double jointVelLimit(Side side, int axis) const override {
        return arms_[static_cast<int>(side)]->getJntVelLimit(axis);
    }
    bool isEnabled(Side side) const override { return arms_[static_cast<int>(side)]->IsEnabled(); }
    void enable(Side side) override {
        require(arms_[static_cast<int>(side)]->SetEnabled() == 0, "Failed to enable arm");
    }
    void disable(Side side) override {
        require(arms_[static_cast<int>(side)]->SetDisabled() == 0, "Failed to disable arm");
    }
    GraspState graspState() const override {
        const auto f = channel_->read();
        GraspState state;
        state.heartbeat = f.heartbeat;
        state.locked = f.locked;
        state.ready = f.ready;
        state.fault = f.fault;
        state.position_error[0] = f.position_error[0];
        state.position_error[1] = f.position_error[1];
        state.rotation_error[0] = f.rotation_error[0];
        state.rotation_error[1] = f.rotation_error[1];
        state.angle = f.angle;
        state.displacement = f.displacement;
        state.ack = f.ack;
        state.result = map(f.result);
        return state;
    }
    uint64_t sendGraspCommand(GraspCommand command) override {
        ipc::Channel::Guard guard(*channel_);
        auto &shared = channel_->data();
        shared.command = map(command);
        return ++shared.request;
    }
    void waitTick() override {
        timespec deadline{};
        clock_gettime(CLOCK_REALTIME, &deadline);
        deadline.tv_nsec += 100000000;
        if (deadline.tv_nsec >= 1000000000) {
            ++deadline.tv_sec;
            deadline.tv_nsec -= 1000000000;
        }
        while (sem_timedwait(tick_, &deadline) != 0) {
            if (errno == EINTR)
                continue;
            throw std::runtime_error("Simulator cycle timeout");
        }
    }

  private:
    int bus_;
    bool claimed_ = false;
    std::atomic<bool> running_{false}, cancel_{false};
    std::unique_ptr<ipc::Channel> channel_;
    std::unique_ptr<rocos::Hardware> hardware_;
    std::unique_ptr<rocos::Robot> arms_[2];
    sem_t *tick_ = SEM_FAILED;
    std::thread heartbeat_;
};

std::unique_ptr<DataLink> makeMuJoCoDataLink(const std::string &urdf_path, int bus) {
    return std::make_unique<MuJoCoDataLink>(urdf_path, bus);
}

} // namespace aviator
