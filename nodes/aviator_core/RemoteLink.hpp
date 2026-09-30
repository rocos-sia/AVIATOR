#pragma once
#include "motion.hpp"
#include "HandLink.hpp"
#include <condition_variable>
#include <thread>
#include <deque>
namespace aviator {
// Owns non-RT bus IO. Planning consumes feedback snapshots; it never accesses an SDK.
class RemoteLink final : public DataLink {
  public:
    explicit RemoteLink(const MotionConfig &, bool authorize = true);
    ~RemoteLink() override;
    void heartbeat();
    void report(const std::string &state, const std::string &source = "NONE");
    void reportSystem(const Json&); // Immutable owner-thread FSM export, separate from executor phase.
    DeviceState snapshot(bool& fresh) const;
    void allowMotion(bool); // Revokes queued ordinary output on protective cancellation.
    const std::string& backend() const { return backend_; }
    double getJointPosition(Side, int) const override;
    double getJointVelocity(Side, int) const override;
    std::array<double, 14> jointTargets() const override;
    double jointVelLimit(Side, int) const override;
    bool isEnabled(Side) const override;
    void enable(Side) override;
    void disable(Side) override;
    GraspState graspState() const override;
    uint64_t sendGraspCommand(GraspCommand) override;
    void setJointPositions(const Joints &) override;
    void runTrajectory(const std::vector<JointFrame> &, const std::atomic<bool> &) override;
    void stopTrajectory() override;
    void beginStream(const std::vector<JointFrame>&) override;
    void appendStream(const std::vector<JointFrame>&) override;
    size_t streamAhead() const override;
    void finishStream() override;
    void waitTick() override;
    double time() const override {
        return double(monotonic_us()) / 1e6;
    }
    std::string diagnostics() const override;

  private:
    friend struct RemoteLinkTestAccess; // Fault injection: block arm IO without blocking hand IO.
    Json operation(const std::string &);
    void io();
    void handTarget(bool close);
    MotionConfig config_;
    HandLink hand_;
    zmq::context_t context_{1};
    std::string session_, server_, epoch_, backend_;
    std::atomic<bool> quit_{false};
    std::atomic<uint64_t> heartbeat_{0};
    mutable std::mutex mutex_;
    std::mutex service_mutex_;
    std::condition_variable changed_;
    DeviceState state_;
    Json arm_body_ = Json::object(), hand_body_ = Json::object();
    Json system_body_;
    bool motion_allowed_ = true;
    Joints speed_{};
    std::shared_ptr<const std::vector<JointFrame>> trajectory_;
    std::deque<JointFrame> stream_;
    uint64_t stream_first_ = 0;
    bool streaming_ = false, stream_finished_ = false;
    uint64_t trajectory_id_ = 0, start_ = 0, received_ = 0, sample_ = 0, ack_ = 0;
    bool enabled_ = false, publishing_ = false, feedback_valid_ = false;
    std::string error_, phase_ = "INIT", source_ = "NONE";
    std::thread thread_;
};
} // namespace aviator
