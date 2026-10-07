#pragma once
#include "runtime.hpp"
#include <mujoco/mujoco.h>
#include <array>
#include <memory>
#include <vector>
#include <deque>

namespace simulation {
using Json = nlohmann::json;
struct Authorization {
    std::string publisher = "aviator_core", session, epoch;
    std::string origin_publisher = "flight_gateway", origin_session;
    std::uint64_t camera_timeout_us = 200000;
};
class Simulation {
public:
    explicit Simulation(const std::string& path, const Authorization& authorization = {}, bool managed = false);
    bool command(const aviator::Message& message, std::uint64_t now, std::string& error);
    void step(std::uint64_t now);
    // Managed mode shares model/data with the arm physics thread. Caller holds its mutex.
    bool handCommand(const aviator::Message&, std::uint64_t now, std::string& error);
    void applyHands(std::uint64_t now);
    aviator::Message handState(std::uint64_t now, const std::string& publisher);
    void setInitialWheel(double angle, double displacement);
    aviator::Message state(bool hand, std::uint64_t now);
    aviator::Message detection(std::uint64_t now, bool captured, bool in_roi,
                               const mjData* snapshot = nullptr);
    void setCameraId(const std::string& id);
    int cameraWidth() const { return camera_resolution_[0]; }
    int cameraHeight() const { return camera_resolution_[1]; }
    const Json& camera_command() const { return camera_command_; }
    mjModel* model() const { return model_.get(); }
    mjData* data() const { return data_.get(); }
    const std::string& session() const { return session_; }
    const std::string& clock() const { return clock_; }
private:
    std::unique_ptr<aviator::InputGuard> hand_guard_;
    Json hand_ack_ = nullptr;
    bool hand_valid_ = false;
    struct ClosingHold {
        int requested = -1, held = -1;
        int residual_direction = 0;
        std::uint64_t last_command = 0;
        std::deque<std::pair<std::uint64_t, int>> samples;
    };
    std::array<std::array<ClosingHold, 6>, 2> closing_holds_{};
    struct Joint { int q, v; double low, high, target; };
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model_{nullptr, mj_deleteModel};
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data_{nullptr, mj_deleteData};
    std::array<std::vector<Joint>, 2> arms_, hands_;
    std::array<int, 2> tcp_{};
    int roll_ = -1, pitch_ = -1;
    std::array<int, 2> camera_resolution_{640, 480};
    std::array<std::unique_ptr<aviator::InputGuard>, 3> guards_;
    std::array<bool, 2> active_{};
    std::array<Json, 3> references_{Json(nullptr), Json(nullptr), Json(nullptr)};
    Json camera_command_ = Json::object();
    std::string session_, clock_;
    std::string camera_id_ = "cockpit";
    std::array<std::uint64_t, 3> sequences_{};
    std::uint64_t frame_ = 0;
    aviator::Message envelope(aviator::Topic topic, int slot, std::uint64_t now, bool valid);
};
}
