#pragma once
#include "simulation.hpp"
#include <EGL/egl.h>
#include <mutex>
namespace simulation {
// Independent fixed sensor; resolution comes from MJCF. Viewer interactions never alter it.
class Camera {
public:
    explicit Camera(Simulation& simulation);
    ~Camera();
    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;
    bool capture(Simulation& simulation, std::mutex* physics_mutex = nullptr);
    const std::vector<unsigned char>& rgb() const { return rgb_; }
    aviator::Message detection(bool in_roi) const;
    const Json& calibration() const { return calibration_; }
private:
    void cleanup();
    std::unique_ptr<mjData, decltype(&mj_deleteData)> snapshot_{nullptr, mj_deleteData};
    EGLDisplay display_ = EGL_NO_DISPLAY;
    EGLContext context_ = EGL_NO_CONTEXT;
    EGLSurface surface_ = EGL_NO_SURFACE;
    mjvScene scene_{};
    mjvCamera camera_{};
    mjvOption option_{};
    mjrContext render_{};
    int width_, height_;
    std::uint64_t sample_time_ = 0;
    Json calibration_;
    aviator::Message detection_;
    std::vector<unsigned char> rgb_;
};
}
