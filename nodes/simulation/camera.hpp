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
    std::vector<unsigned char> rgb_;
};
}
