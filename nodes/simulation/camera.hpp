#pragma once
#include "simulation.hpp"
#include <EGL/egl.h>
namespace simulation {
// Independent, fixed 640x480 sensor. Viewer interactions never alter this camera.
class Camera {
public:
    explicit Camera(Simulation& simulation);
    ~Camera();
    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;
    bool capture(Simulation& simulation);
private:
    void cleanup();
    EGLDisplay display_ = EGL_NO_DISPLAY;
    EGLContext context_ = EGL_NO_CONTEXT;
    EGLSurface surface_ = EGL_NO_SURFACE;
    mjvScene scene_{};
    mjvCamera camera_{};
    mjvOption option_{};
    mjrContext render_{};
    std::vector<unsigned char> rgb_;
};
}
