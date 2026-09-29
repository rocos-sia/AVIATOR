#pragma once
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <mutex>

namespace aviator {
// Window, callbacks and camera are all owned by the main (rendering) thread.
class Viewer {
    mjModel *model_;
    GLFWwindow *window_ = nullptr;
    mjvCamera camera_{};
    mjvOption option_{};
    mjvScene scene_{};
    mjrContext context_{};
    bool left_ = false, middle_ = false, right_ = false;
    double last_x_ = 0, last_y_ = 0;

    static Viewer &self(GLFWwindow *window);
    static void mouseButton(GLFWwindow *, int button, int action, int mods);
    static void mouseMove(GLFWwindow *, double x, double y);
    static void scroll(GLFWwindow *, double x, double y);
    static void key(GLFWwindow *, int key, int scancode, int action, int mods);
    static void focus(GLFWwindow *, int focused);
public:
    explicit Viewer(mjModel *model);
    ~Viewer();
    Viewer(const Viewer &) = delete;
    Viewer &operator=(const Viewer &) = delete;
    void resetCamera();
    bool draw(mjData *data, std::mutex &mutex);
    const mjvCamera &camera() const { return camera_; }
    GLFWwindow *window() const { return window_; }
};
}
