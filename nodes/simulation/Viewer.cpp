#include "Viewer.hpp"
#include <iostream>
#include <stdexcept>

namespace aviator {
Viewer &Viewer::self(GLFWwindow *window) {
    return *static_cast<Viewer *>(glfwGetWindowUserPointer(window));
}
Viewer::Viewer(mjModel *model) : model_(model) {
    if (!glfwInit()) throw std::runtime_error("GLFW initialization failed; use --headless");
    window_ = glfwCreateWindow(1200, 900, "Aviator", nullptr, nullptr);
    if (!window_) { glfwTerminate(); throw std::runtime_error("Cannot create viewer"); }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);
    mjv_defaultOption(&option_);
    mjv_defaultScene(&scene_); mjr_defaultContext(&context_);
    mjv_makeScene(model_, &scene_, 2000); mjr_makeContext(model_, &context_, mjFONTSCALE_150);
    resetCamera();
    glfwSetWindowUserPointer(window_, this);
    glfwSetMouseButtonCallback(window_, mouseButton);
    glfwSetCursorPosCallback(window_, mouseMove);
    glfwSetScrollCallback(window_, scroll);
    glfwSetKeyCallback(window_, key);
    glfwSetWindowFocusCallback(window_, focus);
    std::cout << "Camera: left drag rotate | right drag pan | Shift changes drag axis | "
                 "wheel/middle drag zoom | R reset view | Esc quit\n";
}
Viewer::~Viewer() {
    mjr_freeContext(&context_); mjv_freeScene(&scene_);
    glfwDestroyWindow(window_); glfwTerminate();
}
void Viewer::resetCamera() {
    mjv_defaultCamera(&camera_);
    camera_.azimuth = 90; camera_.elevation = -20; camera_.distance = 3;
    camera_.lookat[2] = 0.5;
}
void Viewer::mouseButton(GLFWwindow *window, int button, int action, int) {
    auto &v = self(window);
    const bool down = action == GLFW_PRESS;
    if (button == GLFW_MOUSE_BUTTON_LEFT) v.left_ = down;
    if (button == GLFW_MOUSE_BUTTON_MIDDLE) v.middle_ = down;
    if (button == GLFW_MOUSE_BUTTON_RIGHT) v.right_ = down;
    glfwGetCursorPos(window, &v.last_x_, &v.last_y_);
}
void Viewer::mouseMove(GLFWwindow *window, double x, double y) {
    auto &v = self(window);
    const double dx = x - v.last_x_, dy = y - v.last_y_;
    v.last_x_ = x; v.last_y_ = y;
    if (!v.left_ && !v.middle_ && !v.right_) return;
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    if (height <= 0) return;
    const bool shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                       glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
    const mjtMouse action = v.right_ ? (shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V) :
        v.left_ ? (shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V) : mjMOUSE_ZOOM;
    // Same camera mapping as MuJoCo's bundled sample/basic.cc.
    mjv_moveCamera(v.model_, action, dx / height, dy / height, &v.scene_, &v.camera_);
}
void Viewer::scroll(GLFWwindow *window, double, double y) {
    auto &v = self(window);
    mjv_moveCamera(v.model_, mjMOUSE_ZOOM, 0, -0.05 * y, &v.scene_, &v.camera_);
}
void Viewer::key(GLFWwindow *window, int key, int, int action, int) {
    if (action != GLFW_PRESS) return;
    if (key == GLFW_KEY_R) self(window).resetCamera(); // does not reset robot/physics
    if (key == GLFW_KEY_ESCAPE) glfwSetWindowShouldClose(window, GLFW_TRUE);
}
void Viewer::focus(GLFWwindow *window, int focused) {
    if (!focused) {
        auto &v = self(window);
        v.left_ = v.middle_ = v.right_ = false;
    }
}
bool Viewer::draw(mjData *data, std::mutex &mutex) {
    glfwPollEvents();
    if (glfwWindowShouldClose(window_)) return false;
    mjrRect viewport{0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
    if (viewport.width <= 0 || viewport.height <= 0) return true;
    { // Only the scene snapshot holds the physics lock; input/GPU/vsync do not.
        std::lock_guard<std::mutex> guard(mutex);
        mjv_updateScene(model_, data, &option_, nullptr, &camera_, mjCAT_ALL, &scene_);
    }
    mjr_render(viewport, &scene_, &context_);
    glfwSwapBuffers(window_);
    return true;
}
}
