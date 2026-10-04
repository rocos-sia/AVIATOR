#include "camera_window.hpp"
#include <algorithm>
#include <stdexcept>
namespace simulation {
CameraWindow::CameraWindow(int width, int height, GLFWwindow* viewer)
    : width_(width), height_(height) {
    glfwWindowHint(GLFW_FLOATING, GLFW_TRUE);
    window_ = glfwCreateWindow(640, std::max(1, 640 * height / width), "D436 RGB - Simulation",
                               nullptr, nullptr);
    glfwWindowHint(GLFW_FLOATING, GLFW_FALSE);
    if (!window_)
        throw std::runtime_error("Cannot create camera preview window");
    int x, y;
    glfwGetWindowPos(viewer, &x, &y);
    glfwSetWindowPos(window_, x + 40, y + 60);
    glfwSetKeyCallback(window_, [](GLFWwindow* w, int key, int, int action, int) {
        if (action == GLFW_PRESS && (key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q))
            glfwSetWindowShouldClose(w, GLFW_TRUE);
    });
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(0);
    glGenTextures(1, &texture_);
    glBindTexture(GL_TEXTURE_2D, texture_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width_, height_, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    glfwMakeContextCurrent(nullptr);
}
CameraWindow::~CameraWindow() {
    glfwMakeContextCurrent(window_);
    glDeleteTextures(1, &texture_);
    glfwMakeContextCurrent(nullptr);
    glfwDestroyWindow(window_);
}
void CameraWindow::update(const std::vector<unsigned char>& rgb) {
    if (glfwWindowShouldClose(window_))
        return;
    glfwMakeContextCurrent(window_);
    glBindTexture(GL_TEXTURE_2D, texture_);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width_, height_, GL_RGB, GL_UNSIGNED_BYTE, rgb.data());
    have_frame_ = true;
    glfwMakeContextCurrent(nullptr);
}
void CameraWindow::draw() {
    if (glfwWindowShouldClose(window_)) {
        glfwHideWindow(window_);
        return;
    }
    int w, h;
    glfwGetFramebufferSize(window_, &w, &h);
    if (w <= 0 || h <= 0)
        return;
    glfwMakeContextCurrent(window_);
    glViewport(0, 0, w, h);
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    if (have_frame_) {
        const double scale = std::min(double(w) / width_, double(h) / height_);
        int vw = int(width_ * scale), vh = int(height_ * scale);
        glViewport((w - vw) / 2, (h - vh) / 2, vw, vh);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, texture_);
        glColor3f(1, 1, 1);
        // Shared RGB buffers have top-left origin.
        glBegin(GL_QUADS);
        glTexCoord2f(0, 0);
        glVertex2f(-1, 1);
        glTexCoord2f(1, 0);
        glVertex2f(1, 1);
        glTexCoord2f(1, 1);
        glVertex2f(1, -1);
        glTexCoord2f(0, 1);
        glVertex2f(-1, -1);
        glEnd();
        glDisable(GL_TEXTURE_2D);
    }
    glfwSwapBuffers(window_);
    glfwMakeContextCurrent(nullptr);
}
} // namespace simulation
