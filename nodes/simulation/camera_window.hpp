#pragma once
#include <GLFW/glfw3.h>
#include <vector>
namespace simulation {
// Owned by the main thread and destroyed before the Viewer terminates GLFW.
class CameraWindow {
  public:
    CameraWindow(int width, int height, GLFWwindow* viewer);
    ~CameraWindow();
    CameraWindow(const CameraWindow&) = delete;
    CameraWindow& operator=(const CameraWindow&) = delete;
    void update(const std::vector<unsigned char>& rgb);
    void draw();

  private:
    GLFWwindow* window_ = nullptr;
    unsigned texture_ = 0;
    int width_, height_;
    bool have_frame_ = false;
};
} // namespace simulation
