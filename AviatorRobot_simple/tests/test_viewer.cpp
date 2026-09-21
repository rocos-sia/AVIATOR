#include "../src/Viewer.hpp"
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

void check(bool ok, const char *message) { if (!ok) throw std::runtime_error(message); }
int main() {
    try {
        char error[1024]{};
        std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
            mj_loadXML(AVIATOR_MODEL_DIR "/aviator.xml", nullptr, error, sizeof(error)), mj_deleteModel);
        if (!model) throw std::runtime_error(error);
        std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
        mj_resetDataKeyframe(model.get(), data.get(), mj_name2id(model.get(), mjOBJ_KEY, "aviator_home"));
        mj_forward(model.get(), data.get());
        std::mutex physics;
        aviator::Viewer viewer(model.get());
        check(viewer.draw(data.get(), physics), "Viewer closed unexpectedly");
        auto *window = viewer.window();
        // Exercise the actual callbacks registered on the real GLFW window.
        auto button = glfwSetMouseButtonCallback(window, nullptr);
        auto move = glfwSetCursorPosCallback(window, nullptr);
        auto scroll = glfwSetScrollCallback(window, nullptr);
        auto key = glfwSetKeyCallback(window, nullptr);
        auto focus = glfwSetWindowFocusCallback(window, nullptr);
        check(button && move && scroll && key && focus, "Missing input callback");
        glfwSetMouseButtonCallback(window, button); glfwSetCursorPosCallback(window, move);
        glfwSetScrollCallback(window, scroll); glfwSetKeyCallback(window, key);
        glfwSetWindowFocusCallback(window, focus);
        const auto initial = viewer.camera();
        auto drag = [&](int which) {
            button(window, which, GLFW_PRESS, 0);
            double x, y; glfwGetCursorPos(window, &x, &y);
            move(window, x + 80, y + 60);
            button(window, which, GLFW_RELEASE, 0);
        };
        drag(GLFW_MOUSE_BUTTON_LEFT);
        check(viewer.camera().azimuth != initial.azimuth || viewer.camera().elevation != initial.elevation,
              "Left drag did not rotate camera");
        std::cout << "Left drag rotates camera\n";
        drag(GLFW_MOUSE_BUTTON_RIGHT);
        double delta = 0;
        for (int i = 0; i < 3; ++i) delta += std::abs(viewer.camera().lookat[i] - initial.lookat[i]);
        check(delta > 0, "Right drag did not pan camera");
        std::cout << "Right drag pans camera\n";
        const auto distance = viewer.camera().distance;
        scroll(window, 0, 1);
        std::cout << "wheel distance: " << distance << " -> " << viewer.camera().distance << std::endl;
        check(viewer.camera().distance != distance, "Wheel did not zoom");
        const auto near = viewer.camera().distance;
        drag(GLFW_MOUSE_BUTTON_MIDDLE);
        check(viewer.camera().distance != near, "Middle drag did not zoom");
        std::cout << "Wheel and middle drag zoom camera\n";
        button(window, GLFW_MOUSE_BUTTON_LEFT, GLFW_PRESS, 0);
        focus(window, GLFW_FALSE);
        const auto unfocused = viewer.camera();
        move(window, 20, 20);
        check(viewer.camera().azimuth == unfocused.azimuth && viewer.camera().elevation == unfocused.elevation,
              "Drag remained active after losing focus");
        key(window, GLFW_KEY_R, 0, GLFW_PRESS, 0);
        check(viewer.camera().azimuth == initial.azimuth && viewer.camera().elevation == initial.elevation &&
              viewer.camera().distance == initial.distance, "R did not reset view");
        for (int i = 0; i < 3; ++i)
            check(viewer.camera().lookat[i] == initial.lookat[i], "R did not restore lookat");
        check(data->time == 0, "Camera operation modified physics time");
        std::cout << "R resets camera; physics state unchanged\n";
        check(viewer.draw(data.get(), physics), "Rendering after camera movement failed");
        key(window, GLFW_KEY_ESCAPE, 0, GLFW_PRESS, 0);
        check(!viewer.draw(data.get(), physics), "Escape did not close viewer");
        std::cout << "Escape closes viewer\n";
    } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
