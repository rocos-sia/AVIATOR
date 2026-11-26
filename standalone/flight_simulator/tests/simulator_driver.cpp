// Runs the production simulator on the SDL main thread, with a virtual joystick.
#include "sdl_input.hpp"
#include "simulator.hpp"
#include <chrono>
#include <iostream>
#include <thread>
#include <unistd.h>
int main(int argc, char** argv) try {
    if (argc != 2)
        return 2;
    auto config = flight_simulator::load_config(argv[1]);
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK) < 0)
        throw std::runtime_error(SDL_GetError());
    const int index = SDL_JoystickAttachVirtual(SDL_JOYSTICK_TYPE_UNKNOWN, 2, 11, 0);
    config.device = std::to_string(index);
    auto* joystick = SDL_JoystickOpen(index);
    if (index < 0 || !joystick)
        throw std::runtime_error("virtual joystick unavailable");
    int stop[2];
    if (pipe(stop) < 0)
        return 2;
    std::thread script([&] {
        using namespace std::chrono_literals;
        // Exercise the monitor during live 80 Hz transmission, including the
        // ten-frame operation below. A page switch must not consume controls.
        const auto page = [](SDL_Scancode key) {
            SDL_Event focus{};
            focus.type = SDL_WINDOWEVENT;
            focus.window.event = SDL_WINDOWEVENT_FOCUS_GAINED;
            SDL_PushEvent(&focus);
            SDL_Event event{};
            event.type = SDL_KEYDOWN;
            event.key.keysym.scancode = key;
            SDL_PushEvent(&event);
        };
        std::this_thread::sleep_for(300ms);
        page(SDL_SCANCODE_F2);
        std::this_thread::sleep_for(300ms);
        SDL_JoystickSetVirtualAxis(joystick, 0, 16384);
        SDL_JoystickSetVirtualAxis(joystick, 1, -16384);
        std::this_thread::sleep_for(400ms);
        SDL_JoystickSetVirtualButton(joystick, 0, 1);
        std::this_thread::sleep_for(180ms);
        SDL_JoystickSetVirtualButton(joystick, 0, 0);
        page(SDL_SCANCODE_F4);
        std::this_thread::sleep_for(200ms);
        SDL_JoystickSetVirtualAxis(joystick, 0, -16384);
        SDL_JoystickSetVirtualAxis(joystick, 1, 16384);
        page(SDL_SCANCODE_F3);
        std::this_thread::sleep_for(220ms);
        SDL_JoystickDetachVirtual(index);
        std::this_thread::sleep_for(500ms);
        const char byte = 1;
        if (::write(stop[1], &byte, 1) != 1)
            std::terminate();
    });
    try {
        flight_simulator::run(config, stop[0]);
    } catch (...) {
        script.join();
        close(stop[0]);
        close(stop[1]);
        throw;
    }
    script.join();
    close(stop[0]);
    close(stop[1]);
    // SdlInput owns SDL shutdown, including the virtual device.
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}
