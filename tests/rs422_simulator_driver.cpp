// Runs the production simulator on the SDL main thread, with a virtual joystick.
#include "rs422_runtime.hpp"
#include "sdl_input.hpp"
#include <chrono>
#include <iostream>
#include <thread>
#include <unistd.h>
int main(int argc, char** argv) try {
    if (argc != 2)
        return 2;
    auto config = flight_gateway::load_config(argv[1]);
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
        std::this_thread::sleep_for(600ms);
        SDL_JoystickSetVirtualAxis(joystick, 0, 16384);
        SDL_JoystickSetVirtualAxis(joystick, 1, -16384);
        std::this_thread::sleep_for(400ms);
        SDL_JoystickSetVirtualButton(joystick, 0, 1);
        std::this_thread::sleep_for(180ms);
        SDL_JoystickSetVirtualButton(joystick, 0, 0);
        std::this_thread::sleep_for(420ms);
        SDL_JoystickDetachVirtual(index);
        std::this_thread::sleep_for(500ms);
        const char byte = 1;
        if (::write(stop[1], &byte, 1) != 1)
            std::terminate();
    });
    try {
        flight_gateway::runRs422(config, stop[0]);
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
