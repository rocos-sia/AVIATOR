#include "sdl_input.hpp"
#include <iostream>
#include <stdexcept>

namespace {
void check(bool value, const char* reason) { if (!value) throw std::runtime_error(reason); }
void focus(bool gained) {
    SDL_Event event{};
    event.type = SDL_WINDOWEVENT;
    event.window.event = gained ? SDL_WINDOWEVENT_FOCUS_GAINED : SDL_WINDOWEVENT_FOCUS_LOST;
    check(SDL_PushEvent(&event) == 1, "push focus");
}
void key(SDL_Scancode code, bool down, bool repeat = false) {
    SDL_Event event{};
    event.type = down ? SDL_KEYDOWN : SDL_KEYUP;
    event.key.keysym.scancode = code;
    event.key.repeat = repeat;
    check(SDL_PushEvent(&event) == 1, "push key");
}
}
int main() {
    try {
        flight_gateway::Config config{};
        config.source = "rs422";
        bool rejected = false;
        try { flight_gateway::SdlInput input(config); }
        catch (const std::runtime_error&) { rejected = true; }
        check(rejected && SDL_WasInit(0) == 0, "RS422 must not initialize SDL");
        config.source = "joystick"; config.device = "auto";
        config.roll_axis = 0; config.pitch_axis = 1;
        config.keyboard.roll_speed = 10;
        flight_gateway::SdlInput input(config);
        flight_gateway::JoystickSample sample{{0, -32768, 32767, 0}, {1, -32768, 32767, 0}};
        std::vector<unsigned> pressed;
        auto poll = [&] { check(input.poll(sample, pressed), "unexpected quit"); };
        focus(true); poll();
        check(sample.fresh(aviator::monotonic_us(), 100000) && sample.roll_value == 0,
              "keyboard-only startup");
        key(SDL_SCANCODE_RIGHT, true); poll(); SDL_Delay(20); poll();
        check(sample.roll_value > 0, "SDL keyboard ramps");
        key(SDL_SCANCODE_RIGHT, false); poll();
        check(sample.roll_value == 0, "keyboard release centers without joystick");
        key(SDL_SCANCODE_1, true); poll();
        check(pressed == std::vector<unsigned>{0}, "SDL keyboard button mapping");
        key(SDL_SCANCODE_1, true, true); poll();
        check(pressed.empty(), "SDL repeat suppressed");
        key(SDL_SCANCODE_1, false); poll();
        key(SDL_SCANCODE_RIGHT, true); poll(); SDL_Delay(20); poll();
        focus(false); poll();
        check(!sample.valid && sample.roll_value == 0, "focus loss clears held keys and invalidates keyboard-only input");
        key(SDL_SCANCODE_1, true); poll();
        check(pressed.empty(), "background keyboard disabled");
        focus(true); poll();
        check(sample.valid && sample.roll_value == 0, "focus regain does not restore held direction");

        const int device = SDL_JoystickAttachVirtual(SDL_JOYSTICK_TYPE_FLIGHT_STICK, 2, 11, 0);
        check(device >= 0, "attach virtual joystick");
        SDL_Joystick* joystick = SDL_JoystickOpen(device);
        check(joystick != nullptr, "open virtual joystick");
        poll();
        check(SDL_JoystickSetVirtualAxis(joystick, 0, 32767) == 0 &&
              SDL_JoystickSetVirtualAxis(joystick, 1, -32768) == 0, "set virtual axes");
        poll();
        check(sample.roll_value == 1 && sample.pitch_value == -1, "SDL joystick normalization");
        key(SDL_SCANCODE_LEFT, true); poll(); SDL_Delay(20); poll();
        check(sample.roll_value < 0 && sample.pitch_value == -1, "keyboard overrides only its active axis");
        const auto keyboard_stamp = sample.sample_us;
        key(SDL_SCANCODE_LEFT, false); poll();
        check(sample.roll_value == 1 && sample.sample_us >= keyboard_stamp,
              "release restores joystick without regressing protocol timestamp");
        key(SDL_SCANCODE_LEFT, true); key(SDL_SCANCODE_RIGHT, true); poll();
        check(sample.roll_value == 0, "opposing keys override joystick with zero");
        focus(false); poll();
        check(sample.valid && sample.roll_value == 1, "joystick remains active in background");
        SDL_JoystickSetVirtualButton(joystick, 0, 1); poll();
        check(pressed == std::vector<unsigned>{0}, "SDL joystick button mapping");
        poll(); check(pressed.empty(), "held joystick button does not repeat");
        SDL_JoystickClose(joystick);
        check(SDL_JoystickDetachVirtual(device) == 0, "detach virtual joystick");
        poll();
        check(!sample.valid && !sample.device_connected, "disconnect with unfocused keyboard invalidates input");
        focus(true); poll();
        check(sample.valid && sample.roll_value == 0 && sample.pitch_value == 0,
              "keyboard remains available after joystick removal");
        check(SDL_JoystickAttachVirtual(SDL_JOYSTICK_TYPE_FLIGHT_STICK, 2, 11, 0) >= 0,
              "replace joystick");
        poll(); focus(false); poll();
        check(sample.valid, "replacement joystick selected without config change");
        SDL_Event quit{}; quit.type = SDL_QUIT;
        SDL_PushEvent(&quit);
        check(!input.poll(sample, pressed), "window quit stops input");
        std::cout << "SDL gateway tests passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n'; return 1;
    }
}
