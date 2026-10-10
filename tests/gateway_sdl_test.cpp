#include "sdl_input.hpp"
#include <iostream>
#include <map>
#include <stdexcept>

namespace {
std::map<SDL_JoystickID, const char*> device_names;
}
extern "C" const char* __real_SDL_JoystickNameForIndex(int index);
extern "C" const char* __wrap_SDL_JoystickNameForIndex(int index) {
    const auto it = device_names.find(SDL_JoystickGetDeviceInstanceID(index));
    return it == device_names.end() ? __real_SDL_JoystickNameForIndex(index) : it->second;
}
namespace {
void check(bool value, const char* reason) { if (!value) throw std::runtime_error(reason); }
int attachFalseJoystick(const char* name) {
    const int index = SDL_JoystickAttachVirtual(SDL_JOYSTICK_TYPE_UNKNOWN, 2, 11, 0);
    check(index >= 0, "attach keyboard/mouse with joystick axes");
    device_names[SDL_JoystickGetDeviceInstanceID(index)] = name;
    auto* device = SDL_JoystickOpen(index);
    check(device != nullptr, "open false joystick");
    check(SDL_JoystickSetVirtualAxis(device, 0, -32768) == 0 &&
          SDL_JoystickSetVirtualAxis(device, 1, -32768) == 0 &&
          SDL_JoystickSetVirtualButton(device, 0, 1) == 0, "set false joystick inputs");
    SDL_JoystickUpdate();
    SDL_JoystickClose(device);
    return index;
}
void detachVirtualJoysticks() {
    for (int i = SDL_NumJoysticks() - 1; i >= 0; --i)
        if (SDL_JoystickIsVirtual(i))
            check(SDL_JoystickDetachVirtual(i) == 0, "detach test device");
}
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
    event.key.keysym.mod = SDL_GetModState();
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
        auto now = aviator::monotonic_us();
        // An explicitly selected keyboard must not inject its full-scale HID axes.
        for (const char* name : {"CHERRY MX 3.0S Dongle Keyboard", "USB mOuSe", "无线键盘", "无线鼠标"}) {
            check(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK) == 0, "initialize virtual input");
            config.device = std::to_string(attachFalseJoystick(name));
            flight_gateway::SdlInput selected(config, [&] { return now; });
            flight_gateway::JoystickSample value{{0, -32768, 32767, 0}, {1, -32768, 32767, 0}};
            std::vector<unsigned> hits;
            focus(true);
            check(selected.poll(value, hits) && value.valid && value.roll_value == 0 &&
                  value.pitch_value == 0 && hits.empty(), "explicit keyboard/mouse index is rejected");
            detachVirtualJoysticks();
            device_names.clear();
        }
        config.device = "auto";
        check(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK) == 0, "initialize virtual input");
        attachFalseJoystick("CHERRY MX 3.0S Dongle kEyBoArD");
        flight_gateway::SdlInput input(config, [&] { return now; });
        flight_gateway::JoystickSample sample{{0, -32768, 32767, 0}, {1, -32768, 32767, 0}};
        std::vector<unsigned> pressed;
        auto poll = [&] { check(input.poll(sample, pressed), "unexpected quit"); };
        focus(true); poll();
        check(sample.fresh(now, 100000) && sample.roll_value == 0 && sample.pitch_value == 0 &&
              pressed.empty(), "false joystick is skipped at startup");
        attachFalseJoystick("USB Mouse"); poll();
        check(sample.roll_value == 0 && sample.pitch_value == 0 && pressed.empty(),
              "false joystick is skipped on hotplug");
        key(SDL_SCANCODE_RIGHT, true); poll(); now += 20000; poll();
        check(sample.roll_value > 0, "SDL keyboard ramps");
        key(SDL_SCANCODE_RIGHT, false); poll();
        const auto held = sample.roll_value;
        now += 20000; poll();
        check(sample.roll_value == held, "release retains keyboard position");
        key(SDL_SCANCODE_1, true); poll();
        check(pressed == std::vector<unsigned>{0}, "SDL keyboard button mapping");
        key(SDL_SCANCODE_1, true, true); poll(); check(pressed.empty(), "SDL repeat suppressed");
        key(SDL_SCANCODE_1, false); poll();
        key(SDL_SCANCODE_RIGHT, true); poll(); now += 20000; poll();
        focus(false); poll();
        const auto unfocused = sample.roll_value;
        check(sample.valid && unfocused > held, "focus loss holds a valid target");
        key(SDL_SCANCODE_1, true); poll(); check(pressed.empty(), "background keyboard disabled");
        now += 299999999; poll();
        check(sample.valid && sample.roll_value == unfocused, "5 minute grace holds without accumulating");
        focus(false); poll(); // Repeated notifications must not renew the grace period.
        now += 1; poll();
        check(!sample.valid && !sample.device_connected, "5 minute boundary invalidates keyboard input");
        focus(true); poll();
        check(sample.valid && sample.roll_value == unfocused, "focus regain retains target, not held keys");
        now += 20000; poll(); check(sample.roll_value == unfocused, "no automatic accumulation after focus regain");
        // A new focus loss gets a full grace period.
        focus(false); poll(); now += 20000; poll(); check(sample.valid, "focus timer resets on genuine regain");
        focus(true); poll();
        key(SDL_SCANCODE_0, true); poll();
        check(pressed.empty() && sample.roll_value == 0 && sample.pitch_value == 0, "zero is local recenter, not button 10");
        key(SDL_SCANCODE_0, false); poll();

        const std::array<SDL_Scancode, 10> keypad{
            SDL_SCANCODE_KP_1, SDL_SCANCODE_KP_2, SDL_SCANCODE_KP_3, SDL_SCANCODE_KP_4,
            SDL_SCANCODE_KP_5, SDL_SCANCODE_KP_6, SDL_SCANCODE_KP_7, SDL_SCANCODE_KP_8,
            SDL_SCANCODE_KP_9, SDL_SCANCODE_KP_MINUS};
        for (const auto mod : {KMOD_NONE, KMOD_NUM}) {
            SDL_SetModState(mod);
            for (unsigned i = 0; i < keypad.size(); ++i) {
                key(keypad[i], true); poll();
                check(pressed == std::vector<unsigned>{i == 9 ? 10u : i}, "keypad maps with either Num Lock state");
                key(keypad[i], true, true); poll(); check(pressed.empty(), "keypad repeat suppressed");
                key(keypad[i], false); poll(); check(pressed.empty(), "keypad release sends no request");
            }
            now += 20000; poll();
            check(sample.roll_value == 0 && sample.pitch_value == 0, "keypad navigation legends do not move axes");
            key(SDL_SCANCODE_RIGHT, true); poll(); now += 20000; poll();
            check(sample.roll_value > 0, "prepare nonzero keypad reset target");
            key(SDL_SCANCODE_KP_0, true); poll();
            check(sample.roll_value == 0 && sample.pitch_value == 0 && pressed.empty(), "keypad zero recenters without service request");
            now += 20000; poll(); check(sample.roll_value == 0, "keypad zero stops held arrows");
            key(SDL_SCANCODE_KP_0, false); key(SDL_SCANCODE_RIGHT, false); poll();
        }
        // Releasing one physical key must not clear another physical key's held state.
        key(SDL_SCANCODE_1, true); poll(); check(pressed == std::vector<unsigned>{0}, "main digit press");
        key(SDL_SCANCODE_KP_1, true); poll(); check(pressed == std::vector<unsigned>{0}, "independent keypad press");
        key(SDL_SCANCODE_KP_1, false); poll();
        key(SDL_SCANCODE_1, true); poll(); check(pressed.empty(), "keypad release cannot retrigger held main digit");
        key(SDL_SCANCODE_1, false); poll();
        focus(false); poll(); key(SDL_SCANCODE_KP_2, true); poll();
        check(pressed.empty(), "background keypad input ignored");
        focus(true); poll(); key(SDL_SCANCODE_KP_2, false); poll();

        const int device = SDL_JoystickAttachVirtual(SDL_JOYSTICK_TYPE_FLIGHT_STICK, 2, 11, 0);
        check(device >= 0, "attach virtual joystick");
        SDL_Joystick* joystick = SDL_JoystickOpen(device);
        check(joystick != nullptr, "open virtual joystick");
        poll();
        check(SDL_JoystickSetVirtualAxis(joystick, 0, 32767) == 0 &&
              SDL_JoystickSetVirtualAxis(joystick, 1, -32768) == 0, "set virtual axes");
        poll(); check(sample.roll_value == 1 && sample.pitch_value == -1, "joystick movement takes over axes");
        key(SDL_SCANCODE_LEFT, true); poll(); now += 20000; poll();
        check(sample.roll_value > 0 && sample.roll_value < 1 && sample.pitch_value == -1,
              "keyboard accumulates from current joystick target on its axis");
        const auto keyboard_stamp = sample.sample_us;
        key(SDL_SCANCODE_LEFT, false); poll();
        const auto keyboard_target = sample.roll_value;
        now += 20000; poll();
        check(sample.roll_value == keyboard_target && sample.sample_us >= keyboard_stamp,
              "release holds target even with a stationary joystick");
        key(SDL_SCANCODE_LEFT, true); key(SDL_SCANCODE_RIGHT, true); poll();
        now += 20000; poll(); check(sample.roll_value == keyboard_target, "opposing keys hold target");
        focus(false); poll();
        now += 300000000; poll();
        check(sample.valid && sample.roll_value == 1, "joystick remains available after keyboard focus timeout");
        SDL_JoystickSetVirtualButton(joystick, 0, 1); poll();
        check(pressed == std::vector<unsigned>{0}, "SDL joystick button mapping");
        poll(); check(pressed.empty(), "held joystick button does not repeat");
        SDL_JoystickClose(joystick);
        check(SDL_JoystickDetachVirtual(device) == 0, "detach virtual joystick"); poll();
        check(!sample.valid && !sample.device_connected, "disconnect after focus timeout invalidates input");
        focus(true); poll(); check(sample.valid, "keyboard remains available after joystick removal");
        check(SDL_JoystickAttachVirtual(SDL_JOYSTICK_TYPE_UNKNOWN, 2, 11, 0) >= 0, "replace with generic joystick");
        poll(); focus(false); poll(); check(sample.valid, "replacement joystick selected without config change");
        SDL_Event quit{}; quit.type = SDL_QUIT; SDL_PushEvent(&quit);
        check(!input.poll(sample, pressed), "window quit stops input");
        detachVirtualJoysticks();
        std::cout << "SDL gateway tests passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n'; return 1;
    }
}
