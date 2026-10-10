#include "Logger.hpp"
#include "sdl_input.hpp"

#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace flight_gateway {
namespace {
constexpr std::array<SDL_Scancode, 4> arrows{
    SDL_SCANCODE_LEFT, SDL_SCANCODE_RIGHT, SDL_SCANCODE_UP, SDL_SCANCODE_DOWN};
constexpr std::array<SDL_Scancode, 11> buttons{
    SDL_SCANCODE_1, SDL_SCANCODE_2, SDL_SCANCODE_3, SDL_SCANCODE_4,
    SDL_SCANCODE_5, SDL_SCANCODE_6, SDL_SCANCODE_7, SDL_SCANCODE_8,
    SDL_SCANCODE_9, SDL_SCANCODE_0, SDL_SCANCODE_MINUS};
// Physical scancodes keep keypad digits usable with Num Lock on or off.
constexpr std::array<SDL_Scancode, 11> keypad_buttons{
    SDL_SCANCODE_KP_1, SDL_SCANCODE_KP_2, SDL_SCANCODE_KP_3, SDL_SCANCODE_KP_4,
    SDL_SCANCODE_KP_5, SDL_SCANCODE_KP_6, SDL_SCANCODE_KP_7, SDL_SCANCODE_KP_8,
    SDL_SCANCODE_KP_9, SDL_SCANCODE_KP_0, SDL_SCANCODE_KP_MINUS};
// Internal compatibility reports only; no evdev device is opened or queried.
input_event report(unsigned type, unsigned code, int value, std::uint64_t now) {
    input_event event{};
    event.type = type; event.code = code; event.value = value;
    event.input_event_sec = now / 1000000;
    event.input_event_usec = now % 1000000;
    return event;
}
JoystickSample sample(const Config& c) {
    return {{c.roll_axis, -32768, 32767, 0, c.invert_roll},
            {c.pitch_axis, -32768, 32767, 0, c.invert_pitch}};
}
bool isKeyboardOrMouse(const char* device_name) {
    std::string name = device_name ? device_name : "";
    std::transform(name.begin(), name.end(), name.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    // Some HID keyboards expose axes and are enumerated as UNKNOWN joysticks.
    // Keep unknown genuine sticks eligible; SDL's joystick type alone is insufficient.
    return name.find("keyboard") != std::string::npos || name.find("mouse") != std::string::npos ||
           name.find("键盘") != std::string::npos || name.find("鼠标") != std::string::npos;
}
}
SdlInput::SdlInput(const Config& config, std::function<std::uint64_t()> clock)
    : config_(config), clock_(std::move(clock)), stick_(sample(config)), keys_(sample(config)),
      keyboard_(keys_, config.keyboard) {
    if (config.source != "joystick")
        throw std::runtime_error("RS422 mode not implemented; joystick and keyboard disabled");
    help_ = std::make_unique<InputHelp>(config);
    SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK) != 0) {
        const std::string error = SDL_GetError();
        SDL_Quit();
        throw std::runtime_error("SDL initialization failed: " + error);
    }
    window_ = SDL_CreateWindow("flight_gateway | Input guide",
                              SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                              InputHelp::width, InputHelp::height, SDL_WINDOW_SHOWN);
    if (!window_) {
        const std::string error = SDL_GetError();
        SDL_Quit();
        throw std::runtime_error("SDL input window failed: " + error);
    }
    key_buttons_.codes = keypad_buttons_.codes = keyboard_buttons;
    for (unsigned i = 0; i < stick_buttons_.codes.size(); ++i)
        stick_buttons_.codes[i] = i + 1;
    SDL_PumpEvents();
    focused_ = (SDL_GetWindowFlags(window_) & SDL_WINDOW_INPUT_FOCUS) != 0;
    unfocused_since_ = clock_();
    resetKeyboard(unfocused_since_);
    openJoystick();
    try { help_->show(window_, focused_, joystick_ && SDL_JoystickGetAttached(joystick_)); }
    catch (...) {
        if (joystick_) SDL_JoystickClose(joystick_);
        SDL_DestroyWindow(window_);
        SDL_Quit();
        throw;
    }
}
SdlInput::~SdlInput() {
    if (joystick_) SDL_JoystickClose(joystick_);
    SDL_DestroyWindow(window_);
    SDL_Quit();
}
void SdlInput::openJoystick() {
    if (joystick_) return;
    const bool automatic = config_.device == "auto";
    const int selected = automatic ? 0 : std::stoi(config_.device);
    for (int i = selected; i < SDL_NumJoysticks(); ++i) {
        const auto* name = SDL_JoystickNameForIndex(i);
        if (isKeyboardOrMouse(name)) {
            aviator::Logger::warn("Ignoring SDL joystick index={} name={}: keyboard/mouse device", i, name);
            if (!automatic) break;
            continue;
        }
        SDL_Joystick* candidate = SDL_JoystickOpen(i);
        if (candidate) {
            const auto axes = SDL_JoystickNumAxes(candidate);
            if (axes > static_cast<int>(std::max(config_.roll_axis, config_.pitch_axis))) {
                joystick_ = candidate;
                stick_ = sample(config_);
                stick_buttons_.pending.fill(0);
                for (unsigned b = 0; b < stick_buttons_.held.size(); ++b)
                    stick_buttons_.held[b] = SDL_JoystickGetButton(joystick_, b) != 0;
                aviator::Logger::info("SDL joystick index={} name={} instance={}",
                    i, SDL_JoystickName(joystick_), SDL_JoystickInstanceID(joystick_));
                return;
            }
            SDL_JoystickClose(candidate);
        }
        if (!automatic) break;
    }
    aviator::Logger::warn("SDL joystick unavailable (device={}); keyboard remains available in the input window",
        config_.device);
}
void SdlInput::resetKeyboard(std::uint64_t now) {
    // Release every direction, including suppressed keys, before accepting new input.
    for (unsigned i = 0; i < arrows.size(); ++i)
        keyboard_.update(report(EV_KEY, keyboard_arrows[i], 0, now), now);
    keyboard_.update(report(EV_SYN, SYN_REPORT, 0, now), now);
    arrows_.fill(false);
    key_buttons_.pending.fill(0);
    keypad_buttons_.pending.fill(0);
    const auto* state = SDL_GetKeyboardState(nullptr);
    for (unsigned i = 0; i < arrows.size(); ++i)
        if (state[arrows[i]]) keyboard_.suppressHeld(keyboard_arrows[i]);
    for (unsigned i = 0; i < buttons.size(); ++i) {
        key_buttons_.held[i] = state[buttons[i]] != 0;
        keypad_buttons_.held[i] = state[keypad_buttons[i]] != 0;
    }
}
bool SdlInput::poll(JoystickSample& output, std::vector<unsigned>& pressed) {
    pressed.clear();
    SDL_Event event{};
    bool drained = false, repaint = false;
    for (unsigned n = 0; n < 128; ++n) {
        if (!SDL_PollEvent(&event)) { drained = true; break; }
        const auto now = clock_();
        if (event.type == SDL_QUIT || (event.type == SDL_WINDOWEVENT &&
            event.window.event == SDL_WINDOWEVENT_CLOSE)) return false;
        if (event.type == SDL_WINDOWEVENT) repaint = true;
        if (event.type == SDL_WINDOWEVENT &&
            (event.window.event == SDL_WINDOWEVENT_FOCUS_LOST ||
             event.window.event == SDL_WINDOWEVENT_FOCUS_GAINED)) {
            const bool focused = event.window.event == SDL_WINDOWEVENT_FOCUS_GAINED;
            if (focused != focused_) {
                focused_ = focused;
                if (!focused_) {
                    unfocused_since_ = now;
                    aviator::Logger::warn("Keyboard focus lost: holding target; keyboard input expires after 5 minutes");
                } else {
                    aviator::Logger::info("Keyboard focus restored; if Core entered SAFE, use the displayed recovery steps");
                }
                resetKeyboard(now);
            }
        }
        if (event.type == SDL_JOYDEVICEREMOVED && joystick_ &&
            event.jdevice.which == SDL_JoystickInstanceID(joystick_)) {
            SDL_JoystickClose(joystick_); joystick_ = nullptr;
            stick_ = sample(config_);
            openJoystick();
        } else if (event.type == SDL_JOYDEVICEADDED) {
            openJoystick();
        }
        // SDL timestamps are wrapping millisecond ticks, not CLOCK_MONOTONIC microseconds.
        const auto age_ms = static_cast<Uint32>(SDL_GetTicks() - event.common.timestamp);
        const bool recent = age_ms < 100 && now > age_ms * 1000ULL;
        const auto stamp = recent ? now - age_ms * 1000ULL : now;
        if ((event.type == SDL_KEYDOWN || event.type == SDL_KEYUP) && focused_) {
            const bool down = event.type == SDL_KEYDOWN;
            if (event.key.repeat || (down && !recent)) continue;
            for (unsigned i = 0; i < arrows.size(); ++i) {
                if (event.key.keysym.scancode != arrows[i]) continue;
                if (down && !keyboard_active_[i / 2])
                    keyboard_.setAxis(i / 2, i < 2 ? output.roll_value : output.pitch_value);
                if (down) keyboard_active_[i / 2] = true;
                // Use receipt time for integration; queued stale presses never start motion.
                keyboard_.update(report(EV_KEY, keyboard_arrows[i], down, now), now);
                keyboard_.update(report(EV_SYN, SYN_REPORT, 0, now), now);
                arrows_[i] = down;
            }
            if (event.key.keysym.scancode == SDL_SCANCODE_0 ||
                event.key.keysym.scancode == SDL_SCANCODE_KP_0) {
                keyboard_.update(report(EV_KEY, KEY_0, down, now), now);
                keyboard_.update(report(EV_SYN, SYN_REPORT, 0, now), now);
                if (down) {
                    keyboard_active_.fill(true);
                    arrows_.fill(false);
                }
            }
            for (unsigned i = 0; i < buttons.size(); ++i) {
                if (!keyboard_buttons[i]) continue;
                auto* source = event.key.keysym.scancode == buttons[i] ? &key_buttons_ :
                               event.key.keysym.scancode == keypad_buttons[i] ? &keypad_buttons_ : nullptr;
                if (!source) continue;
                source->update(report(EV_KEY, keyboard_buttons[i], down, stamp), now, true);
                const auto hits = source->update(report(EV_SYN, SYN_REPORT, 0, stamp), now, true);
                pressed.insert(pressed.end(), hits.begin(), hits.end());
            }
        }
        if ((event.type == SDL_JOYBUTTONDOWN || event.type == SDL_JOYBUTTONUP) && joystick_ &&
            event.jbutton.which == SDL_JoystickInstanceID(joystick_)) {
            const bool down = event.type == SDL_JOYBUTTONDOWN;
            if (down && !recent) continue;
            stick_buttons_.update(report(EV_KEY, event.jbutton.button + 1, down, stamp), now, true);
            const auto hits = stick_buttons_.update(report(EV_SYN, SYN_REPORT, 0, stamp), now, true);
            pressed.insert(pressed.end(), hits.begin(), hits.end());
        }
    }
    const auto checked = clock_();
    const bool expired = !focused_ && checked >= unfocused_since_ &&
                         checked - unfocused_since_ >= focus_timeout_us;
    if (expired && !focus_expired_ && !(joystick_ && SDL_JoystickGetAttached(joystick_)))
        aviator::Logger::warn("Keyboard unfocused for 5 minutes: input invalid; Core CONTROL will enter SAFE");
    focus_expired_ = expired;
    help_->show(window_, focused_, joystick_ && SDL_JoystickGetAttached(joystick_), repaint, expired);
    if (!drained) return true; // Do not renew the lease while input events are queued.
    const auto now = clock_();
    const bool attached = joystick_ && SDL_JoystickGetAttached(joystick_);
    if (attached) {
        const int roll = SDL_JoystickGetAxis(joystick_, config_.roll_axis);
        const int pitch = SDL_JoystickGetAxis(joystick_, config_.pitch_axis);
        stick_.initializePosition(roll, pitch, now);
        if (roll != stick_.roll.value && !arrows_[0] && !arrows_[1]) keyboard_active_[0] = false;
        if (pitch != stick_.pitch.value && !arrows_[2] && !arrows_[3]) keyboard_active_[1] = false;
        if (roll != stick_.roll.value || pitch != stick_.pitch.value) {
            stick_.update(report(EV_ABS, config_.roll_axis, roll, now), now);
            stick_.update(report(EV_ABS, config_.pitch_axis, pitch, now), now);
            stick_.update(report(EV_SYN, SYN_REPORT, 0, now), now);
        }
        stick_.deviceChecked(now);
    }
    keyboard_.deviceChecked(now);
    output = attached ? stick_ : keys_;
    if (!expired && keyboard_active_[0]) {
        output.roll_value = keys_.roll_value;
    }
    if (!expired && keyboard_active_[1]) {
        output.pitch_value = keys_.pitch_value;
    }
    output.device_connected = attached || !expired;
    output.valid = output.device_connected && !output.failed;
    // The merged software target is sampled after this SDL pump, including source changes.
    output.sample_us = output.checked_us = now;
    return true;
}
} // namespace flight_gateway
