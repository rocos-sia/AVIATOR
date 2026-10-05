#pragma once

#include "gateway.hpp"
#include "input_help.hpp"
#include <SDL.h>
#include <functional>

namespace flight_gateway {
// SDL owns all device IO. Reuse the existing report decoder and keyboard integrator.
class SdlInput {
public:
    explicit SdlInput(const Config& config,
                      std::function<std::uint64_t()> clock = aviator::monotonic_us);
    ~SdlInput();
    SdlInput(const SdlInput&) = delete;
    SdlInput& operator=(const SdlInput&) = delete;
    bool poll(JoystickSample& output, std::vector<unsigned>& pressed);
private:
    void openJoystick();
    void resetKeyboard(std::uint64_t now);
    Config config_;
    std::function<std::uint64_t()> clock_;
    static constexpr std::uint64_t focus_timeout_us = 300000000;
    std::uint64_t unfocused_since_ = 0;
    bool focus_expired_ = false;
    std::array<bool, 2> keyboard_active_{};
    std::unique_ptr<InputHelp> help_;
    SDL_Window* window_ = nullptr;
    SDL_Joystick* joystick_ = nullptr;
    JoystickSample stick_, keys_;
    KeyboardInput keyboard_;
    JoystickButtons stick_buttons_, key_buttons_, keypad_buttons_;
    std::array<bool, 4> arrows_{};
    bool focused_ = false;
};
} // namespace flight_gateway
