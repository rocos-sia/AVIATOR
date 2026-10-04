#pragma once

#include "gateway.hpp"
#include "input_help.hpp"
#include <SDL.h>

namespace flight_gateway {
// SDL owns all device IO. Reuse the existing report decoder and keyboard integrator.
class SdlInput {
public:
    explicit SdlInput(const Config& config);
    ~SdlInput();
    SdlInput(const SdlInput&) = delete;
    SdlInput& operator=(const SdlInput&) = delete;
    bool poll(JoystickSample& output, std::vector<unsigned>& pressed);
private:
    void openJoystick();
    void resetKeyboard(std::uint64_t now);
    Config config_;
    std::unique_ptr<InputHelp> help_;
    SDL_Window* window_ = nullptr;
    SDL_Joystick* joystick_ = nullptr;
    JoystickSample stick_, keys_;
    KeyboardInput keyboard_;
    JoystickButtons stick_buttons_, key_buttons_;
    std::array<bool, 4> arrows_{};
    bool focused_ = false;
};
} // namespace flight_gateway
