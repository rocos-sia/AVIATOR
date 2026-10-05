#pragma once

#include "config.hpp"
#include <SDL.h>
#include <array>
#include <memory>

namespace flight_gateway {
class InputHelp {
public:
    explicit InputHelp(const Config& config);
    void show(SDL_Window* window, bool focused, bool attached, bool repaint = false, bool focus_expired = false);
    static constexpr int width = 880, height = 740;
private:
    using Surface = std::unique_ptr<SDL_Surface, decltype(&SDL_FreeSurface)>;
    // Pre-render once: text/font work must not delay the control input loop.
    std::array<Surface, 5> panels_{{{nullptr, SDL_FreeSurface}, {nullptr, SDL_FreeSurface},
                                  {nullptr, SDL_FreeSurface}, {nullptr, SDL_FreeSurface},
                                  {nullptr, SDL_FreeSurface}}};
    int visible_ = -1;
};
} // namespace flight_gateway
