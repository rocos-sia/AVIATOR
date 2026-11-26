#pragma once

#include "config.hpp"
#include "rs422_display.hpp"
#include <SDL.h>
#include <array>
#include <memory>

namespace flight_simulator {
class InputHelp {
  public:
    explicit InputHelp(const Config& config);
    ~InputHelp();
    void feedback(const std::array<ReceptionSnapshot, 2>& snapshots);
    bool viewKey(SDL_Scancode key);
    void show(SDL_Window* window, bool focused, bool attached, bool repaint = false,
              bool focus_expired = false);
    static constexpr int width = 880, height = 740;

  private:
    using Surface = std::unique_ptr<SDL_Surface, decltype(&SDL_FreeSurface)>;
    // Pre-render once: text/font work must not delay the control input loop.
    std::array<Surface, 5> panels_{{{nullptr, SDL_FreeSurface},
                                    {nullptr, SDL_FreeSurface},
                                    {nullptr, SDL_FreeSurface},
                                    {nullptr, SDL_FreeSurface},
                                    {nullptr, SDL_FreeSurface}}};
    int visible_ = -1;
    struct Monitor;
    std::unique_ptr<Monitor> monitor_;
};
} // namespace flight_simulator
