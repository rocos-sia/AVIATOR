#include "input_help.hpp"

#include <ft2build.h>
#include FT_FREETYPE_H
#include <algorithm>
#include <stdexcept>
#include <string_view>

namespace flight_gateway {
namespace {
constexpr SDL_Color white{232, 238, 246, 255}, muted{165, 182, 202, 255};
constexpr SDL_Color amber{255, 204, 112, 255}, green{117, 222, 176, 255};
constexpr std::array<const char*, 11> key_names{"1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "-"};
const char* operationName(const std::string& op) {
    if (op == "enter_standby") return "回 home / 待机";
    if (op == "grasp_wheel") return "握盘";
    if (op == "start_control") return "开始控制";
    if (op == "exit_control") return "退出控制";
    if (op == "leave_wheel") return "松盘 / 撤离";
    if (op == "reset_error") return "故障确认（ERROR → SAFE）";
    return "未分配";
}
std::string keyFor(const Config& config, const std::string& operation) {
    for (unsigned i = 0; i < config.buttons.size(); ++i)
        if (i != 9 && config.buttons[i] == operation) return std::string("[") + key_names[i] + "]";
    return "[未绑定]";
}
struct Font {
    FT_Library library = nullptr;
    FT_Face face = nullptr;
    Font() {
        if (FT_Init_FreeType(&library)) throw std::runtime_error("Cannot initialize input help font");
        for (const char* path : {"/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
                                 "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
                                 "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc"}) {
            if (!FT_New_Face(library, path, 0, &face)) return;
        }
        FT_Done_FreeType(library);
        throw std::runtime_error("Input help needs a Chinese font; install fonts-noto-cjk");
    }
    ~Font() { FT_Done_Face(face); FT_Done_FreeType(library); }
    void text(SDL_Surface* target, int x, int baseline, std::string_view value,
              SDL_Color color = white, unsigned size = 19) {
        FT_Set_Pixel_Sizes(face, 0, size);
        for (size_t i = 0; i < value.size();) {
            auto code = static_cast<unsigned char>(value[i++]);
            unsigned point = code;
            int extra = 0;
            if (code >= 0xf0) { point = code & 7; extra = 3; }
            else if (code >= 0xe0) { point = code & 15; extra = 2; }
            else if (code >= 0xc0) { point = code & 31; extra = 1; }
            while (extra-- && i < value.size()) point = (point << 6) | (value[i++] & 63);
            if (FT_Load_Char(face, point, FT_LOAD_RENDER)) continue;
            const auto& glyph = *face->glyph;
            for (unsigned row = 0; row < glyph.bitmap.rows; ++row) {
                const int y = baseline - glyph.bitmap_top + row;
                if (y < 0 || y >= target->h) continue;
                auto* pixels = reinterpret_cast<Uint32*>(static_cast<Uint8*>(target->pixels) + y * target->pitch);
                for (unsigned col = 0; col < glyph.bitmap.width; ++col) {
                    const int px = x + glyph.bitmap_left + col;
                    if (px < 0 || px >= target->w) continue;
                    const auto alpha = glyph.bitmap.buffer[row * glyph.bitmap.pitch + col];
                    Uint8 r, g, b;
                    SDL_GetRGB(pixels[px], target->format, &r, &g, &b);
                    const auto blend = [alpha](Uint8 bg, Uint8 fg) { return (bg * (255 - alpha) + fg * alpha) / 255; };
                    pixels[px] = SDL_MapRGB(target->format, blend(r, color.r), blend(g, color.g), blend(b, color.b));
                }
            }
            x += glyph.advance.x >> 6;
        }
    }
};
}
InputHelp::InputHelp(const Config& config) {
    Font font;
    for (int state = 0; state < 5; ++state) {
        const bool focused = state & 1, attached = state & 2;
        auto& panel = panels_[state];
        panel.reset(SDL_CreateRGBSurfaceWithFormat(0, width, height, 32, SDL_PIXELFORMAT_ARGB8888));
        if (!panel) throw std::runtime_error("Cannot create input help surface: " + std::string(SDL_GetError()));
        auto* surface = panel.get();
        const auto box = [&](SDL_Rect area, Uint8 r, Uint8 g, Uint8 b) {
            SDL_FillRect(surface, &area, SDL_MapRGB(surface->format, r, g, b));
        };
        SDL_FillRect(surface, nullptr, SDL_MapRGB(surface->format, 19, 28, 41));
        font.text(surface, 28, 45, "flight_gateway  /  输入操作指南", white, 26);
        font.text(surface, 28, 79, focused ? "键盘：已聚焦，可接收按键" : "键盘：未聚焦，请点击本窗口", focused ? green : amber);
        font.text(surface, 475, 79, attached ? "摇杆：已连接，可后台使用" : "摇杆：未连接，仅使用键盘", attached ? green : muted);
        box({24, 99, 832, 82}, 65, 46, 26);
        font.text(surface, 40, 130, state == 4 ? "失焦已满 5 分钟：键盘输入失效，CONTROL 将进入 SAFE" :
                  "仅用键盘时：失焦保持目标，满 5 分钟后进入 SAFE", amber, 22);
        font.text(surface, 40, 161, "失焦立即停止累加；超时进入 SAFE 后，聚焦不会自动恢复控制。", white);
        font.text(surface, 28, 215, "方向键操作", green, 21);
        font.text(surface, 28, 246, "左 / 右：roll − / +       上 / 下：pitch − / +");
        font.text(surface, 28, 274, "按住累加，松开或到限位后保持；[0] 回到初始原点。", muted);
        font.text(surface, 28, 302, "反向键同时按下停止累加；移动摇杆对应轴可接管。速度 / 限位按配置生效。", muted, 18);
        font.text(surface, 28, 340, "状态操作键（主键盘 / 小键盘数字键均可）", green, 21);
        // Six rows: keys 1..6 on the left, 7..9 / 0 / minus on the right.
        for (unsigned i = 0; i < config.buttons.size(); ++i) {
            const auto text = std::string("[") + key_names[i] + "]  " + (i == 9 ? "目标回原点（roll / pitch = 0）" : operationName(config.buttons[i]));
            font.text(surface, i < 6 ? 28 : 475, 372 + (i % 6) * 27, text,
                      i != 9 && config.buttons[i].empty() ? muted : white, 18);
        }
        box({24, 528, 832, 152}, 29, 45, 62);
        font.text(surface, 40, 558, "SAFE 恢复：每一步都要等待状态完成", green, 21);
        font.text(surface, 40, 588, "先聚焦本窗口，确认输入有效、设备就绪并已停止。", white, 18);
        font.text(surface, 40, 615, "仍在握盘：" + keyFor(config, "leave_wheel") + " 松盘撤离 → STANDBY；已脱离可用 " +
                  keyFor(config, "enter_standby") + " 回 home。", white, 18);
        font.text(surface, 40, 642, "再按 " + keyFor(config, "grasp_wheel") + " 握盘 → FOLLOWING，再按 " +
                  keyFor(config, "start_control") + " 开始控制 → CONTROL。", white, 18);
        font.text(surface, 40, 668, "SAFE 不能直接开始控制；故障确认键只用于 ERROR → SAFE。", muted, 17);
        font.text(surface, 28, 718, "长按不会重复请求。关注 Core 状态 / 服务回复；关闭此窗口会退出网关。", muted, 18);
    }
}
void InputHelp::show(SDL_Window* window, bool focused, bool attached, bool repaint, bool focus_expired) {
    const int state = focus_expired && !attached ? 4 : int(focused) | (int(attached) << 1);
    if (!repaint && visible_ == state) return;
    auto* target = SDL_GetWindowSurface(window);
    if (!target || SDL_BlitSurface(panels_[state].get(), nullptr, target, nullptr) != 0 ||
        SDL_UpdateWindowSurface(window) != 0)
        throw std::runtime_error("Cannot display input help: " + std::string(SDL_GetError()));
    visible_ = state;
}
} // namespace flight_gateway
