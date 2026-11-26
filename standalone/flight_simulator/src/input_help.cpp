#include "input_help.hpp"
#include "clock.hpp"

#include <ft2build.h>
#include FT_FREETYPE_H
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace flight_simulator {
namespace {
constexpr SDL_Color white{232, 238, 246, 255}, muted{165, 182, 202, 255};
constexpr SDL_Color amber{255, 204, 112, 255}, green{117, 222, 176, 255};
constexpr std::array<const char*, 11> key_names{"1", "2", "3", "4", "5", "6",
                                                "7", "8", "9", "0", "-"};
const char* operationName(const std::string& op) {
    if (op == "enter_standby")
        return "回 home / 待机";
    if (op == "grasp_wheel")
        return "握盘";
    if (op == "start_control")
        return "开始控制";
    if (op == "exit_control")
        return "退出控制";
    if (op == "leave_wheel")
        return "松盘 / 撤离";
    if (op == "reset_error")
        return "错误复位（松手回 home）";
    return "未分配";
}
std::string keyFor(const Config& config, const std::string& operation) {
    for (unsigned i = 0; i < config.buttons.size(); ++i)
        if (i != 9 && config.buttons[i] == operation)
            return std::string("[") + key_names[i] + "]";
    return "[未绑定]";
}
struct Font {
    FT_Library library = nullptr;
    FT_Face face = nullptr;
    struct Glyph {
        unsigned width = 0, rows = 0;
        int left = 0, top = 0, advance = 0;
        std::vector<unsigned char> pixels;
    };
    // Reuse rasterized glyphs when refreshing telemetry; font rasterization must
    // not run for every character at the 80 Hz control rate.
    std::map<std::pair<unsigned, unsigned>, Glyph> glyphs;
    Font() {
        if (FT_Init_FreeType(&library))
            throw std::runtime_error("Cannot initialize input help font");
        for (const char* path : {"/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
                                 "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
                                 "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc"}) {
            if (!FT_New_Face(library, path, 0, &face))
                return;
        }
        FT_Done_FreeType(library);
        throw std::runtime_error("Input help needs a Chinese font; install fonts-noto-cjk");
    }
    ~Font() {
        FT_Done_Face(face);
        FT_Done_FreeType(library);
    }
    void text(SDL_Surface* target, int x, int baseline, std::string_view value,
              SDL_Color color = white, unsigned size = 19) {
        FT_Set_Pixel_Sizes(face, 0, size);
        for (size_t i = 0; i < value.size();) {
            auto code = static_cast<unsigned char>(value[i++]);
            unsigned point = code;
            int extra = 0;
            if (code >= 0xf0) {
                point = code & 7;
                extra = 3;
            } else if (code >= 0xe0) {
                point = code & 15;
                extra = 2;
            } else if (code >= 0xc0) {
                point = code & 31;
                extra = 1;
            }
            while (extra-- && i < value.size())
                point = (point << 6) | (value[i++] & 63);
            const auto key = std::make_pair(size, point);
            auto found = glyphs.find(key);
            if (found == glyphs.end()) {
                if (FT_Load_Char(face, point, FT_LOAD_RENDER)) continue;
                const auto& source = *face->glyph;
                Glyph glyph{source.bitmap.width, source.bitmap.rows, source.bitmap_left,
                            source.bitmap_top, int(source.advance.x >> 6), {}};
                for (unsigned row = 0; row < glyph.rows; ++row) {
                    const auto* p = source.bitmap.buffer + row * source.bitmap.pitch;
                    glyph.pixels.insert(glyph.pixels.end(), p, p + glyph.width);
                }
                found = glyphs.emplace(key, std::move(glyph)).first;
            }
            const auto& glyph = found->second;
            for (unsigned row = 0; row < glyph.rows; ++row) {
                const int y = baseline - glyph.top + row;
                if (y < 0 || y >= target->h)
                    continue;
                auto* pixels = reinterpret_cast<Uint32*>(static_cast<Uint8*>(target->pixels) +
                                                         y * target->pitch);
                for (unsigned col = 0; col < glyph.width; ++col) {
                    const int px = x + glyph.left + col;
                    if (px < 0 || px >= target->w)
                        continue;
                    const auto alpha = glyph.pixels[row * glyph.width + col];
                    Uint8 r, g, b;
                    SDL_GetRGB(pixels[px], target->format, &r, &g, &b);
                    const auto blend = [alpha](Uint8 bg, Uint8 fg) {
                        return (bg * (255 - alpha) + fg * alpha) / 255;
                    };
                    pixels[px] = SDL_MapRGB(target->format, blend(r, color.r), blend(g, color.g),
                                            blend(b, color.b));
                }
            }
            x += glyph.advance;
        }
    }
};
} // namespace
struct InputHelp::Monitor {
    Font font;
    std::array<ReceptionSnapshot, 2> live{}, saved{};
    ReceptionSnapshot displayed;
    uint64_t frozen_at = 0, rendered_at = 0, timeout;
    int page = 0;
    bool paused = false, dirty = true;
    std::string notice;
    Surface surface{nullptr, SDL_FreeSurface};
    explicit Monitor(uint64_t timeout_us) : timeout(timeout_us) {
        surface.reset(SDL_CreateRGBSurfaceWithFormat(0, width, height, 32, SDL_PIXELFORMAT_ARGB8888));
        if (!surface) throw std::runtime_error("Cannot create RS422 display surface");
    }
    void show(SDL_Window* window, bool focused, bool attached, bool repaint) {
        const auto now = monotonic_us();
        if (!dirty && !repaint && now - rendered_at < 200000) return;
        rendered_at = now;
        dirty = false;
        const auto& s = (paused ? saved : live).at(page - 1);
        displayed = s;
        auto* canvas = surface.get();
        SDL_FillRect(canvas, nullptr, SDL_MapRGB(canvas->format, 19, 28, 41));
        font.text(canvas, 20, 33, "F1 操作指南   F2 报文 A   F3 报文 B   F4 暂停/恢复   F5 复制 HEX", white, 19);
        const auto time = paused ? frozen_at : now;
        const std::string status = receptionStatus(s, time, timeout);
        const auto color = paused ? amber : status == "LIVE" ? green : amber;
        std::string heading = std::string("64 字节上行 · 通道 ") + (page == 1 ? "A" : "B") + " · ";
        heading += paused ? "暂停快照（历史数据）" : status;
        if (!s.frame.empty() && time >= s.received_us)
            heading += " · " + std::to_string((time - s.received_us) / 1000) + " ms 前收到";
        font.text(canvas, 20, 66, heading, color, 21);
        std::ostringstream meta;
        meta << "串口 " << (s.connected ? "open" : "closed") << "  RX " << s.rx_bytes << " B  接纳 " << s.accepted_frames
             << " 帧  旧序号 " << s.ignored_frames << " 帧  丢弃 " << s.rejected_bytes << " B";
        font.text(canvas, 20, 92, meta.str(), muted, 16);
        if (s.frame.empty()) {
            font.text(canvas, 40, 160, "尚未收到有效的 64 字节上行报文", amber, 24);
            font.text(canvas, 40, 199, "检查串口路径、接线、115200 / 8E1 及双方协议版本。", muted, 18);
        } else {
            for (size_t row = 0; row < 4; ++row) {
                std::ostringstream line;
                line << std::hex << std::uppercase << std::setfill('0') << std::setw(2) << row * 16
                     << "  " << hexBytes(s.frame, row * 16, 16);
                font.text(canvas, 20, 118 + int(row) * 20, line.str(), white, 18);
            }
            font.text(canvas, 20, 205, "偏移", green, 17);
            font.text(canvas, 100, 205, "字段", green, 17);
            font.text(canvas, 298, 205, "原始 HEX", green, 17);
            font.text(canvas, 440, 205, "解析值 / 含义", green, 17);
            int y = 229;
            for (const auto& field : describeUplink(s.frame)) {
                font.text(canvas, 20, y, field.offset, muted, 16);
                font.text(canvas, 100, y, field.name, white, 16);
                font.text(canvas, 298, y, field.hex, muted, 16);
                font.text(canvas, 440, y, field.value, white, 16);
                y += 20;
            }
        }
        font.text(canvas, 20, 661, "roll = raw×50/32767 deg；pitch = raw×75/32767−5 mm；−32768 无效。", muted, 16);
        font.text(canvas, 20, 687, notice.empty() ? "显示最近接纳帧；坏帧不更新。STALE / 暂停只供查看；操作回绕不代表完成。" : notice, amber, 16);
        font.text(canvas, 20, 718, std::string(focused ? "键盘已聚焦" : "键盘未聚焦") +
                  (attached ? " · 摇杆已连接" : " · 摇杆未连接") + " · 切页和暂停不改变收发及操作按键", muted, 16);
        auto* target = SDL_GetWindowSurface(window);
        if (!target || SDL_BlitSurface(canvas, nullptr, target, nullptr) != 0 || SDL_UpdateWindowSurface(window) != 0)
            throw std::runtime_error("Cannot display RS422 feedback: " + std::string(SDL_GetError()));
    }
};
InputHelp::InputHelp(const Config& config)
    : monitor_(std::make_unique<Monitor>(config.rs422.receive_timeout_ms * 1000ULL)) {
    Font font;
    for (int state = 0; state < 5; ++state) {
        const bool focused = state & 1, attached = state & 2;
        auto& panel = panels_[state];
        panel.reset(SDL_CreateRGBSurfaceWithFormat(0, width, height, 32, SDL_PIXELFORMAT_ARGB8888));
        if (!panel)
            throw std::runtime_error("Cannot create input help surface: " +
                                     std::string(SDL_GetError()));
        auto* surface = panel.get();
        const auto box = [&](SDL_Rect area, Uint8 r, Uint8 g, Uint8 b) {
            SDL_FillRect(surface, &area, SDL_MapRGB(surface->format, r, g, b));
        };
        SDL_FillRect(surface, nullptr, SDL_MapRGB(surface->format, 19, 28, 41));
        font.text(surface, 28, 45, "flight_simulator  /  输入操作指南", white, 26);
        font.text(surface, 28, 79,
                  focused ? "键盘：已聚焦，可接收按键" : "键盘：未聚焦，请点击本窗口",
                  focused ? green : amber);
        font.text(surface, 475, 79,
                  attached ? "摇杆：已连接，可后台使用"
                           : (config.simulator.require_joystick ? "摇杆：未连接，停止发送"
                                                                : "摇杆：未连接，仅使用键盘"),
                  attached ? green : muted);
        box({24, 99, 832, 82}, 65, 46, 26);
        font.text(surface, 40, 130,
                  config.simulator.require_joystick ? "飞控模拟：手柄断开即停止两路下行发送"
                                                    : "飞控模拟：无手柄且窗口失焦即停止发送",
                  amber, 22);
        font.text(surface, 40, 161, "操作连续发送 10 拍；回绕不等于完成。F2 / F3 查看上行报文。", white);
        font.text(surface, 28, 215, "方向键操作", green, 21);
        font.text(surface, 28, 246, "左 / 右：roll + / −（deg）    上 / 下：pitch + / −（mm）");
        font.text(surface, 28, 274, "按住累加，松开或到限位后保持；[0] 目标归零到 0° / 0 mm。", muted);
        font.text(surface, 28, 302,
                  "反向键同时按下停止累加；移动摇杆对应轴可接管。速度 / 限位按配置生效。", muted,
                  18);
        font.text(surface, 28, 340, "状态操作键（主键盘 / 小键盘数字键均可）", green, 21);
        // Six rows: keys 1..6 on the left, 7..9 / 0 / minus on the right.
        for (unsigned i = 0; i < config.buttons.size(); ++i) {
            const auto text =
                std::string("[") + key_names[i] + "]  " +
                (i == 9 ? "目标归零（0° / 0 mm）" : operationName(config.buttons[i]));
            font.text(surface, i < 6 ? 28 : 475, 372 + (i % 6) * 27, text,
                      i != 9 && config.buttons[i].empty() ? muted : white, 18);
        }
        box({24, 528, 832, 152}, 29, 45, 62);
        font.text(surface, 40, 558, "ERROR 恢复：每一步都要等待状态完成", green, 21);
        font.text(surface, 40, 588, "故障排除、设备停稳后，按 " + keyFor(config, "reset_error") + " 复位，等待松手并回 home。", white, 18);
        font.text(surface, 40, 615, "复位中显示 RELEASING；到达 home 并停稳后进入 STANDBY。", white, 18);
        font.text(surface, 40, 642,
                  "再按 " + keyFor(config, "grasp_wheel") + " 握盘 → FOLLOWING，再按 " +
                      keyFor(config, "start_control") + " 开始控制 → CONTROL。",
                  white, 18);
        font.text(surface, 40, 668, "复位成功进入 STANDBY；手或机械臂无法运动时保持 ERROR。", muted,
                  17);
        font.text(surface, 28, 718, "F1 操作指南 · F2 报文 A · F3 报文 B · F4 暂停/恢复 · F5 复制 HEX",
                  muted, 18);
    }
}
InputHelp::~InputHelp() = default;
void InputHelp::feedback(const std::array<ReceptionSnapshot, 2>& snapshots) {
    monitor_->live = snapshots;
}
bool InputHelp::viewKey(SDL_Scancode key) {
    auto& m = *monitor_;
    if (key >= SDL_SCANCODE_F1 && key <= SDL_SCANCODE_F3) {
        m.page = key - SDL_SCANCODE_F1;
        m.dirty = true;
        m.notice.clear();
        visible_ = -1;
        return true;
    }
    if (!m.page) return false;
    if (key == SDL_SCANCODE_F4) {
        m.paused = !m.paused;
        if (m.paused) {
            m.saved = m.live;
            m.frozen_at = monotonic_us();
        }
        m.notice.clear();
        m.dirty = true;
        return true;
    }
    if (key == SDL_SCANCODE_F5) {
        m.notice = m.displayed.frame.empty() ? "尚无可复制报文"
            : SDL_SetClipboardText(hexBytes(m.displayed.frame).c_str()) == 0 ? "已复制当前显示的 64 字节 HEX"
            : "复制失败：" + std::string(SDL_GetError());
        m.dirty = true;
        return true;
    }
    return false;
}
void InputHelp::show(SDL_Window* window, bool focused, bool attached, bool repaint,
                     bool focus_expired) {
    if (monitor_->page) {
        monitor_->show(window, focused, attached, repaint);
        return;
    }
    const int state = focus_expired && !attached ? 4 : int(focused) | (int(attached) << 1);
    if (!repaint && visible_ == state)
        return;
    auto* target = SDL_GetWindowSurface(window);
    if (!target || SDL_BlitSurface(panels_[state].get(), nullptr, target, nullptr) != 0 ||
        SDL_UpdateWindowSurface(window) != 0)
        throw std::runtime_error("Cannot display input help: " + std::string(SDL_GetError()));
    visible_ = state;
}
} // namespace flight_simulator
