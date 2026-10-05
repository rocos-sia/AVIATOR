#include "gateway.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
bool near(double a, double b) { return std::abs(a - b) < 1e-9; }
input_event event(unsigned type, unsigned code, int value, std::uint64_t time) {
    input_event e{};
    e.type = type; e.code = code; e.value = value;
    e.input_event_sec = time / 1000000; e.input_event_usec = time % 1000000;
    return e;
}
struct Keyboard {
    flight_gateway::JoystickSample sample{{ABS_X, -1, 1, 0}, {ABS_Y, -1, 1, 0}};
    flight_gateway::KeyboardInput input{sample, {0.5, 2.0, 0.75, 0.5}};
    flight_gateway::JoystickButtons buttons;
    std::uint64_t now = 1000000;
    Keyboard() {
        buttons.codes = flight_gateway::keyboard_buttons;
        input.deviceChecked(now);
    }
    std::vector<unsigned> send(unsigned type, unsigned code, int value) {
        auto e = event(type, code, value, now);
        input.update(e, now);
        return buttons.update(e, now, !sample.failed);
    }
    std::vector<unsigned> key(unsigned code, int value) {
        send(EV_KEY, code, value);
        auto pressed = send(EV_SYN, SYN_REPORT, 0);
        input.deviceChecked(now);
        return pressed;
    }
    void tick(std::uint64_t dt = 20000) { now += dt; input.deviceChecked(now); }
};
}
int main() {
    try {
        Keyboard k;
        check(k.sample.fresh(k.now, 100000) && k.sample.roll_value == 0 &&
                  k.sample.pitch_value == 0, "keyboard starts centered and valid");
        k.key(KEY_RIGHT, 1); k.key(KEY_UP, 1);
        for (int i = 0; i < 10; ++i) k.tick();
        check(near(k.sample.roll_value, 0.1) && near(k.sample.pitch_value, -0.4),
              "independent time-based simultaneous axes");
        for (int i = 0; i < 100; ++i) k.tick();
        check(k.sample.roll_value == 0.75 && k.sample.pitch_value == -0.5,
              "holding without repeat reaches configured limits");
        k.key(KEY_RIGHT, 0);
        check(k.sample.roll_value == 0.75 && k.sample.pitch_value == -0.5,
              "release holds the last position");
        k.key(KEY_DOWN, 1);
        check(k.sample.pitch_value == -0.5, "opposing directions hold");
        k.tick(); check(k.sample.pitch_value == -0.5, "opposing keys remain stationary");
        k.key(KEY_UP, 0); k.tick();
        check(near(k.sample.pitch_value, -0.46), "remaining direction resumes from held target");
        k.key(KEY_DOWN, 0);
        check(k.key(KEY_0, 1).empty(), "zero must not send a state request");
        k.key(KEY_0, 0); k.tick();
        check(k.sample.roll_value == 0 && k.sample.pitch_value == 0, "zero returns both targets to origin");
        k.key(KEY_LEFT, 1); k.tick();
        check(near(k.sample.roll_value, -0.01), "left is negative");
        k.sample.roll.inverted = true; k.sample.pitch.inverted = true;
        k.key(KEY_DOWN, 1); k.tick();
        check(near(k.sample.roll_value, 0.02) && near(k.sample.pitch_value, -0.04),
              "keyboard obeys axis inversion");
        std::string payload, error;
        auto message = flight_gateway::command(k.sample, "keyboard-test", "boot", 1, k.now, 1, 100000);
        check(aviator::encode(message, payload, error), "keyboard command encodes");
        aviator::InputPolicy policy;
        policy.publisher_id = "flight_gateway"; policy.session_id = "keyboard-test";
        policy.clock_id = "boot"; policy.source = "JOYSTICK";
        policy.allow_joystick_position_hold = true;
        aviator::InputGuard guard(policy);
        check(guard.accept(message, k.now, error), "existing Core accepts keyboard commands");
        for (unsigned i = 0; i < flight_gateway::keyboard_buttons.size(); ++i) {
            const auto code = flight_gateway::keyboard_buttons[i];
            if (!code) continue;
            const auto pressed = k.key(code, 1);
            check(pressed == std::vector<unsigned>{i}, "number key maps to button index");
            check(k.key(code, 2).empty() && k.key(code, 1).empty(), "repeat cannot retrigger request");
            check(k.key(code, 0).empty(), "release does not request state change");
            check(k.key(code, 1) == std::vector<unsigned>{i}, "new press retriggers");
            k.key(code, 0);
        }
        k.key(KEY_0, 1); k.tick();
        check(k.sample.roll_value == 0 && k.sample.pitch_value == 0, "zero stops already held arrows");
        k.key(KEY_LEFT, 2); k.tick();
        check(k.sample.roll_value == 0, "held arrow cannot undo zero without release");
        Keyboard startup;
        startup.input.suppressHeld(KEY_RIGHT); startup.buttons.held[0] = true;
        startup.key(KEY_RIGHT, 2); startup.tick();
        check(startup.sample.roll_value == 0 && startup.key(KEY_1, 1).empty(),
              "startup held keys do not move or request");
        startup.key(KEY_RIGHT, 0); startup.key(KEY_RIGHT, 1); startup.tick();
        check(startup.sample.roll_value > 0, "startup arrow works after release");
        Keyboard partial;
        partial.send(EV_KEY, KEY_RIGHT, 1); partial.tick();
        check(partial.sample.checked_us == 1000000, "partial frame cannot renew lease");
        partial.tick(100000);
        check(!partial.sample.fresh(partial.now, 100000), "partial frame expires");
        Keyboard lost;
        lost.key(KEY_RIGHT, 1); lost.tick();
        lost.send(EV_SYN, SYN_DROPPED, 0); lost.tick();
        check(lost.sample.failed && !lost.sample.fresh(lost.now, 100000), "dropped events latch invalid");
        lost.key(KEY_RIGHT, 0); check(lost.sample.failed, "release cannot revive failed input");
        for (const auto stamp : {999999ULL, 1000001ULL}) {
            Keyboard bad;
            bad.send(EV_SYN, SYN_REPORT, 0);
            bad.input.update(event(EV_KEY, KEY_RIGHT, 1, stamp), bad.now);
            check(bad.sample.failed, "backward/future timestamp rejected");
        }
        Keyboard stale;
        stale.now += 100000;
        stale.input.update(event(EV_KEY, KEY_RIGHT, 1, 1000000), stale.now);
        check(stale.sample.failed, "stale key event rejected");
        Keyboard disconnected;
        disconnected.sample.invalidate(); disconnected.tick();
        check(!disconnected.sample.valid, "successful check cannot revive disconnect");
        std::cout << "keyboard gateway tests passed\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
