#include "ImpedanceSettling.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

void check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}

int main() try {
    using aviator::ImpedanceSettling;
    std::array<double, 7> velocity{};
    ImpedanceSettling arms[2];
    // One quiet arm cannot allow a switch while the other is still moving.
    for (int k = 0; k <= 80; ++k) {
        const double t = 1 + k * .01;
        const bool left = arms[0].observe(t, t, true, velocity);
        auto right_velocity = velocity;
        if (k < 40) right_velocity[6] = -.021;
        const bool right = arms[1].observe(t, t, true, right_velocity);
        if (k < 70) check(!(left && right), "switched before both arms were continuously quiet");
        if (k >= 71) check(left && right, "quiet dual-arm feedback did not settle");
    }
    // A brief speed excursion (including exactly the threshold) restarts the window.
    velocity[2] = .02;
    check(!arms[0].observe(1.81, 1.81, true, velocity), "speed threshold accepted");
    velocity[2] = 0;
    for (int k = 0; k <= 31; ++k) {
        const double t = 1.82 + k * .01;
        const bool ready = arms[0].observe(t, t, true, velocity);
        if (k < 30) check(!ready, "speed excursion did not restart observation");
        if (k == 31) check(ready, "fresh quiet window failed to recover");
    }
    ImpedanceSettling cached;
    for (int k = 0; k < 500; ++k)
        check(!cached.observe(3 + k * .001, 3, true, velocity), "cached feedback counted as stillness");
    ImpedanceSettling gap;
    for (int k = 0; k < 25; ++k) gap.observe(4 + k * .01, 4 + k * .01, true, velocity);
    check(!gap.observe(4.5, 4.5, true, velocity), "feedback gap counted toward stable window");
    check(!gap.observe(4.51, 4.51, false, velocity), "invalid feedback accepted");
    check(!gap.observe(4.52, 4.53, true, velocity), "future feedback accepted");
    for (int k = 0; k < 35; ++k) gap.observe(5 + k * .01, 5 + k * .01, true, velocity);
    check(!gap.observe(5.35, 5.30, true, velocity), "backward timestamp did not reset window");
    velocity[0] = std::numeric_limits<double>::quiet_NaN();
    check(!gap.observe(5.36, 5.36, true, velocity), "nonfinite speed accepted");
    check(gap.duration() == 0, "invalid speed retained stable duration");
    std::cout << "impedance settling tests passed\n";
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}
