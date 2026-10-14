#pragma once
#include <array>
#include <cmath>

namespace aviator {
// Measure continuous stillness using feedback timestamps, never repeated cache reads.
class ImpedanceSettling {
public:
    static constexpr double speed_limit = 0.02;
    static constexpr double stable_seconds = 0.3;
    static constexpr double feedback_max_age = 0.1;
    static constexpr double timeout_seconds = 5.0;

    bool observe(double now, double stamp, bool valid, const std::array<double, 7>& velocity) {
        bool quiet = valid && std::isfinite(now) && std::isfinite(stamp) && stamp > 0 &&
                     stamp <= now && now - stamp < feedback_max_age;
        for (double v : velocity)
            quiet = quiet && std::isfinite(v) && std::abs(v) < speed_limit;
        if (!quiet) {
            since_ = previous_ = 0;
            return false;
        }
        if (!since_ || stamp < previous_ || stamp - previous_ >= feedback_max_age)
            since_ = stamp;
        previous_ = stamp;
        return duration() >= stable_seconds;
    }
    double duration() const { return since_ ? previous_ - since_ : 0; }
private:
    double since_ = 0, previous_ = 0;
};
}
