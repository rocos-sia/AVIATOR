#include "t6.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>

namespace t6 = aviator::t6;
using Clock = std::chrono::steady_clock;

// ---- timing helpers --------------------------------------------------------

template <typename F>
std::vector<double> time_us(F&& f, int n, int warmup) {
    for (int i = 0; i < warmup; ++i) f();
    std::vector<double> us(n);
    for (int i = 0; i < n; ++i) {
        const auto t0 = Clock::now();
        f();
        const auto t1 = Clock::now();
        us[i] = std::chrono::duration<double, std::micro>(t1 - t0).count();
    }
    return us;
}

void report(const char* name, std::vector<double> us, double budget_us) {
    std::sort(us.begin(), us.end());
    const std::size_t n = us.size();
    const double mean = std::accumulate(us.begin(), us.end(), 0.0) / n;
    double var = 0;
    for (double v : us) var += (v - mean) * (v - mean);
    const double stddev = std::sqrt(var / n);
    auto pct = [&](double p) { return us[std::min<std::size_t>(n - 1, std::size_t(p * (n - 1)))]; };
    const std::size_t over = std::count_if(us.begin(), us.end(),
                                           [&](double v) { return v > budget_us; });
    std::cout << "\n== " << name << " ==\n"
              << std::fixed << std::setprecision(1)
              << "  n=" << n << " (us)  min=" << us.front() << "  mean=" << mean
              << "  p50=" << pct(0.50) << "  p90=" << pct(0.90)
              << "  p95=" << pct(0.95) << "  p99=" << pct(0.99)
              << "  p99.9=" << pct(0.999) << "  max=" << us.back()
              << "\n  stddev=" << stddev << "  -> "
              << std::setprecision(1) << 1e6 / mean << " Hz sustained mean"
              << std::setprecision(1)
              << "\n  budget " << budget_us << " us (" << 1e6 / budget_us << " Hz): "
              << over << "/" << n << " exceed (" << 100.0 * over / n << "%)\n";
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: t6_bench MODEL.onnx LUT_DIRECTORY [N] [warmup]\n";
        return 2;
    }
    const int n = argc > 3 ? std::atoi(argv[3]) : 10000;
    const int warmup = argc > 4 ? std::atoi(argv[4]) : 500;
    constexpr double kBudget = 10000.0; // 10 ms = 100 Hz cycle budget

    try {
        t6::Lookup lookup(argv[2]);
        t6::Actor actor(argv[1]);

        // ---- 1. model output: Actor::predict (ONNX inference) -------------
        std::array<float, 40> obs{};
        obs[0] = 0.0f; obs[1] = -0.08f; obs[2] = 0.0f; obs[3] = 0.0f;
        obs[4] = std::sin(0.05); obs[5] = std::cos(0.05);
        obs[6] = std::sin(0.03); obs[7] = std::cos(0.03);
        obs[8] = 0.1f; obs[9] = 0.1f; obs[10] = 0.1f; obs[11] = 0.1f;
        obs[12] = 0.008f; obs[13] = 0.04f;
        auto us_predict = time_us([&] { (void)actor.predict(obs); }, n, warmup);
        report("Actor::predict  (ONNX [1,40]->[1,2], intra-op threads=1)", us_predict, kBudget);

        // ---- 2. forward LUT: Lookup::query --------------------------------
        const t6::Vec2 x{0.0, -0.08};
        const t6::Vec2 phi{0.05, 0.03};
        auto us_query = time_us([&] { (void)lookup.query(x, phi); }, n, warmup);
        report("Lookup::query   (trilinear forward LUT)", us_query, kBudget);

        // ---- 3. inverse LUT: Lookup::estimate_phase -----------------------
        const t6::Joints measured_q = lookup.query(x, {0, 0}).q;
        t6::PhaseSearch search;
        search.previous = t6::Vec2{0.0, 0.0};
        search.max_phase_change = 0.025;
        search.continuity_weight = 0.001;
        auto us_estimate = time_us(
            [&] { (void)lookup.estimate_phase(x, measured_q, search); }, n, warmup);
        report("Lookup::estimate_phase (inverse-LUT phase search)", us_estimate, kBudget);

        // ---- 4. full closed-loop cycle: FeedbackController::step ----------
        t6::FeedbackLimits limits;
        limits.max_rms_joint_error = 0.02;
        limits.max_joint_error = 0.05;
        limits.max_phase_change = 0.025;
        limits.continuity_weight = 0.001;
        limits.max_joint_speed = 1.5;
        limits.min_clearance = 0.005;
        limits.time_tolerance = 0.002;
        t6::FeedbackController ctrl(lookup, actor, limits);

        t6::Feedback fb;
        fb.x = x;
        fb.xdot = {0, 0};
        fb.q = measured_q;
        fb.time = 0.0;
        const auto rst = ctrl.reset(fb);
        if (rst.status != t6::Status::ok) {
            std::cerr << "reset failed: " << status_name(rst.status) << "\n";
            return 1;
        }
        fb.time = 0.01;
        fb.q = rst.evaluated_sample.q; // on-manifold q at the acquired phase

        auto rearm = [&]() {
            t6::Feedback f = fb;
            f.q = measured_q;
            const auto rr = ctrl.reset(f);
            if (rr.status == t6::Status::ok) fb.q = rr.evaluated_sample.q;
        };

        std::vector<double> us_step(n);
        std::size_t step_failures = 0;
        for (int i = 0; i < warmup; ++i) {
            const auto out = ctrl.step(fb, fb.x);
            if (out.has_command) fb.q = out.q; else rearm();
            fb.time += 0.01;
        }
        step_failures = 0;
        for (int i = 0; i < n; ++i) {
            const auto t0 = Clock::now();
            const auto out = ctrl.step(fb, fb.x);
            const auto t1 = Clock::now();
            us_step[i] = std::chrono::duration<double, std::micro>(t1 - t0).count();
            if (out.has_command) fb.q = out.q; else { ++step_failures; rearm(); }
            fb.time += 0.01;
        }
        report("FeedbackController::step (full 100 Hz cycle)", us_step, kBudget);
        std::cout << "  step failures (rearm): " << step_failures << "/" << n << "\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
    return 0;
}
