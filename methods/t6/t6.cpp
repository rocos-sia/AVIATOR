#include "t6.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <regex>
#include <stdexcept>

namespace aviator::t6 {
namespace {
constexpr double kDt = 0.01;
constexpr double kPhaseRate = 1.5;
constexpr double kClearance = 0.005;
constexpr double kJointSpeed = 1.5;
constexpr double kTol = 1e-9;

template <typename T>
std::vector<T> read_array(const std::string& path, std::size_t count) {
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if (!in || in.tellg() != static_cast<std::streamoff>(count * sizeof(T)))
        throw std::runtime_error("Missing or wrong-sized t6 data: " + path);
    in.seekg(0);
    std::vector<T> data(count);
    if (!in.read(reinterpret_cast<char*>(data.data()), count * sizeof(T)))
        throw std::runtime_error("Cannot read t6 data: " + path);
    return data;
}

double manifest_number(const std::string& text, const std::string& key) {
    const std::regex pattern("\"" + key + "\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");
    std::smatch match;
    if (!std::regex_search(text, match, pattern)) throw std::runtime_error("Missing manifest key: " + key);
    return std::stod(match[1]);
}

constexpr std::array<double, 7> lower = {
    -3.1067, 84.5 * 3.14159265358979323846 / 180.0, -3.1067,
    -1.0472, -3.1067, -1.0472, -1.0472};
constexpr std::array<double, 7> upper = {
    3.1067, 94.5 * 3.14159265358979323846 / 180.0, 3.1067,
    2.5307, 3.1067, 1.0472, 1.0472};
} // namespace

Lookup::Lookup(const std::string& directory) {
    std::ifstream in(directory + "/manifest.json");
    if (!in) throw std::runtime_error("Cannot open LUT manifest");
    const std::string manifest((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    n_th_ = static_cast<int>(manifest_number(manifest, "n_theta"));
    n_s_ = static_cast<int>(manifest_number(manifest, "n_s"));
    n_p_ = static_cast<int>(manifest_number(manifest, "n_phi"));
    th_min_ = manifest_number(manifest, "th_min");
    th_max_ = manifest_number(manifest, "th_max");
    s_min_ = manifest_number(manifest, "s_min");
    s_max_ = manifest_number(manifest, "s_max");
    if (n_th_ < 2 || n_s_ < 2 || n_p_ < 2 || th_max_ <= th_min_ || s_max_ <= s_min_)
        throw std::runtime_error("Invalid LUT dimensions/ranges");
    const auto points = static_cast<std::size_t>(n_th_) * n_s_ * n_p_;
    const auto grids = static_cast<std::size_t>(n_th_) * n_s_;
    phi_ = read_array<float>(directory + "/phi.bin", n_p_);
    if (!std::is_sorted(phi_.begin(), phi_.end()) || phi_.front() == phi_.back())
        throw std::runtime_error("Invalid phase axis");
    q_l_ = read_array<float>(directory + "/qL.bin", points * 7);
    q_r_ = read_array<float>(directory + "/qR.bin", points * 7);
    d_l_ = read_array<float>(directory + "/dL.bin", points);
    d_r_ = read_array<float>(directory + "/dR.bin", points);
    safe_ = read_array<float>(directory + "/safe.bin", grids * 4);
    branch_ = read_array<std::uint8_t>(directory + "/branch.bin", grids);
}

Lookup::Grid Lookup::theta(double v) const {
    if (!std::isfinite(v) || v < th_min_ - kTol || v > th_max_ + kTol)
        throw std::out_of_range("theta outside LUT");
    const double u = (v - th_min_) * (n_th_ - 1) / (th_max_ - th_min_);
    const int i = std::clamp(static_cast<int>(std::floor(u)), 0, n_th_ - 2);
    const double a = th_min_ + i * (th_max_ - th_min_) / (n_th_ - 1);
    const double b = th_min_ + (i + 1) * (th_max_ - th_min_) / (n_th_ - 1);
    return {i, (v - a) / (b - a)};
}

Lookup::Grid Lookup::slide(double v) const {
    if (!std::isfinite(v) || v < s_min_ - kTol || v > s_max_ + kTol)
        throw std::out_of_range("s outside LUT");
    const double u = (v - s_min_) * (n_s_ - 1) / (s_max_ - s_min_);
    const int i = std::clamp(static_cast<int>(std::floor(u)), 0, n_s_ - 2);
    const double a = s_min_ + i * (s_max_ - s_min_) / (n_s_ - 1);
    const double b = s_min_ + (i + 1) * (s_max_ - s_min_) / (n_s_ - 1);
    return {i, (v - a) / (b - a)};
}

Lookup::Grid Lookup::phase(double v) const {
    if (!std::isfinite(v) || v < phi_.front() - kTol || v > phi_.back() + kTol)
        throw std::out_of_range("phase outside LUT");
    const auto it = std::upper_bound(phi_.begin(), phi_.end(), v);
    const int i = std::clamp(static_cast<int>(it - phi_.begin()) - 1, 0, n_p_ - 2);
    return {i, (v - phi_[i]) / (phi_[i + 1] - phi_[i])};
}

std::size_t Lookup::point(int s, int th, int p) const {
    return (static_cast<std::size_t>(s) * n_th_ + th) * n_p_ + p;
}

std::array<double, 4> Lookup::safe_interval(Vec2 x) const {
    const Grid th = theta(x[0]), s = slide(x[1]);
    std::array<double, 4> result{};
    for (int dt = 0; dt < 2; ++dt) for (int ds = 0; ds < 2; ++ds) {
        const double w = (dt ? th.weight : 1 - th.weight) * (ds ? s.weight : 1 - s.weight);
        const auto base = (static_cast<std::size_t>(s.index + ds) * n_th_ + th.index + dt) * 4;
        for (int k = 0; k < 4; ++k) result[k] += w * safe_[base + k];
    }
    return result;
}

Vec2 Lookup::initial_phase(Vec2 x) const {
    const auto b = safe_interval(x);
    return {std::clamp(0.0, b[0], b[1]), std::clamp(0.0, b[2], b[3])};
}

std::array<double, 7> Lookup::joints(const std::vector<float>& data, Grid th, Grid s, Grid p) const {
    std::array<double, 7> out{};
    for (int dt = 0; dt < 2; ++dt) for (int ds = 0; ds < 2; ++ds)
        for (int dp = 0; dp < 2; ++dp) {
            const double w = (dt ? th.weight : 1 - th.weight) *
                             (ds ? s.weight : 1 - s.weight) *
                             (dp ? p.weight : 1 - p.weight);
            const auto base = point(s.index + ds, th.index + dt, p.index + dp) * 7;
            for (int j = 0; j < 7; ++j) out[j] += w * data[base + j];
        }
    return out;
}

double Lookup::scalar(const std::vector<float>& data, Grid th, Grid s, Grid p) const {
    double out = 0;
    for (int dt = 0; dt < 2; ++dt) for (int ds = 0; ds < 2; ++ds)
        for (int dp = 0; dp < 2; ++dp) {
            const double w = (dt ? th.weight : 1 - th.weight) *
                             (ds ? s.weight : 1 - s.weight) *
                             (dp ? p.weight : 1 - p.weight);
            out += w * data[point(s.index + ds, th.index + dt, p.index + dp)];
        }
    return out;
}

LookupResult Lookup::query(Vec2 x, Vec2 phi) const {
    const Grid th = theta(x[0]), s = slide(x[1]);
    const Grid p_l = phase(phi[0]), p_r = phase(phi[1]);
    const auto safe = safe_interval(x);
    LookupResult out;
    const auto left = joints(q_l_, th, s, p_l);
    const auto right = joints(q_r_, th, s, p_r);
    for (int j = 0; j < 7; ++j) {
        out.q[j] = left[j]; out.q[j + 7] = right[j];
    }
    out.clearance = std::min(scalar(d_l_, th, s, p_l), scalar(d_r_, th, s, p_r));
    out.margin_minus = {phi[0] - safe[0], phi[1] - safe[2]};
    out.margin_plus = {safe[1] - phi[0], safe[3] - phi[1]};
    out.joint_margin = std::numeric_limits<double>::infinity();
    for (int j = 0; j < 14; ++j)
        out.joint_margin = std::min(out.joint_margin,
                                    std::min(out.q[j] - lower[j % 7], upper[j % 7] - out.q[j]));
    for (int dt = 0; dt < 2; ++dt) for (int ds = 0; ds < 2; ++ds)
        out.branch = std::max(out.branch, branch_[(s.index + ds) * n_th_ + th.index + dt]);
    return out;
}

PhaseEstimate Lookup::estimate_phase(Vec2 x, const Joints& measured_q,
                                     PhaseSearch search) const {
    for (double q : measured_q)
        if (!std::isfinite(q)) throw std::invalid_argument("Non-finite measured joint angle");
    if (search.max_phase_change <= 0 || search.continuity_weight < 0 ||
        !std::isfinite(search.continuity_weight))
        throw std::invalid_argument("Invalid phase-search limits");
    const Grid th = theta(x[0]), s = slide(x[1]);
    const auto safe = safe_interval(x);
    for (int dt = 0; dt < 2; ++dt) for (int ds = 0; ds < 2; ++ds)
        if (branch_[(s.index + ds) * n_th_ + th.index + dt] >= 2)
            throw std::out_of_range("Cannot estimate phase on excluded branch");
    PhaseEstimate result;
    for (int arm = 0; arm < 2; ++arm) {
        const auto& table = arm == 0 ? q_l_ : q_r_;
        double lo = safe[arm * 2], hi = safe[arm * 2 + 1];
        if (!std::isfinite(lo) || !std::isfinite(hi) || lo > hi ||
            lo < phi_.front() - kTol || hi > phi_.back() + kTol)
            throw std::out_of_range("Invalid safe phase interval");
        if (search.previous) {
            if (!std::isfinite((*search.previous)[arm]))
                throw std::invalid_argument("Non-finite previous phase");
            lo = std::max(lo, (*search.previous)[arm] - search.max_phase_change);
            hi = std::min(hi, (*search.previous)[arm] + search.max_phase_change);
            if (lo > hi) throw std::out_of_range("No phase in continuity window");
        }
        double best_score = std::numeric_limits<double>::infinity();
        double best_error = std::numeric_limits<double>::infinity();
        double best_max_error = std::numeric_limits<double>::infinity();
        double best_phi = lo;
        for (int i = 0; i < n_p_ - 1; ++i) {
            const double a = std::max(lo, static_cast<double>(phi_[i]));
            const double b = std::min(hi, static_cast<double>(phi_[i + 1]));
            if (a > b) continue;
            const double width = phi_[i + 1] - phi_[i];
            const auto qa = joints(table, th, s, {i, (a - phi_[i]) / width});
            const auto qb = joints(table, th, s, {i, (b - phi_[i]) / width});
            double numerator = 0, denominator = 0;
            for (int j = 0; j < 7; ++j) {
                const double delta = qb[j] - qa[j];
                numerator += (measured_q[arm * 7 + j] - qa[j]) * delta;
                denominator += delta * delta;
            }
            const double span = b - a;
            const double prior_weight = search.previous ? search.continuity_weight : 0;
            if (search.previous) {
                numerator += prior_weight * ((*search.previous)[arm] - a) * span;
                denominator += prior_weight * span * span;
            }
            const double t = denominator > 0 ? std::clamp(numerator / denominator, 0.0, 1.0) : 0.0;
            double error = 0, max_error = 0;
            for (int j = 0; j < 7; ++j) {
                const double residual = qa[j] + t * (qb[j] - qa[j]) - measured_q[arm * 7 + j];
                error += residual * residual;
                max_error = std::max(max_error, std::abs(residual));
            }
            const double projected_phi = a + t * span;
            const double score = error + (search.previous ?
                prior_weight * std::pow(projected_phi - (*search.previous)[arm], 2) : 0);
            if (score < best_score) {
                best_score = score;
                best_error = error;
                best_max_error = max_error;
                best_phi = projected_phi;
            }
        }
        if (!std::isfinite(best_error)) throw std::out_of_range("No safe phase segment");
        result.phi[arm] = best_phi;
        result.rms_joint_error[arm] = std::sqrt(best_error / 7.0);
        result.max_joint_error[arm] = best_max_error;
    }
    return result;
}

Controller::Controller(const Lookup& lookup, const Actor& actor) : lookup_(lookup), actor_(actor) {}

StepResult Controller::reset(Vec2 x, Vec2 xdot) {
    return reset(x, xdot, lookup_.initial_phase(x));
}

StepResult Controller::reset(Vec2 x, Vec2 xdot, Vec2 registered_phase) {
    StepResult out;
    ready_ = false;
    phi_ = registered_phase;
    const auto sample = lookup_.query(x, phi_);
    out.q = sample.q; out.phi = phi_; out.clearance = sample.clearance;
    if (sample.margin_minus[0] < -kTol || sample.margin_minus[1] < -kTol ||
        sample.margin_plus[0] < -kTol || sample.margin_plus[1] < -kTol)
        out.status = Status::unsafe_phase;
    else if (sample.branch >= 2) out.status = Status::branch;
    else if (sample.clearance < kClearance || !std::isfinite(sample.clearance)) out.status = Status::clearance;
    else if (!std::isfinite(sample.joint_margin) || sample.joint_margin < 0)
        out.status = Status::joint_limit;
    else out.status = Status::ok;
    ready_ = out.status == Status::ok;
    if (ready_) {
        x_ = x; xdot_ = xdot; prev_action_ = {};
        x_hist_.fill(x); xdot_hist_.fill(xdot); action_hist_.fill({});
        hist_count_ = 21;
    }
    return out;
}

std::array<float, 40> Controller::observation() const {
    if (!ready_) throw std::logic_error("t6 controller has not been reset");
    const auto sample = lookup_.query(x_, phi_);
    std::array<float, 40> o{};
    o[0] = x_[0]; o[1] = x_[1]; o[2] = xdot_[0]; o[3] = xdot_[1];
    o[4] = std::sin(phi_[0]); o[5] = std::cos(phi_[0]);
    o[6] = std::sin(phi_[1]); o[7] = std::cos(phi_[1]);
    o[8] = sample.margin_minus[0]; o[9] = sample.margin_minus[1];
    o[10] = sample.margin_plus[0]; o[11] = sample.margin_plus[1];
    o[12] = sample.clearance; o[13] = sample.joint_margin;
    o[14] = prev_action_[0]; o[15] = prev_action_[1];
    constexpr int lag[4] = {2, 5, 10, 20};
    for (int i = 0; i < 4; ++i) {
        const int h = 20 - lag[i];
        for (int k = 0; k < 2; ++k) {
            o[16 + 2 * i + k] = x_hist_[h][k];
            o[24 + 2 * i + k] = xdot_hist_[h][k];
            o[32 + 2 * i + k] = action_hist_[h][k];
        }
    }
    return o;
}

StepResult Controller::step(Vec2 x_next, Vec2 xdot_next) {
    if (!ready_) throw std::logic_error("t6 controller has not been reset");
    StepResult out;
    out.action = actor_.predict(observation());
    out.phi = {phi_[0] + out.action[0] * kPhaseRate * kDt,
               phi_[1] + out.action[1] * kPhaseRate * kDt};
    LookupResult next, current;
    try {
        next = lookup_.query(x_next, out.phi);
        current = lookup_.query(x_, phi_);
    } catch (const std::out_of_range&) {
        out.status = Status::grid_exit; return out;
    }
    out.q = next.q; out.clearance = next.clearance;
    for (int j = 0; j < 14; ++j)
        out.max_joint_speed = std::max(out.max_joint_speed,
                                       std::abs(next.q[j] - current.q[j]) / kDt);
    if (next.margin_minus[0] < -kTol || next.margin_minus[1] < -kTol ||
        next.margin_plus[0] < -kTol || next.margin_plus[1] < -kTol)
        out.status = Status::unsafe_phase;
    else if (next.branch >= 2) out.status = Status::branch;
    else if (!std::isfinite(next.clearance) || next.clearance < kClearance) out.status = Status::clearance;
    else if (!std::isfinite(next.joint_margin) || next.joint_margin < 0 ||
             !std::all_of(next.q.begin(), next.q.end(), [](double q) { return std::isfinite(q); }))
        out.status = Status::joint_limit;
    else if (out.max_joint_speed > kJointSpeed) out.status = Status::speed;
    else out.status = Status::ok;
    if (out.status == Status::ok) {
        out.has_command = true;
        x_ = x_next; xdot_ = xdot_next; phi_ = out.phi; prev_action_ = out.action;
        for (int i = 0; i < 20; ++i) {
            x_hist_[i] = x_hist_[i + 1];
            xdot_hist_[i] = xdot_hist_[i + 1];
            action_hist_[i] = action_hist_[i + 1];
        }
        x_hist_[20] = x_; xdot_hist_[20] = xdot_; action_hist_[20] = out.action;
    }
    return out;
}

} // namespace aviator::t6
