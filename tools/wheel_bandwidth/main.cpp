#include "WheelEstimate.hpp"
#include "Logger.hpp"
#include "RemoteLink.hpp"
#include "aviator/Aviator.hpp"
#include "aviator/backend.hpp"
#include <csignal>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <sstream>

using namespace aviator;
using namespace wheel_bandwidth;
namespace fs = std::filesystem;
static volatile std::sig_atomic_t interrupted = 0;
static void interrupt(int) { interrupted = 1; }
static double number(const std::string& value) {
    size_t used = 0; const auto n = std::stod(value, &used);
    if (used != value.size() || !std::isfinite(n)) throw std::runtime_error("Invalid number: " + value);
    return n;
}
static int cycles(const std::string& value) {
    const double n = number(value);
    if (n < 0 || n > 100 || std::floor(n) != n) throw std::runtime_error("Cycles must be integers 0..100");
    return int(n);
}
int main(int argc, char** argv) {
    try {
        auto config = defaultSystemConfig();
        fs::path output;
        Sweep sweep;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--help") {
                std::cout << "aviator_core_wheel_bandwidth --config SYSTEM --output DIRECTORY\n"
                    "  [--frequencies 0.1,0.2,0.3,0.5,0.7,1] [--amplitude-deg 10]\n"
                    "  [--settle-cycles 2] [--measure-cycles 5] [--ramp-cycles 1]\n"
                    "Requires grasp_wheel.source=wheel_initial; no camera. Use start_wheel_bandwidth.sh.\n";
                return 0;
            }
            if (i+1 == argc) throw std::runtime_error("Missing value: " + arg);
            const std::string value = argv[++i];
            if (arg == "--config") config = value;
            else if (arg == "--output") output = value;
            else if (arg == "--frequencies") {
                sweep.frequencies.clear(); std::istringstream stream(value); std::string token;
                while (std::getline(stream, token, ',')) sweep.frequencies.push_back(number(token));
                if (value.empty() || value.back() == ',') throw std::runtime_error("Empty frequency");
            } else if (arg == "--amplitude-deg") sweep.amplitude = number(value)*pi/180;
            else if (arg == "--settle-cycles") sweep.settle = cycles(value);
            else if (arg == "--measure-cycles") sweep.measure = cycles(value);
            else if (arg == "--ramp-cycles") sweep.ramp = cycles(value);
            else throw std::runtime_error("Unknown option: " + arg);
        }
        if (output.empty() || sweep.frequencies.empty() || sweep.amplitude <= 0 ||
            sweep.amplitude > 10*pi/180 || sweep.measure < 2 || sweep.ramp < 1)
            throw std::runtime_error("Require output, amplitude (0,10] deg, measure>=2 and ramp>=1");
        const auto settings = loadMotionConfig(config);
        const auto robot_yaml = YAML::LoadFile(settings.robot.string());
        if (robot_yaml["grasp_wheel"]["source"].as<std::string>("") != "wheel_initial")
            throw std::runtime_error("Demo requires its own wheel_initial grasp configuration");
        const double period = robot_yaml["servo_period"].as<double>(.01);
        if (!std::isfinite(period) || period < .001 || period > .05)
            throw std::runtime_error("Demo servo_period must be 0.001..0.05 s");
        double previous = 0;
        for (double f : sweep.frequencies) {
            if (f < .01 || f > 5 || f <= previous || f*std::max(.01, period) > .05)
                throw std::runtime_error("Frequencies must increase, be 0.01..5 Hz, with >=20 target samples/cycle");
            previous = f;
        }
        auto resolve = [&](const char* key) {
            fs::path p = robot_yaml[key].as<std::string>();
            return p.is_absolute() ? p : settings.robot.parent_path()/p;
        };
        const auto posture = YAML::LoadFile(resolve("posture").string());
        const auto limits = posture["joint2_limits_deg"].as<std::vector<double>>();
        if (limits.size() != 2) throw std::runtime_error("Invalid joint2 limits");
        auto fk = makePinIkKinematics(resolve("urdf").string(), limits[0]*pi/180, limits[1]*pi/180);
        WheelEstimate estimator(YAML::LoadFile(resolve("grasp").string()));
        const auto initial = loadInitialWheel(settings.robot);
        fs::create_directories(output);
        if (fs::exists(output/"samples.csv") || fs::exists(output/"commands.csv"))
            throw std::runtime_error("Output already contains a run");
        std::ofstream samples(output/"samples.csv"), commands(output/"commands.csv");
        samples.exceptions(std::ios::badbit | std::ios::failbit);
        commands.exceptions(std::ios::badbit | std::ios::failbit);
        samples << std::setprecision(16)
            << "sample_mono_us,left_sample_us,right_sample_us,t_s,segment,frequency_hz,segment_t_s,measure,ideal_rad,command_rad,reference_rad,left_rad,right_rad,estimated_rad,left_displacement_m,right_displacement_m,arm_difference_rad,left_transverse_m,right_transverse_m,left_tilt_rad,right_tilt_rad";
        for (int s=0; s<2; ++s) for (int j=1; j<=7; ++j) samples << ',' << (s ? "right_q" : "left_q") << j;
        samples << '\n';
        commands << std::setprecision(16) << "sample_mono_us,t_s,segment,frequency_hz,angle_rad,displacement_m\n";
        auto link = std::make_unique<RemoteLink>(settings);
        auto* connection = link.get();
        Aviator robot(std::move(link), nullptr, nullptr, settings.robot.string());
        robot.init();
        std::signal(SIGINT, interrupt); std::signal(SIGTERM, interrupt);
        auto supervised = [&](auto action, bool cancellable = true) {
            if (interrupted && cancellable) throw std::runtime_error("Interrupted");
            auto job = std::async(std::launch::async, action);
            while (job.wait_for(std::chrono::milliseconds(5)) != std::future_status::ready) {
                connection->heartbeat(); connection->report(robot.GetState(), "BANDWIDTH");
                if (interrupted && cancellable) robot.stop();
            }
            job.get();
            if (interrupted && cancellable) throw std::runtime_error("Interrupted");
        };
        int result = 0;
        try {
            supervised([&] { robot.enable(); });
            supervised([&] { robot.approachHandles(); });
            supervised([&] { robot.lockHandles(); });
            supervised([&] { robot.moveWheel(0, initial.displacement, .3); });
            zmq::context_t context(1);
            zmq::socket_t sub(context, zmq::socket_type::sub);
            configure(sub); subscribe(sub, "arm.state"); sub.connect(settings.subscribe);
            ReceiveState receiver;
            const uint64_t start = monotonic_us();
            uint64_t next = start, last_valid = start, last_sample[2]{}, last_seq = 0;
            std::string instance;
            std::vector<std::pair<uint64_t, double>> history;
            int last_segment = -2;
            while (!interrupted) {
                connection->heartbeat(); connection->report(robot.GetState(), "BANDWIDTH");
                const auto now = monotonic_us();
                const double t = double(now-start)/1e6;
                const auto status = robot.GetStatus();
                if (status.fault || !status.motion_error.empty() || robot.GetState() == "FAULT")
                    throw std::runtime_error("Control failure: " + status.motion_error);
                if (t >= sweep.duration()+2) break;
                if (now >= next) {
                    const auto point = sweep.at(t);
                    robot.servoWheel(point.angle, initial.displacement, 1.);
                    history.emplace_back(now, point.angle);
                    commands << now << ',' << t << ',' << point.segment << ',' << point.frequency << ','
                             << point.angle << ',' << initial.displacement << '\n';
                    next = now + uint64_t(period*1e6); // Never replay missed targets.
                    if (point.segment != last_segment) {
                        Logger::info("Bandwidth segment={} frequency={} Hz", point.segment, point.frequency);
                        samples.flush(); commands.flush(); last_segment = point.segment;
                    }
                }
                for (int batch = 0; batch < 64; ++batch) {
                    WireMessage wire; std::string error; Message m;
                    const auto received = receive(sub, receiver, wire, error);
                    if (received == ReceiveResult::empty) break;
                    if (received != ReceiveResult::received || !decode(wire.topic, wire.payload, m, error)) continue;
                    if (m.topic != Topic::arm_state || m.header.publisher_id != "manipulator" ||
                        m.header.clock_id != local_clock_id() || !m.header.valid ||
                        m.body.value("config_id", "") != settings.config_id) continue;
                    if (!instance.empty() && instance != m.header.session_id)
                        throw std::runtime_error("Device restarted during measurement");
                    if (m.header.sequence <= last_seq) continue;
                    const auto received_at = monotonic_us();
                    std::array<uint64_t, 2> stamp{};
                    std::array<std::array<double, 7>, 2> q{};
                    bool valid = true;
                    for (int s=0; s<2; ++s) {
                        const auto& a = m.body.at("arms").at(s ? "right" : "left");
                        if (!a.at("valid").get<bool>() || !a.at("sample_mono_us").is_number_unsigned()) { valid=false; break; }
                        stamp[s] = a.at("sample_mono_us").get<uint64_t>();
                        if (stamp[s] < start || stamp[s] <= last_sample[s] || stamp[s] > received_at ||
                            received_at-stamp[s] >= settings.timeout_us) { valid=false; break; }
                        q[s] = a.at("joint_position").get<std::array<double, 7>>();
                    }
                    if (!valid || std::max(stamp[0],stamp[1])-std::min(stamp[0],stamp[1]) > 5000) continue;
                    Estimate est[2];
                    for (int s=0; s<2; ++s) {
                        pinocchio::SE3 pose;
                        if (!fk->solveFk(static_cast<Side>(s), q[s], pose)) throw std::runtime_error("Feedback FK failed");
                        est[s] = estimator.fromFlange(s, pose);
                    }
                    const auto sample = stamp[0]/2+stamp[1]/2+(stamp[0]%2+stamp[1]%2)/2;
                    const auto at = sweep.at(double(sample-start)/1e6);
                    auto command = std::upper_bound(history.begin(), history.end(), sample,
                        [](uint64_t ts, const auto& entry) { return ts < entry.first; });
                    if (command == history.begin()) continue;
                    --command;
                    samples << sample << ',' << stamp[0] << ',' << stamp[1] << ',' << double(sample-start)/1e6
                        << ',' << at.segment << ',' << at.frequency << ',' << at.local_time << ',' << at.measure
                        << ',' << at.angle << ',' << command->second << ',' << m.body.at("wheel_reference").at("angle").get<double>()
                        << ',' << est[0].angle << ',' << est[1].angle << ',' << meanAngle(est[0].angle,est[1].angle)
                        << ',' << est[0].displacement << ',' << est[1].displacement
                        << ',' << difference(est[0].angle,est[1].angle)
                        << ',' << est[0].transverse_error << ',' << est[1].transverse_error
                        << ',' << est[0].tilt_error << ',' << est[1].tilt_error;
                    for (const auto& arm : q) for (double angle : arm) samples << ',' << angle;
                    samples << '\n';
                    last_valid = received_at; last_seq = m.header.sequence; instance = m.header.session_id;
                    last_sample[0] = stamp[0]; last_sample[1] = stamp[1];
                }
                if (monotonic_us()-last_valid > 500000)
                    throw std::runtime_error("No fresh synchronized joint measurements for 500 ms");
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            if (interrupted) result = 130;
        } catch (const std::exception& e) { Logger::error("Bandwidth test: {}", e.what()); result = interrupted ? 130 : 1; }
        // Keep task heartbeat alive through braking and hand opening, including failed startup.
        robot.stop();
        while (robot.GetState() == "SERVO") {
            connection->heartbeat(); std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (result == 0) {
            try { supervised([&] { robot.moveWheel(0, initial.displacement, .3); }); }
            catch (const std::exception& e) { Logger::error("Return to center failed: {}", e.what()); result = 1; }
        }
        try { supervised([&] { if (robot.GetState() == "LOCKED") robot.unlockHandles(); }, false); }
        catch (const std::exception& e) { Logger::error("Unlock failed: {}", e.what()); result = 1; }
        try { supervised([&] { robot.disable(); }, false); }
        catch (const std::exception& e) { Logger::error("Disable failed: {}", e.what()); result = 1; }
        samples.flush(); commands.flush();
        std::ofstream(output/"run.json") << Json{{"completed", result == 0}, {"exit_code", result},
            {"frequencies_hz", sweep.frequencies}, {"amplitude_deg", sweep.amplitude*180/pi},
            {"settle_cycles", sweep.settle}, {"measure_cycles", sweep.measure}, {"ramp_cycles", sweep.ramp},
            {"measurement", "joint_fk_estimate"}, {"displacement_m", initial.displacement}}.dump(2);
        return result;
    } catch (const std::exception& e) { Logger::error("Error: {}", e.what()); return 1; }
}
