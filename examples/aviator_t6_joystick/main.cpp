// Milestone 2 + 3: passive-wheel + grasp-weld + T6 arm-servo closed loop, run
// with the CURRENT (18 mm-misaligned) LUT on purpose — the user asked to keep
// the mismatch and observe robustness ("留着继续观察"), then wire in the joystick.
//
// Flow (per the corrected design):
//   load model + aviator_home -> close fingers to grip pose
//   -> servo arms to LUT(0,0).q   (grasp the wheel at its keyframe pose)
//   -> FORCE weld (left/right_grasp) regardless of alignment
//   -> servo arms to LUT(0,-0.08).q (pull wheel to the t6 start state)
//   -> T6 closed loop, task reference from the joystick (or a scripted sine).
//
// No per-cycle IK: the T6 method (LUT + ONNX actor) is RL-guaranteed.
//
// Threading (matches examples/AviatorRobot_simple/src/main.cpp): the GLFW viewer
// and its event loop stay in the MAIN thread; the blocking stages + 100 Hz T6
// control loop run in a worker thread. The physics loop is already its own
// thread inside Sim. Rendering holds the physics mutex only for the scene
// snapshot, so the 1 ms physics step is never blocked by GPU/vsync work.
//
// Joystick mapping (examples/t6/README.md): axis0 -32767/0/+32767 -> theta
// -50/0/+50 deg (-0.87266/0/+0.87266 rad); axis1 -> s -159/-80/-1 mm; 5% deadzone
// around zero with renormalization, 20 ms EMA, task slope limit (theta 1.5 rad/s,
// s 0.04 m/s). Reference updates every 20 ms; T6 runs every 10 ms.

#include "sim_backend.hpp"
#include "t6.hpp"
#include "joystick.hpp"
#include "Viewer.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace aviator;
using namespace aviator::t6;

static void sleepSimSeconds(Sim& sim, double seconds) {
    const double t0 = sim.time();
    while (sim.time() - t0 < seconds) std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

static Vec14 toVec14(const Joints& j) {
    Vec14 q{};
    for (int i = 0; i < 14; ++i) q[i] = j[i];
    return q;
}

// Absolute task reference from the joystick: stick position maps to a target
// (theta, s); deadzone + renormalize + EMA + slope limit keep it smooth.
struct JoystickRef {
    double theta = 0.0;    // current reference (rad)
    double s = -0.08;      // current reference (m)
    bool reverse = false;  // flip axis0 sign (per-wheel calibration)
    double theta_rate = 1.5;  // rad/s
    double s_rate = 0.04;     // m/s

    // Map raw axes and advance the reference by dt seconds.
    void update(int axis0_raw, int axis1_raw, double dt) {
        double n0 = deadzoneRenorm(axis0_raw);
        double n1 = deadzoneRenorm(axis1_raw);
        if (reverse) n0 = -n0;
        double th_t = n0 * 0.87266;
        double s_t = -0.08 + n1 * 0.079;

        // 20 ms EMA (time constant tau=0.02).
        const double alpha = dt / (dt + 0.02);
        th_t = alpha * th_t + (1.0 - alpha) * theta;
        s_t = alpha * s_t + (1.0 - alpha) * s;

        // Task slope limit.
        const double dth = std::clamp(th_t - theta, -theta_rate * dt, theta_rate * dt);
        const double ds = std::clamp(s_t - s, -s_rate * dt, s_rate * dt);
        theta += dth;
        s += ds;
    }

    static double deadzoneRenorm(int raw) {
        constexpr int dz = 1638;  // 5% of 32767
        if (std::abs(raw) <= dz) return 0.0;
        return raw > 0 ? (raw - dz) / double(32767 - dz) : (raw + dz) / double(32767 - dz);
    }
};

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);  // flush stage prints immediately

    // Filter out --headless, keep the rest positional:
    //   model lut_dir onnx duration csv [device_path] [axis_reverse]
    //   [theta_rate] [s_rate]
    std::vector<std::string> pos;
    bool headless = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--headless") headless = true;
        else pos.push_back(a);
    }
    auto arg = [&](size_t i, const std::string& def) -> std::string {
        return i < pos.size() ? pos[i] : def;
    };
    const std::string model = arg(0, "models/mjcf/aviator.xml");
    const std::string lut_dir = arg(1, "hil-serl/data/aviator/manifold_phi");
    const std::string onnx = arg(2, "examples/t6/actor_160k.onnx");
    const double duration = pos.size() > 3 ? std::atof(pos[3].c_str()) : 30.0;
    const std::string csv = arg(4, "t6_joystick_robustness.csv");
    const std::string device_path = arg(5, "");
    JoystickRef joy;
    if (pos.size() > 6) joy.reverse = std::atoi(pos[6].c_str()) != 0;
    if (pos.size() > 7) joy.theta_rate = std::atof(pos[7].c_str());
    if (pos.size() > 8) joy.s_rate = std::atof(pos[8].c_str());

    // Joystick is optional: try to connect, fall back to a scripted sine.
    joystick::Device* js = nullptr;
    bool use_joystick = false;
    if (!device_path.empty()) {
        js = new joystick::Device(device_path);
        const auto& st = js->poll();
        use_joystick = st.connected && !st.axes.empty();
        std::printf("=== joystick %s: %s\n", device_path.c_str(),
                    use_joystick ? "connected" : "NOT connected, falling back to scripted");
    }

    // Grip pose (12 independent hand joints): close fingers onto the handle.
    Vec12 grip{};
    for (int s = 0; s < 2; ++s) {
        grip[s * 6 + 0] = 1.2;   // index_1
        grip[s * 6 + 1] = 1.2;   // middle_1
        grip[s * 6 + 2] = 1.2;   // ring_1
        grip[s * 6 + 3] = 1.2;   // little_1
        grip[s * 6 + 4] = 0.0;   // thumb_1
        grip[s * 6 + 5] = 0.55;  // thumb_2
    }

    Sim sim(model, grip, /*realtime=*/true);
    Lookup lut(lut_dir);

    // Viewer owns the main thread (GLFW/X11); blocking control runs in a worker.
    std::unique_ptr<aviator::Viewer> viewer;
    if (!headless) {
        try {
            viewer = std::make_unique<aviator::Viewer>(sim.model());
        } catch (const std::exception& e) {
            std::printf("=== viewer unavailable (%s), running headless\n", e.what());
            viewer.reset();
        }
    }
    std::atomic<bool> done{false};   // set by control thread on completion
    std::atomic<bool> quit{false};   // set by main thread when the window closes

    std::thread control([&]() {
        // ---- Stage 1: grasp the wheel at its keyframe pose (theta=0, s=0) ----
        const Vec2 x_start{0.0, 0.0};
        const LookupResult ls = lut.query(x_start, lut.initial_phase(x_start));
        sim.setArmTarget(toVec14(ls.q));
        sleepSimSeconds(sim, 1.5);

        SimState pre = sim.state();
        std::printf("=== pre-weld  | perr=(%.5f,%.5f) m  rerr=(%.5f,%.5f) rad  aligned=%d\n",
                    pre.position_error[0], pre.position_error[1],
                    pre.rotation_error[0], pre.rotation_error[1], (int)pre.aligned);

        // ---- Stage 2: force weld (robustness probe: ignore the 18 mm gap) ----
        sim.lock();
        sleepSimSeconds(sim, 1.0);

        SimState post = sim.state();
        std::printf("=== post-weld | perr=(%.5f,%.5f) m  rerr=(%.5f,%.5f) rad  fault=%d\n",
                    post.position_error[0], post.position_error[1],
                    post.rotation_error[0], post.rotation_error[1], (int)post.fault);

        // ---- Stage 3: pull wheel to the t6 start state (theta=0, s=-0.08) ----
        const Vec2 x0{0.0, -0.08};
        const LookupResult l0 = lut.query(x0, lut.initial_phase(x0));
        sim.setArmTarget(toVec14(l0.q));
        sleepSimSeconds(sim, 1.5);

        SimState at_start = sim.state();
        std::printf("=== at start  | wheel=(%.4f,%.4f) m  perr=(%.5f,%.5f) m  fault=%d\n",
                    at_start.theta, at_start.s, at_start.position_error[0],
                    at_start.position_error[1], (int)at_start.fault);

        // ---- Stage 4: T6 closed loop ----
        FeedbackLimits limits;
        limits.max_rms_joint_error = 0.02;
        limits.max_joint_error = 0.05;
        limits.max_phase_change = 0.025;
        limits.continuity_weight = 0.001;
        limits.max_joint_speed = 1.5;
        limits.min_clearance = 0.005;
        limits.time_tolerance = 0.002;
        T6 t6(onnx, lut_dir, limits);

        std::ofstream out(csv);
        out << "t,th_ref,s_ref,th,s,th_vel,s_vel,perr_L,perr_R,rerr_L,rerr_R,"
               "status,has_cmd,phi0,phi1,clearance,fault,aligned,locked,intervened,"
               "rms0,rms1,max0,max1,axis0,axis1\n";

        const double t0 = sim.time();   // T6-loop start time (reference relative to it)
        const double t_end = t0 + duration;
        double next_tick = t0 + 0.010;
        double last_joy_t = -1.0;
        int axis0 = 0, axis1 = 0;
        while (!quit.load() && sim.time() < t_end) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            const SimState st = sim.state();
            if (st.time + 1e-9 < next_tick) continue;

            // Task reference: joystick (20 ms update) or scripted sine (fallback).
            double th_ref, s_ref;
            if (use_joystick) {
                const auto& jst = js->poll();
                if (!jst.connected || jst.axes.empty()) use_joystick = false;
                if (jst.axes.size() > 0) axis0 = jst.axes[0];
                if (jst.axes.size() > 1) axis1 = jst.axes[1];
                if (last_joy_t < 0 || next_tick - last_joy_t >= 0.020 - 1e-9) {
                    joy.update(axis0, axis1, 0.020);
                    last_joy_t = next_tick;
                }
                th_ref = joy.theta;
                s_ref = joy.s;
            } else {
                const double tr = next_tick - t0;
                th_ref = 0.3 * std::sin(2.0 * M_PI * tr / 20.0);
                s_ref = -0.08 - 0.04 * std::sin(2.0 * M_PI * tr / 20.0);
            }

            // Drive T6 on the exact 10 ms grid (next_tick) so its timing gate
            // (|dt-0.010|<=tol) never trips on wall-clock poll jitter; state is read
            // at st.time which lags next_tick by <1 ms.
            Feedback fb{{st.theta, st.s}, {st.theta_vel, st.s_vel}, st.q, next_tick};
            StepResult r = t6(fb, {th_ref, s_ref});
            if (r.has_command) sim.setArmTarget(toVec14(r.q));
            next_tick += 0.010;

            out << next_tick << "," << th_ref << "," << s_ref << "," << st.theta << "," << st.s
                << "," << st.theta_vel << "," << st.s_vel << "," << st.position_error[0] << ","
                << st.position_error[1] << "," << st.rotation_error[0] << "," << st.rotation_error[1]
                << "," << (int)r.status << "," << (int)r.has_command << "," << r.phi[0] << ","
                << r.phi[1] << "," << r.clearance << "," << (int)st.fault << ","
                << (int)st.aligned << "," << (int)st.locked << "," << (int)r.intervened << ","
                << r.estimate.rms_joint_error[0] << "," << r.estimate.rms_joint_error[1] << ","
                << r.estimate.max_joint_error[0] << "," << r.estimate.max_joint_error[1] << ","
                << axis0 << "," << axis1 << "\n";
        }

        out.close();
        std::printf("=== done, CSV -> %s, sim time %.3f s\n", csv.c_str(), sim.time());
        done.store(true);
    });

    // Main thread: render loop (or just wait when headless).
    if (viewer) {
        while (!done.load()) {
            if (!viewer->draw(sim.data(), sim.mutex())) {
                quit.store(true);   // window closed -> ask control to stop early
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    control.join();
    delete js;
    return 0;
}
