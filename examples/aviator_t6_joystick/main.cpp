// Milestone 2 + 3: passive-wheel + grasp-weld + T6 arm-servo closed loop, run
// with the calibrated TCP transform and an alignment gate before welding.
//
// Flow (per the corrected design):
//   load model + aviator_home -> close fingers to grip pose
//   -> servo arms to LUT(0,0).q   (grasp the wheel at its keyframe pose)
//   -> weld only after both TCPs are aligned with the handles
//   -> servo arms to LUT(0,-0.08).q (pull wheel to the t6 start state)
//   -> T6 closed loop, task reference from the joystick (or a scripted sine).
//
// No per-cycle IK: T6 checks LUT/actor commands, while physical tracking and
// contact still require runtime monitoring.
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
#include <array>
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

struct ContactQuality {
    int thumb = 0;
    int fingers = 0;
    double least_normal_dot = 1.0;
    double deepest_penetration = 0.0;
    double thumb_distance = 0.0;
    double index_distance = 0.0;
    double middle_distance = 0.0;
    double palm_distance = 0.0;
    bool thumb_above = false;
};

// A weld can align perfectly while every finger touches the same face of a
// handle. Check the actual left/right handle collision mesh before calling a
// pose a physical grasp.
static ContactQuality handleContactQuality(Sim& sim, const char* side, int handle_mesh) {
    std::lock_guard<std::mutex> lk(sim.mutex());
    mjModel* m = sim.model();
    mjData* d = sim.data();
    const int handle_geom = mj_name2id(m, mjOBJ_GEOM,
                                      ("steering_wheel_collision_" + std::to_string(handle_mesh)).c_str());
    std::vector<std::array<double, 3>> thumb_normals, finger_normals;
    ContactQuality result;
    for (int i = 0; i < d->ncon; ++i) {
        const mjContact& c = d->contact[i];
        if (c.dist >= 0 || (c.geom1 != handle_geom && c.geom2 != handle_geom)) continue;
        const int hand_geom = c.geom1 == handle_geom ? c.geom2 : c.geom1;
        const char* body_name = mj_id2name(m, mjOBJ_BODY, m->geom_bodyid[hand_geom]);
        if (!body_name) continue;
        const std::string body(body_name);
        if (body.compare(0, std::char_traits<char>::length(side), side) != 0) continue;
        const double sign = c.geom1 == handle_geom ? 1.0 : -1.0;
        std::array<double, 3> normal{sign * c.frame[0], sign * c.frame[1], sign * c.frame[2]};
        if (body.find("thumb") != std::string::npos) thumb_normals.push_back(normal);
        else if (body.find("index") != std::string::npos ||
                 body.find("middle") != std::string::npos ||
                 body.find("ring") != std::string::npos ||
                 body.find("little") != std::string::npos) finger_normals.push_back(normal);
        result.deepest_penetration = std::max(result.deepest_penetration, -double(c.dist));
    }
    result.thumb = static_cast<int>(thumb_normals.size());
    result.fingers = static_cast<int>(finger_normals.size());
    mjtNum fromto[6];
    auto distance = [&](const char* suffix) {
        const int geom = mj_name2id(m, mjOBJ_GEOM,
                                    (std::string(side) + suffix + "_collision_0").c_str());
        return mj_geomDistance(m, d, handle_geom, geom, 1.0, fromto);
    };
    result.thumb_distance = distance("thumb_4");
    result.index_distance = distance("index_2");
    result.middle_distance = distance("middle_2");
    const int palm = mj_name2id(m, mjOBJ_GEOM,
                                side[0] == 'l' ? "l_base_link_collision_0"
                                               : "r_base_link_collision_0");
    result.palm_distance = mj_geomDistance(m, d, handle_geom, palm, 1.0, fromto);
    const int thumb_body = mj_name2id(m, mjOBJ_BODY, (std::string(side) + "thumb_4").c_str());
    const int index_body = mj_name2id(m, mjOBJ_BODY, (std::string(side) + "index_2").c_str());
    result.thumb_above = d->xpos[3 * thumb_body + 2] > d->xpos[3 * index_body + 2];
    for (const auto& a : thumb_normals)
        for (const auto& b : finger_normals)
            result.least_normal_dot = std::min(result.least_normal_dot,
                                               a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
    return result;
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
    // The fixed hand mount uses the original TCP and weld sites. Keep the
    // LUT/T6 path as the default; the optional physical-grasp probe requires
    // additional sites that may not be present in the selected MJCF.
    bool grasp_only = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--headless") headless = true;
        else if (a == "--grasp-only") grasp_only = true;
        else if (a == "--legacy-t6") grasp_only = false;
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
    if (grasp_only && !device_path.empty())
        std::printf("=== joystick ignored in static grasp test mode\n");
    JoystickRef joy;
    if (pos.size() > 6) joy.reverse = std::atoi(pos[6].c_str()) != 0;
    if (pos.size() > 7) joy.theta_rate = std::atof(pos[7].c_str());
    if (pos.size() > 8) joy.s_rate = std::atof(pos[8].c_str());

    // Joystick is optional: try to connect, fall back to a scripted sine.
    joystick::Device* js = nullptr;
    bool use_joystick = false;
    if (!grasp_only && !device_path.empty()) {
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
    if (grasp_only) {
        grip = {0.52, 0.57, 0.50, 0.03991098, 0.77604755, 0.10718285,
                0.52599652, 0.55878870, 0.50, 0.04051597, 0.77281815, 0.10931270};
    }

    Lookup lut(lut_dir);
    const Vec2 x_start{0.0, 0.0};
    const LookupResult ls = lut.query(x_start, lut.initial_phase(x_start));
    // Opposed thumb/index/middle pinch for the current 10-degree base and
    // 18.26-degree additional flange-Z hand mount. This pose is outside the
    // old LUT, so this mode only tests a stationary physical grasp and weld.
    const Vec14 grasp_q{
         0.33500136, 1.62446586, -1.59836666, 2.01742108, -0.07085989, -0.10542336, 0.08459278,
        -0.31181121, 1.86843677,  1.79887493, 1.98169084, -0.09492177, -0.05794433, 0.19528163};
    const Vec14 initial_q = grasp_only ? grasp_q : toVec14(ls.q);
    std::printf("=== mode: %s\n", grasp_only
                    ? "physical pinch and weld (T6/joystick disabled)"
                    : "LUT/T6 joystick path");
    Sim sim(model, grip, initial_q, /*realtime=*/true, grasp_only);

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
    std::atomic<int> exit_code{0};

    std::thread control([&]() {
        if (grasp_only) {
            // The candidate is initialized exactly at the stationary handle
            // pose and welded before the passive slide can fall under gravity.
            sleepSimSeconds(sim, 1.0);
            SimState st = sim.state();
            std::printf("=== candidate weld | perr=(%.5f,%.5f) m rerr=(%.5f,%.5f) rad fault=%d\n",
                        st.position_error[0], st.position_error[1],
                        st.rotation_error[0], st.rotation_error[1], (int)st.fault);
            if (st.fault || st.position_error[0] >= 0.003 || st.position_error[1] >= 0.003 ||
                st.rotation_error[0] >= 0.035 || st.rotation_error[1] >= 0.035) {
                exit_code.store(2);
                done.store(true);
                return;
            }
            sleepSimSeconds(sim, duration);
            st = sim.state();
            const ContactQuality left = handleContactQuality(sim, "left_", 31);
            const ContactQuality right = handleContactQuality(sim, "right_", 33);
            std::printf("=== held | wheel=(%.5f,%.5f) perr=(%.5f,%.5f) m fault=%d\n",
                        st.theta, st.s, st.position_error[0], st.position_error[1], (int)st.fault);
            std::printf("=== handle contacts | L thumb=%d fingers=%d normal_dot=%.2f depth=%.1f mm"
                        " | R thumb=%d fingers=%d normal_dot=%.2f depth=%.1f mm\n",
                        left.thumb, left.fingers, left.least_normal_dot,
                        1000 * left.deepest_penetration, right.thumb, right.fingers,
                        right.least_normal_dot, 1000 * right.deepest_penetration);
            std::printf("=== fingertip distances mm | L thumb=%.2f index=%.2f middle=%.2f palm=%.1f"
                        " | R thumb=%.2f index=%.2f middle=%.2f palm=%.1f\n",
                        1000 * left.thumb_distance, 1000 * left.index_distance,
                        1000 * left.middle_distance, 1000 * left.palm_distance,
                        1000 * right.thumb_distance, 1000 * right.index_distance,
                        1000 * right.middle_distance, 1000 * right.palm_distance);
            auto pinching = [](const ContactQuality& q) {
                return q.thumb > 0 && q.fingers > 0 && q.least_normal_dot < -0.5 &&
                       q.thumb_distance < 0.0002 && q.index_distance < 0.0002 &&
                       q.middle_distance < 0.0002 && q.palm_distance > 0.005 &&
                       q.deepest_penetration < 0.005 && q.thumb_above;
            };
            if (!pinching(left) || !pinching(right)) {
                std::fprintf(stderr, "physical pinch check failed despite weld alignment\n");
                exit_code.store(3);
            } else {
                std::printf("=== physical pinch verified on both handles\n");
            }
            if (st.fault) exit_code.store(2);
            done.store(true);
            return;
        }
        // ---- Stage 1: grasp the wheel at its keyframe pose (theta=0, s=0) ----
        sim.setArmTarget(initial_q);
        sleepSimSeconds(sim, 1.5);

        SimState pre = sim.state();
        double pre_joint_error = 0.0;
        for (int i = 0; i < 14; ++i)
            pre_joint_error = std::max(pre_joint_error, std::abs(pre.q[i] - initial_q[i]));
        std::printf("=== pre-weld  | perr=(%.5f,%.5f) m  rerr=(%.5f,%.5f) rad  aligned=%d\n",
                    pre.position_error[0], pre.position_error[1],
                    pre.rotation_error[0], pre.rotation_error[1], (int)pre.aligned);
        std::printf("=== pre-weld  | max arm joint target error=%.4f rad, wheel=(%.4f,%.4f)\n",
                    pre_joint_error, pre.theta, pre.s);

        // ---- Stage 2: weld only after the grasp has settled and aligned ----
        if (!pre.aligned || pre.fault) {
            std::fprintf(stderr, "grasp alignment failed before weld; keeping handles unlocked\n");
            exit_code.store(2);
            done.store(true);
            return;
        }
        sim.lock();
        sleepSimSeconds(sim, 1.0);

        SimState post = sim.state();
        std::printf("=== post-weld | perr=(%.5f,%.5f) m  rerr=(%.5f,%.5f) rad  fault=%d\n",
                    post.position_error[0], post.position_error[1],
                    post.rotation_error[0], post.rotation_error[1], (int)post.fault);
        if (post.fault || post.position_error[0] >= 0.003 || post.position_error[1] >= 0.003 ||
            post.rotation_error[0] >= 0.035 || post.rotation_error[1] >= 0.035) {
            sim.unlock();
            std::fprintf(stderr, "grasp became misaligned after weld; stopping before T6\n");
            exit_code.store(2);
            done.store(true);
            return;
        }

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
    return exit_code.load();
}
