#include "startup.hpp"
#include "simulation.hpp"
#include "camera.hpp"
#include "transport.hpp"
#include "Viewer.hpp"
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <thread>

namespace {
volatile std::sig_atomic_t stopping=0;
void stop(int) { stopping=1; }
void usage() {
    std::cout << "Usage: simulation [--model path] [--headless] [--no-camera] [--duration seconds]\n"
        "  [--pub-endpoint endpoint] [--sub-endpoint endpoint]\n"
        "  [--control-epoch UUID] (legacy --core-session/--origin-session are ignored)\n"
        "  [--core-publisher id] [--origin-publisher id] [--camera-timeout-ms ms]\n"
        "Defaults: bus 5555/5556; no command authorization; EGL camera 640x480.\n"
        "--headless disables the viewer; --no-camera disables EGL and reports OFFLINE.\n";
}
std::string default_model() {
    std::error_code error;
    auto executable=std::filesystem::read_symlink("/proc/self/exe",error);
    if (!error) {
        auto installed=executable.parent_path()/"../share/aviator/models/mjcf/aviator.xml";
        if (std::filesystem::exists(installed)) return installed.string();
    }
    return AVIATOR_SIMULATION_MODEL;
}
}
int main(int argc,char** argv) {
    try {
        simulation::Authorization auth;
        std::string model=default_model(), pub_endpoint=aviator::publish_endpoint, sub_endpoint=aviator::subscribe_endpoint;
        bool headless=false, no_camera=false; double duration=0;
        for (int i=1;i<argc;++i) {
            std::string arg=argv[i];
            if (arg=="--help" || arg=="-h") { usage(); return 0; }
            if (arg=="--headless") { headless=true; continue; }
            if (arg=="--no-camera") { no_camera=true; continue; }
            if (++i==argc) throw std::runtime_error("missing value for "+arg);
            std::string value=argv[i];
            if (arg=="--model") model=value;
            else if(arg=="--pub-endpoint") pub_endpoint=value;
            else if(arg=="--sub-endpoint") sub_endpoint=value;
            else if(arg=="--core-session") auth.session=value;
            else if(arg=="--control-epoch") auth.epoch=value;
            else if(arg=="--origin-session") auth.origin_session=value;
            else if(arg=="--core-publisher") auth.publisher=value;
            else if(arg=="--origin-publisher") auth.origin_publisher=value;
            else if(arg=="--duration" || arg=="--camera-timeout-ms") {
                std::size_t used=0; double v=std::stod(value,&used);
                if(used!=value.size() || !std::isfinite(v) || v<=0 || v>86400) throw std::runtime_error("invalid "+arg);
                if(arg=="--duration") duration=v;
                else { auth.camera_timeout_us=static_cast<std::uint64_t>(v*1000); if(!auth.camera_timeout_us) throw std::runtime_error("camera timeout too small"); }
            } else throw std::runtime_error("unknown argument: "+arg);
        }
        if (auth.epoch.empty() && (!auth.session.empty() || !auth.origin_session.empty())) throw std::runtime_error("authorization requires --control-epoch");
        if (pub_endpoint==sub_endpoint) throw std::runtime_error("PUB and SUB endpoints must differ");
        std::signal(SIGINT,stop); std::signal(SIGTERM,stop);
        simulation::Simulation sim(model,auth);
        std::unique_ptr<simulation::Camera> camera;
        if(!no_camera) camera=std::make_unique<simulation::Camera>(sim);
        std::unique_ptr<aviator::Viewer> viewer;
        if(!headless) {
            viewer=std::make_unique<aviator::Viewer>(sim.model());
            glfwSwapInterval(0);
        }
        std::mutex viewer_mutex;
        zmq::context_t context(1); zmq::socket_t pub(context,zmq::socket_type::pub),sub(context,zmq::socket_type::sub);
        aviator::configure(pub); aviator::configure(sub);
        for(auto topic : {"arm.command","hand.command","camera.command"}) aviator::subscribe(sub,topic);
        pub.connect(pub_endpoint); sub.connect(sub_endpoint);
        aviator::ReceiveState receive_state;
        auto publish=[&](const aviator::Message& message) {
            std::string payload,error;
            if(!aviator::encode(message,payload,error)) throw std::runtime_error("encode: "+error);
            aviator::send(pub,aviator::topic_name(message.topic),payload);
        };
        const auto start=aviator::monotonic_us(); auto next_step=start, next_state=start, next_camera=start, next_view=start;
        std::uint64_t rejected=0,last_report=0;
        const auto timestep=static_cast<std::uint64_t>(sim.model()->opt.timestep*1000000);
        if(timestep==0) throw std::runtime_error("timestep below scheduling precision");
        std::cout << "READY session=" << sim.session() << " clock_id=" << sim.clock() << " model=" << model << std::endl;
        aviator::print_startup("simulation", {
            {"PUB connect", pub_endpoint},
            {"PUB topics", "arm.state (100 Hz), hand.state (100 Hz)"},
            {"", "camera.detection (30 Hz; target rates)"},
            {"SUB connect", sub_endpoint},
            {"SUB topics", "arm.command, hand.command, camera.command"},
            {"Viewer", headless ? "HEADLESS" : "GLFW window"},
            {"Camera", no_camera ? "OFFLINE (--no-camera); detection still published" : "EGL 640x480"},
            {"Authorization", auth.epoch.empty() ? "UNCONFIGURED (state publishing remains enabled)" : "Configured publisher/control epoch"},
            {"Session", sim.session()},
            {"Model", model},
            {"Transport", "Async connect; bus connectivity is not yet confirmed."},
            {"Exit", "Ctrl+C"}
        });
        while(!stopping) {
            auto now=aviator::monotonic_us();
            if(duration>0 && (now-start)/1e6>=duration) break;
            // Bounded draining prevents a busy publisher from starving physics.
            for(int budget=0;budget<64;++budget) {
                aviator::WireMessage wire; aviator::Message message; std::string error;
                auto result=aviator::receive(sub,receive_state,wire,error);
                if(result==aviator::ReceiveResult::empty) break;
                if(result==aviator::ReceiveResult::rejected || !aviator::decode(wire.topic,wire.payload,message,error) ||
                   !sim.command(message,aviator::monotonic_us(),error)) {
                    ++rejected;
                    if(now-last_report>=1000000) { std::cerr << "rejected=" << rejected << " reason=" << error << '\n'; last_report=now; }
                }
            }
            now=aviator::monotonic_us();
            if(now>=next_step) {
                sim.step(now); next_step+=timestep;
                // Drop wall-time debt after overload; never forge virtual time as CLOCK_MONOTONIC.
                if(now>next_step && now-next_step>100000) next_step=now+timestep;
                auto sampled=aviator::monotonic_us();
                if(sampled>=next_state) { publish(sim.state(false,sampled)); publish(sim.state(true,sampled)); next_state=sampled+10000; }
            }
            now=aviator::monotonic_us();
            if(now>=next_camera) {
                if(viewer) glfwMakeContextCurrent(nullptr);
                bool in_roi=camera && camera->capture(sim);
                publish(sim.detection(now,bool(camera),in_roi)); next_camera=aviator::monotonic_us()+33333;
            }
            if(viewer && now>=next_view) {
                glfwMakeContextCurrent(viewer->window());
                if(!viewer->draw(sim.data(),viewer_mutex)) break;
                next_view=aviator::monotonic_us()+16667;
            }
            auto remaining=static_cast<std::int64_t>(next_step)-static_cast<std::int64_t>(aviator::monotonic_us());
            if(remaining>0) std::this_thread::sleep_for(std::chrono::microseconds(remaining));
        }
        std::cout << "STOPPED rejected=" << rejected << " sim_time=" << sim.data()->time << '\n';
        return 0;
    } catch(const zmq::error_t& e) {
        if(stopping && e.num()==EINTR) return 0;
        std::cerr << "simulation: " << e.what() << '\n'; return 1;
    } catch(const std::exception& e) { std::cerr << "simulation: " << e.what() << '\n'; return 1; }
}
