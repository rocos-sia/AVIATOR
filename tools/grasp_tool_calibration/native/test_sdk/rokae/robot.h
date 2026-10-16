#pragma once
#include <array>
#include <memory>
#include <string>
#include <system_error>
#include <vector>
#include <stdexcept>
namespace fake_sdk {
inline std::vector<std::string> calls;
inline bool fail_load = false, fail_idle = false;
inline std::array<double,7> load{};
}
namespace rokae {
enum class OperationState { idle, jog, rtControlling, drag, rlProgram, demo, dynamicIdentify,
    frictionIdentify, loadIdentify, moving, jogging };
enum class PowerState { on, off, estop, gstop, unknown };
enum class OperateMode { manual, automatic };
enum class MotionControlMode { Idle, RtCommand };
struct DragParameter { enum Space { jointSpace }; enum Type { freely }; };
struct Load {
    double mass;
    std::array<double,3> cog, inertia;
    Load(double m=0, std::array<double,3> c={}, std::array<double,3> i={}) : mass(m),cog(c),inertia(i) {}
};
struct Toolset { Load load; int end=17, ref=23; };
struct Controller {
    void setLoad(const Load& v, std::error_code& ec) {
        fake_sdk::calls.push_back("setLoad");
        fake_sdk::load={v.mass,v.cog[0],v.cog[1],v.cog[2],v.inertia[0],v.inertia[1],v.inertia[2]};
        ec=fake_sdk::fail_load ? std::make_error_code(std::errc::io_error) : std::error_code{};
    }
};
class ArRobot {
    OperationState operation_=OperationState::idle;
    PowerState power_=PowerState::off;
    OperateMode mode_=OperateMode::automatic;
    MotionControlMode control_=MotionControlMode::Idle;
    std::shared_ptr<Controller> controller_=std::make_shared<Controller>();
 public:
    void connectToRobot(const std::string&) { fake_sdk::calls.push_back("connect"); }
    void disconnectFromRobot(std::error_code& ec) { ec.clear(); fake_sdk::calls.push_back("disconnect"); }
    OperationState operationState(std::error_code& ec) { ec.clear(); return operation_; }
    PowerState powerState(std::error_code& ec) { ec.clear(); return power_; }
    OperateMode operateMode(std::error_code& ec) { ec.clear(); return mode_; }
    Toolset toolset(std::error_code& ec) { ec.clear(); return {}; }
    void setToolset(const Toolset& t, std::error_code& ec) {
        ec.clear();
        if(t.end!=17 || t.ref!=23 || t.load.mass!=.71) throw std::runtime_error("bad toolset");
        fake_sdk::calls.push_back("toolset_load");
    }
    void setMotionControlMode(MotionControlMode mode, std::error_code& ec) {
        ec.clear(); control_=mode;
        fake_sdk::calls.push_back(mode==MotionControlMode::Idle ? "idle" : "rt");
        if(mode==MotionControlMode::Idle && fake_sdk::fail_idle) ec=std::make_error_code(std::errc::io_error);
    }
    std::weak_ptr<Controller> getRtMotionController() {
        if(control_!=MotionControlMode::RtCommand) throw std::runtime_error("not RT");
        return controller_;
    }
    void setOperateMode(OperateMode mode, std::error_code& ec) { ec.clear(); mode_=mode; fake_sdk::calls.push_back("manual"); }
    void setPowerState(bool on, std::error_code& ec) {
        ec.clear(); power_=on ? PowerState::on:PowerState::off;
        fake_sdk::calls.push_back(on ? "power_on":"power_off");
    }
    void enableDrag(DragParameter::Space, DragParameter::Type, std::error_code& ec, bool no_button) {
        ec.clear();
        if(no_button || mode_!=OperateMode::manual || power_!=PowerState::off || control_!=MotionControlMode::Idle)
            throw std::runtime_error("invalid drag setup");
        operation_=OperationState::drag; fake_sdk::calls.push_back("drag_on");
    }
    void disableDrag(std::error_code& ec) { ec.clear(); operation_=OperationState::idle; fake_sdk::calls.push_back("drag_off"); }
    std::array<double,7> jointPos(std::error_code& ec) { ec.clear(); return {}; }
    void XPRS485SendData(int tx_size,int rx_size,const std::vector<uint8_t>& tx,std::vector<uint8_t>& rx,std::error_code& ec) {
        ec.clear(); if(tx_size!=int(tx.size()) || rx_size!=int(rx.size())) throw std::runtime_error("bad SDK buffers");
        fake_sdk::calls.push_back("transfer");
    }
};
}
