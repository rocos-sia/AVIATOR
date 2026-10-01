#include "simulation.hpp"
#include "transport.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <future>
#include <spawn.h>
#include <sys/wait.h>
#include <signal.h>
extern char** environ;
using namespace simulation;
void check(bool condition,const char* why) { if(!condition) throw std::runtime_error(why); }
const std::string core="22222222-2222-4222-8222-222222222222", epoch="aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa", origin="11111111-1111-4111-8111-111111111111";
aviator::Message command(Simulation& sim,aviator::Topic topic,std::uint64_t now,std::uint64_t sequence) {
    aviator::Message m; m.topic=topic;
    m.header={"1.0",sequence,aviator::utc_us(),now,sim.clock(),"aviator_core",core,true};
    if(topic==aviator::Topic::camera_command) {
        m.body={{"camera_id","cockpit_camera"},{"target","YOKE"},{"tracking_enabled",true},{"roi",nullptr},{"min_confidence",.8}};
    } else {
        const bool hand=topic==aviator::Topic::hand_command;
        const char* group=hand?"hands":"arms";
        auto state=sim.state(hand,now);
        m.body={{"mode","JOINT_POSITION"},{"control_epoch",epoch},
            {"origin",{{"publisher_id","flight_gateway"},{"session_id",origin},{"sequence",1},{"sample_mono_us",now},{"clock_id",sim.clock()}}}};
        for(auto side:{"left","right"}) m.body[group][side]["joint_position"]=state.body[group][side]["joint_position"];
    }
    return m;
}
void model_test(const std::string& path) {
    Simulation sim(path,{"aviator_core",core,epoch,"flight_gateway",origin,200000});
    const auto now=aviator::monotonic_us(); std::string error,payload;
    auto m=command(sim,aviator::Topic::arm_command,now,1);
    const double initial=m.body["arms"]["left"]["joint_position"][0];
    m.body["arms"]["left"]["joint_position"][0]=initial+.03;
    aviator::Message decoded;
    check(aviator::encode(m,payload,error),"encode command");
    check(aviator::decode("arm.command",payload,decoded,error),"decode command");
    check(sim.command(decoded,now,error),"accept arm command");
    check(!sim.command(decoded,now,error),"duplicate rejected");
    for(int i=0;i<200;++i) sim.step(now+1000); // fixed test clock isolates dynamics from watchdog
    auto state=sim.state(false,now+1000);
    check(state.body["arms"]["left"]["joint_position"][0].get<double>()>initial+.01,"arm physically follows target");
    check(state.body["arms"]["left"]["status"]=="ACTIVE","active arm");
    check(state.body["arms"]["left"]["tcp_pose"]["frame_id"]=="mujoco_world","pose frame");
    for(const auto& invalid : {"range","length","epoch","publisher","origin","clock","mode","future"}) {
        auto bad=m; bad.header.sequence=2;
        std::string kind=invalid;
        if(kind=="range") bad.body["arms"]["right"]["joint_position"][0]=999;
        if(kind=="length") bad.body["arms"]["right"]["joint_position"]=Json::array();
        if(kind=="epoch") bad.body["control_epoch"]=origin;
        if(kind=="publisher") bad.header.publisher_id="foreign";
        if(kind=="origin") bad.body["origin"]["sample_mono_us"]=now-100001;
        if(kind=="clock") bad.header.clock_id="wrong";
        if(kind=="mode") bad.body["mode"]="JOINT_TRAJECTORY";
        if(kind=="future") bad.header.sample_mono_us=now+1;
        check(!sim.command(bad,now,error),"invalid command rejected");
    }
    m.header.sequence=2; check(sim.command(m,now,error),"rejection did not consume sequence");
    m.header.sequence=3; m.header.valid=false;
    check(!sim.command(m,now,error),"invalid report revokes command"); sim.step(now);
    check(sim.state(false,now).body["arms"]["left"]["status"]=="SAFE","invalid stops execution");
    m.header.sequence=4; m.header.valid=true; check(sim.command(m,now,error),"reaccept");
    sim.step(now+50000);
    check(sim.state(false,now+50000).body["arms"]["left"]["status"]=="SAFE","50ms timeout boundary");
    auto hand=command(sim,aviator::Topic::hand_command,now,1);
    hand.body["mode"]="NORMALIZED_POSITION";
    for(auto side:{"left","right"}) hand.body["hands"][side]={{"drive_position_normalized",{.2,.2,.2,.2,.2,.2}}};
    check(sim.command(hand,now,error),"normalized hand command");
    for(int i=0;i<300;++i) sim.step(now);
    check(sim.state(true,now).body["hands"]["left"]["joint_position"][2].get<double>()>.05,"finger physically follows target");
    auto camera=command(sim,aviator::Topic::camera_command,now,1);
    check(sim.command(camera,now,error),"camera accepted");
    check(sim.detection(now,true,true).header.valid,"valid detection");
    check(!sim.detection(now,true,false).header.valid,"ROI excluded");
    auto lost=sim.detection(now+200000,true,true);
    check(!lost.header.valid && lost.body["yoke"]["roll"].is_null(),"camera timeout null target");
    check(sim.detection(now,false,false).body["frame_id"].is_null(),"offline no frame");
    camera.header.sequence=2; camera.body["roi"]={{"x",639},{"y",0},{"width",2},{"height",1}};
    check(!sim.command(camera,now,error),"bad ROI rejected");
    for(bool h:{false,true}) check(aviator::encode(sim.state(h,now),payload,error),"state encodes");
    check(aviator::encode(lost,payload,error),"detection encodes");
    Simulation readonly(path);
    check(!readonly.command(m,now,error),"default has no authorization");
}
void process_test(const char* executable,const char* path,bool camera) {
    // In-test XSUB/XPUB bus uses ephemeral TCP ports, independent of running production buses.
    zmq::context_t context(1);
    zmq::socket_t xsub(context,zmq::socket_type::xsub),xpub(context,zmq::socket_type::xpub);
    aviator::configure(xsub); aviator::configure(xpub);
    xsub.bind("tcp://127.0.0.1:*"); xpub.bind("tcp://127.0.0.1:*");
    std::string input=xsub.get(zmq::sockopt::last_endpoint),output=xpub.get(zmq::sockopt::last_endpoint);
    zmq::socket_t pub(context,zmq::socket_type::pub),sub(context,zmq::socket_type::sub);
    aviator::configure(pub); aviator::configure(sub); aviator::subscribe(sub,"");
    pub.connect(input); sub.connect(output);
    std::vector<std::string> args={executable,"--headless","--model",path,"--duration","15","--pub-endpoint",input,"--sub-endpoint",output,
        "--core-session",core,"--origin-session",origin,"--control-epoch",epoch};
    if(!camera) args.push_back("--no-camera");
    std::vector<char*> argv; for(auto& a:args) argv.push_back(a.data()); argv.push_back(nullptr);
    pid_t pid; check(posix_spawn(&pid,executable,nullptr,nullptr,argv.data(),environ)==0,"spawn simulation");
    bool arms=false,hands=false,vision=false,active=false,hand_active=false,tracked=false;
    aviator::ReceiveState rs;
    std::uint64_t sequence=0;
    auto end=aviator::monotonic_us()+12000000;
    try {
        while(aviator::monotonic_us()<end) {
            // Forward complete multipart packets and subscription frames, on their owner thread.
            for(auto pair : {std::pair<zmq::socket_t*,zmq::socket_t*>{&xsub,&xpub},{&xpub,&xsub}}) {
                for(int n=0;n<64;++n) {
                    zmq::message_t frame;
                    if(!pair.first->recv(frame,zmq::recv_flags::dontwait)) break;
                    bool more=frame.more(); pair.second->send(frame,more?zmq::send_flags::sndmore:zmq::send_flags::none);
                }
            }
            aviator::WireMessage wire; std::string error;
            while(aviator::receive(sub,rs,wire,error)==aviator::ReceiveResult::received) {
                aviator::Message m; check(aviator::decode(wire.topic,wire.payload,m,error),"wire state decodes");
                if(m.topic==aviator::Topic::arm_state) { arms=true; active |= m.body["arms"]["left"]["status"]=="ACTIVE"; }
                if(m.topic==aviator::Topic::hand_state) {
                    hands=true; hand_active |= m.body["hands"]["left"]["status"]=="ACTIVE";
                }
                if(m.topic==aviator::Topic::camera_detection) {
                    vision=true; tracked |= m.header.valid;
                    if(!camera) check(m.body["status"]=="OFFLINE" && m.body["yoke"]["roll"].is_null(),"offline wire state");
                }
                if(m.topic==aviator::Topic::arm_state) {
                    auto now=aviator::monotonic_us(); aviator::Message cmd;
                    cmd.topic=aviator::Topic::arm_command;
                    cmd.header={"1.0",++sequence,aviator::utc_us(),now,aviator::local_clock_id(),"aviator_core",core,true};
                    cmd.body={{"mode","JOINT_POSITION"},{"control_epoch",epoch},{"origin",{{"publisher_id","flight_gateway"},{"session_id",origin},{"sequence",sequence},{"clock_id",cmd.header.clock_id},{"sample_mono_us",now}}}};
                    for(auto side:{"left","right"}) cmd.body["arms"][side]["joint_position"]=m.body["arms"][side]["joint_position"];
                    std::string payload; check(aviator::encode(cmd,payload,error),"encode wire command"); aviator::send(pub,"arm.command",payload);
                    cmd.topic=aviator::Topic::hand_command;
                    cmd.body.erase("arms"); cmd.body["mode"]="NORMALIZED_POSITION";
                    for(auto side:{"left","right"}) cmd.body["hands"][side]={{"drive_position_normalized",{.1,.1,.1,.1,.1,.1}}};
                    check(aviator::encode(cmd,payload,error),"encode hand command"); aviator::send(pub,"hand.command",payload);
                    cmd.topic=aviator::Topic::camera_command;
                    cmd.body={{"camera_id","cockpit_camera"},{"target","YOKE"},{"tracking_enabled",true},{"roi",nullptr},{"min_confidence",.8}};
                    check(aviator::encode(cmd,payload,error),"encode camera command"); aviator::send(pub,"camera.command",payload);
                }
            }
            if(arms && hands && vision && active && hand_active && (!camera || tracked)) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        kill(pid,SIGTERM); int status; waitpid(pid,&status,0);
        check(WIFEXITED(status) && WEXITSTATUS(status)==0,"simulation exits cleanly");
        check(arms&&hands&&vision&&active&&hand_active,"bus command/state round trip");
        if(camera) check(tracked,"EGL ground-truth detection round trip");
    } catch(...) { kill(pid,SIGKILL); waitpid(pid,nullptr,0); throw; }
}
int main(int argc,char** argv) {
    try {
        if(argc==2) model_test(argv[1]);
        else if(argc==4) process_test(argv[1],argv[2],std::string(argv[3])=="camera");
        else throw std::runtime_error("invalid test arguments");
        std::cout<<"simulation tests passed\n"; return 0;
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
