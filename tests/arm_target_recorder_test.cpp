#include "arm_target_recorder.hpp"
#include "logger.hpp"
#include "runtime.hpp"
#include "transport.hpp"
#include <mcap/mcap.hpp>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <thread>
#include <cmath>
using namespace aviator;
using namespace std::chrono_literals;
void check(bool b, const char* what) { if (!b) throw std::runtime_error(what); }
const std::string core = "11111111-1111-4111-8111-111111111111";
const std::string device = "22222222-2222-4222-8222-222222222222";
const std::string epoch = "33333333-3333-4333-8333-333333333333";
Json vector7(uint64_t tick, int side, double extra = 0) {
    Json q = Json::array();
    for (int j = 0; j < 7; ++j) q.push_back(side + .1 * j + .0001 * tick + extra);
    return q;
}
Json command(bool streaming = true, uint64_t seq = 10) {
    Json m = {{"msg_type", "ArmCommand"}, {"version", "1.0"}, {"publisher_id", "aviator_core"},
        {"session_id", core}, {"clock_id", "boot"}, {"sequence", seq}, {"timestamp", 1000000},
        {"sample_mono_us", 1000000}, {"valid", true}, {"control_epoch", epoch},
        {"mode", "JOINT_TRAJECTORY"}, {"execution", "SYNCHRONIZED_TICKS"},
        {"trajectory_id", 7}, {"first_tick", 100}, {"total_ticks", 200}, {"config_id", "test"},
        {"origin", {{"publisher_id", "aviator_core"}, {"session_id", core}, {"clock_id", "boot"},
                    {"sample_mono_us", 1000000}, {"sequence", seq}}}, {"streaming", streaming}};
    for (int side = 0; side < 2; ++side) {
        auto& p = m["arms"][side ? "right" : "left"]["points"] = Json::array();
        for (int k = 0; k < (streaming ? 51 : 32); ++k) {
            auto t = k * (streaming ? 1 : 2);
            Json point = {{"time_from_start_us", t * 1000}, {"joint_position", vector7(100 + t, side)},
                          {"joint_velocity", std::vector<double>(7, .1)}};
            if (streaming) point["joint_acceleration"] = std::vector<double>(7, .2);
            p.push_back(point);
        }
    }
    return m;
}
Json state(uint64_t seq = 1, uint64_t accepted = 10) {
    Json target = vector7(123, 0);
    for (auto v : vector7(123, 1)) target.push_back(v);
    return {{"msg_type", "ArmState"}, {"version", "1.0"}, {"publisher_id", "manipulator"},
        {"session_id", device}, {"clock_id", "boot"}, {"sequence", seq}, {"timestamp", 1010000},
        {"sample_mono_us", 999000}, {"status_mono_us", 1010000}, {"valid", true}, {"config_id", "test"},
        {"execution", {{"trajectory_id", 7}, {"tick", 123}, {"target", target}, {"stopping", false}, {"fault", false}}},
        {"accepted_command", {{"publisher_id", "aviator_core"}, {"session_id", core}, {"sequence", accepted}, {"control_epoch", epoch}}}};
}
Json correlate(const Json& c, const Json& s) {
    ArmTargetRecorder recorder("logger"); recorder.command(c.dump());
    auto r = recorder.state(s.dump()); check(r.has_value(), "state missing"); return *r;
}
void unit() {
    const auto c = command(), s = state();
    auto r = correlate(c, s);
    check(r["valid"] && r["derivatives_available"], "Servo match failed");
    check(r["joint_position"] == s["execution"]["target"], "used first point instead of cursor");
    check(r["joint_velocity"] == std::vector<double>(14, .1) && r["joint_acceleration"] == std::vector<double>(14, .2), "derivatives/order");
    check(r["sample_mono_us"] == 1010000 && r["source_state"]["sample_mono_us"] == 999000, "execution/measurement times conflated");
    auto relabeled = s;
    relabeled["accepted_command"]["session_id"] = "different-text-marker";
    check(correlate(c, relabeled)["valid"], "session label blocked target recording");
    r = correlate(command(false), s);
    check(r["valid"] && !r["derivatives_available"] && r["joint_acceleration"].is_null(), "ordinary derivatives fabricated");
    check(std::abs(r["joint_position"][0].get<double>() - .0123) < 1e-12, "ordinary odd tick interpolation");
    for (const char* field : {"control_epoch"}) {
        auto bad = s; bad["accepted_command"][field] = new_session_id();
        check(correlate(c, bad)["reason"] == "accepted_window_not_cached", "cross session/epoch match");
    }
    for (const char* field : {"fault", "stopping"}) {
        auto bad = s; bad["execution"][field] = true;
        check(!correlate(c, bad)["valid"], "stop/fault matched to ordinary target");
    }
    auto bad = s; bad["execution"]["target"][0] = 2;
    check(correlate(c, bad)["reason"] == "execution_target_mismatch", "wrong execution target accepted");
    bad = s; bad["execution"]["tick"] = 180;
    check(correlate(c, bad)["reason"] == "cursor_outside_window", "missing point fabricated");
    bad = s; bad["clock_id"] = "other";
    check(!correlate(c,bad)["valid"], "cross clock match");
    bad = s; bad.erase("status_mono_us");
    r = correlate(c,bad);
    check(!r["valid"] && r["sample_mono_us"] == 0 && r["time_basis"] == "unavailable",
          "missing execution timestamp was replaced with measurement timestamp");
    auto malformed = c; malformed["arms"]["left"]["points"][23]["joint_acceleration"] = {1};
    check(!correlate(malformed,s)["valid"], "malformed acceleration accepted");
    malformed = c; malformed["arms"]["left"]["points"][23]["time_from_start_us"] = 9;
    check(correlate(malformed,s)["reason"] == "invalid_time_grid", "invalid time grid accepted");
    ArmTargetRecorder reordered("logger");
    check(!reordered.state(s.dump())->at("valid"), "state without window matched");
    reordered.command(c.dump());
    check(reordered.state(state(2).dump())->at("valid"), "late window never usable");
    check(reordered.state(s.dump())->at("reason") == "duplicate_or_out_of_order_state", "old state accepted");
    auto restarted = s; restarted["session_id"] = new_session_id();
    check(reordered.state(restarted.dump())->at("valid"), "device restart sequence not isolated");
    ArmTargetRecorder changed("logger"); changed.command(c.dump());
    auto newer = command(true,11); changed.command(newer.dump()); // Keep exact ACK, not latest window.
    check(changed.state(s.dump())->at("valid"), "exact older ack lost");
    newer = c; newer["arms"]["left"]["points"][0]["joint_position"][0] = 2;
    changed.command(newer.dump());
    check(changed.state(state(2).dump())->at("reason") == "conflicting_command_identity", "conflicting duplicate silently chosen");
    ArmTargetRecorder bounded("logger");
    for (int i=10; i<75; ++i) bounded.command(command(true,i).dump());
    check(!bounded.state(s.dump())->at("valid") && bounded.summary()["cache_evictions"] == 1, "bounded eviction/failure");
}
uint64_t transport(const std::filesystem::path& dir, const std::string& mode) {
    zmq::context_t context{1}; zmq::socket_t pub(context,zmq::socket_type::xpub);
    configure(pub, {1024, 1024, 0}); pub.bind("tcp://127.0.0.1:*");
    const auto endpoint = pub.get(zmq::sockopt::last_endpoint);
    const auto path = (dir/(mode+".mcap")).string();
    RecorderOptions options; options.arm_command_mode = mode;
    std::atomic<bool> stop{false};
    auto future = std::async(std::launch::async,[&]{return record_bus(endpoint,path,"logger",stop,options);});
    std::vector<std::pair<std::string,std::string>> originals;
    try {
        zmq::pollitem_t p{pub.handle(),0,ZMQ_POLLIN,0}; zmq::poll(&p,1,3s);
        check(p.revents & ZMQ_POLLIN,"logger subscription timeout");
        zmq::message_t subscription; check(pub.recv(subscription).has_value(),"subscription");
        for (int i=0;i<20;++i) {
            originals.emplace_back("arm.command", " \n" + command(true,10+i).dump() + "\t");
            originals.emplace_back("arm.state", " \n" + state(i+1,10+i).dump() + "\t");
        }
        originals.emplace_back("arm.state",state(21,99).dump()); // Explicit missing target record.
        Json other = {{"msg_type","HandState"},{"version","1.0"},{"sequence",1},{"timestamp",1000000},
            {"sample_mono_us",1000000},{"clock_id","boot"},{"publisher_id","inspire_hand"},{"session_id",device},{"valid",false}};
        originals.emplace_back("hand.state",other.dump());
        for (const auto& [topic,data]: originals) check(send(pub,topic,data),"publish test traffic");
        std::this_thread::sleep_for(300ms); stop=true;
        check(future.wait_for(5s)==std::future_status::ready,"logger shutdown");
        const auto summary=future.get();
        check(summary.invalid==0 && summary.dropped==0,"invalid/dropped records");
        check(summary.topics.count("arm.command") == (mode=="full"), "arm.command storage policy");
        check(summary.topics.count("record.arm.target") == (mode=="compact"),"derived topic absent");
        mcap::McapReader reader; check(reader.open(path).ok(),"MCAP read");
        size_t original=0,targets=0,missing=0;
        for (const auto& view:reader.readMessages()) {
            std::string payload(reinterpret_cast<const char*>(view.message.data),view.message.dataSize);
            if (view.channel->topic=="record.arm.target") {
                auto r=Json::parse(payload); ++targets;
                if (!r["valid"]) { ++missing; check(r["joint_position"].is_null(),"missing position fabricated"); }
                else check(r["joint_position"]==state()["execution"]["target"],"MCAP target roundtrip");
                check(view.schema && view.schema->name=="RecordedArmTarget/1.0","derived schema");
                auto schema=Json::parse(std::string(reinterpret_cast<const char*>(view.schema->data.data()),view.schema->data.size()));
                check(schema["properties"]["joint_acceleration"]["maxItems"]==14,"incomplete schema");
            } else {
                while (mode=="compact" && original<originals.size() && originals[original].first=="arm.command") ++original;
                check(original<originals.size() && view.channel->topic==originals[original].first && payload==originals[original].second,
                      "original topic/payload changed");
                ++original;
            }
        }
        check(original==originals.size(),"original records lost");
        check(targets==(mode=="compact"?21u:0u) && missing==(mode=="compact"?1u:0u),"target count");
        reader.close();
    } catch (...) { stop=true; if(future.valid())future.wait(); throw; }
    return std::filesystem::file_size(path);
}
int main() {
    const auto dir=std::filesystem::temp_directory_path()/("arm-target-test-"+new_session_id());
    try {
        std::filesystem::create_directory(dir); unit();
        auto file=dir/"config.yaml";
        std::ofstream(file)<<"config_version: 1\n";
        check(load_recording_config(file.string()).options.arm_command_mode=="full","default must preserve history");
        std::ofstream(file)<<"config_version: 1\narm_command: {mode: compact}\n";
        auto cfg=load_recording_config(file.string());
        check(cfg.options.arm_command_mode=="compact" && recording_config_json(cfg)["arm_command"]["mode"]=="compact","config roundtrip");
        std::ofstream(file)<<"config_version: 1\narm_command: {mode: invalid}\n";
        bool rejected=false;try{load_recording_config(file.string());}catch(const std::exception&){rejected=true;}
        check(rejected,"unknown mode accepted");
        const auto full=transport(dir,"full"), compact=transport(dir,"compact");
        check(compact<full/2,"compact fixture not materially smaller");
        std::cout<<"PASS arm target correlation/storage: full="<<full<<" compact="<<compact<<" bytes\n";
        std::filesystem::remove_all(dir); return 0;
    } catch(const std::exception& e){std::cerr<<e.what()<<" artifacts="<<dir<<'\n';return 1;}
}
