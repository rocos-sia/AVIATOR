#include "calibration_hand.hpp"
#include "runtime.hpp"
#include "../../../examples/xCoreSDK_RH56FTP_485/src/protocol.hpp"
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
using namespace aviator;
using namespace aviator::hand;
using Json = nlohmann::json;
void check(bool value, const char* why) { if (!value) throw std::runtime_error(why); }
struct Transport {
    std::mutex mutex;
    std::atomic<bool> fail_read{false}, lose_ack{false};
    std::atomic<int> error{0};
    Values position{1000, 900, 800, 700, 600, 500}; // device order
    std::vector<std::pair<int, Values>> writes;
    Bytes transfer(const Bytes& tx, size_t count) {
        std::lock_guard<std::mutex> lock(mutex);
        check(tx[0] == 0xEB && tx[1] == 0x90 && tx[2] == 1, "wrong hand request header");
        check(tx.back() == rh56::checksum(Bytes(tx.begin(), tx.end()-1)), "bad TX checksum");
        int address = tx[5] | (tx[6] << 8);
        Bytes payload;
        if (tx[4] == 0x12) {
            check(tx.size() == 20 && count == 9 && tx[3] == 15, "not one six-channel write");
            check(address == angle_set || address == speed_set || address == force_set, "unexpected register write");
            Values values;
            for (int i = 0; i < 6; ++i) values[i] = tx[7+2*i] | (tx[8+2*i] << 8);
            writes.emplace_back(address, values);
            if (address == angle_set) position = values;
            if (address == angle_set && lose_ack.exchange(false)) throw std::runtime_error("injected lost write ACK");
            payload = {1};
        } else {
            check(tx.size() == 9, "wrong read request size");
            if (fail_read) throw std::runtime_error("injected read failure");
            if (address == rh56::errorAct) {
                check(tx[7] == 6 && count == 14, "error read must be six bytes");
                payload = {uint8_t(error.load()),0,0,0,0,0};
            } else {
                check(tx[7] == 12 && count == 20, "angle/force read must be twelve bytes");
                check(address == rh56::angleAct || address == rh56::forceAct, "unexpected read register");
                Values values = address == rh56::angleAct ? position : Values{-1,-2,-3,-4,-5,-6};
                for (int v : values) { payload.push_back(uint8_t(v)); payload.push_back(uint8_t(uint16_t(v) >> 8)); }
            }
        }
        Bytes reply{0x90,0xEB,tx[2],uint8_t(payload.size()+3),tx[4],tx[5],tx[6]};
        reply.reserve(count);
        for (auto value : payload) reply.push_back(value);
        reply.push_back(rh56::checksum(reply));
        return reply;
    }
    size_t count() { std::lock_guard<std::mutex> lock(mutex); return writes.size(); }
};
int main() {
    try {
        Config cfg;
        cfg.command_us = 200000;
        cfg.speed = 800; cfg.force = 2000;
        Transport io[2];
        std::array<Transfer, 2> transfers;
        for (int s=0;s<2;++s) transfers[s]=[&,s](const Bytes& tx,size_t n){return io[s].transfer(tx,n);};
        uint64_t sequence=0;
        std::array<uint64_t,2> revisions{1,1};
        std::array<Values,2> targets{Values{500,100,200,300,400,500}, Values{500,600,700,800,900,1000}};
        auto epoch=new_session_id();
        auto command=[&] {
            Message m;
            m.topic=Topic::hand_command;
            m.header={"1.0",++sequence,utc_us(),monotonic_us(),local_clock_id(),"calibration","test",true};
            m.body={{"mode","NORMALIZED_POSITION"},{"control_epoch",epoch},
                {"origin",{{"publisher_id","calibration"},{"session_id","test"},{"sequence",sequence},
                    {"sample_mono_us",m.header.sample_mono_us},{"clock_id",local_clock_id()}}}};
            for (int s=0;s<2;++s) {
                auto& h=m.body["hands"][s ? "right":"left"];
                h["calibration_revision"]=revisions[s];
                for (int v:targets[s]) h["drive_position_normalized"].push_back(v/1000.);
            }
            return m;
        };
        {
            calibration::CalibrationHands hands(cfg,transfers,{open_pose,open_pose});
            auto wait=[&](auto predicate, int timeout=2500) {
                auto until=monotonic_us()+timeout*1000;
                Json state;
                do {
                    hands.receive(command());
                    state=hands.state().body;
                    if (predicate(state)) return state;
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                } while(monotonic_us()<until);
                throw std::runtime_error("timeout: "+state.dump());
            };
            auto state=wait([](const Json& j){return j["command_valid"]==true;});
            check(state["hands"]["left"]["force"]==Json(Values{-6,-5,-4,-3,-2,-1}), "signed force/channel mapping wrong");
            for(int s=0;s<2;++s) {
                std::lock_guard<std::mutex> lock(io[s].mutex);
                check(io[s].writes.size()==3, "duplicate or missing writes");
                check(io[s].writes[0].first==speed_set && io[s].writes[1].first==force_set &&
                      io[s].writes[2].first==angle_set, "write order mismatch");
                auto expected=targets[s]; std::reverse(expected.begin(),expected.end());
                check(io[s].writes[2].second==expected, "angle mapping wrong");
            }
            io[0].fail_read=true;
            wait([](const Json& j){return j["hands"]["left"]["status"]=="RECOVERING";});
            auto before=io[0].count();
            ++revisions[1]; targets[1].fill(100);
            wait([](const Json& j){return j["hands"]["right"]["applied_revision"]==2;});
            check(io[0].count()==before, "faulted hand received writes");
            io[0].fail_read=false;
            state=wait([](const Json& j){return j["hands"]["left"]["retry_required"]==true;});
            check(io[0].count()==before && state["command_valid"]==false, "old command replayed on recovery");
            ++revisions[0];
            wait([](const Json& j){return j["command_valid"]==true;});
            // Actuator errors must remain visible and isolated too.
            io[0].error=7;
            state=wait([](const Json& j){return j["hands"]["left"]["status"]=="RECOVERING";});
            check(state["hands"]["left"]["error_codes"][5]==7, "error channel mapping missing");
            check(state["hands"]["right"]["command_valid"]==true, "healthy hand stopped");
            io[0].error=0;
            wait([](const Json& j){return j["hands"]["left"]["retry_required"]==true;});
            ++revisions[0]; io[0].lose_ack=true;
            wait([](const Json& j){return j["hands"]["left"]["status"]=="RECOVERING";});
            before=io[0].count();
            wait([](const Json& j){return j["hands"]["left"]["retry_required"]==true;});
            check(io[0].count()==before, "lost ACK caused blind replay");
            ++revisions[0];
            wait([](const Json& j){return j["command_valid"]==true;});
            // Expired keepalives may not silently re-arm an old close target.
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            check(hands.state().body["command_valid"]==false, "stale command accepted");
            state=wait([](const Json& j){return j["hands"]["left"]["retry_required"]==true;});
            ++revisions[0]; ++revisions[1];
            wait([](const Json& j){return j["command_valid"]==true;});
            auto invalid = command();
            invalid.header.valid = false;
            hands.receive(invalid);
            state=wait([](const Json& j){return j["hands"]["left"]["retry_required"]==true;});
            check(state["command_valid"]==false, "invalidated target rearmed without click");
        }
        for(auto& side:io) {
            check(side.writes.back().first==angle_set && side.writes.back().second==Values{1000,1000,1000,1000,1000,500},
                  "shutdown did not request open");
        }
        std::cout << "Calibration hand protocol, isolation, recovery and cleanup passed\n";
        return 0;
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
