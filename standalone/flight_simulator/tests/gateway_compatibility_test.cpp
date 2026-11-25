#include "rs422.hpp"
#include <nodes/flight_gateway/rs422.hpp>
#include <iostream>
#include <stdexcept>
#include <string>

namespace sim = flight_simulator::rs422;
namespace peer = flight_gateway::rs422;

void check(bool ok, const char* message) {
    if (!ok)
        throw std::runtime_error(message);
}

int main() try {
    check(sim::encode(sim::Downlink{}) == peer::encode(peer::Downlink{}), "neutral target differs");
    check(sim::encode(sim::Uplink{}) == peer::encode(peer::Uplink{}), "unknown feedback differs");

    // Simulator commands must be accepted by the actual gateway and dispatched once.
    for (uint8_t operation : {0x10, 0x20, 0x30, 0x40, 0x50, 0x60}) {
        sim::CommandSender sender;
        peer::CommandGate gate;
        check(sender.request(operation), "operation request rejected");
        unsigned dispatched = 0;
        for (unsigned i = 0; i < 10; ++i) {
            sim::Downlink input{uint8_t(252 + i), sender.next(), -32767, 32767};
            const auto wire = sim::encode(input);
            peer::Downlink decoded;
            check(peer::decode(wire, decoded), "gateway rejected simulator command");
            check(peer::encode(decoded) == wire, "downlink fields differ");
            const auto result = gate.update(decoded, i != 0, 1000000 + i * sim::period_us);
            check(result == (i == 4 ? operation : 0), "gateway command confirmation differs");
            dispatched += result != 0;
        }
        check(dispatched == 1 && sender.next() == 0, "operation did not finish after ten frames");
    }

    // Independently built gateway frames exercise every feedback field, including
    // emergency stop and unknown state, without using the simulator's encoder.
    peer::Uplink feedback;
    feedback.counter = 255;
    feedback.selected = 1;
    feedback.communication = 2;
    feedback.operation = 0x60;
    feedback.roll_echo = -12345;
    feedback.pitch_echo = 2184;
    feedback.rear = 1;
    feedback.brake = 0;
    feedback.roll = -32767;
    feedback.pitch = 32767;
    feedback.joint_roll = 32767;
    feedback.joint_pitch = -32767;
    feedback.fault = 8201;
    feedback.last_fault = 1301;
    feedback.severity = 0x30;
    feedback.version = 0x0201;
    feedback.checksum = 0xabcd;
    for (unsigned state = 0; state < 256; ++state) {
        check(sim::stateCode(peer::stateName(state)) == peer::stateCode(peer::stateName(state)),
              "state mapping differs");
        check(std::string(sim::stateName(state)) == peer::stateName(state), "state label differs");
        feedback.state = uint8_t(state);
        const auto wire = peer::encode(feedback);
        sim::Uplink decoded;
        peer::Uplink peer_decoded;
        const bool accepted = peer::decode(wire, peer_decoded);
        check(sim::decode(wire, decoded) == accepted, "state acceptance differs");
        if (accepted)
            check(sim::encode(decoded) == wire, "uplink fields differ");
    }

    // A code dictionary mismatch would make a whole otherwise healthy frame STALE.
    for (unsigned code = 0; code <= 0xffff; ++code)
        check(sim::knownFault(code) == peer::knownFault(code), "fault dictionary differs");

    // Compare receive validation over every value of each byte, including reserved
    // bytes. Re-seal each frame so validation reaches the fields under test.
    for (bool uplink : {false, true}) {
        const auto baseline = uplink ? peer::encode(peer::Uplink{}) : sim::encode(sim::Downlink{});
        for (size_t offset = 2; offset < baseline.size() - 1; ++offset) {
            for (unsigned value = 0; value < 256; ++value) {
                auto wire = baseline;
                wire[offset] = uint8_t(value);
                wire.back() = peer::checksum(wire.data() + 2, wire.size() - 3);
                sim::Uplink su;
                peer::Uplink pu;
                sim::Downlink sd;
                peer::Downlink pd;
                const bool a = uplink ? sim::decode(wire, su) : sim::decode(wire, sd);
                const bool b = uplink ? peer::decode(wire, pu) : peer::decode(wire, pd);
                check(a == b, "field validation differs");
                if (a)
                    check((uplink ? sim::encode(su) : sim::encode(sd)) ==
                              (uplink ? peer::encode(pu) : peer::encode(pd)),
                          "decoded field layout differs");
            }
        }
    }
    std::cout << "PASS: current gateway wire format, state/fault dictionary, command confirmation\n";
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}
