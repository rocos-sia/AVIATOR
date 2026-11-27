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
    // The gateway still implements the older unknown-fault telemetry semantics.
    // Test shared wire layout without pretending that default telemetry is compatible.
    sim::Uplink legacy;
    check(!sim::decode(peer::encode(peer::Uplink{}), legacy), "legacy unknown fault must be rejected");

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
    // emergency stop, without using the simulator's encoder.
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
        if (state != 0xfe)
            check(std::string(sim::stateName(state)) == peer::stateName(state), "shared state label differs");
        feedback.state = uint8_t(state);
        feedback.fault = state == 0xfe ? 0 : 8201;
        feedback.severity = state == 0xfe ? 0 : 0x30;
        const auto wire = peer::encode(feedback);
        sim::Uplink decoded;
        peer::Uplink peer_decoded;
        const bool accepted = peer::decode(wire, peer_decoded);
        check(sim::decode(wire, decoded) == accepted, "state acceptance differs");
        if (accepted)
            check(sim::encode(decoded) == wire, "uplink fields differ");
    }

    // Exactly six additions are intentional; all previous business codes remain valid.
    const auto added = [](unsigned code) { return (code >= 7202 && code <= 7206) || code == 8212; };
    for (unsigned code = 0; code <= 0xffff; ++code)
        check(sim::knownFault(code) == (peer::knownFault(code) || added(code)), "unexpected dictionary difference");
    for (uint16_t code : {7202, 7203, 7204, 7205, 7206, 8212}) {
        feedback.state = 0x10; feedback.fault = code; feedback.last_fault = code; feedback.severity = 0x30;
        const auto wire = peer::encode(feedback);
        sim::Uplink decoded;
        check(sim::decode(wire, decoded) && sim::encode(decoded) == wire, "new alarm wire layout differs");
    }

    // Compare unchanged fields, including every reserved byte. Status/fault rules
    // deliberately differ and are covered above and by independent 1127 vectors.
    feedback.state = 7; feedback.fault = feedback.last_fault = 0; feedback.severity = 0;
    for (bool uplink : {false, true}) {
        const auto baseline = uplink ? peer::encode(feedback) : sim::encode(sim::Downlink{});
        for (size_t offset = 2; offset < baseline.size() - 1; ++offset) {
            if (uplink && (offset == 10 || (offset >= 17 && offset <= 21))) continue;
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
    std::cout << "PASS: shared gateway layout/commands; ICD 1127 dictionary additions and legacy telemetry rejection\n";
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}
