#include "flight_operation.hpp"
#include "rs422.hpp"
#include "rs422_runtime.hpp"
#include <iostream>
#include <stdexcept>
using namespace flight_gateway;
using namespace flight_gateway::rs422;
void check(bool b, const char* reason) {
    if (!b)
        throw std::runtime_error(reason);
}
int main() try {
    check(encode(Downlink{}) == Bytes({0xeb, 0x90, 0, 0, 0, 0, 0, 0, 0x85}),
          "independent downlink vector");
    auto bytes = encode(Downlink{255, 0x30, -1000, 1000});
    Downlink d;
    check(bytes[4] == 0x18 && bytes[5] == 0xfc && bytes[6] == 0xe8 && bytes[7] == 3,
          "signed little endian endpoints");
    check(decode(bytes, d) && d.roll == -1000 && d.pitch == 1000, "decode endpoints");
    bytes[8] ^= 1;
    check(!decode(bytes, d), "bad checksum rejected");
    bytes = encode(Downlink{});
    bytes[3] = 0x70;
    bytes[8] = checksum(bytes.data(), 8);
    check(!decode(bytes, d), "unknown operation rejected");
    Uplink up;
    up.version = 0x1003;
    up.checksum = 0xba78;
    up.selected = 0;
    up.communication = 0;
    up.state = 0;
    up.rear = up.brake = 0;
    up.fault = up.last_fault = 0;
    up.severity = 0;
    up.roll_echo = up.pitch_echo = up.roll = up.pitch = 0;
    bytes = encode(up);
    check(bytes[26] == 0x40 && bytes[22] == 3 && bytes[23] == 0x10 && bytes[24] == 0x78 &&
              bytes[25] == 0xba,
          "independent uplink vector");
    Uplink decoded;
    check(decode(bytes, decoded), "uplink decode");
    up = Uplink{};
    check(decode(encode(up), decoded) && decoded.roll == invalid_position &&
              decoded.fault == unknown_fault,
          "bench unknown sentinel roundtrip");
    Parser parser(false);
    auto down = encode(Downlink{1, 0, -1000, 0});
    Bytes stream{0x90, 0xeb, 0, 0xff};
    auto bad = down;
    bad[8] ^= 2;
    stream.insert(stream.end(), bad.begin(), bad.end());
    stream.insert(stream.end(), down.begin(), down.end());
    stream.insert(stream.end(), down.begin(), down.end());
    unsigned count = 0;
    for (auto b : stream)
        count += parser.feed(&b, 1).size();
    check(count == 2 && parser.rejected > 0, "fragmented noisy stream resynchronizes");
    Parser upper(true);
    auto frame = encode(up);
    Bytes both = frame;
    both.insert(both.end(), frame.begin(), frame.end());
    check(upper.feed(both.data(), both.size()).size() == 2, "coalesced uplink frames");
    CommandGate gate;
    Downlink f;
    f.operation = 0x20;
    for (int i = 0; i < 20; ++i)
        check(!gate.update(f, true), "startup cannot execute held command");
    f.operation = 0;
    for (int i = 0; i < 10; ++i)
        gate.update(f, true);
    f.operation = 0x20;
    for (int i = 0; i < 4; ++i)
        check(!gate.update(f, true), "before fifth frame");
    check(gate.update(f, true) == 0x20, "fifth frame dispatches");
    for (int i = 0; i < 40; ++i)
        check(!gate.update(f, true), "remaining copies never repeat");
    f.operation = 0;
    for (int i = 0; i < 10; ++i)
        gate.update(f, true);
    f.operation = 0x20;
    for (int i = 0; i < 4; ++i)
        gate.update(f, true);
    check(!gate.update(f, false), "gap revokes operation arm");
    CommandSender sender;
    check(!sender.request(0x10), "sender startup neutral");
    for (int i = 0; i < 10; ++i)
        check(sender.next() == 0, "neutral train");
    check(sender.request(0x10), "accept operation");
    for (int i = 0; i < 10; ++i)
        check(sender.next() == 0x10, "exactly ten command frames");
    check(sender.next() == 0 && !sender.request(0x10), "neutral between operations");
    DualReceiver dual(50000, 2, 100);
    uint64_t t = 1000000;
    uint8_t seq = 250;
    auto tick = [&](uint8_t op, bool a = true, bool b = true, int16_t roll = 0) {
        t += 12500;
        Downlink f{seq++, op, roll, 0};
        if (a)
            dual.receive(0, f, t);
        if (b)
            dual.receive(1, f, t);
        return dual.update(t);
    };
    for (int i = 0; i < 15; ++i)
        tick(0);
    check(dual.valid(t) && dual.selected() == 0, "main selected across counter wrap");
    int events = 0;
    for (int i = 0; i < 10; ++i)
        events += tick(0x30) != 0;
    check(events == 1, "dual copies are one operation");
    const auto sample = dual.sampleTime();
    dual.receive(0, dual.channels[0].frame, t + 1000);
    dual.update(t + 1000);
    check(dual.sampleTime() == sample, "duplicates do not renew input");
    for (int i = 0; i < 8; ++i)
        check(!tick(0x30, false, true), "failover does not replay operation");
    check(dual.selected() == 1 && dual.valid(t), "single loss selects backup");
    for (int i = 0; i < 10; ++i)
        tick(0);
    check(dual.selected() == 1, "recovered primary does not preempt");
    dual.update(t + 100000);
    check(!dual.valid(t + 100000), "both stale revoke input");
    DualReceiver burst(50000, 1, 100);
    t = 2000000;
    for (int i = 0; i < 20; ++i) {
        burst.receive(0, {uint8_t(i), 0, 0, 0}, t);
        burst.update(t);
        t += 12500;
    }
    for (int i = 20; i < 25; ++i)
        burst.receive(0, {uint8_t(i), 0x20, 0, 0}, t);
    check(!burst.update(t), "buffered five-frame burst cannot execute");
    DualReceiver conflict(50000, 1, 100);
    t = 3000000;
    conflict.receive(0, {1, 0, 0, 0}, t);
    conflict.receive(1, {1, 0, 500, 0}, t);
    conflict.update(t);
    check(conflict.blocked() && !conflict.valid(t), "same-counter disagreement blocks");
    auto params = nlohmann::json{{"source", "FLIGHT"}, {"operation_code", uint64_t(0x30)}};
    check(aviator::validFlightOperation(params, "start_control", "FLIGHT"),
          "FLIGHT operation accepted");
    check(!aviator::validFlightOperation(params, "grasp_wheel", "FLIGHT") &&
              !aviator::validFlightOperation(params, "start_control", "JOYSTICK"),
          "operation/source mismatch rejected");
    aviator::Message state;
    state.header.valid = true;
    state.header.sample_mono_us = t;
    state.body = {
        {"system",
         {{"state", "RELEASING"}, {"current_error_code", 28673}, {"last_error_code", 1203}}},
        {"wheel_measurement",
         {{"valid", true}, {"sample_mono_us", t}, {"angle", 0}, {"displacement", -.085}}},
        {"wheel_reference", {{"angle", .8}, {"displacement", 0}}}};
    up = Uplink{};
    populateTelemetry(up, state, t + 1000);
    check(up.state == 0x14 && up.fault == unknown_fault && up.last_fault == 1203 && up.roll == 0 &&
              up.pitch == 0 && up.rear == unknown,
          "real feedback and unknown mapping");
    up = Uplink{};
    populateTelemetry(up, state, t + 100000);
    check(up.state == unknown && up.roll == invalid_position, "stale telemetry never refreshed");
    state.body.erase("wheel_measurement");
    up = Uplink{};
    populateTelemetry(up, state, t);
    check(up.roll == invalid_position, "reference is not measurement");
    std::cout << "RS422 codec, gate, redundancy, telemetry and Core parameters passed\n";
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}
