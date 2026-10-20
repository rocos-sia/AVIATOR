#include "flight_operation.hpp"
#include "rs422.hpp"
#include "rs422_runtime.hpp"
#include <algorithm>
#include <iostream>
#include <stdexcept>
using namespace flight_gateway;
using namespace flight_gateway::rs422;
void check(bool b, const char* reason) {
    if (!b)
        throw std::runtime_error(reason);
}
void checkWireBoundaries() {
    auto require = [](bool ok) {
        if (!ok)
            throw std::runtime_error("32/64-byte wire boundary regression");
    };
    Downlink down;
    Uplink up;
    for (bool uplink : {false, true}) {
        auto frame = uplink ? encode(Uplink{}) : encode(Downlink{});
        const size_t length = uplink ? 64 : 32;
        const size_t reserved = uplink ? 26 : 8;
        auto valid = [&](const Bytes& b) { return uplink ? decode(b, up) : decode(b, down); };
        require(frame.size() == length);
        require(std::all_of(frame.begin() + reserved, frame.end() - 1,
                            [](uint8_t b) { return b == 0; }));
        require(valid(frame));
        // A same-length frame sealed by the former header-inclusive complement is rejected.
        auto old_checksum = frame;
        unsigned old_sum = 0;
        for (size_t i = 0; i < length - 1; ++i)
            old_sum += old_checksum[i];
        old_checksum.back() = uint8_t(0 - old_sum);
        require(!valid(old_checksum));
        auto bad_header = frame;
        bad_header[0] = 0;
        require(!valid(bad_header));
        // Every reserved byte is covered by the checksum, but has no business meaning.
        for (size_t i = reserved; i < length - 1; ++i) {
            auto changed = frame;
            changed[i] = 1;
            require(!valid(changed));
            changed.back() = checksum(changed.data() + 2, length - 3);
            require(valid(changed));
        }
        // Header/trailer patterns inside reserved data must not split the frame.
        frame[reserved] = 0xeb;
        frame[reserved + 1] = 0x90;
        frame[reserved + 2] = 0xed;
        frame[reserved + 3] = 0x03;
        frame.back() = checksum(frame.data() + 2, length - 3);
        require(valid(frame));
        Parser fragmented(uplink);
        require(fragmented.feed(frame.data(), length - 1).empty());
        require(fragmented.feed(frame.data() + length - 1, 1) == std::vector<Bytes>{frame});
        Bytes stream{0x90, 0xeb, 0x00};
        auto damaged = frame;
        damaged.back() ^= 1;
        stream.insert(stream.end(), damaged.begin(), damaged.end());
        stream.insert(stream.end(), frame.begin(), frame.end());
        stream.insert(stream.end(), frame.begin(), frame.end());
        Parser noisy(uplink);
        require(noisy.feed(stream.data(), stream.size()) == std::vector<Bytes>({frame, frame}));
        // Neither an old-format frame nor an appended legacy trailer is a new frame.
        Bytes legacy(frame.begin(), frame.begin() + reserved);
        unsigned legacy_sum = 0;
        for (const auto byte : legacy)
            legacy_sum += byte;
        legacy.push_back(uint8_t(0 - legacy_sum));
        if (uplink)
            legacy.insert(legacy.end(), {0xed, 0x03});
        require(!valid(legacy));
        auto truncated = frame;
        truncated.pop_back();
        require(!valid(truncated));
        auto extended = frame;
        extended.insert(extended.end(), {0xed, 0x03});
        require(!valid(extended));
    }
}
void checkRingParser() {
    auto require = [](bool ok) {
        if (!ok)
            throw std::runtime_error("ring parser regression");
    };
    for (bool uplink : {false, true}) {
        auto frame = uplink ? encode(Uplink{}) : encode(Downlink{});
        const size_t length = frame.size(), reserved = uplink ? 26 : 8;
        frame[2] = 17;
        frame.back() = checksum(frame.data() + 2, length - 3);
        // Exercise every physical start offset and every read boundary, including
        // a header split across the end of the ring and empty feed calls.
        for (size_t offset = 0; offset < uplink_size; ++offset) {
            for (size_t split = 0; split <= length; ++split) {
                Parser parser(uplink);
                Bytes noise(offset, 0x55);
                require(parser.feed(noise.data(), noise.size()).empty());
                require(parser.feed(nullptr, 0).empty());
                auto frames = parser.feed(frame.data(), split);
                const auto tail = parser.feed(frame.data() + split, length - split);
                frames.insert(frames.end(), tail.begin(), tail.end());
                require(frames == std::vector<Bytes>{frame});
                require(parser.rejected == offset);
            }
        }
        // One feed far larger than the ring must retain every complete frame
        // and preserve the trailing partial frame for the next call.
        Parser burst(uplink);
        Bytes stream(63, 0x55);
        std::vector<Bytes> expected;
        for (unsigned i = 0; i < 256; ++i) {
            auto next = frame;
            next[2] = uint8_t(i);
            next.back() = checksum(next.data() + 2, length - 3);
            expected.push_back(next);
            stream.insert(stream.end(), next.begin(), next.end());
        }
        stream.insert(stream.end(), frame.begin(), frame.end() - 1);
        require(burst.feed(stream.data(), stream.size()) == expected);
        require(burst.feed(frame.data() + length - 1, 1) == std::vector<Bytes>{frame});
        require(burst.rejected == 63);
        // Missing/inserted bytes and a bad checksum must resynchronize without
        // losing the following good frames, including across ring wrap.
        for (unsigned fault = 0; fault < 3; ++fault) {
            auto damaged = frame;
            if (fault == 0)
                damaged.erase(damaged.begin() + reserved);
            else if (fault == 1)
                damaged.insert(damaged.begin() + reserved, 0x55);
            else
                damaged.back() ^= 1;
            Bytes noisy(63, 0x55);
            noisy.insert(noisy.end(), damaged.begin(), damaged.end());
            noisy.insert(noisy.end(), frame.begin(), frame.end());
            noisy.insert(noisy.end(), frame.begin(), frame.end());
            for (size_t chunk : {size_t(1), size_t(7), noisy.size()}) {
                Parser parser(uplink);
                std::vector<Bytes> frames;
                for (size_t pos = 0; pos < noisy.size(); pos += chunk) {
                    const auto got = parser.feed(noisy.data() + pos,
                                                 std::min(chunk, noisy.size() - pos));
                    frames.insert(frames.end(), got.begin(), got.end());
                }
                require(frames == std::vector<Bytes>({frame, frame}));
                require(parser.rejected == 63 + damaged.size());
            }
        }
        // A false header immediately before the real header must not swallow it.
        Parser overlap(uplink);
        Bytes prefix(63, 0x55);
        prefix.push_back(0xeb);
        prefix.push_back(0x90);
        prefix.insert(prefix.end(), frame.begin(), frame.end());
        require(overlap.feed(prefix.data(), prefix.size()) == std::vector<Bytes>{frame});
        require(overlap.rejected == 65);
        // Reset a wrapped partial frame without resetting cumulative diagnostics.
        Parser reset(uplink), independent(uplink);
        Bytes noise(63, 0x55);
        require(reset.feed(noise.data(), noise.size()).empty());
        require(reset.feed(frame.data(), length - 1).empty());
        require(independent.feed(frame.data(), length).size() == 1);
        reset.clear();
        require(reset.rejected == 63);
        require(reset.feed(frame.data(), length) == std::vector<Bytes>{frame});
    }
}
int main() try {
    checkWireBoundaries();
    checkRingParser();
    check(stateCode("SAFE") == unknown && std::string(stateName(0x09)) == "UNKNOWN",
          "retired SAFE remains mapped");
    check(stateCode("ERROR") == 0x10 && stateCode("READY") == 0x03, "state codes renumbered");
    Bytes neutral(32, 0);
    neutral[0] = 0xeb;
    neutral[1] = 0x90;
    neutral[31] = 0x00;
    check(encode(Downlink{}) == neutral, "independent downlink vector");
    auto bytes = encode(Downlink{255, 0x30, -1000, 1000});
    Downlink d;
    check(bytes[4] == 0x18 && bytes[5] == 0xfc && bytes[6] == 0xe8 && bytes[7] == 3,
          "signed little endian endpoints");
    check(decode(bytes, d) && d.roll == -1000 && d.pitch == 1000, "decode endpoints");
    bytes[31] ^= 1;
    check(!decode(bytes, d), "bad checksum rejected");
    bytes = encode(Downlink{});
    bytes[3] = 0x70;
    bytes[31] = checksum(bytes.data() + 2, 29);
    check(!decode(bytes, d), "unknown operation rejected");
    Uplink up;
    up.version = 0x0201;
    up.checksum = 0xba78;
    up.selected = 0;
    up.communication = 0;
    up.state = 0;
    up.rear = up.brake = 0;
    up.fault = up.last_fault = 0;
    up.severity = 0;
    up.roll_echo = up.pitch_echo = up.roll = up.pitch = 0;
    bytes = encode(up);
    check(bytes.size() == 64 && bytes[63] == 0x35 && bytes[22] == 1 && bytes[23] == 2 &&
              bytes[24] == 0x78 && bytes[25] == 0xba,
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
    bad[31] ^= 2;
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
    uint64_t gate_time = 1000000;
    auto gate_tick = [&](const Downlink& frame, bool consecutive) {
        gate_time += period_us;
        return gate.update(frame, consecutive, gate_time);
    };
    Downlink f;
    f.operation = 0x20;
    for (int i = 0; i < 20; ++i)
        check(!gate_tick(f, true), "startup cannot execute held command");
    f.operation = 0;
    for (int i = 0; i < 10; ++i)
        gate_tick(f, true);
    f.operation = 0x20;
    for (int i = 0; i < 4; ++i)
        check(!gate_tick(f, true), "before fifth frame");
    check(gate_tick(f, true) == 0x20, "fifth frame dispatches");
    for (int i = 0; i < 40; ++i)
        check(!gate_tick(f, true), "remaining copies never repeat");
    f.operation = 0;
    for (int i = 0; i < 10; ++i)
        gate_tick(f, true);
    f.operation = 0x20;
    for (int i = 0; i < 4; ++i)
        gate_tick(f, true);
    check(!gate_tick(f, false), "gap revokes operation arm");
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
    for (unsigned batch_size : {2u, 3u, 4u}) {
        for (unsigned phase = 0; phase < batch_size; ++phase) {
            DualReceiver batched(batch_size == 4 ? 60000 : 50000, 10, 100);
            uint64_t clock = 5000000;
            uint8_t counter = 240;
            std::vector<Downlink> pending;
            std::vector<uint8_t> operations;
            auto send = [&](uint8_t op, bool immediate = false) {
                if (!immediate)
                    clock += period_us;
                pending.push_back({counter++, op, 125, -250});
                if (pending.size() != batch_size)
                    return;
                for (const auto& frame : pending) {
                    batched.receive(0, frame, clock);
                    batched.receive(1, frame, clock);
                }
                if (auto accepted = batched.update(clock))
                    operations.push_back(accepted);
                pending.clear();
            };
            for (unsigned i = 0; i < 40 + phase; ++i)
                send(0);
            for (uint8_t op : {0x10, 0x20, 0x30, 0x40, 0x50, 0x60}) {
                for (unsigned i = 0; i < 10; ++i)
                    send(op);
                for (unsigned i = 0; i < 10; ++i)
                    send(0);
            }
            for (unsigned i = 0; i < 12; ++i)
                send(0);
            check(operations == std::vector<uint8_t>({0x10, 0x20, 0x30, 0x40, 0x50, 0x60}),
                  "USB batches preserve all six operations once, including mixed boundaries and wrap");
            check(batched.valid(clock) && batched.target().roll == 125 &&
                      batched.target().pitch == -250 && batched.coalesced_batches > 0,
                  "batched input retains latest target");
            operations.clear();
            // Even bounded batches cannot turn an instantaneous backlog into confirmation.
            for (unsigned i = 0; i < 12; ++i)
                send(0x20, true);
            check(operations.empty(), "operation requires real elapsed confirmation time");
        }
    }
    DualReceiver conflict(50000, 1, 100);
    t = 3000000;
    conflict.receive(0, {1, 0, 0, 0}, t);
    conflict.receive(1, {1, 0, 500, 0}, t);
    conflict.update(t);
    check(conflict.blocked() && !conflict.valid(t), "same-counter disagreement blocks");
    DualReceiver mixed_conflict(50000, 1, 100);
    mixed_conflict.receive(0, {1, 0x20, 0, 0}, t);
    mixed_conflict.receive(1, {1, 0x10, 0, 0}, t);
    mixed_conflict.receive(0, {2, 0, 0, 0}, t);
    mixed_conflict.receive(1, {2, 0, 0, 0}, t);
    check(!mixed_conflict.update(t) && mixed_conflict.blocked(),
          "matching latest frames cannot hide an earlier conflict inside a batch");
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
