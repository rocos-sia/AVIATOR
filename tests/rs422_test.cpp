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
void checkPhotoelectricTelemetry() {
    constexpr uint64_t t = 1000000;
    PhotoelectricTelemetry input("clock");
    Uplink up;
    input.populate(up, t);
    check(up.brake == 0xff, "no X1 sample must not claim untriggered");
    aviator::Message m;
    m.topic = aviator::Topic::photoelectric_state;
    m.header = {"1.0", 1, t, t, "clock", "aviator_photoelectric", "sensor", true};
    m.body = {{"channel", "X1"}, {"value", 1}};
    check(input.accept(m, t), "X1 high accepted");
    input.populate(up, t);
    check(encode(up)[12] == 0x00 && up.rear == 0xff, "X1=1 means untriggered at offset12 only");
    auto wrong = m;
    wrong.header.publisher_id = "untrusted";
    check(!input.accept(wrong, t), "wrong X1 source rejected");
    wrong = m; wrong.body["channel"] = "X2";
    check(!input.accept(wrong, t), "wrong input channel rejected");
    input.populate(up, t);
    check(up.brake == 0, "unrelated source must not replace X1");
    ++m.header.sequence; // Re-published unchanged sample must retain its original age.
    check(input.accept(m, t + 50000), "unchanged X1 sample allowed");
    input.populate(up, t + 99999);
    check(up.brake == 0, "X1 fresh below timeout");
    input.populate(up, t + 100000);
    check(up.brake == 0xff, "republication cannot extend X1 sample lifetime");
    ++m.header.sequence; m.header.sample_mono_us = t + 100001; m.body["value"] = 0;
    check(input.accept(m, t + 100001), "X1 low accepted");
    input.populate(up, t + 100001);
    check(encode(up)[12] == 0x01, "X1=0 means triggered");
    // Invalid notification with an old timestamp still revokes the displayed measurement.
    ++m.header.sequence; m.header.valid = false;
    check(!input.accept(m, t + 100002), "invalid X1 notification rejected");
    input.populate(up, t + 100002);
    check(up.brake == 0xff, "invalid X1 clears old trigger measurement");
    m.header.valid = true;
    for (const auto& value : {nlohmann::json(true), nlohmann::json(2), nlohmann::json("0"), nlohmann::json(0.0)}) {
        ++m.header.sequence; ++m.header.sample_mono_us; m.body["value"] = value;
        check(!input.accept(m, m.header.sample_mono_us), "X1 requires integer zero or one");
        input.populate(up, m.header.sample_mono_us);
        check(up.brake == 0xff, "malformed X1 must not fabricate signal");
    }
    ++m.header.sequence; ++m.header.sample_mono_us; m.body["value"] = 1;
    check(input.accept(m, m.header.sample_mono_us), "X1 recovery accepted");
    const auto now = m.header.sample_mono_us;
    aviator::Message state;
    state.header.valid = true; state.header.sample_mono_us = now;
    state.body = {{"system", {{"state", "EMERGENCY_STOP"}, {"current_error_code", 0}, {"last_error_code", 0}}},
                  {"discrete_inputs", {{"valid", true}, {"sample_mono_us", now}, {"emergency_brake", true}}}};
    populateTelemetry(up, state, now);
    input.populate(up, now);
    check(up.state == 0xff && up.brake == 0, "live X1 high must not clear Core emergency or use legacy brake flag");
    up = Uplink{}; state.header.valid = false;
    populateTelemetry(up, state, now);
    input.populate(up, now);
    check(up.brake == 0, "X1 measurement must be independent of Core availability");
    ++m.header.sequence; m.header.clock_id = "foreign";
    check(!input.accept(m, now), "X1 clock mismatch rejected");
    input.populate(up, now); check(up.brake == 0xff, "wrong clock does not preserve old signal");
    m.header.clock_id = "clock"; ++m.header.sequence; m.header.sample_mono_us = now + 1;
    check(!input.accept(m, now), "future X1 sample rejected");
    m.header.sample_mono_us = now - 1;
    check(!input.accept(m, now), "regressing X1 sample rejected");
    ++m.header.sequence; m.header.sample_mono_us = now + 2;
    check(input.accept(m, now + 2), "fresh X1 recovers after rejected evidence");
    check(!input.accept(m, now + 3), "duplicate X1 sequence rejected");
    input.populate(up, now + 3); check(up.brake == 0xff, "rejected observation cannot renew X1");
    PhotoelectricTelemetry configured("clock", "custom_sensor");
    check(!configured.accept(m, now + 3), "configured publisher enforced");
    m.header.publisher_id = "custom_sensor";
    check(configured.accept(m, now + 3), "custom X1 publisher accepted");
}
void checkDualFeedback() {
    constexpr uint64_t t = 1000000;
    constexpr double radians = 3.14159265358979323846 / 180;
    aviator::Message camera;
    camera.topic = aviator::Topic::camera_detection;
    camera.header = {"1.0", 1, t, t, "clock", "camera", "session", true};
    camera.body = {{"camera_id", "cockpit"}, {"steering_wheel", {
        {"valid", true}, {"theta_rad", 50 * radians}, {"translation_along_axis_m", -.080}}}};
    CameraTelemetry vision("clock");
    check(vision.accept(camera, t), "camera sample accepted");
    aviator::Message state;
    state.header.valid = true;
    state.header.sample_mono_us = t;
    state.body = {{"system", {{"state", "CONTROL"}, {"current_error_code", 0}, {"last_error_code", 0}}},
        {"software_lock", true}, {"wheel_joint_measurement", {{"source", "joint_fk"},
         {"valid", true}, {"sample_mono_us", t}, {"angle", 50 * radians}, {"displacement", -.150}}}};
    Uplink up;
    populateTelemetry(up, state, t);
    vision.populate(up, t);
    check(up.roll == -32767 && up.pitch == -32767 && up.joint_roll == 32767 && up.joint_pitch == 32767,
          "independent camera/FK signs and expanded physical ranges");
    auto bytes = encode(up);
    check(bytes[13] == 0x01 && bytes[14] == 0x80 && bytes[15] == 0x01 && bytes[16] == 0x80 &&
          bytes[26] == 0xff && bytes[27] == 0x7f && bytes[28] == 0xff && bytes[29] == 0x7f,
          "camera and FK feedback exact little-endian offsets");
    Uplink decoded;
    check(decode(bytes, decoded) && decoded.joint_roll == 32767 && decoded.joint_pitch == 32767,
          "expanded FK endpoints decode without saturation");
    for (size_t offset : {13, 15, 26, 28}) {
        auto bad = bytes;
        bad[offset] = 0xff; bad[offset + 1] = 0x7f;
        bad.back() = checksum(bad.data() + 2, bad.size() - 3);
        check(decode(bad, decoded), "full scale positive feedback is valid");
    }
    vision.populate(up, t + 100000);
    check(up.roll == invalid_position && up.joint_roll == 32767, "camera expiry does not overwrite FK");
    camera.header.sequence++; camera.header.sample_mono_us++;
    camera.body["steering_wheel"]["valid"] = false;
    check(!vision.accept(camera, t + 1), "target lost revokes visual feedback");
    vision.populate(up, t + 1);
    check(up.roll == invalid_position && up.pitch == invalid_position, "no old visual sample after target loss");
    camera.header.sequence++; camera.header.sample_mono_us++;
    camera.body["steering_wheel"]["valid"] = true;
    camera.body["steering_wheel"]["theta_rad"] = -49 * radians;
    camera.body["steering_wheel"]["translation_along_axis_m"] = .069;
    check(vision.accept(camera, t + 2), "visual recovery");
    vision.populate(up, t + 2);
    state.body["wheel_joint_measurement"]["valid"] = false;
    populateTelemetry(up, state, t + 2);
    check(up.roll == encodeRoll(49) && up.pitch == encodePitch(69) && up.joint_roll == invalid_position,
          "invalid FK leaves fresh expanded camera measurement intact");
    state.body["wheel_joint_measurement"]["valid"] = true;
    state.body["software_lock"] = false;
    populateTelemetry(up, state, t + 2);
    check(up.joint_roll == invalid_position, "unlocked arms cannot report wheel pose");
    state.body["software_lock"] = true;
    state.body["wheel_joint_measurement"]["source"] = "wheel_reference";
    populateTelemetry(up, state, t + 2);
    check(up.joint_roll == invalid_position, "reference cannot masquerade as joint FK");
    camera.header.sequence++; camera.header.sample_mono_us++;
    camera.body["steering_wheel"]["theta_rad"] = 53 * radians;
    camera.body["steering_wheel"]["translation_along_axis_m"] = -.091;
    check(!vision.accept(camera, t + 3), "out-of-physical-range visual measurement invalid");
    vision.populate(up, t + 3);
    check(up.roll == invalid_position && up.pitch == invalid_position, "outliers are not clipped");
    camera.header.sequence++; camera.header.sample_mono_us = t - 1;
    camera.body["steering_wheel"]["theta_rad"] = 0;
    camera.body["steering_wheel"]["translation_along_axis_m"] = 0;
    check(!vision.accept(camera, t + 4), "regressing sample cannot revive feedback");
    camera.header.sample_mono_us = t + 5;
    camera.header.clock_id = "foreign";
    check(!vision.accept(camera, t + 5), "camera clock mismatch rejected");
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
        const size_t reserved = uplink ? 30 : 8;
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
void checkPhysicalEncoding() {
    check(encodeRoll(-50) == -32767 && encodeRoll(50) == 32767, "ICD roll endpoints");
    check(encodePitch(-80) == -32767 && encodePitch(70) == 32767, "ICD pitch endpoints");
    check(decodePitch(0) == -5 && encodePitch(0) == 2184, "physical zero differs from raw zero");
    for (int raw = -32767; raw <= 32767; ++raw) {
        check(encodeRoll(decodeRoll(raw)) == raw && encodePitch(decodePitch(raw)) == raw,
              "all valid wire positions survive physical-unit roundtrip");
    }
    auto bytes = encode(Downlink{});
    bytes[4] = 0; bytes[5] = 0x80;
    bytes.back() = checksum(bytes.data()+2, bytes.size()-3);
    Downlink ignored;
    check(!decode(bytes, ignored), "invalid position cannot become a command");
    for (const auto limits : std::array<std::array<double, 3>, 4>{{
            {4, 0, 0}, {6, 0, 1}, {0, 7, 0}, {0, 8, 1}}}) {
        DualReceiver receiver(50000, 1, 5, 7.5);
        receiver.receive(0, Downlink{1, 0, 0, encodePitch(0)}, 1000000);
        receiver.update(1000000);
        check(receiver.valid(1000000), "initial channel admitted");
        receiver.receive(1, Downlink{2, 0, encodeRoll(limits[0]), encodePitch(limits[1])}, 1050001);
        receiver.update(1050001);
        check(receiver.blocked() == bool(limits[2]), "failover compares deg and mm independently");
    }
}
void checkIcdPolarity() {
    constexpr uint64_t t = 1000000;
    constexpr double radians = 3.14159265358979323846 / 180;
    // Fixed physical poses: left/push positive, right/pull negative, then center.
    for (const auto& pose : std::array<std::array<double, 4>, 5>{{
             {50, -.080, 32767, 2184}, {-50, -.080, -32767, 2184},
             {0, -.150, 0, 32767}, {0, 0, 0, -32767}, {0, -.080, 0, 2184}}}) {
        const double angle = pose[0] * radians, displacement = pose[1];
        aviator::Message camera;
        camera.topic = aviator::Topic::camera_detection;
        camera.header = {"1.0", 1, t, t, "clock", "camera", "session", true};
        camera.body = {{"camera_id", "cockpit"}, {"steering_wheel", {
            {"valid", true}, {"theta_rad", -angle},
            {"translation_along_axis_m", -displacement - .080}}}};
        aviator::Message state;
        state.header.valid = true;
        state.header.sample_mono_us = t;
        state.body = {{"system", {{"state", "CONTROL"}, {"current_error_code", 0}, {"last_error_code", 0}}},
            {"software_lock", true}, {"wheel_joint_measurement", {{"source", "joint_fk"},
             {"valid", true}, {"sample_mono_us", t}, {"angle", angle}, {"displacement", displacement}}}};
        const auto original_camera = camera.body, original_state = state.body;
        CameraTelemetry vision("clock");
        check(vision.accept(camera, t), "ICD pose camera sample accepted");
        Uplink up, decoded;
        populateTelemetry(up, state, t);
        vision.populate(up, t);
        check(decode(encode(up), decoded) && decoded.roll == pose[2] && decoded.pitch == pose[3] &&
                  decoded.joint_roll == pose[2] && decoded.joint_pitch == pose[3],
              "camera and FK share ICD left/push polarity at physical endpoints and center");
        check(camera.body == original_camera && state.body == original_state,
              "wire conversion leaves calibration and Core measurements unchanged");
    }
}
void checkRingParser() {
    auto require = [](bool ok) {
        if (!ok)
            throw std::runtime_error("ring parser regression");
    };
    for (bool uplink : {false, true}) {
        auto frame = uplink ? encode(Uplink{}) : encode(Downlink{});
        const size_t length = frame.size(), reserved = uplink ? 30 : 8;
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
    checkPhysicalEncoding();
    checkIcdPolarity();
    checkWireBoundaries();
    checkRingParser();
    check(stateCode("SAFE") == unknown_state && std::string(stateName(0x09)) == "UNKNOWN",
          "retired SAFE remains mapped");
    check(stateCode("EMERGENCY_STOP") == 0xff && std::string(stateName(0xff)) == "EMERGENCY_STOP",
          "ICD emergency state is 0xff");
    check(stateCode("UNKNOWN") == 0xfe && std::string(stateName(0xfe)) == "UNKNOWN",
          "unknown state is distinct from emergency");
    check(stateCode("ERROR") == 0x10 && stateCode("READY") == 0x03, "state codes renumbered");
    Bytes neutral(32, 0);
    neutral[0] = 0xeb;
    neutral[1] = 0x90;
    neutral[6] = 0x88; neutral[7] = 0x08; // physical pitch zero = raw 2184
    neutral[31] = 0x90;
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
    up.state = 0xff;
    up.rear = up.brake = 0;
    up.fault = up.last_fault = 0;
    up.severity = 0;
    up.roll_echo = up.pitch_echo = up.roll = up.pitch = up.joint_roll = up.joint_pitch = 0;
    bytes = encode(up);
    check(bytes.size() == 64 && bytes[10] == 0xff && bytes[63] == 0x34 && bytes[22] == 1 && bytes[23] == 2 &&
              bytes[24] == 0x78 && bytes[25] == 0xba,
          "independent uplink vector");
    Uplink decoded;
    check(decode(bytes, decoded) && decoded.state == 0xff, "emergency uplink decode");
    bytes[10] = 0;
    bytes.back() = checksum(bytes.data() + 2, bytes.size() - 3);
    check(!decode(bytes, decoded), "obsolete emergency state 0x00 rejected");
    up = Uplink{};
    check(decode(encode(up), decoded) && decoded.roll == invalid_position &&
              decoded.state == 0xfe && decoded.fault == unknown_fault && decoded.selected == 0xff &&
              decoded.rear == 0xff && decoded.brake == 0xff && decoded.severity == 0xff,
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
    for (int i = 0; i < 4; ++i)
        check(!gate_tick(f, true), "startup operation waits for five frames");
    check(gate_tick(f, true) == 0x20, "startup needs no neutral sequence");
    for (int i = 0; i < 40; ++i)
        check(!gate_tick(f, true), "remaining copies never repeat");
    check(!gate_tick(f, false), "counter gap does not replay dispatched command");
    gate.interrupt();
    for (int i = 0; i < 10; ++i)
        check(!gate_tick(f, true), "interruption preserves dispatched command latch");
    f.operation = 0x10;
    for (int i = 0; i < 4; ++i)
        check(!gate_tick(f, true), "different code starts a fresh confirmation");
    check(gate_tick(f, true) == 0x10, "different operation needs no intervening zero");
    f.operation = 0;
    check(!gate_tick(f, true), "one zero separates repeated operations");
    f.operation = 0x10;
    for (int i = 0; i < 4; ++i)
        check(!gate_tick(f, true), "repeat still needs five confirmed frames");
    check(gate_tick(f, true) == 0x10, "repeat after one zero dispatches once");
    f.operation = 0x20;
    for (int i = 0; i < 4; ++i)
        check(!gate_tick(f, true), "pending different operation");
    check(!gate_tick(f, false), "gap restarts confirmation");
    for (int i = 0; i < 3; ++i)
        check(!gate_tick(f, true), "confirmation after gap starts from first fresh frame");
    check(gate_tick(f, true) == 0x20, "five fresh frames after gap dispatch without zero");
    f.operation = 0x30;
    for (int i = 0; i < 10; ++i)
        check(!gate.update(f, true, gate_time), "instantaneous frames cannot confirm");
    check(gate.update(f, true, gate_time + 4 * period_us) == 0x30,
          "confirmation requires real elapsed time");
    CommandSender sender;
    check(sender.request(0x10), "sender accepts operation at startup");
    check(!sender.request(0x20), "active ten-frame train cannot be interrupted");
    for (int i = 0; i < 10; ++i)
        check(sender.next() == 0x10, "exactly ten command frames");
    check(sender.request(0x20), "different command starts immediately");
    for (int i = 0; i < 10; ++i)
        check(sender.next() == 0x20, "no forced zero between different commands");
    check(sender.request(0x20), "adjacent same-code click accepted");
    check(sender.next() == 0, "one zero separates adjacent same-code clicks");
    for (int i = 0; i < 10; ++i)
        check(sender.next() == 0x20, "same-code click still emits ten command frames");
    check(sender.next() == 0 && sender.request(0x20), "idle has no cooldown");
    check(sender.next() == 0x20, "already emitted idle needs no extra separator");
    sender.reset();
    check(sender.next() == 0 && sender.request(0x10), "reset cancels train without cooldown");
    DualReceiver dual(50000, 2, 5.0, 7.5);
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
    for (int i = 0; i < 10; ++i) {
        check(!tick(0x30, true, true, i), "held operation does not repeat as position changes");
        check(dual.target().roll == i, "position updates independently of operation latch");
    }
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
    DualReceiver startup(50000, 10, 5.0, 7.5);
    int startup_events = 0;
    for (unsigned i = 0; i < 40; ++i) {
        const auto clock = 1000000 + i * period_us;
        startup.receive(0, {uint8_t(i), 0x10, 0, 0}, clock);
        startup_events += startup.update(clock) == 0x10;
    }
    check(startup_events == 1, "startup confirms held operation after channel recovery without neutral");
    DualReceiver burst(50000, 1, 5.0, 7.5);
    t = 2000000;
    for (int i = 0; i < 20; ++i) {
        burst.receive(0, {uint8_t(i), 0, 0, 0}, t);
        burst.update(t);
        t += 12500;
    }
    for (int i = 20; i < 25; ++i)
        burst.receive(0, {uint8_t(i), 0x20, 0, 0}, t);
    check(!burst.update(t), "buffered five-frame burst cannot execute");
    DualReceiver interrupted_batch(50000, 1, 5.0, 7.5);
    t = 4000000;
    for (unsigned i = 0; i < 4; ++i) {
        interrupted_batch.receive(0, {uint8_t(i), 0x10, 0, 0}, t);
        check(!interrupted_batch.update(t), "partial confirmation before mixed batch");
        t += period_us;
    }
    interrupted_batch.receive(0, {4, 0x10, 0, 0}, t);
    interrupted_batch.receive(0, {6, 0x10, 0, 0}, t);
    check(!interrupted_batch.update(t), "later counter gap cancels confirmation within batch");
    for (unsigned i = 7; i <= 10; ++i) {
        t += period_us;
        interrupted_batch.receive(0, {uint8_t(i), 0x10, 0, 0}, t);
        check(interrupted_batch.update(t) == (i == 10 ? 0x10 : 0),
              "cancelled confirmation can complete after five fresh frames without idle");
    }
    for (unsigned batch_size : {2u, 3u, 4u}) {
        for (unsigned phase = 0; phase < batch_size; ++phase) {
            DualReceiver batched(batch_size == 4 ? 60000 : 50000, 10, 5.0, 7.5);
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
    DualReceiver conflict(50000, 1, 5.0, 7.5);
    t = 3000000;
    conflict.receive(0, {1, 0, 0, 0}, t);
    conflict.receive(1, {1, 0, 500, 0}, t);
    conflict.update(t);
    check(conflict.blocked() && !conflict.valid(t), "same-counter disagreement blocks");
    DualReceiver mixed_conflict(50000, 1, 5.0, 7.5);
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
        {"software_lock", true},
        {"wheel_joint_measurement",
         {{"source", "joint_fk"}, {"valid", true}, {"sample_mono_us", t}, {"angle", 0}, {"displacement", -.080}}},
        {"wheel_reference", {{"angle", .8}, {"displacement", 0}}}};
    up = Uplink{};
    populateTelemetry(up, state, t + 1000);
    check(up.state == 0x14 && up.fault == unknown_fault && up.last_fault == 1203 && up.joint_roll == 0 &&
              up.joint_pitch == 2184 && up.roll == invalid_position && up.pitch == invalid_position && up.rear == unknown,
          "real feedback and unknown mapping");
    up = Uplink{};
    state.body["system"]["state"] = "EMERGENCY_STOP";
    populateTelemetry(up, state, t + 1000);
    check(encode(up)[10] == 0xff, "fresh Core emergency reaches wire as 0xff");
    up = Uplink{};
    populateTelemetry(up, state, t + 100000);
    check(encode(up)[10] == 0xfe && up.roll == invalid_position, "stale Core emergency becomes unknown 0xfe");
    up = Uplink{};
    state.header.valid = false;
    populateTelemetry(up, state, t);
    check(encode(up)[10] == 0xfe, "invalid Core feedback remains unknown 0xfe");
    state.header.valid = true;
    state.body.erase("wheel_joint_measurement");
    up = Uplink{};
    populateTelemetry(up, state, t);
    check(up.roll == invalid_position && up.joint_roll == invalid_position, "reference is not measurement");
    checkDualFeedback();
    checkPhotoelectricTelemetry();
    std::cout << "RS422 codec, gate, redundancy, telemetry and Core parameters passed\n";
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}
