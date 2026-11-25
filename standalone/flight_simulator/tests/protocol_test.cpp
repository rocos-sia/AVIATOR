#include "rs422.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace flight_simulator::rs422;
void check(bool ok) {
    if (!ok)
        throw std::runtime_error("protocol assertion failed");
}
void checkPhysicalPositions() {
    // Fixed ICD vectors, independent of either production codec.
    struct Target { double roll, pitch; Bytes prefix; uint8_t sum; };
    const Target targets[] = {
        {-50, -80, {0xeb, 0x90, 0, 0, 0x01, 0x80, 0x01, 0x80}, 0x02},
        {50, 70, {0xeb, 0x90, 0, 0, 0xff, 0x7f, 0xff, 0x7f}, 0xfc},
        {0, -5, {0xeb, 0x90, 0, 0, 0, 0, 0, 0}, 0},
        {0, 0, {0xeb, 0x90, 0, 0, 0, 0, 0x88, 0x08}, 0x90},
    };
    for (const auto& target : targets) {
        auto expected = target.prefix;
        expected.resize(32, 0);
        expected.back() = target.sum;
        check(encode(Downlink{0, 0, encodeRoll(target.roll), encodePitch(target.pitch)}) == expected);
        Downlink decoded;
        check(decode(expected, decoded));
        check(std::abs(decodeRoll(decoded.roll) - target.roll) <= 25.0 / 32767);
        check(std::abs(decodePitch(decoded.pitch) - target.pitch) <= 37.5 / 32767);
    }
    // Unknown camera/FK axes remain independent; they must not become physical zero.
    Uplink up;
    up.roll = -32767;
    up.joint_pitch = 32767;
    Uplink decoded;
    check(decode(encode(up), decoded));
    check(decoded.roll == -32767 && decoded.pitch == invalid_position);
    check(decoded.joint_roll == invalid_position && decoded.joint_pitch == 32767);
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
    checkPhysicalPositions();
    checkWireBoundaries();
    checkRingParser();
    check(stateCode("SAFE") == unknown_state && std::string(stateName(0x09)) == "UNKNOWN");
    check(stateCode("EMERGENCY_STOP") == 0xff && std::string(stateName(0xff)) == "EMERGENCY_STOP");
    check(stateCode("UNKNOWN") == 0xfe && std::string(stateName(0xfe)) == "UNKNOWN");
    check(stateCode("ERROR") == 0x10 && stateCode("READY") == 0x03);
    Bytes neutral(32, 0);
    neutral[0] = 0xeb;
    neutral[1] = 0x90;
    neutral[6] = 0x88; neutral[7] = 0x08; // physical pitch zero = raw 2184
    neutral[31] = 0x90;
    check(encode(Downlink{}) == neutral);
    Bytes extreme{0xeb, 0x90, 0xff, 0x60, 0x18, 0xfc, 0xe8, 0x03};
    extreme.resize(32, 0);
    extreme[31] = 0x5e;
    check(encode(Downlink{255, 0x60, -1000, 1000}) == extreme);
    Downlink down;
    check(decode(extreme, down) && down.roll == -1000 && down.pitch == 1000 && down.counter == 255);
    auto bad = neutral;
    bad[31] ^= 1;
    check(!decode(bad, down));
    bad = neutral;
    bad[3] = 0x11;
    bad[31] = checksum(bad.data() + 2, 29);
    check(!decode(bad, down));
    bad = neutral;
    bad[4] = 0x00;
    bad[5] = 0x80;
    bad[31] = checksum(bad.data() + 2, 29);
    check(!decode(bad, down));
    Parser parser(false);
    Bytes noisy{0x12, 0xeb, 0x77};
    noisy.insert(noisy.end(), bad.begin(), bad.end());
    noisy.insert(noisy.end(), extreme.begin(), extreme.end());
    check(parser.feed(noisy.data(), noisy.size() - 1).empty());
    check(parser.feed(noisy.data() + noisy.size() - 1, 1) == std::vector<Bytes>{extreme});
    // Independently laid-out peer frame (known version/software checksum).
    Bytes up(64, 0);
    up[0] = 0xeb;
    up[1] = 0x90;
    up[10] = 7;
    up[11] = up[12] = 0xff;
    up[22] = 1;
    up[23] = 2;
    up[24] = 0xcd;
    up[25] = 0xab;
    up[63] = 0x80;
    Uplink reply;
    check(decode(up, reply));
    check(reply.version == 0x0201 && reply.checksum == 0xabcd && reply.state == 7);
    check(encode(reply) == up);
    Uplink unknown_reply;
    check(decode(encode(unknown_reply), reply));
    check(reply.state == 0xfe && reply.roll == invalid_position && reply.fault == unknown_fault);
    check(reply.selected == 0xff && reply.rear == 0xff && reply.brake == 0xff && reply.severity == 0xff);
    for (uint8_t state : {0xff, 0xfe, 0x00}) {
        auto frame = up;
        frame[10] = state;
        frame.back() = checksum(frame.data() + 2, frame.size() - 3);
        check(decode(frame, reply) == (state != 0));
        if (state != 0) check(reply.state == state && encode(reply) == frame);
    }
    up[63] ^= 1;
    check(!decode(up, reply));
    CommandSender sender;
    check(sender.request(0x10));
    check(!sender.request(0x20));
    for (int i = 0; i < 10; ++i)
        check(sender.next() == 0x10);
    check(sender.request(0x20));
    for (int i = 0; i < 10; ++i)
        check(sender.next() == 0x20);
    check(sender.request(0x20));
    check(sender.next() == 0);
    for (int i = 0; i < 10; ++i)
        check(sender.next() == 0x20);
    check(sender.next() == 0);
    check(sender.request(0x20));
    check(sender.next() == 0x20);
    sender.reset();
    check(sender.next() == 0);
    check(sender.request(0x10));
    check(encodeRoll(25) == 16384 && encodePitch(-80) == -32767 && encodePitch(70) == 32767 && encodePitch(0) == 2184);
    std::cout << "PASS: independent wire vectors, validation, resync, command timing\n";
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}
