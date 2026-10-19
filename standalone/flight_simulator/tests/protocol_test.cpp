#include "rs422.hpp"
#include <algorithm>
#include <iostream>
#include <stdexcept>
using namespace flight_simulator::rs422;
void check(bool ok) {
    if (!ok)
        throw std::runtime_error("protocol assertion failed");
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
        // Every reserved byte is covered by the checksum, but has no business meaning.
        for (size_t i = reserved; i < length - 1; ++i) {
            auto changed = frame;
            changed[i] = 1;
            require(!valid(changed));
            changed.back() = checksum(changed.data(), length - 1);
            require(valid(changed));
        }
        // Header/trailer patterns inside reserved data must not split the frame.
        frame[reserved] = 0xeb;
        frame[reserved + 1] = 0x90;
        frame[reserved + 2] = 0xed;
        frame[reserved + 3] = 0x03;
        frame.back() = checksum(frame.data(), length - 1);
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
        legacy.push_back(checksum(legacy.data(), legacy.size()));
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
int main() try {
    checkWireBoundaries();
    check(stateCode("SAFE") == unknown && std::string(stateName(0x09)) == "UNKNOWN");
    check(stateCode("ERROR") == 0x10 && stateCode("READY") == 0x03);
    Bytes neutral(32, 0);
    neutral[0] = 0xeb;
    neutral[1] = 0x90;
    neutral[31] = 0x85;
    check(encode(Downlink{}) == neutral);
    Bytes extreme{0xeb, 0x90, 0xff, 0x60, 0x18, 0xfc, 0xe8, 0x03};
    extreme.resize(32, 0);
    extreme[31] = 0x27;
    check(encode(Downlink{255, 0x60, -1000, 1000}) == extreme);
    Downlink down;
    check(decode(extreme, down) && down.roll == -1000 && down.pitch == 1000 && down.counter == 255);
    auto bad = neutral;
    bad[31] ^= 1;
    check(!decode(bad, down));
    bad = neutral;
    bad[3] = 0x11;
    bad[31] = checksum(bad.data(), 31);
    check(!decode(bad, down));
    bad = neutral;
    bad[4] = 0xe9;
    bad[5] = 3;
    bad[31] = checksum(bad.data(), 31);
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
    up[22] = 3;
    up[23] = 0x10;
    up[24] = 0xcd;
    up[25] = 0xab;
    up[63] = 0xf5;
    Uplink reply;
    check(decode(up, reply));
    check(reply.version == 0x1003 && reply.checksum == 0xabcd && reply.state == 7);
    check(encode(reply) == up);
    Uplink unknown_reply;
    check(decode(encode(unknown_reply), reply));
    check(reply.roll == invalid_position && reply.fault == unknown_fault);
    up[63] ^= 1;
    check(!decode(up, reply));
    CommandSender sender;
    check(!sender.request(0x10));
    for (int i = 0; i < 10; ++i)
        check(sender.next() == 0);
    check(sender.request(0x10));
    check(!sender.request(0x20));
    for (int i = 0; i < 10; ++i)
        check(sender.next() == 0x10);
    check(!sender.request(0x20));
    for (int i = 0; i < 10; ++i)
        check(sender.next() == 0);
    check(sender.request(0x20));
    sender.reset();
    check(sender.next() == 0);
    check(quantize(.5) == 500 && quantize(-1) == -1000);
    std::cout << "PASS: independent wire vectors, validation, resync, command timing\n";
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}
