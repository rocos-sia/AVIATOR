#include "rs422.hpp"
#include <iostream>
#include <stdexcept>
using namespace flight_simulator::rs422;
void check(bool ok) {
    if (!ok)
        throw std::runtime_error("protocol assertion failed");
}
int main() try {
    check(stateCode("SAFE") == unknown && std::string(stateName(0x09)) == "UNKNOWN");
    check(stateCode("ERROR") == 0x10 && stateCode("READY") == 0x03);
    const Bytes neutral{0xeb, 0x90, 0, 0, 0, 0, 0, 0, 0x85};
    check(encode(Downlink{}) == neutral);
    const Bytes extreme{0xeb, 0x90, 0xff, 0x60, 0x18, 0xfc, 0xe8, 0x03, 0x27};
    check(encode(Downlink{255, 0x60, -1000, 1000}) == extreme);
    Downlink down;
    check(decode(extreme, down) && down.roll == -1000 && down.pitch == 1000 && down.counter == 255);
    auto bad = neutral;
    bad[8] ^= 1;
    check(!decode(bad, down));
    bad = neutral;
    bad[3] = 0x11;
    bad[8] = checksum(bad.data(), 8);
    check(!decode(bad, down));
    bad = neutral;
    bad[4] = 0xe9;
    bad[5] = 3;
    bad[8] = checksum(bad.data(), 8);
    check(!decode(bad, down));
    Parser parser(false);
    Bytes noisy{0x12, 0xeb, 0x77};
    noisy.insert(noisy.end(), bad.begin(), bad.end());
    noisy.insert(noisy.end(), extreme.begin(), extreme.end());
    check(parser.feed(noisy.data(), noisy.size() - 1).empty());
    check(parser.feed(noisy.data() + noisy.size() - 1, 1) == std::vector<Bytes>{extreme});
    // Independently laid-out peer frame (known version/software checksum).
    Bytes up(29, 0);
    up[0] = 0xeb;
    up[1] = 0x90;
    up[10] = 7;
    up[11] = up[12] = 0xff;
    up[22] = 3;
    up[23] = 0x10;
    up[24] = 0xcd;
    up[25] = 0xab;
    up[26] = 0xf5;
    up[27] = 0xed;
    up[28] = 3;
    Uplink reply;
    check(decode(up, reply));
    check(reply.version == 0x1003 && reply.checksum == 0xabcd && reply.state == 7);
    check(encode(reply) == up);
    Uplink unknown_reply;
    check(decode(encode(unknown_reply), reply));
    check(reply.roll == invalid_position && reply.fault == unknown_fault);
    up[28] = 4;
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
