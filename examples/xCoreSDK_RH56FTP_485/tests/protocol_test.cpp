#include "protocol.hpp"
#include <iostream>
#include <map>

namespace {
void require(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
template<class F> void rejects(F action) {
    bool rejected = false;
    try { action(); } catch (const std::exception&) { rejected = true; }
    require(rejected, "Expected rejection");
}
rh56::Bytes reply(const rh56::Bytes& tx, rh56::Bytes data) {
    rh56::Bytes rx{0x90,0xEB,tx[2],uint8_t(data.size()+3),tx[4],tx[5],tx[6]};
    for (auto value : data) rx.push_back(value);
    rx.push_back(rh56::checksum(rx)); return rx;
}
}
int main() {
    try {
        using namespace rh56;
        const Bytes expectedRead{0xEB,0x90,1,4,0x11,0x0A,6,12,0x32};
        require(request(1,0x11,angleAct,{12}) == expectedRead, "Factory six-angle read frame");
        const Bytes fullPayload{0xE8,3,0xE8,3,0xE8,3,0xE8,3,0xE8,3,0xE8,3};
        const Bytes expectedWrite{0xEB,0x90,1,15,0x12,0xCE,5,
            0xE8,3,0xE8,3,0xE8,3,0xE8,3,0xE8,3,0xE8,3,0x77};
        require(request(1,0x12,angleSet,fullPayload) == expectedWrite, "Factory six-angle write frame");
        const Bytes fullReply{0x90,0xEB,1,15,0x11,0x0A,6,
            0xE8,3,0xE8,3,0xE8,3,0xE8,3,0xE8,3,0xE8,3,0xB3};
        require(response(expectedRead,fullReply,12) == fullPayload, "Factory six-angle response");
        std::vector<Bytes> sent;
        std::map<int,uint8_t> memory;
        Hand hand(1,[&](const Bytes& tx, size_t rxLength) {
            require(tx.size() <= 64 && rxLength <= 64, "SDK payload limit");
            sent.push_back(tx);
            const int address = tx[5] | (tx[6] << 8);
            if (tx[4] == 0x12) {
                require(tx.size() == 20 && rxLength == 9, "Write transaction lengths");
                for (size_t j = 7; j < tx.size()-1; ++j) memory[address+int(j)-7] = tx[j];
                return reply(tx,{1});
            }
            require(tx.size() == 9 && rxLength == size_t(tx[7]) + 8, "Read transaction lengths");
            Bytes data;
            for (int j=0;j<tx[7];++j) data.push_back(memory[address+j]);
            return reply(tx,data);
        });
        const Values values{0,1,255,256,999,1000};
        hand.write(angleSet,values);
        require(sent.size()==1 && sent[0].size()==20, "Six-channel write must use one SDK transaction");
        require((sent[0][5] | (sent[0][6]<<8)) == angleSet, "Write register start address");
        require(hand.read(angleSet)==values, "Six-channel round trip");
        require(sent.size()==2 && sent[1].size()==9 && sent[1][7]==12,
                "Six-channel read must use one SDK transaction");
        require((sent[1][5] | (sent[1][6]<<8)) == angleSet, "Read register start address");
        const Values forces{-4000,-1,0,1,1234,4000};
        for (int j=0;j<6;++j) {
            auto v=uint16_t(forces[j]); memory[forceAct+2*j]=uint8_t(v); memory[forceAct+2*j+1]=uint8_t(v>>8);
        }
        require(hand.read(forceAct,true)==forces, "Signed force decoding");
        for (int j=0;j<6;++j) memory[errorAct+j]=uint8_t(j);
        require(hand.errors()==Values({0,1,2,3,4,5}), "Byte-wide error registers");
        auto valid = reply(expectedRead,{1,0,2,0,3,0,4,0,5,0,6,0});
        for (size_t offset : {0u,1u,2u,3u,4u,5u,6u,19u}) {
            auto bad=valid; bad[offset]^=1;
            if (offset!=19) { bad.pop_back(); bad.push_back(checksum(bad)); }
            rejects([&] { response(expectedRead,bad,12); });
        }
        auto corrupt=valid; corrupt[7]^=1;
        rejects([&] { response(expectedRead,corrupt,12); });
        valid.pop_back(); rejects([&] { response(expectedRead,valid,12); });
        rejects([&] { request(0,0x11,angleAct,{6}); });
        rejects([&] { request(1,0x11,angleAct,{13}); });
        rejects([&] { hand.write(angleSet,{-1,0,0,0,0,0}); });
        rejects([&] { hand.write(forceSet,{3001,0,0,0,0,0}); });
        rejects([&] { hand.write(speedSet,{1001,0,0,0,0,0}); });
        rejects([&] { hand.write(1005,values); }); // No accidental persistent writes.
        int calls=0;
        Hand failed(1,[&](const Bytes& tx,size_t) { ++calls; return reply(tx,{0}); });
        rejects([&] { failed.write(angleSet,values); });
        require(calls==1,"Rejected write must not retry");
        calls=0;
        Hand transportFailure(1,[&](const Bytes&,size_t) -> Bytes { ++calls; throw std::runtime_error("SDK failure"); });
        rejects([&] { transportFailure.read(angleAct); });
        require(calls==1,"Failed read must not retry");
        rejects([&] { request(1,0x12,angleSet,Bytes(13,0)); });
        std::cout << "Single-frame six-channel transfers, signed values and malformed responses passed\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
