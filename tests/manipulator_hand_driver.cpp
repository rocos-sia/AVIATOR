#include "HandRuntime.hpp"
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>
using namespace aviator::hand;
volatile std::sig_atomic_t interrupted = 0;
void stop(int) { interrupted = 1; }
int main(int argc, char** argv) try {
    std::string pub = "tcp://127.0.0.1:5555", sub = "tcp://127.0.0.1:5556";
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (i + 1 >= argc)
            throw std::runtime_error("missing endpoint");
        if (a == "--endpoint")
            sub = argv[++i];
        else if (a == "--state-endpoint")
            pub = argv[++i];
        else
            throw std::runtime_error("bad option");
    }
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    Config cfg;
    cfg.fake = true;
    Runtime runtime(cfg, {fakeDevice(), fakeDevice()});
    Bus bus(runtime, pub, sub);
    while (!interrupted)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}
