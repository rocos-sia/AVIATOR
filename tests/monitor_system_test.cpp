#include "system.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <unistd.h>

void check(bool ok, const char* message) {
    if (!ok)
        throw std::runtime_error(message);
}
int main() {
    char directory[] = "/tmp/aviator-monitor-system-XXXXXX";
    if (!mkdtemp(directory))
        return 1;
    const std::filesystem::path root(directory);
    try {
        std::filesystem::create_directories(root / "net");
        std::filesystem::create_directories(root / "block/sda/device");
        auto write = [&](const char* file, const std::string& data) {
            std::ofstream(root / file) << data;
        };
        auto io = [&](unsigned read, unsigned written, unsigned rx, unsigned tx) {
            write("diskstats",
                  "8 0 sda 1 0 " + std::to_string(read) + " 0 1 0 " + std::to_string(written) +
                      " 0\n8 1 sda1 1 0 9999 0 1 0 9999 0\n7 0 loop0 1 0 9999 0 1 0 9999 0\n");
            write("net/dev", "Inter-| Receive | Transmit\n eth0: " + std::to_string(rx) +
                                 " 0 0 0 0 0 0 0 " + std::to_string(tx) +
                                 " 0 0 0 0 0 0 0\n lo: 9999 0 0 0 0 0 0 0 9999\n");
        };
        write("uptime", "90061.25 100000\n");
        write("stat", "cpu 100 0 0 100 0 0 0 0 99 0\ncpu0 100 0 0 100 0 0 0 0 99 0\n");
        write("meminfo",
              "MemTotal: 1000 kB\nMemAvailable: 400 kB\nSwapTotal: 0 kB\nSwapFree: 0 kB\n");
        io(100, 200, 1000, 2000);
        monitor::SystemStats stats(root, root, 500000);
        auto first = stats.snapshot(1000000);
        check(first["program_uptime_seconds"] == 0.5, "program uptime starts before first request");
        check(first["uptime_seconds"] == 90061.25, "host uptime");
        check(first["cpu"]["percent"].is_null() && first["disk"]["read_bytes_per_sec"].is_null(),
              "first sample needs baseline");
        check(first["memory"]["percent"] == 60 && first["memory"]["used_bytes"] == 614400,
              "available memory includes reclaimable cache");
        check(first["swap"]["percent"] == 0, "disabled swap");
        write("stat", "cpu 150 0 0 250 0 0 0 0 120 0\ncpu0 150 0 0 250 0 0 0 0 120 0\n");
        io(104, 208, 3048, 6096);
        check(stats.snapshot(1500000) == first, "shared sampling cache");
        auto next = stats.snapshot(3000000);
        check(next["program_uptime_seconds"] == 2.5, "program uptime advances monotonically");
        check(next["cpu"]["percent"] == 25 && next["cpu"]["cores"][0]["percent"] == 25,
              "CPU delta excludes duplicate guest ticks");
        check(next["disk"]["read_bytes_per_sec"] == 1024 &&
                  next["disk"]["write_bytes_per_sec"] == 2048,
              "512-byte sectors, elapsed interval, no partition double count");
        check(next["network"]["receive_bytes_per_sec"] == 1024 &&
                  next["network"]["send_bytes_per_sec"] == 2048,
              "network rates exclude loopback");
        io(1, 1, 1, 1);
        auto reset = stats.snapshot(4000000);
        check(reset["disk"]["read_bytes_per_sec"].is_null() &&
                  reset["network"]["send_bytes_per_sec"].is_null(),
              "counter reset must not spike");
        auto idle = stats.snapshot(5000000);
        check(idle["disk"]["read_bytes_per_sec"] == 0 && idle["network"]["send_bytes_per_sec"] == 0,
              "idle is zero after reset baseline");
        std::filesystem::remove(root / "stat");
        std::filesystem::remove(root / "meminfo");
        std::filesystem::remove(root / "diskstats");
        std::filesystem::remove(root / "net/dev");
        auto missing = stats.snapshot(6000000);
        check(missing["cpu"]["percent"].is_null() && missing["memory"].is_null() &&
                  missing["disk"].is_null() && missing["network"].is_null(),
              "missing metrics are unavailable");
        io(9000, 9000, 9000, 9000);
        check(stats.snapshot(7000000)["disk"]["read_bytes_per_sec"].is_null(),
              "reappearing devices need a baseline");
        std::filesystem::remove_all(root);
        std::cout << "system metrics passed\n";
    } catch (const std::exception& error) {
        std::filesystem::remove_all(root);
        std::cerr << error.what() << '\n';
        return 1;
    }
}
