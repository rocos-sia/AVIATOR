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
        std::filesystem::create_directories(root / "class/drm/card0/device");
        std::filesystem::create_directories(root / "class/drm/card1/device");
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
        write("class/drm/card0/device/gpu_busy_percent", "42\n");
        monitor::SystemStats stats(root, root, 500000);
        auto first = stats.snapshot(1000000);
        check(first["gpu"]["percent"] == 42, "GPU busy percentage is available without a baseline");
        check(first["program_uptime_seconds"] == 0.5, "program uptime starts before first request");
        check(first["uptime_seconds"] == 90061.25, "host uptime");
        check(first["cpu"]["percent"].is_null() && first["disk"]["read_bytes_per_sec"].is_null(),
              "first sample needs baseline");
        check(first["memory"]["percent"] == 60 && first["memory"]["used_bytes"] == 614400,
              "available memory includes reclaimable cache");
        check(first["swap"]["percent"] == 0, "disabled swap");
        write("stat", "cpu 150 0 0 250 0 0 0 0 120 0\ncpu0 150 0 0 250 0 0 0 0 120 0\n");
        io(104, 208, 3048, 6096);
        write("class/drm/card0/device/gpu_busy_percent", "0\n");
        check(stats.snapshot(1500000) == first, "shared sampling cache");
        auto next = stats.snapshot(3000000);
        check(next["gpu"]["percent"] == 0, "idle GPU is a valid zero");
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
        write("class/drm/card0/device/gpu_busy_percent", "20\n");
        write("class/drm/card1/device/gpu_busy_percent", "60\n");
        std::filesystem::create_directory_symlink(root / "class/drm/card1",
                                                  root / "class/drm/renderD128");
        std::filesystem::create_directory_symlink(root / "class/drm/card1",
                                                  root / "class/drm/card1-HDMI-A-1");
        auto reset = stats.snapshot(4000000);
        check(reset["gpu"]["percent"] == 40,
              "average GPUs without counting render nodes/connectors twice");
        check(reset["disk"]["read_bytes_per_sec"].is_null() &&
                  reset["network"]["send_bytes_per_sec"].is_null(),
              "counter reset must not spike");
        write("class/drm/card0/device/gpu_busy_percent", "101\n");
        write("class/drm/card1/device/gpu_busy_percent", "N/A\n");
        auto idle = stats.snapshot(5000000);
        check(idle["gpu"]["percent"].is_null(), "invalid GPU metrics are unavailable");
        check(idle["disk"]["read_bytes_per_sec"] == 0 && idle["network"]["send_bytes_per_sec"] == 0,
              "idle is zero after reset baseline");
        std::filesystem::remove(root / "stat");
        std::filesystem::remove(root / "meminfo");
        std::filesystem::remove(root / "diskstats");
        std::filesystem::remove(root / "net/dev");
        std::filesystem::remove_all(root / "class/drm");
        auto missing = stats.snapshot(6000000);
        check(missing["cpu"]["percent"].is_null() && missing["memory"].is_null() &&
                  missing["disk"].is_null() && missing["network"].is_null() &&
                  missing["gpu"]["percent"].is_null(),
              "missing metrics are unavailable");
        io(9000, 9000, 9000, 9000);
        check(stats.snapshot(7000000)["disk"]["read_bytes_per_sec"].is_null(),
              "reappearing devices need a baseline");
        std::filesystem::create_directories(root / "devices/0000:00:02.0");
        std::filesystem::create_directories(root / "class/drm/card0");
        std::filesystem::create_directory_symlink(root / "devices/0000:00:02.0", root / "class/drm/card0/device");
        write("devices/0000:00:02.0/vendor", "0x8086\n");
        std::filesystem::create_directories(root / "101/fdinfo");
        std::filesystem::create_directories(root / "102/fdinfo");
        auto intel = [&](const char* file, const char* client, const std::string& counters,
                         const char* driver = "i915") {
            write(file, std::string("drm-driver: ") + driver +
                            "\ndrm-pdev: 0000:00:02.0\ndrm-client-id: " + client + "\n" + counters);
        };
        intel("101/fdinfo/3", "1",
              "drm-engine-render: 100000000 ns\ndrm-engine-video: 0 ns\ndrm-engine-capacity-video: "
              "2\n");
        intel("102/fdinfo/3", "2", "drm-engine-render: 100000000 ns\n");
        std::filesystem::create_symlink(root / "101/fdinfo/3", root / "101/fdinfo/4");
        std::filesystem::create_symlink(root / "101/fdinfo/3", root / "102/fdinfo/4");
        check(stats.snapshot(8000000)["gpu"]["percent"].is_null(), "Intel GPU needs a baseline");
        intel("101/fdinfo/3", "1",
              "drm-engine-render: 600000000 ns\ndrm-engine-video: 1600000000 "
              "ns\ndrm-engine-capacity-video: 2\n");
        intel("102/fdinfo/3", "2", "drm-engine-render: 300000000 ns\n");
        check(stats.snapshot(9000000)["gpu"]["percent"] == 80,
              "Intel uses busiest engine, normalizes capacity and deduplicates shared clients");
        check(stats.snapshot(10000000)["gpu"]["percent"] == 0, "idle Intel GPU is zero");
        std::filesystem::remove(root / "102/fdinfo/3");
        intel("101/fdinfo/3", "1",
              "drm-engine-render: 100000000 ns\ndrm-engine-video: 100000000 "
              "ns\ndrm-engine-capacity-video: 2\n");
        check(stats.snapshot(11000000)["gpu"]["percent"] == 0,
              "Intel counter regression does not spike");
        intel("101/fdinfo/3", "1",
              "drm-engine-render: 800000000 ns\ndrm-engine-video: 1800000000 "
              "ns\ndrm-engine-capacity-video: 2\n");
        check(stats.snapshot(12000000)["gpu"]["percent"] == 20,
              "Intel counter recovery uses high water mark");
        intel("101/fdinfo/3", "3",
              "drm-cycles-rcs: 100\ndrm-total-cycles-rcs: 1000\ndrm-engine-capacity-rcs: 2\n",
              "xe");
        check(stats.snapshot(13000000)["gpu"]["percent"].is_null(),
              "Xe GPU needs a cycle baseline");
        intel("101/fdinfo/3", "3",
              "drm-cycles-rcs: 900\ndrm-total-cycles-rcs: 2000\ndrm-engine-capacity-rcs: 2\n",
              "xe");
        check(stats.snapshot(14000000)["gpu"]["percent"] == 40,
              "Xe uses GPU clock and engine capacity");
        intel("101/fdinfo/3", "3",
              "drm-cycles-rcs: 900\ndrm-total-cycles-rcs: 3000\ndrm-engine-capacity-rcs: 0\n",
              "xe");
        check(stats.snapshot(15000000)["gpu"]["percent"].is_null(),
              "invalid engine capacity is unavailable");
        std::filesystem::remove(root / "101/fdinfo/3");
        check(stats.snapshot(16000000)["gpu"]["percent"].is_null(),
              "unreadable Intel counters are unavailable");
        std::filesystem::remove_all(root);
        std::cout << "system metrics passed\n";
    } catch (const std::exception& error) {
        std::filesystem::remove_all(root);
        std::cerr << error.what() << '\n';
        return 1;
    }
}
