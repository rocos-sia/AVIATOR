#include "system.hpp"
#include <algorithm>
#include <fstream>
#include <sstream>

namespace monitor {
Json SystemStats::snapshot(std::uint64_t now) {
    if (!cached_.is_null() && now >= sampled_at_ && now - sampled_at_ < 1000000)
        return cached_;
    const double seconds =
        !cached_.is_null() && now > sampled_at_ ? (now - sampled_at_) / 1000000.0 : 0;
    Json result{
        {"sample_mono_us", now},
        {"uptime_seconds", nullptr},
        {"program_uptime_seconds", now >= started_us_ ? (now - started_us_) / 1000000.0 : 0.0},
        {"cpu", {{"percent", nullptr}, {"cores", Json::array()}}},
        {"memory", nullptr},
        {"swap", nullptr},
        {"disk", nullptr},
        {"network", nullptr}};
    double uptime;
    if (std::ifstream(proc_ / "uptime") >> uptime && uptime >= 0)
        result["uptime_seconds"] = uptime;

    Counters cpu;
    std::ifstream stat(proc_ / "stat");
    std::string line, name;
    while (std::getline(stat, line)) {
        std::istringstream row(line);
        row >> name;
        if (name.rfind("cpu", 0) != 0)
            continue;
        std::array<std::uint64_t, 8> ticks{};
        bool valid = true;
        for (auto& tick : ticks)
            if (!(row >> tick))
                valid = false;
        if (!valid)
            continue;
        std::uint64_t total = 0;
        for (auto tick : ticks)
            total += tick; // guest times are already included in user/nice.
        const auto idle = ticks[3] + ticks[4];
        cpu[name] = {total, idle};
        Json percent = nullptr;
        auto old = cpu_.find(name);
        if (seconds > 0 && old != cpu_.end() && total > old->second[0] && idle >= old->second[1])
            percent = std::clamp(
                100.0 * (1.0 - double(idle - old->second[1]) / double(total - old->second[0])), 0.0,
                100.0);
        if (name == "cpu")
            result["cpu"]["percent"] = percent;
        else
            result["cpu"]["cores"].push_back({{"name", name}, {"percent", percent}});
    }
    cpu_ = std::move(cpu);

    std::map<std::string, std::uint64_t> memory;
    std::ifstream meminfo(proc_ / "meminfo");
    while (std::getline(meminfo, line)) {
        std::istringstream row(line);
        std::uint64_t kb;
        if (row >> name >> kb)
            memory[name] = kb * 1024;
    }
    for (const auto& fields : {std::array<const char*, 3>{"memory", "MemTotal:", "MemAvailable:"},
                               std::array<const char*, 3>{"swap", "SwapTotal:", "SwapFree:"}}) {
        if (!memory.count(fields[1]) || !memory.count(fields[2]))
            continue;
        const auto total = memory[fields[1]], used = total - std::min(total, memory[fields[2]]);
        result[fields[0]] = {{"total_bytes", total},
                             {"used_bytes", used},
                             {"percent", total ? 100.0 * used / total : 0.0}};
    }

    // Diff each device separately: hotplug/reset must not create unsigned spikes.
    auto rates = [&](const Counters& current, const Counters& previous, const char* first,
                     const char* second) -> Json {
        if (current.empty())
            return nullptr;
        Json rate{{first, nullptr}, {second, nullptr}};
        std::array<double, 2> sum{};
        bool valid = seconds > 0;
        for (const auto& [device, values] : current) {
            auto old = previous.find(device);
            if (old == previous.end()) {
                valid = false;
                continue;
            }
            for (unsigned i = 0; i < 2; ++i) {
                if (values[i] < old->second[i])
                    valid = false;
                else if (seconds > 0)
                    sum[i] += (values[i] - old->second[i]) / seconds;
            }
        }
        if (valid) {
            rate[first] = sum[0];
            rate[second] = sum[1];
        }
        return rate;
    };
    Counters disks;
    std::ifstream diskstats(proc_ / "diskstats");
    while (std::getline(diskstats, line)) {
        std::istringstream row(line);
        unsigned major, minor;
        std::uint64_t reads, merged, read_sectors, read_ms, writes, write_merged, write_sectors;
        if (!(row >> major >> minor >> name >> reads >> merged >> read_sectors >> read_ms >>
              writes >> write_merged >> write_sectors))
            continue;
        // Whole hardware devices only: skip partitions and stacked dm/md/loop devices.
        std::error_code error;
        if (std::filesystem::exists(sys_ / "block" / name / "device", error))
            disks[name] = {read_sectors * 512, write_sectors * 512};
    }
    result["disk"] = rates(disks, disks_, "read_bytes_per_sec", "write_bytes_per_sec");
    disks_ = std::move(disks);

    Counters network;
    std::ifstream netdev(proc_ / "net/dev");
    while (std::getline(netdev, line)) {
        const auto colon = line.find(':');
        if (colon == std::string::npos)
            continue;
        std::istringstream label(line.substr(0, colon)), row(line.substr(colon + 1));
        label >> name;
        if (name == "lo")
            continue;
        std::uint64_t received, sent, ignored;
        if (!(row >> received))
            continue;
        for (int i = 0; i < 7; ++i)
            row >> ignored;
        if (row >> sent)
            network[name] = {received, sent};
    }
    result["network"] = rates(network, network_, "receive_bytes_per_sec", "send_bytes_per_sec");
    network_ = std::move(network);
    sampled_at_ = now;
    cached_ = std::move(result);
    return cached_;
}
} // namespace monitor
