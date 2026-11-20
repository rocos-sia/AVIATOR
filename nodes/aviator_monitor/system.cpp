#include "system.hpp"
#include <algorithm>
#include <cmath>
#include <charconv>
#include <ctime>
#include <dlfcn.h>
#include <fstream>
#include <optional>
#include <set>
#include <sstream>

namespace monitor {
namespace {
// Load the stable NVML ABI optionally: Monitor also runs without a CUDA SDK/driver.
class NvidiaStats {
    struct Device;
    struct Utilization {
        unsigned gpu, memory;
    };
    void* library_ = dlopen("libnvidia-ml.so.1", RTLD_LAZY | RTLD_LOCAL);
    using Init = int (*)();
    using Count = int (*)(unsigned*);
    using Handle = int (*)(unsigned, Device**);
    using Rates = int (*)(Device*, Utilization*);
    Init init_ = symbol<Init>("nvmlInit_v2"), shutdown_ = symbol<Init>("nvmlShutdown");
    Count count_ = symbol<Count>("nvmlDeviceGetCount_v2");
    Handle handle_ = symbol<Handle>("nvmlDeviceGetHandleByIndex_v2");
    Rates rates_ = symbol<Rates>("nvmlDeviceGetUtilizationRates");
    bool ready_ = false;
    template <typename T> T symbol(const char* name) {
        return library_ ? reinterpret_cast<T>(dlsym(library_, name)) : nullptr;
    }

  public:
    ~NvidiaStats() {
        if (ready_)
            shutdown_();
        if (library_)
            dlclose(library_);
    }
    void sample(double& total, unsigned& samples) {
        if (!init_ || !shutdown_ || !count_ || !handle_ || !rates_)
            return;
        if (!ready_)
            ready_ = init_() == 0;
        if (!ready_)
            return;
        unsigned count = 0;
        if (count_(&count) != 0)
            return;
        for (unsigned i = 0; i < count; ++i) {
            Device* device = nullptr;
            Utilization utilization{};
            if (handle_(i, &device) == 0 && rates_(device, &utilization) == 0 &&
                utilization.gpu <= 100) {
                total += utilization.gpu;
                ++samples;
            }
        }
    }
};
using GpuFields = std::map<std::string, std::string>;
GpuFields gpuFields(const std::filesystem::path& path) {
    std::ifstream file(path);
    GpuFields fields;
    std::string line;
    while (std::getline(file, line)) {
        if (line.compare(0, 4, "drm-") != 0) continue;
        const auto colon = line.find(':');
        if (colon == std::string::npos) continue;
        const auto key = line.substr(0, colon);
        if (key != "drm-driver" && key != "drm-pdev" && key != "drm-client-id" &&
            key.compare(0, 11, "drm-engine-") != 0 &&
            key.compare(0, 11, "drm-cycles-") != 0 &&
            key.compare(0, 17, "drm-total-cycles-") != 0) continue;
        const auto begin = line.find_first_not_of(" \t", colon + 1);
        if (begin == std::string::npos) continue;
        const auto end = line.find_first_of(" \t\r", begin);
        fields[key] = line.substr(begin, end == std::string::npos ? end : end - begin);
    }
    return fields;
}
std::optional<std::uint64_t> processStart(const std::filesystem::path& process) {
    std::ifstream file(process / "stat");
    std::string line, field;
    if (!std::getline(file, line)) return std::nullopt;
    // comm may contain spaces and ')'; the final ')' precedes field 3.
    const auto end = line.rfind(')');
    if (end == std::string::npos) return std::nullopt;
    std::istringstream row(line.substr(end + 1));
    for (int n = 3; n < 22; ++n) if (!(row >> field)) return std::nullopt;
    std::uint64_t start;
    if (!(row >> field)) return std::nullopt;
    const auto parsed = std::from_chars(field.data(), field.data() + field.size(), start);
    if (parsed.ec != std::errc{} || parsed.ptr != field.data() + field.size()) return std::nullopt;
    return start;
}
std::uint64_t threadCpuUs() {
    timespec time{};
    if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &time) != 0) return 0;
    return std::uint64_t(time.tv_sec) * 1000000 + time.tv_nsec / 1000;
}
} // namespace

Json SystemStats::intelGpuSample(const std::set<std::string>& devices, std::uint64_t now,
                                double seconds, double& total, unsigned& samples) {
    const auto started = aviator::monotonic_us(), cpu_started = threadCpuUs();
    const bool discover = !gpu_discovered_ || devices != gpu_devices_ ||
        now < gpu_discovered_at_ || now - gpu_discovered_at_ >= gpu_discovery_us_;
    struct ClientSample { GpuFields fields; bool continuous = false; };
    std::map<std::string, ClientSample> readings;
    std::map<std::filesystem::path, GpuClient> candidates;
    std::map<std::filesystem::path, std::optional<std::uint64_t>> starts;
    unsigned files_read = 0;
    auto read = [&](const std::filesystem::path& path) {
        const auto process = path.parent_path().parent_path();
        auto [it, inserted] = starts.try_emplace(process);
        if (inserted) it->second = processStart(process);
        if (!it->second) return;
        const auto previous = gpu_clients_.find(path);
        if (!discover && (previous == gpu_clients_.end() ||
                          previous->second.starttime != *it->second)) return;
        ++files_read;
        auto fields = gpuFields(path);
        const auto driver = fields.find("drm-driver"), device = fields.find("drm-pdev"),
                   client = fields.find("drm-client-id");
        if (driver == fields.end() || (driver->second != "i915" && driver->second != "xe") ||
            client == fields.end() || client->second.empty() || device == fields.end() ||
            !devices.count(device->second)) return;
        GpuClient candidate{*it->second, device->second, client->second};
        const bool continuous = previous != gpu_clients_.end() &&
            previous->second.starttime == candidate.starttime &&
            previous->second.device == candidate.device && previous->second.client == candidate.client;
        const auto id = candidate.device + "/" + candidate.client;
        candidates.emplace(path, std::move(candidate));
        auto [sample, fresh] = readings.try_emplace(id);
        if (fresh) sample->second.fields = std::move(fields);
        // dup/fork aliases share counters, but any surviving alias establishes continuity.
        sample->second.continuous |= continuous;
    };
    if (discover) {
        std::error_code error;
        std::filesystem::directory_iterator processes(proc_, error), end;
        for (; !error && processes != end; processes.increment(error)) {
            const auto pid = processes->path().filename().string();
            if (pid.empty() || pid.find_first_not_of("0123456789") != std::string::npos) continue;
            std::error_code fd_error;
            std::filesystem::directory_iterator files(processes->path() / "fdinfo", fd_error);
            for (; !fd_error && files != end; files.increment(fd_error)) read(files->path());
        }
        gpu_discovered_ = true;
        gpu_discovered_at_ = now;
        gpu_devices_ = devices;
    } else {
        for (const auto& [path, unused] : gpu_clients_) read(path);
    }
    gpu_clients_ = std::move(candidates);
    Counters current;
    std::map<std::string, std::map<std::string, double>> engines;
    for (const auto& [client, sample] : readings) {
        const auto& fields = sample.fields;
        const auto& device = fields.at("drm-pdev");
        auto number = [&](const std::string& field, std::uint64_t& result) {
            auto it = fields.find(field);
            if (it == fields.end() || it->second.empty() ||
                it->second.find_first_not_of("0123456789") != std::string::npos)
                return false;
            const auto& text = it->second;
            const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result);
            return parsed.ec == std::errc{} && parsed.ptr == text.data() + text.size();
        };
        for (const auto& [field, unused] : fields) {
            const bool cycles = field.rfind("drm-cycles-", 0) == 0;
            if (!cycles && (field.rfind("drm-engine-", 0) != 0 ||
                            field.rfind("drm-engine-capacity-", 0) == 0))
                continue;
            const auto engine = field.substr(11);
            // Prefer time counters when a driver exports both representations.
            if (cycles && fields.count("drm-engine-" + engine))
                continue;
            std::uint64_t busy = 0, ticks = 0, capacity = 1;
            if (!number(field, busy) ||
                (cycles && !number("drm-total-cycles-" + engine, ticks)))
                continue;
            if (fields.count("drm-engine-capacity-" + engine) &&
                (!number("drm-engine-capacity-" + engine, capacity) || !capacity))
                continue;
            const auto id = client + "/" + field + "/" + std::to_string(capacity);
            current[id] = {busy, ticks};
            const auto old = intel_gpu_.find(id);
            if (old == intel_gpu_.end() || !sample.continuous || seconds <= 0)
                continue;
            // DRM counters may briefly go backwards; retain the high water mark.
            if (busy < old->second[0]) {
                current[id][0] = old->second[0];
                busy = old->second[0];
            }
            if (cycles && ticks <= old->second[1])
                continue;
            const double elapsed = cycles ? double(ticks - old->second[1]) : seconds * 1e9;
            engines[device][engine] += 100.0 * (busy - old->second[0]) / elapsed / capacity;
        }
    }
    intel_gpu_ = std::move(current);
    for (const auto& [device, loads] : engines) {
        double busiest = 0;
        for (const auto& [engine, percent] : loads)
            busiest = std::max(busiest, percent);
        total += std::clamp(busiest, 0.0, 100.0);
        ++samples;
    }
    return {{"sample_mono_us", now}, {"discovery_mono_us", gpu_discovered_at_},
            {"discovery_interval_ms", gpu_discovery_us_ / 1000},
            {"discovery_age_ms", (now - gpu_discovered_at_) / 1000},
            {"full_scan", discover}, {"fdinfo_reads", files_read}, {"clients", readings.size()},
            {"scan_duration_us", aviator::monotonic_us() - started},
            {"scan_cpu_us", threadCpuUs() - cpu_started}};
}
Json SystemStats::gpuPercent(std::uint64_t now, double seconds) {
    double total = 0;
    unsigned samples = 0;
    // Keep alternate sysfs roots isolated from the host GPU, just like CPU/I/O fixtures.
    if (sys_ == "/sys") {
        static NvidiaStats nvidia;
        nvidia.sample(total, samples);
    }
    std::error_code error;
    std::set<std::string> intel_devices;
    std::filesystem::directory_iterator entries(sys_ / "class/drm", error), end;
    for (; !error && entries != end; entries.increment(error)) {
        const auto name = entries->path().filename().string();
        if (name.rfind("card", 0) != 0 || name.size() == 4 ||
            name.find_first_not_of("0123456789", 4) != std::string::npos)
            continue;
        double percent;
        if (std::ifstream(entries->path() / "device/gpu_busy_percent") >> percent &&
            std::isfinite(percent) && percent >= 0 && percent <= 100) {
            total += percent;
            ++samples;
        } else {
            unsigned vendor = 0;
            std::error_code path_error;
            if (std::ifstream(entries->path() / "device/vendor") >> std::hex >> vendor &&
                vendor == 0x8086) {
                const auto device =
                    std::filesystem::canonical(entries->path() / "device", path_error);
                if (!path_error)
                    intel_devices.insert(device.filename().string());
            }
        }
    }
    Json result{{"sample_mono_us", now}};
    if (!intel_devices.empty())
        result = intelGpuSample(intel_devices, now, seconds, total, samples);
    else {
        intel_gpu_.clear();
        gpu_clients_.clear();
        gpu_devices_.clear();
        gpu_discovered_ = false;
    }
    result["percent"] = samples ? Json(total / samples) : Json(nullptr);
    return result;
}
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
        {"gpu", gpuPercent(now, seconds)},
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
