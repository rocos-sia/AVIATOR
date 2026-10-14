#include "HandDevice.hpp"
#include "Logger.hpp"
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <mutex>
#include <stdexcept>
namespace aviator::hand {
namespace {
void require(bool ok, const char* why) {
    if (!ok)
        throw std::runtime_error(why);
}
uint8_t sum(const Bytes& b, size_t end) {
    unsigned value = 0;
    for (size_t i = 2; i < end; ++i)
        value += b[i];
    return uint8_t(value);
}
class SerialDevice final : public Device {
    int id_;
    Transfer transfer_;
    std::mutex transaction_mutex_;
    std::condition_variable transaction_ready_;
    uint64_t next_ticket_ = 0, serving_ = 0;
    std::string label_ = "unknown";
    double queue_ms_ = 0;
    struct Timing { unsigned count = 0, errors = 0; double total = 0, peak = 0, queue_peak = 0; };
    std::array<Timing, 4> timings_{};
    std::chrono::steady_clock::time_point report_at_ = std::chrono::steady_clock::now();
    class TransactionGuard {
        SerialDevice& device_;
    public:
        explicit TransactionGuard(SerialDevice& device) : device_(device) {
            const auto start = std::chrono::steady_clock::now();
            std::unique_lock<std::mutex> lock(device_.transaction_mutex_);
            const auto ticket = device_.next_ticket_++;
            device_.transaction_ready_.wait(lock, [&] { return ticket == device_.serving_; });
            device_.queue_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        }
        ~TransactionGuard() {
            std::lock_guard<std::mutex> lock(device_.transaction_mutex_);
            ++device_.serving_;
            device_.transaction_ready_.notify_all();
        }
    };
    void record(const Bytes& tx, std::chrono::steady_clock::time_point start, bool failed) {
        const int address = tx[5] | (tx[6] << 8);
        const int kind = tx[4] == 0x11 ? (address == 1546 ? 1 : 2)
                         : (address >= angle_set && address < angle_set + 12 ? 0 : 3);
        const auto now = std::chrono::steady_clock::now();
        auto& t = timings_[kind];
        const double ms = std::chrono::duration<double, std::milli>(now - start).count();
        ++t.count;
        t.errors += failed;
        t.total += ms;
        t.peak = std::max(t.peak, ms);
        t.queue_peak = std::max(t.queue_peak, queue_ms_);
        queue_ms_ = 0;
        const double elapsed = std::chrono::duration<double>(now - report_at_).count();
        if (elapsed < 5) return;
        const char* names[]{"position_write", "position_read", "telemetry_read", "settings_write"};
        for (size_t i = 0; i < timings_.size(); ++i) {
            const auto& v = timings_[i];
            Logger::info("Hand RS485 timing side={} op={} hz={:.2f} calls={} errors={} sdk_avg_ms={:.2f} sdk_max_ms={:.2f} queue_max_ms={:.2f}",
                         label_, names[i], v.count / elapsed, v.count, v.errors,
                         v.count ? v.total / v.count : 0., v.peak, v.queue_peak);
        }
        timings_ = {};
        report_at_ = now;
    }
    Bytes exchange(const Bytes& tx, size_t count, const std::function<void()>& check = {}) {
        // Caller owns this hand's transaction lock. No arm lifecycle lock.
        if (check)
            check();
        const auto start = std::chrono::steady_clock::now();
        try {
            auto result = transfer_(tx, count);
            record(tx, start, false);
            return result;
        } catch (const std::exception& e) {
            record(tx, start, true);
            throw std::runtime_error(std::string(tx[4] == 0x11 ? "RS485 read" : "RS485 write") +
                " id=" + std::to_string(id_) + " register=" +
                std::to_string(tx[5] | (tx[6] << 8)) + ": " + e.what());
        }
    }
    Bytes readBytes(uint16_t address, size_t count) {
        auto tx = request(id_, 0x11, address, {uint8_t(count)});
        TransactionGuard guard(*this);
        return response(tx, exchange(tx, count + 8), count);
    }

  public:
    void setLabel(const std::string& label) override { label_ = label; }
    SerialDevice(int id, Transfer transfer) : id_(id), transfer_(std::move(transfer)) {
        require(id >= 1 && id <= 254, "RH56 ID outside 1..254");
    }
    Values readPosition() override {
        auto angles = readBytes(1546, 12);
        Values result;
        for (int i = 0; i < 6; ++i) {
            const int j = 5 - i;
            result[i] = angles[2*j] | (angles[2*j+1] << 8);
        }
        return result;
    }
    Sample readTelemetry() override {
        auto telemetry = readBytes(1582, 42); // force, current, error, status, temperature
        Sample result;
        for (int i = 0; i < 6; ++i) {
            int j = 5 - i;
            result.force[i] = telemetry[2 * j] | (telemetry[2 * j + 1] << 8);
            result.current[i] = telemetry[12 + 2 * j] | (telemetry[13 + 2 * j] << 8);
            result.errors[i] = telemetry[24 + j];
            result.status[i] = telemetry[30 + j];
            result.temperature[i] = telemetry[36 + j];
        }
        return result;
    }
    void write(uint16_t address, const Values& values,
               const std::function<void()>& check) override {
        require(address == angle_set || address == speed_set || address == force_set ||
                    address == mode_set,
                "Unsupported RH56 control register");
        int width = address == mode_set ? 1 : 2;
        int limit = address == force_set ? 3000 : address == mode_set ? 0 : 1000;
        for (int v : values)
            require(v == unchanged || (v >= (address == angle_set ? -1 : 0) && v <= limit),
                    "Invalid RH56 register value");
        // FIFO per-hand transactions: neither commands nor feedback can starve
        // the other. This queue is independent of the arm lifecycle.
        TransactionGuard guard(*this);
        // Write only changed contiguous device channels. A held channel must
        // never be restarted by a neighbouring channel's angle update.
        for (int j = 0; j < 6;) {
            if (values[5 - j] == unchanged) {
                ++j;
                continue;
            }
            int first = j;
            Bytes data;
            while (j < 6 && values[5 - j] != unchanged) {
                int v = values[5 - j++];
                data.push_back(uint8_t(v));
                if (width == 2)
                    data.push_back(uint8_t(uint16_t(v) >> 8));
            }
            auto tx = request(id_, 0x12, address + first * width, data);
            response(tx, exchange(tx, 9, check), 1);
        }
    }
};
class FakeDevice final : public Device {
    std::mutex mutex_;
    std::array<double, 6> position_{500, 1000, 1000, 1000, 1000, 1000}, target_ = position_;
    Values speed_{500, 500, 500, 500, 500, 500};
    double rate_;
    std::chrono::steady_clock::time_point last_ = std::chrono::steady_clock::now();
    void advance() {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_).count();
        last_ = now;
        for (int i = 0; i < 6; ++i) {
            double step = dt * rate_ * 1000 * speed_[i] / 500.;
            position_[i] += std::clamp(target_[i] - position_[i], -step, step);
        }
    }

  public:
    explicit FakeDevice(double rate) : rate_(rate) {
        require(std::isfinite(rate) && rate > 0, "Invalid fake hand motion rate");
    }
    Values readPosition() override {
        std::lock_guard<std::mutex> lock(mutex_);
        advance();
        Values p;
        for (int i = 0; i < 6; ++i) {
            p[i] = int(std::floor(position_[i] + .5));
        }
        return p;
    }
    Sample readTelemetry() override {
        Sample s;
        s.temperature.fill(30);
        return s;
    }
    void write(uint16_t address, const Values& v, const std::function<void()>& check) override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (check)
            check();
        advance();
        for (int i = 0; i < 6; ++i)
            if (v[i] != unchanged) {
                if (address == angle_set)
                    target_[i] = v[i] == -1 ? position_[i] : v[i];
                else if (address == speed_set)
                    speed_[i] = v[i];
            }
    }
};
} // namespace
Bytes request(int id, uint8_t op, uint16_t address, const Bytes& payload) {
    require(id >= 1 && id <= 254, "Invalid RH56 ID");
    require((op == 0x11 && payload.size() == 1 && payload[0] >= 1 && payload[0] <= 56) ||
                (op == 0x12 && !payload.empty() && payload.size() <= 56),
            "RH56 transaction exceeds 64 bytes or invalid operation");
    Bytes b{0xEB,
            0x90,
            uint8_t(id),
            uint8_t(payload.size() + 3),
            op,
            uint8_t(address),
            uint8_t(address >> 8)};
    b.reserve(payload.size() + 8);
    for (auto value : payload)
        b.push_back(value);
    b.push_back(sum(b, b.size()));
    return b;
}
Bytes response(const Bytes& tx, const Bytes& rx, size_t count) {
    require(tx.size() >= 9 && count >= 1 && count <= 56 && rx.size() == count + 8,
            "RH56 response size mismatch");
    require(rx[0] == 0x90 && rx[1] == 0xEB && rx[2] == tx[2] && rx[3] == count + 3 &&
                rx[4] == tx[4] && rx[5] == tx[5] && rx[6] == tx[6] &&
                rx.back() == sum(rx, rx.size() - 1),
            "RH56 response header/ID/address/op/checksum mismatch");
    require(tx[4] != 0x12 || (count == 1 && rx[7] == 1), "RH56 write rejected");
    return Bytes(rx.begin() + 7, rx.end() - 1);
}
std::unique_ptr<Device> rs485Device(int id, Transfer transfer) {
    return std::make_unique<SerialDevice>(id, std::move(transfer));
}
std::unique_ptr<Device> fakeDevice(double rate) { return std::make_unique<FakeDevice>(rate); }
} // namespace aviator::hand
