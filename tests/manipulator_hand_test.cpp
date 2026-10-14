#include "HandRuntime.hpp"
#include "runtime.hpp"
#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>
using namespace aviator;
using namespace aviator::hand;
using Json = nlohmann::json;
namespace {
void check(bool ok, const char* why) {
    if (!ok)
        throw std::runtime_error(why);
}
template <class F> void rejects(F f) {
    bool rejected = false;
    try {
        f();
    } catch (const std::exception&) {
        rejected = true;
    }
    check(rejected, "expected rejection");
}
struct Instrument {
    std::mutex mutex;
    Sample sample;
    std::vector<std::pair<uint16_t, Values>> writes;
    std::atomic<int> read_delay{0}, write_delay{0};
    std::atomic<unsigned> delayed_writes{0};
    std::atomic<bool> fail_write{false}, fail_read{false};
    std::atomic<bool> fail_position{false}, fail_telemetry{false};
};
class TestDevice : public Device {
    std::shared_ptr<Instrument> p;

  public:
    TestDevice(std::shared_ptr<Instrument> value) : p(value) {}
    Sample readSample() {
        std::this_thread::sleep_for(std::chrono::milliseconds(p->read_delay));
        if (p->fail_read)
            throw std::runtime_error("injected read failure");
        std::lock_guard<std::mutex> lock(p->mutex);
        return p->sample;
    }
    Values readPosition() override {
        if (p->fail_position) throw std::runtime_error("injected position failure");
        return readSample().position;
    }
    Sample readTelemetry() override {
        if (p->fail_telemetry) throw std::runtime_error("injected telemetry failure");
        return readSample();
    }
    void write(uint16_t a, const Values& v, const std::function<void()>& check) override {
        if (check)
            check();
        const int delay = p->write_delay.exchange(0);
        if (delay) ++p->delayed_writes;
        std::this_thread::sleep_for(std::chrono::milliseconds(delay));
        if (p->fail_write)
            throw std::runtime_error("injected write failure");
        std::lock_guard<std::mutex> lock(p->mutex);
        p->writes.emplace_back(a, v);
    }
};
uint64_t seq = 0;
std::string epoch = new_session_id();
Message command(double target = .2) {
    Message m;
    m.topic = Topic::hand_command;
    m.header = {"1.0",          ++seq,  utc_us(), monotonic_us(), local_clock_id(),
                "aviator_core", "test", true};
    m.body = {{"control_epoch", epoch},
              {"mode", "NORMALIZED_POSITION"},
              {"origin",
               {{"publisher_id", "aviator_core"},
                {"session_id", "test"},
                {"sequence", seq},
                {"sample_mono_us", m.header.sample_mono_us},
                {"clock_id", local_clock_id()}}}};
    for (auto s : {"left", "right"})
        m.body["hands"][s]["drive_position_normalized"] = std::vector<double>(6, target);
    return m;
}
template <class F> Json wait(Runtime& r, F f, int timeout = 2000) {
    auto until = monotonic_us() + timeout * 1000;
    Json j;
    do {
        auto m = r.state();
        j = m.body;
        j["valid"] = m.header.valid;
        if (f(j))
            return j;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    } while (monotonic_us() < until);
    throw std::runtime_error("timeout: " + j.dump());
}
Bytes reply(const Bytes& tx, const Bytes& payload) {
    Bytes rx{0x90, 0xEB, tx[2], uint8_t(payload.size() + 3), tx[4], tx[5], tx[6]};
    for (auto v : payload)
        rx.push_back(v);
    uint8_t sum = 0;
    for (size_t i = 2; i < rx.size(); ++i)
        sum += rx[i];
    rx.push_back(sum);
    return rx;
}
void protocol() {
    auto tx = request(1, 0x11, 1546, {12});
    check(tx == Bytes({0xeb, 0x90, 1, 4, 0x11, 0xa, 6, 12, 0x32}), "read request");
    auto rx = reply(tx, Bytes(12, 0));
    check(response(tx, rx, 12).size() == 12, "read response");
    for (size_t i = 0; i < rx.size(); ++i) {
        auto bad = rx;
        bad[i] ^= 1;
        rejects([&] { response(tx, bad, 12); });
    }
    rejects([&] { request(0, 0x11, 1546, {12}); });
    rejects([&] { request(1, 0x11, 1546, {57}); });
    rejects([&] { request(1, 0x12, 1486, Bytes(57)); });
    check(request(1, 0x12, 1486, Bytes(56)).size() == 64, "64 byte write");
    auto full = request(1, 0x11, 1546, {56});
    check(response(full, reply(full, Bytes(56)), 56).size() == 56, "64 byte response");
    std::vector<Bytes> sent;
    auto device = rs485Device(2, [&](const Bytes& b, size_t count) {
        sent.push_back(b);
        check(b.size() <= 64 && count <= 64, "transfer bound");
        if (b[4] == 0x12)
            return reply(b, {1});
        Bytes data(count - 8);
        for (size_t i = 0; i < data.size(); ++i)
            data[i] = uint8_t(i);
        return reply(b, data);
    });
    auto s = device->read();
    check(s.position[0] == 0x0b0a && s.position[5] == 0x0100, "channel order");
    check(s.current[0] == 0x1716 && s.errors[0] == 29 && s.status[0] == 35 &&
              s.temperature[0] == 41,
          "telemetry offsets");
    sent.clear();
    device->write(angle_set, Values{unchanged, -1, unchanged, unchanged, 42, unchanged});
    check(sent.size() == 2, "partial writes must omit held channels");
    check(sent[0][5] == 0xd0 && sent[0][7] == 42, "angle address by byte offset");
    check(sent[1][5] == 0xd6 && sent[1][7] == 255 && sent[1][8] == 255, "signed stop encoding");
    rejects([&] { device->write(speed_set, Values{-1, 0, 0, 0, 0, 0}); });
    auto rejecting = rs485Device(1, [&](const Bytes& b, size_t) { return reply(b, {0}); });
    rejects([&] { rejecting->write(mode_set, Values{}); });
}
void config() {
    auto directory = std::filesystem::temp_directory_path() / ("hand config " + new_session_id());
    std::filesystem::create_directories(directory);
    struct Cleanup {
        std::filesystem::path p;
        ~Cleanup() { std::filesystem::remove_all(p); }
    } cleanup{directory};
    auto system = directory / "system.yaml", hand_file = directory / "device.yaml";
    std::ofstream(system) << "xcore_sdk_hand: {fake_hand: true, config: device.yaml}\ncore_hand: "
                             "{publisher_id: test_hand}\n";
    std::ofstream(hand_file) << "left_id: 2\nright_id: 3\nright_enabled: false\nspeed: {grasp: "
                                "300, hold: 50}\nforce: 800\nthreshold: 5\n";
    auto c = loadConfig(system);
    check(c.fake && c.publisher == "test_hand" && c.ids[0] == 2 && c.ids[1] == 3 && !c.sides[1] &&
              c.speed == 300 && c.hold_speed == 50 && c.force == 800 && c.threshold == 5,
          "hand config mapping");
    std::ofstream(system) << "xcore_sdk_hand: {fake_hand: false, config: device.yaml}\n";
    check(loadConfig(system, true).fake, "CLI fake override");
    std::ofstream(hand_file) << "left_id: 255\n";
    rejects([&] { loadConfig(system); });
    std::ofstream(hand_file) << "force: {grasp: 3001}\n";
    rejects([&] { loadConfig(system); });
    std::ofstream(hand_file) << "threshold: -1\n";
    rejects([&] { loadConfig(system); });
}
void serialized_transport() {
    std::atomic<bool> reading{false};
    std::atomic<int> writes{0}, active{0}, peak{0};
    auto d = rs485Device(1, [&](const Bytes& tx, size_t n) {
        int count = ++active;
        if (count > peak)
            peak = count;
        if (tx[4] == 0x11) {
            reading = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
        } else
            ++writes;
        --active;
        return reply(tx, tx[4] == 0x11 ? Bytes(n - 8) : Bytes{1});
    });
    std::thread reader([&] { d->read(); });
    while (!reading)
        std::this_thread::yield();
    auto deadline = monotonic_us() + 10000;
    rejects([&] {
        d->write(angle_set, Values{},
                 [&] { check(monotonic_us() < deadline, "expired while waiting for read"); });
    });
    reader.join();
    check(writes == 0 && peak == 1, "expired command sent or overlapping transactions");
}
void fake() {
    Config cfg;
    cfg.fake = true;
    cfg.period_us = 5000;
    cfg.write_period_us = cfg.position_period_us = cfg.telemetry_period_us = 5000;
    Runtime r(cfg, {fakeDevice(8), fakeDevice(8)});
    wait(r, [](auto j) { return j["valid"] == true; });
    auto c = command();
    r.receive(c);
    auto j = wait(r, [&](auto j) {
        return !j["accepted_command"].is_null() &&
               j["accepted_command"]["sequence"] == c.header.sequence;
    });
    check(j["simulated"] == true && j["hands"]["left"]["sample_time_basis"] == "host_simulation",
          "fake metadata");
    auto accepted = j["accepted_command"];
    auto bad = command();
    bad.header.clock_id = "wrong";
    r.receive(bad);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    check(r.state().body["accepted_command"] == accepted, "bad clock ACK");
    bad = command();
    bad.body["hands"]["left"]["drive_position_normalized"][0] = 1.1;
    r.receive(bad);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    check(r.state().body["accepted_command"] == accepted, "out of range ACK");
    bad = command();
    bad.body["control_epoch"] = new_session_id();
    r.receive(bad);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    check(r.state().body["accepted_command"] == accepted, "foreign epoch ACK");
    wait(r, [](auto j) {
        return j["command_valid"] == false &&
               j["hands"]["left"]["drive_position_normalized"] == Json({.5, 1, 1, 1, 1, 1});
    });
    c = command();
    c.body["mode"] = "GRASP_SETPOINT";
    for (auto side : {"left", "right"})
        c.body["hands"][side] = {{"grasp", {{"closure", .3}}}};
    r.receive(c);
    wait(r, [&](auto j) { return j["accepted_command"]["sequence"] == c.header.sequence; });
    c = command();
    c.header.valid = false;
    r.receive(c);
    wait(r, [&](auto j) {
        return j["accepted_command"]["sequence"] == c.header.sequence &&
               j["command_valid"] == false;
    });
}
void slow_serial_commands(bool moving = false) {
    Config cfg;
    std::atomic<bool> intermediate[2]{};
    auto device = [&](int side) {
        auto position = std::make_shared<Values>(open_pose);
        return rs485Device(1, [&, position, side](const Bytes& tx, size_t count) {
            // Reproduce controller round trips without connecting to hardware.
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
            const int address = tx[5] | (tx[6] << 8);
            if (tx[4] == 0x12) {
                if (address >= angle_set && address < angle_set + 12) {
                    for (size_t i = 7; i + 1 < tx.size() - 1; i += 2) {
                        const int channel = 5 - ((address - angle_set) / 2 + (i - 7) / 2);
                        const int value = tx[i] | (tx[i + 1] << 8);
                        if (value != 65535) (*position)[channel] = value;
                        if (channel == 5 && value > 200 && value < 900) intermediate[side] = true;
                    }
                }
                return reply(tx, {1});
            }
            Bytes data(count - 8);
            if (address == 1546) {
                for (int i = 0; i < 6; ++i) {
                    data[2*i] = uint8_t((*position)[5-i]);
                    data[2*i+1] = uint8_t((*position)[5-i] >> 8);
                }
            }
            return reply(tx, data);
        });
    };
    Runtime r(cfg, {device(0), device(1)});
    wait(r, [](auto j) {
        return j["hands"]["right"]["commanded_drive_position_normalized"] == Json({.5, 1, 1, 1, 1, 1});
    }, 4000);
    const auto began = monotonic_us();
    const auto until = began + 3000000;
    bool reached = false;
    while (monotonic_us() < until) {
        const double progress = std::min(1., (monotonic_us() - began) / 1500000.);
        r.receive(command(moving ? .9 - .7 * progress : .2));
        const auto j = r.state().body;
        reached = j["command_valid"] == true && !j["accepted_command"].is_null() &&
                  j["hands"]["left"]["drive_position_normalized"] == Json(std::vector<double>(6, .2)) &&
                  j["hands"]["right"]["drive_position_normalized"] == Json(std::vector<double>(6, .2));
        if (reached) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    check(reached, "40 ms serial round trips starved hand target/ACK despite fresh commands");
    if (moving) check(intermediate[0] && intermediate[1], "hands skipped intermediate synchronized targets");
}
void equivalent_command_guards() {
    for (bool changing_target : {false, true})
    for (bool foreign_epoch : {false, true}) {
        Config cfg;
        cfg.period_us = 5000;
    cfg.write_period_us = cfg.position_period_us = cfg.telemetry_period_us = 5000;
        auto left = std::make_shared<Instrument>(), right = std::make_shared<Instrument>();
        left->sample.position = right->sample.position = Values{500, 500, 500, 500, 500, 500};
        Runtime r(cfg, {std::make_unique<TestDevice>(left), std::make_unique<TestDevice>(right)});
        wait(r, [](auto j) { return j["valid"] == true; });
        auto c = command(.2);
        r.receive(c);
        auto ack = wait(r, [&](auto j) {
            return j["accepted_command"]["sequence"] == c.header.sequence;
        })["accepted_command"];
        size_t configured;
        {
            std::lock_guard<std::mutex> lock(left->mutex);
            configured = std::count_if(left->writes.begin(), left->writes.end(),
                                      [](const auto& write) { return write.first == mode_set; });
        }
        left->write_delay = 200;
        r.receive(command(.4));
        const auto start_deadline = monotonic_us() + 1000000;
        while (!left->delayed_writes && monotonic_us() < start_deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        check(left->delayed_writes == 1, "slow write did not start");
        const auto until = monotonic_us() + 260000;
        while (monotonic_us() < until) {
            auto bad = command(changing_target ? .6 : .4);
            if (foreign_epoch) bad.body["control_epoch"] = new_session_id();
            else bad.body["origin"]["sample_mono_us"] = monotonic_us() - 1000000;
            r.receive(bad);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            check(r.state().body["accepted_command"] == ack,
                  "foreign or stale-heartbeat command renewed an expired write/ACK");
        }
        {
            std::lock_guard<std::mutex> lock(left->mutex);
            check(configured == size_t(std::count_if(left->writes.begin(), left->writes.end(),
                        [](const auto& write) { return write.first == mode_set; })),
                  "expiry discarded confirmed mode configuration");
            const auto last = std::find_if(left->writes.rbegin(), left->writes.rend(),
                                          [](const auto& write) { return write.first == angle_set; });
            check(last != left->writes.rend() && last->second == Values{-1, -1, -1, -1, -1, -1},
                  "foreign or stale pending command suppressed expiry protection");
        }
    }
}
void scheduled_io() {
    Config cfg;
    cfg.sides[1] = false;
    std::atomic<unsigned> writes{0}, positions{0}, telemetry{0}, settings{0};
    std::atomic<bool> fail_telemetry{false};
    auto device = rs485Device(1, [&](const Bytes& tx, size_t count) {
        const int address = tx[5] | (tx[6] << 8);
        if (tx[4] == 0x12) {
            if (address >= angle_set && address < angle_set + 12) ++writes;
            else ++settings;
            return reply(tx, {1});
        }
        if (address == 1546) ++positions;
        else {
            ++telemetry;
            if (fail_telemetry) throw std::runtime_error("telemetry unavailable");
        }
        return reply(tx, Bytes(count - 8));
    });
    Runtime r(cfg, {std::move(device), nullptr});
    wait(r, [](auto j) { return j["valid"] == true &&
        j["hands"]["left"]["commanded_drive_position_normalized"] == Json({.5,1,1,1,1,1}); });
    writes = positions = telemetry = settings = 0;
    const auto start = monotonic_us();
    unsigned n = 0;
    while (monotonic_us() - start < 1200000) {
        r.receive(command(.1 + .01 * (++n % 70)));
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    check(writes >= 20 && writes <= 25, "position writes did not follow 20 Hz cap");
    check(positions >= 10 && positions <= 13, "position reads did not follow 10 Hz schedule");
    check(telemetry >= 5 && telemetry <= 7, "telemetry reads did not follow 5 Hz schedule");
    check(settings == 0, "steady position stream rewrote cached speed/force/mode");
    fail_telemetry = true;
    const auto old = wait(r, [](const Json& j) {
        return j["hands"]["left"]["feedback_last_error"] != "";
    })["hands"]["left"];
    auto j = wait(r, [&](const Json& j) {
        const auto& h = j["hands"]["left"];
        return h["telemetry_available"] == false && h["error_codes"].empty() &&
               h["sample_mono_us"].get<uint64_t>() > old["sample_mono_us"].get<uint64_t>();
    });
    check(j["valid"] == false && j["hands"]["left"]["telemetry_sample_mono_us"] == old["telemetry_sample_mono_us"],
          "position updates hid stale telemetry or fabricated its timestamp");
}
void persistent_read_failure(bool telemetry) {
    Config cfg;
    cfg.write_period_us = cfg.position_period_us = cfg.telemetry_period_us = 5000;
    cfg.feedback_us = 80000;
    auto left = std::make_shared<Instrument>(), right = std::make_shared<Instrument>();
    left->sample.position = right->sample.position = Values{500, 500, 500, 500, 500, 500};
    Runtime r(cfg, {std::make_unique<TestDevice>(left), std::make_unique<TestDevice>(right)});
    wait(r, [](auto j) { return j["valid"] == true; });
    auto first = command(.2);
    r.receive(first);
    wait(r, [&](auto j) { return j["accepted_command"]["sequence"] == first.header.sequence; });
    if (telemetry) left->fail_telemetry = true;
    else left->fail_position = true;
    const auto bad = wait(r, [](auto j) { return j["hands"]["left"]["feedback_last_error"] != ""; });
    const char* stamp = telemetry ? "telemetry_sample_mono_us" : "sample_mono_us";
    // Continue fresh commands well beyond the feedback deadline. A healthy
    // other read stream must neither renew this timestamp nor suppress stop.
    const auto until = monotonic_us() + 180000;
    while (monotonic_us() < until) {
        r.receive(command(.2));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    auto state = r.state().body;
    check(state["hands"]["left"][stamp] == bad["hands"]["left"][stamp],
          "failed polls fabricated fresh feedback");
    check(state["hands"]["left"]["status"] == "STALE", "persistent outage did not become stale");
    for (const auto& instrument : {left, right}) {
        std::lock_guard<std::mutex> lock(instrument->mutex);
        const auto last = std::find_if(instrument->writes.rbegin(), instrument->writes.rend(),
                                      [](const auto& w) { return w.first == angle_set; });
        check(last != instrument->writes.rend() && last->second == Values{-1,-1,-1,-1,-1,-1},
              "fresh commands suppressed protection during a persistent read failure");
    }
    left->fail_position = left->fail_telemetry = false;
    wait(r, [](auto j) { return j["valid"] == true; });
    auto recovered = command(.7);
    r.receive(recovered);
    wait(r, [&](auto j) { return j["accepted_command"]["sequence"] == recovered.header.sequence &&
        j["hands"]["left"]["commanded_drive_position_normalized"] == Json(std::vector<double>(6, .7)); });
}
void policy() {
    Config cfg;
    cfg.period_us = 5000;
    cfg.write_period_us = cfg.position_period_us = cfg.telemetry_period_us = 5000;
    cfg.hold_us = 60000;
    cfg.feedback_us = 80000;
    auto left = std::make_shared<Instrument>(), right = std::make_shared<Instrument>();
    left->sample.position = right->sample.position = Values{500, 500, 500, 500, 500, 500};
    left->sample.current[0] = 0xff80;
    Runtime r(cfg, {std::make_unique<TestDevice>(left), std::make_unique<TestDevice>(right)});
    auto j = wait(r, [](auto j) { return j["valid"] == true; });
    check(j["hands"]["left"]["current"][0] == -128, "signed current");
    for (int i = 0; i < 14; ++i) {
        r.receive(command(0));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    j = wait(r, [](auto j) { return j["hands"]["left"]["closing_hold_active"][0] == true; });
    size_t count;
    {
        std::lock_guard<std::mutex> lock(left->mutex);
        count = left->writes.size();
    }
    for (int i = 0; i < 4; ++i) {
        r.receive(command(.2));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    {
        std::lock_guard<std::mutex> lock(left->mutex);
        check(count == left->writes.size(), "blocked direction must not rewrite held channels");
    }
    r.receive(command(.8));
    wait(r, [](auto j) { return j["hands"]["left"]["closing_hold_active"][0] == false; });
    {
        std::lock_guard<std::mutex> lock(left->mutex);
        left->sample.errors[0] = 4;
    }
    wait(r, [](auto j) {
        return j["hold_control_error"] != "" &&
               j["hands"]["left"]["fault_protection"]["fault_count"] == 1;
    });
    {
        std::lock_guard<std::mutex> lock(left->mutex);
        left->sample.errors.fill(4);
    }
    wait(r, [](auto j) {
        return j["hands"]["left"]["fault_protection"]["open_requested"] == true &&
               j["hands"]["left"]["commanded_drive_position_normalized"] ==
                   Json({.5, 1, 1, 1, 1, 1});
    });
    {
        std::lock_guard<std::mutex> lock(right->mutex);
        auto it = std::find_if(right->writes.rbegin(), right->writes.rend(),
                               [](auto v) { return v.first == angle_set; });
        check(it != right->writes.rend() && it->second == Values{-1, -1, -1, -1, -1, -1},
              "other hand must stop");
    }
    {
        std::lock_guard<std::mutex> lock(left->mutex);
        left->sample.errors.fill(0);
    }
    wait(r, [](auto j) { return j["hold_control_error"] == ""; });
    auto c = command();
    r.receive(c);
    wait(r, [&](auto j) { return j["accepted_command"]["sequence"] == c.header.sequence; });
    auto ack = r.state().body["accepted_command"];
    left->fail_write = true;
    r.receive(command(.9));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    check(r.state().body["accepted_command"] == ack, "failed writes must not advance ACK");
    {
        std::lock_guard<std::mutex> lock(right->mutex);
        auto it = std::find_if(right->writes.rbegin(), right->writes.rend(),
                               [](auto v) { return v.first == angle_set; });
        check(it != right->writes.rend() && it->second == Values{-1, -1, -1, -1, -1, -1},
              "failed left transport must not skip right protection");
    }
    left->fail_write = false;
    left->write_delay = 140;
    r.receive(command(.1));
    std::this_thread::sleep_for(std::chrono::milliseconds(170));
    check(r.state().body["accepted_command"] == ack, "expired in-flight write advanced ACK");
    left->fail_read = true;
    wait(r, [](auto j) { return j["hands"]["left"]["status"] == "STALE"; });
    {
        std::lock_guard<std::mutex> lock(left->mutex);
        left->writes.clear();
    }
    auto unavailable = command(.7);
    r.receive(unavailable);
    wait(r,
         [&](auto j) { return j["accepted_command"]["sequence"] == unavailable.header.sequence; });
    {
        std::lock_guard<std::mutex> lock(left->mutex);
        for (const auto& [address, values] : left->writes)
            if (address == angle_set)
                check(std::find(values.begin(), values.end(), 700) == values.end(),
                      "stale feedback resumed position command");
    }
    left->fail_read = false;
    wait(r, [](auto j) { return j["hands"]["left"]["valid"] == true; });
    left->read_delay = 180;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto start = monotonic_us();
    j = r.state().body;
    check(monotonic_us() - start < 20000, "state blocked on slow IO");
    check(j["hands"]["right"]["valid"] == true && j["hands"]["left"]["valid"] == false,
          "independent feedback freshness");
    check(j["hands"]["left"]["drive_position_normalized"].empty(),
          "stale measurements must be empty");
    left->read_delay = 0;
}
} // namespace
int main() try {
    protocol();
    config();
    serialized_transport();
    slow_serial_commands();
    slow_serial_commands(true);
    equivalent_command_guards();
    scheduled_io();
    persistent_read_failure(false);
    persistent_read_failure(true);
    fake();
    policy();
    std::cout << "PASS manipulator hand protocol, policy, fake and IO isolation\n";
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}
