#include "window.hpp"
#include <rokae/robot.h>
#include <QApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QTemporaryDir>
#include <iostream>

namespace {
void require(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
template<class F> void until(F condition) {
    QElapsedTimer timer; timer.start();
    while (!condition()) {
        QApplication::processEvents();
        if (timer.elapsed() > 6000) throw std::runtime_error("Timed out waiting for fault-isolation state");
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
template<class T> T* widget(demo::Window& w, const QString& name) {
    auto* result = w.findChild<T*>(name);
    require(result, "Missing UI control"); return result;
}
demo::Config config() { demo::Config c; c.pollMs = 100; return c; }
void stop(demo::Backend& backend) {
    backend.stop();
    require(!backend.submit(1, {}), "Stop must reject left commands");
    require(!backend.submit(2, {}), "Stop must reject right commands");
    until([&] { return !backend.snapshot().active; });
}
void checkHealthy(demo::Backend& backend, int healthy, int target) {
    const int failed = 1 - healthy;
    const auto before = backend.snapshot();
    require(before.active && !before.stopping && !before.ready, "One fault must not stop the whole session");
    require(before.arms[healthy].handReady, "Healthy arm lost readiness");
    require(!backend.submit(1 << failed, {}), "Failed arm accepted a command");
    require(!backend.submit(3, {}), "Dual mask accepted with one failed arm");
    require(!backend.submitBoth({demo::Command{}, demo::Command{}}), "Dual submit accepted with one failed arm");
    require(fake_sdk::arms[healthy].powerOff == 0 && fake_sdk::arms[healthy].disconnects == 0,
            "Fault cleaned up healthy arm");
    demo::Command command; command.angles.fill(target);
    require(backend.submit(1 << healthy, command), "Healthy arm rejected Apply");
    until([&] { return backend.snapshot().arms[healthy].angles == command.angles; });
    require(fake_sdk::arms[healthy].writes == 3, "Rejected dual command partially reached healthy arm");
    const auto feedback = backend.snapshot().arms[healthy].feedbackMs;
    until([&] { return backend.snapshot().arms[healthy].feedbackMs > feedback; });
    demo::Command read; read.action = demo::Action::Read;
    require(backend.submit(1 << healthy, read), "Healthy arm rejected Read");
}
void startupFault(int failed) {
    fake_sdk::reset(); fake_sdk::arms[failed].failPowerOn = true;
    demo::Backend backend;
    fake_sdk::ReleaseStalls release;
    backend.start(config());
    until([&] { const auto s = backend.snapshot(); return s.arms[failed].phase == demo::Phase::Failed &&
        s.arms[1-failed].handReady && fake_sdk::arms[failed].disconnects == 1; });
    checkHealthy(backend, 1-failed, 345);
    require(fake_sdk::arms[failed].powerOff == 1, "Failed arm did not power off");
    stop(backend);
    require(fake_sdk::arms[1-failed].powerOff == 1, "Global stop did not clean healthy arm");
    fake_sdk::reset();
    backend.start(config());
    until([&] { return backend.snapshot().ready; });
    stop(backend);
}
void recovery(bool startup, bool actuator) {
    fake_sdk::reset();
    auto& arm = fake_sdk::arms[1];
    auto fault = [&](bool enabled) {
        if (actuator) arm.errorCode = enabled ? 1 : 0;
        else arm.failRead = enabled;
    };
    if (startup) fault(true);
    demo::Backend backend;
    backend.start(config());
    if (!startup) {
        until([&] { return backend.snapshot().ready; });
        fault(true);
    }
    until([&] { const auto s = backend.snapshot(); return s.arms[0].handReady &&
        s.arms[1].phase == demo::Phase::Recovering; });
    checkHealthy(backend, 0, 456);
    const int attempts = arm.attempts;
    until([&] { return arm.attempts > attempts + 5; });
    if (actuator) require(backend.snapshot().arms[1].errors[0] == 1, "Missing actuator fault feedback");
    if (!startup && !actuator) {
        // Repeated failures must survive the old 3-second stale-feedback cutoff.
        const auto began = demo::nowMs();
        until([&] { return demo::nowMs() - began > 3200; });
        require(backend.snapshot().arms[1].phase == demo::Phase::Recovering, "Persistent fault ended session");
    }
    require(arm.powerOff == 0 && arm.disconnects == 0, "Recoverable fault powered off or disconnected");
    fault(false);
    until([&] { return backend.snapshot().ready; });
    require(backend.snapshot().arms[1].error.empty(), "Recovered arm retained error");
    require(arm.writes == 0, "Recovery replayed a rejected command");
    demo::Command command; command.angles.fill(321);
    require(backend.submit(2, command), "Recovered arm rejected new command");
    until([&] { return backend.snapshot().arms[1].angles == command.angles; });
    stop(backend);
    require(arm.powerOff == 1 && arm.disconnects == 1, "Stop did not clean recovered arm");
}
void stalledRecovery() {
    fake_sdk::reset();
    demo::Backend backend;
    fake_sdk::ReleaseStalls release;
    backend.start(config());
    until([&] { return backend.snapshot().ready; });
    fake_sdk::arms[1].stall = true;
    until([] { return fake_sdk::arms[1].enteredStall.load(); });
    require(backend.submit(2, {}), "Cannot queue command before timeout");
    until([&] { return backend.snapshot().arms[1].phase == demo::Phase::Recovering; });
    checkHealthy(backend, 0, 456);
    fake_sdk::arms[1].stall = false;
    until([&] { return backend.snapshot().ready; });
    require(fake_sdk::arms[1].powerOff == 0 && fake_sdk::arms[1].disconnects == 0, "Watchdog ended recoverable session");
    require(fake_sdk::arms[1].writes == 0, "Queued command executed after fault");
    stop(backend);
}
void writeAndProtocolRecovery() {
    fake_sdk::reset();
    demo::Backend backend;
    backend.start(config());
    until([&] { return backend.snapshot().ready; });
    fake_sdk::arms[1].loseWriteAck = true;
    demo::Command command; command.speeds.fill(123); command.angles.fill(456);
    require(backend.submit(2, command), "Cannot submit write with lost ACK");
    until([&] { return backend.snapshot().arms[1].phase == demo::Phase::Recovering; });
    until([&] { return backend.snapshot().ready; });
    require(fake_sdk::arms[1].writes == 1, "Uncertain write retried or continued remaining parameters");
    require(backend.snapshot().arms[1].speeds == command.speeds, "Recovery did not read back partial write");
    require(backend.snapshot().arms[1].angles[0] == 1000, "Failed command resumed angle write");
    fake_sdk::arms[1].badChecksum = true;
    until([&] { return backend.snapshot().arms[1].phase == demo::Phase::Recovering; });
    fake_sdk::arms[1].badChecksum = false;
    until([&] { return backend.snapshot().ready; });
    fake_sdk::arms[1].failRead = true;
    until([&] { return backend.snapshot().arms[1].phase == demo::Phase::Recovering; });
    fake_sdk::arms[1].failCleanup = true;
    stop(backend);
    require(backend.snapshot().arms[1].phase == demo::Phase::Failed, "Cleanup failure not reported");
    require(fake_sdk::arms[1].disconnects == 1, "Cleanup failure skipped disconnect");
}
void uiFault() {
    fake_sdk::reset(); fake_sdk::arms[1].failRead = true;
    demo::Window window(config());
    fake_sdk::ReleaseStalls release;
    window.show();
    auto* start = widget<QPushButton>(window, "start");
    start->click();
    auto* left = widget<QPushButton>(window, "send0");
    until([&] { return left->isEnabled() && fake_sdk::arms[1].attempts > 1; });
    require(widget<QPushButton>(window, "read0")->isEnabled(), "Left Read disabled after right fault");
    require(!widget<QPushButton>(window, "send1")->isEnabled(), "Failed right Apply enabled");
    require(!widget<QPushButton>(window, "read1")->isEnabled(), "Failed right Read enabled");
    require(!widget<QPushButton>(window, "sendBoth")->isEnabled(), "Dual Apply enabled with failed right arm");
    require(widget<QLabel>(window, "sessionStatus")->text().contains("左手可独立控制"), "Missing partial-ready status");
    widget<QSpinBox>(window, "angle_0_0")->setValue(123);
    left->click();
    until([&] { return widget<QLabel>(window, "actual_0_0")->text() == "123"; });
    fake_sdk::arms[1].failRead = false;
    until([&] { return widget<QPushButton>(window, "sendBoth")->isEnabled(); });
    require(widget<QPushButton>(window, "send1")->isEnabled(), "Recovered right Apply disabled");
    require(widget<QPushButton>(window, "read1")->isEnabled(), "Recovered right Read disabled");
    require(fake_sdk::arms[1].disconnects == 0, "UI recovery reconnected arm");
    widget<QPushButton>(window, "stop")->click();
    until([&] { return start->isEnabled(); });
    require(!left->isEnabled(), "Left Apply enabled after global stop");
    window.close();
}
}
int main(int argc, char** argv) {
    QApplication app(argc, argv);
    QTemporaryDir logs;
    const auto cwd = QDir::currentPath();
    try {
        require(logs.isValid() && QDir::setCurrent(logs.path()), "Cannot create test logs");
        startupFault(1); startupFault(0);
        recovery(true, false); recovery(false, false);
        recovery(true, true); recovery(false, true);
        stalledRecovery(); writeAndProtocolRecovery();
        uiFault();
        QDir::setCurrent(cwd);
        std::cout << "Startup/runtime/actuator/protocol/watchdog recovery, no write replay, UI recovery and cleanup isolation passed\n";
        return 0;
    } catch (const std::exception& e) {
        QDir::setCurrent(cwd);
        std::cerr << e.what() << '\n'; return 1;
    }
}
