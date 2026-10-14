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
    fake_sdk::reset(); fake_sdk::arms[failed].failRead = true;
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
void runtimeFault(bool stalled) {
    fake_sdk::reset();
    demo::Backend backend;
    fake_sdk::ReleaseStalls release;
    backend.start(config());
    until([&] { return backend.snapshot().ready; });
    if (stalled) {
        fake_sdk::arms[1].stall = true;
        until([] { return fake_sdk::arms[1].enteredStall.load(); });
        // Already queued commands must be discarded when the watchdog fires.
        require(backend.submit(2, {}), "Cannot queue command before timeout");
    } else {
        fake_sdk::arms[1].failCleanup = true;
        fake_sdk::arms[1].failRead = true;
    }
    until([&] { return backend.snapshot().arms[1].phase == demo::Phase::Failed; });
    checkHealthy(backend, 0, 456);
    fake_sdk::arms[1].stall = false;
    until([] { return fake_sdk::arms[1].disconnects == 1; });
    require(!backend.snapshot().arms[1].handReady, "Late response restored failed arm readiness");
    require(fake_sdk::arms[1].writes == 0, "Queued command executed after fault");
    stop(backend);
}
void uiFault() {
    fake_sdk::reset(); fake_sdk::arms[1].failRead = true;
    demo::Window window(config());
    fake_sdk::ReleaseStalls release;
    window.show();
    auto* start = widget<QPushButton>(window, "start");
    start->click();
    auto* left = widget<QPushButton>(window, "send0");
    until([&] { return left->isEnabled() && fake_sdk::arms[1].disconnects == 1; });
    require(widget<QPushButton>(window, "read0")->isEnabled(), "Left Read disabled after right fault");
    require(!widget<QPushButton>(window, "send1")->isEnabled(), "Failed right Apply enabled");
    require(!widget<QPushButton>(window, "read1")->isEnabled(), "Failed right Read enabled");
    require(!widget<QPushButton>(window, "sendBoth")->isEnabled(), "Dual Apply enabled with failed right arm");
    require(widget<QLabel>(window, "sessionStatus")->text().contains("左手可独立控制"), "Missing partial-ready status");
    widget<QSpinBox>(window, "angle_0_0")->setValue(123);
    left->click();
    until([&] { return widget<QLabel>(window, "actual_0_0")->text() == "123"; });
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
        runtimeFault(false); runtimeFault(true);
        uiFault();
        QDir::setCurrent(cwd);
        std::cout << "Startup/runtime/watchdog/cleanup faults isolated; surviving arm controls, queue rejection, stop and reconnect passed\n";
        return 0;
    } catch (const std::exception& e) {
        QDir::setCurrent(cwd);
        std::cerr << e.what() << '\n'; return 1;
    }
}
