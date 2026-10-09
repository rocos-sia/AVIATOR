// 只使用离线 Session，不允许测试连接真机。
#define STIFFNESS_UI_TEST
#include "../change_stiffness_ui.cpp"
#include <QElapsedTimer>
#include <QTest>
#include <limits>

namespace {
void require(bool condition, const char *message) {
    if (!condition) throw std::runtime_error(message);
}
template<class Predicate>
void waitFor(Predicate ready, const char *message) {
    QElapsedTimer timer;
    timer.start();
    while (!ready() && timer.elapsed() < 4000) QTest::qWait(10);
    require(ready(), message);
}
template<class T>
T *widget(StiffnessWindow &window, const char *name) {
    auto *result = window.findChild<T *>(name);
    require(result != nullptr, name);
    return result;
}
} // namespace

int main(int argc, char **argv) {
    QApplication app(argc, argv);
    app.setQuitOnLastWindowClosed(false);
    try {
        for (double bad : {-1.0, 301.0, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
            auto values = stiffness::defaults;
            values[6] = bad;
            bool rejected = false;
            try { stiffness::validate(values); } catch (const std::exception &) { rejected = true; }
            require(rejected, "backend must reject invalid stiffness");
        }
        StiffnessWindow window(true);
        window.show();
        auto *start = widget<QPushButton>(window, "start");
        auto *stop = widget<QPushButton>(window, "stop");
        auto *apply = widget<QPushButton>(window, "apply");
        auto *automatic = widget<QCheckBox>(window, "autoApply");
        auto *j7 = widget<QDoubleSpinBox>(window, "joint7");
        auto *applied = widget<QLabel>(window, "applied7");
        auto *status = widget<QLabel>(window, "status");
        auto *preset = widget<QComboBox>(window, "armPreset");
        require(start->isEnabled() && !stop->isEnabled(), "initial button state");
        require(j7->maximum() == 300 && widget<QDoubleSpinBox>(window, "joint1")->maximum() == 3000,
                "joint-specific limits");
        start->click();
        require(!start->isEnabled() && !preset->isEnabled(), "connection controls disabled during start");
        waitFor([&] { return apply->isEnabled(); }, "offline start");
        require(applied->text() == "50", "initial applied stiffness");

        // 精确输入；500ms 去抖前已应用值不能提前变化。
        j7->setFocus(); j7->selectAll(); QTest::keyClicks(j7, "0.1"); QTest::keyClick(j7, Qt::Key_Return);
        QTest::qWait(200);
        require(applied->text() == "50", "must not acknowledge before apply");
        waitFor([&] { return applied->text() == "0.1" && apply->isEnabled(); }, "auto apply exact value");
        j7->setValue(1); QTest::qWait(100); j7->setValue(2); QTest::qWait(100); j7->setValue(3);
        waitFor([&] { return applied->text() == "3" && apply->isEnabled(); }, "debounce latest value");

        auto *slider = widget<QSlider>(window, "slider7");
        slider->setSliderDown(true);
        slider->setValue(70);
        QTest::qWait(700);
        require(applied->text() == "3", "dragging must not apply until released");
        slider->setSliderDown(false);
        waitFor([&] { return applied->text() == "7" && apply->isEnabled(); }, "apply after slider release");

        // 精确值不被滑块 0.1 分辨率反向覆盖。
        j7->setValue(0.001);
        waitFor([&] { return applied->text() == "0.001" && apply->isEnabled(); }, "preserve fine precision");
        automatic->setChecked(false);
        j7->setValue(12.3); QTest::qWait(700);
        require(applied->text() == "0.001", "manual mode must not auto apply");
        apply->click();
        require(!j7->isEnabled() && stop->isEnabled(), "stop available during apply");
        waitFor([&] { return applied->text() == "12.3" && apply->isEnabled(); }, "manual apply");
        window.grab().save("/tmp/change_stiffness_ui.png");

        j7->setValue(0.1); apply->click(); stop->click();
        waitFor([&] { return start->isEnabled(); }, "stop while applying");
        require(!status->text().contains("保持中"), "stopped state");
        start->click();
        waitFor([&] { return apply->isEnabled(); }, "restart after stop");
        require(applied->text() == "0.1", "restart uses current input");
        window.close();
        waitFor([&] { return !window.isVisible(); }, "close waits for worker cleanup");

        StiffnessWindow connecting(true);
        connecting.show();
        widget<QPushButton>(connecting, "start")->click();
        connecting.close();
        waitFor([&] { return !connecting.isVisible(); }, "close during start");

        StiffnessWindow signalled(true);
        signalled.show();
        widget<QPushButton>(signalled, "start")->click();
        signalStop(SIGINT);
        waitFor([&] { return !signalled.isVisible(); }, "signal stops and closes");
        processStop.store(false);
        std::cout << "PASS: offline UI, validation, auto/manual apply, precision, stop, restart, close and SIGINT\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}
