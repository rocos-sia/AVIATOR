#include "window.hpp"
#include <QApplication>
#include <QCheckBox>
#include <QElapsedTimer>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QTemporaryFile>
#include <QTemporaryDir>
#include <QDir>
#include <iostream>
#include <thread>

namespace {
void require(bool condition,const char* message) { if(!condition) throw std::runtime_error(message); }
template<class F> void until(F condition) {
    QElapsedTimer timer; timer.start();
    while (!condition()) {
        QApplication::processEvents();
        if(timer.elapsed()>4000) throw std::runtime_error("Timed out waiting for offline state");
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
template<class T> T* widget(demo::Window& w,const QString& name) {
    auto* out=w.findChild<T*>(name); require(out!=nullptr,"Missing UI control"); return out;
}
}
int main(int argc,char** argv) {
    QApplication app(argc,argv);
    QTemporaryDir logs;
    const auto cwd = QDir::currentPath();
    try {
        require(logs.isValid() && QDir::setCurrent(logs.path()), "Cannot create offline test logs");
        demo::Config c; c.offline=true; c.pollMs=100;
        demo::Backend backend;
        require(!backend.submit(3,{}),"Commands must be rejected before start");
        backend.start(c);
        until([&]{return backend.snapshot().ready;});
        std::array<demo::Command,2> commands;
        commands[0].angles={100,200,300,400,500,600};
        commands[1].angles={600,500,400,300,200,100};
        require(backend.submitBoth(commands),"Dual-hand submit failed");
        until([&]{const auto s=backend.snapshot();return s.arms[0].angles==commands[0].angles && s.arms[1].angles==commands[1].angles;});
        require(backend.snapshot().arms[0].feedbackMs>0,"Missing hand feedback timestamp");
        backend.stop();
        require(!backend.submitBoth(commands),"Commands accepted while stopping");
        until([&]{return !backend.snapshot().active;});
        backend.start(c); until([&]{return backend.snapshot().ready;});
        backend.stop(); until([&]{return !backend.snapshot().active;});

        QTemporaryFile file;
        require(file.open(),"Cannot create config fixture");
        // Old arm-control settings must no longer influence hand-only sessions.
        file.write("[left]\nhand_id=2\npower=tool24v\nlocal_ip=invalid\n[right]\nhand_id=3\n[control]\nstiffness=invalid\npoll_ms=200\nrt_priority=invalid\n");file.flush();
        auto loaded=demo::loadConfig(file.fileName());
        require(loaded.arms[0].handId==2 && loaded.arms[1].handId==3 && loaded.pollMs==200,"Config parsing");
        auto initConfig = c; initConfig.init485 = true;
        backend.start(initConfig);
        until([&]{return !backend.snapshot().active;});
        require(!backend.snapshot().ready && !backend.snapshot().arms[0].feedbackMs &&
                !backend.snapshot().arms[1].feedbackMs, "Initialization-only session must not read hands");
        demo::Window window(c); window.show();
        auto* start=widget<QPushButton>(window,"start");
        auto* both=widget<QPushButton>(window,"sendBoth");
        auto* stop=widget<QPushButton>(window,"stop");
        require(!both->isEnabled(),"UI allowed commands before readiness");
        start->click();
        until([&]{return both->isEnabled();});
        require(!start->isEnabled(),"UI allowed repeated start");
        widget<QSpinBox>(window,"angle_0_0")->setValue(123);
        widget<QSpinBox>(window,"angle_1_0")->setValue(789);
        both->click();
        until([&]{return widget<QLabel>(window,"actual_0_0")->text()=="123" && widget<QLabel>(window,"actual_1_0")->text()=="789";});
        const auto preview = qEnvironmentVariable("RH56_PREVIEW_PATH");
        if (!preview.isEmpty()) require(window.grab().save(preview), "Cannot save UI preview");
        stop->click();
        until([&]{return start->isEnabled();});
        require(!both->isEnabled(),"UI allowed commands after stop");
        auto* init = widget<QCheckBox>(window,"init485");
        init->setChecked(true);
        start->click();
        until([&]{return start->isEnabled();});
        require(!both->isEnabled(),"Initialization enabled hand commands");
        init->setChecked(false);
        start->click(); until([&]{return both->isEnabled();});
        window.close(); until([&]{return !window.isVisible();});
        std::cout << "Offline dual-hand lifecycle, independent commands, legacy config and UI passed\n";
        QDir::setCurrent(cwd);
        return 0;
    } catch(const std::exception& e) {QDir::setCurrent(cwd);std::cerr<<e.what()<<'\n';return 1;}
}
