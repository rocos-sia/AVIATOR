#include "window.hpp"
#include "runtime.hpp"
#include <QApplication>
#include <QCommandLineParser>
#include <QDir>
#include <QFileInfo>
#include <atomic>
#include <csignal>
#include <iostream>
namespace {
std::atomic<bool> interrupted{false};
static_assert(std::atomic<bool>::is_always_lock_free);
void signalStop(int) { interrupted.store(true); }
}
int main(int argc, char** argv) {
    QApplication app(argc, argv);
    QCoreApplication::setApplicationName("xCoreSDK_RH56FTP_485");
    QCommandLineParser parser;
    parser.setApplicationDescription("Dual RH56FTP hand control through xCoreSDK RS485; motor power on/off only");
    parser.addHelpOption();
    parser.addOption({"offline", "Offline UI/backend demonstration; no robot connections"});
    parser.addOption({"config", "INI configuration file", "path", QDir(QCoreApplication::applicationDirPath()).filePath("config.ini")});
    parser.process(app);
    try {
        auto config = demo::loadConfig(parser.value("config"));
        config.offline = parser.isSet("offline");
        const auto runtime = demo::prepareSdkRuntime();
        std::cout << "SDK 运行与日志目录：" << runtime.toStdString() << std::endl;
        demo::Window window(config);
        std::signal(SIGINT, signalStop); std::signal(SIGTERM, signalStop);
        QTimer signalTimer;
        QObject::connect(&signalTimer, &QTimer::timeout, &window, [&] {
            if (interrupted.exchange(false)) { window.requestStop(); window.close(); }
        });
        signalTimer.start(50);
        window.show();
        return app.exec();
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n'; return 1;
    }
}
