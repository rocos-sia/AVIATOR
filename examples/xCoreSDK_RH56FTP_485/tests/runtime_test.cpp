#include "runtime.hpp"
#include "window.hpp"
#include "sdk_robot.hpp"
#include <rokae/robot.h>
#include <QCoreApplication>
#include <iostream>

namespace {
void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
}
int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    QCoreApplication::setApplicationName("xCoreSDK_RH56FTP_485");
    const auto original = QDir::currentPath();
    QTemporaryDir fixture;
    try {
        require(fixture.isValid(), "Cannot create test fixture");
        qputenv("XDG_DATA_HOME", (fixture.path() + "/data").toUtf8());
        require(QDir::setCurrent(fixture.path()), "Cannot set launch directory");
        require(QDir().mkdir("logs"), "Cannot create old logs");
        require(QFile::setPermissions("logs", QFile::ReadOwner | QFile::ExeOwner), "Cannot protect old logs");
        QFile config("relative.ini");
        require(config.open(QIODevice::WriteOnly), "Cannot create relative config");
        config.write("[left]\nhand_id=7\n"); config.close();
        const auto settings = demo::loadConfig("relative.ini");
        const auto runtime = demo::prepareSdkRuntime();
        require(runtime.startsWith(fixture.path() + "/data/"), "Runtime outside user data directory");
        require(QDir::currentPath() == runtime, "SDK still using launch directory");
        require(settings.arms[0].handId == 7, "Relative config was not loaded before directory change");
        {
            // Construct the actual SDK without connecting. AR builds can defer log creation.
            auto left = demo::makeSdkRobot();
            auto right = demo::makeSdkRobot();
            require(!left->sdkVersion().empty() && !right->sdkVersion().empty(), "Missing SDK version");
        }
        // Do not require a constructor side effect that differs between SDK releases.
        QFile writeProbe(runtime + "/logs/runtime-test.log");
        require(writeProbe.open(QIODevice::WriteOnly) && writeProbe.write("ok") == 2 && writeProbe.flush(),
                "SDK runtime log directory is not writable");
        writeProbe.close();
        require(QDir(fixture.path() + "/logs").entryList(QDir::Files).empty(), "SDK used old logs");
        require(QFile::setPermissions(fixture.path() + "/logs", QFile::ReadOwner | QFile::WriteOwner | QFile::ExeOwner), "Cannot restore permissions");
        require(QDir::setCurrent(original), "Cannot restore working directory");
        std::cout << "SDK initialization with unwritable launch logs and relative config passed; no robot connections\n";
        return 0;
    } catch (const std::exception& e) {
        QDir::setCurrent(original);
        QFile::setPermissions(fixture.path() + "/logs", QFile::ReadOwner | QFile::WriteOwner | QFile::ExeOwner);
        std::cerr << e.what() << '\n';
        return 1;
    }
}
