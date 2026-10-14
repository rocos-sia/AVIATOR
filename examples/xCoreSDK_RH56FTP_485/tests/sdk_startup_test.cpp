#include "runtime.hpp"
#include "sdk_robot.hpp"
#include <QCoreApplication>
#include <atomic>
#include <exception>
#include <iostream>
#include <thread>

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);
    QCoreApplication::setApplicationName("xCoreSDK_RH56FTP_485");
    const auto original = QDir::currentPath();
    QTemporaryDir fixture;
    try {
        if (!fixture.isValid()) throw std::runtime_error("Cannot create test directory");
        qputenv("XDG_DATA_HOME", fixture.path().toUtf8());
        demo::prepareSdkRuntime();
        // No SDK calls before this barrier: the first iteration must exercise
        // cold process-wide logger initialization, not an already warmed logger.
        for (int cycle = 0; cycle < 50; ++cycle) {
            std::atomic<int> ready{0}, constructed{0};
            std::exception_ptr errors[2];
            auto run = [&](int i) {
                ++ready;
                while (ready.load() != 2) std::this_thread::yield();
                std::unique_ptr<rokae::ArRobot> robot;
                try { robot = demo::makeSdkRobot(); }
                catch (...) { errors[i] = std::current_exception(); }
                ++constructed;
                // Keep both instances alive together, as in the dual-arm UI.
                while (constructed.load() != 2) std::this_thread::yield();
            };
            std::thread left(run, 0), right(run, 1);
            left.join(); right.join();
            for (const auto& error : errors) if (error) std::rethrow_exception(error);
        }
        QDir::setCurrent(original);
        std::cout << "Concurrent SDK cold start and 50 dual-arm construction/destruction cycles passed; no robot connections\n";
        return 0;
    } catch (const std::exception& e) {
        QDir::setCurrent(original);
        std::cerr << e.what() << '\n';
        return 1;
    }
}
