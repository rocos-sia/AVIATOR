#pragma once
#include <QDir>
#include <QCoreApplication>
#include <QFileInfo>
#include <QFile>
#include <QStandardPaths>
#include <QTemporaryDir>
#include <stdexcept>

namespace demo {
inline QString defaultConfigPath() {
    const auto directory = QCoreApplication::applicationDirPath();
    const auto installed = directory + "/../share/aviator/examples/xCoreSDK_RH56FTP_485/config.ini";
    return QFileInfo::exists(installed) ? installed : directory + "/config.ini";
}

// xCoreSDK writes logs relative to the process working directory. Call once,
// after loading relative config paths and before creating any SDK objects/threads.
inline QString prepareSdkRuntime() {
    const auto sessions = QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation) + "/sessions";
    if (!QDir().mkpath(sessions))
        throw std::runtime_error("无法创建 SDK 日志目录：" + sessions.toStdString());
    QTemporaryDir session(sessions + "/run-XXXXXX");
    if (!session.isValid())
        throw std::runtime_error("无法创建 SDK 运行目录：" + session.errorString().toStdString());
    const auto path = session.path();
    const auto logs = path + "/logs";
    if (!QDir().mkpath(logs))
        throw std::runtime_error("无法创建 SDK 日志目录：" + logs.toStdString());
    QFile probe(logs + "/.write-check");
    if (!probe.open(QIODevice::WriteOnly) || probe.write("ok") != 2 || !probe.flush())
        throw std::runtime_error("SDK 日志目录无法写入：" + logs.toStdString() + "：" + probe.errorString().toStdString());
    probe.close();
    probe.remove();
    if (!QDir::setCurrent(path))
        throw std::runtime_error("无法切换 SDK 运行目录：" + path.toStdString());
    // Keep logs for diagnosis after the application exits or crashes.
    session.setAutoRemove(false);
    return path;
}
} // namespace demo
