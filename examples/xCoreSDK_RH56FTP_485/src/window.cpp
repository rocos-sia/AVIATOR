#include "window.hpp"
#include <QCheckBox>
#include <QCloseEvent>
#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QFile>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QSettings>
#include <QSlider>
#include <QSpinBox>
#include <QVBoxLayout>

namespace demo {
Config loadConfig(const QString& file) {
    if (!QFileInfo(file).isReadable()) throw std::runtime_error("Cannot read config: " + file.toStdString());
    QSettings settings(file, QSettings::IniFormat);
    Config c;
    for (int i = 0; i < 2; ++i) {
        settings.beginGroup(i == 0 ? "left" : "right");
        auto& arm = c.arms[i];
        arm.robotIp = settings.value("robot_ip", QString::fromStdString(arm.robotIp)).toString().trimmed().toStdString();
        bool ok = false;
        arm.handId = settings.value("hand_id", 1).toInt(&ok);
        if (!ok || arm.handId < 1 || arm.handId > 254) throw std::runtime_error("hand_id must be 1..254");
        settings.endGroup();
    }
    bool ok = false;
    c.pollMs = settings.value("control/poll_ms", 500).toInt(&ok);
    if (!ok || c.pollMs < 100 || c.pollMs > 10000) throw std::runtime_error("poll_ms must be 100..10000");
    if (settings.status() != QSettings::NoError) throw std::runtime_error("Invalid INI config");
    return c;
}
namespace {
QString phase(Phase p) {
    switch (p) {
    case Phase::Idle: return "未运行";
    case Phase::Starting: return "连接 / 初始化";
    case Phase::Connected: return "485 已连接";
    case Phase::Recovering: return "等待恢复（自动读取中）";
    case Phase::Stopping: return "正在停止";
    case Phase::Failed: return "故障";
    }
    return {};
}
QString age(long long ms) { return ms ? QString::number(nowMs() - ms) + " ms" : "—"; }
QSpinBox* spin(int maximum, int value, const QString& name) {
    auto* box = new QSpinBox;
    box->setRange(0, maximum); box->setValue(value); box->setObjectName(name);
    box->setKeyboardTracking(false); box->setMinimumWidth(68);
    return box;
}
}
Window::Window(Config config, QWidget* parent) : QWidget(parent), initial_(std::move(config)) {
    setWindowTitle("RH56FTP 双手测试 · xCoreSDK 485");
    resize(1380, 900);
    setStyleSheet("QWidget {font-size: 13px;} QGroupBox {font-weight: bold; border: 1px solid #ccd6e2; border-radius: 6px; margin-top: 12px; padding: 12px 8px 8px;} QGroupBox::title {subcontrol-origin: margin; left: 10px; padding: 0 4px;} QPushButton {padding: 7px 12px;} QLineEdit,QSpinBox,QDoubleSpinBox {padding: 3px;} QPushButton#start {background: #1768b5; color: white;} QPushButton#stop {background: #b73333; color: white;} QPushButton:disabled, QPushButton#start:disabled, QPushButton#stop:disabled {color: #888; background: #ddd;}");
    auto* root = new QVBoxLayout(this);
    auto* title = new QLabel("双手 485 测试");
    title->setStyleSheet("font-size: 24px; font-weight: bold; color: #193c60;");
    root->addWidget(title);
    auto* subtitle = new QLabel("连接并上电 / 手部命令经对应机械臂 SDK 发送 / 每类参数一帧更新六通道；保持机械臂当前控制模式。");
    subtitle->setWordWrap(true); root->addWidget(subtitle);
    connections_ = new QGroupBox("会话配置（停止后可修改）");
    auto* connection = new QGridLayout(connections_);
    for (int i = 0; i < 2; ++i) {
        auto& p = panels_[i];
        p.robotIp = new QLineEdit(QString::fromStdString(initial_.arms[i].robotIp));
        p.id = spin(254, initial_.arms[i].handId, QString("handId%1").arg(i)); p.id->setMinimum(1);
        connection->addWidget(new QLabel(i == 0 ? "左臂" : "右臂"), i, 0);
        connection->addWidget(new QLabel("机器人 IP"), i, 1); connection->addWidget(p.robotIp, i, 2);
        connection->addWidget(new QLabel("手 ID"), i, 3); connection->addWidget(p.id, i, 4);
    }
    offline_ = new QCheckBox("离线演示（不连接机器人）"); offline_->setChecked(initial_.offline); offline_->setObjectName("offline");
    connection->addWidget(offline_, 2, 0, 1, 3);
    connection->addWidget(new QLabel("串口匹配条件：115200 / 8N1；初始化后需重启控制器"), 2, 3, 1, 2);
    init485_ = new QCheckBox("仅初始化双侧 485（115200/8N1；不上电；完成后重启控制器）");
    init485_->setObjectName("init485"); init485_->setChecked(initial_.init485);
    connection->addWidget(init485_, 3, 0, 1, 5);
    root->addWidget(connections_);
    auto* scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    auto* body = new QWidget;
    auto* hands = new QHBoxLayout(body);
    const QStringList names{"小指", "无名指", "中指", "食指", "拇指弯曲", "拇指旋转"};
    for (int i = 0; i < 2; ++i) {
        auto& p = panels_[i];
        auto* box = new QGroupBox(i == 0 ? "左手 / 左臂" : "右手 / 右臂");
        auto* column = new QVBoxLayout(box);
        p.status = new QLabel("未运行"); p.status->setWordWrap(true); column->addWidget(p.status);
        p.controls = new QGroupBox("手部参数与反馈（修改后点击下发）");
        auto* grid = new QGridLayout(p.controls);
        const QStringList headers{"通道", "目标角度", "数值", "速度", "力阈值", "实测角度", "受力", "错误"};
        for (int j = 0; j < headers.size(); ++j) grid->addWidget(new QLabel(headers[j]), 0, j);
        for (int j = 0; j < 6; ++j) {
            auto* slider = new QSlider(Qt::Horizontal); slider->setRange(0,1000); slider->setValue(1000); slider->setMinimumWidth(65);
            p.angle[j] = spin(1000, 1000, QString("angle_%1_%2").arg(i).arg(j));
            p.speed[j] = spin(1000, 200, QString("speed_%1_%2").arg(i).arg(j));
            p.force[j] = spin(3000, 300, QString("force_%1_%2").arg(i).arg(j));
            connect(slider, &QSlider::valueChanged, p.angle[j], &QSpinBox::setValue);
            connect(p.angle[j], qOverload<int>(&QSpinBox::valueChanged), slider, &QSlider::setValue);
            grid->addWidget(new QLabel(names[j]), j+1, 0); grid->addWidget(slider, j+1, 1);
            grid->addWidget(p.angle[j], j+1, 2); grid->addWidget(p.speed[j], j+1, 3); grid->addWidget(p.force[j], j+1, 4);
            p.actual[j] = new QLabel("—"); p.actual[j]->setObjectName(QString("actual_%1_%2").arg(i).arg(j));
            p.measuredForce[j] = new QLabel("—"); p.measuredForce[j]->setToolTip("协议有符号原始受力值");
            p.error[j] = new QLabel("—");
            grid->addWidget(p.actual[j], j+1, 5); grid->addWidget(p.measuredForce[j], j+1, 6); grid->addWidget(p.error[j], j+1, 7);
        }
        auto* presets = new QHBoxLayout;
        auto* open = new QPushButton("填入伸展"); auto* close = new QPushButton("填入握合");
        auto* sendButton = new QPushButton("下发本手参数"); sendButton->setObjectName(QString("send%1").arg(i));
        presets->addWidget(open); presets->addWidget(close); presets->addWidget(sendButton);
        grid->addLayout(presets, 7, 0, 1, 8);
        grid->addWidget(new QLabel("角度 0–1000；预设只填前五通道，拇指旋转保留。"), 8, 0, 1, 8);
        connect(open, &QPushButton::clicked, this, [this,i] { for (int j=0;j<5;++j) panels_[i].angle[j]->setValue(1000); });
        connect(close, &QPushButton::clicked, this, [this,i] { for (int j=0;j<5;++j) panels_[i].angle[j]->setValue(0); });
        connect(sendButton, &QPushButton::clicked, this, [this,i] { send(i, Action::Apply); });
        column->addWidget(p.controls);
        auto* read = new QPushButton("立即读取本手反馈"); read->setObjectName(QString("read%1").arg(i));
        connect(read, &QPushButton::clicked, this, [this,i] { send(i, Action::Read); }); column->addWidget(read);
        column->addStretch(); hands->addWidget(box);
    }
    scroll->setWidget(body); root->addWidget(scroll, 1);
    auto* actions = new QHBoxLayout;
    start_ = new QPushButton("连接双手（电机上电）"); start_->setObjectName("start");
    both_ = new QPushButton("双手下发各自参数"); both_->setObjectName("sendBoth");
    stop_ = new QPushButton("断开双手（电机下电）"); stop_->setObjectName("stop");
    stop_->setToolTip("停止 485 读写，请求双臂电机下电后断开 SDK。手部不会自动张开或断电。");
    actions->addWidget(start_); actions->addWidget(both_); actions->addStretch(); actions->addWidget(stop_);
    root->addLayout(actions);
    status_ = new QLabel; status_->setObjectName("sessionStatus"); status_->setWordWrap(true); root->addWidget(status_);
    raw_ = new QCheckBox("显示原始 485 收发帧（仅真机）"); root->addWidget(raw_);
    log_ = new QPlainTextEdit; log_->setReadOnly(true); log_->setMaximumBlockCount(600); log_->setMaximumHeight(90);
    root->addWidget(log_);
    append("SDK 运行与日志目录：" + QDir::currentPath());
    connect(start_, &QPushButton::clicked, this, [this] { start(); });
    connect(stop_, &QPushButton::clicked, this, [this] { requestStop(); });
    connect(both_, &QPushButton::clicked, this, [this] {
        try {
            if (!backend_.submitBoth({command(0), command(1)})) append("下发未接纳：双手尚未就绪或队列已满");
            else append("双手参数已入队；两侧独立执行，不保证同时到达");
        } catch (const std::exception& e) { append(QString::fromUtf8(e.what())); }
    });
    connect(&timer_, &QTimer::timeout, this, [this] { refresh(); });
    timer_.start(50); refresh();
}
Command Window::command(int i) const {
    Command c;
    for (int j = 0; j < 6; ++j) {
        c.angles[j] = panels_[i].angle[j]->value(); c.speeds[j] = panels_[i].speed[j]->value(); c.limits[j] = panels_[i].force[j]->value();
    }
    return c;
}
void Window::append(const QString& message, bool display) {
    const auto line = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz ") + message;
    if (display) log_->appendPlainText(line);
    QFile sessionLog(QDir::current().filePath("session.log"));
    const auto bytes = (line + '\n').toUtf8();
    if (!sessionLog.open(QIODevice::WriteOnly | QIODevice::Append) ||
        sessionLog.write(bytes) != bytes.size() || !sessionLog.flush())
        log_->appendPlainText("保存 session.log 失败：" + sessionLog.errorString());
}
void Window::start() {
    auto c = initial_; c.offline = offline_->isChecked(); c.init485 = init485_->isChecked();
    for (int i = 0; i < 2; ++i) {
        auto& p = panels_[i];
        c.arms[i] = {p.robotIp->text().trimmed().toStdString(), p.id->value()};
    }
    try {
        backend_.start(c);
        for (auto& p : panels_) p.seeded = false;
        append(c.offline ? "启动离线演示" : c.init485 ? "仅初始化双侧 485：设置 115200/8N1 和 500 ms 超时，完成后需重启控制器" : "启动双手真机会话：连接 AR SDK → 电机上电 → 485 读写（沿用当前参数和超时）");
    } catch (const std::exception& e) { append(QString::fromUtf8(e.what())); }
    refresh();
}
void Window::send(int i, Action action) {
    try {
        auto c = command(i); c.action = action;
        if (!backend_.submit(1 << i, c)) append(i == 0 ? "请求未接纳：左手尚未就绪或队列已满" : "请求未接纳：右手尚未就绪或队列已满");
    } catch (const std::exception& e) { append(QString::fromUtf8(e.what())); }
}
void Window::requestStop() { backend_.stop(); refresh(); }
void Window::refresh() {
    const auto s = backend_.snapshot();
    connections_->setEnabled(!s.active && !closing_); start_->setEnabled(!s.active && !closing_);
    start_->setText(init485_->isChecked() ? "初始化双侧 485（不上电）" : "连接双手（电机上电）");
    stop_->setText(init485_->isChecked() ? "停止初始化并断开" : "断开双手（电机下电）");
    stop_->setEnabled(s.active && !s.stopping); both_->setEnabled(s.ready);
    const QString prefix = offline_->isChecked() ? "[离线演示] " : "[真机] ";
    std::array<bool, 2> ready{};
    for (int i = 0; i < 2; ++i)
        ready[i] = s.active && !s.stopping && s.arms[i].handReady && s.arms[i].phase == Phase::Connected;
    status_->setText(prefix + (s.active ? (s.stopping ? "正在停止并清理连接；等待 SDK 返回。" :
        s.ready ? "双手可控制；断开按钮请求双臂电机下电。手部不会自动断电或张开。" :
        ready[0] ? "左手可独立控制；右手未就绪或故障，详见右侧状态。" :
        ready[1] ? "右手可独立控制；左手未就绪或故障，详见左侧状态。" :
        init485_->isChecked() ? "正在连接并初始化 485 参数…" : "暂无可控制的手；各侧连接或清理状态见下方。") :
        "会话未运行；如有清理错误，请查看下方日志。"));
    for (int i = 0; i < 2; ++i) {
        auto& p = panels_[i]; const auto& a = s.arms[i];
        p.controls->setEnabled(ready[i]);
        findChild<QPushButton*>(QString("read%1").arg(i))->setEnabled(ready[i]);
        p.status->setText(phase(a.phase) + " | 手反馈年龄 " + age(a.feedbackMs) +
            QString(" / 最近 485 %1 ms").arg(a.transferMs,0,'f',1) +
            (a.error.empty() ? "" : "\n" + QString::fromStdString(a.error)));
        p.status->setStyleSheet(a.error.empty() ? "" : "color: #b32222;");
        if (a.feedbackMs) {
            for (int j = 0; j < 6; ++j) {
                p.actual[j]->setText(QString::number(a.angles[j])); p.measuredForce[j]->setText(QString::number(a.forces[j]));
                p.error[j]->setText(QString::number(a.errors[j]));
                p.error[j]->setStyleSheet(a.errors[j] ? "color: #b32222;" : "");
                if (!p.seeded) p.angle[j]->setValue(a.angles[j]);
            }
            p.seeded = true;
        } else for (int j = 0; j < 6; ++j) {
            p.actual[j]->setText("—"); p.measuredForce[j]->setText("—"); p.error[j]->setText("—");
        }
    }
    for (const auto& message : backend_.takeLogs()) {
        const auto text = QString::fromStdString(message);
        // The checkbox filters only the UI. Preserve every frame in session.log.
        append(text, raw_->isChecked() || !text.contains("TX "));
    }
    if (closing_ && !s.active) close();
}
void Window::closeEvent(QCloseEvent* event) {
    if (backend_.snapshot().active) {
        closing_ = true; backend_.stop(); event->ignore();
    } else event->accept();
}
} // namespace demo
