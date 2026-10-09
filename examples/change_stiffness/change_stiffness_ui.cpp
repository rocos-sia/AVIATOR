#include "stiffness_session.hpp"

#include <QApplication>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <condition_variable>
#include <csignal>
#include <iostream>
#include <mutex>
#include <optional>
#include <vector>

using stiffness::Joints;
namespace {
std::atomic<bool> processStop{false};
static_assert(std::atomic<bool>::is_always_lock_free);
void signalStop(int) { processStop.store(true); }

enum class Phase { Idle, Starting, Holding, Applying, Stopping, Failed };
struct Shared {
    std::mutex mutex;
    std::condition_variable wake;
    std::atomic<bool> stop{false};
    Phase phase = Phase::Idle;
    bool done = true;
    std::optional<Joints> pending;
    std::optional<Joints> applied;
    Joints target{};
    std::vector<std::string> messages;
};
class StiffnessWindow : public QWidget {
public:
    explicit StiffnessWindow(bool offline = false) {
        setWindowTitle("关节刚度调节 · xMateER7 Pro");
        resize(900, 740);
        setStyleSheet(
            "QWidget { font-size: 14px; }"
            "QGroupBox { font-weight: bold; border: 1px solid #d4dce6; border-radius: 8px; margin-top: 14px; padding: 16px 10px 10px; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 5px; }"
            "QPushButton { padding: 9px 16px; }"
            "QDoubleSpinBox, QLineEdit { padding: 5px; }"
            "QPushButton#start { background: #1662c4; color: white; border-radius: 5px; }"
            "QPushButton#stop { background: #b83030; color: white; border-radius: 5px; }"
            "QPushButton:disabled, QPushButton#start:disabled, QPushButton#stop:disabled { color: #9299a2; background: #e5e8ec; }"
        );
        auto *layout = new QVBoxLayout(this);
        auto *title = new QLabel("关节刚度调节");
        title->setStyleSheet("font-size: 26px; font-weight: bold; color: #173759;");
        layout->addWidget(title);
        auto *description = new QLabel("保持连接时的关节位置 · 调参时保持 RT 模式，短暂停止运动后恢复保持");
        description->setWordWrap(true);
        layout->addWidget(description);

        auto *connection = new QGroupBox("连接");
        auto *connectionGrid = new QGridLayout(connection);
        preset_ = new QComboBox;
        preset_->setObjectName("armPreset");
        preset_->addItems({"左臂", "右臂", "自定义"});
        robotIp_ = new QLineEdit("192.168.1.160");
        localIp_ = new QLineEdit("192.168.1.100");
        offline_ = new QCheckBox("离线演示（不连接机器人）");
        offline_->setObjectName("offline");
        offline_->setChecked(offline);
        connectionGrid->addWidget(preset_, 0, 0);
        connectionGrid->addWidget(new QLabel("机器人 IP"), 0, 1);
        connectionGrid->addWidget(robotIp_, 0, 2);
        connectionGrid->addWidget(new QLabel("本机 IP"), 0, 3);
        connectionGrid->addWidget(localIp_, 0, 4);
        connectionGrid->addWidget(offline_, 1, 0, 1, 5);
        layout->addWidget(connection);
        connect(preset_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int index) {
            if (index < 2) {
                robotIp_->setText(index == 0 ? "192.168.1.160" : "192.168.2.160");
                localIp_->setText(index == 0 ? "192.168.1.100" : "192.168.2.100");
            }
        });

        auto *parameters = new QGroupBox("关节刚度 · Nm/rad");
        auto *grid = new QGridLayout(parameters);
        grid->addWidget(new QLabel("关节"), 0, 0);
        grid->addWidget(new QLabel("调节"), 0, 1);
        grid->addWidget(new QLabel("待设置"), 0, 2);
        grid->addWidget(new QLabel("已应用"), 0, 3);
        grid->addWidget(new QLabel("上限"), 0, 4);
        grid->setColumnStretch(1, 1);
        for (unsigned i = 0; i < 7; ++i) {
            sliders_[i] = new QSlider(Qt::Horizontal);
            sliders_[i]->setRange(0, static_cast<int>(stiffness::limits[i] * 10));
            sliders_[i]->setValue(static_cast<int>(stiffness::defaults[i] * 10));
            sliders_[i]->setObjectName(QString("slider%1").arg(i + 1));
            inputs_[i] = new QDoubleSpinBox;
            inputs_[i]->setObjectName(QString("joint%1").arg(i + 1));
            inputs_[i]->setDecimals(3);
            inputs_[i]->setRange(0, stiffness::limits[i]);
            inputs_[i]->setSingleStep(i < 4 ? 10 : 0.1);
            inputs_[i]->setValue(stiffness::defaults[i]);
            inputs_[i]->setKeyboardTracking(false);
            inputs_[i]->setMinimumWidth(120);
            applied_[i] = new QLabel("—");
            applied_[i]->setObjectName(QString("applied%1").arg(i + 1));
            applied_[i]->setMinimumWidth(85);
            grid->addWidget(new QLabel(QString("J%1").arg(i + 1)), i + 1, 0);
            grid->addWidget(sliders_[i], i + 1, 1);
            grid->addWidget(inputs_[i], i + 1, 2);
            grid->addWidget(applied_[i], i + 1, 3);
            grid->addWidget(new QLabel(QString::number(stiffness::limits[i])), i + 1, 4);
            connect(sliders_[i], &QSlider::valueChanged, this, [this, i](int value) {
                inputs_[i]->setValue(value / 10.0);
            });
            connect(inputs_[i], qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this, i](double value) {
                const QSignalBlocker blocker(sliders_[i]);
                sliders_[i]->setValue(qRound(value * 10));
                scheduleApply();
            });
            connect(sliders_[i], &QSlider::sliderPressed, this, [this] { debounce_.stop(); });
            connect(sliders_[i], &QSlider::sliderReleased, this, [this] { scheduleApply(); });
        }
        autoApply_ = new QCheckBox("自动应用：停止调节 500 ms 后提交");
        autoApply_->setObjectName("autoApply");
        autoApply_->setChecked(true);
        reset_ = new QPushButton("恢复初始参数");
        grid->addWidget(autoApply_, 8, 0, 1, 3);
        grid->addWidget(reset_, 8, 3, 1, 2);
        layout->addWidget(parameters);
        connect(autoApply_, &QCheckBox::toggled, this, [this](bool on) {
            debounce_.stop();
            if (on) scheduleApply();
        });
        connect(reset_, &QPushButton::clicked, this, [this] {
            for (unsigned i = 0; i < 7; ++i) inputs_[i]->setValue(stiffness::defaults[i]);
        });

        auto *buttons = new QHBoxLayout;
        start_ = new QPushButton("连接并原地保持（上电）"); start_->setObjectName("start");
        apply_ = new QPushButton("应用刚度"); apply_->setObjectName("apply");
        stop_ = new QPushButton("停止并下电"); stop_->setObjectName("stop");
        buttons->addWidget(start_); buttons->addWidget(apply_); buttons->addStretch(); buttons->addWidget(stop_);
        layout->addLayout(buttons);
        status_ = new QLabel; status_->setObjectName("status");
        status_->setStyleSheet("font-size: 16px; font-weight: bold; padding: 6px;");
        layout->addWidget(status_);
        target_ = new QLabel("启动目标（rad）：—"); target_->setWordWrap(true);
        layout->addWidget(target_);
        log_ = new QPlainTextEdit; log_->setReadOnly(true); log_->setMaximumBlockCount(300);
        layout->addWidget(log_, 1);
        connect(start_, &QPushButton::clicked, this, [this] { startSession(); });
        connect(apply_, &QPushButton::clicked, this, [this] { submit(); });
        connect(stop_, &QPushButton::clicked, this, [this] { requestStop(); });
        debounce_.setSingleShot(true); debounce_.setInterval(500);
        connect(&debounce_, &QTimer::timeout, this, [this] { submit(); });
        connect(&poll_, &QTimer::timeout, this, [this] { refresh(); });
        poll_.start(50);
        refresh();
    }

    ~StiffnessWindow() override {
        shared_.stop.store(true);
        shared_.wake.notify_all();
        if (worker_.joinable()) worker_.join();
    }

protected:
    void closeEvent(QCloseEvent *event) override {
        if (worker_.joinable()) {
            closing_ = true;
            requestStop();
            event->ignore(); // 让事件循环继续显示清理状态，完成下电后再关窗口。
        } else event->accept();
    }

private:
    Joints values() const {
        Joints result{};
        for (unsigned i = 0; i < 7; ++i) result[i] = inputs_[i]->value();
        return result;
    }
    void append(const std::string &text) {
        log_->appendPlainText(QTime::currentTime().toString("HH:mm:ss") + "  " + QString::fromStdString(text));
    }
    void scheduleApply() {
        if (phase_ != Phase::Holding || !autoApply_->isChecked()) return;
        for (auto *slider : sliders_) if (slider->isSliderDown()) return;
        debounce_.start();
    }
    void submit() {
        debounce_.stop();
        const auto next = values();
        {
            std::lock_guard<std::mutex> lock(shared_.mutex);
            if (shared_.phase != Phase::Holding || shared_.stop.load() || shared_.applied == next) return;
            shared_.pending = next;
            shared_.phase = Phase::Applying;
        }
        shared_.wake.notify_one();
        refresh();
    }
    void requestStop() {
        debounce_.stop();
        shared_.stop.store(true);
        {
            std::lock_guard<std::mutex> lock(shared_.mutex);
            shared_.pending.reset();
            if (!shared_.done) shared_.phase = Phase::Stopping;
        }
        shared_.wake.notify_one();
    }
    void startSession() {
        if (worker_.joinable() || closing_) return;
        const auto robotIp = robotIp_->text().trimmed().toStdString();
        const auto localIp = localIp_->text().trimmed().toStdString();
        const bool offline = offline_->isChecked();
        if (!offline && (robotIp.empty() || localIp.empty())) {
            append("请填写机器人 IP 和本机 IP");
            return;
        }
        const auto initial = values();
        shared_.stop.store(false);
        {
            std::lock_guard<std::mutex> lock(shared_.mutex);
            shared_.phase = Phase::Starting;
            shared_.done = false;
            shared_.pending.reset(); shared_.applied.reset();
        }
        target_->setText("启动目标（rad）：读取中…");
        for (auto *label : applied_) label->setText("—");
        append(offline ? "启动离线演示" : "正在连接 " + robotIp + "，本机 " + localIp);
        try {
        worker_ = std::thread([this, robotIp, localIp, initial, offline] {
            const auto message = [this](const std::string &text) {
                std::lock_guard<std::mutex> lock(shared_.mutex);
                shared_.messages.push_back(text);
            };
            stiffness::Session session(shared_.stop, offline, message);
            bool failed = false;
            try {
                session.start(robotIp, localIp, initial);
                {
                    std::lock_guard<std::mutex> lock(shared_.mutex);
                    shared_.target = session.target();
                    shared_.applied = initial;
                    if (!shared_.stop.load()) shared_.phase = Phase::Holding;
                }
                while (!shared_.stop.load()) {
                    std::optional<Joints> next;
                    {
                        std::unique_lock<std::mutex> lock(shared_.mutex);
                        shared_.wake.wait_for(lock, std::chrono::milliseconds(50), [this] {
                            return shared_.stop.load() || shared_.pending.has_value();
                        });
                        if (shared_.stop.load()) break;
                        next = shared_.pending;
                        shared_.pending.reset();
                    }
                    session.checkLoop();
                    if (next) {
                        session.apply(*next);
                        std::lock_guard<std::mutex> lock(shared_.mutex);
                        shared_.applied = *next;
                        if (!shared_.stop.load()) shared_.phase = Phase::Holding;
                    }
                }
            } catch (const std::exception &e) {
                failed = !shared_.stop.load();
                message(std::string(failed ? "控制错误：" : "停止：") + e.what());
            } catch (...) {
                failed = true;
                message("未知控制错误");
            }
            shared_.stop.store(true);
            const auto errors = session.shutdown();
            if (!errors.empty()) { failed = true; message("清理错误：" + errors); }
            message(failed ? "控制已结束，请检查错误日志" : (offline ? "离线演示已停止" : "已停止控制、下电并断开连接"));
            std::lock_guard<std::mutex> lock(shared_.mutex);
            shared_.phase = failed ? Phase::Failed : Phase::Idle;
            shared_.done = true;
        });
        } catch (const std::exception &e) {
            shared_.stop.store(true);
            std::lock_guard<std::mutex> lock(shared_.mutex);
            shared_.phase = Phase::Failed;
            shared_.done = true;
            shared_.messages.push_back(std::string("无法启动控制线程：") + e.what());
        }
        refresh();
    }
    void refresh() {
        if (processStop.load() && !closing_) { closing_ = true; requestStop(); }
        bool done;
        std::optional<Joints> applied;
        Joints target;
        std::vector<std::string> messages;
        {
            std::lock_guard<std::mutex> lock(shared_.mutex);
            phase_ = shared_.phase;
            done = shared_.done;
            applied = shared_.applied;
            target = shared_.target;
            messages.swap(shared_.messages);
        }
        for (const auto &text : messages) append(text);
        if (done && worker_.joinable()) worker_.join();
        if (closing_ && done) { close(); return; }
        const bool idle = done;
        const bool holding = phase_ == Phase::Holding;
        const bool editable = idle || holding;
        robotIp_->setEnabled(idle); localIp_->setEnabled(idle); preset_->setEnabled(idle); offline_->setEnabled(idle);
        start_->setEnabled(idle); apply_->setEnabled(holding); stop_->setEnabled(!idle && phase_ != Phase::Stopping);
        start_->setText(offline_->isChecked() ? "开始离线演示" : "连接并原地保持（上电）");
        stop_->setText(offline_->isChecked() ? "停止演示" : "停止并下电");
        autoApply_->setEnabled(editable); reset_->setEnabled(editable);
        for (unsigned i = 0; i < 7; ++i) {
            inputs_[i]->setEnabled(editable); sliders_[i]->setEnabled(editable);
            if (applied) applied_[i]->setText(QString::number((*applied)[i], 'g', 7));
        }
        const QString prefix = offline_->isChecked() ? "离线演示 · " : "真机 · ";
        QString state;
        switch (phase_) {
        case Phase::Idle: state = "未运行"; break;
        case Phase::Starting: state = "正在连接并读取启动位置…"; break;
        case Phase::Holding: state = values() == applied ? "保持中 · 参数已应用" : "保持中 · 有待应用参数"; break;
        case Phase::Applying: state = "保持 RT 模式 · 正在更新刚度并恢复运动…"; break;
        case Phase::Stopping: state = "正在停止并下电…"; break;
        case Phase::Failed: state = "发生错误 · 请检查日志（清理失败时请确认设备状态）"; break;
        }
        status_->setText(prefix + state);
        if (applied) {
            QStringList parts;
            for (double q : target) parts << QString::number(q, 'f', 4);
            target_->setText(offline_->isChecked() ? "启动目标：离线演示，无真机关节反馈"
                                                 : "启动目标（rad）：[" + parts.join(", ") + "]");
        }
    }

    Shared shared_;
    std::thread worker_;
    Phase phase_ = Phase::Idle;
    bool closing_ = false;
    QTimer debounce_, poll_;
    QLineEdit *robotIp_, *localIp_;
    QComboBox *preset_;
    QCheckBox *offline_, *autoApply_;
    std::array<QDoubleSpinBox *, 7> inputs_{};
    std::array<QSlider *, 7> sliders_{};
    std::array<QLabel *, 7> applied_{};
    QPushButton *start_, *apply_, *stop_, *reset_;
    QLabel *status_, *target_;
    QPlainTextEdit *log_;
};
} // namespace

#ifndef STIFFNESS_UI_TEST
int main(int argc, char **argv) {
    bool offline = false;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--dry-run") offline = true;
        else if (argument == "--help") {
            std::cout << "Usage: change_stiffness_ui [--dry-run]\n";
            return 0;
        } else {
            std::cerr << "Unknown argument: " << argument << '\n';
            return 2;
        }
    }
    std::signal(SIGINT, signalStop);
    std::signal(SIGTERM, signalStop);
    QApplication app(argc, argv);
    StiffnessWindow window(offline);
    window.show();
    return app.exec();
}
#endif
