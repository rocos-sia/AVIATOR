#pragma once
#include "backend.hpp"
#include <QWidget>
#include <QTimer>
class QCheckBox;
class QGroupBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QSpinBox;

namespace demo {
Config loadConfig(const QString& file);
class Window : public QWidget {
public:
    explicit Window(Config config, QWidget* parent = nullptr);
    void requestStop();
protected:
    void closeEvent(QCloseEvent* event) override;
private:
    struct Panel {
        QLineEdit* robotIp{};
        QSpinBox* id{};
        QGroupBox* controls{};
        std::array<QSpinBox*, 6> angle{}, speed{}, force{};
        std::array<QLabel*, 6> actual{}, measuredForce{}, error{};
        QLabel* status{};
        bool seeded = false;
    };
    Backend backend_;
    Config initial_;
    std::array<Panel, 2> panels_;
    QGroupBox* connections_{};
    QCheckBox *offline_{}, *raw_{}, *init485_{};
    QPushButton *start_{}, *stop_{}, *both_{};
    QLabel* status_{};
    QPlainTextEdit* log_{};
    QTimer timer_;
    bool closing_ = false;
    Command command(int i) const;
    void start();
    void send(int i, Action action);
    void refresh();
    void append(const QString& message, bool display = true);
};
} // namespace demo
