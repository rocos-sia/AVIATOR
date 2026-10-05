"""示教器主界面：左侧触觉热力图，右侧实时状态与运动控制。"""
from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QCheckBox, QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout, QHeaderView,
    QLabel, QMainWindow, QPushButton, QSlider, QSpinBox, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget,
)

from .handlink import FINGERS, N_FINGERS
from .source import Source
from .touchmap import FOUR_FINGERS, build_image, parse_finger, parse_palm

pg.setConfigOptions(antialias=True, background="k", foreground="w")

TAB_COLS = ["自由度", "设定", "实际角度", "受力(g)", "电流(mA)", "温度(℃)", "故障", "状态"]

# 手册 2.6.18 表 53：故障码位定义（1612 状态信息手册未给编码，按原始值显示）
ERR_BITS = [(0x01, "堵转"), (0x02, "过温"), (0x04, "过流"), (0x08, "电机异常"), (0x10, "通讯")]


def _colormap():
    for name in ("inferno", "viridis", "thermal"):
        try:
            return pg.colormap.get(name)
        except Exception:
            continue
    return None


class TouchPanel(QWidget):
    """单个部位的触觉热力图。"""

    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.level = 1000.0
        self._arr = None

        lay = QVBoxLayout(self)
        lay.setContentsMargins(2, 0, 2, 0)
        lay.setSpacing(1)

        self.label = QLabel(title)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setStyleSheet("font-weight: bold; color: #ddd;")
        lay.addWidget(self.label)

        self.plot = pg.PlotWidget()
        self.plot.setMenuEnabled(False)
        self.plot.setMouseEnabled(False, False)
        self.plot.setAspectLocked(True)
        self.plot.hideAxis("left")
        self.plot.hideAxis("bottom")
        self.plot.getViewBox().setDefaultPadding(0)

        self.img = pg.ImageItem()
        cmap = _colormap()
        if cmap is not None:
            self.img.setColorMap(cmap)
        self.plot.addItem(self.img)
        lay.addWidget(self.plot, 1)

    def update_image(self, arr: np.ndarray) -> None:
        self._arr = arr
        self.img.setImage(arr, autoLevels=False)
        self.img.setLevels((0.0, max(1.0, self.level)))
        h, w = arr.shape
        self.plot.setRange(xRange=(0, w), yRange=(0, h), padding=0)

    def set_level(self, level: float) -> None:
        self.level = level
        if self._arr is not None:
            self.img.setLevels((0.0, max(1.0, level)))


class MainWindow(QMainWindow):
    def __init__(self, source: Source):
        super().__init__()
        self.source = source
        self.poller = None
        self._send_timer = QTimer(self)
        self._send_timer.setSingleShot(True)
        self._send_timer.timeout.connect(self._send_angles)

        self.setWindowTitle(f"RH56E2 灵巧手示教器 — {source.name}")
        self.resize(1280, 780)

        root = QWidget()
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)

        outer.addWidget(self._build_toolbar())
        body = QHBoxLayout()
        outer.addLayout(body, 1)

        body.addWidget(self._build_touch_area(), 3)
        body.addWidget(self._build_control_area(), 2)

        self.statusBar().showMessage("未连接")

    # ------------------------------------------------------------ 顶部栏

    def _build_toolbar(self) -> QWidget:
        bar = QWidget()
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(6, 4, 6, 0)

        self.lbl_conn = QLabel("● 未连接")
        self.lbl_conn.setStyleSheet("color: #e66; font-weight: bold;")
        lay.addWidget(self.lbl_conn)

        lay.addSpacing(20)
        lay.addWidget(QLabel("触觉量程上限:"))
        self.sp_level = QSpinBox()
        self.sp_level.setRange(50, 4095)
        self.sp_level.setValue(1000)
        self.sp_level.setSingleStep(100)
        self.sp_level.valueChanged.connect(self._on_level)
        lay.addWidget(self.sp_level)

        lay.addStretch(1)
        self.lbl_rate = QLabel("状态 -- Hz / 触觉 -- Hz")
        lay.addWidget(self.lbl_rate)
        return bar

    def _on_level(self, v: int) -> None:
        for p in self.touch_panels.values():
            p.set_level(float(v))

    # ------------------------------------------------------------ 左：触觉

    def _build_touch_area(self) -> QWidget:
        box = QGroupBox("触觉（压阻式，0-4095）")
        grid = QGridLayout(box)
        grid.setSpacing(4)
        self.touch_panels: dict[str, TouchPanel] = {}

        for col, name in enumerate(FOUR_FINGERS):
            p = TouchPanel(name)
            self.touch_panels[name] = p
            grid.addWidget(p, 0, col)

        p_thumb = TouchPanel("拇指")
        self.touch_panels["拇指"] = p_thumb
        grid.addWidget(p_thumb, 1, 0, 1, 2)

        p_palm = TouchPanel("掌心")
        self.touch_panels["掌心"] = p_palm
        grid.addWidget(p_palm, 1, 2, 1, 2)

        for c in range(4):
            grid.setColumnStretch(c, 1)
        grid.setRowStretch(0, 3)
        grid.setRowStretch(1, 3)
        return box

    # ------------------------------------------------------------ 右：控制

    def _build_control_area(self) -> QWidget:
        panel = QWidget()
        lay = QVBoxLayout(panel)
        lay.setContentsMargins(0, 0, 0, 0)

        # 状态表
        self.table = QTableWidget(N_FINGERS, len(TAB_COLS))
        self.table.setHorizontalHeaderLabels(TAB_COLS)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        for r, f in enumerate(FINGERS):
            self.table.setItem(r, 0, QTableWidgetItem(f))
            for c in range(1, len(TAB_COLS)):
                self.table.setItem(r, c, QTableWidgetItem("--"))
        lay.addWidget(self.table, 1)

        # 角度滑条
        gb = QGroupBox("角度设定 (0-1000)")
        g = QGridLayout(gb)
        self.sliders: list[QSlider] = []
        self.spins: list[QSpinBox] = []
        for i, f in enumerate(FINGERS):
            g.addWidget(QLabel(f), i, 0)
            s = QSlider(Qt.Horizontal)
            s.setRange(0, 1000)
            s.setValue(1000)
            sp = QSpinBox()
            sp.setRange(0, 1000)
            sp.setValue(1000)
            s.valueChanged.connect(sp.setValue)
            sp.valueChanged.connect(s.setValue)
            s.valueChanged.connect(self._on_slider)
            self.sliders.append(s)
            self.spins.append(sp)
            g.addWidget(s, i, 1)
            g.addWidget(sp, i, 2)
        lay.addWidget(gb)

        # 速度 / 力控
        gb2 = QGroupBox("速度 / 力控阈值（整手）")
        g2 = QGridLayout(gb2)
        self.sl_speed = QSlider(Qt.Horizontal)
        self.sl_speed.setRange(0, 1000)
        self.sl_speed.setValue(1000)
        self.sl_speed.valueChanged.connect(self._on_slider)
        g2.addWidget(QLabel("速度"), 0, 0)
        g2.addWidget(self.sl_speed, 0, 1, 1, 2)

        self.sl_force = QSlider(Qt.Horizontal)
        self.sl_force.setRange(0, 3000)
        self.sl_force.setValue(1000)
        self.sl_force.valueChanged.connect(self._on_slider)
        g2.addWidget(QLabel("力控"), 1, 0)
        g2.addWidget(self.sl_force, 1, 1, 1, 2)
        lay.addWidget(gb2)

        # 按钮
        gb3 = QGroupBox("操作")
        g3 = QGridLayout(gb3)
        self.chk_live = QCheckBox("滑条实时下发")
        self.chk_live.setChecked(True)
        g3.addWidget(self.chk_live, 0, 0, 1, 2)

        for col, (text, fn) in enumerate([
            ("全部张开 (1000)", lambda: self._set_all_angles(1000)),
            ("全部握紧 (0)",    lambda: self._set_all_angles(0)),
            ("清除错误",        self._clear_error),
            ("保存到 Flash",    self._save_flash),
        ]):
            b = QPushButton(text)
            b.clicked.connect(fn)
            g3.addWidget(b, 1 + col // 2, col % 2)
        lay.addWidget(gb3)

        return panel

    # ------------------------------------------------------------ 下发

    def _on_slider(self) -> None:
        # 速度/力控是整手值，直接发
        if self.chk_live.isChecked():
            self._send_timer.start(40)

    def _set_all_angles(self, v: int) -> None:
        for s in self.sliders:
            s.blockSignals(True)
            s.setValue(v)
            s.blockSignals(False)
        for sp in self.spins:
            sp.setValue(v)
        self.chk_live.setChecked(True)
        self._send_angles()

    def _send_angles(self) -> None:
        if self.poller is None:
            return
        vals = [s.value() for s in self.sliders]
        try:
            self.source.write_speed_set([self.sl_speed.value()] * N_FINGERS)
            self.source.write_force_set([self.sl_force.value()] * N_FINGERS)
            self.source.write_angle_set(vals)
            self.statusBar().showMessage("已下发 " + " ".join(map(str, vals)), 1500)
        except Exception as e:
            self.statusBar().showMessage(f"下发失败: {e}")

    def _clear_error(self) -> None:
        try:
            self.source.clear_error()
            self.statusBar().showMessage("已发送清除错误", 2000)
        except Exception as e:
            self.statusBar().showMessage(f"失败: {e}")

    def _save_flash(self) -> None:
        try:
            self.source.save_flash()
            self.statusBar().showMessage("已发送保存到 Flash", 2000)
        except Exception as e:
            self.statusBar().showMessage(f"失败: {e}")

    # ------------------------------------------------------------ 数据回调

    def on_state(self, d: dict) -> None:
        ang, force = d["angle"], d["force"]
        cur, temp = d["current"], d["temp"]
        err, status = d["err"], d["status"]
        for r in range(N_FINGERS):
            def set_(c, text, color=None):
                it = self.table.item(r, c)
                s = str(text)
                if it.text() != s:          # 只在变化时写，避免每轮 48 次刷新
                    it.setText(s)
                if color and it.data(Qt.UserRole) != color:
                    it.setForeground(pg.mkColor(color))
                    it.setData(Qt.UserRole, color)
            set_(1, self.spins[r].value())
            set_(2, ang[r])
            set_(3, force[r])
            set_(4, cur[r])
            set_(5, temp[r])
            flags = "/".join(t for b, t in ERR_BITS if err[r] & b)
            set_(6, flags or "正常", "#e66" if err[r] else "#888")
            set_(7, status[r])
        self._state_hz = d.get("hz", 0.0)
        self._refresh_rate()

    def on_touch(self, d: dict) -> None:
        for name, raw in d.items():
            panel = self.touch_panels.get(name)
            if panel is None:
                continue
            if name == "掌心":
                arr = parse_palm(raw)
            else:
                parsed = parse_finger(name, raw)
                arr = build_image(name, parsed) if parsed else None
            if arr is not None:
                panel.update_image(arr)
        self._touch_hz = d.get("hz", 0.0)
        self._refresh_rate()

    def _refresh_rate(self) -> None:
        self.lbl_rate.setText(
            f"状态 {getattr(self, '_state_hz', 0):.0f} Hz / 触觉 {getattr(self, '_touch_hz', 0):.0f} Hz")

    def on_failed(self, msg: str) -> None:
        self.lbl_conn.setText("● 通信异常")
        self.lbl_conn.setStyleSheet("color: #e66; font-weight: bold;")
        self.statusBar().showMessage(msg)

    def set_connected(self, ok: bool) -> None:
        if ok:
            self.lbl_conn.setText(f"● 已连接 ({self.source.name})")
            self.lbl_conn.setStyleSheet("color: #6e6; font-weight: bold;")
        else:
            self.lbl_conn.setText("● 未连接")
            self.lbl_conn.setStyleSheet("color: #e66; font-weight: bold;")

    def closeEvent(self, ev) -> None:
        if self.poller is not None:
            self.poller.stop()
        try:
            self.source.close()
        except Exception:
            pass
        super().closeEvent(ev)
