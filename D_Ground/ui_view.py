#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ui_view.py

D题：立体货架盘点无人机地面站 UI 文件

职责：
1. 绘制界面
2. 发出用户操作信号
3. 提供界面更新接口

本文件不直接发送 UDP。
本文件不直接处理无人机业务逻辑。
"""

from PyQt5.QtWidgets import (
    QMainWindow,
    QWidget,
    QLabel,
    QPushButton,
    QLineEdit,
    QListWidget,
    QGridLayout,
    QVBoxLayout,
    QHBoxLayout,
    QFrame,
    QSizePolicy,
    QWIDGETSIZE_MAX,
)
from PyQt5.QtCore import Qt, pyqtSignal, QTime
from PyQt5.QtGui import QFont


class GroundStationUI(QMainWindow):
    """
    地面站主界面
    """

    # UI 操作信号
    task1_clicked = pyqtSignal()
    task2_scan_clicked = pyqtSignal()
    task2_start_clicked = pyqtSignal()
    query_entered = pyqtSignal(str)
    network_changed = pyqtSignal(int, str, int)
    clear_log_requested = pyqtSignal()
    reset_requested = pyqtSignal()
    emergency_stop_clicked = pyqtSignal()

    def __init__(self):
        super().__init__()

        self.cell_buttons = {}
        self.inventory_data = {}
        self.node_indicators = {}

        self.last_highlight_coord = None
        self._base_width = 1024
        self._base_height = 600
        self._scale_ready = False
        self._current_scale = 1.0
        self._font_bases = {}
        self._fixed_size_bases = {}
        self._min_size_bases = {}
        self._max_size_bases = {}
        self.log_list = None

        self.init_ui()

    # ==================================================
    # 总体 UI
    # ==================================================

    def init_ui(self):
        self.setWindowTitle("TI杯D题 - 立体货架盘点无人机地面站")
        self.resize(self._base_width, self._base_height)
        self.setMinimumSize(self._base_width, self._base_height)

        self.setStyleSheet("""
            QMainWindow {
                background-color: #F4F6F8;
            }

            QLabel {
                color: #263238;
                font-family: "Microsoft YaHei UI";
            }

            QPushButton {
                font-family: "Microsoft YaHei UI";
            }

            QLineEdit {
                font-family: "Microsoft YaHei UI";
            }
        """)

        root = QWidget()
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(10, 10, 10, 10)
        root_layout.setSpacing(8)

        main_layout = QHBoxLayout()
        main_layout.setSpacing(8)

        left_panel = self.build_left_panel()
        right_panel = self.build_right_panel()

        right_panel.setMaximumWidth(360)

        main_layout.addWidget(left_panel, 70)
        main_layout.addWidget(right_panel, 30)

        root_layout.addLayout(main_layout, 1)

        self.setCentralWidget(root)
        self._capture_scaling_targets()
        self._apply_scale(1.0)
        self._scale_ready = True

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if not self._scale_ready:
            return
        scale = self._compute_scale()
        if abs(scale - self._current_scale) > 0.01:
            self._apply_scale(scale)

    def _compute_scale(self) -> float:
        return min(self.width() / self._base_width, self.height() / self._base_height)

    @staticmethod
    def _scaled_value(value: int, scale: float) -> int:
        return max(1, int(round(value * scale)))

    def _capture_scaling_targets(self):
        self._font_bases.clear()
        self._fixed_size_bases.clear()
        self._min_size_bases.clear()
        self._max_size_bases.clear()

        for widget in self.findChildren(QWidget):
            font = QFont(widget.font())
            if font.pointSizeF() > 0 or font.pixelSize() > 0:
                self._font_bases[widget] = font

            min_w = widget.minimumWidth()
            max_w = widget.maximumWidth()
            min_h = widget.minimumHeight()
            max_h = widget.maximumHeight()

            if min_w == max_w and min_w > 0:
                self._fixed_size_bases.setdefault(widget, {})["w"] = min_w
            elif min_w > 0:
                self._min_size_bases.setdefault(widget, {})["w"] = min_w

            if min_h == max_h and min_h > 0:
                self._fixed_size_bases.setdefault(widget, {})["h"] = min_h
            elif min_h > 0:
                self._min_size_bases.setdefault(widget, {})["h"] = min_h

            if max_w < QWIDGETSIZE_MAX and min_w != max_w:
                self._max_size_bases.setdefault(widget, {})["w"] = max_w
            if max_h < QWIDGETSIZE_MAX and min_h != max_h:
                self._max_size_bases.setdefault(widget, {})["h"] = max_h

    def _apply_scale(self, scale: float):
        for widget, font in self._font_bases.items():
            scaled = QFont(font)
            if font.pointSizeF() > 0:
                scaled.setPointSizeF(font.pointSizeF() * scale)
            elif font.pixelSize() > 0:
                scaled.setPixelSize(self._scaled_value(font.pixelSize(), scale))
            widget.setFont(scaled)

        for widget, sizes in self._min_size_bases.items():
            if "w" in sizes:
                widget.setMinimumWidth(self._scaled_value(sizes["w"], scale))
            if "h" in sizes:
                widget.setMinimumHeight(self._scaled_value(sizes["h"], scale))

        for widget, sizes in self._max_size_bases.items():
            if "w" in sizes:
                widget.setMaximumWidth(self._scaled_value(sizes["w"], scale))
            if "h" in sizes:
                widget.setMaximumHeight(self._scaled_value(sizes["h"], scale))

        for widget, sizes in self._fixed_size_bases.items():
            if "w" in sizes and "h" in sizes:
                widget.setFixedSize(
                    self._scaled_value(sizes["w"], scale),
                    self._scaled_value(sizes["h"], scale),
                )
            else:
                if "w" in sizes:
                    widget.setFixedWidth(self._scaled_value(sizes["w"], scale))
                if "h" in sizes:
                    widget.setFixedHeight(self._scaled_value(sizes["h"], scale))

        self._current_scale = scale

    def build_title_bar(self):
        panel = QFrame()
        panel.setStyleSheet(self.style_panel())
        panel.setFixedHeight(36)

        layout = QHBoxLayout(panel)
        layout.setContentsMargins(10, 0, 10, 0)

        title = QLabel("立体货架盘点无人机地面站")
        title.setFont(QFont("Microsoft YaHei UI", 16, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("border: none; color: #1565C0;")

        layout.addWidget(title)

        return panel

    # ==================================================
    # 左侧：货架表格
    # ==================================================

    def build_left_panel(self):
        panel = QFrame()
        panel.setStyleSheet(self.style_panel())

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(6)

        title = QLabel("货架盘点结果")
        title.setFont(QFont("Microsoft YaHei UI", 18, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("""
            QLabel {
                color: #1565C0;
                padding: 4px;
                border: none;
            }
        """)

        layout.addWidget(title)

        table_layout = QGridLayout()
        table_layout.setSpacing(4)
        table_layout.setContentsMargins(0, 0, 0, 0)

        blank = QLabel("")
        blank.setFixedWidth(30)
        blank.setStyleSheet("border: none;")
        table_layout.addWidget(blank, 0, 0)

        for col in range(1, 7):
            label = QLabel(str(col))
            label.setFont(QFont("Microsoft YaHei UI", 14, QFont.Bold))
            label.setAlignment(Qt.AlignCenter)
            label.setFixedHeight(28)
            label.setStyleSheet(self.style_col_header())
            table_layout.addWidget(label, 0, col)

        rows = ["A", "B", "C", "D"]

        for row_index, row_name in enumerate(rows, start=1):
            row_label = QLabel(row_name)
            row_label.setFont(QFont("Microsoft YaHei UI", 14, QFont.Bold))
            row_label.setAlignment(Qt.AlignCenter)
            row_label.setFixedWidth(30)
            row_label.setStyleSheet(self.style_row_header())
            table_layout.addWidget(row_label, row_index, 0)

            for col in range(1, 7):
                coord = f"{row_name}{col}"

                self.inventory_data[coord] = None

                btn = QPushButton(f"{coord}\n--")
                btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
                btn.setMinimumHeight(52)
                btn.setFont(QFont("Microsoft YaHei UI", 12, QFont.Bold))
                btn.setCursor(Qt.PointingHandCursor)
                btn.setStyleSheet(self.style_cell_empty())

                table_layout.addWidget(btn, row_index, col)
                self.cell_buttons[coord] = btn

        for r in range(5):
            table_layout.setRowStretch(r, 1)

        for c in range(7):
            table_layout.setColumnStretch(c, 1)

        table_container = QWidget()
        table_container.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        table_container_layout = QVBoxLayout(table_container)
        table_container_layout.setContentsMargins(0, 0, 0, 0)
        table_container_layout.addLayout(table_layout)

        layout.addWidget(table_container, 1)

        status_and_network_layout = QHBoxLayout()
        status_and_network_layout.setContentsMargins(0, 0, 0, 0)
        status_and_network_layout.setSpacing(6)
        
        status_panel = self.build_status_panel()
        network_panel = self.build_network_panel()
        
        status_and_network_layout.addWidget(status_panel, 1)
        status_and_network_layout.addWidget(network_panel, 1)
        
        layout.addLayout(status_and_network_layout)
        
        layout.addWidget(self.build_log_panel(), 1)

        return panel

    # ==================================================
    # 右侧主面板
    # ==================================================

    def build_right_panel(self):
        panel = QFrame()
        panel.setStyleSheet(self.style_panel())

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        layout.addWidget(self.build_query_panel(), 1)
        layout.addSpacing(4)
        layout.addWidget(self.build_bottom_panel())

        return panel

    # ==================================================
    # 网络配置
    # ==================================================

    def build_network_panel(self):
        panel = QFrame()
        panel.setStyleSheet(self.style_sub_panel())
        panel.setMaximumHeight(100)

        layout = QGridLayout(panel)
        layout.setContentsMargins(6, 5, 6, 5)
        layout.setHorizontalSpacing(2)
        layout.setVerticalSpacing(4)

        self.recv_port_input = QLineEdit("8888")
        self.ip_input = QLineEdit("192.168.151.102")
        self.send_port_input = QLineEdit("8889")

        for edit in [self.recv_port_input, self.ip_input, self.send_port_input]:
            edit.setFixedHeight(22)
            edit.setFont(QFont("Consolas", 8))
            edit.setStyleSheet(self.style_line_edit())

        label_recv = QLabel("收：")
        label_ip = QLabel("IP：")
        label_send = QLabel("发：")

        for label in [label_recv, label_ip, label_send]:
            label.setFont(QFont("Microsoft YaHei UI", 8))
            label.setStyleSheet("border: none;")

        input_width = 140
        self.recv_port_input.setFixedWidth(input_width)
        self.ip_input.setFixedWidth(input_width)
        self.send_port_input.setFixedWidth(input_width)

        self.apply_network_btn = QPushButton("应用网络配置")
        self.apply_network_btn.setFixedSize(84, 42)
        self.apply_network_btn.setFont(QFont("Microsoft YaHei UI", 8, QFont.Bold))
        self.apply_network_btn.setStyleSheet(self.style_small_blue_button())
        self.apply_network_btn.clicked.connect(self.on_apply_network_clicked)

        layout.addWidget(label_recv, 0, 0)
        layout.addWidget(self.recv_port_input, 0, 1, alignment=Qt.AlignLeft)
        layout.addWidget(label_ip, 1, 0)
        layout.addWidget(self.ip_input, 1, 1, alignment=Qt.AlignLeft)
        layout.addWidget(label_send, 2, 0)
        layout.addWidget(self.send_port_input, 2, 1, alignment=Qt.AlignLeft)
        layout.addWidget(self.apply_network_btn, 0, 2, 3, 1, alignment=Qt.AlignVCenter)

        layout.setColumnStretch(0, 0)
        layout.setColumnStretch(1, 0)
        layout.setColumnStretch(2, 0)

        return panel

    def on_apply_network_clicked(self):
        """
        用户点击应用网络配置
        """
        try:
            local_port = int(self.recv_port_input.text().strip())
        except ValueError:
            local_port = 8888
            self.recv_port_input.setText("8888")

        drone_ip = self.ip_input.text().strip()

        if not drone_ip:
            drone_ip = "192.168.151.102"
            self.ip_input.setText(drone_ip)

        try:
            drone_port = int(self.send_port_input.text().strip())
        except ValueError:
            drone_port = 8889
            self.send_port_input.setText("8889")

        self.network_changed.emit(local_port, drone_ip, drone_port)

    def set_drone_ip(self, ip: str):
        self.ip_input.setText(ip)

    # ==================================================
    # 查询区 + 小键盘
    # ==================================================

    def build_query_panel(self):
        panel = QFrame()
        panel.setStyleSheet(self.style_sub_panel())
        panel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        title = QLabel("编号查询")
        title.setFont(QFont("Microsoft YaHei UI", 11, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("border: none; color: #1565C0;")
        layout.addWidget(title)

        self.query_input = QLineEdit()
        self.query_input.setReadOnly(True)
        self.query_input.setAlignment(Qt.AlignCenter)
        self.query_input.setFont(QFont("Consolas", 14, QFont.Bold))
        self.query_input.setFixedHeight(30)
        self.query_input.setStyleSheet("""
            QLineEdit {
                background-color: white;
                color: #111111;
                border: 2px solid #90CAF9;
                border-radius: 6px;
                padding: 2px;
            }
        """)
        layout.addWidget(self.query_input)

        keypad_layout = QGridLayout()
        keypad_layout.setSpacing(4)

        keys = [
            "1", "2", "3",
            "4", "5", "6",
            "7", "8", "9",
            "del", "0", "enter"
        ]

        for index, key in enumerate(keys):
            row = index // 3
            col = index % 3

            btn = QPushButton(key)
            btn.setFixedHeight(30)
            btn.setFont(QFont("Microsoft YaHei UI", 10, QFont.Bold))
            btn.clicked.connect(lambda checked=False, k=key: self.handle_keypad(k))

            if key == "enter":
                btn.setStyleSheet(self.style_key_enter())
            elif key == "del":
                btn.setStyleSheet(self.style_key_del())
            else:
                btn.setStyleSheet(self.style_key_number())

            keypad_layout.addWidget(btn, row, col)

        for row in range(4):
            keypad_layout.setRowStretch(row, 1)

        layout.addLayout(keypad_layout, 3)

        self.query_output = QLabel("查询结果显示区")
        self.query_output.setFont(QFont("Microsoft YaHei UI", 9, QFont.Bold))
        self.query_output.setAlignment(Qt.AlignCenter)
        self.query_output.setWordWrap(True)
        self.query_output.setMinimumHeight(40)
        self.query_output.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.query_output.setStyleSheet("""
            QLabel {
                background-color: #263238;
                color: #00E676;
                border-radius: 6px;
                border: none;
                padding: 3px;
            }
        """)
        layout.addWidget(self.query_output, 1)

        self.target_info = QLabel("目标信息：暂无")
        self.target_info.setFont(QFont("Microsoft YaHei UI", 8, QFont.Bold))
        self.target_info.setAlignment(Qt.AlignCenter)
        self.target_info.setWordWrap(True)
        self.target_info.setMinimumHeight(38)
        self.target_info.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.target_info.setStyleSheet("""
            QLabel {
                background-color: #FFF8E1;
                color: #E65100;
                border-radius: 6px;
                border: 1px solid #FFE082;
                padding: 3px;
            }
        """)
        layout.addWidget(self.target_info, 1)
        layout.addStretch(1)

        return panel

    def handle_keypad(self, key: str):
        """
        小键盘事件
        """
        current = self.query_input.text()

        if key == "del":
            self.query_input.setText(current[:-1])
            return

        if key == "enter":
            self.query_entered.emit(self.query_input.text())
            return

        # 货物编号 1~24，最多两位
        if len(current) < 2:
            self.query_input.setText(current + key)

    # ==================================================
    # 节点指示灯
    # ==================================================

    def build_node_panel(self):
        return self.build_status_panel()

    def add_node_indicator(self, layout, label_text: str, key: str, row: int, col: int):
        item = QFrame()
        item.setStyleSheet("border: none;")

        vbox = QVBoxLayout(item)
        vbox.setContentsMargins(0, 0, 0, 0)
        vbox.setSpacing(2)

        light = QLabel("●")
        light.setAlignment(Qt.AlignCenter)
        light.setFont(QFont("Arial", 14, QFont.Bold))
        light.setStyleSheet("""
            QLabel {
                color: #BDBDBD;
                border: none;
            }
        """)

        label = QLabel(label_text)
        label.setAlignment(Qt.AlignCenter)
        label.setFont(QFont("Microsoft YaHei UI", 7, QFont.Bold))
        label.setStyleSheet("""
            QLabel {
                color: #455A64;
                border: none;
            }
        """)

        vbox.addWidget(light)
        vbox.addWidget(label)

        layout.addWidget(item, row, col)

        self.node_indicators[key] = light

    def set_node_ready(self, key: str, ready: bool = True):
        light = self.node_indicators.get(key)

        if not light:
            return

        color = "#00C853" if ready else "#BDBDBD"

        light.setStyleSheet(f"""
            QLabel {{
                color: {color};
                border: none;
            }}
        """)

    # ==================================================
    # 状态面板
    # ==================================================

    def build_status_panel(self):
        panel = QFrame()
        panel.setStyleSheet(self.style_sub_panel())
        panel.setMaximumHeight(96)

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(4)

        node_layout = QGridLayout()
        node_layout.setSpacing(3)

        self.add_node_indicator(node_layout, "视觉", "VISION", 0, 0)
        self.add_node_indicator(node_layout, "通信", "RECEIVER", 0, 1)
        self.add_node_indicator(node_layout, "飞控", "FSM", 0, 2)

        layout.addLayout(node_layout)

        self.comm_status_label = QLabel("通信：待连接")
        self.comm_status_label.setFont(QFont("Microsoft YaHei UI", 8, QFont.Bold))
        self.comm_status_label.setWordWrap(True)
        self.comm_status_label.setStyleSheet("""
            QLabel {
                color: #1565C0;
                border: none;
            }
        """)

        self.task_status_label = QLabel("任务：待启动")
        self.task_status_label.setFont(QFont("Microsoft YaHei UI", 8, QFont.Bold))
        self.task_status_label.setWordWrap(True)
        self.task_status_label.setStyleSheet("""
            QLabel {
                color: #2E7D32;
                border: none;
            }
        """)

        status_row = QHBoxLayout()
        status_row.setSpacing(4)
        status_row.addWidget(self.comm_status_label)
        status_row.addWidget(self.task_status_label)

        layout.addLayout(status_row)

        return panel

    def set_comm_status(self, text: str):
        self.comm_status_label.setText(f"通信：{text}")

    def set_task_status(self, text: str):
        self.task_status_label.setText(f"任务：{text}")

    # ==================================================
    # 日志面板
    # ==================================================

    def build_log_panel(self):
        panel = QFrame()
        panel.setStyleSheet(self.style_sub_panel())

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        title = QLabel("日志")
        title.setFont(QFont("Microsoft YaHei UI", 10, QFont.Bold))
        title.setStyleSheet("""
            QLabel {
                color: #37474F;
                border: none;
            }
        """)

        self.log_list = QListWidget()
        log_font = QFont("Microsoft YaHei UI")
        log_font.setPointSizeF(8.8)
        self.log_list.setFont(log_font)
        self.log_list.setMinimumHeight(80)
        self.log_list.setSpacing(1)
        self.log_list.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.log_list.setStyleSheet("""
            QListWidget {
                background-color: white;
                border: 1px solid #CFD8DC;
                border-radius: 4px;
                color: #263238;
                padding: 4px;
            }
        """)

        layout.addWidget(title)
        layout.addWidget(self.log_list, 1)

        return panel

    def append_log(self, msg: str):
        timestamp = QTime.currentTime().toString("HH:mm:ss")
        self.log_list.addItem(f"[{timestamp}] {msg}")
        self.log_list.scrollToBottom()

    def clear_log(self):
        self.log_list.clear()

    # ==================================================
    # 底部按钮
    # ==================================================

    def build_bottom_panel(self):
        panel = QFrame()
        panel.setStyleSheet(self.style_sub_panel())
        panel.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)

        layout = QGridLayout(panel)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setHorizontalSpacing(6)
        layout.setVerticalSpacing(4)

        self.task1_btn = QPushButton("任务1：巡查")
        self.task2_scan_btn = QPushButton("任务2.1")
        self.task2_start_btn = QPushButton("任务2.2")
        self.clear_log_btn = QPushButton("清空日志")
        self.reset_btn = QPushButton("全局复位")
        self.emergency_stop_btn = QPushButton("紧急刹停")

        buttons = [
            self.task1_btn,
            self.task2_scan_btn,
            self.task2_start_btn,
            self.clear_log_btn,
            self.reset_btn,
            self.emergency_stop_btn,
        ]

        for btn in buttons:
            btn.setFixedHeight(28)
            btn.setFont(QFont("Microsoft YaHei UI", 9, QFont.Bold))
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        self.task1_btn.setStyleSheet(self.style_bottom_blue())
        self.task2_scan_btn.setStyleSheet(self.style_bottom_green())
        self.clear_log_btn.setStyleSheet(self.style_bottom_gray())
        self.reset_btn.setStyleSheet(self.style_bottom_red())
        self.emergency_stop_btn.setStyleSheet(self.style_bottom_red())

        self.task1_btn.clicked.connect(self.task1_clicked.emit)
        self.task2_scan_btn.clicked.connect(self.task2_scan_clicked.emit)
        self.task2_start_btn.clicked.connect(self.task2_start_clicked.emit)
        self.clear_log_btn.clicked.connect(self.clear_log_requested.emit)
        self.reset_btn.clicked.connect(self.reset_requested.emit)
        self.emergency_stop_btn.clicked.connect(self.emergency_stop_clicked.emit)

        layout.addWidget(self.task1_btn, 0, 0, 1, 2)
        layout.addWidget(self.task2_scan_btn, 1, 0)
        layout.addWidget(self.task2_start_btn, 1, 1)
        layout.addWidget(self.clear_log_btn, 2, 0)
        layout.addWidget(self.reset_btn, 2, 1)
        layout.addWidget(self.emergency_stop_btn, 3, 0, 1, 2)

        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)

        self.set_task2_start_enabled(False)

        return panel

    # ==================================================
    # 盘点表格接口
    # ==================================================

    def is_valid_coord(self, coord: str) -> bool:
        if not coord:
            return False

        return coord.upper() in self.cell_buttons

    def update_inventory_cell(self, coord: str, item_id: str):
        coord = coord.upper()

        if coord not in self.cell_buttons:
            self.append_log(f"无效坐标：{coord}")
            return

        self.clear_highlight()

        if str(item_id).isdigit():
            value = str(int(item_id))
        else:
            value = str(item_id)

        self.inventory_data[coord] = value

        btn = self.cell_buttons[coord]
        btn.setText(f"{coord}\nID:{value}")
        btn.setStyleSheet(self.style_cell_done())

    def find_coord_by_item_id(self, item_id: str):
        if not str(item_id).isdigit():
            return None

        target = str(int(item_id))

        for coord, value in self.inventory_data.items():
            if value is None:
                continue

            if str(value).isdigit() and str(int(value)) == target:
                return coord

        return None

    def highlight_cell(self, coord: str):
        coord = coord.upper()

        if coord not in self.cell_buttons:
            return

        self.clear_highlight()

        btn = self.cell_buttons[coord]
        btn.setStyleSheet(self.style_cell_highlight())

        self.last_highlight_coord = coord

    def clear_highlight(self):
        if not self.last_highlight_coord:
            return

        coord = self.last_highlight_coord

        if coord in self.cell_buttons:
            if self.inventory_data.get(coord) is None:
                self.cell_buttons[coord].setStyleSheet(self.style_cell_empty())
            else:
                self.cell_buttons[coord].setStyleSheet(self.style_cell_done())

        self.last_highlight_coord = None

    def set_query_output(self, text: str):
        self.query_output.setText(text)

    def set_target_info(self, text: str):
        self.target_info.setText(text)

    def set_task2_start_enabled(self, enabled: bool):
        self.task2_start_btn.setEnabled(enabled)

        if enabled:
            self.task2_start_btn.setStyleSheet(self.style_bottom_green())
        else:
            self.task2_start_btn.setStyleSheet(self.style_bottom_gray())

    def reset_inventory_table(self):
        self.clear_highlight()

        for coord in self.inventory_data.keys():
            self.inventory_data[coord] = None
            btn = self.cell_buttons.get(coord)
            if btn:
                btn.setText(f"{coord}\n--")
                btn.setStyleSheet(self.style_cell_empty())

    def reset_node_indicators(self):
        for key in self.node_indicators.keys():
            self.set_node_ready(key, False)

    def reset_all(self):
        self.reset_inventory_table()
        self.reset_node_indicators()

        self.query_input.setText("")
        self.set_query_output("查询结果显示区")
        self.set_target_info("目标信息：暂无")
        self.set_task_status("待启动")
        self.set_task2_start_enabled(False)

    # ==================================================
    # 样式函数
    # ==================================================

    def style_panel(self):
        return """
            QFrame {
                background-color: white;
                border-radius: 10px;
                border: 1px solid #DDE3EA;
            }
        """

    def style_sub_panel(self):
        return """
            QFrame {
                background-color: #FAFAFA;
                border-radius: 8px;
                border: 1px solid #E0E0E0;
            }
        """

    def style_col_header(self):
        return """
            QLabel {
                background-color: #E3F2FD;
                color: #0D47A1;
                border-radius: 8px;
                border: 1px solid #BBDEFB;
            }
        """

    def style_row_header(self):
        return """
            QLabel {
                background-color: #E8F5E9;
                color: #1B5E20;
                border-radius: 8px;
                border: 1px solid #C8E6C9;
            }
        """

    def style_cell_empty(self):
        return """
            QPushButton {
                background-color: #F5F5F5;
                color: #455A64;
                border: 2px solid #CFD8DC;
                border-radius: 10px;
                padding: 4px;
            }
            QPushButton:pressed {
                background-color: #E0E0E0;
            }
        """

    def style_cell_done(self):
        return """
            QPushButton {
                background-color: #C8E6C9;
                color: #1B5E20;
                border: 3px solid #43A047;
                border-radius: 10px;
                padding: 4px;
            }
        """

    def style_cell_highlight(self):
        return """
            QPushButton {
                background-color: #FFF176;
                color: #E65100;
                border: 4px solid #FF6F00;
                border-radius: 10px;
                padding: 4px;
            }
        """

    def style_line_edit(self):
        return """
            QLineEdit {
                background-color: white;
                color: #111111;
                border: 1px solid #CFD8DC;
                border-radius: 4px;
                padding: 2px 5px;
            }
        """

    def style_small_blue_button(self):
        return """
            QPushButton {
                background-color: #1976D2;
                color: white;
                border-radius: 5px;
                border: none;
            }
            QPushButton:pressed {
                background-color: #0D47A1;
            }
        """

    def style_key_number(self):
        return """
            QPushButton {
                background-color: #FFFFFF;
                color: #263238;
                border: 1px solid #B0BEC5;
                border-radius: 6px;
            }
            QPushButton:pressed {
                background-color: #E3F2FD;
            }
        """

    def style_key_del(self):
        return """
            QPushButton {
                background-color: #FFE0B2;
                color: #E65100;
                border: 1px solid #FFB74D;
                border-radius: 6px;
            }
            QPushButton:pressed {
                background-color: #FFCC80;
            }
        """

    def style_key_enter(self):
        return """
            QPushButton {
                background-color: #1976D2;
                color: white;
                border: none;
                border-radius: 6px;
            }
            QPushButton:pressed {
                background-color: #0D47A1;
            }
        """

    def style_bottom_blue(self):
        return """
            QPushButton {
                background-color: #1976D2;
                color: white;
                border-radius: 8px;
                border: none;
            }
            QPushButton:pressed {
                background-color: #0D47A1;
            }
        """

    def style_bottom_green(self):
        return """
            QPushButton {
                background-color: #43A047;
                color: white;
                border-radius: 8px;
                border: none;
            }
            QPushButton:pressed {
                background-color: #1B5E20;
            }
        """

    def style_bottom_gray(self):
        return """
            QPushButton {
                background-color: #607D8B;
                color: white;
                border-radius: 8px;
                border: none;
            }
            QPushButton:pressed {
                background-color: #37474F;
            }
        """

    def style_bottom_red(self):
        return """
            QPushButton {
                background-color: #D32F2F;
                color: white;
                border-radius: 8px;
                border: none;
            }
            QPushButton:pressed {
                background-color: #B71C1C;
            }
        """
