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

        self.init_ui()

    # ==================================================
    # 总体 UI
    # ==================================================

    def init_ui(self):
        self.setWindowTitle("TI杯D题 - 立体货架盘点无人机地面站")
        self.resize(1024, 600)

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
        main_layout.setSpacing(10)

        left_panel = self.build_left_panel()
        right_panel = self.build_right_panel()
        bottom_panel = self.build_bottom_panel()

        main_layout.addWidget(left_panel, 5)
        main_layout.addWidget(right_panel, 2)

        root_layout.addLayout(main_layout, 1)
        root_layout.addWidget(bottom_panel, 0)

        self.setCentralWidget(root)

    # ==================================================
    # 左侧：货架表格
    # ==================================================

    def build_left_panel(self):
        panel = QFrame()
        panel.setStyleSheet(self.style_panel())

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        title = QLabel("货架盘点结果")
        title.setFont(QFont("Microsoft YaHei UI", 22, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("""
            QLabel {
                color: #1565C0;
                padding: 6px;
                border: none;
            }
        """)

        layout.addWidget(title)

        table_layout = QGridLayout()
        table_layout.setSpacing(6)

        blank = QLabel("")
        blank.setFixedWidth(44)
        blank.setStyleSheet("border: none;")
        table_layout.addWidget(blank, 0, 0)

        # 列标题：1~6
        for col in range(1, 7):
            label = QLabel(str(col))
            label.setFont(QFont("Microsoft YaHei UI", 18, QFont.Bold))
            label.setAlignment(Qt.AlignCenter)
            label.setFixedHeight(44)
            label.setStyleSheet(self.style_col_header())
            table_layout.addWidget(label, 0, col)

        # 行标题：A~D
        rows = ["A", "B", "C", "D"]

        for row_index, row_name in enumerate(rows, start=1):
            row_label = QLabel(row_name)
            row_label.setFont(QFont("Microsoft YaHei UI", 18, QFont.Bold))
            row_label.setAlignment(Qt.AlignCenter)
            row_label.setFixedWidth(44)
            row_label.setStyleSheet(self.style_row_header())
            table_layout.addWidget(row_label, row_index, 0)

            for col in range(1, 7):
                coord = f"{row_name}{col}"

                self.inventory_data[coord] = None

                btn = QPushButton(f"{coord}\n--")
                btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
                btn.setMinimumHeight(92)
                btn.setFont(QFont("Consolas", 20, QFont.Bold))
                btn.setCursor(Qt.PointingHandCursor)
                btn.setStyleSheet(self.style_cell_empty())

                table_layout.addWidget(btn, row_index, col)
                self.cell_buttons[coord] = btn

        for r in range(5):
            table_layout.setRowStretch(r, 1)

        for c in range(7):
            table_layout.setColumnStretch(c, 1)

        layout.addLayout(table_layout, 1)

        return panel

    # ==================================================
    # 右侧主面板
    # ==================================================

    def build_right_panel(self):
        panel = QFrame()
        panel.setStyleSheet(self.style_panel())

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        layout.addWidget(self.build_network_panel())
        layout.addWidget(self.build_query_panel())
        layout.addWidget(self.build_node_panel())
        layout.addWidget(self.build_status_panel())
        layout.addWidget(self.build_log_panel(), 1)

        return panel

    # ==================================================
    # 网络配置
    # ==================================================

    def build_network_panel(self):
        panel = QFrame()
        panel.setStyleSheet(self.style_sub_panel())

        layout = QGridLayout(panel)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        title = QLabel("网络配置")
        title.setFont(QFont("Microsoft YaHei UI", 11, QFont.Bold))
        title.setStyleSheet("border: none; color: #37474F;")
        layout.addWidget(title, 0, 0, 1, 4)

        self.recv_port_input = QLineEdit("8888")
        self.ip_input = QLineEdit("192.168.1.100")
        self.send_port_input = QLineEdit("8889")

        for edit in [self.recv_port_input, self.ip_input, self.send_port_input]:
            edit.setFixedHeight(30)
            edit.setFont(QFont("Consolas", 10))
            edit.setStyleSheet(self.style_line_edit())

        label_recv = QLabel("收:")
        label_ip = QLabel("IP:")
        label_send = QLabel("发:")

        for label in [label_recv, label_ip, label_send]:
            label.setFont(QFont("Microsoft YaHei UI", 9))
            label.setStyleSheet("border: none;")

        layout.addWidget(label_recv, 1, 0)
        layout.addWidget(self.recv_port_input, 1, 1)

        layout.addWidget(label_send, 1, 2)
        layout.addWidget(self.send_port_input, 1, 3)

        layout.addWidget(label_ip, 2, 0)
        layout.addWidget(self.ip_input, 2, 1, 1, 3)

        self.apply_network_btn = QPushButton("应用网络配置")
        self.apply_network_btn.setFixedHeight(32)
        self.apply_network_btn.setFont(QFont("Microsoft YaHei UI", 10, QFont.Bold))
        self.apply_network_btn.setStyleSheet(self.style_small_blue_button())
        self.apply_network_btn.clicked.connect(self.on_apply_network_clicked)

        layout.addWidget(self.apply_network_btn, 3, 0, 1, 4)

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
            drone_ip = "192.168.1.100"
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

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        title = QLabel("编号查询")
        title.setFont(QFont("Microsoft YaHei UI", 13, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("border: none; color: #1565C0;")
        layout.addWidget(title)

        self.query_input = QLineEdit()
        self.query_input.setReadOnly(True)
        self.query_input.setAlignment(Qt.AlignCenter)
        self.query_input.setFont(QFont("Consolas", 22, QFont.Bold))
        self.query_input.setFixedHeight(44)
        self.query_input.setStyleSheet("""
            QLineEdit {
                background-color: white;
                color: #111111;
                border: 2px solid #90CAF9;
                border-radius: 8px;
                padding: 3px;
            }
        """)
        layout.addWidget(self.query_input)

        keypad_layout = QGridLayout()
        keypad_layout.setSpacing(5)

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
            btn.setFixedHeight(42)
            btn.setFont(QFont("Microsoft YaHei UI", 13, QFont.Bold))
            btn.clicked.connect(lambda checked=False, k=key: self.handle_keypad(k))

            if key == "enter":
                btn.setStyleSheet(self.style_key_enter())
            elif key == "del":
                btn.setStyleSheet(self.style_key_del())
            else:
                btn.setStyleSheet(self.style_key_number())

            keypad_layout.addWidget(btn, row, col)

        layout.addLayout(keypad_layout)

        self.query_output = QLabel("查询结果显示区")
        self.query_output.setFont(QFont("Microsoft YaHei UI", 12, QFont.Bold))
        self.query_output.setAlignment(Qt.AlignCenter)
        self.query_output.setWordWrap(True)
        self.query_output.setMinimumHeight(44)
        self.query_output.setStyleSheet("""
            QLabel {
                background-color: #263238;
                color: #00E676;
                border-radius: 8px;
                border: none;
                padding: 5px;
            }
        """)
        layout.addWidget(self.query_output)

        self.target_info = QLabel("目标信息：暂无")
        self.target_info.setFont(QFont("Microsoft YaHei UI", 10, QFont.Bold))
        self.target_info.setAlignment(Qt.AlignCenter)
        self.target_info.setWordWrap(True)
        self.target_info.setMinimumHeight(36)
        self.target_info.setStyleSheet("""
            QLabel {
                background-color: #FFF8E1;
                color: #E65100;
                border-radius: 8px;
                border: 1px solid #FFE082;
                padding: 5px;
            }
        """)
        layout.addWidget(self.target_info)

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
        panel = QFrame()
        panel.setStyleSheet(self.style_sub_panel())

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        title = QLabel("节点状态")
        title.setFont(QFont("Microsoft YaHei UI", 12, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("border: none; color: #37474F;")

        layout.addWidget(title)

        node_layout = QGridLayout()
        node_layout.setSpacing(6)

        self.add_node_indicator(node_layout, "视觉", "VISION", 0, 0)
        self.add_node_indicator(node_layout, "通信", "RECEIVER", 0, 1)
        self.add_node_indicator(node_layout, "飞控", "FSM", 0, 2)

        layout.addLayout(node_layout)

        return panel

    def add_node_indicator(self, layout, label_text: str, key: str, row: int, col: int):
        item = QFrame()
        item.setStyleSheet("border: none;")

        vbox = QVBoxLayout(item)
        vbox.setContentsMargins(0, 0, 0, 0)
        vbox.setSpacing(2)

        light = QLabel("●")
        light.setAlignment(Qt.AlignCenter)
        light.setFont(QFont("Arial", 24, QFont.Bold))
        light.setStyleSheet("""
            QLabel {
                color: #BDBDBD;
                border: none;
            }
        """)

        label = QLabel(label_text)
        label.setAlignment(Qt.AlignCenter)
        label.setFont(QFont("Microsoft YaHei UI", 9, QFont.Bold))
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

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(5)

        self.comm_status_label = QLabel("通信：待连接")
        self.comm_status_label.setFont(QFont("Microsoft YaHei UI", 10, QFont.Bold))
        self.comm_status_label.setWordWrap(True)
        self.comm_status_label.setStyleSheet("""
            QLabel {
                color: #1565C0;
                border: none;
            }
        """)

        self.task_status_label = QLabel("任务：待启动")
        self.task_status_label.setFont(QFont("Microsoft YaHei UI", 10, QFont.Bold))
        self.task_status_label.setWordWrap(True)
        self.task_status_label.setStyleSheet("""
            QLabel {
                color: #2E7D32;
                border: none;
            }
        """)

        layout.addWidget(self.comm_status_label)
        layout.addWidget(self.task_status_label)

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
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(5)

        title = QLabel("日志")
        title.setFont(QFont("Microsoft YaHei UI", 12, QFont.Bold))
        title.setStyleSheet("""
            QLabel {
                color: #37474F;
                border: none;
            }
        """)

        self.log_list = QListWidget()
        self.log_list.setStyleSheet("""
            QListWidget {
                background-color: white;
                border: 1px solid #CFD8DC;
                border-radius: 6px;
                color: #263238;
                font-size: 11px;
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
        panel.setStyleSheet(self.style_panel())

        layout = QGridLayout(panel)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(5)

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
            btn.setFixedHeight(30)
            btn.setFont(QFont("Microsoft YaHei UI", 10, QFont.Bold))
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
        layout.addWidget(self.clear_log_btn, 0, 5, 1, 2)

        layout.addWidget(self.task2_scan_btn, 1, 0)
        layout.addWidget(self.task2_start_btn, 1, 1)
        layout.addWidget(self.reset_btn, 1, 5, 1, 2)
        layout.addWidget(self.emergency_stop_btn, 1, 7)

        for i in range(8):
            layout.setColumnStretch(i, 1)

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
