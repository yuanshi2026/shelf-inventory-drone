#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
route_map_dialog.py

任务2：定点盘点航线图弹窗

功能：
1. 显示 400cm × 500cm 场地示意图
2. 显示起飞点、降落点、两个双面货架
3. 显示 A/B/C/D 四个面与二维码点位
4. 高亮目标坐标
5. 显示定点盘点规划航线
"""

import math
from PyQt5.QtWidgets import QDialog, QVBoxLayout, QLabel, QWidget
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QFont, QPolygonF


class RouteMapWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.target_id = None
        self.target_coord = None

        self.setMinimumSize(620, 420)

    def set_target(self, target_id=None, target_coord=None):
        self.target_id = str(target_id) if target_id is not None else None
        self.target_coord = str(target_coord).upper() if target_coord else None
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()

        margin = 35
        field = self._field_rect(w, h, margin)

        painter.fillRect(self.rect(), QColor("#F7F9FB"))

        self._draw_title(painter, w)
        self._draw_field(painter, field)
        self._draw_shelves(painter, field)
        self._draw_points(painter, field)

        if self.target_coord:
            self._draw_route(painter, field)
            self._draw_target(painter, field)
        else:
            self._draw_no_target(painter, field)

    def _field_rect(self, w, h, margin):
        # 留出顶部标题和底部说明区域
        top = margin + 25
        bottom = h - margin - 20
        left = margin
        right = w - margin
        return left, top, right, bottom

    def _draw_title(self, painter, w):
        painter.setPen(QPen(QColor("#263238"), 1))
        painter.setFont(QFont("Microsoft YaHei UI", 13, QFont.Bold))
        painter.drawText(
            0, 8, w, 24,
            Qt.AlignCenter,
            "任务2定点盘点航线图"
        )

    def _draw_field(self, painter, field):
        left, top, right, bottom = field

        painter.setPen(QPen(QColor("#111111"), 3))
        painter.setBrush(Qt.NoBrush)
        painter.drawRect(left, top, right - left, bottom - top)

        painter.setPen(QPen(QColor("#666666"), 1))
        painter.setFont(QFont("Microsoft YaHei UI", 8))
        painter.drawText(left + 8, top + 18, "作业区 400cm × 500cm")

        # 起飞点 S：左下角
        sx, sy = self._start_point(field)
        painter.setPen(QPen(QColor("#000000"), 2))
        painter.setBrush(QBrush(QColor("#EAEAEA")))
        painter.drawRect(int(sx - 15), int(sy - 15), 30, 30)
        painter.setPen(QPen(QColor("#000000"), 1))
        painter.setFont(QFont("Microsoft YaHei UI", 10, QFont.Bold))
        painter.drawText(int(sx - 7), int(sy + 5), "S")

        # 降落点 L：右上角
        lx, ly = self._land_point(field)
        painter.setPen(QPen(QColor("#000000"), 2))
        painter.setBrush(QBrush(QColor("#EAEAEA")))
        painter.drawEllipse(QPointF(lx, ly), 18, 18)
        painter.setPen(QPen(QColor("#000000"), 1))
        painter.drawText(int(lx - 6), int(ly + 5), "L")

    def _draw_shelves(self, painter, field):
        left, top, right, bottom = field

        shelf1_x = left + (right - left) * 0.30
        shelf2_x = left + (right - left) * 0.68

        shelf_top = top + (bottom - top) * 0.27
        shelf_bottom = top + (bottom - top) * 0.72

        painter.setPen(QPen(QColor("#222222"), 4))
        painter.drawLine(int(shelf1_x), int(shelf_top), int(shelf1_x), int(shelf_bottom))
        painter.drawLine(int(shelf2_x), int(shelf_top), int(shelf2_x), int(shelf_bottom))

        painter.setPen(QPen(QColor("#263238"), 1))
        painter.setFont(QFont("Microsoft YaHei UI", 12, QFont.Bold))

        painter.drawText(int(shelf1_x - 55), int(shelf_top - 12), "A")
        painter.drawText(int(shelf1_x + 40), int(shelf_top - 12), "B")
        painter.drawText(int(shelf2_x - 55), int(shelf_top - 12), "C")
        painter.drawText(int(shelf2_x + 40), int(shelf_top - 12), "D")

    def _draw_points(self, painter, field):
        for face in ["A", "B", "C", "D"]:
            for idx in range(1, 7):
                coord = f"{face}{idx}"
                x, y = self._coord_point(field, coord)

                if coord == self.target_coord:
                    continue

                painter.setPen(QPen(QColor("#111111"), 2))
                painter.setBrush(QBrush(QColor("#FFFFFF")))
                painter.drawEllipse(QPointF(x, y), 11, 11)

                painter.setPen(QPen(QColor("#666666"), 1))
                painter.setFont(QFont("Microsoft YaHei UI", 7))
                painter.drawText(int(x - 7), int(y + 4), str(idx))

    def _draw_target(self, painter, field):
        x, y = self._coord_point(field, self.target_coord)

        painter.setPen(QPen(QColor("#D32F2F"), 3))
        painter.setBrush(QBrush(QColor("#FFD54F")))
        painter.drawEllipse(QPointF(x, y), 15, 15)

        painter.setPen(QPen(QColor("#B71C1C"), 2))
        painter.setFont(QFont("Microsoft YaHei UI", 10, QFont.Bold))
        painter.drawText(int(x + 18), int(y + 5), f"{self.target_coord}")

    def _draw_route(self, painter, field):
        start = self._start_point(field)
        land = self._land_point(field)
        target = self._coord_point(field, self.target_coord)
        entry = self._entry_point(field, self.target_coord[0])

        _, top, _, _ = field
        safe_y = top + 40

        # 仅使用水平/竖直线，先到目标面入口，再到目标点，最后走上方安全通道到降落点
        points = [
            start,
            (entry[0], start[1]),
            entry,
            (target[0], entry[1]),
            target,
            (target[0], safe_y),
            (land[0], safe_y),
            land,
        ]

        painter.setPen(QPen(QColor("#1976D2"), 3))
        painter.setBrush(QBrush(QColor("#1976D2")))

        for i in range(len(points) - 1):
            self._draw_arrow(painter, points[i], points[i + 1])

    def _draw_arrow(self, painter, p1, p2):
        x1, y1 = p1
        x2, y2 = p2

        painter.drawLine(int(x1), int(y1), int(x2), int(y2))

        angle = math.atan2(y2 - y1, x2 - x1)
        arrow_len = 12
        arrow_angle = math.pi / 7

        p_left = QPointF(
            x2 - arrow_len * math.cos(angle - arrow_angle),
            y2 - arrow_len * math.sin(angle - arrow_angle)
        )
        p_right = QPointF(
            x2 - arrow_len * math.cos(angle + arrow_angle),
            y2 - arrow_len * math.sin(angle + arrow_angle)
        )

        arrow_head = QPolygonF([
            QPointF(x2, y2),
            p_left,
            p_right
        ])

        painter.drawPolygon(arrow_head)

    def _draw_no_target(self, painter, field):
        left, top, right, bottom = field

        painter.setPen(QPen(QColor("#D32F2F"), 1))
        painter.setFont(QFont("Microsoft YaHei UI", 14, QFont.Bold))
        painter.drawText(
            left, top, right - left, bottom - top,
            Qt.AlignCenter,
            "未找到目标坐标，无法规划航线"
        )

    def _start_point(self, field):
        left, top, right, bottom = field
        return left + 60, bottom - 45

    def _land_point(self, field):
        left, top, right, bottom = field
        return right - 75, top + 55

    def _entry_point(self, field, face):
        left, top, right, bottom = field

        shelf1_x = left + (right - left) * 0.30
        shelf2_x = left + (right - left) * 0.68
        mid_y = top + (bottom - top) * 0.50

        if face == "A":
            return shelf1_x - 75, mid_y
        if face == "B":
            return shelf1_x + 75, mid_y
        if face == "C":
            return shelf2_x - 75, mid_y
        if face == "D":
            return shelf2_x + 75, mid_y

        return left + 60, bottom - 45

    def _coord_point(self, field, coord):
        """
        把 A1~D6 映射到示意图上的点。
        每个面画成 2列 × 3行。
        """
        left, top, right, bottom = field

        face = coord[0]
        try:
            idx = int(coord[1:])
        except Exception:
            idx = 1

        idx = max(1, min(6, idx))

        shelf1_x = left + (right - left) * 0.30
        shelf2_x = left + (right - left) * 0.68

        base_top = top + (bottom - top) * 0.33
        row_gap = (bottom - top) * 0.13
        col_gap = 26

        # 位置规则：每个面为 2 列 × 3 行，且 1/2/3 更靠近板子
        # A/C（左侧面）:
        # 4 1
        # 5 2
        # 6 3
        # B/D（右侧面）:
        # 3 6
        # 2 5
        # 1 4
        if face in ("A", "C"):
            row_map = {1: 0, 2: 1, 3: 2, 4: 0, 5: 1, 6: 2}
            near_map = {1: True, 2: True, 3: True, 4: False, 5: False, 6: False}
        else:
            row_map = {1: 2, 2: 1, 3: 0, 4: 2, 5: 1, 6: 0}
            near_map = {1: True, 2: True, 3: True, 4: False, 5: False, 6: False}

        row = row_map.get(idx, 0)
        is_near = near_map.get(idx, True)

        if face == "A":
            x_near = shelf1_x - 44
            x_far = x_near - col_gap
            x = x_near if is_near else x_far
        elif face == "B":
            x_near = shelf1_x + 44
            x_far = x_near + col_gap
            x = x_near if is_near else x_far
        elif face == "C":
            x_near = shelf2_x - 44
            x_far = x_near - col_gap
            x = x_near if is_near else x_far
        elif face == "D":
            x_near = shelf2_x + 44
            x_far = x_near + col_gap
            x = x_near if is_near else x_far
        else:
            x = left + 60

        y = base_top + row * row_gap

        return x, y


class RouteMapDialog(QDialog):
    def __init__(self, target_id=None, target_coord=None, parent=None):
        super().__init__(parent)

        self.target_id = str(target_id) if target_id is not None else None
        self.target_coord = str(target_coord).upper() if target_coord else None

        self.setWindowTitle("任务2定点盘点航线图")
        self.resize(720, 540)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        self.map_widget = RouteMapWidget(self)
        self.info_label = QLabel()
        self.info_label.setAlignment(Qt.AlignCenter)
        self.info_label.setFont(QFont("Microsoft YaHei UI", 10, QFont.Bold))
        self.info_label.setStyleSheet("""
            QLabel {
                color: #263238;
                background-color: #FFFFFF;
                border: 1px solid #CFD8DC;
                border-radius: 6px;
                padding: 6px;
            }
        """)

        layout.addWidget(self.map_widget, 1)
        layout.addWidget(self.info_label)

        self.set_target(self.target_id, self.target_coord)

    def set_target(self, target_id=None, target_coord=None):
        self.target_id = str(target_id) if target_id is not None else None
        self.target_coord = str(target_coord).upper() if target_coord else None

        self.map_widget.set_target(self.target_id, self.target_coord)

        if self.target_id and self.target_coord:
            face = self.target_coord[0]
            route_text = f"起飞点 S → {face}面入口 → {self.target_coord} → 降落点 L"
            self.info_label.setText(
                f"目标编号：{self.target_id}    "
                f"目标坐标：{self.target_coord}    "
                f"规划航线：{route_text}"
            )
        elif self.target_id:
            self.info_label.setText(
                f"目标编号：{self.target_id}    未找到目标坐标，无法规划航线"
            )
        else:
            self.info_label.setText("暂无任务2目标编号")