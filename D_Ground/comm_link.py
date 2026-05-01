#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
comm_link.py

D题：立体货架盘点无人机地面站通信模块

职责：
1. UDP 后台监听
2. UDP 指令发送
3. 协议解析
4. 通过 PyQt 信号通知主控制器

本文件不关心 UI 显示。
本文件不直接修改界面。

推荐协议：

无人机 -> 地面站：

STATUS:VISION_READY
STATUS:RECEIVER_READY
STATUS:FSM_READY
STATUS:BOOT_WAITING
STATUS:TASK1_RUNNING
STATUS:TASK2_RUNNING
STATUS:LANDING
STATUS:MISSION_FINISHED

SCAN:A1:17
SCAN:B3:05

TARGET_ID:17
TARGET_RESULT:C4:17:OK

REPLY:TASK1_STARTED
REPLY:TASK2_STARTED

地面站 -> 无人机：

CMD:START_TASK1
CMD:START_TASK2
CMD:PING
CMD:LAUNCH
"""

import socket
from PyQt5.QtCore import QThread, pyqtSignal


class UDPComm(QThread):
    """
    UDP 通信线程
    """

    # 通信状态，例如 UDP 启动成功、发送失败等
    comm_status = pyqtSignal(str)

    # 未识别的原始数据
    raw_received = pyqtSignal(str)

    # STATUS:xxx
    status_received = pyqtSignal(str)

    # STATUS:xxx，并附带来源 IP
    status_received_with_addr = pyqtSignal(str, str)

    # SCAN:A1:17
    scan_received = pyqtSignal(str, str)

    # TARGET_ID:17
    target_id_received = pyqtSignal(str)

    # TARGET_RESULT:C4:17:OK
    target_result_received = pyqtSignal(str, str, str)

    # REPLY:xxx
    reply_received = pyqtSignal(str)

    def __init__(self, local_port=8888, drone_ip="192.168.1.100", drone_port=8889):
        super().__init__()

        self.local_port = local_port
        self.drone_ip = drone_ip
        self.drone_port = drone_port

        self.sock = None
        self.is_running = False

    # ==================================================
    # 线程入口
    # ==================================================

    def run(self):
        """
        后台 UDP 接收线程
        """
        self.is_running = True

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(("0.0.0.0", self.local_port))

            self.comm_status.emit(f"UDP监听已启动，端口 {self.local_port}")

        except Exception as e:
            self.comm_status.emit(f"UDP启动失败：{e}")
            return

        while self.is_running:
            try:
                data, addr = self.sock.recvfrom(4096)
                message = data.decode("utf-8", errors="replace").strip()

                if not message:
                    continue

                self.parse_message(message, addr)

            except OSError:
                if not self.is_running:
                    break

            except Exception as e:
                self.comm_status.emit(f"接收异常：{e}")

        self.comm_status.emit("UDP监听已停止")

    # ==================================================
    # 协议解析
    # ==================================================

    def parse_message(self, message: str, addr):
        """
        解析 UDP 消息
        """
        src_ip = ""

        try:
            src_ip = addr[0]
        except Exception:
            src_ip = ""

        # STATUS:xxx
        if message.startswith("STATUS:"):
            payload = message.split(":", 1)[1].strip()

            if payload:
                self.status_received.emit(payload)
                self.status_received_with_addr.emit(payload, src_ip)
            else:
                self.raw_received.emit(message)

            return

        # SCAN:A1:17
        if message.startswith("SCAN:"):
            parts = message.split(":")

            if len(parts) == 3:
                coord = parts[1].strip()
                item_id = parts[2].strip()

                if coord and item_id:
                    self.scan_received.emit(coord, item_id)
                else:
                    self.raw_received.emit(message)
            else:
                self.raw_received.emit(message)

            return

        # TARGET_ID:17
        if message.startswith("TARGET_ID:"):
            payload = message.split(":", 1)[1].strip()

            if payload:
                self.target_id_received.emit(payload)
            else:
                self.raw_received.emit(message)

            return

        # TARGET_RESULT:C4:17:OK
        if message.startswith("TARGET_RESULT:"):
            parts = message.split(":")

            if len(parts) == 4:
                coord = parts[1].strip()
                item_id = parts[2].strip()
                result = parts[3].strip()

                if coord and item_id and result:
                    self.target_result_received.emit(coord, item_id, result)
                else:
                    self.raw_received.emit(message)
            else:
                self.raw_received.emit(message)

            return

        # REPLY:xxx
        if message.startswith("REPLY:"):
            payload = message.split(":", 1)[1].strip()

            if payload:
                self.reply_received.emit(payload)
            else:
                self.raw_received.emit(message)

            return

        # 兼容简化格式：A1:17
        if ":" in message:
            parts = message.split(":")

            if len(parts) == 2:
                coord = parts[0].strip()
                item_id = parts[1].strip()

                if self._looks_like_coord(coord) and item_id.isdigit():
                    self.scan_received.emit(coord, item_id)
                    return

        self.raw_received.emit(message)

    def _looks_like_coord(self, coord: str) -> bool:
        """
        判断字符串是否像 A1~D6
        """
        if not coord:
            return False

        coord = coord.strip().upper()

        if len(coord) != 2:
            return False

        row = coord[0]
        col = coord[1]

        return row in ["A", "B", "C", "D"] and col in ["1", "2", "3", "4", "5", "6"]

    # ==================================================
    # 发送接口
    # ==================================================

    def send_data(self, data_str: str) -> bool:
        """
        发送 UDP 数据到无人机通信节点
        """
        try:
            if self.sock is None:
                self.comm_status.emit("发送失败：UDP尚未启动")
                return False

            payload = data_str.encode("utf-8")
            self.sock.sendto(payload, (self.drone_ip, self.drone_port))

            self.comm_status.emit(f"已发送：{data_str}")
            return True

        except Exception as e:
            self.comm_status.emit(f"发送失败：{e}")
            return False

    def update_target(self, drone_ip: str, drone_port: int = None):
        """
        更新无人机目标地址
        """
        if drone_ip:
            self.drone_ip = drone_ip

        if drone_port is not None:
            self.drone_port = int(drone_port)

        self.comm_status.emit(f"目标地址更新为 {self.drone_ip}:{self.drone_port}")

    # ==================================================
    # 停止线程
    # ==================================================

    def stop(self):
        """
        安全停止 UDP 线程
        """
        self.is_running = False

        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass

        self.sock = None