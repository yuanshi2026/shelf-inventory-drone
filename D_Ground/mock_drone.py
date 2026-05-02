#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
mock_drone.py

D题：立体货架盘点无人机系统 - 虚拟无人机 / 仿真节点

用途：
1. 和 ground_station.py 同时运行
2. 监听地面站发来的 CMD:START_TASK1
3. 模拟无人机按 A1~A6、B1~B6、C1~C6、D1~D6 顺序盘点
4. 每次任务随机打乱 1~24 编号
5. 向地面站发送 SCAN:坐标:编号
6. 重复点击任务1时，会重新生成随机编号并覆盖地面站原结果

默认通信：
地面站监听端口：8888
mock_drone 监听端口：8889
"""

import socket
import time
import random
import threading


# =========================
# 配置区
# =========================

GROUND_STATION_IP = "127.0.0.1"
GROUND_STATION_PORT = 8888

MOCK_DRONE_LISTEN_IP = "0.0.0.0"
MOCK_DRONE_LISTEN_PORT = 8889

# 每个格子之间的模拟间隔，单位：秒
SCAN_INTERVAL = 0.08

# 是否周期性发送 BOOT_WAITING 状态
ENABLE_HEARTBEAT = True
HEARTBEAT_INTERVAL = 3.0


# =========================
# D题固定坐标顺序
# =========================

def build_d_task1_coords():
    """
    生成 D题任务1遍历顺序：
    A1 A2 A3 A4 A5 A6
    B1 B2 B3 B4 B5 B6
    C1 C2 C3 C4 C5 C6
    D1 D2 D3 D4 D5 D6
    """
    coords = []

    for row in ["A", "B", "C", "D"]:
        for col in range(1, 7):
            coords.append(f"{row}{col}")

    return coords


# =========================
# MockDrone 主类
# =========================

class MockDrone:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((MOCK_DRONE_LISTEN_IP, MOCK_DRONE_LISTEN_PORT))

        self.running = True
        self.task_running = False
        self.emergency_stop = False
        self.ros_launched = False
        self.ros_launching = False
        self.task2_target_id = None

        self.ground_station_ip = GROUND_STATION_IP
        self.ground_station_port = GROUND_STATION_PORT
        self.task1_coords = build_d_task1_coords()
        self.task1_scan_map = {}

        print("========================================")
        print("🚁 D题 Mock Drone 已启动")
        print("========================================")
        print(f"监听地址：{MOCK_DRONE_LISTEN_IP}:{MOCK_DRONE_LISTEN_PORT}")
        print(f"地面站地址：{self.ground_station_ip}:{self.ground_station_port}")
        print("等待地面站发送指令 ...")
        print("========================================\n")

    # =========================
    # UDP 发送
    # =========================

    def send_to_ground(self, message: str):
        """
        向地面站发送 UDP 消息
        """
        self.sock.sendto(
            message.encode("utf-8"),
            (self.ground_station_ip, self.ground_station_port)
        )
        print(f"[TX] {message}")

    def update_ground_station_target(self, addr):
        """
        记录地面站地址，便于回传数据
        """
        try:
            ip, port = addr
        except Exception:
            return

        if ip != self.ground_station_ip or port != self.ground_station_port:
            self.ground_station_ip = ip
            self.ground_station_port = port
            print(f"[INFO] 地面站地址更新为 {ip}:{port}")

    # =========================
    # BOOT_WAITING 心跳
    # =========================

    def heartbeat_loop(self):
        """
        周期性发送 BOOT_WAITING
        用于通知地面站无人机已上线
        """
        while self.running and not self.ros_launched:
            if ENABLE_HEARTBEAT:
                self.send_to_ground("STATUS:BOOT_WAITING")
            time.sleep(HEARTBEAT_INTERVAL)

    # =========================
    # 一键启动 ROS
    # =========================

    def launch_ros(self):
        if self.ros_launching:
            return

        self.ros_launching = True

        try:
            self.send_to_ground("REPLY:LAUNCH_OK")
            time.sleep(0.2)

            self.send_to_ground("STATUS:RECEIVER_READY")
            time.sleep(0.2)
            self.send_to_ground("STATUS:VISION_READY")
            time.sleep(0.2)
            self.send_to_ground("STATUS:FSM_READY")
            time.sleep(0.2)
            self.send_to_ground("STATUS:ROS_READY")

            self.ros_launched = True
        finally:
            self.ros_launching = False

    # =========================
    # 任务1模拟
    # =========================

    def run_task1(self):
        """
        模拟 D题任务1：遍历盘点
        """
        if self.task_running:
            print("[WARN] 当前任务仍在执行，忽略重复启动指令")
            self.send_to_ground("REPLY:TASK_ALREADY_RUNNING")
            return

        self.task_running = True
        self.emergency_stop = False

        try:
            print("\n========================================")
            print("📦 开始模拟任务1：遍历盘点")
            print("========================================")

            self.send_to_ground("REPLY:TASK1_STARTED")
            time.sleep(0.07)

            self.send_to_ground("STATUS:TAKEOFF")
            time.sleep(0.17)

            self.send_to_ground("STATUS:TASK1_RUNNING")
            time.sleep(0.1)

            item_ids = list(range(1, len(self.task1_coords) + 1))
            random.shuffle(item_ids)
            self.task1_scan_map = {}

            for index, coord in enumerate(self.task1_coords):
                if self.emergency_stop:
                    self.send_to_ground("STATUS:ERROR")
                    print("[WARN] 任务1收到紧急刹停，终止执行")
                    return

                item_id = item_ids[index]

                # 模拟无人机正在扫描
                self.send_to_ground("STATUS:SCANNING")
                time.sleep(SCAN_INTERVAL)

                # 发送盘点结果
                self.task1_scan_map[str(item_id)] = coord
                self.send_to_ground(f"SCAN:{coord}:{item_id}")

                time.sleep(SCAN_INTERVAL)

            self.send_to_ground("STATUS:LANDING")
            time.sleep(0.2)

            self.send_to_ground("STATUS:TASK_FINISHED")
            self.send_to_ground("REPLY:TASK1_FINISHED")

            print("----------------------------------------")
            print("✅ 任务1模拟完成，等待下一次启动")
            print("重复点击任务1会重新随机编号并覆盖地面站表格")
            print("========================================\n")

        finally:
            self.task_running = False

    # =========================
    # 任务2模拟
    # =========================

    def run_task2(self):
        """
        任务2：定点盘点模拟
        """
        if self.task_running:
            print("[WARN] 当前任务仍在执行，忽略重复启动指令")
            self.send_to_ground("REPLY:TASK_ALREADY_RUNNING")
            return

        self.task_running = True
        self.emergency_stop = False

        try:
            print("\n🎯 收到任务2启动指令，开始模拟定点盘点")
            self.send_to_ground("REPLY:TASK2_STARTED")
            self.send_to_ground("STATUS:TASK2_RUNNING")
            time.sleep(0.3)

            if not self.task2_target_id:
                target_id = random.randint(1, 24)
                self.task2_target_id = str(target_id)
                self.send_to_ground(f"TARGET_ID:{self.task2_target_id}")
                time.sleep(0.2)

            target_id_str = str(self.task2_target_id).strip()
            if not target_id_str.isdigit():
                target_id = random.randint(1, 24)
                target_id_str = str(target_id)
            else:
                target_id = int(target_id_str)

            coord = self.task1_scan_map.get(target_id_str)
            if not coord:
                if 1 <= target_id <= len(self.task1_coords):
                    coord = self.task1_coords[target_id - 1]
                else:
                    coord = self.task1_coords[0]

            if self.emergency_stop:
                self.send_to_ground("STATUS:ERROR")
                print("[WARN] 任务2收到紧急刹停，终止执行")
                return

            self.send_to_ground("STATUS:SCANNING")
            time.sleep(SCAN_INTERVAL)
            self.send_to_ground(f"TARGET_RESULT:{coord}:{target_id}:OK")
            time.sleep(0.2)
            self.send_to_ground("STATUS:TASK_FINISHED")
        finally:
            self.task_running = False

    def run_task2_scan_target(self):
        """
        任务2.1：识别目标
        """
        target_id = random.randint(1, 24)
        self.task2_target_id = str(target_id)
        print(f"\n🎯 任务2目标识别完成：编号 {target_id}")
        self.send_to_ground(f"TARGET_ID:{target_id}")

    # =========================
    # 主接收循环
    # =========================

    def recv_loop(self):
        """
        接收地面站指令
        """
        while self.running:
            try:
                data, addr = self.sock.recvfrom(4096)
                message = data.decode("utf-8", errors="replace").strip()

                if not message:
                    continue

                print(f"[RX] 来自 {addr}: {message}")
                self.update_ground_station_target(addr)

                if message == "CMD:START_TASK1":
                    threading.Thread(
                        target=self.run_task1,
                        daemon=True
                    ).start()

                elif message == "CMD:TASK2_SCAN_TARGET":
                    threading.Thread(
                        target=self.run_task2_scan_target,
                        daemon=True
                    ).start()

                elif message == "CMD:START_TASK2":
                    threading.Thread(
                        target=self.run_task2,
                        daemon=True
                    ).start()

                elif message == "CMD:PING":
                    self.send_to_ground("REPLY:PONG")

                elif message == "CMD:LAUNCH":
                    threading.Thread(
                        target=self.launch_ros,
                        daemon=True
                    ).start()

                elif message == "CMD:EMERGENCY_STOP":
                    self.emergency_stop = True
                    self.send_to_ground("STATUS:ERROR")
                    self.send_to_ground("REPLY:EMERGENCY_STOPPED")

                else:
                    print(f"[WARN] 未识别指令：{message}")
                    self.send_to_ground(f"REPLY:UNKNOWN_CMD:{message}")

            except KeyboardInterrupt:
                self.running = False
                break

            except Exception as e:
                print(f"[ERROR] 接收异常：{e}")

    # =========================
    # 启动
    # =========================

    def run(self):
        if ENABLE_HEARTBEAT:
            threading.Thread(
                target=self.heartbeat_loop,
                daemon=True
            ).start()

        self.recv_loop()


if __name__ == "__main__":
    drone = MockDrone()
    drone.run()
