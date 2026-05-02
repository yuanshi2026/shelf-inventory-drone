#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import socket
import threading
import time

import rospy
from std_msgs.msg import Bool, String


class GroundUDPBridge:
    def __init__(self):
        rospy.init_node("ground_udp_bridge_node")

        self.local_ip = rospy.get_param("~local_ip", "0.0.0.0")          # 无人机本机监听 IP
        self.local_port = int(rospy.get_param("~local_port", 8889))      # 无人机本机监听端口

        self.ground_ip = rospy.get_param("~ground_ip", "192.168.1.20")   # 树莓派地面站 IP
        self.ground_port = int(rospy.get_param("~ground_port", 8888))    # 树莓派地面站监听端口

        self.heartbeat_interval = float(
            rospy.get_param("~heartbeat_interval", 2.0)
        )  # 状态心跳发送间隔

        self.task1_running = False       # 任务 1 是否正在执行
        self.task2_running = False       # 任务 2 是否正在执行
        self.latest_fsm_state = "IDLE"   # 最近一次 FSM 状态
        self.latest_target_id = -1       # 任务 2 目标货物编号

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)     # UDP 通信 socket
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.local_ip, self.local_port))

        self.start_pub = rospy.Publisher(
            "/uav/start",
            Bool,
            queue_size=5
        )  # 任务 1 启动话题

        self.stop_pub = rospy.Publisher(
            "/uav/stop",
            Bool,
            queue_size=5
        )  # 紧急停止话题

        self.task2_start_pub = rospy.Publisher(
            "/uav/start_task2",
            Bool,
            queue_size=5
        )  # 任务 2 启动话题，后续任务 2 FSM 可订阅

        self.target_scan_pub = rospy.Publisher(
            "/uav/scan_target",
            Bool,
            queue_size=5
        )  # 任务 2 起飞前目标扫描触发话题

        rospy.Subscriber(
            "/fsm/state",
            String,
            self.fsm_state_callback,
            queue_size=20
        )  # 订阅 FSM 状态

        rospy.Subscriber(
            "/inventory/result",
            String,
            self.inventory_result_callback,
            queue_size=50
        )  # 订阅任务 1 盘点结果

        rospy.Subscriber(
            "/target/id",
            String,
            self.target_id_callback,
            queue_size=10
        )  # 订阅任务 2 目标编号结果

        rospy.Subscriber(
            "/target/result",
            String,
            self.target_result_callback,
            queue_size=10
        )  # 订阅任务 2 定向盘点结果

        self.running = True              # UDP 线程运行标志

        self.recv_thread = threading.Thread(
            target=self.recv_loop,
            daemon=True
        )  # UDP 接收线程

        self.heartbeat_thread = threading.Thread(
            target=self.heartbeat_loop,
            daemon=True
        )  # 状态心跳线程

        self.recv_thread.start()
        self.heartbeat_thread.start()

        rospy.loginfo("ground_udp_bridge_node started.")
        rospy.loginfo("UAV listen UDP: %s:%d", self.local_ip, self.local_port)
        rospy.loginfo("Ground target UDP: %s:%d", self.ground_ip, self.ground_port)

    def send_udp(self, text):
        """发送 UDP 字符串给树莓派地面站。"""

        try:
            self.sock.sendto(
                text.encode("utf-8"),
                (self.ground_ip, self.ground_port)
            )
            rospy.loginfo("UDP TX: %s", text)

        except Exception as e:
            rospy.logwarn("UDP send failed: %s", str(e))

    # ==================== 本次修改 1：start/stop 指令重复发布，降低漏收概率 ====================
    def publish_bool_repeated(self, pub, value, times=5, interval=0.05):
        """重复发布 Bool 指令。"""

        for _ in range(times):  # 连续发布多次
            pub.publish(Bool(data=value))
            time.sleep(interval)
    # ===============================================================================

    def recv_loop(self):
        """持续接收地面站 UDP 指令。"""

        while self.running and not rospy.is_shutdown():  # 节点未关闭时持续接收
            try:
                data, addr = self.sock.recvfrom(4096)

                text = data.decode("utf-8", errors="replace").strip()
                src_ip = addr[0]

                if not text:  # 空消息直接忽略
                    continue

                self.ground_ip = src_ip  # 收到地面站消息后自动更新地面站 IP

                rospy.loginfo("UDP RX from %s:%d -> %s", addr[0], addr[1], text)
                self.handle_command(text)

            except Exception as e:
                if self.running:  # 非主动关闭时才打印异常
                    rospy.logwarn("UDP receive failed: %s", str(e))

    def heartbeat_loop(self):
        """周期性发送无人机节点状态。"""

        while self.running and not rospy.is_shutdown():  # 节点运行时持续发送状态
            self.send_udp("STATUS:RECEIVER_READY")
            time.sleep(0.05)

            self.send_udp("STATUS:FSM_READY")
            time.sleep(0.05)

            self.send_udp("STATUS:VISION_READY")
            time.sleep(0.05)

            if not self.task1_running and not self.task2_running:  # 未执行任务时等待启动
                self.send_udp("STATUS:BOOT_WAITING")

            time.sleep(self.heartbeat_interval)

    def handle_command(self, text):
        """处理地面站发来的 CMD 指令。"""

        if text == "CMD:START_TASK1":  # 启动任务 1：遍历盘点
            self.task1_running = True
            self.task2_running = False

            self.send_udp("REPLY:TASK1_STARTED")
            self.send_udp("STATUS:TASK1_RUNNING")

            # ==================== 本次修改 2：启动指令重复发布 ====================
            self.publish_bool_repeated(self.start_pub, True)
            # ==================================================================
            return

        if text == "CMD:START_TASK2":  # 启动任务 2：定向盘点
            self.task1_running = False
            self.task2_running = True

            self.send_udp("REPLY:TASK2_STARTED")
            self.send_udp("STATUS:TASK2_RUNNING")

            self.task2_start_pub.publish(Bool(data=True))
            return

        if text == "CMD:TASK2_SCAN_TARGET":  # 任务 2 起飞前扫描抽取二维码
            self.send_udp("STATUS:SCANNING")
            self.target_scan_pub.publish(Bool(data=True))
            return

        if text == "CMD:EMERGENCY_STOP":  # 紧急停止
            self.task1_running = False
            self.task2_running = False

            self.send_udp("STATUS:ERROR")
            self.send_udp("REPLY:EMERGENCY_STOP_OK")

            # ==================== 本次修改 3：停止指令重复发布 ====================
            self.publish_bool_repeated(self.stop_pub, True)
            # ==================================================================
            return

        if text == "CMD:PING":  # 通信测试指令
            self.send_udp("REPLY:PONG")
            return

        self.send_udp("REPLY:UNKNOWN_CMD")

    def fsm_state_callback(self, msg):
        """把 FSM 状态转换成地面站 STATUS 消息。"""

        state = msg.data.strip()
        self.latest_fsm_state = state

        if state == "IDLE":  # FSM 等待启动
            self.send_udp("STATUS:BOOT_WAITING")

        elif state == "TAKEOFF":  # 正在起飞
            self.send_udp("STATUS:TAKEOFF")

        elif state == "GOTO_MISSION_POINT":  # 正在执行航线
            self.send_udp("STATUS:MISSION_RUNNING")

        elif state in ["SEARCH_QR", "ALIGN_QR", "LASER_FLASH", "SEND_RESULT"]:  # 正在扫描
            self.send_udp("STATUS:SCANNING")

        elif state == "NEXT_POINT":  # 当前点完成，进入下一点
            if self.task1_running:
                self.send_udp("STATUS:TASK1_RUNNING")
            elif self.task2_running:
                self.send_udp("STATUS:TASK2_RUNNING")

        elif state in ["RETURN_LAND", "LAND"]:  # 正在降落
            self.send_udp("STATUS:LANDING")

        elif state == "FINISH":  # 任务完成
            self.send_udp("STATUS:MISSION_FINISHED")
            self.send_udp("STATUS:TASK_FINISHED")
            self.task1_running = False
            self.task2_running = False

        elif state == "EMERGENCY_STOP":  # FSM 急停
            self.send_udp("STATUS:ERROR")

    def inventory_result_callback(self, msg):
        """把任务 1 盘点结果转换成 SCAN:<坐标>:<编号>。"""

        try:
            data = json.loads(msg.data)

            goods_id = int(data.get("id", -1))          # 货物编号
            face = str(data.get("face", "-")).upper()   # 货架面 A/B/C/D
            slot = int(data.get("slot", 0))             # 货架固定位置 1~6
            status = str(data.get("status", "OK"))      # 识别状态

            if status != "OK":  # 失败结果暂不写入地面站表格
                self.send_udp("STATUS:SCANNING")
                return

            if goods_id < 1 or goods_id > 24:  # 货物编号异常时忽略
                rospy.logwarn("Invalid goods_id: %d", goods_id)
                return

            if face not in ["A", "B", "C", "D"]:  # 货架面异常时忽略
                rospy.logwarn("Invalid face: %s", face)
                return

            if slot < 1 or slot > 6:  # 坐标编号异常时忽略
                rospy.logwarn("Invalid slot: %d", slot)
                return

            coord = "{}{}".format(face, slot)         # 坐标，例如 A1
            udp_text = "SCAN:{}:{}".format(coord, goods_id)

            self.send_udp(udp_text)
            self.send_udp("REPLY:SCAN_OK")

        except Exception as e:
            rospy.logwarn("Parse inventory result failed: %s", str(e))

    def target_id_callback(self, msg):
        """把任务 2 目标编号发送给地面站。"""

        text = msg.data.strip()

        try:
            target_id = int(text)
            self.latest_target_id = target_id

            if target_id < 1 or target_id > 24:  # 目标编号异常时忽略
                rospy.logwarn("Invalid target id: %d", target_id)
                return

            self.send_udp("TARGET_ID:{}".format(target_id))

        except Exception as e:
            rospy.logwarn("Parse target id failed: %s", str(e))

    def target_result_callback(self, msg):
        """把任务 2 结果发送给地面站。"""

        try:
            data = json.loads(msg.data)

            goods_id = int(data.get("id", -1))          # 目标货物编号
            face = str(data.get("face", "-")).upper()   # 目标所在货架面
            slot = int(data.get("slot", 0))             # 目标所在位置
            result = str(data.get("result", "OK"))      # 定向盘点结果

            coord = "{}{}".format(face, slot)
            udp_text = "TARGET_RESULT:{}:{}:{}".format(coord, goods_id, result)

            self.send_udp(udp_text)

        except Exception as e:
            rospy.logwarn("Parse target result failed: %s", str(e))

    def close(self):
        """关闭 UDP socket。"""

        self.running = False

        try:
            self.sock.close()
        except Exception:
            pass


if __name__ == "__main__":
    bridge = None

    try:
        bridge = GroundUDPBridge()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    finally:
        if bridge is not None:
            bridge.close()