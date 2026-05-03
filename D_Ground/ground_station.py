#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ground_station.py

D题：立体货架盘点无人机地面站主控文件

职责：
1. 创建 UI
2. 创建通信线程
3. 绑定 UI 信号
4. 绑定通信信号
5. 处理业务逻辑

注意：
- 本文件不直接写复杂 UI
- 本文件不直接写 UDP socket 细节
- UI 和通信通过信号连接
"""

import json
import os
import sys
from datetime import datetime
from PyQt5.QtWidgets import QApplication

from ui_view import GroundStationUI
from comm_link import UDPComm
from route_map_dialog import RouteMapDialog


class MainController:
    COMM_WAIT_TIMEOUT_MS = 1500

    def __init__(self):
        self.ui = GroundStationUI()
        self.reset_state_cache()
        self.coord_to_id = {}
        self.id_to_coord = {}
        self.inventory_saved_logged = False
        self.data_file_path = os.path.join(os.path.dirname(__file__), "data.json")

        try:
            local_port = int(self.ui.recv_port_input.text().strip())
        except ValueError:
            local_port = 8888

        try:
            drone_port = int(self.ui.send_port_input.text().strip())
        except ValueError:
            drone_port = 8889

        drone_ip = self.ui.ip_input.text().strip() or "192.168.151.102"

        self.comm = UDPComm(
            local_port=local_port,
            drone_ip=drone_ip,
            drone_port=drone_port
        )

        self.bind_ui_signals()
        self.bind_comm_signals()
        QApplication.instance().aboutToQuit.connect(self.close)

        self.comm.start()
        self.ui.show()

        self.load_inventory_data()

        self.ui.append_log("地面站启动完成")
        self.ui.append_log("当前为 D 题基础框架版")
        self.ui.append_log("等待无人机通信节点上线")

    # ==================================================
    # 信号绑定
    # ==================================================

    def bind_ui_signals(self):
        """
        绑定 UI 发出的信号
        """
        self.ui.task1_clicked.connect(self.handle_start_task1)
        self.ui.task2_scan_clicked.connect(self.handle_task2_scan_target)
        self.ui.task2_start_clicked.connect(self.handle_start_task2)
        self.ui.ros_launch_clicked.connect(self.handle_launch_ros)
        self.ui.query_entered.connect(self.handle_query)
        self.ui.network_changed.connect(self.handle_network_changed)
        self.ui.clear_log_requested.connect(self.handle_clear_log)
        self.ui.reset_requested.connect(self.handle_reset_all)
        self.ui.stop_clicked.connect(self.handle_stop)
        self.ui.land_clicked.connect(self.handle_land)
        self.ui.route_map_clicked.connect(self.handle_show_route_map)

    def bind_comm_signals(self):
        """
        绑定通信模块发出的信号
        """
        self.comm.comm_status.connect(self.handle_comm_status)

        self.comm.status_received.connect(self.handle_status)
        self.comm.status_received_with_addr.connect(self.handle_status_with_addr)

        self.comm.scan_received.connect(self.handle_scan)
        self.comm.target_id_received.connect(self.handle_target_id)
        self.comm.target_result_received.connect(self.handle_target_result)
        self.comm.reply_received.connect(self.handle_reply)
        self.comm.raw_received.connect(self.handle_raw_data)

    # ==================================================
    # UI 事件处理
    # ==================================================

    def handle_start_task1(self):
        """
        启动任务1：遍历盘点
        """
        self.reset_inventory_data()
        self.ui.append_log("按钮：启动任务1")
        self.ui.set_task_status("已发送任务1启动指令")

        self.sync_comm_target_from_ui()
        self.comm.send_data("CMD:START_TASK1")

    def _parse_valid_task2_target_id(self, target_id: str):
        target_text = (target_id or "").strip()
        if not target_text:
            return None
        if not target_text.isdigit():
            return None

        target_number = int(target_text)
        if target_number < 1 or target_number > 24:
            return None

        return str(target_number)

    def handle_start_task2(self):
        """
        启动任务二：定点盘点
        """
        target_id = self._parse_valid_task2_target_id(self.task2_target_id)
        if not target_id:
            self.ui.append_log("任务二启动失败：目标编号无效（请先执行任务2.1并确保编号在1~24）")
            self.ui.set_task_status("任务二启动失败：目标编号无效")
            self.ui.set_task2_start_enabled(False)
            return

        self.ui.append_log("按钮：启动任务二定点盘点")
        self.ui.set_task_status("已发送任务二定点盘点启动指令")

        self.sync_comm_target_from_ui()
        self.comm.send_data(f"CMD:START_TASK2:{target_id}")
        self.ui.set_task2_start_enabled(False)

    def handle_task2_scan_target(self):
        """
        任务二：识别目标二维码
        """
        self.ui.append_log("按钮：任务二识别目标")
        self.ui.set_task_status("已发送任务二目标识别指令")

        self.sync_comm_target_from_ui()
        self.comm.send_data("CMD:TASK2_SCAN_TARGET")

    def handle_stop(self):
        self.ui.append_log("已发送刹停指令")
        self.ui.set_task_status("已发送刹停指令")
        self.sync_comm_target_from_ui()
        self.comm.send_data("CMD:EMERGENCY_STOP")

    def handle_land(self):
        self.ui.append_log("已发送降落指令")
        self.ui.set_task_status("已发送降落指令")
        self.sync_comm_target_from_ui()
        self.comm.send_data("CMD:LAND")

    def handle_show_route_map(self):
        """
        显示任务2航线图弹窗
        """
        target_id = self.task2_target_id
        target_coord = None

        if target_id:
            target_coord = self.id_to_coord.get(str(target_id))

        dialog = RouteMapDialog(target_id=target_id, target_coord=target_coord, parent=self.ui)
        dialog.exec_()

    def handle_launch_ros(self):
        """
        一键启动 ROS
        """
        self.ui.append_log("已发送ROS启动指令，等待机载端响应……")
        self.ui.set_ros_launch_state("启动中...")
        self.sync_comm_target_from_ui()
        self.comm.send_data("CMD:LAUNCH")

    def handle_query(self, item_id: str):
        """
        根据货物编号查询坐标
        """
        item_id = item_id.strip()

        if not item_id:
            self.ui.set_query_output("请输入编号")
            self.ui.append_log("查询失败：输入为空")
            return

        if not item_id.isdigit():
            self.ui.set_query_output("编号必须是数字")
            self.ui.append_log(f"查询失败：非法输入 {item_id}")
            return

        item_number = int(item_id)

        if item_number < 1 or item_number > 24:
            self.ui.set_query_output("编号范围应为 1~24")
            self.ui.append_log(f"查询失败：编号 {item_number} 超出范围")
            return

        coord = self.ui.find_coord_by_item_id(str(item_number))

        if coord:
            self.ui.set_query_output(f"编号 {item_number} 位于 {coord}")
            self.ui.highlight_cell(coord)
            self.ui.append_log(f"查询成功：编号 {item_number} 位于 {coord}")
        else:
            self.ui.set_query_output(f"未找到编号 {item_number}")
            self.ui.append_log(f"查询编号 {item_number}：未找到")

    def handle_network_changed(self, local_port: int, drone_ip: str, drone_port: int):
        """
        应用新的网络配置
        """
        self.ui.append_log(
            f"应用网络配置：本地端口 {local_port}，目标 {drone_ip}:{drone_port}"
        )

        self.stop_comm_thread()

        self.comm = UDPComm(
            local_port=local_port,
            drone_ip=drone_ip,
            drone_port=drone_port
        )

        self.bind_comm_signals()
        self.comm.start()

        self.user_ip_applied = bool(drone_ip)
        self.ui.append_log("网络配置应用完成")

    def handle_clear_log(self):
        self.ui.clear_log()
        self.ui.append_log("日志已清空")

    def handle_reset_all(self):
        self.ui.reset_all()
        self.reset_state_cache()
        self.ui.append_log("已执行全局复位")

    def reset_state_cache(self):
        self.node_ready_state = {
            "VISION": False,
            "RECEIVER": False,
            "FSM": False,
            "ROS": False,
        }
        self.last_task_status = None
        self.last_comm_status = None
        self.task2_target_id = None
        self.comm_ready = False
        self.drone_ip_seen = None
        self.drone_last_seen = None
        self.user_ip_applied = False

    def sync_comm_target_from_ui(self):
        drone_ip = self.ui.ip_input.text().strip() or "192.168.151.102"
        try:
            drone_port = int(self.ui.send_port_input.text().strip())
        except ValueError:
            drone_port = self.comm.drone_port

        if drone_ip == self.comm.drone_ip and drone_port == self.comm.drone_port:
            return

        self.comm.update_target(drone_ip, drone_port)

    def reset_inventory_data(self, reset_ui: bool = True):
        self.coord_to_id = {}
        self.id_to_coord = {}
        self.inventory_saved_logged = False

        if reset_ui:
            self.ui.reset_inventory_table()

        self.save_inventory_data()

    def update_inventory_data(self, coord: str, item_id: int):
        self.coord_to_id[coord] = int(item_id)
        self.id_to_coord[str(int(item_id))] = coord

        should_log = not self.inventory_saved_logged
        if self.save_inventory_data(log_on_success=should_log) and should_log:
            self.inventory_saved_logged = True

    def save_inventory_data(self, log_on_success: bool = False) -> bool:
        data = {
            "task": "task1",
            "updated_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "coord_to_id": self.coord_to_id,
            "id_to_coord": self.id_to_coord,
        }

        try:
            with open(self.data_file_path, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)

            if log_on_success:
                self.ui.append_log("数据已保存至 data.json")

            return True
        except Exception as e:
            self.ui.append_log(f"保存 data.json 失败：{e}")
            return False


    def load_inventory_data(self):
        if not os.path.exists(self.data_file_path):
            self.ui.append_log("未找到历史 data.json，使用空表")
            return

        try:
            with open(self.data_file_path, "r", encoding="utf-8") as f:
                data = json.load(f)

            coord_to_id = data.get("coord_to_id", {})
            if not isinstance(coord_to_id, dict):
                self.ui.append_log("历史 data.json 格式错误：coord_to_id 非字典")
                return

            loaded_count = 0
            for coord, item_id in coord_to_id.items():
                coord_key = str(coord).strip().upper()
                item_text = str(item_id).strip()

                if not self.ui.is_valid_coord(coord_key):
                    continue
                if not item_text.isdigit():
                    continue

                number = int(item_text)
                if number < 1 or number > 24:
                    continue

                self.coord_to_id[coord_key] = number
                self.id_to_coord[str(number)] = coord_key
                self.ui.update_inventory_cell(coord_key, str(number), is_history=True)
                loaded_count += 1

            self.ui.append_log(f"已加载历史盘点数据 {loaded_count} 条")

        except Exception as e:
            self.ui.append_log(f"读取 data.json 失败：{e}")

    # ==================================================
    # 通信事件处理
    # ==================================================

    def handle_comm_status(self, text: str):
        self.ui.set_comm_status(text)
        self.last_comm_status = text

    def handle_status(self, status: str):
        """
        处理无人机状态
        例如：
        STATUS:VISION_READY
        STATUS:RECEIVER_READY
        STATUS:FSM_READY
        STATUS:TASK1_RUNNING
        """
        status_key = status.strip().upper()

        if not self.comm_ready:
            self.comm_ready = True
            self.ui.set_ros_launch_enabled(True)

        node_status_map = {
            "VISION_READY": ("VISION", "视觉节点已就绪"),
            "RECEIVER_READY": ("RECEIVER", "通信接收节点已就绪"),
            "FSM_READY": ("FSM", "飞控状态机已就绪"),
            "ROS_READY": ("ROS", "ROS系统启动完成"),
        }

        if status_key in node_status_map:
            node_key, log_text = node_status_map[status_key]
            self.ui.set_node_ready(node_key, True)
            if node_key == "ROS":
                self.ui.set_ros_launch_state("ROS已启动")
            if not self.node_ready_state.get(node_key, False):
                self.node_ready_state[node_key] = True
                self.ui.append_log(log_text)
            return

        if status_key == "BOOT_WAITING":
            return

        task_status_map = {
            "TAKEOFF": "无人机起飞中",
            "TAKEOFF_OK": "无人机起飞成功",
            "TASK1_RUNNING": "任务1执行中",
            "TASK2_RUNNING": "任务二执行中",
            "MISSION_RUNNING": "任务执行中",
            "SCANNING": "正在盘点",
            "LANDING": "无人机降落中",
            "MISSION_FINISHED": "任务完成",
            "TASK_FINISHED": "任务完成",
            "ERROR": "无人机状态异常",
        }

        if status_key in task_status_map:
            text = task_status_map[status_key]
            self.ui.set_task_status(text)
            if status_key != self.last_task_status:
                self.ui.append_log(f"状态：{text}")
                self.last_task_status = status_key
            return

        self.ui.append_log(f"未知状态：{status}")

    def handle_status_with_addr(self, status: str, src_ip: str):
        """
        带来源 IP 的状态包
        用于自动捕获无人机 IP
        """
        status_key = status.strip().upper()

        if status_key == "BOOT_WAITING":
            if not src_ip or src_ip.startswith("127."):
                return
            self.drone_last_seen = datetime.now()
            previous_ip = self.drone_ip_seen

            if not self.user_ip_applied:
                self.comm.update_target(src_ip, self.comm.drone_port)
                self.ui.set_drone_ip(src_ip)

            if previous_ip is None:
                self.drone_ip_seen = src_ip
                if not self.user_ip_applied:
                    self.ui.append_log(f"捕获到无人机上线：{src_ip}")
                return

            if src_ip != previous_ip:
                self.drone_ip_seen = src_ip
                if not self.user_ip_applied:
                    self.ui.append_log(f"无人机 IP 已更新：{previous_ip} -> {src_ip}")

    def handle_scan(self, coord: str, item_id: str):
        """
        处理盘点结果
        协议示例：
        SCAN:A1:17
        """
        coord = coord.strip().upper()
        item_id = item_id.strip()

        if not self.ui.is_valid_coord(coord):
            self.ui.append_log(f"盘点数据无效：未知坐标 {coord}")
            return

        if not item_id.isdigit():
            self.ui.append_log(f"盘点数据无效：编号不是数字 {item_id}")
            return

        number = int(item_id)

        if number < 1 or number > 24:
            self.ui.append_log(f"盘点数据无效：编号 {number} 超出 1~24")
            return

        self.ui.update_inventory_cell(coord, str(number))
        self.update_inventory_data(coord, number)
        self.ui.set_task_status(f"已盘点 {coord}：编号 {number}")
        self.ui.append_log(f"盘点成功：{coord} -> 编号 {number}")

    def handle_target_id(self, item_id: str):
        """
        定点盘点前，收到目标编号
        协议示例：
        TARGET_ID:17
        """
        item_id = item_id.strip()
        valid_target_id = self._parse_valid_task2_target_id(item_id)

        if not valid_target_id:
            self.task2_target_id = None
            self.ui.set_target_info(f"目标编号无效：{item_id or '空'}")
            self.ui.set_task_status("目标识别失败：编号无效")
            self.ui.append_log(f"任务二目标识别失败：非法编号 {item_id or '空'}")
            self.ui.set_task2_start_enabled(False)
            return

        self.task2_target_id = valid_target_id
        self.ui.set_target_info(f"目标编号：{valid_target_id}")
        self.ui.set_task_status("目标识别完成，等待人员撤离后启动定点盘点")
        self.ui.append_log(f"任务二目标识别完成：编号 {valid_target_id}")
        self.ui.set_task2_start_enabled(True)

    def handle_target_result(self, coord: str, item_id: str, result: str):
        """
        定点盘点结果
        协议示例：
        TARGET_RESULT:C4:17:OK
        """
        coord = coord.strip().upper()
        item_id = item_id.strip()
        result = result.strip().upper()

        if self.ui.is_valid_coord(coord):
            self.ui.update_inventory_cell(coord, item_id)
            self.ui.highlight_cell(coord)

        if result == "OK":
            text = f"定点盘点成功：编号 {item_id} 位于 {coord}"
        else:
            text = f"定点盘点失败：编号 {item_id}，坐标 {coord}"

        self.ui.set_query_output(text)
        self.ui.set_target_info(text)
        self.ui.set_task_status(text)
        self.ui.append_log(text)

    def handle_reply(self, reply: str):
        """
        处理机载端回复
        """
        reply_key = reply.strip().upper()

        reply_map = {
            "TASK1_STARTED": "任务1已启动",
            "TASK2_STARTED": "任务二已启动",
            "MISSION_SAVED": "任务数据已保存",
            "MISSION_LOADED": "任务数据已加载",
            "TAKEOFF_OK": "起飞确认成功",
            "SCAN_OK": "盘点确认成功",
        }

        text = reply_map.get(reply_key, f"收到回复：{reply}")

        self.ui.set_task_status(text)
        self.ui.append_log(text)

    def handle_raw_data(self, data: str):
        """
        未识别数据
        """
        self.ui.append_log(f"原始数据：{data}")

    # ==================================================
    # 关闭
    # ==================================================

    def close(self):
        try:
            self.stop_comm_thread()
        except Exception:
            pass

    def stop_comm_thread(self):
        if not self.comm:
            return

        self.comm.stop()
        finished = self.comm.wait(self.COMM_WAIT_TIMEOUT_MS)
        if not finished:
            self.ui.append_log(f"通信线程停止超时（{self.COMM_WAIT_TIMEOUT_MS}ms）")


if __name__ == "__main__":
    app = QApplication(sys.argv)

    controller = MainController()

    exit_code = app.exec_()

    controller.close()

    sys.exit(exit_code)
