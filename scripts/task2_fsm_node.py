#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os
from copy import deepcopy

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from mavros_msgs.msg import State

try:
    import rospkg
except Exception:
    rospkg = None


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class Task2FSM:
    def __init__(self):
        rospy.init_node("task2_fsm_node")

        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/task2/cmd_vel")

        # task2_routes 由 launch 通过 <rosparam file=... command="load"/> 导入。
        # 推荐结构见文末说明。
        self.routes = rospy.get_param("/task2_routes")

        self.use_relative = bool(self.routes.get("use_relative", True))
        self.takeoff_height = float(self.routes.get("takeoff_height", 0.8))

        # 相机初始离地高度补偿。
        # 约定：task2_routes.yaml 里的 takeoff_height 和所有航点 z，
        # 统一表示“相机中心目标离地高度”，而不是飞控定位点高度。
        # 因此记录 home_z 时会减去 camera_initial_height。
        self.camera_initial_height = float(
            self.routes.get("camera_initial_height", 0.0)
        )

        self.land_point = self.routes["land"]
        # 第二问新版 YAML：不再使用单点 approach，改用按面配置的中间安全航线。
        self.routes_to_face = self.routes.get("routes_to_face", {})
        self.routes_from_face = self.routes.get("routes_from_face", {})
        self.scan_points = self.routes["scan_points"]

        self.arrive_pos_eps = float(self.routes.get("arrive_pos_eps", 0.15))
        self.arrive_z_eps = float(self.routes.get("arrive_z_eps", 0.10))
        self.arrive_yaw_eps = math.radians(float(self.routes.get("arrive_yaw_eps_deg", 10.0)))

        self.kp_xy = float(self.routes.get("kp_xy", 0.40))
        self.kp_z = float(self.routes.get("kp_z", 0.40))
        self.kp_yaw = float(self.routes.get("kp_yaw", 0.55))
        self.max_vel_xy = float(self.routes.get("max_vel_xy", 0.18))
        self.max_vel_z = float(self.routes.get("max_vel_z", 0.10))
        self.max_yaw_rate = float(self.routes.get("max_yaw_rate", 0.35))

        # 与任务1看齐：距离限速 + 限加速度，避免起步、刹车、切航点时速度突变。
        self.max_acc_xy = float(self.routes.get("max_acc_xy", 0.18))
        self.max_acc_z = float(self.routes.get("max_acc_z", 0.12))
        self.max_acc_yaw = float(self.routes.get("max_acc_yaw", 0.35))
        self.min_slow_distance_xy = float(self.routes.get("min_slow_distance_xy", 0.20))
        self.min_slow_distance_z = float(self.routes.get("min_slow_distance_z", 0.15))
        self.yaw_first_threshold = math.radians(float(self.routes.get("yaw_first_threshold_deg", 18.0)))
        self.arrive_hold_time = float(self.routes.get("arrive_hold_time", 0.40))

        # 与任务1看齐：到扫码点后先纯悬停停稳，再打开视觉。
        self.scan_pre_hover_time = float(self.routes.get("scan_pre_hover_time", 2.0))
        self.scan_pre_hover_timeout = float(self.routes.get("scan_pre_hover_timeout", 3.0))
        self.settle_vel_xy_eps = float(self.routes.get("settle_vel_xy_eps", 0.05))
        self.settle_vel_z_eps = float(self.routes.get("settle_vel_z_eps", 0.04))
        self.settle_yaw_rate_eps = float(self.routes.get("settle_yaw_rate_eps", 0.12))
        self.target_scan_timeout = float(self.routes.get("target_scan_timeout", 8.0))

        # ====================【锁点悬停 2026-05-04】任务2扫描阶段锁定任务坐标系目标 ====================
        # 说明：
        #   目标货物扫描点到达后，不再只发 0 速度，而是持续锁定 target_scan 的 x/y/z/yaw。
        #   target_scan 来自 task2_routes.yaml 中 scan_points 的相对坐标，经 resolve_point() 转成 local 坐标。
        #   因此锁定的是任务坐标系目标，不是某一瞬间的实际飞机姿态，避免把偏掉的 yaw 固化下来。
        self.scan_hold_kp_xy = float(self.routes.get("scan_hold_kp_xy", 0.35))
        self.scan_hold_kp_z = float(self.routes.get("scan_hold_kp_z", 0.45))
        self.scan_hold_kp_yaw = float(self.routes.get("scan_hold_kp_yaw", 0.80))

        self.scan_hold_max_vel_xy = float(self.routes.get("scan_hold_max_vel_xy", 0.08))
        self.scan_hold_max_vel_z = float(self.routes.get("scan_hold_max_vel_z", 0.06))
        self.scan_hold_max_yaw_rate = float(self.routes.get("scan_hold_max_yaw_rate", 0.30))

        self.scan_hold_deadband_xy = float(self.routes.get("scan_hold_deadband_xy", 0.025))
        self.scan_hold_deadband_z = float(self.routes.get("scan_hold_deadband_z", 0.025))
        self.scan_hold_deadband_yaw = math.radians(
            float(self.routes.get("scan_hold_deadband_yaw_deg", 2.0))
        )

        # 默认 mission：识别/验证阶段始终锁 YAML 任务点；
        # 可选 aligned_xyz：二维码对准后锁当前 x/y/z，但 yaw 仍锁 YAML 任务航向。
        self.scan_lock_after_align_mode = str(
            self.routes.get("scan_lock_after_align_mode", "mission")
        )
        # ============================================================================

        # 进扫码点前的安全通道 y 坐标。
        # 任务2目标可能是 A1~D6 任意一个点，上下排 z 不一样，
        # 所以不能只靠 YAML 的 routes_to_face 写死最后一个 safe 点。
        # build_target_route() 会根据目标扫码点动态生成一个 safe 点：
        #   x / z / yaw 与目标扫码点一致，y 使用 pre_scan_y。
        # 这样最后 safe -> scan 只会沿 y 方向移动。
        self.pre_scan_y = float(self.routes.get("pre_scan_y", -0.10))

        # 图像对齐参数：只做横向 + 上下微调，不做前后微调。
        self.laser_u = float(self.routes.get("laser_u", 424.0))
        self.laser_v = float(self.routes.get("laser_v", 240.0))
        self.align_pixel_eps = float(self.routes.get("align_pixel_eps", 25.0))
        self.align_kp_u = float(self.routes.get("align_kp_u", 0.0008))
        self.align_kp_v = float(self.routes.get("align_kp_v", 0.0008))
        self.max_align_vel_y = float(self.routes.get("max_align_vel_y", 0.08))
        self.max_align_vel_z = float(self.routes.get("max_align_vel_z", 0.06))

        self.search_timeout = float(self.routes.get("search_timeout", 3.0))
        self.align_timeout = float(self.routes.get("align_timeout", 4.0))
        # 任务2找到目标二维码后也点亮激光；对齐成功/对齐超时都会进入 LASER_FLASH。
        self.laser_flash_time = float(self.routes.get("laser_flash_time", 0.5))
        self.verify_timeout = float(self.routes.get("verify_timeout", 1.0))
        self.qr_fresh_timeout = float(self.routes.get("qr_fresh_timeout", 0.8))
        self.land_finish_timeout = float(self.routes.get("land_finish_timeout", 45.0))

        self.inventory_map_path = rospy.get_param(
            "~inventory_map_path",
            self.default_inventory_map_path()
        )
        self.inventory_map = self.load_inventory_map()

        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=20)
        self.state_pub = rospy.Publisher("/fsm2/state", String, queue_size=20)
        self.vision_enable_pub = rospy.Publisher("/vision/enable", Bool, queue_size=5)
        self.target_result_pub = rospy.Publisher("/target/result", String, queue_size=10)
        self.target_id_pub = rospy.Publisher("/target/id", String, queue_size=10)
        self.land_pub = rospy.Publisher("/uav/land", Bool, queue_size=5)
        self.stop_pub = rospy.Publisher("/uav/stop", Bool, queue_size=5)
        self.laser_pub = rospy.Publisher("/laser/cmd", Bool, queue_size=10)

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=20)
        rospy.Subscriber("/mission/mode", String, self.mode_callback, queue_size=10)
        rospy.Subscriber("/uav/start_task2", Bool, self.start_callback, queue_size=5)
        rospy.Subscriber("/uav/scan_target", Bool, self.scan_target_callback, queue_size=5)
        rospy.Subscriber("/task2/target_id", String, self.target_id_callback, queue_size=10)
        rospy.Subscriber("/uav/stop", Bool, self.stop_callback, queue_size=5)
        rospy.Subscriber("/uav/reset", Bool, self.reset_callback, queue_size=5)
        rospy.Subscriber("/mavros/state", State, self.mavros_state_callback, queue_size=10)
        rospy.Subscriber("/uav/land_status", String, self.land_status_callback, queue_size=10)
        rospy.Subscriber("/qr/result", String, self.qr_callback, queue_size=20)
        rospy.Subscriber("/inventory/result", String, self.inventory_result_callback, queue_size=50)

        self.current_pose_ok = False
        self.x = self.y = self.z = self.yaw = 0.0
        self.vx = self.vy = self.vz = self.yaw_rate = 0.0
        self.home_set = False
        self.home_x = self.home_y = self.home_z = self.home_yaw = 0.0

        self.mode = "IDLE"
        self.state = "IDLE"
        self.state_enter_time = rospy.Time.now()

        self.start_requested = False
        self.stop_requested = False
        self.scan_target_requested = False
        self.emergency_latched = False
        self.emergency_stop_last_pub = rospy.Time(0)
        self.mavros_state = State()
        self.mavros_state_ok = False
        self.arrive_hold_start = None
        self.smooth_cmd = Twist()
        self.last_control_time = rospy.Time.now()

        self.target_id = -1
        self.target_coord = ""
        self.target_face = ""
        self.target_slot = 0
        self.target_scan = None
        self.scan_hold_target = None  # 扫描阶段锁定的任务坐标系 x/y/z/yaw 目标
        self.target_route_to_face = []
        self.target_route_from_face = []
        self.route_index = 0
        self.result_sent = False
        self.laser_started = False

        self.qr = {"found": False, "id": -1, "u": -1.0, "v": -1.0, "area": 0.0}
        self.qr_stamp = rospy.Time(0)

        self.land_requested = False
        self.land_request_last_pub = rospy.Time(0)
        self.land_status = "IDLE"

        self.loop_rate = rospy.Rate(30)
        rospy.loginfo("task2_fsm_node started. cmd_vel=%s odom=%s", self.cmd_vel_topic, self.odom_topic)
        self.set_vision(False)
        self.laser_off()
        self.state_pub.publish(String(data=self.state))

    def default_inventory_map_path(self):
        if rospkg is not None:
            try:
                pkg_path = rospkg.RosPack().get_path("uav_inventory")
                data_dir = os.path.join(pkg_path, "data")
                if not os.path.exists(data_dir):
                    os.makedirs(data_dir)
                return os.path.join(data_dir, "inventory_map.json")
            except Exception:
                pass
        return os.path.expanduser("~/inventory_map.json")

    def load_inventory_map(self):
        if not os.path.exists(self.inventory_map_path):
            rospy.logwarn("inventory_map not found yet: %s", self.inventory_map_path)
            return {}
        try:
            with open(self.inventory_map_path, "r") as f:
                data = json.load(f)
            rospy.loginfo("Loaded inventory_map from %s", self.inventory_map_path)
            return data
        except Exception as e:
            rospy.logwarn("Load inventory_map failed: %s", str(e))
            return {}

    def save_inventory_map(self):
        try:
            parent = os.path.dirname(self.inventory_map_path)
            if parent and not os.path.exists(parent):
                os.makedirs(parent)
            with open(self.inventory_map_path, "w") as f:
                json.dump(self.inventory_map, f, ensure_ascii=False, indent=2)
        except Exception as e:
            rospy.logwarn("Save inventory_map failed: %s", str(e))

    def set_state(self, new_state):
        if new_state == self.state:
            return
        rospy.loginfo("========== FSM2 STATE CHANGE: %s -> %s ==========", self.state, new_state)
        self.state = new_state
        self.state_enter_time = rospy.Time.now()
        self.state_pub.publish(String(data=self.state))

        if new_state == "LASER_FLASH":
            self.laser_started = False

        if new_state == "LAND":
            self.land_requested = False
            self.land_request_last_pub = rospy.Time(0)
            self.land_status = "IDLE"

        # ====================【修改 2026-05-04】扫码相关状态才打开视觉 ====================
        # HOVER_BEFORE_SCAN 也打开视觉：让 RealSense/双目相机提前开始取图并保存使能快照。
        # 本状态不使用二维码结果，进入 SEARCH_QR 前会清空旧 qr，避免提前识别误触发。
        if new_state in ["SCAN_TARGET_ID", "HOVER_BEFORE_SCAN", "SEARCH_QR", "ALIGN_QR", "LASER_FLASH", "VERIFY_TARGET"]:
            self.set_vision(True)
        else:
            self.set_vision(False)

        # 进入新的移动目标时重置到点稳定计时。
        if new_state in ["TAKEOFF", "GOTO_ROUTE_TO_FACE", "GOTO_SCAN_POINT", "RETURN_ROUTE"]:
            self.arrive_hold_start = None

        # 离开目标扫描相关状态后，清除锁点目标，避免返航/降落阶段误用旧扫描点。
        if new_state in ["IDLE", "LOAD_TARGET", "TAKEOFF", "GOTO_ROUTE_TO_FACE", "GOTO_SCAN_POINT",
                         "RETURN_ROUTE", "LAND", "FINISH", "EMERGENCY_STOP"]:
            self.scan_hold_target = None

    def state_elapsed(self):
        return (rospy.Time.now() - self.state_enter_time).to_sec()

    def set_vision(self, enabled):
        self.vision_enable_pub.publish(Bool(data=bool(enabled)))

    def laser_on(self):
        """打开激光。"""
        self.laser_pub.publish(Bool(data=True))

    def laser_off(self):
        """关闭激光。"""
        self.laser_pub.publish(Bool(data=False))

    def stop_motion(self):
        self.smooth_cmd = Twist()
        self.last_control_time = rospy.Time.now()
        self.cmd_pub.publish(Twist())

    def limit_step(self, target_value, current_value, max_delta):
        delta = target_value - current_value
        if delta > max_delta:
            return current_value + max_delta
        if delta < -max_delta:
            return current_value - max_delta
        return target_value

    def publish_smooth_cmd(self, target_cmd):
        now = rospy.Time.now()
        dt = (now - self.last_control_time).to_sec()
        if dt <= 0.0 or dt > 0.20:
            dt = 1.0 / 30.0
        self.last_control_time = now

        max_dxy = self.max_acc_xy * dt
        max_dz = self.max_acc_z * dt
        max_dyaw = self.max_acc_yaw * dt

        self.smooth_cmd.linear.x = self.limit_step(target_cmd.linear.x, self.smooth_cmd.linear.x, max_dxy)
        self.smooth_cmd.linear.y = self.limit_step(target_cmd.linear.y, self.smooth_cmd.linear.y, max_dxy)
        self.smooth_cmd.linear.z = self.limit_step(target_cmd.linear.z, self.smooth_cmd.linear.z, max_dz)
        self.smooth_cmd.angular.z = self.limit_step(target_cmd.angular.z, self.smooth_cmd.angular.z, max_dyaw)

        self.cmd_pub.publish(self.smooth_cmd)

    def brake_motion(self):
        self.publish_smooth_cmd(Twist())

    def distance_limited_speed(self, error_abs, max_speed, max_acc, slow_distance):
        if error_abs <= 0.0:
            return 0.0
        slow_distance = max(slow_distance, 1e-3)
        ratio_speed = max_speed * min(1.0, error_abs / slow_distance)
        brake_speed = math.sqrt(max(0.0, 2.0 * max_acc * error_abs))
        return min(max_speed, ratio_speed, brake_speed)

    def mode_callback(self, msg):
        self.mode = msg.data.strip()

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.x, self.y, self.z = pos.x, pos.y, pos.z
        self.yaw = yaw_from_quaternion(ori)
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.vz = msg.twist.twist.linear.z
        self.yaw_rate = msg.twist.twist.angular.z
        self.current_pose_ok = True

    def start_callback(self, msg):
        if msg.data:
            if self.emergency_latched:
                rospy.logerr("TASK2 start ignored: emergency_latched=True, publish /uav/reset first.")
                return
            self.start_requested = True

    def scan_target_callback(self, msg):
        if msg.data:
            if self.emergency_latched:
                rospy.logerr("TASK2 scan target ignored: emergency_latched=True, publish /uav/reset first.")
                return
            self.scan_target_requested = True

    def stop_callback(self, msg):
        if msg.data:
            if self.emergency_latched:
                return
            self.trigger_emergency_stop()

    def mavros_state_callback(self, msg):
        self.mavros_state = msg
        self.mavros_state_ok = True

    def reset_callback(self, msg):
        if not msg.data:
            return
        if not self.mavros_state_ok:
            rospy.logerr("TASK2 reset rejected: no /mavros/state yet. Cannot prove vehicle is disarmed.")
            return
        if self.mavros_state.armed:
            rospy.logerr("TASK2 reset rejected: vehicle is still armed. Land/disarm and move back to start point first.")
            return

        rospy.logwarn("TASK2 reset accepted, back to IDLE.")
        self.emergency_latched = False
        self.start_requested = False
        self.stop_requested = False
        self.scan_target_requested = False
        self.target_id = -1
        self.target_coord = ""
        self.target_face = ""
        self.target_slot = 0
        self.target_scan = None
        self.scan_hold_target = None
        self.result_sent = False
        self.land_requested = False
        self.land_status = "IDLE"
        self.arrive_hold_start = None
        self.smooth_cmd = Twist()
        self.last_control_time = rospy.Time.now()
        self.qr = {"found": False, "id": -1, "u": -1.0, "v": -1.0, "area": 0.0}
        self.qr_stamp = rospy.Time(0)
        self.set_vision(False)
        self.stop_motion()
        self.laser_off()
        self.set_state("IDLE")

    def land_status_callback(self, msg):
        self.land_status = msg.data.strip()

    def target_id_callback(self, msg):
        text = msg.data.strip()
        try:
            value = int(text)
            if 1 <= value <= 24:
                self.target_id = value
                rospy.logwarn("TASK2 target_id set: %d", self.target_id)
            else:
                rospy.logwarn("Invalid task2 target id: %s", text)
        except Exception:
            rospy.logwarn("Parse /task2/target_id failed: %s", text)

    def qr_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.qr = {
                "found": bool(data.get("found", False)),
                "id": int(data.get("id", -1)),
                "u": float(data.get("u", -1.0)),
                "v": float(data.get("v", -1.0)),
                "area": float(data.get("area", 0.0)),
            }
            self.qr_stamp = rospy.Time.now()
        except Exception as e:
            rospy.logwarn("TASK2 QR parse failed: %s", str(e))

    def inventory_result_callback(self, msg):
        """任务1运行时顺手保存 编号<->坐标，供任务2查表。"""
        try:
            data = json.loads(msg.data)
            goods_id = int(data.get("id", -1))
            face = str(data.get("face", "-")).upper()
            slot = int(data.get("slot", 0))
            status = str(data.get("status", "OK"))
            if status != "OK" or goods_id < 1 or goods_id > 24:
                return
            if face not in ["A", "B", "C", "D"] or slot < 1 or slot > 6:
                return
            coord = "%s%d" % (face, slot)
            self.inventory_map[str(goods_id)] = coord
            self.inventory_map[coord] = goods_id
            self.save_inventory_map()
            rospy.loginfo("TASK2 inventory map updated: %d -> %s", goods_id, coord)
        except Exception as e:
            rospy.logwarn("TASK2 update inventory_map failed: %s", str(e))

    def set_home(self):
        self.home_x = self.x
        self.home_y = self.y

        # 与任务1保持一致：把 home_z 向下虚拟平移相机初始离地高度。
        # 这样 YAML 中的 z 可以按“相机中心高度”填写。
        self.home_z = self.z - self.camera_initial_height
        self.home_yaw = self.yaw
        self.home_set = True

        rospy.loginfo(
            "TASK2 home set: x=%.2f y=%.2f z=%.2f raw_z=%.2f camera_initial_height=%.2f yaw=%.1fdeg",
            self.home_x,
            self.home_y,
            self.home_z,
            self.z,
            self.camera_initial_height,
            math.degrees(self.home_yaw)
        )

    def resolve_point(self, point):
        p = deepcopy(point)
        px, py, pz, pyaw = float(point["x"]), float(point["y"]), float(point["z"]), float(point["yaw"])
        if self.use_relative:
            p["x"] = self.home_x + px
            p["y"] = self.home_y + py
            p["z"] = self.home_z + pz
            p["yaw"] = pyaw
        else:
            p["x"] = px
            p["y"] = py
            p["z"] = pz
            p["yaw"] = pyaw
        return p

    def make_takeoff_target(self):
        return {"x": self.home_x, "y": self.home_y, "z": self.home_z + self.takeoff_height, "yaw": self.home_yaw}

    def goto_target(self, target):
        ex = target["x"] - self.x
        ey = target["y"] - self.y
        ez = target["z"] - self.z
        eyaw = normalize_angle(target["yaw"] - self.yaw)
        horizontal_error = math.sqrt(ex * ex + ey * ey)

        cmd = Twist()

        yaw_speed = self.distance_limited_speed(
            abs(eyaw),
            self.max_yaw_rate,
            self.max_acc_yaw,
            math.radians(8.0)
        )
        cmd.angular.z = math.copysign(yaw_speed, eyaw) if abs(eyaw) > 1e-4 else 0.0

        if abs(eyaw) > self.yaw_first_threshold:
            z_speed = self.distance_limited_speed(abs(ez), self.max_vel_z, self.max_acc_z, self.min_slow_distance_z)
            cmd.linear.z = math.copysign(z_speed, ez) if abs(ez) > 1e-4 else 0.0
            self.publish_smooth_cmd(cmd)
            return

        if horizontal_error > 1e-4:
            xy_speed = self.distance_limited_speed(
                horizontal_error,
                self.max_vel_xy,
                self.max_acc_xy,
                self.min_slow_distance_xy
            )
            cmd.linear.x = xy_speed * ex / horizontal_error
            cmd.linear.y = xy_speed * ey / horizontal_error

        z_speed = self.distance_limited_speed(abs(ez), self.max_vel_z, self.max_acc_z, self.min_slow_distance_z)
        cmd.linear.z = math.copysign(z_speed, ez) if abs(ez) > 1e-4 else 0.0

        self.publish_smooth_cmd(cmd)

    def arrived_target(self, target):
        ex = target["x"] - self.x
        ey = target["y"] - self.y
        ez = target["z"] - self.z
        eyaw = normalize_angle(target["yaw"] - self.yaw)
        horizontal_error = math.sqrt(ex * ex + ey * ey)
        return horizontal_error < self.arrive_pos_eps and abs(ez) < self.arrive_z_eps and abs(eyaw) < self.arrive_yaw_eps

    def velocity_stable(self):
        vel_xy = math.sqrt(self.vx * self.vx + self.vy * self.vy)
        return (
            vel_xy < self.settle_vel_xy_eps and
            abs(self.vz) < self.settle_vel_z_eps and
            abs(self.yaw_rate) < self.settle_yaw_rate_eps
        )

    def compute_scan_hold_cmd(self, target):
        """根据任务坐标系锁定目标，计算扫描阶段 x/y/z/yaw 纠偏速度。"""
        cmd = Twist()

        if target is None:  # 没有锁定目标时，不输出纠偏速度
            return cmd

        ex = float(target["x"]) - self.x
        ey = float(target["y"]) - self.y
        ez = float(target["z"]) - self.z
        eyaw = normalize_angle(float(target["yaw"]) - self.yaw)

        horizontal_error = math.sqrt(ex * ex + ey * ey)

        if horizontal_error > self.scan_hold_deadband_xy:  # 水平偏离超过死区时，拉回任务点
            xy_speed = min(
                self.scan_hold_max_vel_xy,
                self.scan_hold_kp_xy * horizontal_error
            )
            cmd.linear.x = xy_speed * ex / horizontal_error
            cmd.linear.y = xy_speed * ey / horizontal_error

        if abs(ez) > self.scan_hold_deadband_z:  # 高度偏离超过死区时，拉回任务高度
            cmd.linear.z = clamp(
                self.scan_hold_kp_z * ez,
                -self.scan_hold_max_vel_z,
                self.scan_hold_max_vel_z
            )

        if abs(eyaw) > self.scan_hold_deadband_yaw:  # yaw 偏离超过死区时，拉回任务航向
            cmd.angular.z = clamp(
                self.scan_hold_kp_yaw * eyaw,
                -self.scan_hold_max_yaw_rate,
                self.scan_hold_max_yaw_rate
            )

        return cmd

    def limit_scan_cmd(self, cmd, max_xy=None, max_z=None, max_yaw=None):
        """限制扫描阶段纠偏速度，避免锁点和视觉微调叠加后过猛。"""
        if max_xy is None:
            max_xy = self.scan_hold_max_vel_xy
        if max_z is None:
            max_z = self.scan_hold_max_vel_z
        if max_yaw is None:
            max_yaw = self.scan_hold_max_yaw_rate

        xy_speed = math.sqrt(cmd.linear.x * cmd.linear.x + cmd.linear.y * cmd.linear.y)
        if xy_speed > max_xy and xy_speed > 1e-6:  # 水平合速度超限时等比例缩小
            scale = max_xy / xy_speed
            cmd.linear.x *= scale
            cmd.linear.y *= scale

        cmd.linear.z = clamp(cmd.linear.z, -max_z, max_z)
        cmd.angular.z = clamp(cmd.angular.z, -max_yaw, max_yaw)
        return cmd

    def hold_scan_position(self):
        """目标扫描、识别、验证期间持续锁定任务坐标系 x/y/z/yaw。"""
        if self.scan_hold_target is None:  # 没有扫描锁点时退化为平滑刹车
            self.brake_motion()
            return

        cmd = self.compute_scan_hold_cmd(self.scan_hold_target)
        cmd = self.limit_scan_cmd(cmd)
        self.publish_smooth_cmd(cmd)

    def maybe_lock_aligned_xyz_keep_mission_yaw(self):
        """可选：二维码对准成功后锁当前 x/y/z，但 yaw 仍保持 YAML 任务航向。"""
        if self.scan_hold_target is None:
            return
        if self.scan_lock_after_align_mode != "aligned_xyz":
            return

        mission_yaw = float(self.scan_hold_target["yaw"])
        self.scan_hold_target = {
            "type": "hold",
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "yaw": mission_yaw
        }

    def reached_and_hold(self, target):
        if not self.arrived_target(target):
            self.arrive_hold_start = None
            return False
        self.brake_motion()
        if self.arrive_hold_start is None:
            self.arrive_hold_start = rospy.Time.now()
            return False
        if rospy.Time.now() - self.arrive_hold_start < rospy.Duration(self.arrive_hold_time):
            return False
        self.arrive_hold_start = None
        return True

    def publish_body_velocity(self, vx_body, vy_body, vz, yaw_rate):
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        cmd = Twist()
        cmd.linear.x = c * vx_body - s * vy_body
        cmd.linear.y = s * vx_body + c * vy_body
        cmd.linear.z = vz
        cmd.angular.z = yaw_rate
        self.publish_smooth_cmd(cmd)

    def fresh_qr_found(self):
        age = (rospy.Time.now() - self.qr_stamp).to_sec()
        return self.qr["found"] and self.qr["id"] > 0 and age < self.qr_fresh_timeout

    def qr_aligned(self):
        err_u = self.qr["u"] - self.laser_u
        err_v = self.qr["v"] - self.laser_v
        return abs(err_u) < self.align_pixel_eps and abs(err_v) < self.align_pixel_eps

    def visual_align(self):
        """根据二维码像素误差微调，同时继续锁定任务 yaw，避免目标扫描时原地偏航。"""
        err_u = self.qr["u"] - self.laser_u
        err_v = self.qr["v"] - self.laser_v

        vy_body = clamp(-self.align_kp_u * err_u, -self.max_align_vel_y, self.max_align_vel_y)
        vz = clamp(-self.align_kp_v * err_v, -self.max_align_vel_z, self.max_align_vel_z)

        # 基础锁点命令负责把 x/y/z/yaw 拉回 task2_routes.yaml 中的目标扫码点。
        # 这样视觉对准阶段即使二维码像素有抖动，也不会放任机体原地偏航。
        cmd = self.compute_scan_hold_cmd(self.scan_hold_target)

        # 视觉对准允许在机体系横向和高度方向做小幅修正。
        # 如果希望完全固定 YAML 的 x/y/z/yaw，可以把 align_kp_u/align_kp_v 设为 0。
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        cmd.linear.x += -s * vy_body
        cmd.linear.y += c * vy_body
        cmd.linear.z += vz

        cmd = self.limit_scan_cmd(
            cmd,
            max_xy=max(self.scan_hold_max_vel_xy, self.max_align_vel_y),
            max_z=max(self.scan_hold_max_vel_z, self.max_align_vel_z),
            max_yaw=self.scan_hold_max_yaw_rate
        )
        self.publish_smooth_cmd(cmd)

    def small_search_motion(self):
        # 与任务1看齐：任务2不再在货架前左右/上下搜索，只锁定目标扫码点等待二维码。
        self.hold_scan_position()

    def publish_stop_to_bridge(self, force=False):
        now = rospy.Time.now()
        if force or now - self.emergency_stop_last_pub > rospy.Duration(1.0):
            self.stop_pub.publish(Bool(data=True))
            self.emergency_stop_last_pub = now

    def trigger_emergency_stop(self):
        if not self.emergency_latched:
            rospy.logerr("TASK2 EMERGENCY STOP LATCHED")
            self.publish_stop_to_bridge(force=True)
        self.emergency_latched = True
        self.stop_requested = True
        self.start_requested = False
        self.scan_target_requested = False
        self.scan_hold_target = None
        self.set_vision(False)
        self.stop_motion()
        self.set_state("EMERGENCY_STOP")

    def parse_coord(self, coord):
        coord = str(coord).strip().upper()
        if len(coord) < 2:
            return "", 0
        face = coord[0]
        try:
            slot = int(coord[1:])
        except Exception:
            return "", 0
        if face not in ["A", "B", "C", "D"] or slot < 1 or slot > 6:
            return "", 0
        return face, slot

    def make_pre_scan_safe_point(self, raw_scan_point):
        """根据目标扫码点动态生成进入扫码前的 safe 点。

        raw_scan_point 来自 task2_routes.yaml 的 scan_points，例如 B5。
        生成的 safe 点保持 x / z / yaw 不变，只把 y 改成 pre_scan_y。
        这样无人机跑完 routes_to_face 后，最后一段进入二维码点时只沿 y 方向移动，
        不会在进入扫描过程中再改变与货架的横向距离，也不会临时调高度。
        """
        safe_point = deepcopy(raw_scan_point)
        safe_point["type"] = "safe"
        safe_point["y"] = self.pre_scan_y
        return safe_point

    def build_target_route(self):
        self.inventory_map = self.load_inventory_map()
        coord = self.inventory_map.get(str(self.target_id), "")
        if not coord:
            rospy.logwarn("TASK2 target id %d not found in inventory_map", self.target_id)
            return False

        face, slot = self.parse_coord(coord)
        if not face:
            rospy.logwarn("TASK2 invalid coord from inventory_map: %s", coord)
            return False

        if coord not in self.scan_points:
            rospy.logwarn("TASK2 scan point %s not found in task2_routes", coord)
            return False

        self.target_coord = coord
        self.target_face = face
        self.target_slot = slot

        # 目标二维码的真实扫描点。
        scan_raw = self.scan_points[coord]
        self.target_scan = self.resolve_point(scan_raw)
        self.scan_hold_target = None

        # 动态生成进扫码点前的 safe 点。
        # 该点与 target_scan 只有 y 不同，用于保证 safe -> scan 只做 y 方向移动。
        pre_scan_safe_raw = self.make_pre_scan_safe_point(scan_raw)

        route_to_raw = self.routes_to_face.get(face, [])
        route_from_raw = self.routes_from_face.get(face, [])

        if not route_to_raw:
            rospy.logwarn("TASK2 routes_to_face[%s] not found in task2_routes", face)
            return False

        if not route_from_raw:
            rospy.logwarn("TASK2 routes_from_face[%s] not found in task2_routes", face)
            return False

        # 先跑该面的通用安全路线，再追加动态 safe 点。
        # 追加后的最后一个 safe 点与目标 scan 点 x/z/yaw 完全一致，只有 y 不同。
        self.target_route_to_face = [self.resolve_point(p) for p in route_to_raw]
        self.target_route_to_face.append(self.resolve_point(pre_scan_safe_raw))

        self.target_route_from_face = [self.resolve_point(p) for p in route_from_raw]
        self.route_index = 0

        rospy.logwarn(
            "TASK2 route built: id=%d coord=%s face=%s to_points=%d from_points=%d",
            self.target_id, self.target_coord, self.target_face,
            len(self.target_route_to_face), len(self.target_route_from_face)
        )
        return True

    def publish_target_result(self, result_text):
        msg = {
            "id": int(self.target_id),
            "face": str(self.target_face),
            "slot": int(self.target_slot),
            "result": str(result_text),
        }
        self.target_result_pub.publish(String(data=json.dumps(msg)))
        rospy.logwarn("TASK2 result: %s", json.dumps(msg, ensure_ascii=False))

    def run(self):
        while not rospy.is_shutdown():
            if self.emergency_latched:
                self.set_vision(False)
                self.laser_off()
                self.stop_motion()
                self.publish_stop_to_bridge()
                self.set_state("EMERGENCY_STOP")
                self.loop_rate.sleep()
                continue

            if self.stop_requested:
                self.trigger_emergency_stop()
                self.loop_rate.sleep()
                continue

            # SCAN_TARGET_ID 是起飞前原地扫码流程，不要求 mission_manager 切到 TASK2。
            if self.mode != "TASK2" and self.state not in ["IDLE", "FINISH", "SCAN_TARGET_ID"]:
                self.set_vision(False)
                self.stop_motion()
                self.set_state("IDLE")
                self.loop_rate.sleep()
                continue

            if not self.current_pose_ok:
                rospy.logwarn_throttle(1.0, "TASK2 waiting for Odometry...")
                self.loop_rate.sleep()
                continue

            if self.state == "IDLE":
                self.state_idle()
            elif self.state == "SCAN_TARGET_ID":
                self.state_scan_target_id()
            elif self.state == "LOAD_TARGET":
                self.state_load_target()
            elif self.state == "TAKEOFF":
                self.state_takeoff()
            elif self.state == "GOTO_ROUTE_TO_FACE":
                self.state_goto_route_to_face()
            elif self.state == "GOTO_SCAN_POINT":
                self.state_goto_scan_point()
            elif self.state == "HOVER_BEFORE_SCAN":
                self.state_hover_before_scan()
            elif self.state == "SEARCH_QR":
                self.state_search_qr()
            elif self.state == "ALIGN_QR":
                self.state_align_qr()
            elif self.state == "LASER_FLASH":
                self.state_laser_flash()
            elif self.state == "VERIFY_TARGET":
                self.state_verify_target()
            elif self.state == "RETURN_ROUTE":
                self.state_return_route()
            elif self.state == "LAND":
                self.state_land()
            elif self.state == "FINISH":
                self.state_finish()
            elif self.state == "EMERGENCY_STOP":
                self.state_emergency_stop()

            self.loop_rate.sleep()

    def state_idle(self):
        # ====================【修改 2026-05-04】IDLE 不再循环发布 /vision/enable=False ====================
        # 任务1运行时，任务2 FSM 通常仍停在 IDLE。
        # 旧代码会在 IDLE 里 30Hz 发布 False，把任务1 SEARCH_QR/HOVER_BEFORE_SCAN 发出的 True 抢掉，
        # 这就是实飞时视觉队友看到“没有收到使能、相机不拍照”的主要原因。
        # 进入 IDLE 的状态切换处已经会发布一次 False，这里不要持续抢占视觉话题。
        self.laser_off()
        self.stop_motion()

        if self.scan_target_requested:
            self.scan_target_requested = False
            self.target_id = -1
            self.qr = {"found": False, "id": -1, "u": -1.0, "v": -1.0, "area": 0.0}
            self.qr_stamp = rospy.Time(0)
            self.set_state("SCAN_TARGET_ID")
            return

        if self.start_requested and self.mode == "TASK2":
            self.start_requested = False
            self.stop_requested = False
            self.result_sent = False
            self.set_home()
            self.set_state("LOAD_TARGET")

    def state_scan_target_id(self):
        """任务2.1：起飞前原地开视觉扫码，得到目标编号并上报地面站。"""
        self.set_vision(True)
        self.brake_motion()

        if self.fresh_qr_found():
            scanned_id = int(self.qr["id"])
            if 1 <= scanned_id <= 24:
                self.target_id = scanned_id
                self.target_id_pub.publish(String(data=str(scanned_id)))
                rospy.logwarn("TASK2 target scanned: %d", scanned_id)
                self.stop_motion()
                self.set_vision(False)
                self.set_state("IDLE")
                return

        if self.state_elapsed() > self.target_scan_timeout:
            rospy.logwarn("TASK2 target scan timeout. No valid target id scanned.")
            self.stop_motion()
            self.set_vision(False)
            self.set_state("IDLE")

    def state_load_target(self):
        if self.target_id < 1:
            rospy.logwarn_throttle(1.0, "TASK2 waiting for valid target_id...")
            return
        if not self.build_target_route():
            self.target_face = "-"
            self.target_slot = 0
            self.publish_target_result("NOT_FOUND")
            self.set_state("FINISH")
            return
        self.set_state("TAKEOFF")

    def state_takeoff(self):
        target = self.make_takeoff_target()
        if self.reached_and_hold(target):
            self.stop_motion()
            self.route_index = 0
            if self.target_route_to_face:
                self.set_state("GOTO_ROUTE_TO_FACE")
            else:
                self.set_state("GOTO_SCAN_POINT")
            return
        if self.arrive_hold_start is None:
            self.goto_target(target)

    def state_goto_route_to_face(self):
        if self.route_index >= len(self.target_route_to_face):
            self.stop_motion()
            self.set_state("GOTO_SCAN_POINT")
            return

        target = self.target_route_to_face[self.route_index]
        if self.reached_and_hold(target):
            self.stop_motion()
            self.route_index += 1
            if self.route_index >= len(self.target_route_to_face):
                self.set_state("GOTO_SCAN_POINT")
            return

        if self.arrive_hold_start is None:
            self.goto_target(target)

    def state_goto_scan_point(self):
        if self.reached_and_hold(self.target_scan):
            self.stop_motion()
            # 这里锁的是 task2_routes.yaml 中目标扫码点转换后的任务坐标，
            # 不是当前飞机实际位姿，避免把还没停稳或已经偏航的姿态保存下来。
            self.scan_hold_target = deepcopy(self.target_scan)
            self.set_state("HOVER_BEFORE_SCAN")
            return
        if self.arrive_hold_start is None:
            self.goto_target(self.target_scan)

    def state_hover_before_scan(self):
        # ====================【修改 2026-05-04】停稳阶段也打开视觉使能 ====================
        # 这里打开视觉只是让相机开始取图/保存快照；本状态不判断 fresh_qr_found()。
        # 进入 SEARCH_QR 前会清空 qr 缓存，所以不会因为提前识别而误触发。
        self.set_vision(True)
        self.hold_scan_position()  # 扫码前停稳阶段持续锁定目标 x/y/z/yaw
        elapsed = self.state_elapsed()

        if elapsed < self.scan_pre_hover_time:
            return

        if self.velocity_stable():
            self.qr = {"found": False, "id": -1, "u": -1.0, "v": -1.0, "area": 0.0}
            self.qr_stamp = rospy.Time(0)
            self.stop_motion()
            self.set_state("SEARCH_QR")
            return

        if elapsed > self.scan_pre_hover_timeout:
            rospy.logwarn(
                "TASK2 scan pre-hover timeout, continue scan. vx=%.3f vy=%.3f vz=%.3f yaw_rate=%.3f",
                self.vx, self.vy, self.vz, self.yaw_rate
            )
            self.stop_motion()
            self.set_state("SEARCH_QR")

    def state_search_qr(self):
        self.set_vision(True)
        self.hold_scan_position()  # 搜索二维码时不再只发 0 速度，而是锁定目标点

        if self.fresh_qr_found():
            self.stop_motion()
            self.set_state("ALIGN_QR")
            return

        if self.state_elapsed() < self.search_timeout:
            return

        self.stop_motion()
        self.publish_target_result("FAIL")
        self.route_index = 0
        self.set_state("RETURN_ROUTE")

    def state_align_qr(self):
        self.set_vision(True)
        if not self.fresh_qr_found():
            self.stop_motion()
            self.set_state("SEARCH_QR")
            return
        if self.qr_aligned():
            self.maybe_lock_aligned_xyz_keep_mission_yaw()
            self.stop_motion()
            self.set_state("LASER_FLASH")
            return
        if self.state_elapsed() > self.align_timeout:
            # ====================【修改 2026-05-04】对齐失败仍点亮激光并继续验证 ====================
            # 能进入 ALIGN_QR，说明已经读到了二维码编号。
            # 对齐超时只代表激光点没有完全压到二维码中心，不代表目标二维码识别失败。
            # 所以这里不再直接 FAIL，而是先点亮激光，再进入 VERIFY_TARGET 判断编号是否匹配。
            rospy.logwarn(
                "TASK2 align timeout, but QR id=%d has been detected. Flash laser and continue VERIFY_TARGET.",
                int(self.qr["id"])
            )
            self.stop_motion()
            self.set_state("LASER_FLASH")
            return
            # ===========================================================================
        self.visual_align()

    def state_laser_flash(self):
        """LASER_FLASH：任务2识别到目标点二维码后点亮激光，然后进入编号验证。"""
        self.set_vision(True)
        self.hold_scan_position()

        if not self.laser_started:
            self.laser_started = True
            self.laser_on()
            self.state_enter_time = rospy.Time.now()
            return

        if self.state_elapsed() >= self.laser_flash_time:
            self.laser_off()
            self.set_state("VERIFY_TARGET")

    def state_verify_target(self):
        self.set_vision(True)
        self.hold_scan_position()  # 验证目标编号期间继续锁定目标 x/y/z/yaw
        if self.fresh_qr_found():
            if int(self.qr["id"]) == int(self.target_id):
                self.publish_target_result("OK")
            else:
                self.publish_target_result("MISMATCH_%d" % int(self.qr["id"]))
            self.route_index = 0
            self.set_state("RETURN_ROUTE")
            return
        if self.state_elapsed() > self.verify_timeout:
            self.publish_target_result("FAIL")
            self.route_index = 0
            self.set_state("RETURN_ROUTE")

    def state_return_route(self):
        self.set_vision(False)
        self.scan_hold_target = None

        if not self.target_route_from_face:
            self.target_route_from_face = [self.resolve_point(self.land_point)]
            self.route_index = 0

        if self.route_index >= len(self.target_route_from_face):
            self.stop_motion()
            self.set_state("LAND")
            return

        target = self.target_route_from_face[self.route_index]
        if self.reached_and_hold(target):
            self.stop_motion()
            self.route_index += 1
            if self.route_index >= len(self.target_route_from_face):
                self.set_state("LAND")
            return

        if self.arrive_hold_start is None:
            self.goto_target(target)

    def state_land(self):
        self.set_vision(False)
        self.laser_off()
        self.stop_motion()
        now = rospy.Time.now()
        if (not self.land_requested) or (now - self.land_request_last_pub > rospy.Duration(1.0)):
            self.land_pub.publish(Bool(data=True))
            self.land_requested = True
            self.land_request_last_pub = now
            rospy.logwarn_throttle(1.0, "TASK2 LAND: publishing /uav/land=True, waiting for DISARMED.")
        if self.land_status == "DISARMED":
            self.stop_motion()
            self.set_state("FINISH")
            return
        if self.state_elapsed() > self.land_finish_timeout:
            rospy.logwarn_throttle(1.0, "TASK2 LAND timeout, current land_status=%s", self.land_status)

    def state_finish(self):
        self.scan_hold_target = None
        # ====================【修改 2026-05-04】不要在 FINISH 中循环发布 /vision/enable=False ====================
        # 进入 FINISH 时 set_state() 已经发布过一次 False。
        # 如果这里继续 30Hz 发布 False，后续启动任务1时会把任务1的视觉 True 抢掉。
        self.laser_off()
        self.stop_motion()

    def state_emergency_stop(self):
        self.scan_hold_target = None
        self.set_vision(False)
        self.laser_off()
        self.stop_motion()
        self.publish_stop_to_bridge()


if __name__ == "__main__":
    try:
        node = Task2FSM()
        node.run()
    except rospy.ROSInterruptException:
        pass
