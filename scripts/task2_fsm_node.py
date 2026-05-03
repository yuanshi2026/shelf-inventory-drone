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
        self.target_route_to_face = []
        self.target_route_from_face = []
        self.route_index = 0
        self.result_sent = False

        self.qr = {"found": False, "id": -1, "u": -1.0, "v": -1.0, "area": 0.0}
        self.qr_stamp = rospy.Time(0)

        self.land_requested = False
        self.land_request_last_pub = rospy.Time(0)
        self.land_status = "IDLE"

        self.loop_rate = rospy.Rate(30)
        rospy.loginfo("task2_fsm_node started. cmd_vel=%s odom=%s", self.cmd_vel_topic, self.odom_topic)
        self.set_vision(False)
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

        if new_state == "LAND":
            self.land_requested = False
            self.land_request_last_pub = rospy.Time(0)
            self.land_status = "IDLE"

        # 扫码相关状态才打开视觉；飞行、停稳、降落、急停都关闭视觉。
        if new_state in ["SCAN_TARGET_ID", "SEARCH_QR", "ALIGN_QR", "VERIFY_TARGET"]:
            self.set_vision(True)
        else:
            self.set_vision(False)

        # 进入新的移动目标时重置到点稳定计时。
        if new_state in ["TAKEOFF", "GOTO_ROUTE_TO_FACE", "GOTO_SCAN_POINT", "RETURN_ROUTE"]:
            self.arrive_hold_start = None

    def state_elapsed(self):
        return (rospy.Time.now() - self.state_enter_time).to_sec()

    def set_vision(self, enabled):
        self.vision_enable_pub.publish(Bool(data=bool(enabled)))

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
        err_u = self.qr["u"] - self.laser_u
        err_v = self.qr["v"] - self.laser_v
        vy_body = clamp(-self.align_kp_u * err_u, -self.max_align_vel_y, self.max_align_vel_y)
        vz = clamp(-self.align_kp_v * err_v, -self.max_align_vel_z, self.max_align_vel_z)
        self.publish_body_velocity(0.0, vy_body, vz, 0.0)

    def small_search_motion(self):
        # 与任务1看齐：任务2不再在货架前左右/上下搜索，只原地刹车等待二维码。
        self.brake_motion()

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
        self.target_scan = self.resolve_point(self.scan_points[coord])

        route_to_raw = self.routes_to_face.get(face, [])
        route_from_raw = self.routes_from_face.get(face, [])

        if not route_to_raw:
            rospy.logwarn("TASK2 routes_to_face[%s] not found in task2_routes", face)
            return False

        if not route_from_raw:
            rospy.logwarn("TASK2 routes_from_face[%s] not found in task2_routes", face)
            return False

        self.target_route_to_face = [self.resolve_point(p) for p in route_to_raw]
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
        self.set_vision(False)
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
            self.set_state("HOVER_BEFORE_SCAN")
            return
        if self.arrive_hold_start is None:
            self.goto_target(self.target_scan)

    def state_hover_before_scan(self):
        self.set_vision(False)
        self.brake_motion()
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
        self.brake_motion()

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
            self.stop_motion()
            self.set_state("VERIFY_TARGET")
            return
        if self.state_elapsed() > self.align_timeout:
            self.stop_motion()
            self.publish_target_result("FAIL")
            self.route_index = 0
            self.set_state("RETURN_ROUTE")
            return
        self.visual_align()

    def state_verify_target(self):
        self.set_vision(True)
        self.stop_motion()
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
        self.set_vision(False)
        self.stop_motion()

    def state_emergency_stop(self):
        self.set_vision(False)
        self.stop_motion()
        self.publish_stop_to_bridge()


if __name__ == "__main__":
    try:
        node = Task2FSM()
        node.run()
    except rospy.ROSInterruptException:
        pass
