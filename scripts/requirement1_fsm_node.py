#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
from copy import deepcopy

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from mavros_msgs.msg import State


def clamp(value, min_value, max_value):
    """将 value 限制在 [min_value, max_value] 范围内。"""
    return max(min_value, min(max_value, value))


def normalize_angle(angle):
    """将角度归一化到 [-pi, pi]。"""

    while angle > math.pi:  # 角度过大则减 2pi
        angle -= 2.0 * math.pi

    while angle < -math.pi:  # 角度过小则加 2pi
        angle += 2.0 * math.pi

    return angle


def yaw_from_quaternion(q):
    """从四元数中提取 yaw 偏航角。"""

    x = q.x
    y = q.y
    z = q.z
    w = q.w

    siny_cosp = 2.0 * (w * z + x * y)       # yaw 计算中间量
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z) # yaw 计算中间量

    return math.atan2(siny_cosp, cosy_cosp)


class Requirement1FSM:
    def __init__(self):
        rospy.init_node("requirement1_fsm_node")

        self.odom_topic = rospy.get_param("~odom_topic", "/Odometry")        # 定位输入话题
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/task1/cmd_vel") # 速度输出话题：任务仲裁器模式下默认输出到 /task1/cmd_vel

        self.mission = rospy.get_param("/mission") # YAML 中的 mission 参数字典

        self.use_relative = bool(self.mission.get("use_relative", True)) # 是否使用相对起飞点坐标
        self.takeoff_height = float(self.mission.get("takeoff_height", 1.0)) # 起飞高度

        # ====================【高度补偿 2026-05-03】相机初始离地高度 ====================
        # 作用：
        #   YAML 里的 takeoff_height 和各个航点 z，统一理解为“相机中心目标离地高度”。
        #   由于起飞前相机本身已经离地 camera_initial_height，
        #   所以记录 home_z 时需要减掉这个高度，避免实际相机高度偏高。
        #
        # 例子：
        #   takeoff_height = 1.50
        #   camera_initial_height = 0.15
        #   最终无人机定位点目标高度 = home_z + 1.50 = self.z - 0.15 + 1.50 = self.z + 1.35
        #   相机实际高度约为 1.35 + 0.15 = 1.50
        self.camera_initial_height = float(
            self.mission.get("camera_initial_height", 0.0)
        )
        # =======================================================================

        self.land_point = self.mission["land"]   # 降落点配置
        self.points = self.mission["points"]     # 航点列表

        self.arrive_pos_eps = float(self.mission.get("arrive_pos_eps", 0.15)) # 水平到达阈值
        self.arrive_z_eps = float(self.mission.get("arrive_z_eps", 0.10))     # 高度到达阈值
        self.arrive_yaw_eps = math.radians(
            float(self.mission.get("arrive_yaw_eps_deg", 10.0))
        ) # 偏航角到达阈值，转成弧度

        self.kp_xy = float(self.mission.get("kp_xy", 0.45))   # 水平 P 控制系数
        self.kp_z = float(self.mission.get("kp_z", 0.45))     # 高度 P 控制系数
        self.kp_yaw = float(self.mission.get("kp_yaw", 0.60)) # 偏航 P 控制系数

        self.max_vel_xy = float(self.mission.get("max_vel_xy", 0.25))       # 最大水平速度
        self.max_vel_z = float(self.mission.get("max_vel_z", 0.15))         # 最大竖直速度
        self.max_yaw_rate = float(self.mission.get("max_yaw_rate", 0.40))   # 最大偏航角速度

        self.laser_u = float(self.mission.get("laser_u", 320.0)) # 激光点图像横坐标
        self.laser_v = float(self.mission.get("laser_v", 240.0)) # 激光点图像纵坐标
        self.align_pixel_eps = float(self.mission.get("align_pixel_eps", 25.0)) # 对准像素阈值

        self.align_kp_u = float(self.mission.get("align_kp_u", 0.0008)) # 图像横向误差控制系数
        self.align_kp_v = float(self.mission.get("align_kp_v", 0.0008)) # 图像纵向误差控制系数
        self.max_align_vel_y = float(self.mission.get("max_align_vel_y", 0.10)) # 最大视觉横移速度
        self.max_align_vel_z = float(self.mission.get("max_align_vel_z", 0.08)) # 最大视觉升降速度

        self.search_timeout = float(self.mission.get("search_timeout", 3.0))       # 搜索二维码超时时间
        self.align_timeout = float(self.mission.get("align_timeout", 4.0))         # 对准二维码超时时间
        self.laser_flash_time = float(self.mission.get("laser_flash_time", 0.5))   # 激光点亮时间
        self.ack_timeout = float(self.mission.get("ack_timeout", 0.4))             # 等待 ACK 时间

        # ====================【新增】正常任务结束降落参数 ====================
        # land_finish_timeout：FSM 在 LAND 状态等待 bridge 完成 AUTO.LAND + DISARM 的最长时间。
        # 注意：超时后这里不会强行 FINISH，只会周期性报警；真机上不应绕过上锁确认。
        self.land_finish_timeout = float(
            self.mission.get("land_finish_timeout", 45.0)
        )
        # ================================================================

        self.cmd_pub = rospy.Publisher(
            self.cmd_vel_topic,
            Twist,
            queue_size=20
        ) # 速度指令发布器

        self.laser_pub = rospy.Publisher(
            "/laser/cmd",
            Bool,
            queue_size=10
        ) # 激光开关发布器

        self.result_pub = rospy.Publisher(
            "/inventory/result",
            String,
            queue_size=20
        ) # 盘点结果发布器

        self.state_pub = rospy.Publisher(
            "/fsm/state",
            String,
            queue_size=20
        ) # FSM 状态发布器

        # ====================【新增】正常任务完成后请求 mavros_sitl_bridge 自动降落/上锁 ====================
        self.land_pub = rospy.Publisher(
            "/uav/land",
            Bool,
            queue_size=5
        ) # 正常降落请求发布器；不要复用 /uav/stop，/uav/stop 只保留给急停
        # ==================================================================================

        # ====================【本次修改-安全 1】急停时同步通知 bridge 锁死 ====================
        self.stop_pub = rospy.Publisher(
            "/uav/stop",
            Bool,
            queue_size=5
        ) # FSM 自身进入急停时，重复通知 bridge 也进入急停锁存
        # ============================================================================

        rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self.odom_callback,
            queue_size=20
        ) # 订阅定位信息

        rospy.Subscriber(
            "/qr/result",
            String,
            self.qr_callback,
            queue_size=20
        ) # 订阅二维码结果

        rospy.Subscriber(
            "/uav/start",
            Bool,
            self.start_callback,
            queue_size=5
        ) # 订阅一键启动

        rospy.Subscriber(
            "/uav/stop",
            Bool,
            self.stop_callback,
            queue_size=5
        ) # 订阅停止信号

        # ====================【本次安全修改 2026-05-03 1】人工复位入口 ====================
        rospy.Subscriber(
            "/uav/reset",
            Bool,
            self.reset_callback,
            queue_size=5
        ) # 急停后人工复位，才允许重新回到 IDLE
        # ==========================================================================

        # ====================【本次安全修改 2026-05-03 2】订阅 MAVROS 状态，复位前确认已上锁 ====================
        rospy.Subscriber(
            "/mavros/state",
            State,
            self.mavros_state_callback,
            queue_size=10
        ) # /uav/reset 只允许在 armed=False 时生效
        # =================================================================================

        rospy.Subscriber(
            "/ground/ack",
            String,
            self.ack_callback,
            queue_size=20
        ) # 订阅地面站 ACK

        # ====================【新增】订阅 bridge 的降落/上锁进度 ====================
        rospy.Subscriber(
            "/uav/land_status",
            String,
            self.land_status_callback,
            queue_size=10
        ) # 收到 DISARMED 后，FSM 才认为任务真正 FINISH
        # ======================================================================
       



        ##########==================##################变量定义及初始话###########================================================================
        self.current_pose_ok = False # 是否已收到定位
        self.x = 0.0                 # 当前 x 坐标
        self.y = 0.0                 # 当前 y 坐标
        self.z = 0.0                 # 当前高度
        self.yaw = 0.0               # 当前偏航角

        self.home_set = False        # 是否已记录起飞点
        self.home_x = 0.0            # 起飞点 x
        self.home_y = 0.0            # 起飞点 y
        self.home_z = 0.0            # 起飞点 z
        self.home_yaw = 0.0          # 起飞时偏航角

        self.state = "IDLE"          # 当前 FSM 状态
        self.state_enter_time = rospy.Time.now() # 进入当前状态的时间

        self.current_index = 0       # 当前执行的航点序号
        self.current_scan_point = None # 当前扫码点信息

        self.qr = {
            "found": False,
            "id": -1,
            "u": -1.0,
            "v": -1.0,
            "area": 0.0
        } # 当前二维码识别结果

        self.qr_stamp = rospy.Time(0) # 最近一次收到二维码结果的时间

        self.start_requested = False # 是否收到启动请求
        self.stop_requested = False  # 是否收到停止请求
        # ====================【本次修改-安全 3】FSM 急停锁存状态 ====================
        self.emergency_latched = False # 急停锁存；True 后不再继续旧航点
        self.emergency_reason = ""     # 急停原因，便于终端排查
        # ===================================================================
        self.last_ack_id = None      # 最近一次收到的 ACK 编号
        self.last_sent_goods_id = None # 最近一次发送的货物编号

        self.laser_started = False   # 当前激光状态中是否已经打开过激光
        self.result_sent = False     # 当前 SEND_RESULT 状态中是否已发送结果
        self.land_requested = False              # 是否已经向 /uav/land 发布过降落请求
        self.land_request_last_pub = rospy.Time(0) # 上次发布 /uav/land 的时间，用于低频重发
        self.land_status = "IDLE"               # bridge 回传的降落/上锁状态

        # ====================【本次安全修改 2026-05-03 3】复位安全判断与急停通知节流 ====================
        self.mavros_state = State()               # MAVROS 当前状态，用于判断是否已经上锁
        self.mavros_state_ok = False              # 是否已经收到过 /mavros/state
        self.emergency_stop_last_pub = rospy.Time(0) # 上次向 bridge 重发 /uav/stop 的时间
        # =================================================================================
        # ================================================================================================================================================
        
        
        
        
        #========================fsm主循环以及回调函数==========================================================
        self.loop_rate = rospy.Rate(30) # FSM 主循环频率
        rospy.loginfo("requirement1_fsm_node started.")
        self.state_pub.publish(String(data=self.state))

    def mavros_state_callback(self, msg):
        """保存 MAVROS armed/mode 状态，用于限制 /uav/reset 只能在已上锁后生效。"""
        self.mavros_state = msg
        self.mavros_state_ok = True

    def odom_callback(self, msg):
        """更新无人机当前位置和偏航角。"""

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.x = position.x
        self.y = position.y
        self.z = position.z
        self.yaw = yaw_from_quaternion(orientation)

        self.current_pose_ok = True

    def qr_callback(self, msg):
        """解析二维码识别结果 JSON。"""

        try:
            data = json.loads(msg.data)

            self.qr = {
                "found": bool(data.get("found", False)),
                "id": int(data.get("id", -1)),
                "u": float(data.get("u", -1.0)),
                "v": float(data.get("v", -1.0)),
                "area": float(data.get("area", 0.0))
            }

            self.qr_stamp = rospy.Time.now()

        except Exception as e:  # JSON 格式错误时进入这里
            rospy.logwarn("QR parse failed: %s", str(e))

    def start_callback(self, msg):
        """收到一键启动信号。"""
        if msg.data:  # 只响应 True
            # ====================【本次修改-安全 4】急停后禁止直接继续任务 ====================
            if self.emergency_latched:
                rospy.logerr(
                    "Start ignored: FSM emergency_latched=True, reason=%s. Publish /uav/reset first.",
                    self.emergency_reason
                )
                return
            # =====================================================================
            self.start_requested = True

    def stop_callback(self, msg):
        """收到停止信号。

        注意：本节点也会向 /uav/stop 低频重发急停通知给 bridge。
        因此急停已经锁存后，收到同一话题的 True 要直接忽略，
        避免自己发布的 /uav/stop 再触发自己、形成回环刷屏。
        """
        if msg.data:  # 只响应 True
            if self.emergency_latched:
                return
            self.trigger_emergency_stop("/uav/stop received")

    # ====================【本次安全修改 2026-05-03 4】急停锁存与人工复位 ====================
    def publish_stop_to_bridge(self, force=False):
        """向 bridge 发布 /uav/stop；急停后低频重发，避免 30Hz 刷屏。"""
        now = rospy.Time.now()

        if force or now - self.emergency_stop_last_pub > rospy.Duration(1.0):
            self.stop_pub.publish(Bool(data=True))
            self.emergency_stop_last_pub = now

    def trigger_emergency_stop(self, reason):
        """进入不可自动恢复的急停悬停状态，清除旧航点追踪动作。"""
        first_enter = not self.emergency_latched

        if first_enter:  # 第一次进入急停时打印原因
            rospy.logerr("FSM EMERGENCY HOVER LATCHED: %s", reason)

        self.emergency_latched = True
        self.emergency_reason = reason
        self.start_requested = False
        self.stop_requested = True
        self.current_scan_point = None
        self.result_sent = False
        self.laser_started = False
        self.stop_motion()
        self.laser_off()

        # 只在第一次进入急停时强制通知 bridge。之后由 state_emergency_stop() 低频重发。
        if first_enter:
            self.publish_stop_to_bridge(force=True)

        self.set_state("EMERGENCY_STOP")

    def reset_callback(self, msg):
        """人工复位 FSM；只允许已上锁后清状态，复位后从 IDLE 重新开始。"""
        if not msg.data:  # 只响应 True
            return

        # 规则写死：未收到 MAVROS 状态，或者飞机仍 armed，都拒绝复位。
        if not self.mavros_state_ok:
            rospy.logerr("FSM reset rejected: no /mavros/state yet. Cannot prove vehicle is disarmed.")
            return

        if self.mavros_state.armed:
            rospy.logerr("FSM reset rejected: vehicle is still armed. Land/disarm and move back to start point first.")
            return

        rospy.logwarn("FSM reset accepted. Return to IDLE and clear all old mission progress.")
        self.emergency_latched = False
        self.emergency_reason = ""
        self.start_requested = False
        self.stop_requested = False
        self.current_index = 0
        self.current_scan_point = None
        self.last_ack_id = None
        self.last_sent_goods_id = None
        self.result_sent = False
        self.laser_started = False
        self.land_requested = False
        self.land_status = "IDLE"
        self.land_request_last_pub = rospy.Time(0)
        self.qr = {
            "found": False,
            "id": -1,
            "u": -1.0,
            "v": -1.0,
            "area": 0.0
        }
        self.qr_stamp = rospy.Time(0)
        self.home_set = False
        self.stop_motion()
        self.laser_off()
        self.set_state("IDLE")
    # =================================================================================

    def ack_callback(self, msg):
        """保存地面站 ACK 编号。"""
        self.last_ack_id = msg.data.strip()

    def land_status_callback(self, msg):
        """保存 bridge 回传的降落状态，例如 WAIT_LANDED、LANDED、DISARMED。"""
        self.land_status = msg.data.strip()
    
    def set_state(self, new_state):
        """切换 FSM 状态。"""

        if new_state == self.state:  # 状态没变则不处理
            return

        # ==================== 本次修改 1：状态切换时显著打印 ====================
        rospy.loginfo("========== FSM STATE CHANGE: %s -> %s ==========", self.state, new_state)
        # =======================================================================

        self.state = new_state
        self.state_enter_time = rospy.Time.now()
        self.state_pub.publish(String(data=self.state))

        if new_state == "LASER_FLASH":  # 进入激光状态时重置激光标志
            self.laser_started = False

        if new_state == "SEND_RESULT":  # 进入发送结果状态时重置发送标志
            self.result_sent = False

        if new_state == "LAND":
            self.land_requested = False
            self.land_request_last_pub = rospy.Time(0)
            self.land_status = "IDLE"
    #===========================================================================================================================    
    def state_elapsed(self):
        """返回当前状态已经持续的时间。"""
        return (rospy.Time.now() - self.state_enter_time).to_sec()

    def stop_motion(self):
        """发布零速度。"""
        self.cmd_pub.publish(Twist())

    def laser_on(self):
        """打开激光。"""
        self.laser_pub.publish(Bool(data=True))

    def laser_off(self):
        """关闭激光。"""
        self.laser_pub.publish(Bool(data=False))

    def set_home(self):
        """记录起飞点。"""

        self.home_x = self.x
        self.home_y = self.y

        # ====================【高度补偿 2026-05-03】相机高度补偿 ====================
        # 原逻辑：
        #   self.home_z = self.z
        #
        # 问题：
        #   YAML 中写 z=1.50 时，FSM 会让无人机定位点上升 1.50m；
        #   但相机起飞前已经离地 camera_initial_height，
        #   所以相机实际高度会变成 1.50 + camera_initial_height。
        #
        # 新逻辑：
        #   把 home_z 向下虚拟平移 camera_initial_height。
        #   这样所有使用 home_z + z 的目标高度，都会自动降低 camera_initial_height。
        #
        # 注意：
        #   这里是减，不是加。
        self.home_z = self.z - self.camera_initial_height
        # =======================================================================

        self.home_yaw = self.yaw
        self.home_set = True

        rospy.loginfo(
            "Home set: x=%.2f y=%.2f z=%.2f raw_z=%.2f camera_initial_height=%.2f yaw=%.1f deg",
            self.home_x,
            self.home_y,
            self.home_z,
            self.z,
            self.camera_initial_height,
            math.degrees(self.home_yaw)
        )

    def resolve_point(self, point):
        """将 YAML 航点转换成实际 local 坐标。"""

        p = deepcopy(point)

        px = float(point["x"])      # YAML 中的 x 坐标
        py = float(point["y"])      # YAML 中的 y 坐标
        pz = float(point["z"])      # YAML 中的 z 高度
        pyaw = float(point["yaw"])  # YAML 中的目标 yaw

        # ==================== 本次修改 2：相对坐标只平移，不再根据 home_yaw 旋转 ====================
        if self.use_relative:  # 使用相对起飞点坐标
            x_map = self.home_x + px
            y_map = self.home_y + py
            z_map = self.home_z + pz
            yaw_map = pyaw

        else:  # 直接使用 YAML 中的绝对坐标
            x_map = px
            y_map = py
            z_map = pz
            yaw_map = pyaw
        # ==========================================================================================

        p["x"] = x_map
        p["y"] = y_map
        p["z"] = z_map
        p["yaw"] = yaw_map

        return p

    def make_takeoff_target(self):
        """生成起飞目标点。"""

        return {
            "type": "safe",
            "x": self.home_x,
            "y": self.home_y,
            # ==================== 本次修改 1：起飞高度改为相对 home_z ====================
            "z": self.home_z + self.takeoff_height,
            # =======================================================================
            "yaw": self.home_yaw
        }

    def make_land_target(self):
        """生成降落点上方目标点。"""
        return self.resolve_point(self.land_point)

    def goto_target(self, target):
        """使用 P 控制飞向目标点。"""

        ex = target["x"] - self.x
        ey = target["y"] - self.y
        ez = target["z"] - self.z
        eyaw = normalize_angle(target["yaw"] - self.yaw)

        cmd = Twist()

        cmd.linear.x = clamp(self.kp_xy * ex, -self.max_vel_xy, self.max_vel_xy)
        cmd.linear.y = clamp(self.kp_xy * ey, -self.max_vel_xy, self.max_vel_xy)
        cmd.linear.z = clamp(self.kp_z * ez, -self.max_vel_z, self.max_vel_z)
        cmd.angular.z = clamp(self.kp_yaw * eyaw, -self.max_yaw_rate, self.max_yaw_rate)

        self.cmd_pub.publish(cmd)

    def arrived_target(self, target):
        """判断是否到达目标点。"""

        ex = target["x"] - self.x
        ey = target["y"] - self.y
        ez = target["z"] - self.z
        eyaw = normalize_angle(target["yaw"] - self.yaw)

        horizontal_error = math.sqrt(ex * ex + ey * ey)

        return (
            horizontal_error < self.arrive_pos_eps and
            abs(ez) < self.arrive_z_eps and
            abs(eyaw) < self.arrive_yaw_eps
        )

    def publish_body_velocity(self, vx_body, vy_body, vz, yaw_rate):
        """将机体系速度转换到 local 坐标后发布。"""

        c = math.cos(self.yaw)
        s = math.sin(self.yaw)

        vx_map = c * vx_body - s * vy_body
        vy_map = s * vx_body + c * vy_body

        cmd = Twist()
        cmd.linear.x = vx_map
        cmd.linear.y = vy_map
        cmd.linear.z = vz
        cmd.angular.z = yaw_rate

        self.cmd_pub.publish(cmd)

    def fresh_qr_found(self):
        """判断是否有新鲜有效二维码结果。"""

        age = (rospy.Time.now() - self.qr_stamp).to_sec()

        return (
            self.qr["found"] and
            self.qr["id"] > 0 and
            age < 0.4
        )

    def qr_aligned(self):
        """判断二维码中心是否对准激光点。"""

        err_u = self.qr["u"] - self.laser_u
        err_v = self.qr["v"] - self.laser_v

        return (
            abs(err_u) < self.align_pixel_eps and
            abs(err_v) < self.align_pixel_eps
        )

    def visual_align(self):
        """根据二维码像素误差进行视觉微调。"""

        err_u = self.qr["u"] - self.laser_u
        err_v = self.qr["v"] - self.laser_v

        vy_body = -self.align_kp_u * err_u
        vz = -self.align_kp_v * err_v

        vy_body = clamp(vy_body, -self.max_align_vel_y, self.max_align_vel_y)
        vz = clamp(vz, -self.max_align_vel_z, self.max_align_vel_z)

        self.publish_body_velocity(
            vx_body=0.0,
            vy_body=vy_body,
            vz=vz,
            yaw_rate=0.0
        )

    def small_search_motion(self):
        """二维码未出现时做小范围搜索。"""

        t = self.state_elapsed()

        if t < 1.0:  # 先向一侧搜索
            self.publish_body_velocity(0.0, 0.04, 0.0, 0.0)

        elif t < 2.0:  # 再向另一侧搜索
            self.publish_body_velocity(0.0, -0.04, 0.0, 0.0)

        elif t < 2.5:  # 再稍微上移
            self.publish_body_velocity(0.0, 0.0, 0.03, 0.0)

        else:  # 最后稍微下移
            self.publish_body_velocity(0.0, 0.0, -0.03, 0.0)

    def publish_inventory_result(self, status):
        """发布当前扫码点的盘点结果。"""

        if self.current_scan_point is None:  # 没有扫码点则不能发布
            return

        if status == "OK":  # 识别成功时使用二维码编号
            goods_id = int(self.qr["id"])
        else:  # 失败时编号记为 -1
            goods_id = -1

        result = {
            "id": goods_id,
            "face": str(self.current_scan_point.get("face", "-")),
            "slot": int(self.current_scan_point.get("slot", 0)),
            "status": status
        }

        self.result_pub.publish(String(data=json.dumps(result)))
        self.last_sent_goods_id = goods_id

        rospy.loginfo(
            "Inventory result: id=%d face=%s slot=%d status=%s",
            result["id"],
            result["face"],
            result["slot"],
            result["status"]
        )

    def run(self):
        """FSM 主循环。"""

        while not rospy.is_shutdown():  # ROS 未关闭时持续运行
            # ====================【本次安全修改 2026-05-03 5】急停锁存优先级最高 ====================
            if self.emergency_latched:
                self.laser_off()
                self.stop_motion()
                self.publish_stop_to_bridge()  # 低频重复通知 bridge 保持急停悬停锁存
                self.set_state("EMERGENCY_STOP")
                self.loop_rate.sleep()
                continue

            if self.stop_requested:  # 急停优先处理
                self.trigger_emergency_stop("stop_requested flag set")
                self.loop_rate.sleep()
                continue
            # =====================================================================

            if not self.current_pose_ok:  # 没有定位时不允许执行任务
                rospy.logwarn_throttle(1.0, "Waiting for Odometry...")
                self.loop_rate.sleep()
                continue

            if self.state == "IDLE":  # 等待启动
                self.state_idle()

            elif self.state == "TAKEOFF":  # 起飞
                self.state_takeoff()

            elif self.state == "GOTO_MISSION_POINT":  # 飞向航点
                self.state_goto_mission_point()

            elif self.state == "SEARCH_QR":  # 搜索二维码
                self.state_search_qr()

            elif self.state == "ALIGN_QR":  # 对准二维码
                self.state_align_qr()

            elif self.state == "LASER_FLASH":  # 点亮激光
                self.state_laser_flash()

            elif self.state == "SEND_RESULT":  # 发送结果
                self.state_send_result()

            elif self.state == "NEXT_POINT":  # 切换下一个点
                self.state_next_point()

            elif self.state == "RETURN_LAND":  # 返回降落点
                self.state_return_land()

            elif self.state == "LAND":  # 降落
                self.state_land()

            elif self.state == "FINISH":  # 完成
                self.state_finish()

            elif self.state == "EMERGENCY_STOP":  # 急停
                self.state_emergency_stop()

            self.loop_rate.sleep()

    def state_idle(self):
        """IDLE：等待一键启动。"""

        self.stop_motion()
        self.laser_off()

        if self.start_requested:  # 收到启动后记录起飞点并进入起飞
            self.start_requested = False
            self.stop_requested = False

            self.set_home()

            self.current_index = 0
            self.current_scan_point = None

            # ====================【新增】新任务启动时清除上一次降落状态 ====================
            self.land_requested = False
            self.land_status = "IDLE"
            self.land_request_last_pub = rospy.Time(0)
            # ======================================================================

            self.set_state("TAKEOFF")

    def state_takeoff(self):
        """TAKEOFF：起飞到指定高度。"""

        target = self.make_takeoff_target()
        self.goto_target(target)

        if self.arrived_target(target):  # 到达起飞高度后进入任务航点
            self.stop_motion()
            self.set_state("GOTO_MISSION_POINT")

    def state_goto_mission_point(self):
        """GOTO_MISSION_POINT：飞到当前任务点。"""

        if self.current_index >= len(self.points):  # 所有航点完成后返航降落
            self.set_state("RETURN_LAND")
            return

        raw_point = self.points[self.current_index]
        target = self.resolve_point(raw_point)

        self.goto_target(target)

        if not self.arrived_target(target):  # 未到达则继续飞
            return

        self.stop_motion()

        point_type = raw_point.get("type", "safe")

        if point_type == "safe":  # 安全点到达后直接进入下一个航点
            self.current_index += 1
            self.set_state("GOTO_MISSION_POINT")
            return

        if point_type == "scan":  # 扫码点到达后开始搜索二维码
            self.current_scan_point = raw_point
            self.set_state("SEARCH_QR")
            return

        rospy.logwarn("Unknown point type: %s", point_type)
        self.current_index += 1
        self.set_state("GOTO_MISSION_POINT")

    def state_search_qr(self):
        """SEARCH_QR：搜索二维码。"""

        if self.fresh_qr_found():  # 找到二维码后进入对准
            self.stop_motion()
            self.set_state("ALIGN_QR")
            return

        if self.state_elapsed() < self.search_timeout:  # 未超时时继续小范围搜索
            self.small_search_motion()
            return

        rospy.logwarn("QR search timeout.")
        self.stop_motion()
        self.publish_inventory_result(status="FAIL")
        self.set_state("NEXT_POINT")

    def state_align_qr(self):
        """ALIGN_QR：对准二维码。"""

        if not self.fresh_qr_found():  # 二维码丢失则重新搜索
            self.stop_motion()
            self.set_state("SEARCH_QR")
            return

        if self.qr_aligned():  # 对准后点亮激光
            self.stop_motion()
            self.set_state("LASER_FLASH")
            return

        if self.state_elapsed() > self.align_timeout:  # 对准超时则记录失败
            rospy.logwarn("QR align timeout.")
            self.stop_motion()
            self.publish_inventory_result(status="FAIL")
            self.set_state("NEXT_POINT")
            return

        self.visual_align()

    def state_laser_flash(self):
        """LASER_FLASH：激光点亮一段时间。"""

        self.stop_motion()

        if not self.laser_started:  # 第一次进入时打开激光并重新计时
            self.laser_started = True
            self.laser_on()
            self.state_enter_time = rospy.Time.now()
            return

        if self.state_elapsed() >= self.laser_flash_time:  # 到时间后关闭激光
            self.laser_off()
            self.set_state("SEND_RESULT")

    def state_send_result(self):
        """SEND_RESULT：发布盘点结果。"""

        if not self.result_sent:  # 只发送一次结果
            self.publish_inventory_result(status="OK")
            self.result_sent = True

        expected_ack = str(self.last_sent_goods_id)

        if self.last_ack_id == expected_ack:  # 收到地面站 ACK 后进入下一个点
            self.set_state("NEXT_POINT")
            return

        if self.state_elapsed() > self.ack_timeout:  # 没有 ACK 也不阻塞任务
            self.set_state("NEXT_POINT")

    def state_next_point(self):
        """NEXT_POINT：切换到下一个航点。"""

        self.current_index += 1
        self.current_scan_point = None

        self.set_state("GOTO_MISSION_POINT")

    def state_return_land(self):
        """RETURN_LAND：飞到降落点上方。"""

        target = self.make_land_target()
        self.goto_target(target)

        if self.arrived_target(target):  # 到达降落点上方后开始下降
            self.stop_motion()
            self.set_state("LAND")

    def state_land(self):
        """LAND：请求 mavros_sitl_bridge 切 AUTO.LAND，并等待真正落地上锁。"""

        # ====================【重大修改】不再由 FSM 自己发 -0.12m/s 慢慢降到底 ====================
        # 原逻辑：cmd.linear.z = -0.12，z <= home_z + 0.18 后直接 FINISH。
        # 问题：PX4 可能还没判定 landed，此时直接 disarm 会出现 disarming denied。
        # 新逻辑：FSM 只发布 /uav/land，请 mavros_sitl_bridge 执行：
        #        AUTO.LAND -> 等 landed_state=ON_GROUND -> arming false -> 回传 DISARMED。
        # =================================================================================

        self.stop_motion()  # LAND 状态中不再继续给桥发送旧速度指令
        self.laser_off()    # 降落阶段确保激光关闭

        now = rospy.Time.now()  # 当前 ROS 时间

        if (
            not self.land_requested or
            now - self.land_request_last_pub > rospy.Duration(1.0)
        ):
            self.land_pub.publish(Bool(data=True))
            self.land_requested = True
            self.land_request_last_pub = now
            rospy.logwarn_throttle(1.0, "FSM LAND: publishing /uav/land=True, waiting for bridge DISARMED.")

        if self.land_status == "DISARMED":  # bridge 已确认 PX4 上锁
            rospy.logwarn("FSM LAND: bridge reports DISARMED, mission FINISH.")
            self.stop_motion()
            self.set_state("FINISH")
            return

        if self.state_elapsed() > self.land_finish_timeout:  # 超时只报警，不强行 FINISH
            rospy.logwarn_throttle(1.0,
                "FSM LAND timeout: still waiting for DISARMED, current /uav/land_status=%s",
                self.land_status
            )

    def state_finish(self):
        """FINISH：任务完成。"""
        self.stop_motion()
        self.laser_off()

    def state_emergency_stop(self):
        """EMERGENCY_STOP：急停悬停锁存，不再继续旧任务。"""
        self.stop_motion()
        self.laser_off()
        self.publish_stop_to_bridge()  # 低频提醒 bridge 保持急停悬停锁存


if __name__ == "__main__":
    try:
        node = Requirement1FSM()
        node.run()
    except rospy.ROSInterruptException:
        pass