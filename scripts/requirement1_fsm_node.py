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

        # ====================【修改 2026-05-04】扫码点使用更严格到达阈值 ====================
        # 作用：
        #   B5/B2 这类直线中间扫码点如果仍用 0.15m 到达圈，容易刚进圈就带速度进入扫码。
        #   safe 过渡点继续用 arrive_pos_eps，scan 点单独用 scan_arrive_*。
        self.scan_arrive_pos_eps = float(
            self.mission.get("scan_arrive_pos_eps", 0.06)
        )
        self.scan_arrive_z_eps = float(
            self.mission.get("scan_arrive_z_eps", 0.06)
        )
        self.scan_arrive_yaw_eps = math.radians(
            float(self.mission.get("scan_arrive_yaw_eps_deg", 6.0))
        )
        # =================================================================================

        self.kp_xy = float(self.mission.get("kp_xy", 0.45))   # 水平 P 控制系数
        self.kp_z = float(self.mission.get("kp_z", 0.45))     # 高度 P 控制系数
        self.kp_yaw = float(self.mission.get("kp_yaw", 0.60)) # 偏航 P 控制系数

        self.max_vel_xy = float(self.mission.get("max_vel_xy", 0.25))       # 最大水平速度
        self.max_vel_z = float(self.mission.get("max_vel_z", 0.15))         # 最大竖直速度
        self.max_yaw_rate = float(self.mission.get("max_yaw_rate", 0.40))   # 最大偏航角速度

        # ====================【实飞稳定修改 2026-05-03】速度规划与限加速度参数 ====================
        # 说明：
        #   原来的 goto_target() 是简单 P 控制，误差一大速度立刻打满，到点后又立刻切零，
        #   真机上容易出现冲、晃、刹不住。
        #
        #   新控制逻辑采用“距离限速 + 指令限加速度”：
        #       1. 离目标远时允许接近最大速度；
        #       2. 离目标近时按刹车距离自动降速；
        #       3. 每一帧速度变化量受 max_acc_* 限制，避免速度突变。
        self.max_acc_xy = float(self.mission.get("max_acc_xy", 0.18))       # 水平方向最大速度变化率，单位 m/s^2
        self.max_acc_z = float(self.mission.get("max_acc_z", 0.12))         # 竖直方向最大速度变化率，单位 m/s^2
        self.max_acc_yaw = float(self.mission.get("max_acc_yaw", 0.35))     # 偏航角速度最大变化率，单位 rad/s^2

        # ====================【微调 2026-05-04】起飞专用竖直速度 ====================
        # 起飞阶段可以略快一点，但不影响后续扫二维码时的高度控制参数。
        self.takeoff_max_vel_z = float(
            self.mission.get("takeoff_max_vel_z", self.max_vel_z)
        ) # 起飞阶段最大竖直速度，单位 m/s
        self.takeoff_max_acc_z = float(
            self.mission.get("takeoff_max_acc_z", self.max_acc_z)
        ) # 起飞阶段竖直加速度限幅，单位 m/s^2
        # =======================================================================

        self.min_slow_distance_xy = float(
            self.mission.get("min_slow_distance_xy", 0.20)
        ) # 水平距离小于该值附近时明显减速，避免到点冲过

        self.min_slow_distance_z = float(
            self.mission.get("min_slow_distance_z", 0.15)
        ) # 高度误差小于该值附近时明显减速

        self.yaw_first_threshold = math.radians(
            float(self.mission.get("yaw_first_threshold_deg", 18.0))
        ) # 偏航误差大于该角度时，优先原地转向，减少边转边飘

        self.arrive_hold_time = float(
            self.mission.get("arrive_hold_time", 0.40)
        ) # 到达航点后稳定等待时间，单位 s

        self.arrive_hold_start = None       # 到达阈值后的稳定计时起点
        self.smooth_cmd = Twist()           # 上一次限加速度后的速度指令
        self.last_control_time = rospy.Time.now() # 上一次速度规划更新时间
        # =================================================================================

        self.laser_u = float(self.mission.get("laser_u", 424.0)) # 激光点图像横坐标
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

        # ====================【实飞稳定修改 2026-05-03】扫描点只悬停，不再移动搜索 ====================
        # 到达 scan 点后，FSM 只悬停 scan_hover_time 秒。
        # 没有真二维码/假二维码时，悬停结束后记录 FAIL 并继续下一个点。
        # 这样不会再执行横移、反向横移、上移、下移等搜索动作。
        self.scan_hover_time = float(self.mission.get("scan_hover_time", 2.0)) # 扫描点悬停等待时间
        self.scan_no_qr_policy = str(
            self.mission.get("scan_no_qr_policy", "fail_continue")
        ) # fail_continue：无二维码记 FAIL；fake_ok：无二维码也生成演示 OK

        # ====================【稳定悬停增强 2026-05-03】扫码前先停稳，再开始识别 ====================
        # 说明：
        #   到达 scan 航点后，不立刻打开视觉、不立刻对准二维码；
        #   先进入 HOVER_BEFORE_SCAN 状态，纯零速度悬停刹车一段时间。
        #   这样能避免“刚冲进到达误差圈，机体还有残余速度就开始扫码”造成摇摆。
        self.scan_pre_hover_time = float(
            self.mission.get("scan_pre_hover_time", 0.5)
        ) # 扫码前最短锁点悬停时间，单位 s

        self.scan_pre_hover_timeout = float(
            self.mission.get("scan_pre_hover_timeout", 2.0)
        ) # 速度噪声导致无法满足停稳条件时，最多等待时间，单位 s

        self.settle_vel_xy_eps = float(
            self.mission.get("settle_vel_xy_eps", 0.05)
        ) # 判定水平速度已停稳的阈值，单位 m/s

        self.settle_vel_z_eps = float(
            self.mission.get("settle_vel_z_eps", 0.04)
        ) # 判定竖直速度已停稳的阈值，单位 m/s

        self.settle_yaw_rate_eps = float(
            self.mission.get("settle_yaw_rate_eps", 0.12)
        ) # 判定偏航角速度已停稳的阈值，单位 rad/s
        # =================================================================================

        # ====================【锁点悬停 2026-05-04】扫描阶段锁定任务坐标系目标 ====================
        # 说明：
        #   scan 点到达后，不再只发 0 速度，而是持续锁定该 scan 航点的 x/y/z/yaw。
        #   这里的 x/y/z/yaw 来自 YAML 相对坐标 resolve_point() 后的任务目标，
        #   不是某一瞬间的飞机实际位姿，所以不会把“还没停稳/已经偏航”的姿态固化下来。
        self.scan_hold_kp_xy = float(self.mission.get("scan_hold_kp_xy", 0.35))
        self.scan_hold_kp_z = float(self.mission.get("scan_hold_kp_z", 0.45))
        self.scan_hold_kp_yaw = float(self.mission.get("scan_hold_kp_yaw", 0.80))

        self.scan_hold_max_vel_xy = float(self.mission.get("scan_hold_max_vel_xy", 0.08))
        self.scan_hold_max_vel_z = float(self.mission.get("scan_hold_max_vel_z", 0.06))
        self.scan_hold_max_yaw_rate = float(self.mission.get("scan_hold_max_yaw_rate", 0.30))

        self.scan_hold_deadband_xy = float(self.mission.get("scan_hold_deadband_xy", 0.025))
        self.scan_hold_deadband_z = float(self.mission.get("scan_hold_deadband_z", 0.025))
        self.scan_hold_deadband_yaw = math.radians(
            float(self.mission.get("scan_hold_deadband_yaw_deg", 2.0))
        )

        # mission：激光/结果阶段继续锁 YAML 航点；
        # aligned_xyz：二维码对准成功后，锁当前 x/y/z，但 yaw 仍锁 YAML 航向。
        self.scan_lock_after_align_mode = str(
            self.mission.get("scan_lock_after_align_mode", "mission")
        )
        # =================================================================================

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

        # ====================【新增】视觉节点使能控制 ====================
        self.vision_enable_pub = rospy.Publisher(
            "/vision/enable",
            Bool,
            queue_size=5
        ) # 扫码相关状态打开视觉节点，其它状态关闭，避免视觉算法常驻占资源
        # ===============================================================

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

        # ====================【稳定悬停增强 2026-05-03】保存里程计速度，用于判断是否真正停稳 ====================
        # 这些速度来自 Odometry 的 twist 字段，不参与路径规划，只用于“到点后是否已经停稳”的判断。
        self.vx = 0.0                # 当前 local x 方向速度，单位 m/s
        self.vy = 0.0                # 当前 local y 方向速度，单位 m/s
        self.vz = 0.0                # 当前 z 方向速度，单位 m/s
        self.yaw_rate = 0.0          # 当前偏航角速度，单位 rad/s
        # =================================================================================

        self.home_set = False        # 是否已记录起飞点
        self.home_x = 0.0            # 起飞点 x
        self.home_y = 0.0            # 起飞点 y
        self.home_z = 0.0            # 起飞点 z
        self.home_yaw = 0.0          # 起飞时偏航角

        self.state = "IDLE"          # 当前 FSM 状态
        self.state_enter_time = rospy.Time.now() # 进入当前状态的时间

        self.current_index = 0       # 当前执行的航点序号
        self.current_scan_point = None # 当前扫码点信息

        # ====================【锁点悬停 2026-05-04】当前扫描阶段锁定目标 ====================
        # scan_hold_target 保存的是 resolve_point(raw_point) 后的任务目标，
        # 也就是 home_x/home_y/home_z 加上 YAML 相对坐标后的 x/y/z/yaw。
        # 它不是当前实际位姿，因此不会因为飞机尚未停稳或 yaw 已经偏了而保存错误方向。
        self.scan_hold_target = None
        # =================================================================================

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
        self.set_vision(False)
        self.state_pub.publish(String(data=self.state))

    def set_vision(self, enabled):
        """控制二维码视觉节点是否执行识别。"""
        self.vision_enable_pub.publish(Bool(data=bool(enabled)))

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

        # ====================【稳定悬停增强 2026-05-03】读取 Odometry 中的速度 ====================
        # 注意：/mavros/local_position/odom 通常会提供 twist，用来判断飞机是否真的停稳。
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.vz = msg.twist.twist.linear.z
        self.yaw_rate = msg.twist.twist.angular.z
        # =================================================================================

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
        self.set_vision(False)
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
        self.scan_hold_target = None
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
        self.arrive_hold_start = None
        self.smooth_cmd = Twist()
        self.last_control_time = rospy.Time.now()
        self.set_vision(False)
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

        # ====================【新增】根据 FSM 状态控制视觉节点 ====================
        if new_state in ["SEARCH_QR", "ALIGN_QR", "LASER_FLASH", "SEND_RESULT"]:
            self.set_vision(True)
        else:
            self.set_vision(False)
        # ===============================================================

        # ====================【实飞稳定修改 2026-05-03】进入新目标状态时重置到点稳定计时 ====================
        if new_state in ["TAKEOFF", "GOTO_MISSION_POINT", "RETURN_LAND"]:
            self.arrive_hold_start = None
        # =================================================================================
    #===========================================================================================================================    
    def state_elapsed(self):
        """返回当前状态已经持续的时间。"""
        return (rospy.Time.now() - self.state_enter_time).to_sec()

    def stop_motion(self):
        """立即发布零速度。

        说明：
            该函数用于急停、结束、降落、激光照射等必须立刻清零速度的场景。
            普通到点刹车不要调用它，而是调用 brake_motion()，这样速度会按限加速度平滑归零。
        """
        self.smooth_cmd = Twist()
        self.last_control_time = rospy.Time.now()
        self.cmd_pub.publish(Twist())

    # ====================【实飞稳定修改 2026-05-03】速度规划基础函数 ====================
    def limit_step(self, target_value, current_value, max_delta):
        """限制单次速度变化量，避免速度指令突然跳变。"""

        delta = target_value - current_value

        if delta > max_delta:  # 增量过大时，只允许增加 max_delta
            return current_value + max_delta

        if delta < -max_delta:  # 减量过大时，只允许减少 max_delta
            return current_value - max_delta

        return target_value

    def publish_smooth_cmd(self, target_cmd):
        """发布经过限加速度处理后的速度指令。"""

        now = rospy.Time.now()
        dt = (now - self.last_control_time).to_sec()

        if dt <= 0.0 or dt > 0.20:  # 时间异常或循环卡顿时，按 30Hz 估算
            dt = 1.0 / 30.0

        self.last_control_time = now

        max_dxy = self.max_acc_xy * dt       # 本周期水平速度最多允许变化量
        max_dz = self.max_acc_z * dt         # 本周期竖直速度最多允许变化量
        max_dyaw = self.max_acc_yaw * dt     # 本周期偏航角速度最多允许变化量

        self.smooth_cmd.linear.x = self.limit_step(
            target_cmd.linear.x,
            self.smooth_cmd.linear.x,
            max_dxy
        )

        self.smooth_cmd.linear.y = self.limit_step(
            target_cmd.linear.y,
            self.smooth_cmd.linear.y,
            max_dxy
        )

        self.smooth_cmd.linear.z = self.limit_step(
            target_cmd.linear.z,
            self.smooth_cmd.linear.z,
            max_dz
        )

        self.smooth_cmd.angular.z = self.limit_step(
            target_cmd.angular.z,
            self.smooth_cmd.angular.z,
            max_dyaw
        )

        self.cmd_pub.publish(self.smooth_cmd)

    def brake_motion(self):
        """平滑刹车，用于普通到点后的稳定等待。"""
        self.publish_smooth_cmd(Twist())

    def distance_limited_speed(self, error_abs, max_speed, max_acc, slow_distance):
        """根据剩余距离计算目标速度大小。

        逻辑：
            距离较远时速度接近 max_speed；
            距离较近时用线性降速和 sqrt(2*a*s) 双重约束，
            让无人机接近航点时提前变慢，而不是冲进阈值后突然刹车。
        """

        if error_abs <= 0.0:  # 没有误差时速度为 0
            return 0.0

        slow_distance = max(slow_distance, 1e-3)
        ratio_speed = max_speed * min(1.0, error_abs / slow_distance)
        brake_speed = math.sqrt(max(0.0, 2.0 * max_acc * error_abs))

        return min(max_speed, ratio_speed, brake_speed)
    # =================================================================================

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

    def goto_target(self, target, max_vel_z=None, max_acc_z=None):
        """使用“距离限速 + 限加速度”的方式飞向目标点。

        与旧 P 控制的区别：
            旧逻辑直接 cmd = kp * error，误差大时速度突然打满；
            新逻辑先根据剩余距离规划目标速度，再用 publish_smooth_cmd() 限制速度变化率，
            因此起步、刹车、切航点都会更平滑。
        """

        ex = target["x"] - self.x
        ey = target["y"] - self.y
        ez = target["z"] - self.z
        eyaw = normalize_angle(target["yaw"] - self.yaw)

        horizontal_error = math.sqrt(ex * ex + ey * ey)

        cmd = Twist()

        # 起飞阶段可传入略大的竖直速度/加速度；其他阶段默认使用全局稳定参数。
        if max_vel_z is None:
            max_vel_z = self.max_vel_z
        if max_acc_z is None:
            max_acc_z = self.max_acc_z

        yaw_speed = self.distance_limited_speed(
            abs(eyaw),
            self.max_yaw_rate,
            self.max_acc_yaw,
            math.radians(8.0)
        )
        cmd.angular.z = math.copysign(yaw_speed, eyaw) if abs(eyaw) > 1e-4 else 0.0

        # 偏航误差较大时先转向，水平速度置零，避免无人机边转边斜着飘。
        if abs(eyaw) > self.yaw_first_threshold:
            z_speed = self.distance_limited_speed(
                abs(ez),
                max_vel_z,
                max_acc_z,
                self.min_slow_distance_z
            )
            cmd.linear.z = math.copysign(z_speed, ez) if abs(ez) > 1e-4 else 0.0
            self.publish_smooth_cmd(cmd)
            return

        if horizontal_error > 1e-4:  # 有水平误差时，沿误差方向生成速度向量
            xy_speed = self.distance_limited_speed(
                horizontal_error,
                self.max_vel_xy,
                self.max_acc_xy,
                self.min_slow_distance_xy
            )
            cmd.linear.x = xy_speed * ex / horizontal_error
            cmd.linear.y = xy_speed * ey / horizontal_error

        z_speed = self.distance_limited_speed(
            abs(ez),
            max_vel_z,
            max_acc_z,
            self.min_slow_distance_z
        )
        cmd.linear.z = math.copysign(z_speed, ez) if abs(ez) > 1e-4 else 0.0

        self.publish_smooth_cmd(cmd)

    def arrived_target(self, target, pos_eps=None, z_eps=None, yaw_eps=None):
        """判断是否到达目标点。

        pos_eps / z_eps / yaw_eps 允许扫码点使用更严格阈值，
        但不改变 GOTO -> HOVER -> SEARCH 的整体状态转换。
        """

        if pos_eps is None:
            pos_eps = self.arrive_pos_eps
        if z_eps is None:
            z_eps = self.arrive_z_eps
        if yaw_eps is None:
            yaw_eps = self.arrive_yaw_eps

        ex = target["x"] - self.x
        ey = target["y"] - self.y
        ez = target["z"] - self.z
        eyaw = normalize_angle(target["yaw"] - self.yaw)

        horizontal_error = math.sqrt(ex * ex + ey * ey)

        return (
            horizontal_error < pos_eps and
            abs(ez) < z_eps and
            abs(eyaw) < yaw_eps
        )

    def velocity_stable(self):
        """判断无人机速度是否已经足够小，避免带速度进入扫码阶段。"""

        vel_xy = math.sqrt(self.vx * self.vx + self.vy * self.vy)

        return (
            vel_xy < self.settle_vel_xy_eps and
            abs(self.vz) < self.settle_vel_z_eps and
            abs(self.yaw_rate) < self.settle_yaw_rate_eps
        )

    def compute_scan_hold_cmd(self, target):
        """根据任务坐标系锁定目标，计算扫描阶段的 xyz/yaw 纠偏速度。"""

        cmd = Twist()

        if target is None:  # 没有锁定目标时，不输出纠偏速度
            return cmd

        ex = float(target["x"]) - self.x
        ey = float(target["y"]) - self.y
        ez = float(target["z"]) - self.z
        eyaw = normalize_angle(float(target["yaw"]) - self.yaw)

        horizontal_error = math.sqrt(ex * ex + ey * ey)

        if horizontal_error > self.scan_hold_deadband_xy:  # 水平偏离超过死区时，拉回任务航点
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

        if abs(eyaw) > self.scan_hold_deadband_yaw:  # yaw 偏离超过死区时，拉回任务航向角
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
        """扫描、激光、发结果期间持续锁定任务坐标系 x/y/z/yaw。"""

        if self.scan_hold_target is None:  # 非扫描状态下退化为平滑刹车
            self.brake_motion()
            return

        cmd = self.compute_scan_hold_cmd(self.scan_hold_target)
        cmd = self.limit_scan_cmd(cmd)
        self.publish_smooth_cmd(cmd)

    def maybe_lock_aligned_xyz_keep_mission_yaw(self):
        """可选：对准成功后锁当前 x/y/z，但 yaw 仍保持 YAML 任务航向。"""

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

        self.publish_smooth_cmd(cmd)

    # ====================【实飞稳定修改 2026-05-03】到点后稳定等待 ====================
    def reached_and_hold(self, target, pos_eps=None, z_eps=None, yaw_eps=None, hold_target=False):
        """判断是否到达目标，并在目标附近稳定等待 arrive_hold_time。

        hold_target=True 时，进入到达阈值后不再只刹车，而是继续小速度锁定目标点。
        这用于扫码点，避免 B5/B2 这类中间点刚进阈值就带速度进入扫码。
        """

        if not self.arrived_target(target, pos_eps, z_eps, yaw_eps):  # 还没进入到达阈值，继续正常飞行
            self.arrive_hold_start = None
            return False

        if hold_target:
            # 扫码点进入阈值后继续拉回目标 x/y/z/yaw，而不是只发刹车命令。
            cmd = self.compute_scan_hold_cmd(target)
            cmd = self.limit_scan_cmd(cmd)
            self.publish_smooth_cmd(cmd)
        else:
            self.brake_motion()  # safe/起飞/降落点仍保持原来的平滑刹车等待

        if self.arrive_hold_start is None:
            self.arrive_hold_start = rospy.Time.now()
            return False

        if rospy.Time.now() - self.arrive_hold_start < rospy.Duration(self.arrive_hold_time):
            return False

        self.arrive_hold_start = None
        return True
    # =================================================================================

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
        """根据二维码像素误差微调，同时继续锁定任务 yaw，避免扫码时原地转。"""

        err_u = self.qr["u"] - self.laser_u
        err_v = self.qr["v"] - self.laser_v

        vy_body = -self.align_kp_u * err_u
        vz = -self.align_kp_v * err_v

        vy_body = clamp(vy_body, -self.max_align_vel_y, self.max_align_vel_y)
        vz = clamp(vz, -self.max_align_vel_z, self.max_align_vel_z)

        # 基础锁点命令负责把位置和 yaw 拉回任务坐标系目标。
        # 这样视觉对准阶段即使二维码像素有抖动，也不会放任机体原地偏航。
        cmd = self.compute_scan_hold_cmd(self.scan_hold_target)

        # ====================【修改 2026-05-04】视觉对齐禁止改变货架距离 x ====================
        # 原来使用当前 yaw 把机体系横向速度转换到 map，yaw 只要不是严格 0/pi，
        # 就会产生少量 x 方向速度，B 面可能被慢慢拉向货架。
        # 现在按 YAML 任务航向计算横向修正，并只允许视觉修正 map-y 和 z；
        # map-x 始终由上面的锁点控制负责保持 50cm 扫描距离。
        mission_yaw = float(self.scan_hold_target["yaw"]) if self.scan_hold_target is not None else self.yaw
        cmd.linear.y += math.cos(mission_yaw) * vy_body
        cmd.linear.z += vz
        # =================================================================================

        cmd = self.limit_scan_cmd(
            cmd,
            max_xy=max(self.scan_hold_max_vel_xy, self.max_align_vel_y),
            max_z=max(self.scan_hold_max_vel_z, self.max_align_vel_z),
            max_yaw=self.scan_hold_max_yaw_rate
        )
        self.publish_smooth_cmd(cmd)

    def small_search_motion(self):
        """二维码搜索动作已取消。

        保留这个函数名是为了兼容旧代码，但现在不会再发布横移/升降速度。
        真二维码稳定接入前，扫描点只悬停，不主动找码。
        """
        self.hold_scan_position()

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

            elif self.state == "HOVER_BEFORE_SCAN":  # 扫码前先停稳
                self.state_hover_before_scan()

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

        if self.reached_and_hold(target):  # 起飞高度稳定 0.4 秒后再进入任务航点
            self.stop_motion()
            self.set_state("GOTO_MISSION_POINT")
            return

        if self.arrive_hold_start is None:  # 还没到达阈值时才继续飞；到达后保持刹车等待
            self.goto_target(
                target,
                max_vel_z=self.takeoff_max_vel_z,
                max_acc_z=self.takeoff_max_acc_z
            )

    def state_goto_mission_point(self):
        """GOTO_MISSION_POINT：飞到当前任务点。"""

        if self.current_index >= len(self.points):  # 所有航点完成后返航降落
            self.set_state("RETURN_LAND")
            return

        raw_point = self.points[self.current_index]
        target = self.resolve_point(raw_point)
        point_type = raw_point.get("type", "safe")

        is_scan_point = (point_type == "scan")
        pos_eps = self.scan_arrive_pos_eps if is_scan_point else self.arrive_pos_eps
        z_eps = self.scan_arrive_z_eps if is_scan_point else self.arrive_z_eps
        yaw_eps = self.scan_arrive_yaw_eps if is_scan_point else self.arrive_yaw_eps

        if not self.reached_and_hold(
            target,
            pos_eps=pos_eps,
            z_eps=z_eps,
            yaw_eps=yaw_eps,
            hold_target=is_scan_point
        ):  # 未到达或刚到点未稳定够 arrive_hold_time
            if self.arrive_hold_start is None:  # 还没到达阈值时才继续飞；到达后由 reached_and_hold 继续锁点/刹车等待
                self.goto_target(target)
            return

        self.stop_motion()

        if point_type == "safe":  # 安全点到达后直接进入下一个航点
            self.current_index += 1
            self.set_state("GOTO_MISSION_POINT")
            return

        if point_type == "scan":  # 扫码点到达后先停稳，再开始搜索二维码
            self.current_scan_point = raw_point
            # 这里锁的是任务坐标系下的目标 x/y/z/yaw，不是当前实际位姿。
            # 因此即使飞机刚进入误差圈还没完全停稳，也不会把偏掉的位置/航向保存下来。
            self.scan_hold_target = target
            self.set_state("HOVER_BEFORE_SCAN")
            return

        rospy.logwarn("Unknown point type: %s", point_type)
        self.current_index += 1
        self.set_state("GOTO_MISSION_POINT")

    def state_hover_before_scan(self):
        """HOVER_BEFORE_SCAN：进入扫码点后先纯悬停，等机体真的停稳再打开视觉。"""

        self.set_vision(False)   # 停稳阶段不识别，避免视觉结果提前触发对准动作
        self.hold_scan_position() # 持续锁定任务 x/y/z/yaw，而不是只发 0 速度

        elapsed = self.state_elapsed()

        if elapsed < self.scan_pre_hover_time:  # 至少悬停 scan_pre_hover_time 秒
            return

        if self.velocity_stable():  # 速度也足够小，才正式开始扫码
            self.qr = {
                "found": False,
                "id": -1,
                "u": -1.0,
                "v": -1.0,
                "area": 0.0
            }
            self.qr_stamp = rospy.Time(0)
            self.stop_motion()
            self.set_state("SEARCH_QR")
            return

        if elapsed > self.scan_pre_hover_timeout:  # 防止里程计速度噪声导致永久卡住
            rospy.logwarn(
                "Scan pre-hover timeout, continue scan. vx=%.3f vy=%.3f vz=%.3f yaw_rate=%.3f",
                self.vx,
                self.vy,
                self.vz,
                self.yaw_rate
            )
            self.stop_motion()
            self.set_state("SEARCH_QR")

    def state_search_qr(self):
        """SEARCH_QR：正式扫码阶段，只原地等待二维码，不再移动搜索。"""

        # ====================【稳定悬停增强 2026-05-03】正式扫码，不做任何搜索运动 ====================
        # 与旧版 scan_hover_time 的区别：
        #   旧版是先固定悬停 scan_hover_time，悬停结束后才检查二维码；
        #   新版是进入 SEARCH_QR 后立刻允许识别，识别到就进入 ALIGN_QR，
        #   没识别到则原地等待 search_timeout，不再横移/上下搜索。
        self.set_vision(True)
        self.hold_scan_position()

        if self.fresh_qr_found():  # 一旦识别到新鲜二维码，进入对准/点亮流程
            self.stop_motion()
            self.set_state("ALIGN_QR")
            return

        if self.state_elapsed() < self.search_timeout:  # 未超时则继续原地等待
            return

        self.stop_motion()

        if self.scan_no_qr_policy == "fake_ok":  # 仅用于演示：无二维码也生成一个假结果
            fake_id = (self.current_index % 24) + 1
            self.qr = {
                "found": True,
                "id": fake_id,
                "u": self.laser_u,
                "v": self.laser_v,
                "area": 4000.0
            }
            self.qr_stamp = rospy.Time.now()
            self.publish_inventory_result(status="OK")
        else:
            self.publish_inventory_result(status="FAIL")

        self.set_state("NEXT_POINT")
        # =================================================================================

    def state_align_qr(self):
        """ALIGN_QR：对准二维码。"""

        self.set_vision(True)

        if not self.fresh_qr_found():  # 二维码丢失则重新搜索
            self.stop_motion()
            self.set_state("SEARCH_QR")
            return

        if self.qr_aligned():  # 对准后点亮激光
            self.maybe_lock_aligned_xyz_keep_mission_yaw()
            self.stop_motion()
            self.set_state("LASER_FLASH")
            return

        if self.state_elapsed() > self.align_timeout:  # 对准超时：二维码已识别，仍然点亮激光并上传结果
            # ====================【修改 2026-05-04】对齐失败不再丢弃扫码结果 ====================
            # 逻辑说明：
            #   能进入 ALIGN_QR，说明 SEARCH_QR 阶段已经读到了二维码编号；
            #   对齐超时只代表激光点没有完全对到二维码中心，不能代表二维码识别失败。
            #   按当前比赛展示需求：即使对齐失败，也要点亮激光，并把已识别到的二维码编号上传地面站。
            # 后续流程：
            #   LASER_FLASH 会打开 /laser/cmd；
            #   SEND_RESULT 会 publish_inventory_result(status="OK")，地面站收到 SCAN:faceSlot:id。
            rospy.logwarn(
                "QR align timeout, but QR id=%d has been detected. Flash laser and send scan result.",
                int(self.qr["id"])
            )
            self.stop_motion()
            self.set_state("LASER_FLASH")
            return
            # ===========================================================================

        self.visual_align()

    def state_laser_flash(self):
        """LASER_FLASH：激光点亮一段时间。"""

        self.hold_scan_position()  # 激光照射期间继续锁定任务 x/y/z/yaw

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

        self.hold_scan_position()  # 等待 ACK 时继续锁定任务 x/y/z/yaw

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
        self.scan_hold_target = None

        self.set_state("GOTO_MISSION_POINT")

    def state_return_land(self):
        """RETURN_LAND：飞到降落点上方。"""

        target = self.make_land_target()

        if self.reached_and_hold(target):  # 降落点上方稳定 0.4 秒后再请求 AUTO.LAND
            self.stop_motion()
            self.set_state("LAND")
            return

        if self.arrive_hold_start is None:  # 还没到达阈值时才继续飞；到达后保持刹车等待
            self.goto_target(target)

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
        self.set_vision(False)
        self.stop_motion()
        self.laser_off()

    def state_emergency_stop(self):
        """EMERGENCY_STOP：急停悬停锁存，不再继续旧任务。"""
        self.set_vision(False)
        self.stop_motion()
        self.laser_off()
        self.publish_stop_to_bridge()  # 低频提醒 bridge 保持急停悬停锁存


if __name__ == "__main__":
    try:
        node = Requirement1FSM()
        node.run()
    except rospy.ROSInterruptException:
        pass