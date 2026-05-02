#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode


class MavrosSITLBridge:
    def __init__(self):
        rospy.init_node("mavros_sitl_bridge")

        self.cmd_topic = rospy.get_param(
            "~cmd_topic",
            "/uav/cmd_vel"
        )  # FSM 输出速度话题

        self.mavros_vel_topic = rospy.get_param(
            "~mavros_vel_topic",
            "/mavros/setpoint_velocity/cmd_vel_unstamped"
        )  # MAVROS 速度控制话题

        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))        # setpoint 发布频率
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.5))  # 指令超时时间

        self.current_state = State()             # MAVROS 当前状态，包含 mode/armed/connected
        self.extended_state = ExtendedState()    # MAVROS 扩展状态，包含 landed_state
        self.extended_state_ok = False           # 是否已经收到过 /mavros/extended_state

        self.last_cmd = Twist()                  # 最近一次收到的速度指令
        self.last_cmd_time = rospy.Time.now()    # 最近一次收到速度指令的时间

        self.start_requested = False             # 是否允许进入 OFFBOARD 并解锁
        self.stop_requested = False              # 是否收到急停/停止请求

        # ====================【新增】正常任务结束降落/上锁状态变量 ====================
        # land_requested：收到 /uav/land 后置 True，bridge 负责切 AUTO.LAND。
        # disarm_requested：落地后置 True，bridge 负责调用 /mavros/cmd/arming false。
        # land_status：发布给 FSM 的降落状态，FSM 等到 DISARMED 后才进入 FINISH。
        self.land_requested = False
        self.disarm_requested = False
        self.land_status = "IDLE"
        self.last_land_status = ""
        # ======================================================================

        self.last_req = rospy.Time.now()         # 上一次请求 OFFBOARD/ARM/LAND/DISARM 的时间

        rospy.Subscriber(
            "/mavros/state",
            State,
            self.state_callback,
            queue_size=10
        )  # 订阅 MAVROS 基本状态

        # ====================【新增】订阅 landed_state，确认真正落地后再 disarm ====================
        rospy.Subscriber(
            "/mavros/extended_state",
            ExtendedState,
            self.extended_state_callback,
            queue_size=10
        )  # 订阅 MAVROS 扩展状态，用 landed_state 判断是否落地
        # ===========================================================================

        rospy.Subscriber(
            self.cmd_topic,
            Twist,
            self.cmd_callback,
            queue_size=20
        )  # 订阅 FSM 速度指令

        rospy.Subscriber(
            "/uav/start",
            Bool,
            self.start_callback,
            queue_size=5
        )  # 订阅一键启动信号

        rospy.Subscriber(
            "/uav/stop",
            Bool,
            self.stop_callback,
            queue_size=5
        )  # 订阅急停/停止信号

        # ====================【新增】正常结束控制话题，不再复用 /uav/stop ====================
        rospy.Subscriber(
            "/uav/land",
            Bool,
            self.land_callback,
            queue_size=5
        )  # FSM 正常任务完成后发布 True，请求 PX4 AUTO.LAND

        rospy.Subscriber(
            "/uav/disarm",
            Bool,
            self.disarm_callback,
            queue_size=5
        )  # 手动安全上锁请求；若还在空中，会先走 AUTO.LAND
        # ====================================================================

        self.vel_pub = rospy.Publisher(
            self.mavros_vel_topic,
            Twist,
            queue_size=20
        )  # 发布 MAVROS 速度 setpoint

        # ====================【新增】bridge 向 FSM 回报降落/上锁进度 ====================
        self.land_status_pub = rospy.Publisher(
            "/uav/land_status",
            String,
            queue_size=10,
            latch=True
        )  # FSM 订阅该话题，收到 DISARMED 后进入 FINISH
        # =====================================================================

        rospy.loginfo("Waiting for MAVROS services...")
        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.wait_for_service("/mavros/set_mode")

        self.arming_client = rospy.ServiceProxy(
            "/mavros/cmd/arming",
            CommandBool
        )  # 解锁/上锁服务客户端

        self.set_mode_client = rospy.ServiceProxy(
            "/mavros/set_mode",
            SetMode
        )  # 切模式服务客户端

        self.publish_land_status("IDLE")
        rospy.loginfo("mavros_sitl_bridge started.")

    def state_callback(self, msg):
        """保存 MAVROS 当前状态。"""
        self.current_state = msg

    def extended_state_callback(self, msg):
        """保存 MAVROS 扩展状态，主要用于判断 PX4 是否确认落地。"""
        self.extended_state = msg
        self.extended_state_ok = True

    def cmd_callback(self, msg):
        """保存 FSM 最新速度指令。"""
        self.last_cmd = msg
        self.last_cmd_time = rospy.Time.now()

    def start_callback(self, msg):
        """收到一键启动后，允许切 OFFBOARD 并解锁。"""
        if msg.data:  # 收到 True 才启动
            rospy.loginfo("Start requested.")
            self.start_requested = True
            self.stop_requested = False

            # ====================【新增】新任务开始时清除上一次降落/上锁请求 ====================
            self.land_requested = False
            self.disarm_requested = False
            self.publish_land_status("IDLE")
            # =====================================================================

    def stop_callback(self, msg):
        """收到急停/停止信号后，走 AUTO.LAND，而不是直接 disarm。"""
        if msg.data:  # 收到 True 才停止
            rospy.logwarn("Stop requested. Switch to AUTO.LAND, then disarm after landed.")
            self.start_requested = False
            self.stop_requested = True

            # ====================【新增】急停也不直接上锁，先降落，再等 landed_state ====================
            self.land_requested = True
            self.disarm_requested = False
            self.publish_land_status("LAND_REQUESTED_BY_STOP")
            # ===========================================================================

    # ====================【新增】/uav/land 正常任务结束降落入口 ====================
    def land_callback(self, msg):
        """FSM 正常任务完成后，请求 PX4 进入 AUTO.LAND。"""
        if msg.data:  # 收到 True 才执行降落流程
            rospy.logwarn("/uav/land received. Switch PX4 to AUTO.LAND.")
            self.start_requested = False
            self.stop_requested = False
            self.land_requested = True
            self.disarm_requested = False
            self.publish_land_status("LAND_REQUESTED")
    # ========================================================================

    # ====================【新增】/uav/disarm 安全上锁入口 ====================
    def disarm_callback(self, msg):
        """手动请求安全上锁；如果尚未落地，则先进入 AUTO.LAND。"""
        if msg.data:  # 收到 True 才处理
            rospy.logwarn("/uav/disarm received. Safe disarm requested.")
            self.start_requested = False
            self.stop_requested = False
            self.disarm_requested = True

            if not self.is_landed():  # 空中不直接上锁，避免 PX4 拒绝或危险
                self.land_requested = True
                self.publish_land_status("DISARM_WAIT_LAND")
            else:
                self.publish_land_status("DISARM_REQUESTED")
    # ======================================================================

    def publish_land_status(self, status):
        """发布降落/上锁状态；状态变化时同时打印日志。"""
        self.land_status = status
        self.land_status_pub.publish(String(data=status))

        if status != self.last_land_status:  # 状态变化时打印，避免刷屏
            rospy.logwarn("Landing status: %s", status)
            self.last_land_status = status

    def zero_cmd(self):
        """生成零速度指令。"""
        return Twist()

    def get_safe_cmd(self):
        """返回安全速度指令；若 FSM 指令超时则返回零速度。"""
        age = (rospy.Time.now() - self.last_cmd_time).to_sec()  # 距离上次速度指令的时间

        if age > self.cmd_timeout:  # 超时则不再使用旧指令
            return self.zero_cmd()

        return self.last_cmd

    def is_landed(self):
        """判断 PX4 是否已经确认落地。"""
        return (
            self.extended_state_ok and
            self.extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND
        )

    def try_enter_offboard_and_arm(self):
        """尝试切换 OFFBOARD 模式并解锁。"""
        now = rospy.Time.now()  # 当前 ROS 时间

        if now - self.last_req < rospy.Duration(1.0):  # 限制服务调用频率
            return

        if self.current_state.mode != "OFFBOARD":  # 还不是 OFFBOARD 时先切模式
            try:
                response = self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                if response.mode_sent:  # 飞控接受切模式请求
                    rospy.loginfo("OFFBOARD mode requested.")
                else:
                    rospy.logwarn("OFFBOARD request rejected.")
            except Exception as e:  # 服务调用失败时进入这里
                rospy.logwarn("Set OFFBOARD failed: %s", str(e))

            self.last_req = now
            return

        if not self.current_state.armed:  # 已经是 OFFBOARD 后再解锁
            try:
                response = self.arming_client(True)
                if response.success:  # 飞控接受解锁请求
                    rospy.loginfo("Arm requested.")
                else:
                    rospy.logwarn("Arm request rejected.")
            except Exception as e:  # 服务调用失败时进入这里
                rospy.logwarn("Arming failed: %s", str(e))

            self.last_req = now

    # ====================【新增】切 PX4 AUTO.LAND，而不是自己用速度降落到底 ====================
    def try_set_auto_land(self):
        """请求 PX4 进入 AUTO.LAND 模式；成功后等待 landed_state。"""
        now = rospy.Time.now()  # 当前 ROS 时间

        if self.current_state.mode == "AUTO.LAND":  # 已经处于自动降落模式
            self.publish_land_status("WAIT_LANDED")
            return

        if now - self.last_req < rospy.Duration(1.0):  # 限制服务调用频率
            return

        try:
            response = self.set_mode_client(base_mode=0, custom_mode="AUTO.LAND")
            if response.mode_sent:  # PX4 接受 AUTO.LAND 模式请求
                self.publish_land_status("AUTO_LAND_SENT")
                rospy.logwarn("AUTO.LAND mode requested.")
            else:
                self.publish_land_status("AUTO_LAND_REJECTED")
                rospy.logwarn("AUTO.LAND request rejected.")
        except Exception as e:  # 服务调用失败时进入这里
            self.publish_land_status("AUTO_LAND_FAILED")
            rospy.logwarn("Set AUTO.LAND failed: %s", str(e))

        self.last_req = now
    # ======================================================================

    # ====================【新增】只在 PX4 确认落地后调用 disarm ====================
    def try_disarm_after_landed(self):
        """等待 landed_state=ON_GROUND 后，再调用 /mavros/cmd/arming false。"""
        now = rospy.Time.now()  # 当前 ROS 时间

        if not self.current_state.armed:  # 已经上锁，流程完成
            self.land_requested = False
            self.stop_requested = False
            self.disarm_requested = False
            self.publish_land_status("DISARMED")
            return

        if not self.is_landed():  # 未确认落地时，绝不直接 disarm
            if not self.extended_state_ok:
                self.publish_land_status("WAIT_EXTENDED_STATE")
            else:
                self.publish_land_status("WAIT_LANDED")
            return

        if now - self.last_req < rospy.Duration(1.0):  # 限制服务调用频率
            return

        try:
            response = self.arming_client(False)
            if response.success:  # PX4 接受上锁请求
                self.publish_land_status("DISARMED")
                self.land_requested = False
                self.stop_requested = False
                self.disarm_requested = False
                rospy.logwarn("Disarm requested after PX4 confirmed landed.")
            else:
                self.publish_land_status("DISARM_DENIED")
                rospy.logwarn("Disarm denied, will retry after landed check.")
        except Exception as e:  # 服务调用失败时进入这里
            self.publish_land_status("DISARM_FAILED")
            rospy.logwarn("Disarm failed: %s", str(e))

        self.last_req = now
    # ======================================================================

    # ====================【新增】完整降落/上锁流程调度 ====================
    def handle_land_and_disarm(self):
        """先切 AUTO.LAND，再等 PX4 确认落地，最后上锁。"""
        self.vel_pub.publish(self.zero_cmd())  # 降落流程中不继续转发 FSM 旧速度

        if self.land_requested:  # 已请求降落时，持续尝试进入 AUTO.LAND
            self.try_set_auto_land()

        if self.is_landed():  # PX4 确认落地后，才允许进入 disarm 流程
            self.publish_land_status("LANDED")
            self.disarm_requested = True

        if self.disarm_requested:  # 请求上锁时，只在落地后真正调用 arming false
            self.try_disarm_after_landed()
    # ==================================================================

    def run(self):
        """主循环：飞行时转发速度 setpoint；结束时执行 AUTO.LAND + DISARM。"""
        rate = rospy.Rate(self.rate_hz)  # 控制循环频率

        while not rospy.is_shutdown():  # ROS 未关闭时持续运行
            # ====================【新增】降落/上锁流程优先级最高 ====================
            if self.land_requested or self.disarm_requested or self.stop_requested:
                self.handle_land_and_disarm()
                rate.sleep()
                continue
            # ================================================================

            if self.start_requested:  # 启动后进入 OFFBOARD 控制流程
                self.try_enter_offboard_and_arm()
                self.vel_pub.publish(self.get_safe_cmd())
            else:
                self.vel_pub.publish(self.zero_cmd())  # 未启动时持续发零 setpoint

            rate.sleep()


if __name__ == "__main__":
    try:
        node = MavrosSITLBridge()
        node.run()
    except rospy.ROSInterruptException:
        pass
