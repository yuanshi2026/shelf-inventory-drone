#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL


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

        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))       # setpoint 发布频率
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.5)) # 指令超时时间

        self.current_state = State()          # MAVROS 当前状态
        self.last_cmd = Twist()               # 最近一次收到的速度指令
        self.last_cmd_time = rospy.Time.now() # 最近一次收到速度指令的时间

        self.start_requested = False          # 是否收到一键启动
        self.stop_requested = False           # 是否收到停止指令
        self.land_sent = False                # 是否已经发送过降落命令
        self.last_req = rospy.Time.now()      # 上一次请求 OFFBOARD / ARM 的时间

        rospy.Subscriber(
            "/mavros/state",
            State,
            self.state_callback,
            queue_size=10
        )  # 订阅 MAVROS 状态

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
        )  # 订阅停止信号

        self.vel_pub = rospy.Publisher(
            self.mavros_vel_topic,
            Twist,
            queue_size=20
        )  # 发布 MAVROS 速度 setpoint

        rospy.loginfo("Waiting for MAVROS services...")
        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.wait_for_service("/mavros/set_mode")

        self.arming_client = rospy.ServiceProxy(
            "/mavros/cmd/arming",
            CommandBool
        )  # 解锁服务客户端

        self.set_mode_client = rospy.ServiceProxy(
            "/mavros/set_mode",
            SetMode
        )  # 切模式服务客户端

        try:
            rospy.wait_for_service("/mavros/cmd/land", timeout=3.0)
            self.land_client = rospy.ServiceProxy(
                "/mavros/cmd/land",
                CommandTOL
            )  # 降落服务客户端
        except Exception:
            self.land_client = None
            rospy.logwarn("No /mavros/cmd/land service, stop will only send zero velocity.")

        rospy.loginfo("mavros_sitl_bridge started.")

    def state_callback(self, msg):
        """保存 MAVROS 当前状态。"""
        self.current_state = msg

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
            self.land_sent = False

    def stop_callback(self, msg):
        """收到停止信号后，进入停止 / 降落流程。"""
        if msg.data:  # 收到 True 才停止
            rospy.logwarn("Stop requested.")
            self.stop_requested = True
            self.start_requested = False

    def zero_cmd(self):
        """生成零速度指令。"""
        return Twist()

    def get_safe_cmd(self):
        """返回安全速度指令；若 FSM 指令超时则返回零速度。"""

        age = (rospy.Time.now() - self.last_cmd_time).to_sec()  # 距离上次速度指令的时间

        if age > self.cmd_timeout:  # 超时则不再使用旧指令
            return self.zero_cmd()

        return self.last_cmd

    def try_enter_offboard_and_arm(self):
        """尝试切换 OFFBOARD 模式并解锁。"""

        now = rospy.Time.now()  # 当前 ROS 时间

        if now - self.last_req < rospy.Duration(1.0):  # 限制服务调用频率
            return

        if self.current_state.mode != "OFFBOARD":  # 还不是 OFFBOARD 时先切模式
            try:
                response = self.set_mode_client(custom_mode="OFFBOARD")
                if response.mode_sent:  # 飞控接受切模式请求
                    rospy.loginfo("OFFBOARD mode requested.")
                else:
                    rospy.logwarn("OFFBOARD request rejected.")
            except Exception as e:
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
            except Exception as e:
                rospy.logwarn("Arming failed: %s", str(e))

            self.last_req = now

    def try_land(self):
        """尝试调用 MAVROS 降落服务。"""

        if self.land_sent:  # 避免重复发送降落命令
            return

        self.land_sent = True

        if self.land_client is None:  # 没有降落服务则只发送零速度
            rospy.logwarn("No land service, only zero velocity.")
            return

        try:
            self.land_client(
                min_pitch=0.0,
                yaw=0.0,
                latitude=0.0,
                longitude=0.0,
                altitude=0.0
            )
            rospy.logwarn("Land command sent.")
        except Exception as e:
            rospy.logwarn("Land failed: %s", str(e))

    def run(self):
        """主循环：持续发布 MAVROS setpoint。"""

        rate = rospy.Rate(self.rate_hz)  # 控制循环频率

        while not rospy.is_shutdown():  # ROS 未关闭时持续运行
            if self.stop_requested:  # 停止优先级最高
                self.vel_pub.publish(self.zero_cmd())
                self.try_land()
                rate.sleep()
                continue

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