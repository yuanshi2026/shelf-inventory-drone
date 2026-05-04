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

        # ====================【本次安全修改 2026-05-03 1】急停悬停锁存状态 ====================
        # abort_latched 一旦置 True，bridge 不再自动切 OFFBOARD、不再自动解锁、不再转发旧速度。
        # /uav/stop 现在只表示“紧急悬停锁存”，不会直接停桨，也不会自动继续旧任务。
        # 只有收到 /uav/reset=True 且飞机已经上锁后，才允许清除锁存并重新开始任务。
        self.abort_latched = False               # 急停锁存标志
        self.abort_reason = ""                   # 急停原因，便于终端排查
        # =================================================================================

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
            "/uav/start_task2",
            Bool,
            self.start_callback,
            queue_size=5
        )  # 订阅任务2启动信号

        rospy.Subscriber(
            "/uav/stop",
            Bool,
            self.stop_callback,
            queue_size=5
        )  # 订阅急停/停止信号

        # ====================【本次修改-安全 2】人工复位入口 ====================
        rospy.Subscriber(
            "/uav/reset",
            Bool,
            self.reset_callback,
            queue_size=5
        )  # 急停后必须人工复位，才允许再次启动
        # ====================================================================

        # ====================【新增】正常结束控制话题，不再复用 /uav/stop ====================
        rospy.Subscriber(
            "/uav/land",
            Bool,
            self.land_callback,
            queue_size=5
        )  # FSM 受控下降接地/近地后发布 True，请求 PX4 AUTO.LAND 停桨/上锁

        rospy.Subscriber(
            "/uav/request_land",
            Bool,
            self.controlled_land_callback,
            queue_size=5
        )  # 急停悬停后的受控降落请求：释放 ABORT_HOVER，允许 FSM 继续发速度下降

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
        """保存 MAVROS 当前状态，并检测飞行中异常退出。"""
        prev_armed = self.current_state.armed        # 上一帧是否 armed
        prev_mode = self.current_state.mode          # 上一帧飞行模式
        self.current_state = msg

        # ====================【本次修改-安全 3】检测 kill / 退出 OFFBOARD ====================
        # 如果任务执行中突然 disarm，常见原因是遥控器 kill 或飞控保护触发。
        # 如果任务执行中突然退出 OFFBOARD，说明外部控制链路已不可靠。
        # 这两种情况都必须锁死，避免 kill 解除后自动重新 arm 并冲向旧航点。
        active_control = (
            self.start_requested and
            not self.land_requested and
            not self.disarm_requested and
            not self.stop_requested and
            not self.abort_latched
        )

        if active_control and prev_armed and not msg.armed:  # 飞行中突然上锁/kill
            self.trigger_abort("armed changed True -> False during mission")
            return

        if active_control and prev_mode == "OFFBOARD" and msg.mode != "OFFBOARD":  # 飞行中退出 OFFBOARD
            self.trigger_abort("mode changed OFFBOARD -> %s during mission" % msg.mode)
            return
        # =====================================================================

    def extended_state_callback(self, msg):
        """保存 MAVROS 扩展状态，主要用于判断 PX4 是否确认落地。"""
        self.extended_state = msg
        self.extended_state_ok = True

    def cmd_callback(self, msg):
        """保存 FSM 最新速度指令。"""
        # ====================【本次修改-安全 4】急停/降落时不再缓存任务速度 ====================
        # 否则 kill 解除后，bridge 可能继续转发 FSM 为旧航点计算出来的新速度。
        if (
            self.abort_latched or
            self.stop_requested or
            self.land_requested or
            self.disarm_requested or
            not self.start_requested
        ):
            self.last_cmd = self.zero_cmd()
            self.last_cmd_time = rospy.Time.now()
            return
        # =====================================================================

        self.last_cmd = msg
        self.last_cmd_time = rospy.Time.now()

    def start_callback(self, msg):
        """收到一键启动后，允许切 OFFBOARD 并解锁。"""
        if msg.data:  # 收到 True 才启动
            # ====================【本次修改-安全 5】急停锁存后禁止直接重启 ====================
            if self.abort_latched:
                rospy.logerr(
                    "Start ignored: abort_latched=True, reason=%s. Publish /uav/reset after disarmed first.",
                    self.abort_reason
                )
                return
            # =====================================================================

            rospy.loginfo("Start requested.")
            self.start_requested = True
            self.stop_requested = False

            # ====================【新增】新任务开始时清除上一次降落/上锁请求 ====================
            self.land_requested = False
            self.disarm_requested = False
            self.last_cmd = self.zero_cmd()
            self.last_cmd_time = rospy.Time.now()
            self.publish_land_status("IDLE")
            # =====================================================================

    def stop_callback(self, msg):
        """收到急停信号后，进入悬停锁存；不继续任务，也不直接降落/停桨。"""
        if msg.data:  # 收到 True 才停止
            # ====================【本次安全修改 2026-05-03 2】/uav/stop 改为紧急悬停锁存 ====================
            # 旧逻辑：/uav/stop 后立刻设置 land_requested=True，马上尝试 AUTO.LAND。
            # 新逻辑：/uav/stop 只锁死旧任务并持续发布零速度，让飞机先悬停刹住。
            # 后续由人工确认安全后，通过 /uav/land 或手动接管完成降落；落地上锁后再 /uav/reset。
            self.trigger_abort("/uav/stop received, emergency hover latched")
            self.land_requested = False
            self.disarm_requested = False
            self.publish_land_status("ABORT_HOVER")
            # =====================================================================================

    # ====================【本次安全修改 2026-05-03 3】急停锁存与人工复位 ====================
    def trigger_abort(self, reason):
        """进入急停悬停锁存：停止旧任务、清零速度、禁止自动 OFFBOARD/ARM。"""
        if not self.abort_latched:  # 第一次进入急停时打印原因
            rospy.logerr("ABORT HOVER LATCHED: %s", reason)

        self.abort_latched = True
        self.abort_reason = reason
        self.start_requested = False
        self.stop_requested = True
        self.land_requested = False
        self.disarm_requested = False
        self.last_cmd = self.zero_cmd()
        self.last_cmd_time = rospy.Time.now()
        self.vel_pub.publish(self.zero_cmd())
        self.publish_land_status("ABORT_HOVER")

    def reset_callback(self, msg):
        """急停后人工复位；只有已经上锁时才允许清除 abort_latched。"""
        if not msg.data:  # 只响应 True
            return

        # ====================【本次安全修改 2026-05-03 4】复位必须在已上锁后生效 ====================
        # 这里把规则写死：飞机仍 armed 时拒绝 reset，避免空中清状态后再次 start 造成危险。
        if self.current_state.armed:  # 飞机还 armed 时禁止复位
            rospy.logerr("Reset rejected: vehicle is still armed. Land/disarm first, then move back to start point.")
            self.publish_land_status("RESET_REJECTED_ARMED")
            return
        # =====================================================================================

        rospy.logwarn("Abort reset accepted. Bridge returns to IDLE.")
        self.abort_latched = False
        self.abort_reason = ""
        self.start_requested = False
        self.stop_requested = False
        self.land_requested = False
        self.disarm_requested = False
        self.last_cmd = self.zero_cmd()
        self.last_cmd_time = rospy.Time.now()
        self.publish_land_status("IDLE")
    # ========================================================================

    # ====================【新增】/uav/request_land 受控降落入口 ====================
    def controlled_land_callback(self, msg):
        """急停悬停后，允许 FSM 接管速度，先递减 z 受控下降。

        注意：这里不切 AUTO.LAND；真正的 AUTO.LAND 由 FSM 在接地/近地后发布 /uav/land 触发。
        """
        if not msg.data:
            return

        rospy.logwarn("/uav/request_land received. Release ABORT_HOVER for controlled descent; final /uav/land will stop props.")
        self.abort_latched = False
        self.abort_reason = ""
        self.stop_requested = False
        self.land_requested = False
        self.disarm_requested = False
        self.start_requested = True
        self.last_cmd = self.zero_cmd()
        self.last_cmd_time = rospy.Time.now()
        self.publish_land_status("CONTROLLED_LAND")

    # ====================【新增】/uav/land 最终 AUTO.LAND 停桨入口 ====================
    def land_callback(self, msg):
        """请求 PX4 进入 AUTO.LAND；现在主要用于受控下降接地/近地后的停桨上锁。"""
        if msg.data:  # 收到 True 才执行降落流程
            # 注意：CMD:LAND 不应再直接打到 /uav/land。
            # /uav/land 只作为受控下降接地/近地后的最终停桨上锁请求。
            rospy.logwarn("/uav/land received. Final AUTO.LAND/disarm requested.")
            self.start_requested = False
            self.stop_requested = False
            self.land_requested = True
            self.disarm_requested = False
            self.last_cmd = self.zero_cmd()
            self.last_cmd_time = rospy.Time.now()
            self.publish_land_status("LAND_REQUESTED")
            # =================================================================================
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
        """主循环：飞行时转发速度 setpoint；急停时悬停锁存，降落时执行 AUTO.LAND + DISARM。"""
        rate = rospy.Rate(self.rate_hz)  # 控制循环频率

        while not rospy.is_shutdown():  # ROS 未关闭时持续运行
            # ====================【本次安全修改 2026-05-03 6】主循环优先级重排 ====================
            # 1. /uav/land 或 /uav/disarm：进入 AUTO.LAND + 落地后上锁流程。
            # 2. abort_latched 或 stop_requested：只发布零速度悬停锁存，不自动降落、不恢复旧任务。
            # 3. start_requested：正常 OFFBOARD/ARM/速度转发。
            if self.land_requested or self.disarm_requested:
                self.handle_land_and_disarm()
                rate.sleep()
                continue

            if self.abort_latched or self.stop_requested:
                self.vel_pub.publish(self.zero_cmd())
                rate.sleep()
                continue
            # =================================================================================

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
