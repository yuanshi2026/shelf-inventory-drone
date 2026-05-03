#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from mavros_msgs.msg import State


class MissionManager:
    """
    Simple task arbiter.

    - FSM1 publishes /task1/cmd_vel
    - FSM2 publishes /task2/cmd_vel
    - This node forwards only the active task velocity to /uav/cmd_vel
    - mavros_sitl_bridge subscribes /uav/cmd_vel and talks to MAVROS

    Emergency policy with e6 mavros bridge:
    - /uav/request_stop -> manager publishes /uav/stop=True
    - e6 bridge interprets /uav/stop as ABORT_HOVER latch, not auto-land
    - Ground station should later send CMD:LAND -> UDP bridge publishes /uav/land=True
    - After DISARMED, ground station sends CMD:RESET -> UDP bridge publishes /uav/reset=True
    - This manager returns to IDLE only after reset is accepted while disarmed
    """

    def __init__(self):
        rospy.init_node("mission_manager_node")

        self.mode = "IDLE"  # IDLE / TASK1 / TASK2 / EMERGENCY
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.5))
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))

        self.task1_cmd = Twist()
        self.task2_cmd = Twist()
        self.task1_cmd_time = rospy.Time(0)
        self.task2_cmd_time = rospy.Time(0)

        self.mavros_state = State()
        self.mavros_state_ok = False
        self.land_status = "IDLE"

        # Final velocity output to mavros_sitl_bridge
        self.cmd_pub = rospy.Publisher("/uav/cmd_vel", Twist, queue_size=20)

        # Current mission mode for debugging / FSM reference
        self.mode_pub = rospy.Publisher("/mission/mode", String, queue_size=10, latch=True)

        # Start signals to concrete FSMs
        self.task1_start_pub = rospy.Publisher("/uav/start", Bool, queue_size=5)
        self.task2_start_pub = rospy.Publisher("/uav/start_task2", Bool, queue_size=5)

        # Emergency stop to FSMs and e6 mavros bridge
        self.stop_pub = rospy.Publisher("/uav/stop", Bool, queue_size=5)

        # Requests from UDP bridge
        rospy.Subscriber("/uav/request_task1", Bool, self.request_task1_callback, queue_size=5)
        rospy.Subscriber("/uav/request_task2", Bool, self.request_task2_callback, queue_size=5)
        rospy.Subscriber("/uav/request_stop", Bool, self.request_stop_callback, queue_size=5)

        # Reset and status signals used by e6 bridge/FSM1
        # UDP bridge should publish /uav/reset=True directly when receiving CMD:RESET.
        rospy.Subscriber("/uav/reset", Bool, self.reset_callback, queue_size=5)
        rospy.Subscriber("/uav/land_status", String, self.land_status_callback, queue_size=10)
        rospy.Subscriber("/mavros/state", State, self.mavros_state_callback, queue_size=10)

        # Velocity inputs from FSMs
        rospy.Subscriber("/task1/cmd_vel", Twist, self.task1_cmd_callback, queue_size=20)
        rospy.Subscriber("/task2/cmd_vel", Twist, self.task2_cmd_callback, queue_size=20)

        # Task completion states
        rospy.Subscriber("/fsm/state", String, self.fsm1_state_callback, queue_size=20)
        rospy.Subscriber("/fsm2/state", String, self.fsm2_state_callback, queue_size=20)

        rospy.loginfo("mission_manager_node_e6 started.")
        self.publish_mode()

    def zero_cmd(self):
        return Twist()

    def publish_mode(self):
        self.mode_pub.publish(String(data=self.mode))

    def set_mode(self, new_mode):
        if new_mode == self.mode:
            return
        rospy.logwarn("MISSION MODE: %s -> %s", self.mode, new_mode)
        self.mode = new_mode
        self.publish_mode()

    def mavros_state_callback(self, msg):
        self.mavros_state = msg
        self.mavros_state_ok = True

    def land_status_callback(self, msg):
        self.land_status = msg.data.strip()

    def request_task1_callback(self, msg):
        if not msg.data:
            return

        if self.mode != "IDLE":
            rospy.logwarn("Reject TASK1 request: current mode is %s", self.mode)
            return

        self.set_mode("TASK1")
        rospy.sleep(0.1)
        self.task1_start_pub.publish(Bool(data=True))
        rospy.logwarn("TASK1 accepted: published /uav/start=True")

    def request_task2_callback(self, msg):
        if not msg.data:
            return

        if self.mode != "IDLE":
            rospy.logwarn("Reject TASK2 request: current mode is %s", self.mode)
            return

        self.set_mode("TASK2")
        rospy.sleep(0.1)
        self.task2_start_pub.publish(Bool(data=True))
        rospy.logwarn("TASK2 accepted: published /uav/start_task2=True")

    def request_stop_callback(self, msg):
        if not msg.data:
            return

        rospy.logerr("Emergency stop requested: entering EMERGENCY mode and publishing /uav/stop=True")
        self.cmd_pub.publish(self.zero_cmd())
        self.set_mode("EMERGENCY")
        self.stop_pub.publish(Bool(data=True))

    def reset_callback(self, msg):
        """Return mission manager to IDLE after e6 bridge/FSM reset.

        Safety rule: do not clear manager EMERGENCY while MAVROS still reports armed.
        This matches the e6 bridge/FSM reset policy.
        """
        if not msg.data:
            return

        if not self.mavros_state_ok:
            rospy.logerr("Mission reset rejected: no /mavros/state yet; cannot prove disarmed.")
            return

        if self.mavros_state.armed:
            rospy.logerr("Mission reset rejected: vehicle is still armed.")
            return

        rospy.logwarn("Mission reset accepted: manager returns to IDLE.")
        self.task1_cmd = self.zero_cmd()
        self.task2_cmd = self.zero_cmd()
        self.cmd_pub.publish(self.zero_cmd())
        self.set_mode("IDLE")

    def task1_cmd_callback(self, msg):
        self.task1_cmd = msg
        self.task1_cmd_time = rospy.Time.now()

    def task2_cmd_callback(self, msg):
        self.task2_cmd = msg
        self.task2_cmd_time = rospy.Time.now()

    def fsm1_state_callback(self, msg):
        state = msg.data.strip()

        if self.mode == "TASK1" and state == "FINISH":
            rospy.logwarn("TASK1 finished: manager returns to IDLE.")
            self.cmd_pub.publish(self.zero_cmd())
            self.set_mode("IDLE")

        if self.mode == "TASK1" and state == "EMERGENCY_STOP":
            rospy.logerr("TASK1 reported EMERGENCY_STOP: manager enters EMERGENCY.")
            self.cmd_pub.publish(self.zero_cmd())
            self.set_mode("EMERGENCY")

    def fsm2_state_callback(self, msg):
        state = msg.data.strip()

        if self.mode == "TASK2" and state == "FINISH":
            rospy.logwarn("TASK2 finished: manager returns to IDLE.")
            self.cmd_pub.publish(self.zero_cmd())
            self.set_mode("IDLE")

        if self.mode == "TASK2" and state == "EMERGENCY_STOP":
            rospy.logerr("TASK2 reported EMERGENCY_STOP: manager enters EMERGENCY.")
            self.cmd_pub.publish(self.zero_cmd())
            self.set_mode("EMERGENCY")

    def get_active_cmd(self):
        now = rospy.Time.now()

        if self.mode == "TASK1":
            age = (now - self.task1_cmd_time).to_sec()
            if age > self.cmd_timeout:
                return self.zero_cmd()
            return self.task1_cmd

        if self.mode == "TASK2":
            age = (now - self.task2_cmd_time).to_sec()
            if age > self.cmd_timeout:
                return self.zero_cmd()
            return self.task2_cmd

        # IDLE / EMERGENCY: never forward task velocity
        return self.zero_cmd()

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.cmd_pub.publish(self.get_active_cmd())
            rate.sleep()


if __name__ == "__main__":
    try:
        node = MissionManager()
        node.run()
    except rospy.ROSInterruptException:
        pass
