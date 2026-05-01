#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class MavrosPoseToOdom:
    def __init__(self):
        rospy.init_node("mavros_pose_to_odom")

        self.pose_topic = rospy.get_param(
            "~pose_topic",
            "/mavros/local_position/pose"
        )  # 输入位姿话题，来自 MAVROS

        self.odom_topic = rospy.get_param(
            "~odom_topic",
            "/Odometry"
        )  # 输出里程计话题，给 FSM 使用

        self.odom_pub = rospy.Publisher(
            self.odom_topic,
            Odometry,
            queue_size=20
        )  # Odometry 发布器

        rospy.Subscriber(
            self.pose_topic,
            PoseStamped,
            self.pose_callback,
            queue_size=20
        )  # 订阅 MAVROS 位姿

        rospy.loginfo("mavros_pose_to_odom started.")
        rospy.loginfo("Subscribe: %s", self.pose_topic)
        rospy.loginfo("Publish: %s", self.odom_topic)

    def pose_callback(self, msg):
        """将 PoseStamped 转成 Odometry。"""

        odom = Odometry()  # 新建 Odometry 消息

        odom.header = msg.header                # 沿用原始时间戳和坐标系
        odom.child_frame_id = "base_link"       # 无人机机体系名称
        odom.pose.pose = msg.pose               # 复制位置和姿态

        self.odom_pub.publish(odom)             # 发布给 FSM


if __name__ == "__main__":
    try:
        node = MavrosPoseToOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass