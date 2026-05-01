#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import rospy
from std_msgs.msg import String


class SimFakeQRNode:
    def __init__(self):
        rospy.init_node("sim_fake_qr_node")

        self.laser_u = float(rospy.get_param("~laser_u", 320.0)) # 假二维码中心横坐标
        self.laser_v = float(rospy.get_param("~laser_v", 240.0)) # 假二维码中心纵坐标

        self.state = "IDLE"       # 当前 FSM 状态
        self.last_state = "IDLE"  # 上一次 FSM 状态
        self.fake_id = 0          # 当前假二维码编号

        self.qr_pub = rospy.Publisher(
            "/qr/result",
            String,
            queue_size=20
        )  # 二维码结果发布器

        rospy.Subscriber(
            "/fsm/state",
            String,
            self.state_callback,
            queue_size=20
        )  # 订阅 FSM 状态

        rospy.loginfo("sim_fake_qr_node started.")

    def state_callback(self, msg):
        """根据 FSM 状态决定是否生成新的假二维码编号。"""

        self.last_state = self.state
        self.state = msg.data

        if self.state == "SEARCH_QR" and self.last_state != "SEARCH_QR":  # 刚进入搜索二维码状态
            self.fake_id += 1

            if self.fake_id > 24:  # 货物编号限制在 1~24
                self.fake_id = 1

            rospy.loginfo("Fake QR id = %d", self.fake_id)

    def make_qr_msg(self, found):
        """生成二维码识别结果消息。"""

        if found:  # 当前状态需要模拟识别成功
            data = {
                "found": True,
                "id": self.fake_id,
                "text": str(self.fake_id),
                "u": self.laser_u,
                "v": self.laser_v,
                "area": 4000.0,
                "stamp": rospy.Time.now().to_sec()
            }
        else:  # 当前状态不应该看到二维码
            data = {
                "found": False,
                "id": -1,
                "text": "",
                "u": -1.0,
                "v": -1.0,
                "area": 0.0,
                "stamp": rospy.Time.now().to_sec()
            }

        return String(data=json.dumps(data))

    def run(self):
        """主循环：根据 FSM 状态持续发布假二维码结果。"""

        rate = rospy.Rate(20)  # 假二维码发布频率

        while not rospy.is_shutdown():  # ROS 未关闭时持续运行
            active_states = [
                "SEARCH_QR",
                "ALIGN_QR",
                "LASER_FLASH",
                "SEND_RESULT"
            ]  # 这些状态下认为二维码可见

            found = self.state in active_states and self.fake_id > 0  # 判断是否发布 found=true

            self.qr_pub.publish(self.make_qr_msg(found))
            rate.sleep()


if __name__ == "__main__":
    try:
        node = SimFakeQRNode()
        node.run()
    except rospy.ROSInterruptException:
        pass