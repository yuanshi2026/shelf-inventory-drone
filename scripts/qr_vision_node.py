#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import re
import json
import time
import os

import cv2
import numpy as np
import rospy
import pyrealsense2 as rs

from pyzbar import pyzbar
from std_msgs.msg import Bool, String


class QRVisionNode:
    def __init__(self):
        rospy.init_node("qr_vision_node")

        # =========================
        # 参数
        # =========================
        self.enable_topic = rospy.get_param("~enable_topic", "/vision/enable")
        self.qr_result_topic = rospy.get_param("~qr_result_topic", "/qr/result")

        self.width = int(rospy.get_param("~width", 848))
        self.height = int(rospy.get_param("~height", 480))
        self.fps = int(rospy.get_param("~fps", 30))

        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 10.0))
        self.disabled_publish_rate_hz = float(rospy.get_param("~disabled_publish_rate_hz", 1.0))

        self.show_debug = bool(rospy.get_param("~show_debug", False))

        # 使能瞬间拍照保存：测试视觉是否正常启动、画面里二维码长什么样
        self.save_enable_snapshot = bool(rospy.get_param("~save_enable_snapshot", True))
        default_snapshot_dir = os.path.join(os.path.expanduser("~"), "qr_vision_snapshots")
        self.snapshot_dir = rospy.get_param("~snapshot_dir", default_snapshot_dir)

        # 二维码有效编号范围
        self.min_id = int(rospy.get_param("~min_id", 1))
        self.max_id = int(rospy.get_param("~max_id", 24))

        self.vision_enabled = False
        self.frame_count = 0
        self.fail_count = 0

        # 只在视觉从关闭 -> 开启时保存一张，不会每帧狂存
        self.pending_enable_snapshot = False
        self.snapshot_count = 0

        self.last_publish_time = rospy.Time(0)
        self.last_disabled_publish_time = rospy.Time(0)

        self.detector = cv2.QRCodeDetector()

        self.init_snapshot_dir()

        self.result_pub = rospy.Publisher(
            self.qr_result_topic,
            String,
            queue_size=10
        )

        rospy.Subscriber(
            self.enable_topic,
            Bool,
            self.enable_callback,
            queue_size=5
        )

        # =========================
        # RealSense 初始化
        # =========================
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(
            rs.stream.color,
            self.width,
            self.height,
            rs.format.bgr8,
            self.fps
        )

        rospy.loginfo("Starting RealSense D435i color stream: %dx%d@%d",
                      self.width, self.height, self.fps)

        self.profile = self.pipeline.start(self.config)

        self.configure_camera()

        rospy.loginfo("qr_vision_node started.")
        rospy.loginfo("Subscribe: %s", self.enable_topic)
        rospy.loginfo("Publish:   %s", self.qr_result_topic)
        if self.save_enable_snapshot:
            rospy.loginfo("Enable snapshots will be saved to: %s", self.snapshot_dir)

    def configure_camera(self):
        """简单设置相机参数。失败不影响运行。"""
        try:
            color_sensor = self.profile.get_device().first_color_sensor()

            if color_sensor.supports(rs.option.enable_auto_exposure):
                color_sensor.set_option(rs.option.enable_auto_exposure, 1)

            if color_sensor.supports(rs.option.sharpness):
                color_sensor.set_option(rs.option.sharpness, 70)

            if color_sensor.supports(rs.option.contrast):
                color_sensor.set_option(rs.option.contrast, 55)

            if color_sensor.supports(rs.option.saturation):
                color_sensor.set_option(rs.option.saturation, 50)

        except Exception as e:
            rospy.logwarn("Camera option setup failed, ignored: %s", str(e))

    def enable_callback(self, msg):
        """FSM 控制视觉是否工作。

        当 /vision/enable 从 False 变 True 时，只标记一次拍照请求。
        真正保存照片放在主循环拿到 RealSense 当前帧之后执行，避免回调里阻塞。
        """
        new_state = bool(msg.data)

        if new_state and not self.vision_enabled:
            self.pending_enable_snapshot = True
            rospy.loginfo("Vision enabled, snapshot will be saved on next camera frame.")

        self.vision_enabled = new_state

    def init_snapshot_dir(self):
        """初始化使能拍照保存目录，并按已有图片数量继续编号。"""
        if not self.save_enable_snapshot:
            return

        try:
            if not os.path.exists(self.snapshot_dir):
                os.makedirs(self.snapshot_dir)

            max_index = 0
            for name in os.listdir(self.snapshot_dir):
                if not name.lower().endswith((".jpg", ".jpeg", ".png")):
                    continue
                base = os.path.splitext(name)[0]
                if base.isdigit():
                    max_index = max(max_index, int(base))

            self.snapshot_count = max_index

        except Exception as e:
            self.save_enable_snapshot = False
            rospy.logwarn("Snapshot directory init failed, snapshot disabled: %s", str(e))

    def save_current_frame_snapshot(self, frame):
        """保存当前画面。文件名按 1.jpg、2.jpg ... 顺序递增。"""
        if not self.save_enable_snapshot:
            return

        try:
            self.snapshot_count += 1
            filename = "{}.jpg".format(self.snapshot_count)
            path = os.path.join(self.snapshot_dir, filename)

            ok = cv2.imwrite(path, frame)
            if ok:
                rospy.loginfo("Enable snapshot saved: %s", path)
            else:
                rospy.logwarn("Enable snapshot save failed: %s", path)

        except Exception as e:
            rospy.logwarn("Enable snapshot save exception: %s", str(e))

    def make_result(self, found=False, qr_id=-1, u=-1.0, v=-1.0, area=0.0):
        """生成发给 FSM 的 JSON 字符串。"""
        result = {
            "found": bool(found),
            "id": int(qr_id),
            "u": float(u),
            "v": float(v),
            "area": float(area)
        }
        return json.dumps(result)

    def publish_false(self):
        """发布未识别到二维码。"""
        msg = self.make_result(
            found=False,
            qr_id=-1,
            u=-1.0,
            v=-1.0,
            area=0.0
        )
        self.result_pub.publish(String(data=msg))

    def extract_valid_id(self, text):
        """
        从二维码内容中提取 1~24 的编号。
        支持：
            "17"
            "ID:17"
            "goods=8"
        """
        if text is None:
            return -1

        text = str(text).strip()

        # 提取所有数字串
        nums = re.findall(r"\d+", text)

        if not nums:
            return -1

        # 优先取第一个落在 1~24 的数字
        for n in nums:
            try:
                value = int(n)
                if self.min_id <= value <= self.max_id:
                    return value
            except Exception:
                pass

        return -1

    def polygon_area(self, pts):
        """计算二维码四边形面积。"""
        if pts is None or len(pts) < 4:
            return 0.0

        pts = np.array(pts, dtype=np.float32).reshape(-1, 2)
        return float(abs(cv2.contourArea(pts)))

    def preprocess_fast(self, frame):
        """
        OpenCV 兜底增强。
        不做太多，避免卡顿。
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        clahe = cv2.createCLAHE(
            clipLimit=2.0,
            tileGridSize=(8, 8)
        )
        gray_clahe = clahe.apply(gray)

        blur = cv2.GaussianBlur(gray_clahe, (0, 0), 1.0)
        sharpen = cv2.addWeighted(gray_clahe, 1.5, blur, -0.5, 0)

        return [
            ("gray", gray),
            ("sharpen", sharpen),
        ]

    def decode_opencv(self, frame):
        """
        OpenCV 二维码识别。
        先原图，失败多次后才做轻量增强。
        """
        data, points, _ = self.detector.detectAndDecode(frame)

        if data and points is not None:
            return data, points, "opencv_raw"

        if self.fail_count < 2:
            return "", None, ""

        if self.frame_count % 3 != 0:
            return "", None, ""

        for scale in [1.0, 1.25]:
            if scale != 1.0:
                img = cv2.resize(
                    frame,
                    None,
                    fx=scale,
                    fy=scale,
                    interpolation=cv2.INTER_LINEAR
                )
            else:
                img = frame

            candidates = self.preprocess_fast(img)

            for name, candidate in candidates:
                data, points, _ = self.detector.detectAndDecode(candidate)

                if data and points is not None:
                    points = points / scale
                    return data, points, "opencv_{}_scale{}".format(name, scale)

        return "", None, ""

    def pyzbar_points_to_numpy(self, result):
        """把 pyzbar 的二维码角点转成 numpy 格式。"""
        polygon = result.polygon

        if polygon and len(polygon) >= 4:
            pts = np.array(
                [[p.x, p.y] for p in polygon],
                dtype=np.float32
            )
        else:
            x, y, w, h = result.rect
            pts = np.array(
                [
                    [x, y],
                    [x + w, y],
                    [x + w, y + h],
                    [x, y + h]
                ],
                dtype=np.float32
            )

        return pts.reshape(1, -1, 2)

    def decode_pyzbar(self, frame):
        """
        pyzbar 兜底识别。
        OpenCV 失败时使用，对倾斜、轻微模糊通常更稳。
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        candidates = []
        candidates.append(("pyzbar_gray", gray))

        if self.fail_count >= 2:
            clahe = cv2.createCLAHE(
                clipLimit=2.0,
                tileGridSize=(8, 8)
            )
            gray_clahe = clahe.apply(gray)
            candidates.append(("pyzbar_clahe", gray_clahe))

        if self.fail_count >= 4 and self.frame_count % 3 == 0:
            adaptive = cv2.adaptiveThreshold(
                gray,
                255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY,
                31,
                5
            )
            candidates.append(("pyzbar_adaptive", adaptive))

        for method, img in candidates:
            results = pyzbar.decode(img)

            if not results:
                continue

            # 多个二维码时，取画面面积最大的那个
            best = max(
                results,
                key=lambda r: r.rect.width * r.rect.height
            )

            try:
                data = best.data.decode("utf-8", errors="replace").strip()
            except Exception:
                data = str(best.data)

            if not data:
                continue

            points = self.pyzbar_points_to_numpy(best)

            return data, points, method

        return "", None, ""

    def decode_qr(self, frame):
        """
        总识别流程：
        1. OpenCV 优先
        2. pyzbar 兜底
        """
        data, points, method = self.decode_opencv(frame)

        if data:
            return data, points, method

        data, points, method = self.decode_pyzbar(frame)

        return data, points, method

    def draw_debug(self, frame, found, qr_id, u, v, area, method):
        """调试显示画面。比赛运行时建议关闭。"""
        if found:
            cv2.circle(
                frame,
                (int(u), int(v)),
                5,
                (0, 0, 255),
                -1
            )

            cv2.putText(
                frame,
                "ID: {}  Center: ({:.0f}, {:.0f})".format(qr_id, u, v),
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            cv2.putText(
                frame,
                "Area: {:.0f}  Method: {}".format(area, method),
                (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2
            )
        else:
            cv2.putText(
                frame,
                "QR: not found",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        cv2.imshow("qr_vision_node", frame)
        cv2.waitKey(1)

    def process_frame(self, frame):
        """
        处理一帧图像，返回：
        found, id, u, v, area, method
        """
        data, points, method = self.decode_qr(frame)

        if not data or points is None:
            self.fail_count += 1
            return False, -1, -1.0, -1.0, 0.0, ""

        qr_id = self.extract_valid_id(data)

        if qr_id < 0:
            self.fail_count += 1
            return False, -1, -1.0, -1.0, 0.0, method

        pts = points.astype(np.float32).reshape(-1, 2)

        u = float(np.mean(pts[:, 0]))
        v = float(np.mean(pts[:, 1]))
        area = self.polygon_area(pts)

        self.fail_count = 0

        return True, qr_id, u, v, area, method

    def run(self):
        rate = rospy.Rate(self.publish_rate_hz)

        try:
            while not rospy.is_shutdown():
                self.frame_count += 1

                # RealSense 保持取流，避免频繁 start/stop
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()

                if not color_frame:
                    rate.sleep()
                    continue

                frame = np.asanyarray(color_frame.get_data())

                # 如果刚收到视觉使能，上来就保存当前画面。
                # 注意：这是原始相机画面，不依赖二维码是否识别成功。
                if self.vision_enabled and self.pending_enable_snapshot:
                    self.save_current_frame_snapshot(frame)
                    self.pending_enable_snapshot = False

                now = rospy.Time.now()

                # 视觉未使能：不跑二维码识别，只低频发布 found=false
                if not self.vision_enabled:
                    if (
                        now - self.last_disabled_publish_time >
                        rospy.Duration(1.0 / self.disabled_publish_rate_hz)
                    ):
                        self.publish_false()
                        self.last_disabled_publish_time = now

                    if self.show_debug:
                        self.draw_debug(
                            frame,
                            False,
                            -1,
                            -1.0,
                            -1.0,
                            0.0,
                            "disabled"
                        )

                    rate.sleep()
                    continue

                # 视觉使能：正常识别
                found, qr_id, u, v, area, method = self.process_frame(frame)

                msg = self.make_result(
                    found=found,
                    qr_id=qr_id,
                    u=u,
                    v=v,
                    area=area
                )

                self.result_pub.publish(String(data=msg))

                if found:
                    rospy.loginfo_throttle(
                        0.5,
                        "QR found: id=%d u=%.1f v=%.1f area=%.1f method=%s",
                        qr_id,
                        u,
                        v,
                        area,
                        method
                    )
                else:
                    rospy.loginfo_throttle(
                        1.0,
                        "QR not found."
                    )

                if self.show_debug:
                    self.draw_debug(
                        frame,
                        found,
                        qr_id,
                        u,
                        v,
                        area,
                        method
                    )

                rate.sleep()

        finally:
            rospy.loginfo("Stopping RealSense pipeline...")
            self.pipeline.stop()

            if self.show_debug:
                cv2.destroyAllWindows()


if __name__ == "__main__":
    node = QRVisionNode()
    node.run()