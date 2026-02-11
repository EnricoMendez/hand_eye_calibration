# Author: Enrico Mendez
# Date: 23 Enero 2026
# Description: Detect aruco markers via stream (OpenCV 4.6 compatible)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data


class ArucoDetection(Node):
    def __init__(self):
        super().__init__('aruco_detection')
        self.get_logger().info('aruco_detection initialized')

        # State
        self.image = None
        self.bridge = CvBridge()

        # "Log once" flags
        self._got_first_image = False
        self._warned_no_image = False

        # ArUco (OpenCV 4.6 style)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        # Support both old and new OpenCV ArUco APIs.
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.parameters = cv2.aruco.DetectorParameters_create()
        else:
            self.parameters = cv2.aruco.DetectorParameters()
        self.detector = None
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # Pub/Sub
        self.pub_detection = self.create_publisher(Image, 'detection', 10)

        self.sub_rs_d405 = self.create_subscription(
            Image,
            '/camera/camera/color/image_rect_raw',
            self.rs_d405_callback,
            qos_profile_sensor_data
        )

        # Timer
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def rs_d405_callback(self, msg: Image):
        if not self._got_first_image:
            self.get_logger().info('First image received')
            self._got_first_image = True

        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def timer_callback(self):
        if self.image is None:
            if not self._warned_no_image:
                self.get_logger().info('No image received yet')
                self._warned_no_image = True
            return

        # reset "no image" warn flag once images start flowing
        self._warned_no_image = False

        frame = self.image.copy()
        corners, ids, rejected = self.detect(frame)

        if ids is not None and len(ids) > 0:
            marker_ids = ids.flatten().tolist()
            self.get_logger().info(
                "Detected marker(s): " + ", ".join(map(str, marker_ids)),
                throttle_duration_sec=2.0
            )
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        else:
            self.get_logger().info('No markers detected', throttle_duration_sec=2.0)

        msg_detection = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub_detection.publish(msg_detection)

    def detect(self, src):
        gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)

        if self.detector is not None:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray,
                self.aruco_dict,
                parameters=self.parameters
            )
        return corners, ids, rejected


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
