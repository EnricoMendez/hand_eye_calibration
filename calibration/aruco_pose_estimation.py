# Author: Enrico Mendez
# Date: 23 Enero 2026
# Description: Estimate ArUco marker poses via stream (OpenCV 4.6 compatible)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray
import cv2
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
import numpy as np


class ArucoPoseEstimation(Node):
    def __init__(self):
        super().__init__('aruco_pose_estimation')
        self.get_logger().info('aruco_pose_estimation initialized')

        # State
        self.image = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.bridge = CvBridge()

        # "Log once" flags
        self._got_first_image = False
        self._got_camera_info = False
        self._warned_no_image = False
        self._warned_no_camera_info = False

        # Params
        self.declare_parameter('marker_length', 0.075)  # meters
        self.declare_parameter('image_topic', '/camera/camera/color/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        marker_length = self.get_parameter('marker_length').value
        self.marker_length = float(marker_length)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)

        # ArUco (OpenCV 4.6 style)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        # In 4.6 this is the most compatible constructor:
        self.parameters = cv2.aruco.DetectorParameters_create()

        # Pub/Sub
        self.pub_detection = self.create_publisher(Image, 'detection', 10)
        self.pub_marker_poses = self.create_publisher(PoseArray, 'marker_poses', 10)

        self.sub_rs_d405 = self.create_subscription(
            Image,
            self.image_topic,
            self.rs_d405_callback,
            qos_profile_sensor_data
        )
        self.sub_camera_info = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
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

    def camera_info_callback(self, msg: CameraInfo):
        if not self._got_camera_info:
            self.get_logger().info('First camera info received')
            self._got_camera_info = True

        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    def timer_callback(self):
        if self.image is None:
            if not self._warned_no_image:
                self.get_logger().info('No image received yet')
                self._warned_no_image = True
            return

        if self.camera_matrix is None or self.dist_coeffs is None:
            if not self._warned_no_camera_info:
                self.get_logger().info('No camera info received yet')
                self._warned_no_camera_info = True
            return

        # reset "no image" warn flag once images start flowing
        self._warned_no_image = False
        self._warned_no_camera_info = False

        frame = self.image.copy()
        corners, ids, rejected = self.detect(frame)

        pose_array = PoseArray()
        pose_array.header.frame_id = 'camera_color_optical_frame'
        pose_array.header.stamp = self.get_clock().now().to_msg()

        if ids is not None and len(ids) > 0:
            marker_ids = ids.flatten().tolist()
            id0_indices = [i for i, marker_id in enumerate(marker_ids) if marker_id == 0]

            if len(id0_indices) > 0:
                id0_corners = [corners[i] for i in id0_indices]
                id0_ids = np.array([[0] for _ in id0_indices], dtype=ids.dtype)
                cv2.aruco.drawDetectedMarkers(frame, id0_corners, id0_ids)

                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    id0_corners,
                    self.marker_length,
                    self.camera_matrix,
                    self.dist_coeffs
                )

                for i in range(len(id0_indices)):
                    rvec = rvecs[i][0]
                    tvec = tvecs[i][0]
                    pose = self.build_pose(rvec, tvec)
                    pose_array.poses.append(pose)

                    cv2.drawFrameAxes(
                        frame,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvec,
                        tvec,
                        self.marker_length * 0.5
                    )

                    self.get_logger().info(
                        f"Marker 0: x={tvec[0]:.3f}, y={tvec[1]:.3f}, z={tvec[2]:.3f} m",
                        throttle_duration_sec=1.0
                    )
            else:
                self.get_logger().info('No marker with ID 0 detected', throttle_duration_sec=2.0)
        else:
            self.get_logger().info('No markers detected', throttle_duration_sec=2.0)

        self.pub_marker_poses.publish(pose_array)
        msg_detection = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub_detection.publish(msg_detection)

    def build_pose(self, rvec, tvec):
        pose = Pose()
        pose.position.x = float(tvec[0])
        pose.position.y = float(tvec[1])
        pose.position.z = float(tvec[2])

        # Convert Rodrigues vector to quaternion.
        rot_mat, _ = cv2.Rodrigues(rvec)
        qw = np.sqrt(max(0.0, 1.0 + rot_mat[0, 0] + rot_mat[1, 1] + rot_mat[2, 2])) / 2.0
        qx = np.sqrt(max(0.0, 1.0 + rot_mat[0, 0] - rot_mat[1, 1] - rot_mat[2, 2])) / 2.0
        qy = np.sqrt(max(0.0, 1.0 - rot_mat[0, 0] + rot_mat[1, 1] - rot_mat[2, 2])) / 2.0
        qz = np.sqrt(max(0.0, 1.0 - rot_mat[0, 0] - rot_mat[1, 1] + rot_mat[2, 2])) / 2.0
        qx = np.copysign(qx, rot_mat[2, 1] - rot_mat[1, 2])
        qy = np.copysign(qy, rot_mat[0, 2] - rot_mat[2, 0])
        qz = np.copysign(qz, rot_mat[1, 0] - rot_mat[0, 1])

        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        pose.orientation.w = float(qw)
        return pose

    def detect(self, src):
        gray = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)

        # OpenCV 4.6 API (no ArucoDetector)
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.parameters
        )
        return corners, ids, rejected


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseEstimation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
