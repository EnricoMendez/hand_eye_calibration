"""Interactive eye-in-hand calibration node.

Quick usage:
- Run node:
  ros2 run hand_eye_calibration hand_eye_calibrator --ros-args \
    -p marker_topic:=/marker_poses -p min_samples:=15 \
    -p base_frame:=link_base -p ee_frame:=link_eef \
    -p camera_frame:=camera_color_optical_frame

Keyboard controls:
- s: capture sample
- c: compute calibration
- w: write calibration JSON
- r: reset captured samples
- q: quit node

Equivalent services:
- /capture_sample (std_srvs/Trigger)
- /compute_calibration (std_srvs/Trigger)
- /save_calibration (std_srvs/Trigger)
- /reset_samples (std_srvs/Trigger)
"""

import json
import os
import select
import sys
import termios
import threading
import tty
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_ros import (
    Buffer,
    StaticTransformBroadcaster,
    TransformException,
    TransformListener,
)
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray


def quaternion_to_rot(qx, qy, qz, qw):
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    n = np.linalg.norm(q)
    if n == 0.0:
        return np.eye(3, dtype=np.float64)
    qx, qy, qz, qw = q / n
    return np.array(
        [
            [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
            [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
            [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )


def rot_to_quaternion(rot):
    m = np.asarray(rot, dtype=np.float64)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        qw = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        qw = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        qw = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s
    return np.array([qx, qy, qz, qw], dtype=np.float64)


def invert_transform(rot, trans):
    rot_inv = rot.T
    trans_inv = -rot_inv @ trans.reshape(3)
    return rot_inv, trans_inv


def compose_transform(rot_ab, trans_ab, rot_bc, trans_bc):
    rot_ac = rot_ab @ rot_bc
    trans_ac = rot_ab @ trans_bc.reshape(3) + trans_ab.reshape(3)
    return rot_ac, trans_ac


def default_output_file():
    package_name = "hand_eye_calibration"
    filename = "hand_eye_calibration_result.json"

    # Source workspace candidate: <repo_root>/Config/<filename>
    source_cfg_dir = Path(__file__).resolve().parents[1] / "Config"
    if source_cfg_dir.is_dir() and os.access(source_cfg_dir, os.W_OK):
        return str(source_cfg_dir / filename)

    # Installed package candidate: <share>/<package>/Config/<filename>
    try:
        share_cfg_dir = Path(get_package_share_directory(package_name)) / "Config"
        if share_cfg_dir.is_dir() and os.access(share_cfg_dir, os.W_OK):
            return str(share_cfg_dir / filename)
    except Exception:
        pass

    # Fallback for read-only installs.
    return str(Path("/tmp") / filename)


class HandEyeCalibrator(Node):
    def __init__(self):
        super().__init__("hand_eye_calibrator")

        self.declare_parameter("marker_topic", "/marker_poses")
        self.declare_parameter("base_frame", "link_base")
        self.declare_parameter("ee_frame", "link_eef")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("result_child_frame", "camera_color_optical_frame_calibrated")
        self.declare_parameter("min_samples", 15)
        self.declare_parameter("marker_pose_index", 0)
        self.declare_parameter("solver_method", "TSAI")
        self.declare_parameter("output_file", default_output_file())
        self.declare_parameter("viz_topic", "/hand_eye_calibration/axes")
        self.declare_parameter("axis_length", 0.08)
        self.declare_parameter("camera_estimated_frame", "camera_estimated")
        self.declare_parameter("marker_estimated_frame", "marker_estimated")

        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.ee_frame = str(self.get_parameter("ee_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.result_child_frame = str(self.get_parameter("result_child_frame").value)
        self.min_samples = int(self.get_parameter("min_samples").value)
        self.marker_pose_index = int(self.get_parameter("marker_pose_index").value)
        self.solver_method = str(self.get_parameter("solver_method").value).upper()
        self.output_file = str(self.get_parameter("output_file").value)
        self.viz_topic = str(self.get_parameter("viz_topic").value)
        self.axis_length = float(self.get_parameter("axis_length").value)
        self.camera_estimated_frame = str(self.get_parameter("camera_estimated_frame").value)
        self.marker_estimated_frame = str(self.get_parameter("marker_estimated_frame").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.pub_tf = self.create_publisher(TFMessage, "/tf", 10)

        self.latest_marker_pose = None
        self.latest_marker_frame = None

        self.R_gripper2base = []
        self.t_gripper2base = []
        self.R_target2cam = []
        self.t_target2cam = []

        self.last_result = None
        self.calib_rot_g2c = None
        self.calib_trans_g2c = None
        self._auto_compute_done = False

        self._pending_capture = False
        self._pending_compute = False
        self._pending_save = False
        self._pending_reset = False
        self._pending_shutdown = False
        self._key_lock = threading.Lock()
        self._keyboard_stop_event = threading.Event()
        self._keyboard_tty_file = None

        self.sub_marker = self.create_subscription(
            PoseArray, self.marker_topic, self.marker_callback, 10
        )
        self.pub_axes = self.create_publisher(MarkerArray, self.viz_topic, 10)

        # Keep service mode available too.
        self.srv_capture = self.create_service(Trigger, "capture_sample", self.capture_sample_cb)
        self.srv_compute = self.create_service(Trigger, "compute_calibration", self.compute_cb)
        self.srv_save = self.create_service(Trigger, "save_calibration", self.save_cb)
        self.srv_reset = self.create_service(Trigger, "reset_samples", self.reset_cb)

        self.status_timer = self.create_timer(1.0, self.status_timer_cb)
        self.command_timer = self.create_timer(0.1, self.command_timer_cb)
        self.viz_timer = self.create_timer(0.2, self.publish_axes_cb)

        self.get_logger().info(
            f"hand_eye_calibrator ready | marker_topic={self.marker_topic}, "
            f"base_frame={self.base_frame}, ee_frame={self.ee_frame}, camera_frame={self.camera_frame}"
        )
        self.get_logger().info(
            "Keyboard controls: [s]=capture, [c]=compute, [w]=save, [r]=reset, [q]=quit"
        )
        self._start_keyboard_thread()

    def marker_callback(self, msg: PoseArray):
        if not msg.poses:
            return
        idx = self.marker_pose_index
        if idx < 0 or idx >= len(msg.poses):
            idx = 0
        self.latest_marker_pose = msg.poses[idx]
        self.latest_marker_frame = msg.header.frame_id

    def _method_constant(self):
        methods = {
            "TSAI": cv2.CALIB_HAND_EYE_TSAI,
            "PARK": cv2.CALIB_HAND_EYE_PARK,
            "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
            "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
            "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
        }
        return methods.get(self.solver_method, cv2.CALIB_HAND_EYE_TSAI)

    def _pose_to_rt(self, pose: Pose):
        rot = quaternion_to_rot(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )
        trans = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=np.float64)
        return rot, trans

    def _tf_to_rt(self, transform: TransformStamped):
        t = transform.transform.translation
        q = transform.transform.rotation
        rot = quaternion_to_rot(q.x, q.y, q.z, q.w)
        trans = np.array([t.x, t.y, t.z], dtype=np.float64)
        return rot, trans

    def _get_ee_transform(self):
        tf_msg = self.tf_buffer.lookup_transform(
            self.base_frame, self.ee_frame, rclpy.time.Time()
        )
        return self._tf_to_rt(tf_msg)

    def _capture_sample(self):
        if self.latest_marker_pose is None:
            return False, "No marker pose received yet on marker_topic."

        if self.latest_marker_frame and self.latest_marker_frame != self.camera_frame:
            self.get_logger().warn(
                f"Marker frame is '{self.latest_marker_frame}', expected '{self.camera_frame}'. "
                "Continuing with incoming frame."
            )

        try:
            rot_g2b, trans_g2b = self._get_ee_transform()
        except TransformException as exc:
            return False, f"TF lookup failed ({self.base_frame} <- {self.ee_frame}): {exc}"

        rot_t2c, trans_t2c = self._pose_to_rt(self.latest_marker_pose)

        self.R_gripper2base.append(rot_g2b)
        self.t_gripper2base.append(trans_g2b.reshape(3, 1))
        self.R_target2cam.append(rot_t2c)
        self.t_target2cam.append(trans_t2c.reshape(3, 1))

        count = len(self.R_gripper2base)
        self._auto_compute_done = False
        return True, f"Sample captured. Total samples: {count}/{self.min_samples}"

    def _compute_calibration(self):
        n = len(self.R_gripper2base)
        if n < self.min_samples:
            return False, f"Not enough samples: {n}/{self.min_samples}"

        try:
            # OpenCV returns camera->gripper ({}^gT_c), i.e. ee_frame -> camera_frame.
            rot_cam2gripper, trans_cam2gripper = cv2.calibrateHandEye(
                self.R_gripper2base,
                self.t_gripper2base,
                self.R_target2cam,
                self.t_target2cam,
                method=self._method_constant(),
            )
        except Exception as exc:
            return False, f"calibrateHandEye failed: {exc}"

        trans_cam2gripper = np.asarray(trans_cam2gripper, dtype=np.float64).reshape(3)
        quat_cam2gripper = rot_to_quaternion(rot_cam2gripper)

        rot_gripper2cam, trans_gripper2cam = invert_transform(rot_cam2gripper, trans_cam2gripper)
        quat_gripper2cam = rot_to_quaternion(rot_gripper2cam)

        # Used later as base->ee compose ee->camera = base->camera.
        self.calib_rot_g2c = rot_cam2gripper
        self.calib_trans_g2c = trans_cam2gripper

        self.last_result = {
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "samples": n,
            "method": self.solver_method,
            "camera_to_gripper": {
                "rotation_matrix": rot_cam2gripper.tolist(),
                "translation_xyz": trans_cam2gripper.tolist(),
            },
            "gripper_to_camera": {
                "rotation_matrix": rot_gripper2cam.tolist(),
                "translation_xyz": trans_gripper2cam.tolist(),
                "quaternion_xyzw": quat_gripper2cam.tolist(),
            },
            "frames": {
                "base_frame": self.base_frame,
                "ee_frame": self.ee_frame,
                "camera_frame_input": self.camera_frame,
                "result_child_frame": self.result_child_frame,
            },
        }

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.ee_frame
        tf_msg.child_frame_id = self.result_child_frame
        tf_msg.transform.translation.x = float(trans_cam2gripper[0])
        tf_msg.transform.translation.y = float(trans_cam2gripper[1])
        tf_msg.transform.translation.z = float(trans_cam2gripper[2])
        tf_msg.transform.rotation.x = float(quat_cam2gripper[0])
        tf_msg.transform.rotation.y = float(quat_cam2gripper[1])
        tf_msg.transform.rotation.z = float(quat_cam2gripper[2])
        tf_msg.transform.rotation.w = float(quat_cam2gripper[3])
        self.tf_static_broadcaster.sendTransform(tf_msg)

        self._auto_compute_done = True
        return (
            True,
            f"Calibration computed with {n} samples. Published static TF: "
            f"{self.ee_frame} -> {self.result_child_frame}",
        )

    def _save_result(self):
        if self.last_result is None:
            return False, "No calibration result available. Run compute_calibration first."
        try:
            with open(self.output_file, "w", encoding="utf-8") as f:
                json.dump(self.last_result, f, indent=2)
        except Exception as exc:
            return False, f"Failed to save file: {exc}"
        return True, f"Calibration saved to {self.output_file}"

    def _reset_samples(self):
        self.R_gripper2base.clear()
        self.t_gripper2base.clear()
        self.R_target2cam.clear()
        self.t_target2cam.clear()
        self.last_result = None
        self.calib_rot_g2c = None
        self.calib_trans_g2c = None
        self._auto_compute_done = False
        return True, "Samples and last result cleared."

    def _build_axis_markers(self, ns, marker_id_start, rot, trans, axis_len):
        axis_defs = [
            ((1.0, 0.0, 0.0), rot[:, 0], "x"),
            ((0.0, 1.0, 0.0), rot[:, 1], "y"),
            ((0.0, 0.0, 1.0), rot[:, 2], "z"),
        ]
        markers = []
        now = self.get_clock().now().to_msg()

        for i, (color_rgb, axis_dir, _axis_name) in enumerate(axis_defs):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = self.base_frame
            m.ns = ns
            m.id = marker_id_start + i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = 0.008
            m.scale.y = 0.014
            m.scale.z = 0.02
            m.color.a = 1.0
            m.color.r = float(color_rgb[0])
            m.color.g = float(color_rgb[1])
            m.color.b = float(color_rgb[2])
            m.points = []

            p0 = Pose().position
            p0.x = float(trans[0])
            p0.y = float(trans[1])
            p0.z = float(trans[2])
            m.points.append(p0)

            p1 = Pose().position
            p1.x = float(trans[0] + axis_len * axis_dir[0])
            p1.y = float(trans[1] + axis_len * axis_dir[1])
            p1.z = float(trans[2] + axis_len * axis_dir[2])
            m.points.append(p1)
            markers.append(m)

        return markers

    def _build_delete_markers(self, ns, marker_id_start):
        markers = []
        now = self.get_clock().now().to_msg()
        for i in range(3):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = self.base_frame
            m.ns = ns
            m.id = marker_id_start + i
            m.action = Marker.DELETE
            markers.append(m)
        return markers

    def _make_tf(self, parent_frame, child_frame, rot, trans):
        quat = rot_to_quaternion(rot)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = parent_frame
        tf_msg.child_frame_id = child_frame
        tf_msg.transform.translation.x = float(trans[0])
        tf_msg.transform.translation.y = float(trans[1])
        tf_msg.transform.translation.z = float(trans[2])
        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])
        return tf_msg

    def publish_axes_cb(self):
        try:
            rot_b2e, trans_b2e = self._get_ee_transform()
        except TransformException:
            return

        marker_array = MarkerArray()
        tf_list = []
        marker_array.markers.extend(
            self._build_axis_markers("end_effector", 0, rot_b2e, trans_b2e, self.axis_length)
        )

        if self.calib_rot_g2c is not None and self.calib_trans_g2c is not None:
            rot_b2c, trans_b2c = compose_transform(
                rot_b2e, trans_b2e, self.calib_rot_g2c, self.calib_trans_g2c
            )
            tf_list.append(self._make_tf(self.base_frame, self.camera_estimated_frame, rot_b2c, trans_b2c))
            marker_array.markers.extend(
                self._build_axis_markers("camera_estimated", 100, rot_b2c, trans_b2c, self.axis_length)
            )

            if self.latest_marker_pose is not None:
                rot_c2m, trans_c2m = self._pose_to_rt(self.latest_marker_pose)
                rot_b2m, trans_b2m = compose_transform(rot_b2c, trans_b2c, rot_c2m, trans_c2m)
                tf_list.append(self._make_tf(self.base_frame, self.marker_estimated_frame, rot_b2m, trans_b2m))
                marker_array.markers.extend(
                    self._build_axis_markers("marker_estimated", 200, rot_b2m, trans_b2m, self.axis_length)
                )
            else:
                marker_array.markers.extend(self._build_delete_markers("marker_estimated", 200))
        else:
            marker_array.markers.extend(self._build_delete_markers("camera_estimated", 100))
            marker_array.markers.extend(self._build_delete_markers("marker_estimated", 200))

        if tf_list:
            self.pub_tf.publish(TFMessage(transforms=tf_list))
        self.pub_axes.publish(marker_array)

    def capture_sample_cb(self, _req, res):
        res.success, res.message = self._capture_sample()
        return res

    def compute_cb(self, _req, res):
        res.success, res.message = self._compute_calibration()
        return res

    def save_cb(self, _req, res):
        res.success, res.message = self._save_result()
        return res

    def reset_cb(self, _req, res):
        res.success, res.message = self._reset_samples()
        return res

    def status_timer_cb(self):
        n = len(self.R_gripper2base)
        ee_text = "EE: N/A"
        marker_text = "Marker: N/A"

        try:
            _, ee_t = self._get_ee_transform()
            ee_text = f"EE xyz=({ee_t[0]:+.3f}, {ee_t[1]:+.3f}, {ee_t[2]:+.3f})"
        except TransformException:
            ee_text = f"EE: TF {self.base_frame}->{self.ee_frame} not available"

        if self.latest_marker_pose is not None:
            p = self.latest_marker_pose.position
            marker_text = f"Marker xyz=({p.x:+.3f}, {p.y:+.3f}, {p.z:+.3f})"

        self.get_logger().info(
            f"{ee_text} | {marker_text} | samples={n}/{self.min_samples}",
            throttle_duration_sec=1.0,
        )

        if n >= self.min_samples and not self._auto_compute_done:
            ok, msg = self._compute_calibration()
            if ok:
                self.get_logger().info(f"Auto compute: {msg}")
            else:
                self.get_logger().error(f"Auto compute failed: {msg}")

    def command_timer_cb(self):
        with self._key_lock:
            do_capture = self._pending_capture
            do_compute = self._pending_compute
            do_save = self._pending_save
            do_reset = self._pending_reset
            do_shutdown = self._pending_shutdown
            self._pending_capture = False
            self._pending_compute = False
            self._pending_save = False
            self._pending_reset = False
            self._pending_shutdown = False

        if do_capture:
            ok, msg = self._capture_sample()
            if ok:
                self.get_logger().info(msg)
            else:
                self.get_logger().warn(msg)

        if do_compute:
            ok, msg = self._compute_calibration()
            if ok:
                self.get_logger().info(msg)
            else:
                self.get_logger().warn(msg)

        if do_save:
            ok, msg = self._save_result()
            if ok:
                self.get_logger().info(msg)
            else:
                self.get_logger().warn(msg)

        if do_reset:
            ok, msg = self._reset_samples()
            if ok:
                self.get_logger().info(msg)
            else:
                self.get_logger().warn(msg)

        if do_shutdown:
            self.get_logger().info("Keyboard quit requested. Shutting down...")
            rclpy.shutdown()

    def _start_keyboard_thread(self):
        keyboard_fd = None
        close_on_exit = False
        source = "stdin"

        if sys.stdin.isatty():
            keyboard_fd = sys.stdin.fileno()
            source = "stdin"
        else:
            try:
                self._keyboard_tty_file = open("/dev/tty", "rb", buffering=0)
                keyboard_fd = self._keyboard_tty_file.fileno()
                close_on_exit = True
                source = "/dev/tty"
            except OSError as exc:
                self.get_logger().warn(
                    f"Keyboard capture disabled (no TTY available): {exc}"
                )
                return

        self.get_logger().info(f"Keyboard input enabled from {source}.")
        thread = threading.Thread(
            target=self._keyboard_loop, args=(keyboard_fd, close_on_exit), daemon=True
        )
        thread.start()

    def _keyboard_loop(self, fd, close_on_exit):
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while rclpy.ok() and not self._keyboard_stop_event.is_set():
                readable, _, _ = select.select([fd], [], [], 0.1)
                if not readable:
                    continue
                key_bytes = os.read(fd, 1)
                if not key_bytes:
                    continue
                key = key_bytes.decode(errors="ignore").lower()
                with self._key_lock:
                    if key == "s":
                        self._pending_capture = True
                    elif key == "c":
                        self._pending_compute = True
                    elif key == "w":
                        self._pending_save = True
                    elif key == "r":
                        self._pending_reset = True
                    elif key == "q":
                        self._pending_shutdown = True
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            if close_on_exit and self._keyboard_tty_file is not None:
                self._keyboard_tty_file.close()
                self._keyboard_tty_file = None

    def destroy_node(self):
        self._keyboard_stop_event.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCalibrator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
