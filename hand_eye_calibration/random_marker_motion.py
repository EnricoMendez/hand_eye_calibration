import random
import threading
import time

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from xarm_msgs.srv import GetFloat32List, MoveCartesian, SetInt16, SetInt16ById


class RandomMarkerMotion(Node):
    def __init__(self):
        super().__init__("random_marker_motion")

        self.declare_parameter("hw_ns", "ufactory")
        self.declare_parameter("marker_topic", "/marker_poses")
        self.declare_parameter("marker_stale_sec", 1.0)
        self.declare_parameter("max_moves", 50)
        self.declare_parameter("dwell_sec", 1.0)
        self.declare_parameter("speed", 80.0)
        self.declare_parameter("acc", 400.0)
        self.declare_parameter("move_timeout_sec", 12.0)
        self.declare_parameter("use_home_center", True)
        self.declare_parameter("range_x", 0.04)
        self.declare_parameter("range_y", 0.04)
        self.declare_parameter("range_z", 0.03)
        self.declare_parameter("range_roll", 0.20)
        self.declare_parameter("range_pitch", 0.20)
        self.declare_parameter("range_yaw", 0.35)
        self.declare_parameter("auto_enable_motion", True)
        self.declare_parameter("motion_enable_id", 8)
        self.declare_parameter("random_seed", -1)

        self.hw_ns = str(self.get_parameter("hw_ns").value).strip("/")
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.marker_stale_sec = float(self.get_parameter("marker_stale_sec").value)
        self.max_moves = int(self.get_parameter("max_moves").value)
        self.dwell_sec = float(self.get_parameter("dwell_sec").value)
        self.speed = float(self.get_parameter("speed").value)
        self.acc = float(self.get_parameter("acc").value)
        self.move_timeout_sec = float(self.get_parameter("move_timeout_sec").value)
        self.use_home_center = bool(self.get_parameter("use_home_center").value)
        self.range_x = float(self.get_parameter("range_x").value)
        self.range_y = float(self.get_parameter("range_y").value)
        self.range_z = float(self.get_parameter("range_z").value)
        self.range_roll = float(self.get_parameter("range_roll").value)
        self.range_pitch = float(self.get_parameter("range_pitch").value)
        self.range_yaw = float(self.get_parameter("range_yaw").value)
        self.auto_enable_motion = bool(self.get_parameter("auto_enable_motion").value)
        self.motion_enable_id = int(self.get_parameter("motion_enable_id").value)
        self.random_seed = int(self.get_parameter("random_seed").value)

        if self.random_seed >= 0:
            random.seed(self.random_seed)

        self.last_marker_time = 0.0
        self.move_count = 0
        self.home_pose = None
        self.stop_event = threading.Event()

        self.sub_marker = self.create_subscription(
            PoseArray, self.marker_topic, self.marker_callback, 10
        )

        self.client_get_position = self.create_client(
            GetFloat32List, self._svc_name("get_position")
        )
        self.client_set_position = self.create_client(
            MoveCartesian, self._svc_name("set_position")
        )
        self.client_set_mode = self.create_client(SetInt16, self._svc_name("set_mode"))
        self.client_set_state = self.create_client(SetInt16, self._svc_name("set_state"))
        self.client_motion_enable = self.create_client(
            SetInt16ById, self._svc_name("motion_enable")
        )

        self.worker = threading.Thread(target=self.worker_loop, daemon=True)
        self.worker.start()

        self.get_logger().info(
            f"random_marker_motion ready | ns=/{self.hw_ns}, marker_topic={self.marker_topic}"
        )

    def _svc_name(self, name):
        return f"/{self.hw_ns}/{name}" if self.hw_ns else f"/{name}"

    def marker_callback(self, msg: PoseArray):
        if msg.poses:
            self.last_marker_time = time.monotonic()

    def marker_visible(self):
        return (time.monotonic() - self.last_marker_time) <= self.marker_stale_sec

    def wait_service(self, client, name):
        while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Waiting service: {name}")
        return rclpy.ok()

    def call_with_timeout(self, client, request, timeout_sec):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                try:
                    return future.result()
                except Exception:
                    return None
            time.sleep(0.01)
        return None

    def get_current_pose(self):
        req = GetFloat32List.Request()
        res = self.call_with_timeout(
            self.client_get_position, req, timeout_sec=self.move_timeout_sec
        )
        if res is None or res.ret != 0 or len(res.datas) < 6:
            return None
        return list(res.datas[:6])

    def set_position(self, pose):
        req = MoveCartesian.Request()
        req.pose = [float(v) for v in pose]
        req.speed = float(self.speed)
        req.acc = float(self.acc)
        req.mvtime = 0.0
        req.wait = True
        req.timeout = float(self.move_timeout_sec)
        req.radius = -1.0
        req.motion_type = 0
        req.is_tool_coord = False
        req.relative = False
        res = self.call_with_timeout(
            self.client_set_position, req, timeout_sec=self.move_timeout_sec + 2.0
        )
        return res is not None and res.ret == 0

    def init_motion(self):
        if not self.auto_enable_motion:
            return True

        req_enable = SetInt16ById.Request()
        req_enable.id = int(self.motion_enable_id)
        req_enable.data = 1
        res = self.call_with_timeout(
            self.client_motion_enable, req_enable, timeout_sec=3.0
        )
        if res is None or res.ret != 0:
            self.get_logger().warn("motion_enable failed")

        req_mode = SetInt16.Request()
        req_mode.data = 0
        res = self.call_with_timeout(self.client_set_mode, req_mode, timeout_sec=3.0)
        if res is None or res.ret != 0:
            self.get_logger().warn("set_mode failed")

        req_state = SetInt16.Request()
        req_state.data = 0
        res = self.call_with_timeout(self.client_set_state, req_state, timeout_sec=3.0)
        if res is None or res.ret != 0:
            self.get_logger().warn("set_state failed")

        return True

    def sample_target(self, center):
        x, y, z, roll, pitch, yaw = center
        return [
            x + random.uniform(-self.range_x, self.range_x),
            y + random.uniform(-self.range_y, self.range_y),
            z + random.uniform(-self.range_z, self.range_z),
            roll + random.uniform(-self.range_roll, self.range_roll),
            pitch + random.uniform(-self.range_pitch, self.range_pitch),
            yaw + random.uniform(-self.range_yaw, self.range_yaw),
        ]

    def worker_loop(self):
        svc_list = [
            (self.client_get_position, "get_position"),
            (self.client_set_position, "set_position"),
            (self.client_set_mode, "set_mode"),
            (self.client_set_state, "set_state"),
            (self.client_motion_enable, "motion_enable"),
        ]
        for client, name in svc_list:
            if not self.wait_service(client, name):
                return

        self.init_motion()

        while rclpy.ok() and not self.stop_event.is_set():
            if self.move_count >= self.max_moves:
                self.get_logger().info(f"Finished random motion ({self.move_count} moves).")
                return

            if not self.marker_visible():
                self.get_logger().info(
                    "Marker not visible, waiting...",
                    throttle_duration_sec=2.0,
                )
                time.sleep(0.2)
                continue

            current_pose = self.get_current_pose()
            if current_pose is None:
                self.get_logger().warn("Could not read current robot pose.")
                time.sleep(0.3)
                continue

            if self.home_pose is None:
                self.home_pose = current_pose
                self.get_logger().info(
                    "Home pose captured as random center."
                    if self.use_home_center
                    else "Using current pose as random center each move."
                )

            center = self.home_pose if self.use_home_center else current_pose
            target = self.sample_target(center)

            ok = self.set_position(target)
            if ok:
                self.move_count += 1
                self.get_logger().info(
                    f"Move {self.move_count}/{self.max_moves} -> "
                    f"x={target[0]:+.3f}, y={target[1]:+.3f}, z={target[2]:+.3f}"
                )
            else:
                self.get_logger().warn("set_position failed, retrying...")
            time.sleep(self.dwell_sec)

    def destroy_node(self):
        self.stop_event.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RandomMarkerMotion()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
