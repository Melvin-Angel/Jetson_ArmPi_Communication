"""ROS 2 client adapter for the ArmPi Pro TCP bridge."""
import math
import os
import queue
import socket
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

# Deployment keeps protocol/ adjacent to the two workspaces.  It can also be
# installed as a Python package; PYTHONPATH is intentionally sufficient.
PROTOCOL_DIR = os.environ.get("ARMPI_PROTOCOL_PATH", "")
if PROTOCOL_DIR and PROTOCOL_DIR not in sys.path:
    sys.path.insert(0, PROTOCOL_DIR)
from armpi_protocol import JsonLineSocket, ProtocolError, make_message


def quaternion_to_euler(quaternion):
    """Return roll, pitch, yaw in radians without scipy."""
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return roll, pitch, math.atan2(siny_cosp, cosy_cosp)


class JetsonToArmPi(Node):
    def __init__(self):
        super().__init__("jetson_to_armpi")
        self.declare_parameter("pi_host", "192.168.50.20")
        self.declare_parameter("port", 8765)
        self.declare_parameter("heartbeat_s", 0.2)
        self.declare_parameter("arm_motion_duration_s", 1.5)
        self.pi_host = self.get_parameter("pi_host").value
        self.port = int(self.get_parameter("port").value)
        self.outbound = queue.Queue(maxsize=100)
        self.inbound = queue.Queue(maxsize=100)
        self.running = True
        self.worker = threading.Thread(target=self._network_loop, daemon=True)
        self.worker.start()

        self.create_subscription(Twist, "/armpi/cmd_vel", self._on_twist, 10)
        self.create_subscription(PoseStamped, "/armpi/arm_pose_cmd", self._on_pose, 10)
        self.create_subscription(String, "/armpi/gripper_cmd", self._on_gripper, 10)
        self.create_subscription(Empty, "/armpi/stop", self._on_stop, 10)
        self.state_pub = self.create_publisher(String, "/armpi/state", 10)
        self.status_pub = self.create_publisher(String, "/armpi/status", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/armpi/pose_estimated", 10)
        self.create_service(Trigger, "/armpi/get_state", self._on_get_state)
        self.create_timer(float(self.get_parameter("heartbeat_s").value), self._heartbeat)
        self.create_timer(0.02, self._drain_inbound)
        self.get_logger().info("Jetson bridge configured for %s:%d" % (self.pi_host, self.port))

    def _queue(self, kind, payload=None):
        try:
            self.outbound.put_nowait(make_message(kind, payload))
        except queue.Full:
            self.get_logger().error("outbound TCP queue full; dropping command")

    def _on_twist(self, msg):
        self._queue("base_velocity", {"vx_mps": msg.linear.x, "vy_mps": msg.linear.y,
                                      "wz_radps": msg.angular.z})

    def _on_pose(self, msg):
        roll, pitch, yaw = quaternion_to_euler(msg.pose.orientation)
        if abs(roll) > 0.02 or abs(yaw) > 0.02:
            self.get_logger().warn("ArmPi Pro IK is pitch-only; requested roll/yaw are not executed")
        self._queue("arm_pose", {"position_m": [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
                                 "pitch_deg": math.degrees(pitch),
                                 "duration_s": float(self.get_parameter("arm_motion_duration_s").value)})

    def _on_gripper(self, msg):
        command = msg.data.strip().lower()
        if command not in ("open", "close"):
            self.get_logger().warn("gripper command must be 'open' or 'close'")
            return
        self._queue("gripper", {"command": command, "duration_s": 0.5})

    def _on_stop(self, _msg):
        self._queue("stop")

    def _on_get_state(self, _request, response):
        self._queue("get_state")
        response.success = True
        response.message = "State requested; response will be published on /armpi/state"
        return response

    def _heartbeat(self):
        self._queue("heartbeat")

    def _network_loop(self):
        while self.running:
            sock = None
            try:
                sock = socket.create_connection((self.pi_host, self.port), timeout=2.0)
                sock.settimeout(0.05)
                channel = JsonLineSocket(sock)
                self._put_inbound({"type": "connection", "payload": {"connected": True}})
                while self.running:
                    try:
                        while True:
                            channel.send(self.outbound.get_nowait())
                    except queue.Empty:
                        pass
                    try:
                        for message in channel.recv_messages():
                            self._put_inbound(message)
                    except socket.timeout:
                        pass
            except (OSError, ConnectionError, ProtocolError) as exc:
                self._put_inbound({"type": "connection", "payload": {"connected": False, "reason": str(exc)}})
                time.sleep(1.0)
            finally:
                if sock is not None:
                    sock.close()

    def _put_inbound(self, message):
        try:
            self.inbound.put_nowait(message)
        except queue.Full:
            pass

    def _drain_inbound(self):
        while True:
            try:
                message = self.inbound.get_nowait()
            except queue.Empty:
                return
            kind, payload = message.get("type"), message.get("payload", {})
            if kind == "connection":
                self.status_pub.publish(String(data=str(payload)))
                continue
            self.state_pub.publish(String(data=str(message)))
            if kind in ("state", "telemetry") and payload.get("pose"):
                pose = payload["pose"]
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "arm_base"
                pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = pose["position_m"]
                # This is a commanded pitch-only estimate, not forward kinematics.
                half_pitch = math.radians(pose["pitch_deg"]) / 2.0
                pose_msg.pose.orientation.y = math.sin(half_pitch)
                pose_msg.pose.orientation.w = math.cos(half_pitch)
                self.pose_pub.publish(pose_msg)

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JetsonToArmPi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
