#!/usr/bin/env python3
"""ROS 1 ArmPi Pro adapter and single-client TCP server."""
import math
import os
import socket
import sys
import threading
import time

import rospy
from chassis_control.msg import SetVelocity
from hiwonder_servo_msgs.msg import MultiRawIdPosDur, RawIdPosDur, ServoStateList
from kinematics import ik_transform

PROTOCOL_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../protocol"))
if PROTOCOL_DIR not in sys.path:
    sys.path.insert(0, PROTOCOL_DIR)
from armpi_protocol import JsonLineSocket, ProtocolError, make_message, validate_message


class ArmPiToJetson(object):
    def __init__(self):
        rospy.init_node("armpi_to_jetson")
        self.bind_host = rospy.get_param("~bind_host", "192.168.50.20")
        self.port = int(rospy.get_param("~port", 8765))
        self.watchdog_s = float(rospy.get_param("~watchdog_s", 0.5))
        self.max_linear_mps = float(rospy.get_param("~max_linear_mps", 0.25))
        self.max_angular_radps = float(rospy.get_param("~max_angular_radps", 1.0))
        self.workspace_min = rospy.get_param("~workspace_min_m", [-0.20, 0.05, -0.10])
        self.workspace_max = rospy.get_param("~workspace_max_m", [0.20, 0.30, 0.30])
        self.gripper_open = int(rospy.get_param("~gripper_open_pulse", 200))
        self.gripper_close = int(rospy.get_param("~gripper_close_pulse", 500))

        self.velocity_pub = rospy.Publisher("/chassis_control/set_velocity", SetVelocity, queue_size=1)
        self.servo_pub = rospy.Publisher(
            "/servo_controllers/port_id_1/multi_id_pos_dur", MultiRawIdPosDur, queue_size=1)
        rospy.Subscriber("/servo_controllers/port_id_1/servo_states", ServoStateList,
                         self._servo_states_callback, queue_size=1)
        self.ik = ik_transform.ArmIK()
        self.last_command_time = time.monotonic()
        self.last_pose = None
        self.servo_states = {}
        self.channel = None
        self.channel_lock = threading.Lock()
        self.send_lock = threading.Lock()
        rospy.Timer(rospy.Duration(0.1), self._watchdog)
        rospy.Timer(rospy.Duration(0.2), self._telemetry)

    def _servo_states_callback(self, message):
        self.servo_states = {str(state.id): {"goal": state.goal, "position": state.position,
                                             "error": state.error, "voltage_mv": state.voltage}
                             for state in message.servo_states}

    def _state_payload(self):
        return {"pose": self.last_pose, "pose_is_estimated": True,
                "servos": self.servo_states, "watchdog_s": self.watchdog_s}

    def _send(self, message):
        with self.channel_lock:
            channel = self.channel
        if channel is None:
            return
        try:
            with self.send_lock:
                channel.send(message)
        except (OSError, ConnectionError, ProtocolError):
            pass

    def _reply(self, request, kind, payload):
        self._send(make_message(kind, payload, request["id"]))

    def _publish_stop(self):
        self.velocity_pub.publish(0.0, 0.0, 0.0)

    def _watchdog(self, _event):
        if time.monotonic() - self.last_command_time > self.watchdog_s:
            self._publish_stop()

    def _telemetry(self, _event):
        self._send(make_message("telemetry", self._state_payload()))

    def _handle_base_velocity(self, request):
        p = request["payload"]
        vx, vy, wz = float(p["vx_mps"]), float(p["vy_mps"]), float(p["wz_radps"])
        speed = math.hypot(vx, vy)
        if speed > self.max_linear_mps or abs(wz) > self.max_angular_radps:
            raise ProtocolError("velocity exceeds configured safety limit")
        direction_deg = math.degrees(math.atan2(vy, vx)) if speed else 0.0
        # ArmPi Pro expects velocity in mm/s and direction in degrees.
        self.velocity_pub.publish(speed * 1000.0, direction_deg, wz)
        return {"velocity_mps": speed, "direction_deg": direction_deg, "wz_radps": wz}

    def _handle_arm_pose(self, request):
        p = request["payload"]
        position = [float(v) for v in p["position_m"]]
        if any(v < low or v > high for v, low, high in zip(position, self.workspace_min, self.workspace_max)):
            raise ProtocolError("pose is outside configured workspace")
        pitch = float(p["pitch_deg"])
        duration_s = float(p["duration_s"])
        if not 0.05 <= duration_s <= 10.0:
            raise ProtocolError("duration_s must be within [0.05, 10]")
        # Vendor ArmIK supplies position + pitch only; it returns pulses for servos 3..6.
        target = self.ik.setPitchRanges(tuple(position), pitch, -180, 0)
        if not target:
            raise ProtocolError("no inverse-kinematics solution")
        pulses = target[1]
        duration_ms = int(round(duration_s * 1000))
        self.servo_pub.publish(MultiRawIdPosDur([
            RawIdPosDur(3, int(pulses["servo3"]), duration_ms),
            RawIdPosDur(4, int(pulses["servo4"]), duration_ms),
            RawIdPosDur(5, int(pulses["servo5"]), duration_ms),
            RawIdPosDur(6, int(pulses["servo6"]), duration_ms),
        ]))
        self.last_pose = {"position_m": position, "pitch_deg": target[2]}
        return {"target_pulses": pulses, "pose": self.last_pose, "duration_s": duration_s}

    def _handle_gripper(self, request):
        p = request["payload"]
        pulse = int(p.get("pulse", self.gripper_open if p.get("command") == "open" else self.gripper_close))
        if not 0 <= pulse <= 1000:
            raise ProtocolError("gripper pulse must be in [0, 1000]")
        duration_ms = int(round(float(p.get("duration_s", 0.5)) * 1000))
        self.servo_pub.publish(MultiRawIdPosDur([RawIdPosDur(1, pulse, duration_ms)]))
        return {"pulse": pulse, "duration_s": duration_ms / 1000.0}

    def _handle(self, request):
        validate_message(request)
        kind = request["type"]
        if kind == "base_velocity":
            result = self._handle_base_velocity(request)
        elif kind == "arm_pose":
            result = self._handle_arm_pose(request)
        elif kind == "gripper":
            result = self._handle_gripper(request)
        elif kind == "stop":
            self._publish_stop()
            result = {"stopped": True}
        elif kind == "get_state":
            self._reply(request, "state", self._state_payload())
            return
        elif kind == "heartbeat":
            result = {"alive": True}
        else:
            raise ProtocolError("unsupported command type: %s" % kind)
        self.last_command_time = time.monotonic()
        self._reply(request, "accepted", result)

    def serve(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.bind_host, self.port))
        server.listen(1)
        server.settimeout(1.0)
        rospy.loginfo("ArmPi TCP bridge listening on %s:%d", self.bind_host, self.port)
        while not rospy.is_shutdown():
            try:
                conn, address = server.accept()
            except socket.timeout:
                continue
            rospy.loginfo("Jetson connected from %s", address[0])
            conn.settimeout(0.5)
            channel = JsonLineSocket(conn)
            with self.channel_lock:
                self.channel = channel
            try:
                while not rospy.is_shutdown():
                    try:
                        messages = channel.recv_messages()
                    except socket.timeout:
                        continue
                    for request in messages:
                        try:
                            self._handle(request)
                        except ProtocolError as exc:
                            self._reply(request, "rejected", {"reason": str(exc)})
            except (ConnectionError, OSError, ProtocolError) as exc:
                rospy.logwarn("Jetson connection closed: %s", exc)
            finally:
                with self.channel_lock:
                    self.channel = None
                conn.close()
                self._publish_stop()
        server.close()


if __name__ == "__main__":
    ArmPiToJetson().serve()
