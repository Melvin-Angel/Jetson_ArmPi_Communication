"""Versioned, newline-delimited JSON protocol shared by the ROS 1 and ROS 2 nodes."""
from __future__ import annotations

import json
import socket
import time
import uuid

PROTOCOL_VERSION = 1
MAX_FRAME_BYTES = 16 * 1024
COMMAND_TYPES = {"base_velocity", "arm_pose", "gripper", "stop", "get_state", "heartbeat"}


class ProtocolError(ValueError):
    pass


def make_message(message_type, payload=None, request_id=None):
    return {
        "version": PROTOCOL_VERSION,
        "id": request_id or str(uuid.uuid4()),
        "type": message_type,
        "timestamp_ns": time.time_ns(),
        "payload": payload or {},
    }


def _number(value, name):
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise ProtocolError("%s must be a number" % name)
    return float(value)


def validate_message(message):
    if not isinstance(message, dict):
        raise ProtocolError("frame must be a JSON object")
    required = {"version", "id", "type", "timestamp_ns", "payload"}
    missing = required - set(message)
    if missing:
        raise ProtocolError("missing fields: %s" % ", ".join(sorted(missing)))
    if message["version"] != PROTOCOL_VERSION:
        raise ProtocolError("unsupported protocol version")
    if not isinstance(message["id"], str) or not message["id"]:
        raise ProtocolError("id must be a non-empty string")
    if not isinstance(message["type"], str):
        raise ProtocolError("type must be a string")
    if not isinstance(message["payload"], dict):
        raise ProtocolError("payload must be an object")

    payload = message["payload"]
    kind = message["type"]
    if kind == "base_velocity":
        for key in ("vx_mps", "vy_mps", "wz_radps"):
            _number(payload.get(key), key)
    elif kind == "arm_pose":
        position = payload.get("position_m")
        if not isinstance(position, list) or len(position) != 3:
            raise ProtocolError("position_m must be a three-element array")
        for index, value in enumerate(position):
            _number(value, "position_m[%d]" % index)
        _number(payload.get("pitch_deg"), "pitch_deg")
        _number(payload.get("duration_s"), "duration_s")
    elif kind == "gripper":
        if payload.get("command") not in ("open", "close") and "pulse" not in payload:
            raise ProtocolError("gripper requires command open/close or pulse")
        if "pulse" in payload:
            _number(payload["pulse"], "pulse")
        if "duration_s" in payload:
            _number(payload["duration_s"], "duration_s")
    return message


def encode(message):
    validate_message(message)
    raw = json.dumps(message, separators=(",", ":"), allow_nan=False).encode("utf-8") + b"\n"
    if len(raw) > MAX_FRAME_BYTES:
        raise ProtocolError("frame exceeds maximum size")
    return raw


class JsonLineSocket(object):
    """A buffered NDJSON socket. `recv_messages` returns all complete frames."""
    def __init__(self, sock):
        self.sock = sock
        self._buffer = b""

    def send(self, message):
        self.sock.sendall(encode(message))

    def recv_messages(self):
        data = self.sock.recv(4096)
        if not data:
            raise ConnectionError("peer disconnected")
        self._buffer += data
        if len(self._buffer) > MAX_FRAME_BYTES:
            raise ProtocolError("unterminated frame exceeds maximum size")
        messages = []
        while b"\n" in self._buffer:
            raw, self._buffer = self._buffer.split(b"\n", 1)
            if not raw:
                continue
            if len(raw) > MAX_FRAME_BYTES:
                raise ProtocolError("frame exceeds maximum size")
            try:
                message = json.loads(raw.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise ProtocolError("invalid JSON: %s" % exc)
            messages.append(validate_message(message))
        return messages
