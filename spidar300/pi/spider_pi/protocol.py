"""UDP messages between the laptop (spider_gait pi_bridge) and the Raspberry Pi. One JSON object per datagram.

laptop -> pi
  {"type": "cmd", "vx": .., "vy": .., "wz": ..}          velocity, resend >= 1/cmd_timeout Hz
  {"type": "stand_up" | "lie_down", "id": n}            answered with "reply"
  {"type": "set", "id": n, "params": {"period": 0.6}}   gait parameters, answered with "reply"
pi -> laptop (to the last sender)
  {"type": "reply", "id": n, "ok": bool, "message": ".."}
  {"type": "state", "mode": "standing", "names": [...], "position": [...], "command": [...]}
"""
from __future__ import annotations

import json

DEFAULT_PORT = 5005
MAX_DATAGRAM = 8192


def encode(msg: dict) -> bytes:
    return json.dumps(msg, separators=(",", ":")).encode()


def decode(data: bytes) -> dict | None:
    try:
        msg = json.loads(data.decode())
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None
    return msg if isinstance(msg, dict) and "type" in msg else None
