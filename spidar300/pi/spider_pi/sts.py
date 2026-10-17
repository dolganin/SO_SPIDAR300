"""Feetech STS servo bus protocol (STS3215 and friends), over a half-duplex serial adapter.

Packet: 0xFF 0xFF id length instruction params... checksum, where
length = len(params) + 2 and checksum = ~(id + length + instruction + sum(params)) & 0xFF.
STS registers are little-endian.
"""
from __future__ import annotations

import time

PING, READ, WRITE, SYNC_READ, SYNC_WRITE = 0x01, 0x02, 0x03, 0x82, 0x83
BROADCAST = 0xFE

# STS3215 control table
ID = 0x05
TORQUE_ENABLE = 0x28
ACCELERATION = 0x29
GOAL_POSITION = 0x2A        # 2 bytes, then goal time (2) and goal speed (2)
LOCK = 0x37
PRESENT_POSITION = 0x38     # 2 bytes
PRESENT_VOLTAGE = 0x3E      # 1 byte, 0.1 V
PRESENT_TEMPERATURE = 0x3F  # 1 byte, deg C

TICKS_PER_TURN = 4096


class BusError(IOError):
    pass


def checksum(body: bytes) -> int:
    return ~sum(body) & 0xFF


def packet(servo_id: int, instruction: int, params: bytes = b"") -> bytes:
    body = bytes([servo_id, len(params) + 2, instruction]) + params
    return b"\xff\xff" + body + bytes([checksum(body)])


def u16(value: int) -> bytes:
    return int(value).to_bytes(2, "little")


class StsBus:
    def __init__(self, port: str, baudrate: int = 1_000_000, timeout: float = 0.02):
        import serial

        self.serial = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        self.timeout = timeout

    def close(self) -> None:
        self.serial.close()

    # -- low level -------------------------------------------------------------------

    def _send(self, data: bytes) -> None:
        self.serial.reset_input_buffer()
        self.serial.write(data)
        self.serial.flush()

    def _receive(self, servo_id: int) -> tuple[int, bytes]:
        """Read one status packet from servo_id: (error byte, params)."""
        deadline = time.monotonic() + self.timeout
        buf = b""
        while time.monotonic() < deadline:
            buf += self.serial.read(max(1, self.serial.in_waiting))
            start = buf.find(b"\xff\xff")
            while start >= 0 and len(buf) >= start + 4:
                sid, length = buf[start + 2], buf[start + 3]
                end = start + 4 + length
                if len(buf) < end:
                    break
                body = buf[start + 2:end - 1]
                if checksum(body) == buf[end - 1] and sid == servo_id:
                    return buf[start + 4], buf[start + 5:end - 1]
                buf = buf[start + 1:]
                start = buf.find(b"\xff\xff")
        raise BusError(f"servo {servo_id}: no reply")

    # -- instructions ----------------------------------------------------------------

    def ping(self, servo_id: int) -> bool:
        self._send(packet(servo_id, PING))
        try:
            self._receive(servo_id)
            return True
        except BusError:
            return False

    def read(self, servo_id: int, address: int, length: int) -> bytes:
        self._send(packet(servo_id, READ, bytes([address, length])))
        error, data = self._receive(servo_id)
        if len(data) != length:
            raise BusError(f"servo {servo_id}: short read")
        return data

    def write(self, servo_id: int, address: int, data: bytes) -> None:
        self._send(packet(servo_id, WRITE, bytes([address]) + data))
        if servo_id != BROADCAST:
            self._receive(servo_id)

    def sync_write(self, address: int, data: dict[int, bytes]) -> None:
        """Same register block on many servos in one packet; no replies."""
        length = len(next(iter(data.values())))
        params = bytes([address, length]) + b"".join(bytes([sid]) + d for sid, d in data.items())
        self.serial.write(packet(BROADCAST, SYNC_WRITE, params))
        self.serial.flush()

    def sync_read(self, address: int, length: int, ids: list[int]) -> dict[int, bytes]:
        """Each servo answers in turn; missing servos are left out of the result."""
        self._send(packet(BROADCAST, SYNC_READ, bytes([address, length]) + bytes(ids)))
        out = {}
        for sid in ids:
            try:
                _, data = self._receive(sid)
            except BusError:
                continue
            if len(data) == length:
                out[sid] = data
        return out

    # -- STS3215 helpers -------------------------------------------------------------

    def torque(self, ids: list[int], enable: bool) -> None:
        self.sync_write(TORQUE_ENABLE, {sid: bytes([1 if enable else 0]) for sid in ids})

    def positions(self, ids: list[int]) -> dict[int, int]:
        return {sid: int.from_bytes(d, "little") for sid, d in self.sync_read(PRESENT_POSITION, 2, ids).items()}

    def move(self, goals: dict[int, tuple[int, int]]) -> None:
        """goals: id -> (position ticks, speed ticks/s; 0 = maximum)."""
        self.sync_write(GOAL_POSITION, {sid: u16(pos) + u16(0) + u16(speed) for sid, (pos, speed) in goals.items()})

    def set_id(self, old: int, new: int) -> None:
        self.write(old, LOCK, b"\x00")
        self.write(old, ID, bytes([new]))
        self.write(new, LOCK, b"\x01")
