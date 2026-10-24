"""STS3215 servo bus on top of Feetech's scservo_sdk (sms_sts), as used in leg_twister.ipynb / servo.py.

Board: Waveshare / Feetech serial bus servo driver over USB (/dev/ttyACM0), 1 Mbaud.
"""
from __future__ import annotations

TICKS_PER_TURN = 4096
CENTER = 2048

ADDR_OFFSET = 31            # EEPROM, sign bit 11: present position = raw - offset
ADDR_TORQUE_ENABLE = 40
ADDR_PRESENT_POSITION = 56
COMM_SUCCESS = 0


class BusError(IOError):
    pass


class StsBus:
    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 1_000_000, acc: int = 0):
        from scservo_sdk import PortHandler, sms_sts

        self.port = PortHandler(port)
        if not self.port.openPort():
            raise BusError(f"cannot open {port}")
        if not self.port.setBaudRate(baudrate):
            raise BusError(f"cannot set {port} to {baudrate} baud")
        self.sdk = sms_sts(self.port)
        self.acc = acc

    def close(self) -> None:
        self.port.closePort()

    def ping(self, servo_id: int) -> bool:
        _, result, _ = self.sdk.ping(servo_id)
        return result == COMM_SUCCESS

    def torque(self, ids: list[int], enable: bool) -> None:
        for i in ids:
            self.sdk.write1ByteTxRx(i, ADDR_TORQUE_ENABLE, 1 if enable else 0)

    def positions(self, ids: list[int]) -> dict[int, int]:
        """Present positions of the servos that answered."""
        out = {}
        for i in ids:
            pos, result, _ = self.sdk.ReadPos(i)
            if result == COMM_SUCCESS:
                out[i] = pos
        return out

    def move(self, goals: dict[int, tuple[int, int]]) -> None:
        """goals: id -> (position ticks, speed ticks/s; 0 = maximum). One sync-write packet."""
        for i, (pos, speed) in goals.items():
            self.sdk.SyncWritePosEx(i, int(pos), int(speed), self.acc)
        self.sdk.groupSyncWrite.txPacket()
        self.sdk.groupSyncWrite.clearParam()

    # -- EEPROM ----------------------------------------------------------------------

    def _write_eeprom(self, servo_id: int, address: int, value: int, nbytes: int) -> None:
        self.sdk.unLockEprom(servo_id)
        write = self.sdk.write1ByteTxRx if nbytes == 1 else self.sdk.write2ByteTxRx
        result, error = write(servo_id, address, value)
        self.sdk.LockEprom(servo_id if address != 5 else value)
        if result != COMM_SUCCESS or error:
            raise BusError(f"servo {servo_id}: EEPROM write {address} failed ({result}, {error})")

    def set_id(self, old: int, new: int) -> None:
        self._write_eeprom(old, 5, new, 1)

    def set_offset(self, servo_id: int, offset: int) -> None:
        if abs(offset) > 2047:
            raise ValueError(f"offset {offset} out of range")
        self._write_eeprom(servo_id, ADDR_OFFSET, self.sdk.scs_toscs(int(offset), 11), 2)

    def center_here(self, servo_id: int) -> int:
        """Make the servo's current pose read as CENTER (2048), like leg_twister.ipynb. Returns the offset."""
        self.set_offset(servo_id, 0)
        raw = self.positions([servo_id]).get(servo_id)
        if raw is None:
            raise BusError(f"servo {servo_id}: no position")
        offset = raw - CENTER
        if offset > 2047:
            offset -= TICKS_PER_TURN
        elif offset < -2047:
            offset += TICKS_PER_TURN
        self.set_offset(servo_id, offset)
        return offset
