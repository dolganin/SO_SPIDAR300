"""STS3215 servo bus on top of Feetech's scservo_sdk (sms_sts), as used in leg_twister.ipynb / servo.py.

Works with both flavours of scservo_sdk:
  * FTServo_Python (github.com/ftservo) — has the sms_sts class used in the notebooks;
  * feetech-servo-sdk from PyPI (what LeRobot installs) — no sms_sts, only PacketHandler;
    _PypiSmsSts below provides the same calls on top of it.

Board: Waveshare / Feetech serial bus servo driver over USB (/dev/ttyACM0), 1 Mbaud.
"""
from __future__ import annotations

import time

TICKS_PER_TURN = 4096
CENTER = 2048

ADDR_OFFSET = 31            # EEPROM, sign bit 11: present position = raw - offset
ADDR_TORQUE_ENABLE = 40
ADDR_PRESENT_POSITION = 56
COMM_SUCCESS = 0
EEPROM_RETRIES = 3
EEPROM_PAUSE = 0.03         # s; EEPROM writes take a few ms on the servo
CENTER_TOLERANCE = 3        # ticks


class BusError(IOError):
    pass


class _PypiSmsSts:
    """The subset of sms_sts used here, built from PyPI scservo_sdk (PacketHandler takes the port per call)."""

    ACC = 41

    def __init__(self, port):
        import scservo_sdk as sdk

        self._sdk, self._port = sdk, port
        self._ph = sdk.PacketHandler(0)           # 0 = STS byte order (little-endian)
        self.groupSyncWrite = sdk.GroupSyncWrite(port, self._ph, self.ACC, 7)

    @staticmethod
    def scs_toscs(a, b):
        return (-a | (1 << b)) if a < 0 else a

    def ping(self, scs_id):
        return self._ph.ping(self._port, scs_id)

    def ReadPos(self, scs_id):
        value, result, error = self._ph.read2ByteTxRx(self._port, scs_id, ADDR_PRESENT_POSITION)
        return self._sdk.SCS_TOHOST(value, 15), result, error

    def write1ByteTxRx(self, scs_id, address, value):
        return self._ph.write1ByteTxRx(self._port, scs_id, address, value)

    def write2ByteTxRx(self, scs_id, address, value):
        return self._ph.write2ByteTxRx(self._port, scs_id, address, value)

    def SyncWritePosEx(self, scs_id, position, speed, acc):
        lo, hi = self._sdk.SCS_LOBYTE, self._sdk.SCS_HIBYTE
        position = self.scs_toscs(position, 15)
        return self.groupSyncWrite.addParam(scs_id, [acc, lo(position), hi(position), 0, 0, lo(speed), hi(speed)])

    def LockEprom(self, scs_id):
        return self.write1ByteTxRx(scs_id, 55, 1)

    def unLockEprom(self, scs_id):
        return self.write1ByteTxRx(scs_id, 55, 0)


class StsBus:
    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 1_000_000, acc: int = 0):
        from scservo_sdk import PortHandler

        self.port = PortHandler(port)
        if not self.port.openPort():
            raise BusError(f"cannot open {port}")
        if not self.port.setBaudRate(baudrate):
            raise BusError(f"cannot set {port} to {baudrate} baud")
        try:
            from scservo_sdk import sms_sts
        except ImportError:
            self.sdk = _PypiSmsSts(self.port)
        else:
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
        """Unlock, write, lock; retried, with pauses: a servo busy writing EEPROM may miss the next packet."""
        write = self.sdk.write1ByteTxRx if nbytes == 1 else self.sdk.write2ByteTxRx
        last = None
        for _ in range(EEPROM_RETRIES):
            self.sdk.unLockEprom(servo_id)
            time.sleep(EEPROM_PAUSE)
            result, error = write(servo_id, address, value)
            time.sleep(EEPROM_PAUSE)
            self.sdk.LockEprom(servo_id if address != 5 else value)
            time.sleep(EEPROM_PAUSE)
            if result == COMM_SUCCESS and not error:
                return
            last = (result, error)
        raise BusError(f"servo {servo_id}: EEPROM write {address} failed {EEPROM_RETRIES} times {last}; "
                       "check the servo power")

    def set_id(self, old: int, new: int) -> None:
        self._write_eeprom(old, 5, new, 1)

    def set_offset(self, servo_id: int, offset: int) -> None:
        if abs(offset) > 2047:
            raise ValueError(f"offset {offset} out of range")
        self._write_eeprom(servo_id, ADDR_OFFSET, self.sdk.scs_toscs(int(offset), 11), 2)

    def _position(self, servo_id: int) -> int:
        for _ in range(EEPROM_RETRIES):
            pos = self.positions([servo_id]).get(servo_id)
            if pos is not None:
                return pos
            time.sleep(EEPROM_PAUSE)
        raise BusError(f"servo {servo_id}: no position reply; check the servo power")

    def center_here(self, servo_id: int) -> int:
        """Make the servo's current pose read as CENTER (2048), like leg_twister.ipynb. Returns the offset."""
        for _ in range(EEPROM_RETRIES):
            self.set_offset(servo_id, 0)
            raw = self._position(servo_id)
            offset = raw - CENTER
            if offset > 2047:
                offset -= TICKS_PER_TURN
            elif offset < -2047:
                offset += TICKS_PER_TURN
            self.set_offset(servo_id, offset)
            if abs(self._position(servo_id) - CENTER) <= CENTER_TOLERANCE:
                return offset
        raise BusError(f"servo {servo_id}: does not read {CENTER} after setting the offset; was the leg held still?")
