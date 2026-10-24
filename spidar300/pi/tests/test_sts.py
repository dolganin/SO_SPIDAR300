"""StsBus on a stub of scservo_sdk.sms_sts (same calls as leg_twister.ipynb)."""
from spider_pi import sts


class StubSdk:
    """Minimal sms_sts: positions, EEPROM offset (sign bit 11), sync write."""

    def __init__(self, raw):
        self.raw = dict(raw)                 # encoder before offset
        self.offset = {i: 0 for i in raw}
        self.locked = {i: True for i in raw}
        self.synced = []
        self.packets = []
        sdk = self

        class Group:
            def txPacket(self):
                sdk.packets.append(list(sdk.synced))
                return 0

            def clearParam(self):
                sdk.synced.clear()

        self.groupSyncWrite = Group()

    @staticmethod
    def scs_toscs(a, b):
        return (-a | (1 << b)) if a < 0 else a

    def ping(self, i):
        return 777, (0 if i in self.raw else -6), 0

    def ReadPos(self, i):
        if i not in self.raw:
            return 0, -6, 0
        return (self.raw[i] - self.offset[i]) % 4096, 0, 0

    def SyncWritePosEx(self, i, pos, speed, acc):
        self.synced.append((i, pos, speed, acc))
        return True

    def write1ByteTxRx(self, i, addr, value):
        return 0, 0

    def write2ByteTxRx(self, i, addr, value):
        assert not self.locked[i], "EEPROM must be unlocked"
        if addr == sts.ADDR_OFFSET:
            self.offset[i] = -(value & 0x7FF) if value & (1 << 11) else value
        return 0, 0

    def unLockEprom(self, i):
        self.locked[i] = False

    def LockEprom(self, i):
        self.locked[i] = True


def bus_with(raw):
    bus = sts.StsBus.__new__(sts.StsBus)
    bus.sdk = StubSdk(raw)
    bus.acc = 0
    return bus


def test_move_is_one_sync_packet():
    bus = bus_with({111: 0, 112: 0})
    bus.move({111: (2048, 100), 112: (1000, 50)})
    assert bus.sdk.packets == [[(111, 2048, 100, 0), (112, 1000, 50, 0)]]
    assert bus.sdk.synced == []


def test_positions_skip_silent_servos():
    bus = bus_with({111: 1234})
    assert bus.positions([111, 999]) == {111: 1234}


def test_center_here_positive_negative_and_wrapping():
    # 3561 and 2373 are from leg_twister.ipynb; 4 is a tibia sitting next to the 0/4095 wrap
    bus = bus_with({112: 3561, 132: 2373, 113: 4, 123: 1000})
    for i in (112, 132, 113, 123):
        bus.center_here(i)
        assert bus.positions([i]) == {i: 2048}
        assert abs(bus.sdk.offset[i]) <= 2047
        assert bus.sdk.locked[i]
    assert bus.sdk.offset[112] == 1513
    assert bus.sdk.offset[123] == -1048
