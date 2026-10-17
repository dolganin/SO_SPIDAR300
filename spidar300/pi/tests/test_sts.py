from spider_pi import sts


def test_ping_packet():
    # Feetech manual example: ping id 1
    assert sts.packet(1, sts.PING) == bytes([0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB])


def test_sync_write_layout():
    class Port:
        def __init__(self):
            self.data = b""
        def write(self, d):
            self.data += d
        def flush(self):
            pass

    bus = sts.StsBus.__new__(sts.StsBus)
    bus.serial = Port()
    bus.move({1: (2048, 100), 2: (1000, 50)})
    d = bus.serial.data
    assert d[:5] == bytes([0xFF, 0xFF, sts.BROADCAST, 4 + 2 * 7, sts.SYNC_WRITE])
    assert d[5:7] == bytes([sts.GOAL_POSITION, 6])
    assert d[7:14] == bytes([1]) + (2048).to_bytes(2, "little") + b"\0\0" + (100).to_bytes(2, "little")
    assert d[-1] == sts.checksum(d[2:-1])


def test_receive_skips_noise_and_other_ids():
    class Port:
        def __init__(self, data):
            self.buf = data
            self.in_waiting = len(data)
        def read(self, n):
            out, self.buf = self.buf[:n], self.buf[n:]
            self.in_waiting = len(self.buf)
            return out

    reply_2 = sts.packet(2, 0x00, b"\x34\x12")
    reply_3 = sts.packet(3, 0x00, b"\x00\x08")
    bus = sts.StsBus.__new__(sts.StsBus)
    bus.timeout = 0.05
    bus.serial = Port(b"\x00\xff" + reply_2 + reply_3)
    assert bus._receive(3) == (0x00, b"\x00\x08")
