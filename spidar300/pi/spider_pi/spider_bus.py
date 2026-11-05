"""Thin adapter to the existing, notebook-tested spider_control.Spider.

No packet implementation, SDK compatibility shim or EEPROM calibration here.
"""
from pathlib import Path
import sys


def open_spider(port):
    root = str(Path(__file__).resolve().parents[2])
    if root not in sys.path:
        sys.path.insert(0, root)
    from spider_control import Spider
    return Spider(port=port)


class SpiderBus:
    def __init__(self, port, spider=None):
        self.spider = spider if spider is not None else open_spider(port)
        self.default_positions = dict(self.spider._default_positions)

    def close(self):
        self.spider._port_handler.closePort()

    def ping(self, servo_id):
        try:
            self.spider.position(servo_id)
            return True
        except IOError:
            return False

    def positions(self, ids):
        out = {}
        for i in ids:
            try:
                out[i] = self.spider.position(i)
            except IOError:
                pass
        return out

    def torque(self, ids, enable):
        for i in ids:
            if enable:
                self.spider.enable_torque(i)
            else:
                self.spider.disable_torque(i)

    def default(self, blocking=False):
        # Same calls and values as test_spider_control.ipynb; arrival is polled
        # by the runtime so UDP replies and SIGINT remain responsive.
        self.spider.enable_torque()
        self.spider.move_to_default_position(speed=2000, acc=0, blocking=blocking)

    def move(self, goals):
        for i, (position, speed) in goals.items():
            self.spider.move_to(i, position, speed, acc=0, blocking=False)

    def diagnostics(self, ids):
        registers = {"min": (9, 2), "max": (11, 2), "offset": (31, 2),
                     "mode": (33, 1), "torque": (40, 1), "goal": (42, 2),
                     "position": (56, 2), "load_raw": (60, 2),
                     "voltage_raw": (62, 1), "temperature": (63, 1),
                     "status": (65, 1), "moving": (66, 1), "current_raw": (69, 2)}
        out = {}
        for i in ids:
            row = {}
            for name, (address, width) in registers.items():
                try:
                    read = self.spider.read1Byte if width == 1 else self.spider.read2Byte
                    row[name] = read(i, address)
                except IOError:
                    row[name] = None
            out[i] = row
        return out
