"""SPIDAR300 Raspberry Pi runtime: tripod gait + STS3215 servo bus, driven over UDP."""
import sys
from pathlib import Path

# gait.py / kinematics.py are plain Python inside the ROS package; reuse them from the repo.
_GAIT = Path(__file__).resolve().parents[2] / "sim" / "ros2_ws" / "src" / "spider_gait"
if str(_GAIT) not in sys.path:
    sys.path.insert(0, str(_GAIT))
