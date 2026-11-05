from pathlib import Path
from unittest.mock import Mock, call
import pytest
from spider_pi.calibrate import jog
from spider_pi.robot import load_config
from spider_pi.servos import ServoMap
from spider_gait.kinematics import Hexapod


def setup_jog():
    cfg = load_config(Path(__file__).resolve().parents[1] / "config.yaml")
    robot = Hexapod.load(cfg["legs"])
    return robot, ServoMap(cfg["servos"], robot.joint_names)


def test_jog_moves_only_one_servo_without_changing_calibration(monkeypatch):
    robot, servos = setup_jog()
    bus = Mock()
    bus.positions.side_effect = [{113: 2920}, {113: 2855}]
    monkeypatch.setattr("builtins.input", lambda *args: "")
    jog(bus, robot, servos, "leg_lf_tibia", 0.1)
    assert bus.move.call_args_list == [call({113: (2920, 100)}), call({113: (2855, 100)})]
    bus.torque.assert_called_once_with([113], True)
    assert servos.cal[servos.index("leg_lf_tibia")].zero == 2920


def test_jog_releases_selected_servo_if_feedback_fails(monkeypatch):
    robot, servos = setup_jog()
    bus = Mock()
    bus.positions.side_effect = [{113: 2920}, {}]
    monkeypatch.setattr("builtins.input", lambda *args: "")
    with pytest.raises(IOError):
        jog(bus, robot, servos, "leg_lf_tibia", 0.1)
    assert bus.torque.call_args_list == [call([113], True), call([113], False)]


def test_jog_rejects_large_motion_before_touching_hardware():
    robot, servos = setup_jog()
    bus = Mock()
    with pytest.raises(ValueError):
        jog(bus, robot, servos, "leg_lf_tibia", 1.0)
    assert not bus.mock_calls


def test_jog_cannot_cross_measured_lr_tibia_stop(monkeypatch):
    robot, servos = setup_jog()
    bus = Mock()
    bus.positions.return_value = {133: 1060}
    monkeypatch.setattr("builtins.input", lambda *args: "")
    with pytest.raises(ValueError, match="outside"):
        jog(bus, robot, servos, "leg_lr_tibia", 0.1)
    bus.move.assert_not_called()
    bus.torque.assert_not_called()
