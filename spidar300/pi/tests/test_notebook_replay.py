from types import ModuleType
from unittest.mock import Mock, call

import pytest

from spider_pi.notebook_replay import cells_for, replay, main


def test_executes_actual_spider_notebook(monkeypatch):
    import sys
    module = ModuleType("spider_control")
    spider = Mock()
    module.Spider = Mock(return_value=spider)
    monkeypatch.setitem(sys.modules, "spider_control", module)
    replay("spider-test")
    module.Spider.assert_called_once_with()
    assert spider.mock_calls == [
        call.ping(), call.enable_torque(),
        call.move_to_default_position(speed=2000, acc=0, blocking=True),
        call.disable_torque(), call._port_handler.closePort(),
    ]


def test_original_error_is_not_hidden_and_torque_is_released(monkeypatch):
    import sys
    module = ModuleType("spider_control")
    spider = Mock()
    module.Spider = Mock(return_value=spider)
    spider.move_to_default_position.side_effect = IOError("Servo 113 returned error: 32")
    monkeypatch.setitem(sys.modules, "spider_control", module)
    with pytest.raises(IOError, match="error: 32"):
        replay("spider-test")
    spider.disable_torque.assert_called_once_with()
    spider._port_handler.closePort.assert_called_once_with()


def test_default_recipe_includes_no_eeprom_cells():
    cells = cells_for("leg-default")
    assert [i for i, _ in cells] == [0, 1, 5, 6]
    source = "\n".join(s for _, s in cells)
    assert "servo.WritePosEx(id, move_to, 50, 10)" in source
    assert "write2ByteTxRx" not in source
    assert "unLockEprom" not in source


def test_show_does_not_open_hardware(capsys):
    main(["--show"])
    assert "spider.move_to_default_position" in capsys.readouterr().out
