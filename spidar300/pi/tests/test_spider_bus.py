from unittest.mock import Mock, call

from spider_pi.spider_bus import SpiderBus


def test_default_calls_original_spider_exactly_as_notebook():
    spider = Mock()
    spider._default_positions = {113: 700}
    bus = SpiderBus("unused", spider=spider)
    bus.default()
    assert spider.mock_calls == [
        call.enable_torque(),
        call.move_to_default_position(speed=2000, acc=0, blocking=False),
    ]
    assert bus.default_positions == {113: 700}


def test_motion_reading_and_release_use_original_methods():
    spider = Mock()
    spider._default_positions = {113: 700}
    spider.position.return_value = 1234
    bus = SpiderBus("unused", spider=spider)
    assert bus.positions([113]) == {113: 1234}
    bus.move({113: (700, 2000)})
    spider.move_to.assert_called_once_with(113, 700, 2000, acc=0, blocking=False)
    bus.torque([113], False)
    spider.disable_torque.assert_called_once_with(113)
    bus.close()
    spider._port_handler.closePort.assert_called_once_with()


def test_diagnostics_reads_original_spider_without_motion():
    spider = Mock()
    spider._default_positions = {113: 700}
    spider.read2Byte.return_value = 700
    spider.read1Byte.return_value = 0
    bus = SpiderBus("unused", spider=spider)
    assert bus.diagnostics([113])[113]["goal"] == 700
    spider.move_to.assert_not_called()
    spider.enable_torque.assert_not_called()
    spider.move_to_default_position.assert_not_called()


def test_actual_spider_default_sends_original_sdk_writes(monkeypatch):
    import importlib.util
    import sys
    from pathlib import Path
    from types import ModuleType

    sdk = ModuleType("scservo_sdk")
    driver = Mock()
    sdk.sms_sts = Mock(return_value=driver)
    sdk.PortHandler = Mock()
    monkeypatch.setitem(sys.modules, "scservo_sdk", sdk)
    servo = ModuleType("servo")
    servo.Servo = Mock()
    monkeypatch.setitem(sys.modules, "servo", servo)
    path = Path(__file__).resolve().parents[2] / "spider_control.py"
    spec = importlib.util.spec_from_file_location("original_spider_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    spider = module.Spider(port="/dev/test")
    bus = SpiderBus("unused", spider=spider)
    bus.default()
    assert driver.WritePosEx.call_args_list == [
        call(i, spider._default_positions[i], 2000, acc=0) for i in spider._id_list
    ]
    assert len(driver.WritePosEx.call_args_list) == 18
    assert driver.write1ByteTxRx.call_args_list == [call(i, 40, 1) for i in spider._id_list]
    driver.SyncWritePosEx.assert_not_called()


def test_standalone_default_uses_original_blocking_wait():
    spider = Mock()
    spider._default_positions = {113: 700}
    bus = SpiderBus("unused", spider=spider)
    bus.default(blocking=True)
    spider.move_to_default_position.assert_called_once_with(speed=2000, acc=0, blocking=True)
