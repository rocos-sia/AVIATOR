"""Calibration CLI checks with a fake hand; these tests never connect to hardware."""
import io
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

spec = importlib.util.spec_from_file_location(
    "rh56ftp_force_calibrate", Path(__file__).resolve().parents[2] / "force_calibrate.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def state():
    return {"angle": [998] * 5 + [500], "force": [1695, 79, 105, 112, 97, 172],
            "current": [0] * 6, "err": [0] * 6}


class CalibrationTests(unittest.TestCase):
    def run_cli(self, hand, answer="CALIBRATE"):
        now = [0.0]
        def sleep(seconds):
            now[0] += seconds
        self.output = io.StringIO()
        with patch.object(module, "HandLink", return_value=hand) as factory, \
                patch("builtins.input", return_value=answer), \
                patch.object(module.time, "monotonic", side_effect=lambda: now[0]), \
                patch.object(module.time, "sleep", side_effect=sleep), \
                patch("sys.stdout", self.output), patch("sys.stderr", self.output):
            result = module.main(["--host", "192.168.11.210", "--observe-seconds", "1"])
        factory.assert_called_once_with("192.168.11.210", 6000, 1.0)
        hand.close.assert_called_once_with()
        return result

    def test_single_write_and_zero_force_observation(self):
        hand = Mock()
        after = {**state(), "force": [0] * 6}
        hand.read_state.side_effect = [state(), state(), after, after]
        self.assertEqual(self.run_cli(hand), 0)
        hand.force_calibrate.assert_called_once_with()
        hand.write_angle_set.assert_not_called()
        hand.save_flash.assert_not_called()
        hand.reset_para.assert_not_called()
        self.assertIn("1695", self.output.getvalue())
        self.assertIn("0..0 / 0..0 / 0..0 / 0..0 / 0..0 / 0..0", self.output.getvalue())

    def test_cancellation_does_not_write(self):
        hand = Mock()
        hand.read_state.return_value = state()
        self.assertEqual(self.run_cli(hand, answer="no"), 1)
        hand.force_calibrate.assert_not_called()

    def test_bad_position_current_or_fault_prevents_write(self):
        for field, values in (("angle", [949] + [998] * 4 + [500]),
                              ("angle", [998] * 5 + [1001]),
                              ("current", [1] + [0] * 5),
                              ("err", [5] + [0] * 5),
                              ("force", [0] * 5)):
            with self.subTest(field=field, values=values):
                hand = Mock()
                hand.read_state.return_value = {**state(), field: values}
                self.assertEqual(self.run_cli(hand), 1)
                hand.force_calibrate.assert_not_called()

    def test_state_change_during_confirmation_prevents_write(self):
        hand = Mock()
        hand.read_state.side_effect = [state(), {**state(), "angle": [0] * 6}]
        self.assertEqual(self.run_cli(hand), 1)
        hand.force_calibrate.assert_not_called()

    def test_write_failure_is_not_retried_or_claimed_successful(self):
        hand = Mock()
        hand.read_state.return_value = state()
        hand.force_calibrate.side_effect = RuntimeError("TCP response timeout")
        self.assertEqual(self.run_cli(hand), 1)
        hand.force_calibrate.assert_called_once_with()
        self.assertIn("是否收到校准请求尚不确定", self.output.getvalue())
        self.assertNotIn("Modbus 返回成功", self.output.getvalue())

    def test_feedback_failure_reports_that_write_already_succeeded(self):
        hand = Mock()
        hand.read_state.side_effect = [state(), state(), RuntimeError("read failed")]
        self.assertEqual(self.run_cli(hand), 1)
        hand.force_calibrate.assert_called_once_with()
        self.assertIn("校准指令已写入，但反馈检查未完成", self.output.getvalue())

    def test_connection_failure_closes_without_writing(self):
        hand = Mock()
        hand.connect.side_effect = RuntimeError("connection refused")
        self.assertEqual(self.run_cli(hand), 1)
        hand.force_calibrate.assert_not_called()

    def test_invalid_arguments_are_rejected_before_connection(self):
        for args in ([], ["--host", ""], ["--host", "left", "--port", "0"],
                     ["--host", "left", "--timeout", "nan"],
                     ["--host", "left", "--observe-seconds", "0"]):
            with self.subTest(args=args), patch.object(module, "HandLink") as factory, \
                    patch("sys.stderr", io.StringIO()), self.assertRaises(SystemExit):
                module.main(args)
            factory.assert_not_called()


if __name__ == "__main__":
    unittest.main()
