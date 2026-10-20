"""Exercise the interactive POS_VEL example without connecting to hardware."""
import contextlib
import io
from pathlib import Path
import runpy
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np


SCRIPT = Path(__file__).resolve().parents[1] / "example" / "4_pos_vel_control.py"


class PosVelExampleTests(unittest.TestCase):
    def run_example(self, commands, n_gripper=1):
        initial = np.radians([1, 2, 3, 4, 5, 6] + [12] * n_gripper)
        arm = SimpleNamespace(num_joints=6, mode_pos_vel=Mock(), send_pos_vel=Mock())
        gripper = SimpleNamespace(num_joints=n_gripper, mode_pos_vel=Mock(),
                                  send_pos_vel=Mock())
        robot = SimpleNamespace(
            arm=arm, gripper=gripper, has_gripper=bool(n_gripper), rate=200,
            connect=Mock(), enable_all=Mock(), disconnect=Mock(),
            get_positions=Mock(return_value=initial), start_control_loop=Mock(),
        )
        actuator = ModuleType("reBotArm_control_py.actuator")
        actuator.RebotArm = Mock(return_value=robot)
        inputs = iter(commands)
        samples = []

        def read_input(prompt):
            callback = robot.start_control_loop.call_args.args[0]
            callback(robot, .005)
            samples.append(np.concatenate([
                arm.send_pos_vel.call_args.args[0].copy(),
                gripper.send_pos_vel.call_args.args[0].copy() if n_gripper else [],
            ]))
            value = next(inputs, EOFError())
            if isinstance(value, BaseException):
                raise value
            return value

        output = io.StringIO()
        with patch.dict(sys.modules, {"reBotArm_control_py.actuator": actuator}), \
                patch("builtins.input", side_effect=read_input), \
                contextlib.redirect_stdout(output):
            runpy.run_path(str(SCRIPT), run_name="__main__")
        robot.disconnect.assert_called_once_with()
        return samples, output.getvalue(), robot

    def test_six_values_with_comment_preserve_initial_gripper_target(self):
        samples, output, _ = self.run_example([
            "30 0 0 0 0 0 # 控制1号电机正转30度", "q",
        ])
        np.testing.assert_allclose(np.degrees(samples[0]), [1, 2, 3, 4, 5, 6, 12])
        np.testing.assert_allclose(np.degrees(samples[1]), [30, 0, 0, 0, 0, 0, 12])
        self.assertIn("6 或 7个角度", output)

    def test_seven_values_update_gripper_and_next_six_preserve_it(self):
        samples, _, _ = self.run_example([
            "30 0 0 0 0 0 2 # arm + gripper", "15 0 0 0 0 0", "q",
        ])
        np.testing.assert_allclose(np.degrees(samples[1]), [30, 0, 0, 0, 0, 0, 2])
        np.testing.assert_allclose(np.degrees(samples[2]), [15, 0, 0, 0, 0, 0, 2])

    def test_invalid_input_leaves_all_targets_unchanged(self):
        for invalid in ("30 0 0 0 0", "30 0 0 0 0 0 2 9", "30 0 0 0 0 0 bad",
                        "nan 0 0 0 0 0", "30 0 0 0 0 0 inf"):
            with self.subTest(invalid=invalid):
                samples, _, _ = self.run_example([invalid, "q"])
                np.testing.assert_array_equal(samples[1], samples[0])

    def test_without_gripper_accepts_six_and_rejects_seven(self):
        samples, output, robot = self.run_example([
            "30 0 0 0 0 0 2", "30 0 0 0 0 0", "q",
        ], n_gripper=0)
        np.testing.assert_array_equal(samples[1], samples[0])
        np.testing.assert_allclose(np.degrees(samples[2]), [30, 0, 0, 0, 0, 0])
        self.assertIn("需要 6 个值", output)
        robot.gripper.send_pos_vel.assert_not_called()

    def test_eof_and_keyboard_interrupt_disconnect(self):
        for ending in (EOFError(), KeyboardInterrupt()):
            with self.subTest(ending=type(ending).__name__):
                self.run_example(["# comment only", "state", ending])
