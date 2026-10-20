"""FK example regressions using real URDFs, without motor connections."""
import contextlib
import importlib.util
import io
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from reBotArm_control_py.kinematics import compute_fk, load_robot_model, get_arm_joint_count


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("fk_example", ROOT / "example/5_fk_test.py")
example = importlib.util.module_from_spec(spec)
spec.loader.exec_module(example)


@unittest.skipIf("test double" in example.pin.__version__, "Requires native Pinocchio")
class FkExampleTests(unittest.TestCase):
    def test_six_angles_with_comment(self):
        q = example.parse_joint_input("30 0 0 0 0 0 # arm only", 6)
        np.testing.assert_allclose(q, np.radians([30, 0, 0, 0, 0, 0]))

    def test_rejects_motor_gripper_or_raw_urdf_values_as_angles(self):
        for line in ("0 0 0 0 0", "0 0 0 0 0 0 2", "0 0 0 0 0 0 0 0",
                     "bad 0 0 0 0 0", "nan 0 0 0 0 0", "inf 0 0 0 0 0"):
            with self.subTest(line=line), contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaises(SystemExit):
                    example.parse_joint_input(line, 6)

    def test_rs_and_dm_six_angle_cli_matches_full_model_fk(self):
        for vendor, frame in (("RS", "gripper_end"), ("DM", "end_link")):
            with self.subTest(vendor=vendor):
                model = load_robot_model(str(ROOT / f"urdf/{vendor}/urdf/ReBot_Arm_{vendor}.urdf"))
                q = np.zeros(model.nq)
                q[:6] = np.radians([30, 10, -20, 0, 0, 0])
                expected = compute_fk(model, q, frame_name=frame)

                def fk(model, full_q):
                    np.testing.assert_allclose(full_q, q)
                    return compute_fk(model, full_q, frame_name=frame)

                output = io.StringIO()
                with patch.object(example, "load_robot_model", return_value=model), \
                        patch.object(example, "get_arm_joint_count", side_effect=lambda m: get_arm_joint_count(m, m.getFrameId(frame))), \
                        patch.object(example, "compute_fk", side_effect=fk) as called, \
                        patch("builtins.input", return_value="30 10 -20 0 0 0 # arm"), \
                        contextlib.redirect_stdout(output):
                    example.main()
                called.assert_called_once()
                self.assertIn("输入 6 个关节角度", output.getvalue())
                self.assertIn("URDF 夹指位移保持 0 米", output.getvalue())
                self.assertIn(f"X = {expected[0][0]:+.6f}", output.getvalue())

                # Finger opening lies downstream of the configured end frame.
                q[6:] = [.01, .01 if vendor == "RS" else -.01]
                opened = compute_fk(model, q, frame_name=frame)
                for a, b in zip(opened, expected):
                    np.testing.assert_allclose(a, b)
