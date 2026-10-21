"""Check the examples against both current URDFs without motor or browser I/O."""
import contextlib
import importlib.util
import io
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

import numpy as np
import pinocchio as pin

from reBotArm_control_py.kinematics import compute_fk, get_arm_joint_count, load_robot_model


ROOT = Path(__file__).resolve().parents[1]


def load_example(name, path):
    spec = importlib.util.spec_from_file_location(name, ROOT / "example" / path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@unittest.skipIf("test double" in pin.__version__, "Requires native Pinocchio")
class ExampleModelMappingTests(unittest.TestCase):
    def models(self):
        for vendor, frame in (("RS", "gripper_end"), ("DM", "end_link")):
            model = load_robot_model(str(ROOT / f"urdf/{vendor}/urdf/ReBot_Arm_{vendor}.urdf"))
            yield vendor, model, model.getFrameId(frame)

    def test_current_models_have_six_arm_variables_and_two_finger_variables(self):
        for vendor, model, frame_id in self.models():
            with self.subTest(vendor=vendor):
                self.assertEqual((model.nq, model.nv), (8, 8))
                self.assertEqual(get_arm_joint_count(model, frame_id), 6)
                self.assertEqual(list(model.names)[1:7], [f"joint{i}" for i in range(1, 7)])
                for joint in model.joints[7:]:
                    self.assertTrue(joint.shortname().startswith("JointModelP"))

    def test_ik_cli_reports_only_arm_angles(self):
        example = load_example("ik_example", "6_ik_test.py")
        for vendor, model, frame_id in self.models():
            with self.subTest(vendor=vendor):
                target = compute_fk(model, pin.neutral(model), frame_name=model.frames[frame_id].name)[0]
                output = io.StringIO()
                with patch.object(example, "load_robot_model", return_value=model), \
                        patch.object(example, "get_end_effector_frame_id", return_value=frame_id), \
                        patch.object(example, "get_arm_joint_count", side_effect=lambda m, *args: get_arm_joint_count(m, frame_id)), \
                        patch.object(example, "solve_ik", wraps=example.solve_ik) as solve, \
                        patch("builtins.input", return_value=" ".join(map(str, target))), \
                        contextlib.redirect_stdout(output):
                    example.main()
                self.assertEqual(solve.call_args.kwargs["controlled_joints"], 6)
                self.assertIn("Converged  : 是 / Yes", output.getvalue())
                self.assertIn("arm=6", output.getvalue())
                self.assertNotIn("gripper_joint", output.getvalue())
                self.assertNotIn("finger_left", output.getvalue())

    def test_sim_ik_pads_only_arm_angles_for_display(self):
        example = load_example("ik_sim_example", "sim/ik_sim.py")
        for vendor, model, frame_id in self.models():
            with self.subTest(vendor=vendor):
                seed = pin.neutral(model)
                seed[6:] = [.01, .01 if vendor == "RS" else -.01]
                pos, rot, _ = compute_fk(model, seed, frame_name=model.frames[frame_id].name)
                result = example.solve_arm_ik(model, frame_id, seed, pos, rot)
                self.assertTrue(result.success)
                self.assertEqual(result.q.shape, (8,))
                np.testing.assert_array_equal(result.q[6:], [0, 0])

    def test_fk_sim_accepts_six_and_seven_angles_without_mapping_motor_angle_to_fingers(self):
        for vendor, model, frame_id in self.models():
            with self.subTest(vendor=vendor):
                example = load_example("fk_sim_example", "sim/fk_sim.py")
                viz = Mock(model=model)
                output = io.StringIO()

                def fk(m, q):
                    return compute_fk(m, q, frame_name=model.frames[frame_id].name)

                with patch.object(example, "Visualizer", return_value=viz), \
                        patch.object(example, "get_arm_joint_count", return_value=6), \
                        patch.object(example, "compute_fk", side_effect=fk), \
                        patch.object(example.signal, "signal"), \
                        patch.object(example.time, "sleep"), \
                        patch("builtins.input", side_effect=["30 0 0 0 0 0", "30 0 0 0 0 0 12 # gripper motor degrees", "q"]), \
                        contextlib.redirect_stdout(output):
                    example.main()
                self.assertEqual(viz.update.call_count, 3)
                six = viz.update.call_args_list[1].args[0]
                seven = viz.update.call_args_list[2].args[0]
                np.testing.assert_array_equal(six, seven)
                np.testing.assert_allclose(seven[:6], np.radians([30, 0, 0, 0, 0, 0]))
                np.testing.assert_array_equal(seven[6:], 0.)
                self.assertIn("已忽略夹爪电机角度 12°", output.getvalue())

    def test_fk_sim_rejects_invalid_optional_gripper_angles_and_wrong_counts(self):
        example = load_example("fk_sim_example", "sim/fk_sim.py")
        model = load_robot_model(str(ROOT / "urdf/RS/urdf/ReBot_Arm_RS.urdf"))
        viz = Mock(model=model)
        with patch.object(example, "Visualizer", return_value=viz), \
                patch.object(example, "get_arm_joint_count", return_value=6), \
                patch.object(example.signal, "signal"), \
                patch.object(example.time, "sleep"), \
                patch("builtins.input", side_effect=["30 0 0 0 0 0 nan", "30 0 0 0 0 0 inf", "30 0 0 0 0 0 bad", "30 0 0 0 0", "30 0 0 0 0 0 1 2", "q"]), \
                contextlib.redirect_stdout(io.StringIO()):
            example.main()
        viz.update.assert_called_once()

    def test_sim_trajectory_does_not_move_fingers_in_null_space(self):
        example = load_example("traj_sim_example", "sim/traj_sim.py")
        for vendor, model, frame_id in self.models():
            with self.subTest(vendor=vendor):
                q_start = pin.neutral(model)
                direction = 1 if vendor == "RS" else -1
                q_start[1:3] = direction * np.array([.7, 1.1])
                q_end = q_start.copy()
                q_end[0] = .02

                def fk(m, q):
                    return compute_fk(m, q, frame_name=model.frames[frame_id].name)

                target = pin.SE3(fk(model, q_end)[2])
                solved, success = example._solve_ik(model, frame_id, target, q_start, example.IKParams())
                self.assertTrue(success)
                np.testing.assert_array_equal(solved[6:], [0, 0])
                with patch.object(example, "compute_fk", side_effect=fk), \
                        patch.object(example.time, "sleep"), \
                        contextlib.redirect_stdout(io.StringIO()):
                    _, trajectory, _, stats = example.run_trajectory(
                        Mock(), model, frame_id, q_start, q_end, .08, dt=.02, null_gain=.1)
                self.assertEqual(stats.success_rate, 1.)
                for point in trajectory:
                    np.testing.assert_array_equal(point.q[6:], [0, 0])

    def test_hardware_examples_exclude_gripper_and_preserve_orientation(self):
        for filename, method in (("7_arm_ik_control.py", "move_to_ik"),
                                 ("8_arm_traj_control.py", "move_to_traj")):
            with self.subTest(example=filename):
                example = load_example("arm_pose_example", filename)
                robot, controller = Mock(), Mock()
                robot.arm.num_joints = 6
                robot.get_state.return_value = (np.array([0.] * 6 + [2.]), None, None)
                with patch.object(example, "RebotArm", return_value=robot), \
                        patch.object(example, "RebotArmEndPose", return_value=controller), \
                        patch.object(example, "joint_to_pose", return_value=(np.zeros(3), np.array([.1, .2, .3]))) as pose, \
                        patch("builtins.input", side_effect=["end_state", "1", "nan 0 0", "0.2 0 0.3 # keep orientation", "q"]), \
                        contextlib.redirect_stdout(io.StringIO()):
                    example.main()
                for call in pose.call_args_list:
                    np.testing.assert_array_equal(call.args[0], np.zeros(6))
                expected = dict(x=.2, y=0., z=.3, roll=.1, pitch=.2, yaw=.3)
                if method == "move_to_traj":
                    expected["duration"] = 2.
                getattr(controller, method).assert_called_once_with(**expected)
                controller.end.assert_called_once_with()

    def test_controller_trajectory_tracks_only_hardware_arm_joints(self):
        from reBotArm_control_py.actuator import RebotArm
        from reBotArm_control_py.controllers import rebotarm_endpose_controller as module
        robot = RebotArm()
        controller = module.RebotArmEndPose(robot)
        controller._running = True
        q = np.zeros(robot.num_joints)
        full_q = pin.neutral(controller._model)
        position, rotation, _ = compute_fk(controller._model, full_q)
        rpy = pin.rpy.matrixToRpy(rotation)
        with patch.object(robot, "get_state", return_value=(q, q, q)), \
                patch.object(module, "track_trajectory", wraps=module.track_trajectory) as track, \
                patch.object(module.threading, "Thread"):
            self.assertTrue(controller.move_to_traj(*position, *rpy, duration=.04))
        self.assertEqual(track.call_args.kwargs["controlled_joints"], robot.arm.num_joints)
