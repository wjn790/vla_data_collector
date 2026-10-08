from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

from supervisor.long_task_s1_s2_b2_s3_executor import (
    CHAIN,
    ExtendedRuntime,
    POLICY_ROBOT_CONFIGS,
    SafetyStop,
    dry_run,
    load_all,
    required_prior_ids,
    smooth_arm_hand_transition_commands,
    smooth_transition_commands,
)


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config" / "inference_b0_s1_b1_s2_b2_s3.yaml"


class S1S2B2S3ExecutorTest(unittest.TestCase):
    def test_dedicated_profile_adds_b2_and_s3_without_changing_old_profile(self) -> None:
        supervisor, _priors, _collector, skills = load_all(CONFIG)
        self.assertEqual(CHAIN, ("B0", "S1", "S2", "B2", "S3"))
        self.assertEqual(supervisor["execution_profile"], "b0_s1_b1_s2_b2_s3")
        self.assertEqual(supervisor["policy"]["robot_config"], "svt_cabinet_s1_s2_s3")
        self.assertIn(supervisor["policy"]["robot_config"], POLICY_ROBOT_CONFIGS)
        self.assertEqual(
            supervisor["policy"]["deployment_contract_id"],
            "svt-cabinet-s1-s2-s3-from-base-step25000-v1",
        )
        self.assertEqual(len(supervisor["policy"]["normalization_sha256"]), 64)
        self.assertEqual(supervisor["control"]["inference_mode"], "synchronous")
        self.assertEqual(supervisor["control"]["execute_frames"], 25)
        self.assertNotIn("prefetch_after_frames", supervisor["control"])
        self.assertNotIn("prefetch_soft_deadline_ms", supervisor["control"])
        self.assertEqual(supervisor["control"]["inference_watchdog_ms"], 3000)
        self.assertEqual(supervisor["control"]["policy_warmup_repeats"], 2)
        self.assertTrue(supervisor["control"]["s1_hand_phase_filter_enabled"])
        self.assertTrue(supervisor["s1_preset"]["enabled"])
        self.assertEqual(required_prior_ids(skills), ("B0", "B1", "B2"))
        self.assertIsNone(skills["S2"].max_duration_sec)
        self.assertIsNone(skills["S3"].max_duration_sec)

    def test_current_synchronous_contract_completes_block_before_return(self) -> None:
        runtime = ExtendedRuntime.__new__(ExtendedRuntime)
        runtime.control = {"execute_frames": 3}
        runtime.executed_action_block_index = 0
        events = []
        runtime._execute_skill_action = lambda *args, **kwargs: events.append(
            ("execute", kwargs["chunk_frame_index"])
        )
        runtime._operator_answer = lambda: None
        runtime.hold = lambda: events.append(("hold",))
        runtime.event_log = SimpleNamespace(
            write=lambda event, **kwargs: events.append((event,))
        )
        runtime._start_policy_request = lambda *args: self.fail(
            "the current synchronous contract must not prefetch"
        )

        result = runtime._execute_action_block(
            "S1",
            SimpleNamespace(prompt="grasp the handle"),
            np.zeros((50, 14)),
            np.zeros((50, 12)),
            np.zeros(14),
            np.zeros(12),
            publish_base_zero=True,
        )

        self.assertEqual(
            events,
            [
                ("execute", 0),
                ("execute", 1),
                ("execute", 2),
                ("hold",),
                ("synchronous_action_block_complete",),
            ],
        )
        self.assertIsNone(result)

    def test_legacy_contract_requests_next_chunk_after_the_block(self) -> None:
        runtime = ExtendedRuntime.__new__(ExtendedRuntime)
        runtime.control = {"execute_frames": 3}
        runtime.executed_action_block_index = 0
        events = []
        request = object()
        runtime._execute_skill_action = lambda *args, **kwargs: events.append(
            ("execute", kwargs["chunk_frame_index"])
        )
        runtime._operator_answer = lambda: None
        runtime.hold = lambda: events.append(("hold",))
        runtime.event_log = SimpleNamespace(
            write=lambda event, **kwargs: events.append((event,))
        )
        runtime._start_policy_request = lambda executor, stage, prompt: (
            events.append(("request", stage, prompt)) or request
        )

        result = runtime._execute_action_block(
            object(),
            "S2",
            SimpleNamespace(prompt="push the door"),
            np.zeros((50, 14)),
            np.zeros((50, 12)),
            np.zeros(14),
            np.zeros(12),
            publish_base_zero=True,
        )

        self.assertEqual(
            events,
            [
                ("execute", 0),
                ("execute", 1),
                ("execute", 2),
                ("hold",),
                ("synchronous_action_block_complete",),
                ("request", "S2", "push the door"),
            ],
        )
        self.assertEqual(result, (request, None, False))

    def test_s3_preset_matches_training_first_frame_median(self) -> None:
        supervisor, _priors, _collector, _skills = load_all(CONFIG)
        preset = supervisor["s3_preset"]
        np.testing.assert_allclose(
            preset["left_arm_position"],
            [-0.031281, -0.185206, -0.141337, 1.544785, 0.403983, -0.075532, 0.006294],
            atol=1.0e-6,
        )
        self.assertEqual(
            preset["left_hand_position"],
            [254.0, 255.0, 254.0, 231.0, 254.0, 254.0],
        )

    def test_s3_transition_is_smooth_and_velocity_bounded(self) -> None:
        start = np.zeros(14)
        target = np.zeros(14)
        target[:7] = [0.1, -0.2, 0.3, 0.4, -0.1, 0.2, -0.3]
        commands = smooth_transition_commands(
            start,
            target,
            hz=15.0,
            max_velocity_radps=0.2,
            min_duration_sec=2.0,
            max_duration_sec=12.0,
        )
        np.testing.assert_allclose(commands[0], start)
        np.testing.assert_allclose(commands[-1], target)
        self.assertLessEqual(
            np.max(np.abs(np.diff(commands, axis=0))) * 15.0,
            0.2 + 1.0e-9,
        )
        np.testing.assert_allclose(np.diff(commands, axis=0)[0], 0.0, atol=2.0e-4)
        np.testing.assert_allclose(np.diff(commands, axis=0)[-1], 0.0, atol=2.0e-4)

    def test_s3_transition_rejects_an_excessively_distant_start(self) -> None:
        with self.assertRaisesRegex(SafetyStop, "above max_duration_sec"):
            smooth_transition_commands(
                np.zeros(14),
                np.full(14, 3.0),
                hz=15.0,
                max_velocity_radps=0.2,
                min_duration_sec=2.0,
                max_duration_sec=12.0,
            )

    def test_s3_arm_and_o6_transition_share_a_bounded_smooth_curve(self) -> None:
        start_arm = np.zeros(14)
        target_arm = np.zeros(14)
        target_arm[3] = 0.4
        start_hand = np.zeros(12)
        target_hand = np.zeros(12)
        target_hand[:6] = 255.0
        arm, hand = smooth_arm_hand_transition_commands(
            start_arm,
            target_arm,
            start_hand,
            target_hand,
            hz=15.0,
            max_arm_velocity_radps=0.2,
            max_hand_velocity_units_per_sec=150.0,
            min_duration_sec=2.0,
            max_duration_sec=12.0,
        )
        self.assertEqual(arm.shape[0], hand.shape[0])
        np.testing.assert_allclose(arm[0], start_arm)
        np.testing.assert_allclose(arm[-1], target_arm)
        np.testing.assert_allclose(hand[0], start_hand)
        np.testing.assert_allclose(hand[-1], target_hand)
        self.assertLessEqual(np.max(np.abs(np.diff(arm, axis=0))) * 15.0, 0.2)
        self.assertLessEqual(np.max(np.abs(np.diff(hand, axis=0))) * 15.0, 150.0)

    def test_local_dry_run_never_publishes_motion(self) -> None:
        report = dry_run(CONFIG)
        self.assertEqual(report["state_chain"], list(CHAIN))
        self.assertIs(report["motion_published"], False)
        self.assertEqual(set(report["priors"]), {"B0", "B1", "B2"})


if __name__ == "__main__":
    unittest.main()
