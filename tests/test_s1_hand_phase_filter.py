from __future__ import annotations

import numpy as np

from supervisor.long_task_supervisor import S1HandPhaseFilter


def make_filter() -> S1HandPhaseFilter:
    return S1HandPhaseFilter(
        {
            "s1_hand_phase_filter_enabled": True,
            "s1_hand_open_threshold": 200,
            "s1_hand_closed_threshold": 150,
            "s1_three_finger_closed_threshold": 40,
            "s1_three_finger_confirm_frames": 5,
            "s1_grasp_confirm_frames": 5,
        }
    )


def targets(hand_values: list[float], arm_value: float = 1.0):
    return np.full(14, arm_value), np.asarray(hand_values + [0.0] * 6)


def test_premature_full_grasp_holds_left_arm_and_hand() -> None:
    hand_filter = make_filter()
    arm, hand = targets([105, 108, 98, 2, 3, 4])
    last_arm = np.arange(14, dtype=float)
    last_hand = np.arange(12, dtype=float)

    filtered_arm, filtered_hand, action, transition = hand_filter.apply(
        arm, hand, last_arm, last_hand
    )

    assert action == "hold_premature_grasp"
    assert transition is None
    np.testing.assert_array_equal(filtered_arm[:7], last_arm[:7])
    np.testing.assert_array_equal(filtered_hand[:6], last_hand[:6])
    assert hand_filter.phase == "approach"


def test_grasp_debounce_holds_the_left_pair_then_accepts_together() -> None:
    hand_filter = make_filter()
    last_arm = np.zeros(14)
    last_hand = np.asarray([245, 248, 244, 8, 7, 6] + [0.0] * 6)
    three_arm, three_hand = targets([245, 248, 244, 8, 7, 6])

    for _ in range(5):
        _, _, action, _ = hand_filter.apply(
            three_arm, three_hand, last_arm, last_hand
        )
    assert action == "pass"
    assert hand_filter.phase == "three_closed"

    grasp_arm, grasp_hand = targets([105, 108, 98, 2, 3, 4], arm_value=2.0)
    for _ in range(4):
        filtered_arm, filtered_hand, action, transition = hand_filter.apply(
            grasp_arm, grasp_hand, last_arm, last_hand
        )
        assert action == "hold_grasp_candidate"
        assert transition is None
        np.testing.assert_array_equal(filtered_arm[:7], last_arm[:7])
        np.testing.assert_array_equal(filtered_hand[:6], last_hand[:6])

    filtered_arm, filtered_hand, action, transition = hand_filter.apply(
        grasp_arm, grasp_hand, last_arm, last_hand
    )
    assert action == "accept_grasp"
    assert transition == "three_closed->grasped"
    np.testing.assert_array_equal(filtered_arm, grasp_arm)
    np.testing.assert_array_equal(filtered_hand, grasp_hand)
    assert hand_filter.phase == "grasped"


def test_grasp_latch_blocks_reopen_but_keeps_arm_adjustment() -> None:
    hand_filter = make_filter()
    hand_filter.phase = "grasped"
    last_arm, last_hand = targets([105, 108, 98, 2, 3, 4], arm_value=1.0)
    adjusted_arm, reopen_hand = targets([245, 248, 244, 230, 240, 238], arm_value=3.0)

    filtered_arm, filtered_hand, action, transition = hand_filter.apply(
        adjusted_arm, reopen_hand, last_arm, last_hand
    )

    assert action == "hold_grasp_latch"
    assert transition is None
    np.testing.assert_array_equal(filtered_arm, adjusted_arm)
    np.testing.assert_array_equal(filtered_hand[:6], last_hand[:6])
    assert hand_filter.phase == "grasped"
