from __future__ import annotations

import importlib.util
import threading
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def load_controller_module():
    candidates = (
        ROOT / "o6_button_controller.py",
        Path("/home/svt/glove_control/wuji_bridge/o6_button_controller.py"),
    )
    path = next((candidate for candidate in candidates if candidate.is_file()), None)
    assert path is not None
    spec = importlib.util.spec_from_file_location("test_o6_button_controller", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeLogger:
    def __init__(self) -> None:
        self.messages: list[str] = []

    def info(self, message: str) -> None:
        self.messages.append(message)


def test_wuji_multithread_access_disables_sdk_construction_thread_check() -> None:
    module = load_controller_module()

    class FakeHand:
        def __init__(self) -> None:
            self.disable_calls = 0

        def disable_thread_safe_check(self) -> None:
            self.disable_calls += 1

    hand = FakeHand()
    module.enable_wuji_multithread_access(hand)

    assert hand.disable_calls == 1


def test_left_c_runs_three_fingers_then_thumb_and_index_after_three_seconds() -> None:
    module = load_controller_module()
    node = object.__new__(module.HandButtonController)
    node.left_active = False
    node.left_action = {"toggle": True}
    node.left_action_key = "4"
    node.left_fixed = {0: 102, 1: 102, 2: 91, 3: 0, 4: 0, 5: 0}
    node.left_staged_activation_enabled = True
    node.left_first_stage_fixed = {3: 0, 4: 0, 5: 0}
    node.left_stage_delay_sec = 3.0
    node.left_snapshot = None
    node.left_pending_target = None
    node.left_stage_deadline = None
    node.left_command = [200, 201, 202, 203, 204, 205]
    node.left_actual = list(node.left_command)
    node.feedback_lock = threading.Lock()
    node.last_button_events = []
    logger = FakeLogger()
    node.get_logger = lambda: logger
    sent: list[list[int]] = []

    def send(target: list[int]) -> None:
        node.left_command = list(target)
        sent.append(list(target))

    node._send_o6 = send
    node._toggle_left()

    assert sent == [[200, 201, 202, 0, 0, 0]]
    assert node.left_pending_target == [102, 102, 91, 0, 0, 0]
    assert node.left_stage_deadline is not None
    deadline = node.left_stage_deadline
    node._advance_left_staged_action(deadline - 0.001)
    assert len(sent) == 1
    node._advance_left_staged_action(deadline)
    assert sent[-1] == [102, 102, 91, 0, 0, 0]
    assert node.left_pending_target is None
    assert node.left_stage_deadline is None

    node._toggle_left()
    assert sent[-1] == [200, 201, 202, 203, 204, 205]
    assert node.left_active is False
