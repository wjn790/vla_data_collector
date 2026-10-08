from __future__ import annotations

import sys
from pathlib import Path


GLOVE_ROOT = Path("/home/svt/glove_control")
if not GLOVE_ROOT.is_dir():
    GLOVE_ROOT = Path(__file__).resolve().parents[1] / "glove_control"
sys.path.insert(0, str(GLOVE_ROOT))

from hand_command_arbiter import HandCommandArbiter, HandCommandLease


def test_model_mapping_preserves_unmodelled_wuji_joints() -> None:
    arbiter = HandCommandArbiter()
    left, right = arbiter.expand_model_action(
        [300, -5, 2, 3, 4, 5, 10, 11, 12, 13, 14, 15],
        list(range(20)),
    )
    assert left == [255, 0, 2, 3, 4, 5]
    assert right[2] == 10 and right[3] == 10
    assert right[0] == 11
    assert right[4] == 12
    assert right[8] == 13
    assert right[12] == 14
    assert right[16] == 15
    assert right[7] == 7 and right[19] == 19


def test_s5_glove_mode_changes_only_projected_index_channel() -> None:
    latched = [float(index) for index in range(20)]
    glove = [100.0 + index for index in range(20)]
    output = HandCommandArbiter.s5_index_only(glove, latched)
    changed = [index for index, (old, new) in enumerate(zip(latched, output)) if old != new]
    assert changed == [4]
    assert output[4] == 104.0


def test_command_lease_rejects_a_second_hand_publisher(tmp_path: Path) -> None:
    path = tmp_path / "hands.lock"
    first = HandCommandLease(path, mode="button_primitive", owner="test-one").acquire()
    try:
        second = HandCommandLease(path, mode="s5_index_only", owner="test-two")
        try:
            second.acquire()
            raise AssertionError("second hand publisher unexpectedly acquired the lease")
        except RuntimeError as exc:
            assert "already held" in str(exc)
    finally:
        first.close()
    HandCommandLease(path, mode="s5_index_only", owner="test-two").acquire().close()
