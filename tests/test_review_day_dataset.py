from review_day_dataset import technical_disposition


def test_fully_valid_episode_is_ready_after_bounded_repair() -> None:
    value, reasons = technical_disposition(
        valid_rate=1.0, max_invalid_run=0, invalid_reasons={}
    )
    assert value == "ready_after_bounded_repair"
    assert "operator_success_confirmation_still_required" in reasons


def test_fully_valid_episode_with_dense_repairs_requires_visual_review() -> None:
    value, reasons = technical_disposition(
        valid_rate=1.0,
        max_invalid_run=0,
        invalid_reasons={},
        repaired_rate=0.20,
        max_repaired_rate=0.05,
    )
    assert value == "manual_visual_review_high_repair_rate"
    assert "repaired_frame_rate_exceeds_quality_threshold" in reasons


def test_strong_repair_candidate() -> None:
    value, _ = technical_disposition(
        valid_rate=0.92, max_invalid_run=3, invalid_reasons={"camera:camera_top": 30}
    )
    assert value == "strong_repair_candidate"


def test_long_arm_gap_is_excluded() -> None:
    value, _ = technical_disposition(
        valid_rate=0.93, max_invalid_run=10, invalid_reasons={"arm": 10}
    )
    assert value == "exclude_sync_gap"


def test_dense_camera_loss_recommends_retake() -> None:
    value, _ = technical_disposition(
        valid_rate=0.81, max_invalid_run=4, invalid_reasons={"camera:camera_top": 70}
    )
    assert value == "retake_recommended"
