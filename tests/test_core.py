from __future__ import annotations

import json
import multiprocessing as mp
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common import prefix_vector, reorder_joint_state
from camera_sync import CameraSynchronizer, sync_timestamp_ns
from camera_capture import camera_worker, frame_gap_diagnostics
from hand_log_source import decode_hand_record, project_wuji
from collect_episode import CameraDiagnostics, hand_record_is_ready
from offline_align import (
    align_episode,
    nearest_record,
    refresh_paired_segment_indices,
    repair_isolated_frames,
)
from realign_day import merge_alignment_policy


PROJECTION = [
    {"name": "thumb_flex", "terms": [{"index": 2, "weight": 0.5}, {"index": 3, "weight": 0.5}]},
    {"name": "thumb_yaw", "terms": [{"index": 0, "weight": 1.0}]},
    {"name": "index", "terms": [{"index": 4, "weight": 1.0}]},
    {"name": "middle", "terms": [{"index": 8, "weight": 1.0}]},
    {"name": "ring", "terms": [{"index": 12, "weight": 1.0}]},
    {"name": "pinky", "terms": [{"index": 16, "weight": 1.0}]},
]


class ProjectionTests(unittest.TestCase):
    def test_projection(self) -> None:
        raw = list(range(20))
        self.assertEqual(project_wuji(raw, PROJECTION), [2.5, 0.0, 4.0, 8.0, 12.0, 16.0])

    def test_command_fallback_is_explicit(self) -> None:
        record = {
            "_timestamp_ns": 123,
            "left_o6_command": list(range(6)),
            "right_wuji_command": [list(range(offset, offset + 4)) for offset in range(0, 20, 4)],
        }
        value = decode_hand_record(record, {"right_virtual_projection": PROJECTION})
        self.assertEqual(value["state_source"], "command_fallback")
        self.assertEqual(len(value["action"]), 12)

    def test_startup_requires_fresh_hardware_feedback(self) -> None:
        reference_ns = 1_800_000_000_000_000_000
        config = {
            "hands": {
                "stale_after_ms": 250,
                "right_virtual_projection": PROJECTION,
            }
        }
        record = {
            "_timestamp_ns": reference_ns - 10_000_000,
            "_actual_state_timestamp_ns": reference_ns - 20_000_000,
            "left_o6_command": list(range(6)),
            "right_wuji_command": list(range(20)),
            "left_o6_actual": list(range(6)),
            "right_wuji_actual": list(range(20)),
        }
        self.assertTrue(hand_record_is_ready(record, config, reference_ns))
        record["_actual_state_timestamp_ns"] = reference_ns - 300_000_000
        self.assertFalse(hand_record_is_ready(record, config, reference_ns))


class JointOrderingTests(unittest.TestCase):
    def test_arm_command_uses_first_seven_of_eight_channels(self) -> None:
        self.assertEqual(prefix_vector(list(range(8)), 7), list(range(7)))

    def test_arm_command_rejects_too_few_channels(self) -> None:
        self.assertIsNone(prefix_vector(list(range(6)), 7))

    def test_joint_state_is_reordered_by_name(self) -> None:
        sample = {
            "timestamp_ns": 1,
            "received_ns": 2,
            "name": ["b", "ignored", "a"],
            "position": [20, 99, 10],
            "velocity": [2, 9, 1],
            "effort": [0.2, 0.9, 0.1],
        }
        result = reorder_joint_state(sample, ["a", "b"])
        self.assertIsNotNone(result)
        self.assertEqual(result["position"], [10.0, 20.0])


def camera_sample(camera: str, timestamp_ms: float) -> dict:
    timestamp_ns = int(timestamp_ms * 1_000_000)
    return {
        "camera": camera,
        "host_timestamp_ns": timestamp_ns,
        "hardware_timestamp_ns": timestamp_ns,
        "hardware_timestamp_domain": "timestamp_domain.global_time",
    }


class CameraSyncTests(unittest.TestCase):
    def test_hardware_global_time_is_used(self) -> None:
        sample = camera_sample("left", 100.0)
        sample["host_timestamp_ns"] = 110_000_000
        self.assertEqual(sync_timestamp_ns(sample), 100_000_000)

    def test_pair_under_limit_is_selected_once(self) -> None:
        sync = CameraSynchronizer(["left", "right"])
        sync.add(camera_sample("left", 100.0))
        sync.add(camera_sample("right", 82.5))
        left, right, status = sync.select_wrist_pair("left", "right", 110_000_000, 50, 30)
        self.assertIsNotNone(left)
        self.assertIsNotNone(right)
        self.assertTrue(status["valid"])
        self.assertAlmostEqual(status["skew_ms"], 17.5)
        left, right, status = sync.select_wrist_pair("left", "right", 110_000_000, 50, 30)
        self.assertIsNone(left)
        self.assertIsNone(right)
        self.assertEqual(status["reason"], "missing_new_frame")

    def test_49ms_pair_is_rejected(self) -> None:
        sync = CameraSynchronizer(["left", "right"])
        sync.add(camera_sample("left", 100.0))
        sync.add(camera_sample("right", 149.0))
        left, right, status = sync.select_wrist_pair("left", "right", 150_000_000, 60, 30)
        self.assertIsNone(left)
        self.assertIsNone(right)
        self.assertFalse(status["valid"])
        self.assertEqual(status["reason"], "skew_exceeded")
        self.assertAlmostEqual(status["skew_ms"], 49.0)

    def test_pair_selection_prioritizes_exposure_skew(self) -> None:
        sync = CameraSynchronizer(["left", "right"])
        sync.add(camera_sample("left", 100.0))
        sync.add(camera_sample("left", 130.0))
        sync.add(camera_sample("right", 102.0))
        sync.add(camera_sample("right", 155.0))
        left, right, status = sync.select_wrist_pair(
            "left", "right", 160_000_000, 70, 30
        )
        self.assertTrue(status["valid"])
        self.assertAlmostEqual(sync_timestamp_ns(left), 100_000_000)
        self.assertAlmostEqual(sync_timestamp_ns(right), 102_000_000)
        self.assertAlmostEqual(status["skew_ms"], 2.0)


class CameraCaptureTests(unittest.TestCase):
    def test_frame_gap_estimates_missing_camera_periods(self) -> None:
        diagnostic = frame_gap_diagnostics(
            1_000_000_000,
            1_100_000_000,
            fps=30,
            warning_threshold_ms=50,
        )
        self.assertAlmostEqual(diagnostic["hardware_timestamp_gap_ms"], 100.0)
        self.assertEqual(diagnostic["estimated_missing_frames"], 2)
        self.assertTrue(diagnostic["hardware_gap_warning"])

    def test_camera_diagnostics_summarizes_and_counts_warnings(self) -> None:
        diagnostics = CameraDiagnostics(
            {"camera_top": {"enabled": True, "backend": "zed"}},
            {
                "print_warnings": False,
                "hardware_gap_warn_ms": 50,
                "grab_duration_warn_ms": 50,
                "encoder_queue_delay_warn_ms": 50,
            },
        )
        diagnostics.observe(
            {
                "kind": "frame",
                "camera": "camera_top",
                "grab_duration_ms": 70.0,
                "hardware_timestamp_gap_ms": 100.0,
                "hardware_gap_warning": True,
                "estimated_missing_frames": 2,
                "encoder_queue_delay_ms": 60.0,
                "encoding_duration_ms": 8.0,
                "capture_to_encoding_complete_ms": 75.0,
                "encoder_queue_depth_at_submit": 2,
                "zed_grab_error_count": 1,
                "zed_grab_error_codes": {"CAMERA_REBOOTING": 1},
            }
        )
        diagnostics.observe(
            {
                "kind": "camera_diagnostic",
                "camera": "camera_top",
                "event": "zed_grab_error",
                "zed_grab_error_count": 3,
                "zed_grab_error_codes": {"CAMERA_REBOOTING": 3},
            }
        )
        diagnostics.observe(
            {
                "kind": "capture_drop",
                "camera": "camera_top",
                "dropped_count": 2,
            }
        )

        summary = diagnostics.summary()["camera_top"]
        self.assertEqual(summary["frame_count_observed"], 1)
        self.assertEqual(summary["hardware_gap_warning_count"], 1)
        self.assertEqual(summary["estimated_missing_frames"], 2)
        self.assertEqual(summary["grab_duration_warning_count"], 1)
        self.assertEqual(summary["encoder_queue_delay_warning_count"], 1)
        self.assertEqual(summary["capture_drop_count"], 2)
        self.assertEqual(summary["zed_grab_error_count"], 3)
        self.assertEqual(summary["zed_grab_error_codes"], {"CAMERA_REBOOTING": 3})
        self.assertAlmostEqual(summary["timing"]["grab_duration_ms"]["p90"], 70.0)

    def test_camera_diagnostics_rebuilds_from_complete_raw_stream(self) -> None:
        diagnostics = CameraDiagnostics(
            {"camera_top": {"enabled": True, "backend": "zed"}},
            {"print_warnings": False},
        )
        diagnostics.observe(
            {
                "kind": "frame",
                "camera": "camera_top",
                "capture_sequence": 0,
                "encoder_queue_delay_ms": 1.0,
            }
        )
        with tempfile.TemporaryDirectory() as temporary:
            path = (
                Path(temporary)
                / "raw"
                / "cameras"
                / "camera_top"
                / "timestamps.jsonl"
            )
            path.parent.mkdir(parents=True)
            records = [
                {
                    "capture_sequence": 0,
                    "encoder_queue_delay_ms": 10.0,
                    "hardware_timestamp_gap_ms": None,
                },
                {
                    "capture_sequence": 2,
                    "encoder_queue_delay_ms": 30.0,
                    "hardware_timestamp_gap_ms": 66.7,
                    "hardware_gap_warning": True,
                    "estimated_missing_frames": 1,
                },
            ]
            path.write_text(
                "".join(json.dumps(record) + "\n" for record in records),
                encoding="utf-8",
            )
            diagnostics.rebuild_from_raw(Path(temporary))

        summary = diagnostics.summary()["camera_top"]
        self.assertEqual(summary["frame_count_observed"], 2)
        self.assertEqual(summary["capture_drop_count"], 1)
        self.assertEqual(summary["hardware_gap_warning_count"], 1)
        self.assertEqual(summary["estimated_missing_frames"], 1)
        self.assertAlmostEqual(
            summary["timing"]["encoder_queue_delay_ms"]["median"], 20.0
        )

    def test_mock_capture_flushes_background_encoder(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            context = mp.get_context("spawn")
            output_queue = context.Queue(maxsize=64)
            stop_event = context.Event()
            process = context.Process(
                target=camera_worker,
                args=(
                    "mock_camera",
                    {
                        "backend": "mock",
                        "width": 64,
                        "height": 48,
                        "fps": 30,
                        "jpeg_quality": 80,
                        "encoder_queue_frames": 4,
                    },
                    output_queue,
                    stop_event,
                    temporary,
                ),
            )
            process.start()
            time.sleep(3.5)
            stop_event.set()
            process.join(timeout=15)
            self.assertFalse(process.is_alive())
            self.assertEqual(process.exitcode, 0)

            timestamp_path = (
                Path(temporary)
                / "raw"
                / "cameras"
                / "mock_camera"
                / "timestamps.jsonl"
            )
            records = [json.loads(line) for line in timestamp_path.read_text().splitlines()]
            self.assertGreaterEqual(len(records), 5)
            self.assertTrue(
                all((Path(temporary) / record["path"]).is_file() for record in records)
            )
            self.assertTrue(
                all(isinstance(record.get("encoding_completed_ns"), int) for record in records)
            )
            capture_sequences = [record.get("capture_sequence") for record in records]
            self.assertEqual(capture_sequences[0], 0)
            self.assertTrue(
                all(
                    isinstance(previous, int)
                    and isinstance(current, int)
                    and current > previous
                    for previous, current in zip(
                        capture_sequences, capture_sequences[1:]
                    )
                )
            )
            self.assertTrue(
                all(
                    isinstance(record.get("encoder_queue_delay_ms"), (int, float))
                    and record["encoder_queue_delay_ms"] >= 0
                    for record in records
                )
            )
            self.assertTrue(
                all(
                    isinstance(record.get("encoding_duration_ms"), (int, float))
                    and record["encoding_duration_ms"] >= 0
                    for record in records
                )
            )
            self.assertTrue(
                all(
                    isinstance(
                        record.get("capture_to_encoding_complete_ms"), (int, float)
                    )
                    and record["capture_to_encoding_complete_ms"] >= 0
                    for record in records
                )
            )

    def test_transport_latency_does_not_reject_synchronized_exposures(self) -> None:
        sync = CameraSynchronizer(["left", "right"])
        left = camera_sample("left", 100.0)
        right = camera_sample("right", 82.5)
        left["host_timestamp_ns"] = 165_000_000
        right["host_timestamp_ns"] = 164_000_000
        sync.add(left)
        sync.add(right)
        selected_left, selected_right, status = sync.select_wrist_pair(
            "left", "right", 170_000_000, 50, 30
        )
        self.assertIsNotNone(selected_left)
        self.assertIsNotNone(selected_right)
        self.assertTrue(status["valid"])
        self.assertAlmostEqual(status["skew_ms"], 17.5)


class OfflineAlignmentTests(unittest.TestCase):
    def test_deployed_alignment_policy_overrides_historical_repair_limits(self) -> None:
        episode_config = {
            "fps": 15,
            "offline_alignment": {
                "camera_max_delta_ms": 40,
                "repair": {
                    "enabled": False,
                    "max_neighbor_span_ms": 170,
                    "max_consecutive_frames": 1,
                    "historical_only": True,
                },
                "historical_only": True,
            },
        }
        policy_config = {
            "offline_alignment": {
                "camera_max_delta_ms": 30,
                "repair": {
                    "enabled": True,
                    "max_neighbor_span_ms": 250,
                    "max_consecutive_frames": 2,
                },
            }
        }

        merged = merge_alignment_policy(episode_config, policy_config)

        self.assertEqual(merged["fps"], 15)
        self.assertEqual(merged["offline_alignment"]["camera_max_delta_ms"], 30)
        self.assertTrue(merged["offline_alignment"]["historical_only"])
        self.assertTrue(merged["offline_alignment"]["repair"]["enabled"])
        self.assertEqual(
            merged["offline_alignment"]["repair"]["max_neighbor_span_ms"], 250
        )
        self.assertEqual(
            merged["offline_alignment"]["repair"]["max_consecutive_frames"], 2
        )
        self.assertTrue(merged["offline_alignment"]["repair"]["historical_only"])
        self.assertEqual(
            episode_config["offline_alignment"]["repair"]["max_consecutive_frames"], 1
        )

    def test_nearest_record_checks_both_sides(self) -> None:
        records = [{"id": 0}, {"id": 1}, {"id": 2}]
        record, delta = nearest_record(18, records, [10, 20, 30])
        self.assertEqual(record, {"id": 1})
        self.assertEqual(delta, 0.000002)

    def test_paired_segment_indices_follow_offline_timestamps(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            episode = Path(temporary)
            (episode / "paired_segments.json").write_text(
                json.dumps(
                    {
                        "segments": [
                            {
                                "skill_id": "S1",
                                "task": "combined grasp and base-retreat hold",
                                "prompt_version": "v2",
                                "start_ns": 100,
                                "end_ns": 300,
                                "start_inclusive": True,
                                "end_inclusive": False,
                                "source_start_frame": 4,
                                "source_end_frame": 8,
                                "frame_count": 5,
                            },
                            {
                                "skill_id": "S2",
                                "task": "release and push the door with the hand back",
                                "prompt_version": "v3",
                                "start_ns": 300,
                                "end_ns": 400,
                                "start_inclusive": True,
                                "end_inclusive": True,
                                "source_start_frame": 9,
                                "source_end_frame": 12,
                                "frame_count": 4,
                            },
                        ]
                    }
                ),
                encoding="utf-8",
            )
            frames = [
                {"frame_index": index, "timestamp_ns": timestamp}
                for index, timestamp in enumerate((100, 200, 300, 400))
            ]

            summary = refresh_paired_segment_indices(episode, frames)

            self.assertTrue(summary["updated"])
            document = json.loads((episode / "paired_segments.json").read_text())
            self.assertEqual(document["source_frame_index_basis"], "offline_aligned_frames_v1")
            self.assertEqual(document["offline_aligned_source_frame_count"], 4)
            first, second = document["segments"]
            self.assertEqual(
                (first["source_start_frame"], first["source_end_frame"], first["frame_count"]),
                (0, 1, 2),
            )
            self.assertEqual(
                (second["source_start_frame"], second["source_end_frame"], second["frame_count"]),
                (2, 3, 2),
            )
            self.assertEqual(first["online_source_start_frame"], 4)
            self.assertEqual(second["online_source_end_frame"], 12)
            self.assertEqual([frame["skill_id"] for frame in frames], ["S1", "S1", "S2", "S2"])
            self.assertEqual(
                [frame["prompt_version"] for frame in frames],
                ["v2", "v2", "v3", "v3"],
            )
            self.assertEqual(
                [frame["task"] for frame in frames],
                [
                    "combined grasp and base-retreat hold",
                    "combined grasp and base-retreat hold",
                    "release and push the door with the hand back",
                    "release and push the door with the hand back",
                ],
            )
            self.assertEqual(summary["covered_frame_count"], 4)
            self.assertEqual(summary["relabeled_frame_count"], 4)

    def test_paired_segments_reject_an_unassigned_frame(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            episode = Path(temporary)
            (episode / "paired_segments.json").write_text(
                json.dumps(
                    {
                        "segments": [
                            {
                                "skill_id": "S1",
                                "task": "combined skill",
                                "prompt_version": "v2",
                                "start_ns": 100,
                                "end_ns": 200,
                                "start_inclusive": True,
                                "end_inclusive": False,
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )
            frames = [
                {"frame_index": 0, "timestamp_ns": 100},
                {"frame_index": 1, "timestamp_ns": 200},
            ]

            with self.assertRaisesRegex(ValueError, "do not cover every"):
                refresh_paired_segment_indices(episode, frames)

    def test_isolated_arm_and_hand_gaps_are_repaired(self) -> None:
        frames = []
        for index in range(5):
            valid = index not in (1, 3)
            frames.append(
                {
                    "frame_index": index,
                    "timestamp_ns": 1_800_000_000_000_000_000 + index * 66_666_667,
                    "observation": {
                        "state": {
                            "arm_position": [float(index)] * 14,
                            "arm_velocity": [float(index)] * 14,
                            "arm_effort": [float(index)] * 14,
                            "hand_position": [float(index)] * 12,
                        }
                    },
                    "action": {
                        "arm_position": [float(index)] * 14,
                        "hand_position": [float(index)] * 12,
                    },
                    "source": {
                        "arm_action": "controller_command",
                        "hand_state": "hardware_feedback",
                    },
                    "alignment_delta_ms": {
                        "arm": 2.0,
                        "arm_command_left": 35.0 if index == 3 else 2.0,
                        "arm_command_right": 34.0 if index == 3 else 2.0,
                        "hand": 38.0 if index == 1 else 2.0,
                    },
                    "validity": {
                        "valid_for_training": valid,
                        "arm_aligned": index != 3,
                        "hand_aligned": index != 1,
                        "cameras_aligned": {"left": True, "right": True},
                        "wrist_pair_valid": True,
                    },
                }
            )

        summary = repair_isolated_frames(
            frames,
            {
                "enabled": True,
                "max_delta_ms": 50,
                "max_neighbor_span_ms": 170,
                "arm_state_endpoint_max_delta_rad": 3.0,
                "arm_action_endpoint_max_delta_rad": 3.0,
                "hand_state_o6_endpoint_max_delta": 3.0,
                "hand_state_wuji_endpoint_max_delta_rad": 3.0,
                "hand_action_o6_endpoint_max_delta": 3.0,
                "hand_action_wuji_endpoint_max_delta_rad": 3.0,
            },
            output_fps=15,
            arm_limit_ms=30,
            hand_limit_ms=30,
        )
        self.assertEqual(summary["repaired_frame_count"], 2)
        self.assertEqual(summary["repaired_by_source"]["arm"], 1)
        self.assertEqual(summary["repaired_by_source"]["hand"], 1)
        self.assertTrue(all(frame["validity"]["valid_for_training"] for frame in frames))
        self.assertEqual(frames[1]["validity"]["repair_sources"], ["hand"])
        self.assertEqual(frames[3]["validity"]["repair_sources"], ["arm"])
        self.assertAlmostEqual(frames[1]["action"]["hand_position"][0], 1.0)
        self.assertAlmostEqual(frames[3]["action"]["arm_position"][0], 3.0)

    def test_camera_and_hand_repairs_do_not_gate_each_other(self) -> None:
        frames = []
        for index in range(4):
            frames.append(
                {
                    "frame_index": index,
                    "timestamp_ns": 1_800_000_000_000_000_000 + index * 66_666_667,
                    "observation": {
                        "images": {
                            "left": {
                                "path": f"left/{index}.jpg",
                                "sync_timestamp_ns": 1_800_000_000_000_000_000
                                + index * 66_666_667,
                            }
                        },
                        "state": {"hand_position": [0.0] * 12},
                    },
                    "action": {"hand_position": [0.0] * 12},
                    "source": {"hand_state": "hardware_feedback"},
                    "alignment_delta_ms": {
                        "cameras": {"left": 40.0 if index == 1 else 0.0},
                        "hand": 40.0,
                    },
                    "validity": {
                        "valid_for_training": index in (0, 3),
                        "arm_aligned": True,
                        "hand_aligned": index in (0, 3),
                        "cameras_aligned": {"left": index != 1},
                        "wrist_pair_valid": True,
                    },
                }
            )
        summary = repair_isolated_frames(
            frames,
            {"enabled": True, "max_delta_ms": 50, "max_neighbor_span_ms": 170},
            output_fps=15,
            arm_limit_ms=30,
            hand_limit_ms=30,
        )
        self.assertEqual(summary["repaired_frame_count"], 1)
        self.assertEqual(summary["repaired_by_source"]["camera"], 1)
        self.assertTrue(frames[1]["validity"]["cameras_aligned"]["left"])
        self.assertFalse(frames[1]["validity"]["valid_for_training"])
        self.assertFalse(frames[2]["validity"]["valid_for_training"])

    def test_two_frame_hand_gap_is_repaired(self) -> None:
        frames = []
        for index in range(5):
            valid = index not in (1, 2)
            frames.append(
                {
                    "frame_index": index,
                    "timestamp_ns": 1_800_000_000_000_000_000 + index * 66_666_667,
                    "observation": {"state": {"hand_position": [float(index)] * 12}},
                    "action": {"hand_position": [float(index)] * 12},
                    "source": {"hand_state": "hardware_feedback"},
                    "alignment_delta_ms": {
                        "hand": 40.0 if not valid else 2.0,
                        "hand_action": 38.0 if not valid else 2.0,
                        "hand_state": 40.0 if not valid else 2.0,
                    },
                    "validity": {
                        "valid_for_training": valid,
                        "arm_aligned": True,
                        "hand_aligned": valid,
                        "cameras_aligned": {"left": True, "right": True},
                        "wrist_pair_valid": True,
                    },
                }
            )
        summary = repair_isolated_frames(
            frames,
            {
                "enabled": True,
                "max_delta_ms": 50,
                "max_neighbor_span_ms": 250,
                "max_consecutive_frames": 2,
                "hand_state_o6_endpoint_max_delta": 4.0,
                "hand_state_wuji_endpoint_max_delta_rad": 4.0,
                "hand_action_o6_endpoint_max_delta": 4.0,
                "hand_action_wuji_endpoint_max_delta_rad": 4.0,
            },
            output_fps=15,
            arm_limit_ms=30,
            hand_limit_ms=30,
        )
        self.assertEqual(summary["repaired_frame_count"], 2)
        self.assertEqual(summary["repaired_by_source"]["arm"], 0)
        self.assertEqual(summary["repaired_by_source"]["hand"], 2)
        self.assertTrue(all(frame["validity"]["valid_for_training"] for frame in frames))
        self.assertAlmostEqual(frames[1]["observation"]["state"]["hand_position"][0], 1.0)
        self.assertAlmostEqual(frames[2]["action"]["hand_position"][0], 2.0)

    def _bounded_repair_frame(
        self,
        index: int,
        *,
        arm_valid: bool = True,
        hand_valid: bool = True,
        value: float = 0.0,
    ) -> dict:
        timestamp_ns = 1_800_000_000_000_000_000 + index * 66_666_667
        images = {
            name: {"path": f"{name}/{index}.jpg", "sync_timestamp_ns": timestamp_ns}
            for name in ("camera_top", "camera_wrist_left", "camera_wrist_right")
        }
        return {
            "frame_index": index,
            "timestamp_ns": timestamp_ns,
            "observation": {
                "images": images,
                "state": {
                    "arm_position": [value] * 14,
                    "arm_velocity": [value] * 14,
                    "arm_effort": [value] * 14,
                    "hand_position": [value] * 12,
                    "base_velocity": [0.0] * 3,
                },
            },
            "action": {
                "arm_position": [value] * 14,
                "hand_position": [value] * 12,
                "base_velocity": [0.0] * 3,
            },
            "source": {
                "arm_action": "controller_command",
                "hand_state": "hardware_feedback",
            },
            "source_timestamps_ns": {},
            "alignment_delta_ms": {
                "cameras": {name: 0.0 for name in images},
                "arm": 120.0 if not arm_valid else 0.0,
                "arm_command_left": 120.0 if not arm_valid else 0.0,
                "arm_command_right": 120.0 if not arm_valid else 0.0,
                "hand": 120.0 if not hand_valid else 0.0,
                "hand_action": 120.0 if not hand_valid else 0.0,
                "hand_state": 120.0 if not hand_valid else 0.0,
            },
            "validity": {
                "valid_for_training": arm_valid and hand_valid,
                "offline_aligned": True,
                "arm_aligned": arm_valid,
                "hand_aligned": hand_valid,
                "cameras_aligned": {name: True for name in images},
                "wrist_pair_valid": True,
                "wrist_pair_skew_ms": 0.0,
            },
            "raw": {},
        }

    def test_four_frame_constant_hand_gap_is_repaired_with_provenance(self) -> None:
        frames = [
            self._bounded_repair_frame(
                index,
                hand_valid=index not in range(1, 5),
                value=0.25,
            )
            for index in range(6)
        ]
        summary = repair_isolated_frames(
            frames,
            {
                "enabled": True,
                "max_delta_ms": 250,
                "max_neighbor_span_ms": 450,
                "max_consecutive_frames": 5,
            },
            output_fps=15,
            arm_limit_ms=30,
            hand_limit_ms=30,
        )
        self.assertEqual(summary["repaired_by_source"]["hand"], 4)
        self.assertTrue(all(frame["validity"]["valid_for_training"] for frame in frames))
        self.assertEqual(
            frames[2]["repair"]["sources"]["hand"]["method"],
            "bounded_component_linear_interpolation",
        )
        self.assertEqual(
            frames[2]["repair"]["sources"]["hand"]["endpoint_max_delta"]["action"],
            {"o6": 0.0, "wuji": 0.0},
        )

    def test_hand_state_gap_preserves_an_aligned_discrete_action(self) -> None:
        frames = [self._bounded_repair_frame(index, value=0.0) for index in range(3)]
        middle = frames[1]
        middle["validity"]["valid_for_training"] = False
        middle["validity"]["hand_aligned"] = False
        middle["alignment_delta_ms"]["hand_state"] = 32.0
        middle["alignment_delta_ms"]["hand"] = 32.0
        middle["alignment_delta_ms"]["hand_action"] = 2.0
        middle["action"]["hand_position"] = [142.0] * 6 + [0.1] * 6
        original_action = list(middle["action"]["hand_position"])

        summary = repair_isolated_frames(
            frames,
            {
                "enabled": True,
                "max_delta_ms": 250,
                "hand_state_o6_endpoint_max_delta": 64.0,
                "hand_state_wuji_endpoint_max_delta_rad": 0.15,
            },
            output_fps=15,
            arm_limit_ms=30,
            hand_limit_ms=30,
        )

        self.assertEqual(summary["repaired_by_source"]["hand"], 1)
        self.assertEqual(frames[1]["action"]["hand_position"], original_action)
        self.assertEqual(
            frames[1]["repair"]["sources"]["hand"]["components"],
            ["observation.state.hand_position"],
        )

    def test_five_frame_smooth_arm_gap_is_repaired(self) -> None:
        frames = []
        for index in range(7):
            frame = self._bounded_repair_frame(
                index,
                arm_valid=index not in range(1, 6),
                value=index * 0.01,
            )
            frames.append(frame)
        summary = repair_isolated_frames(
            frames,
            {
                "enabled": True,
                "max_delta_ms": 250,
                "max_neighbor_span_ms": 450,
                "max_consecutive_frames": 5,
                "arm_state_endpoint_max_delta_rad": 0.15,
                "arm_action_endpoint_max_delta_rad": 0.15,
            },
            output_fps=15,
            arm_limit_ms=30,
            hand_limit_ms=30,
        )
        self.assertEqual(summary["repaired_by_source"]["arm"], 5)
        self.assertAlmostEqual(frames[3]["action"]["arm_position"][0], 0.03)

    def test_numeric_gap_with_discontinuous_endpoints_is_rejected(self) -> None:
        frames = [
            self._bounded_repair_frame(index, arm_valid=index not in (1, 2), value=0.0)
            for index in range(4)
        ]
        frames[-1]["observation"]["state"]["arm_position"] = [1.0] * 14
        frames[-1]["action"]["arm_position"] = [1.0] * 14
        summary = repair_isolated_frames(
            frames,
            {
                "enabled": True,
                "max_delta_ms": 250,
                "max_neighbor_span_ms": 450,
                "max_consecutive_frames": 5,
                "arm_state_endpoint_max_delta_rad": 0.15,
                "arm_action_endpoint_max_delta_rad": 0.15,
            },
            output_fps=15,
            arm_limit_ms=30,
            hand_limit_ms=30,
        )
        self.assertEqual(summary["repaired_by_source"]["arm"], 0)
        self.assertFalse(frames[1]["validity"]["valid_for_training"])

    def test_isolated_camera_mismatch_is_accepted_with_provenance(self) -> None:
        frames = [self._bounded_repair_frame(index) for index in range(3)]
        middle = frames[1]
        middle["validity"]["valid_for_training"] = False
        middle["validity"]["cameras_aligned"]["camera_top"] = False
        middle["alignment_delta_ms"]["cameras"]["camera_top"] = 60.0
        summary = repair_isolated_frames(
            frames,
            {"enabled": True, "camera_max_delta_ms": 75},
            output_fps=15,
            arm_limit_ms=30,
            hand_limit_ms=30,
        )
        self.assertEqual(summary["repaired_by_source"]["camera"], 1)
        self.assertTrue(middle is not frames[1])
        self.assertTrue(frames[1]["validity"]["valid_for_training"])
        self.assertIn("camera:camera_top", frames[1]["repair"]["sources"])

    def test_bounded_two_frame_camera_gap_is_accepted(self) -> None:
        frames = [self._bounded_repair_frame(index) for index in range(4)]
        for index, delta in ((1, 80.0), (2, 105.0)):
            frames[index]["validity"]["valid_for_training"] = False
            frames[index]["validity"]["cameras_aligned"]["camera_top"] = False
            frames[index]["alignment_delta_ms"]["cameras"]["camera_top"] = delta

        summary = repair_isolated_frames(
            frames,
            {
                "enabled": True,
                "camera_max_delta_ms": 125,
                "camera_max_consecutive_frames": 2,
            },
            output_fps=15,
            arm_limit_ms=30,
            hand_limit_ms=30,
        )

        self.assertEqual(summary["repaired_by_source"]["camera"], 2)
        self.assertTrue(all(frame["validity"]["valid_for_training"] for frame in frames))
        self.assertEqual(
            frames[1]["repair"]["sources"]["camera:camera_top"]["method"],
            "accept_bounded_nearest_raw_images",
        )
        self.assertEqual(
            frames[2]["repair"]["sources"]["camera:camera_top"]["run_length"], 2
        )

    def test_camera_gap_longer_than_policy_is_rejected(self) -> None:
        frames = [self._bounded_repair_frame(index) for index in range(5)]
        for index in (1, 2, 3):
            frames[index]["validity"]["valid_for_training"] = False
            frames[index]["validity"]["cameras_aligned"]["camera_top"] = False
            frames[index]["alignment_delta_ms"]["cameras"]["camera_top"] = 80.0

        summary = repair_isolated_frames(
            frames,
            {
                "enabled": True,
                "camera_max_delta_ms": 125,
                "camera_max_consecutive_frames": 2,
            },
            output_fps=15,
            arm_limit_ms=30,
            hand_limit_ms=30,
        )

        self.assertEqual(summary["repaired_by_source"]["camera"], 0)
        self.assertFalse(frames[2]["validity"]["valid_for_training"])

    def test_camera_gap_can_reuse_a_bounded_neighbor_image(self) -> None:
        frames = [self._bounded_repair_frame(index) for index in range(3)]
        middle = frames[1]
        middle["validity"]["valid_for_training"] = False
        middle["validity"]["cameras_aligned"]["camera_top"] = False
        middle["alignment_delta_ms"]["cameras"]["camera_top"] = 160.0

        summary = repair_isolated_frames(
            frames,
            {
                "enabled": True,
                "camera_max_delta_ms": 75,
                "camera_max_consecutive_frames": 1,
            },
            output_fps=15,
            arm_limit_ms=30,
            hand_limit_ms=30,
        )

        self.assertEqual(summary["repaired_by_source"]["camera"], 1)
        self.assertTrue(frames[1]["validity"]["valid_for_training"])
        self.assertEqual(
            frames[1]["repair"]["sources"]["camera:camera_top"]["method"],
            "reuse_bounded_neighbor_image",
        )
        self.assertTrue(frames[1]["observation"]["images"]["camera_top"]["path"].endswith("/0.jpg"))

    def test_wrist_pair_gap_reuses_one_synchronized_neighbor_pair(self) -> None:
        frames = [self._bounded_repair_frame(index) for index in range(3)]
        middle = frames[1]
        middle["validity"]["valid_for_training"] = False
        middle["validity"]["cameras_aligned"]["camera_wrist_right"] = False
        middle["validity"]["wrist_pair_valid"] = False
        middle["validity"]["wrist_pair_skew_ms"] = 34.0
        middle["alignment_delta_ms"]["cameras"]["camera_wrist_right"] = 34.0

        summary = repair_isolated_frames(
            frames,
            {
                "enabled": True,
                "camera_max_delta_ms": 125,
                "camera_max_consecutive_frames": 1,
                "wrist_pair_max_consecutive_frames": 1,
                "wrist_pair_camera_max_delta_ms": 125,
            },
            output_fps=15,
            arm_limit_ms=30,
            hand_limit_ms=30,
        )

        self.assertEqual(summary["repaired_by_source"]["wrist_pair"], 1)
        self.assertTrue(frames[1]["validity"]["valid_for_training"])
        self.assertEqual(frames[1]["validity"]["wrist_pair_skew_ms"], 0.0)
        self.assertEqual(
            frames[1]["repair"]["sources"]["wrist_pair"]["method"],
            "reuse_bounded_synchronized_neighbor_pair",
        )

    def test_single_missing_15hz_timestep_is_restored_without_retiming(self) -> None:
        frames = [self._bounded_repair_frame(index) for index in (0, 2)]
        frames[1]["frame_index"] = 1
        summary = repair_isolated_frames(
            frames,
            {
                "enabled": True,
                "camera_max_delta_ms": 75,
                "max_missing_timing_frames": 1,
                "max_timing_gap_ms": 160,
            },
            output_fps=15,
            arm_limit_ms=30,
            hand_limit_ms=30,
        )
        self.assertEqual(summary["inserted_frame_count"], 1)
        self.assertEqual(len(frames), 3)
        self.assertEqual(frames[1]["timestamp_ns"] - frames[0]["timestamp_ns"], 66_666_667)
        self.assertEqual(frames[1]["validity"]["repair_sources"], ["timing"])
        self.assertEqual([frame["frame_index"] for frame in frames], [0, 1, 2])
        self.assertTrue(
            all(
                image["path"].endswith("/0.jpg")
                for image in frames[1]["observation"]["images"].values()
            )
        )

    def test_longer_unbounded_numeric_gap_is_rejected(self) -> None:
        frames = [
            self._bounded_repair_frame(index, hand_valid=index not in range(1, 7))
            for index in range(8)
        ]
        summary = repair_isolated_frames(
            frames,
            {"enabled": True, "max_consecutive_frames": 5, "max_neighbor_span_ms": 450},
            output_fps=15,
            arm_limit_ms=30,
            hand_limit_ms=30,
        )
        self.assertEqual(summary["repaired_by_source"]["hand"], 0)
        self.assertFalse(frames[3]["validity"]["valid_for_training"])

    def test_alignment_rejects_a_49ms_wrist_pair(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            episode = Path(temporary)
            base = 1_800_000_000_000_000_000
            step = 100_000_000
            camera_names = ["camera_top", "camera_wrist_left", "camera_wrist_right"]
            config = {
                "fps": 10,
                "cameras": {
                    "camera_top": {"enabled": True, "fps": 10},
                    "camera_wrist_left": {"enabled": True, "fps": 10},
                    "camera_wrist_right": {"enabled": True, "fps": 10},
                },
                "ros": {"arms": {"joint_names": [f"j{i}" for i in range(14)]}},
                "hands": {"right_virtual_projection": PROJECTION},
                "timing": {
                    "wrist_pair": {
                        "left_camera": "camera_wrist_left",
                        "right_camera": "camera_wrist_right",
                        "max_skew_ms": 30,
                    }
                },
                "offline_alignment": {
                    "master_camera": "camera_wrist_left",
                    "output_fps": 10,
                    "camera_max_delta_ms": 30,
                    "arm_max_delta_ms": 30,
                    "hand_max_delta_ms": 30,
                    "base_max_delta_ms": 100,
                },
            }
            (episode / "config.yaml").write_text("unused", encoding="utf-8")
            (episode / "manifest.json").write_text(
                json.dumps(
                    {
                        "status": "complete",
                        "task": "test task",
                        "skill_id": "S3",
                        "prompt_version": "v1",
                        "recording_start_ns": base,
                        "recording_end_ns": base + 2 * step,
                    }
                ),
                encoding="utf-8",
            )
            (episode / "frames.jsonl").write_text("online\n", encoding="utf-8")

            def write_jsonl(path: Path, values: list[dict]) -> None:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(
                    "".join(json.dumps(value) + "\n" for value in values), encoding="utf-8"
                )

            camera_offsets = {
                "camera_wrist_left": [0, 0],
                "camera_top": [5_000_000, 5_000_000],
                "camera_wrist_right": [10_000_000, 49_000_000],
            }
            for camera in camera_names:
                values = []
                for index, timestamp in enumerate(
                    [base - step, base, base + step, base + 2 * step]
                ):
                    offset = 0
                    if index in (1, 2):
                        offset = camera_offsets[camera][index - 1]
                    path = f"raw/cameras/{camera}/images/{index}.jpg"
                    absolute = episode / path
                    absolute.parent.mkdir(parents=True, exist_ok=True)
                    absolute.write_bytes(b"jpeg")
                    timestamp += offset
                    values.append(
                        {
                            "camera": camera,
                            "path": path,
                            "sequence": index,
                            "host_timestamp_ns": timestamp,
                            "hardware_timestamp_ns": timestamp,
                            "hardware_timestamp_domain": "timestamp_domain.global_time",
                            "width": 2,
                            "height": 2,
                        }
                    )
                write_jsonl(
                    episode / "raw" / "cameras" / camera / "timestamps.jsonl", values
                )

            joint_names = [f"j{i}" for i in range(14)]
            state_values = []
            command_left = []
            command_right = []
            hand_values = []
            for timestamp in [base - step, base, base + step, base + 2 * step]:
                state_values.append(
                    {
                        "received_ns": timestamp + 1_000_000,
                        "timestamp_ns": timestamp,
                        "name": joint_names,
                        "position": list(range(14)),
                        "velocity": [0.0] * 14,
                        "effort": [0.0] * 14,
                    }
                )
                command_left.append(
                    {"received_ns": timestamp + 2_000_000, "value": list(range(7)) + [70]}
                )
                command_right.append(
                    {"received_ns": timestamp + 2_000_000, "value": list(range(7, 14)) + [140]}
                )
                hand_values.append(
                    {
                        "_timestamp_ns": timestamp + 3_000_000,
                        "_actual_state_timestamp_ns": timestamp + 4_000_000,
                        "left_o6_command": list(range(6)),
                        "right_wuji_command": list(range(20)),
                        "left_o6_actual": list(range(6)),
                        "right_wuji_actual": list(range(20)),
                    }
                )
            raw_ros = episode / "raw" / "ros"
            write_jsonl(raw_ros / "arm_joint_state.jsonl", state_values)
            write_jsonl(raw_ros / "arm_command_left.jsonl", command_left)
            write_jsonl(raw_ros / "arm_command_right.jsonl", command_right)
            write_jsonl(episode / "raw" / "hands" / "teleop.jsonl", hand_values)

            report = align_episode(episode, config)
            self.assertEqual(report["frame_count"], 3)
            self.assertEqual(report["valid_frame_count"], 2)
            self.assertEqual(report["timeline_start_ns"], base)
            self.assertEqual(report["continuity"]["internal_invalid_frame_count"], 1)
            self.assertFalse(report["continuity"]["complete_episode_candidate"])
            self.assertFalse((episode / "lerobot_frames.jsonl").exists())
            frames = [json.loads(line) for line in (episode / "frames.jsonl").read_text().splitlines()]
            self.assertEqual(frames[0]["timestamp_ns"], base)
            self.assertEqual(frames[0]["skill_id"], "S3")
            self.assertEqual(frames[0]["prompt_version"], "v1")
            self.assertTrue(frames[0]["validity"]["valid_for_training"])
            self.assertEqual(frames[0]["action"]["arm_position"], list(range(14)))
            self.assertFalse(frames[1]["validity"]["valid_for_training"])
            self.assertAlmostEqual(frames[1]["validity"]["wrist_pair_skew_ms"], 49.0)
            self.assertTrue((episode / "online_frames.jsonl").is_file())


if __name__ == "__main__":
    unittest.main()
