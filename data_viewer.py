#!/usr/bin/env python3
"""Read-only web viewer for aligned SVT VLA collection episodes."""

from __future__ import annotations

import argparse
import bisect
import json
import math
import mimetypes
import re
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse


SCRIPT_DIR = Path(__file__).resolve().parent
STATIC_DIR = SCRIPT_DIR / "viewer"
DATE_PATTERN = re.compile(r"^\d{4}-\d{2}-\d{2}$")
CAMERA_LABELS = {
    "camera_top": "顶部相机",
    "camera_wrist_left": "左腕相机",
    "camera_wrist_right": "右腕相机",
}


def load_json(path: Path, default: Any = None) -> Any:
    if not path.is_file():
        return default
    return json.loads(path.read_text(encoding="utf-8"))


def vector_step(before: Any, after: Any, start: int, end: int) -> float:
    if not isinstance(before, list) or not isinstance(after, list):
        return 0.0
    if len(before) < end or len(after) < end:
        return 0.0
    return math.sqrt(
        sum((float(after[index]) - float(before[index])) ** 2 for index in range(start, end))
    )


class DataRepository:
    def __init__(self, root: Path) -> None:
        self.root = root.expanduser().resolve()
        self._cache: dict[Path, tuple[int, int, list[dict[str, Any]]]] = {}
        self._lock = threading.Lock()

    def episode_dir(self, day: str, episode: str) -> Path:
        if not DATE_PATTERN.fullmatch(day):
            raise ValueError("invalid dataset day")
        if not episode or episode in {".", ".."} or "/" in episode or "\\" in episode:
            raise ValueError("invalid episode name")
        path = (self.root / day / episode).resolve()
        try:
            path.relative_to(self.root)
        except ValueError as exc:
            raise ValueError("episode path escapes the dataset root") from exc
        if not path.is_dir():
            raise FileNotFoundError(f"episode not found: {day}/{episode}")
        return path

    def frames(self, episode_dir: Path) -> list[dict[str, Any]]:
        path = episode_dir / "frames.jsonl"
        if not path.is_file():
            raise FileNotFoundError(f"frames.jsonl not found in {episode_dir.name}")
        stat = path.stat()
        with self._lock:
            cached = self._cache.get(path)
            if cached and cached[0] == stat.st_mtime_ns and cached[1] == stat.st_size:
                return cached[2]
        values: list[dict[str, Any]] = []
        with path.open("r", encoding="utf-8", errors="replace") as stream:
            for line in stream:
                if not line.strip():
                    continue
                try:
                    value = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(value, dict):
                    values.append(value)
        with self._lock:
            self._cache[path] = (stat.st_mtime_ns, stat.st_size, values)
        return values

    def segments(self, episode_dir: Path, manifest: dict[str, Any]) -> tuple[list[dict[str, Any]], dict[str, Any]]:
        document = load_json(episode_dir / "paired_segments.json", {}) or {}
        raw_segments = document.get("segments")
        if isinstance(raw_segments, list) and raw_segments:
            segments = []
            for index, segment in enumerate(raw_segments):
                if not isinstance(segment, dict):
                    continue
                start = int(segment.get("source_start_frame", 0))
                end = int(segment.get("source_end_frame", start))
                segments.append(
                    {
                        "index": index,
                        "skillId": str(segment.get("skill_id") or "未标注"),
                        "promptVersion": str(segment.get("prompt_version") or "-"),
                        "task": str(segment.get("task") or "未标注 Prompt"),
                        "startFrame": start,
                        "endFrame": end,
                        "frameCount": int(segment.get("frame_count", end - start + 1)),
                        "startNs": int(segment.get("start_ns", 0)),
                        "endNs": int(segment.get("end_ns", 0)),
                        "baseMotion": segment.get("base_motion") or {},
                    }
                )
            return segments, document

        frame_count = int(manifest.get("frame_count") or 0)
        start_ns = int(manifest.get("recording_start_ns") or 0)
        end_ns = int(manifest.get("recording_end_ns") or start_ns)
        return [
            {
                "index": 0,
                "skillId": str(manifest.get("skill_id") or "未标注"),
                "promptVersion": str(manifest.get("prompt_version") or "-"),
                "task": str(manifest.get("task") or "未标注 Prompt"),
                "startFrame": 0,
                "endFrame": max(0, frame_count - 1),
                "frameCount": frame_count,
                "startNs": start_ns,
                "endNs": end_ns,
                "baseMotion": manifest.get("base_motion") or {},
            }
        ], {}

    @staticmethod
    def _duration_seconds(manifest: dict[str, Any], segments: list[dict[str, Any]]) -> float:
        start_ns = manifest.get("recording_start_ns")
        end_ns = manifest.get("recording_end_ns")
        if isinstance(start_ns, int) and isinstance(end_ns, int) and end_ns >= start_ns:
            return (end_ns - start_ns) / 1_000_000_000.0
        if segments:
            return max(0.0, (segments[-1]["endNs"] - segments[0]["startNs"]) / 1_000_000_000.0)
        return 0.0

    def episode_summary(self, day: str, episode_dir: Path) -> dict[str, Any]:
        manifest = load_json(episode_dir / "manifest.json", {}) or {}
        report = load_json(episode_dir / "alignment_report.json", {}) or {}
        segments, segment_document = self.segments(episode_dir, manifest)
        skill_ids = list(
            dict.fromkeys(
                str(segment.get("skillId") or "unknown") for segment in segments
            )
        )
        workflow = segment_document.get("workflow") or manifest.get("workflow")
        if not workflow and skill_ids:
            workflow = f"atomic_{'_'.join(skill_id.lower() for skill_id in skill_ids)}"
        frame_count_value = manifest.get("frame_count")
        if frame_count_value is None:
            frame_count_value = report.get("frame_count", 0)
        frame_count = int(frame_count_value or 0)
        valid_count_value = manifest.get("valid_frame_count")
        if valid_count_value is None:
            valid_count_value = report.get("valid_frame_count", frame_count)
        valid_count = int(valid_count_value or 0)
        repair = report.get("repair") or {}
        continuity = report.get("continuity") or {}
        return {
            "day": day,
            "name": episode_dir.name,
            "shortName": episode_dir.name[:6],
            "frameCount": frame_count,
            "validFrameCount": valid_count,
            "validRate": valid_count / frame_count if frame_count else 0.0,
            "durationSeconds": self._duration_seconds(manifest, segments),
            "fps": float(manifest.get("fps") or report.get("output_fps") or 15),
            "status": str(manifest.get("status") or "unknown"),
            "paired": bool(segment_document),
            "episodeType": "continuous" if segment_document else "atomic",
            "workflow": workflow,
            "skillIds": skill_ids,
            "segmentSource": (
                "paired_segments.json" if segment_document else "manifest.json"
            ),
            "completeCandidate": bool(continuity.get("complete_episode_candidate")),
            "repairCount": int(repair.get("repaired_frame_count") or 0),
            "timingGapCount": int(continuity.get("timing_gap_count") or 0),
            "segments": segments,
        }

    def index(self) -> dict[str, Any]:
        days: list[dict[str, Any]] = []
        episode_count = 0
        if not self.root.is_dir():
            return {"root": str(self.root), "days": [], "episodeCount": 0}
        for day_dir in sorted(self.root.iterdir(), reverse=True):
            if not day_dir.is_dir() or not DATE_PATTERN.fullmatch(day_dir.name):
                continue
            episodes = []
            for episode_dir in sorted(day_dir.iterdir(), reverse=True):
                if not episode_dir.is_dir() or not (episode_dir / "frames.jsonl").is_file():
                    continue
                try:
                    episodes.append(self.episode_summary(day_dir.name, episode_dir))
                except (OSError, ValueError, json.JSONDecodeError):
                    continue
            if episodes:
                days.append({"day": day_dir.name, "episodes": episodes})
                episode_count += len(episodes)
        return {"root": str(self.root), "days": days, "episodeCount": episode_count}

    @staticmethod
    def _marker_frame(timestamps: list[int], timestamp_ns: Any) -> int | None:
        if not isinstance(timestamp_ns, int) or not timestamps:
            return None
        return min(bisect.bisect_left(timestamps, timestamp_ns), len(timestamps) - 1)

    def detail(self, day: str, episode: str) -> dict[str, Any]:
        episode_dir = self.episode_dir(day, episode)
        manifest = load_json(episode_dir / "manifest.json", {}) or {}
        report = load_json(episode_dir / "alignment_report.json", {}) or {}
        segments, segment_document = self.segments(episode_dir, manifest)
        frames = self.frames(episode_dir)
        timestamps = [int(frame.get("timestamp_ns") or 0) for frame in frames]
        start_ns = timestamps[0] if timestamps else 0
        compact_frames = []
        previous: dict[str, Any] | None = None
        camera_names: list[str] = []
        for frame in frames:
            state = ((frame.get("observation") or {}).get("state") or {})
            action = frame.get("action") or {}
            images = ((frame.get("observation") or {}).get("images") or {})
            if not camera_names:
                camera_names = list(images)
            previous_action = (previous or {}).get("action") or {}
            arm = action.get("arm_position")
            previous_arm = previous_action.get("arm_position")
            hand = action.get("hand_position")
            previous_hand = previous_action.get("hand_position")
            base = action.get("base_velocity")
            base_speed = (
                math.sqrt(sum(float(value) ** 2 for value in base))
                if isinstance(base, list) and len(base) == 3
                else 0.0
            )
            validity = frame.get("validity") or {}
            compact_frames.append(
                {
                    "index": int(frame.get("frame_index") or 0),
                    "timestampNs": int(frame.get("timestamp_ns") or 0),
                    "relativeSeconds": (int(frame.get("timestamp_ns") or 0) - start_ns)
                    / 1_000_000_000.0,
                    "skillId": str(frame.get("skill_id") or manifest.get("skill_id") or "未标注"),
                    "promptVersion": str(
                        frame.get("prompt_version")
                        or manifest.get("prompt_version")
                        or "-"
                    ),
                    "task": str(frame.get("task") or manifest.get("task") or "未标注 Prompt"),
                    "repaired": bool(validity.get("repaired")),
                    "repairSources": list(validity.get("repair_sources") or []),
                    "valid": bool(validity.get("valid_for_training")),
                    "baseActive": base_speed > 1e-4,
                    "baseSpeed": base_speed,
                    "leftArmStep": vector_step(previous_arm, arm, 0, 7),
                    "rightArmStep": vector_step(previous_arm, arm, 7, 14),
                    "leftHandStep": vector_step(previous_hand, hand, 0, 6),
                    "cameras": {name: bool(value and value.get("path")) for name, value in images.items()},
                    "armStateAvailable": isinstance(state.get("arm_position"), list),
                }
            )
            previous = frame

        phase_markers = segment_document.get("phase_markers") or {}
        marker_frames = {
            key: {
                "timestampNs": value,
                "frame": self._marker_frame(timestamps, value),
            }
            for key, value in phase_markers.items()
            if isinstance(value, int)
        }
        repair = report.get("repair") or {}
        continuity = report.get("continuity") or {}
        return {
            "summary": self.episode_summary(day, episode_dir),
            "manifest": {
                "task": manifest.get("task"),
                "skillId": manifest.get("skill_id"),
                "promptVersion": manifest.get("prompt_version"),
                "attemptOutcome": manifest.get("attempt_outcome"),
                "perturbation": manifest.get("perturbation"),
                "operator": manifest.get("operator"),
            },
            "segments": segments,
            "phaseMarkers": marker_frames,
            "frames": compact_frames,
            "cameraNames": [
                {"name": name, "label": CAMERA_LABELS.get(name, name)} for name in camera_names
            ],
            "quality": {
                "repairCount": int(repair.get("repaired_frame_count") or 0),
                "repairBySource": repair.get("repaired_by_source") or {},
                "internalInvalidCount": int(continuity.get("internal_invalid_frame_count") or 0),
                "timingGapCount": int(continuity.get("timing_gap_count") or 0),
                "completeCandidate": bool(continuity.get("complete_episode_candidate")),
            },
        }

    def image_path(self, day: str, episode: str, frame_index: int, camera: str) -> Path:
        episode_dir = self.episode_dir(day, episode)
        frames = self.frames(episode_dir)
        if frame_index < 0 or frame_index >= len(frames):
            raise IndexError("frame index is outside the episode")
        frame = frames[frame_index]
        if int(frame.get("frame_index", frame_index)) != frame_index:
            frame = next(
                (
                    candidate
                    for candidate in frames
                    if int(candidate.get("frame_index", -1)) == frame_index
                ),
                None,
            )
            if frame is None:
                raise IndexError("frame index is outside the episode")
        images = ((frame.get("observation") or {}).get("images") or {})
        image = images.get(camera)
        if not isinstance(image, dict) or not image.get("path"):
            raise FileNotFoundError(f"camera image is missing: {camera}")
        path = (episode_dir / str(image["path"])).resolve()
        try:
            path.relative_to(episode_dir.resolve())
        except ValueError as exc:
            raise ValueError("camera path escapes the episode") from exc
        if not path.is_file():
            raise FileNotFoundError(f"camera image does not exist: {path.name}")
        return path


class ViewerHandler(BaseHTTPRequestHandler):
    repository: DataRepository
    static_dir: Path = STATIC_DIR

    def log_message(self, fmt: str, *args: Any) -> None:
        print(f"[data-viewer] {self.address_string()} {fmt % args}")

    def _headers(self, status: int, content_type: str, length: int, cache: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(length))
        self.send_header("Cache-Control", cache)
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; img-src 'self' data:; style-src 'self'; script-src 'self'",
        )
        self.end_headers()

    def send_json(self, value: Any, status: int = HTTPStatus.OK) -> None:
        body = json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self._headers(status, "application/json; charset=utf-8", len(body), "no-store")
        self.wfile.write(body)

    def send_file(self, path: Path, cache: str) -> None:
        body = path.read_bytes()
        content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        self._headers(HTTPStatus.OK, content_type, len(body), cache)
        self.wfile.write(body)

    def send_error_json(self, status: int, message: str) -> None:
        self.send_json({"error": message}, status)

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query)
        try:
            if parsed.path == "/api/health":
                self.send_json({"status": "ok", "root": str(self.repository.root)})
                return
            if parsed.path == "/api/index":
                self.send_json(self.repository.index())
                return
            if parsed.path == "/api/episode":
                self.send_json(
                    self.repository.detail(
                        query.get("day", [""])[0], query.get("episode", [""])[0]
                    )
                )
                return
            if parsed.path == "/api/image":
                path = self.repository.image_path(
                    query.get("day", [""])[0],
                    query.get("episode", [""])[0],
                    int(query.get("frame", ["-1"])[0]),
                    query.get("camera", [""])[0],
                )
                self.send_file(path, "private, max-age=31536000, immutable")
                return
            relative = "index.html" if parsed.path == "/" else parsed.path.lstrip("/")
            path = (self.static_dir / relative).resolve()
            try:
                path.relative_to(self.static_dir.resolve())
            except ValueError as exc:
                raise FileNotFoundError(relative) from exc
            if not path.is_file():
                raise FileNotFoundError(relative)
            self.send_file(path, "no-cache")
        except (FileNotFoundError, IndexError) as exc:
            self.send_error_json(HTTPStatus.NOT_FOUND, str(exc))
        except (ValueError, json.JSONDecodeError) as exc:
            self.send_error_json(HTTPStatus.BAD_REQUEST, str(exc))
        except OSError as exc:
            self.send_error_json(HTTPStatus.INTERNAL_SERVER_ERROR, str(exc))


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Serve the read-only SVT data viewer.")
    parser.add_argument(
        "--data-root", type=Path, default=Path("/svtrobo_data/vla_data")
    )
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    return parser


def main() -> int:
    args = build_arg_parser().parse_args()
    if not STATIC_DIR.is_dir():
        raise RuntimeError(f"viewer assets are missing: {STATIC_DIR}")
    ViewerHandler.repository = DataRepository(args.data_root)
    server = ThreadingHTTPServer((args.host, args.port), ViewerHandler)
    print(
        f"SVT data viewer listening on http://{args.host}:{args.port} "
        f"for {ViewerHandler.repository.root}"
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
