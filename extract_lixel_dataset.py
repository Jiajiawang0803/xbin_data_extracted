"""Export a complete Lixel XBIN sensor dataset with original timestamps.

Outputs created by the vendor decoder:
  calib/config/*.yaml       camera/IMU/LiDAR intrinsics and extrinsics
  imu.csv                   timestamped gyroscope and accelerometer samples
  images/camera_[0-2]/*.jpg timestamp-named decoded camera frames
  lidar_points/*.pcd        raw frames with a per-point ``timestamp`` field
  gnss.csv                  timestamped GNSS/RTK position solutions
  poses.csv                 original project trajectory (when embedded in XBIN)
  raw_*_log.*               raw receiver, NTRIP, and PPK data

The input XBIN is opened read-only through LixelStudio's lixel_bag.dll.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor, as_completed
from contextlib import nullcontext
from datetime import datetime, timezone
from pathlib import Path

from extract_lixel_xbin import DEFAULT_LIXEL_DIR
from export_lixel_extrinsics import export_extrinsics


CAMERA_TOPICS = (
    ("left", "/camera_left/h264", "camera_0"),
    ("center", "/camera_center/h264", "camera_2"),
    ("right", "/camera_right/h264", "camera_1"),
)

RTK_TOPICS = (
    "/gnss_data",
    "/raw_gnss_log",
    "/raw_ntrip_log",
    "/raw_ppk_log",
)


def timestamp_us_from_stem(stem: str) -> int:
    seconds, dot, fraction = stem.partition(".")
    if not dot or not seconds.isdigit() or not fraction.isdigit():
        raise ValueError(f"Not a timestamp filename: {stem}")
    return int(seconds) * 1_000_000 + int((fraction + "000000")[:6])


def write_timestamp_index(directory: Path, pattern: str) -> int:
    rows: list[tuple[int, str, str]] = []
    for path in sorted(directory.glob(pattern)):
        try:
            timestamp_us = timestamp_us_from_stem(path.stem)
        except ValueError:
            continue
        rows.append((timestamp_us, f"{timestamp_us / 1_000_000:.6f}", path.name))

    if rows:
        with (directory / "timestamps.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(("timestamp_us", "timestamp_s", "file"))
            writer.writerows(rows)
    return len(rows)


def count_data_rows(path: Path) -> int:
    if not path.is_file():
        return 0
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        return sum(1 for line in handle if line.strip() and not line.startswith("#"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Export calibration YAML, three timestamped camera streams, and raw "
            "per-point-timestamped PCD frames, IMU, RTK/GNSS, and project poses "
            "from a Lixel XBIN"
        )
    )
    parser.add_argument("xbin", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument(
        "--components",
        nargs="+",
        choices=("all", "config", "cameras", "lidar", "imu", "rtk", "poses"),
        default=("all",),
    )
    parser.add_argument(
        "--start-us",
        type=int,
        default=0,
        help="Absolute Unix timestamp in microseconds; 0 starts at the beginning",
    )
    parser.add_argument(
        "--duration-s",
        type=int,
        default=0,
        help="Integer duration in seconds; 0 exports to the end",
    )
    parser.add_argument("--lixel-dir", type=Path, default=DEFAULT_LIXEL_DIR)
    parser.add_argument(
        "--jobs",
        type=int,
        default=min(4, os.cpu_count() or 1),
        help="Number of sensor topics exported in parallel (default: up to 4)",
    )
    parser.add_argument(
        "--resume",
        action="store_true",
        help="Skip topics with a completed marker from a previous run",
    )
    parser.add_argument(
        "--pcd-format",
        choices=("binary", "ascii"),
        default="binary",
        help="LiDAR PCD serialization format (default: binary)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.xbin.is_file():
        raise SystemExit(f"XBIN not found: {args.xbin}")
    if args.duration_s < 0:
        raise SystemExit("--duration-s must be >= 0")
    if args.jobs < 1:
        raise SystemExit("--jobs must be >= 1")

    components = set(args.components)
    if "all" in components:
        components = {"config", "cameras", "lidar", "imu", "rtk", "poses"}

    args.output.mkdir(parents=True, exist_ok=True)
    state_directory = args.output / ".extract_state"

    def marker_path(topic: str) -> Path:
        safe_name = topic.strip("/").replace("/", "__") or "root"
        return state_directory / f"{safe_name}.json"

    def export(topic: str) -> dict[str, object]:
        # Project files are embedded snapshots, not timestamped pose messages.
        # Always select the complete topic, even for a short sensor export.
        start_us = 0 if topic == "/project_data" else args.start_us
        duration_s = 0 if topic == "/project_data" else args.duration_s
        poses_path = args.output / "poses.csv"
        marker = marker_path(topic)
        if args.resume and marker.is_file():
            try:
                state = json.loads(marker.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                state = {}
            if (
                state.get("source_xbin") == str(args.xbin.resolve())
                and state.get("start_us") == start_us
                and state.get("duration_s") == duration_s
                and (
                    topic != "/project_data"
                    or (poses_path.is_file() and poses_path.stat().st_size > 0)
                )
                and (
                    topic != "/lidar"
                    or state.get("pcd_format", "ascii") == args.pcd_format
                )
            ):
                print(f"Skipping completed {topic}", flush=True)
                return {"topic": topic, "completed": True, "skipped": True}

        # lixel_bag.dll keeps decoder-global state. Isolating topics in child
        # processes prevents one vendor unpacker teardown from corrupting the
        # next topic's decoder state.
        command = [
            sys.executable,
            str(Path(__file__).with_name("extract_lixel_xbin.py")),
            str(args.xbin),
            str(args.output),
            "--topic",
            topic,
            "--start-us",
            str(start_us),
            "--duration-s",
            str(duration_s),
            "--lixel-dir",
            str(args.lixel_dir),
        ]
        if topic == "/lidar" and args.pcd_format == "binary":
            command.append("--binary-pcd")
        print(f"Exporting {topic} ...", flush=True)
        # Validate project output in isolation so a stale poses.csv from an
        # earlier source cannot make an empty vendor export look successful.
        export_directory = (
            tempfile.TemporaryDirectory(prefix=".project_export_", dir=args.output)
            if topic == "/project_data"
            else nullcontext(str(args.output))
        )
        with export_directory as directory:
            command[3] = directory
            completed = subprocess.run(
                command,
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
            if topic == "/project_data" and completed.returncode == 0:
                project_directory = Path(directory) / "project_data"
                exported_poses = project_directory / "poses.csv"
                if not exported_poses.is_file() or exported_poses.stat().st_size == 0:
                    print(
                        "ERROR: /project_data did not produce a nonempty "
                        "project_data/poses.csv; this XBIN may not contain project poses.",
                        file=sys.stderr,
                    )
                    return {
                        "topic": topic, "completed": False, "reason": "poses_missing"
                    }
                shutil.copyfile(exported_poses, poses_path)
        if completed.returncode != 0:
            if completed.stdout:
                print(completed.stdout, end="", file=sys.stderr)
            if completed.stderr:
                print(completed.stderr, end="", file=sys.stderr)
            return {
                "topic": topic,
                "completed": False,
                "exit_code": completed.returncode,
            }
        print(f"Completed {topic}", flush=True)
        state_directory.mkdir(parents=True, exist_ok=True)
        marker.write_text(
            json.dumps(
                {
                    "topic": topic,
                    "source_xbin": str(args.xbin.resolve()),
                    "start_us": start_us,
                    "duration_s": duration_s,
                    "pcd_format": args.pcd_format if topic == "/lidar" else None,
                    "completed_utc": datetime.now(timezone.utc).isoformat(),
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        return {"topic": topic, "completed": True, "skipped": False}

    topics: list[str] = []
    if "config" in components:
        topics.append("/config_file")
    if "imu" in components:
        topics.append("/imu")
    if "cameras" in components:
        topics.extend(topic for _name, topic, _directory in CAMERA_TOPICS)
    if "lidar" in components:
        topics.append("/lidar")
    if "rtk" in components:
        topics.extend(RTK_TOPICS)
    if "poses" in components:
        topics.append("/project_data")

    results: list[dict[str, object]] = []
    if args.jobs == 1:
        results.extend(export(topic) for topic in topics)
    else:
        with ThreadPoolExecutor(max_workers=min(args.jobs, len(topics))) as pool:
            futures = {pool.submit(export, topic): topic for topic in topics}
            for future in as_completed(futures):
                results.append(future.result())

    failures = [result for result in results if not result["completed"]]
    if failures:
        failed_topics = ", ".join(str(result["topic"]) for result in failures)
        raise SystemExit(f"Export failed for: {failed_topics}")

    indexes: dict[str, int] = {}
    extrinsic_directory: str | None = None
    if "config" in components:
        direction_explicit_dir = args.output / "calib" / "extrinsics_xyzw"
        generated = export_extrinsics(
            args.output / "calib" / "config", direction_explicit_dir
        )
        extrinsic_directory = "calib/extrinsics_xyzw"
        print(
            f"Extrinsics: {len(generated)} direction-explicit file(s) in "
            f"{direction_explicit_dir}"
        )

    if "imu" in components:
        count = count_data_rows(args.output / "imu.csv")
        indexes["imu"] = count
        print(f"IMU: {count} timestamped sample(s) in {args.output / 'imu.csv'}")

    if "poses" in components:
        poses_path = args.output / "poses.csv"
        count = count_data_rows(poses_path)
        indexes["poses"] = count
        print(f"Poses: {count} sample(s) in {poses_path} (full project trajectory)")

    if "cameras" in components:
        for name, topic, directory_name in CAMERA_TOPICS:
            directory = args.output / "images" / directory_name
            count = write_timestamp_index(directory, "*.jpg")
            indexes[f"camera_{name}"] = count
            print(f"{name} camera: {count} timestamped JPEG(s) in {directory}")

    if "lidar" in components:
        directory = args.output / "lidar_points"
        count = write_timestamp_index(directory, "*.pcd")
        indexes["lidar"] = count
        print(f"LiDAR: {count} timestamped PCD frame(s) in {directory}")

    manifest = {
        "source_xbin": str(args.xbin.resolve()),
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "start_us": args.start_us,
        "duration_s": args.duration_s,
        "parallel_jobs": args.jobs,
        "pcd_format": args.pcd_format,
        "camera_directory_mapping": {
            name: {"topic": topic, "directory": f"images/{directory_name}"}
            for name, topic, directory_name in CAMERA_TOPICS
        },
        "counts": indexes,
        "exports": results,
        "calibration_directory": "calib/config",
        "direction_explicit_extrinsics": extrinsic_directory,
        "imu_output": "imu.csv",
        "poses_output": "poses.csv" if "poses" in components else None,
        "poses_time_range": "full_project" if "poses" in components else None,
        "rtk_outputs": {
            "solution": "gnss.csv",
            "receiver_text": "raw_gnss_log.txt",
            "ntrip_corrections": "raw_ntrip_log.data",
            "ppk_receiver_data": "raw_ppk_log.data",
        },
        "pcd_fields": (
            "x y z normal_x normal_y normal_z intensity timestamp ring"
        ),
    }
    (args.output / "extraction_manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
