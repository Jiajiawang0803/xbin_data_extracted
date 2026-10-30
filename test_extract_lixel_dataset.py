"""Regression checks for project pose export orchestration (no vendor DLL needed)."""

import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import extract_lixel_dataset as dataset
from extract_lixel_xbin import topic_output_files


POSES = "1785795548.042245,0,0,0,0,0,0,1\n"


class PoseExportTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.source = self.root / "input.xbin"
        self.source.touch()
        self.output = self.root / "output"
        self.commands = []

    def vendor_export(self, command, **kwargs):
        self.commands.append(command)
        if command[command.index("--topic") + 1] == "/project_data":
            project = Path(command[3]) / "project_data"
            project.mkdir(parents=True)
            (project / "poses.csv").write_text(POSES, encoding="utf-8")
            (project / "gnss.csv").write_text("project GNSS\n", encoding="utf-8")
        return subprocess.CompletedProcess(command, 0, "", "")

    def run_export(self, *arguments, vendor=None):
        argv = ["extract_lixel_dataset.py", str(self.source), str(self.output),
                "--jobs", "1", *arguments]
        with (
            patch.object(sys, "argv", argv),
            patch.object(dataset.subprocess, "run", side_effect=vendor or self.vendor_export),
            patch.object(dataset, "export_extrinsics", return_value=[]),
            contextlib.redirect_stdout(io.StringIO()),
            contextlib.redirect_stderr(io.StringIO()),
        ):
            return dataset.main()

    def test_poses_only_writes_root_file_and_records_full_range(self):
        self.assertEqual(self.run_export("--components", "poses", "--start-us",
                                         "1785795600000000", "--duration-s", "1"), 0)
        self.assertEqual(len(self.commands), 1)
        command = self.commands[0]
        self.assertEqual(command[command.index("--start-us") + 1], "0")
        self.assertEqual(command[command.index("--duration-s") + 1], "0")
        self.assertEqual((self.output / "poses.csv").read_text(encoding="utf-8"), POSES)
        self.assertFalse((self.output / "project_data").exists())
        self.assertFalse((self.output / "gnss.csv").exists())
        manifest = json.loads((self.output / "extraction_manifest.json").read_text())
        self.assertEqual(manifest["counts"]["poses"], 1)
        self.assertEqual(manifest["poses_output"], "poses.csv")
        self.assertEqual(manifest["poses_time_range"], "full_project")

    def test_default_all_includes_poses(self):
        self.run_export()
        topics = [command[command.index("--topic") + 1] for command in self.commands]
        self.assertEqual(topics.count("/project_data"), 1)
        self.assertIn("/gnss_data", topics)
        self.assertIn("/lidar", topics)

    def test_resume_skips_existing_poses_but_reextracts_deleted_file(self):
        self.run_export("--components", "poses")
        self.run_export("--components", "poses", "--resume")
        self.assertEqual(len(self.commands), 1)
        (self.output / "poses.csv").unlink()
        self.run_export("--components", "poses", "--resume")
        self.assertEqual(len(self.commands), 2)

    def test_empty_export_cannot_pass_using_stale_poses(self):
        self.output.mkdir(parents=True)
        (self.output / "poses.csv").write_text(POSES, encoding="utf-8")
        def empty_export(command, **kwargs):
            return subprocess.CompletedProcess(command, 0, "", "")
        with self.assertRaisesRegex(SystemExit, "/project_data"):
            self.run_export("--components", "poses", vendor=empty_export)
        self.assertFalse((self.output / ".extract_state" / "project_data.json").exists())

    def test_legacy_completion_marker_reexports_to_root(self):
        project = self.output / "project_data"
        project.mkdir(parents=True)
        (project / "poses.csv").write_text(POSES, encoding="utf-8")
        state = self.output / ".extract_state"
        state.mkdir()
        (state / "project_data.json").write_text(json.dumps({
            "source_xbin": str(self.source.resolve()), "start_us": 0, "duration_s": 0
        }), encoding="utf-8")
        self.run_export("--components", "poses", "--resume")
        self.assertEqual((self.output / "poses.csv").read_text(encoding="utf-8"), POSES)
        self.assertEqual(len(self.commands), 1)

    def test_decoder_error_does_not_create_completion_marker(self):
        def missing_topic(command, **kwargs):
            return subprocess.CompletedProcess(command, 1, "", "No /project_data messages")
        with self.assertRaisesRegex(SystemExit, "/project_data"):
            self.run_export("--components", "poses", vendor=missing_topic)
        self.assertFalse((self.output / ".extract_state" / "project_data.json").exists())

    def test_project_file_tracking_excludes_unrelated_sensor_files(self):
        project = self.output / "project_data"
        (project / "model").mkdir(parents=True)
        poses = project / "poses.csv"
        metadata = project / "model" / "metadata.json"
        poses.touch()
        metadata.touch()
        (self.output / "imu.csv").touch()
        self.assertEqual(topic_output_files(self.output, "/project_data"), {poses, metadata})


if __name__ == "__main__":
    unittest.main()
