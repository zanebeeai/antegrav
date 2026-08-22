import json
import tempfile
import unittest
from pathlib import Path

from v1_capture.config import load_config, run_setup_issues
from v1_capture.storage import estimate_storage
from v1_capture.sync import FrameSample, FrameSynchronizer
from v1_capture.parquet import FRAME_FIELDS, AsyncParquetWriter


ROOT = Path(__file__).resolve().parents[1]


class CaptureConfigTests(unittest.TestCase):
    def test_repository_config_and_storage_estimates(self):
        cfg = load_config(ROOT / "v1_capture.json")
        one_hour = estimate_storage(cfg, 60)
        ninety = estimate_storage(cfg, 90)
        self.assertAlmostEqual(one_hour.video_gb, 7.2, places=3)
        self.assertAlmostEqual(one_hour.required_free_gb, 19.3125, places=3)
        self.assertAlmostEqual(ninety.required_free_gb, 23.96875, places=3)
        issues = run_setup_issues(cfg)
        self.assertIn("metadata.track_name is not set", issues)
        self.assertTrue(any("front_wide exposure_range" in issue for issue in issues))

    def test_rejects_missing_camera(self):
        raw = json.loads((ROOT / "v1_capture.json").read_text(encoding="utf-8"))
        del raw["cameras"]["front_narrow"]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.json"
            path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "exactly"):
                load_config(path)


class FrameSynchronizerTests(unittest.TestCase):
    def test_pairs_frames_and_flags_alignment(self):
        sync = FrameSynchronizer(20.0)
        sync.narrow(FrameSample(1_000_000_000, 4, 1, 3.0))
        row = sync.wide_row(FrameSample(1_012_000_000, 5, 0, 2.0))
        self.assertEqual(row["front_narrow_frame_index"], 4)
        self.assertAlmostEqual(row["alignment_error_ms"], 12.0)
        self.assertTrue(row["alignment_valid"])
        self.assertEqual(row["dropped_frame_flags"], "front_narrow:1")

    def test_missing_and_misaligned_frames_are_invalid(self):
        sync = FrameSynchronizer(20.0)
        missing = sync.wide_row(FrameSample(1_000_000_000, 0, 0, 1.0))
        self.assertFalse(missing["alignment_valid"])
        self.assertIn("missing", missing["dropped_frame_flags"])
        sync.narrow(FrameSample(900_000_000, 0, 0, 1.0))
        stale = sync.wide_row(FrameSample(1_000_000_000, 1, 0, 1.0))
        self.assertFalse(stale["alignment_valid"])
        self.assertIn("alignment", stale["dropped_frame_flags"])


class ParquetWriterTests(unittest.TestCase):
    def test_writes_readable_frame_table(self):
        import pyarrow.parquet as pq

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "frames.parquet"
            writer = AsyncParquetWriter(path, FRAME_FIELDS, row_group_size=100)
            writer.append({
                "timestamp_ns": 123,
                "front_wide_frame_index": 0,
                "front_narrow_frame_index": 0,
                "alignment_valid": True,
            })
            writer.close()
            table = pq.read_table(path)
            self.assertEqual(table.num_rows, 1)
            self.assertEqual(table.column("timestamp_ns")[0].as_py(), 123)


if __name__ == "__main__":
    unittest.main()
