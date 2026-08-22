import json
import tempfile
import unittest
from pathlib import Path

from prepare_run import downsample, prepare


class PrepareRunTests(unittest.TestCase):
    def test_downsamples_by_monotonic_timestamp(self):
        rows = [{"timestamp_ns": value} for value in (0, 10, 49, 50, 99, 100)]
        self.assertEqual(
            [row["timestamp_ns"] for row in downsample(rows, 50)],
            [0, 50, 100],
        )

    def test_reports_interrupted_run_without_crashing(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            (run / "metadata.json").write_text(
                json.dumps({
                    "run_id": "test-run",
                    "status": "recording",
                    "monotonic_start_ns": 123,
                }),
                encoding="utf-8",
            )
            for name in ("frames", "telemetry", "events"):
                (run / f"{name}.parquet").write_bytes(b"PAR1")
            for name in ("front_wide", "front_narrow"):
                (run / f"{name}.mp4").write_bytes(b"")

            output = prepare(run)
            payload = json.loads(output.read_text(encoding="utf-8"))

        self.assertFalse(payload["health"]["ok"])
        self.assertEqual(payload["health"]["source_rows"]["telemetry"], 0)
        self.assertIn("Run status is recording, not complete", payload["health"]["warnings"])
        self.assertIn("frames.parquet is incomplete (4 bytes)", payload["health"]["warnings"])
        self.assertFalse(payload["videos"]["wide"]["valid"])


if __name__ == "__main__":
    unittest.main()
