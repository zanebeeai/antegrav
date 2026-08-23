import json
import os

from index_runs import build_index


def test_build_index_orders_runs_and_links_video_without_copying(tmp_path):
    data = tmp_path / "data"
    run = data / "2026-08-23" / "run-new"
    run.mkdir(parents=True)
    (run / "metadata.json").write_text("{}", encoding="utf-8")
    (run / "front_wide.mp4").write_bytes(b"wide-video")
    (run / "front_narrow.mp4").write_bytes(b"narrow-video")
    manifest = {
        "run_id": "run-new",
        "metadata": {
            "status": "complete",
            "utc_start": "2026-08-23T00:50:27+00:00",
        },
        "health": {"ok": True},
        "timeline": {"duration_s": 11.7},
    }
    (run / "viewer.json").write_text(json.dumps(manifest), encoding="utf-8")

    public = tmp_path / "public"
    options = build_index(data, public)

    assert [option["run_id"] for option in options] == ["run-new"]
    assert options[0]["wide_url"].endswith("/front_wide.mp4")
    published = public / "2026-08-23" / "run-new" / "front_wide.mp4"
    assert published.read_bytes() == b"wide-video"
    assert os.path.samefile(run / "front_wide.mp4", published)
