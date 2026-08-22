#!/usr/bin/env python3
"""Entry point and preflight utility for self-driving v1 data capture."""

import argparse
import json
from pathlib import Path

from v1_capture.config import load_config, run_setup_issues
from v1_capture.storage import assert_enough_space, estimate_storage

HERE = Path(__file__).resolve().parent
DEFAULT_CONFIG = HERE / "v1_capture.json"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--estimate-minutes", type=float)
    parser.add_argument("--preflight", action="store_true")
    args = parser.parse_args()
    cfg = load_config(args.config)

    if args.estimate_minutes is not None:
        estimate = estimate_storage(cfg, args.estimate_minutes)
        print(json.dumps(estimate.__dict__, indent=2))
        return 0
    if args.preflight:
        free, estimate = assert_enough_space(cfg)
        setup_issues = run_setup_issues(cfg)
        dependencies = {}
        for module in ("rclpy", "pyarrow", "gi"):
            try:
                __import__(module)
                dependencies[module] = "ok"
            except ImportError as exc:
                dependencies[module] = "missing: %s" % exc
        result = {
            "data_root": str(cfg.data_root),
            "free_gb": round(free, 3),
            "required_free_gb": round(estimate.required_free_gb, 3),
            "estimate": estimate.__dict__,
            "dependencies": dependencies,
            "setup_issues": setup_issues,
        }
        print(json.dumps(result, indent=2))
        return (0 if all(value == "ok" for value in dependencies.values())
                and not setup_issues else 2)

    setup_issues = run_setup_issues(cfg)
    if setup_issues:
        raise SystemExit("capture setup is incomplete:\n- " + "\n- ".join(setup_issues))
    from v1_capture.service import run
    return run(cfg, HERE)


if __name__ == "__main__":
    raise SystemExit(main())
