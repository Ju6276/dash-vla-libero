"""Re-audit the stove task with the frozen safety policy.

Seeds select init-bank rows. The VLA process keeps the seed it set at startup.
Collision diagnostics are not included. Official task files are not modified.
"""

from __future__ import annotations

import json
import os
import sys
import traceback
from argparse import Namespace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
HARNESS = ROOT / "vendor" / "RPent-harness"
os.environ.setdefault("RPENT_REPO_ROOT", str(HARNESS))
sys.path.insert(0, str(HARNESS))

from robots.libero.robot_spec import _init_runtime  # noqa: E402
from rpent.dashboard.events import NullDashboardEventSink  # noqa: E402


def _write_video(path: Path, frames: list) -> None:
    if not frames:
        return
    import cv2

    sample = np.asarray(frames[0])
    height, width = sample.shape[:2]
    writer = cv2.VideoWriter(
        str(path), cv2.VideoWriter_fourcc(*"mp4v"), 20.0, (width, height)
    )
    for frame in frames:
        image = np.asarray(frame)
        if image.ndim == 3 and image.shape[-1] == 3:
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        writer.write(image)
    writer.release()


def _args(task: int, seed: int) -> Namespace:
    return Namespace(
        suite="obstacle_avoidance",
        task=task,
        seed=seed,
        max_episode_steps=int(os.environ.get("SAFETY_MAX_STEPS", "300")),
        env_endpoint=None,
        vla_endpoint=None,
        sam3_endpoint=None,
        molmo_endpoint=None,
        cuda_device=int(os.environ.get("PI05_CUDA_DEVICE", "0")),
        planner=None,
        collect_flywheel_data=False,
        libero_type=None,
        flywheel_root=None,
    )


def _positive_keys(trace: dict) -> list[str]:
    keys = []
    for record in trace.get("step_audits", []):
        if record.get("phase") != "control":
            continue
        for key, value in record.get("audited_cost", {}).items():
            if value and key not in keys:
                keys.append(key)
    return keys


def run_episode(model, task: int, seed: int) -> dict:
    output = ROOT / "runs" / "g5_libero" / "l42" / f"t{task}_s{seed}"
    output.mkdir(parents=True, exist_ok=True)
    report = {
        "role": "l42_raw_policy_reaudit",
        "include_in_diagnostic_collisions": False,
        "suite": "obstacle_avoidance",
        "task": task,
        "seed": seed,
        "skill": "LiberoPrimitives.pi0_doubled",
    }
    daemons = []
    try:
        daemons, runtime = _init_runtime(
            _args(task, seed), output, NullDashboardEventSink(), {"env"}
        )
        env = runtime["env"]
        language = env._client.call("env.get_task_language")
        from robots.libero.tools import LiberoPrimitives

        primitives = LiberoPrimitives(
            env, model, sam3_client=None, check_cancelled=lambda: None
        )
        primitives.set_obs(env.last_obs)
        primitives.start_recording()
        result = primitives.pi0_doubled(
            str(language),
            max_chunks=int(os.environ.get("HARNESS_MAX_CHUNKS", "30")),
        )
        frames = primitives.stop_recording()
        video = output / "episode.mp4"
        try:
            _write_video(video, frames)
        except Exception as exc:
            report["video_error"] = str(exc)
        trace = env._client.call("env.safety_trace")
        transition = trace.get("success_transition")
        report.update(
            {
                "language": language,
                "primitive_result": result.data,
                "task_success": bool(trace["check_success"]),
                "init_index": trace["init_index"],
                "init_bank_size": trace["init_bank_size"],
                "bddl_path": trace["bddl_path"],
                "control_steps": trace["control_steps"],
                "official_positive_steps": trace["constraint_positive_steps"],
                "audited_positive_steps": trace["audited_positive_steps"],
                "settle_audited_positive_steps": trace["settle_audited_positive_steps"],
                "audited_keys_ever_positive": _positive_keys(trace),
                "success_transition": transition,
                "video": str(video),
                "safety_trace": trace,
            }
        )
    except Exception:
        report["error"] = traceback.format_exc()
        log = output / "env_server.log"
        if log.is_file():
            report["env_server_log"] = "\n".join(log.read_text(errors="replace").splitlines()[-60:])
    finally:
        for daemon in reversed(daemons):
            try:
                daemon.stop()
            except Exception:
                pass
        path = output / "episode.json"
        path.write_text(json.dumps(report, indent=2, default=str) + "\n")
    summary = {
        "task": task,
        "seed": seed,
        "task_success": report.get("task_success"),
        "official_positive_steps": report.get("official_positive_steps"),
        "audited_positive_steps": report.get("audited_positive_steps"),
        "audited_keys_ever_positive": report.get("audited_keys_ever_positive"),
        "error": "error" in report,
    }
    print(json.dumps(summary), flush=True)
    return report


def main() -> None:
    out = ROOT / "runs" / "g5_libero" / "l42"
    vla_out = out / "vla"
    vla_out.mkdir(parents=True, exist_ok=True)
    daemons, runtime = _init_runtime(
        _args(0, 0), vla_out, NullDashboardEventSink(), {"vla"}
    )
    model = runtime["model"]
    reports = []
    try:
        for task in (0, 5):
            for seed in range(5):
                reports.append(run_episode(model, task, seed))
    finally:
        for daemon in reversed(daemons):
            try:
                daemon.stop()
            except Exception:
                pass
    summary = [
        {
            "task": item["task"],
            "seed": item["seed"],
            "task_success": item.get("task_success"),
            "official_positive_steps": item.get("official_positive_steps"),
            "audited_positive_steps": item.get("audited_positive_steps"),
            "audited_keys_ever_positive": item.get("audited_keys_ever_positive"),
            "error": "error" in item,
        }
        for item in reports
    ]
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({"wrote": str(out / "summary.json"), "episodes": summary}), flush=True)


if __name__ == "__main__":
    main()
