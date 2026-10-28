#!/usr/bin/env python3
"""Run LIBERO-Safety through RPent's own env server and Pi0.5 server.

The agent process only sees ``LiberoEnvClient`` and ``Pi05VLAClient``.
``LiberoPrimitives.pi0_doubled`` is the skill. There is no in-process simulator
and no rldx call.
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

from robots.libero.robot_spec import LIBERO_SUITE_NAMES, _init_runtime  # noqa: E402
from robots.libero.safety_env import SAFETY_SUITES  # noqa: E402
from rpent.dashboard.events import NullDashboardEventSink  # noqa: E402


def _tail(path: Path, limit: int = 80) -> str:
    if not path.is_file():
        return ""
    lines = path.read_text(errors="replace").splitlines()
    return "\n".join(lines[-limit:])


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


def run_task(suite: str, task: int, seed: int) -> dict:
    if suite not in LIBERO_SUITE_NAMES:
        raise KeyError(f"{suite} is not in the RPent LIBERO allowlist")
    if suite not in SAFETY_SUITES:
        raise KeyError(f"{suite} is not a LIBERO-Safety suite")
    output = ROOT / "runs" / "g5_libero" / f"harness_runtime_{suite}_t{task}_s{seed}"
    output.mkdir(parents=True, exist_ok=True)
    args = Namespace(
        suite=suite,
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
    report: dict = {
        "framework": "robots.libero.robot_spec._init_runtime",
        "env_server": "robots/libero/env_server.py",
        "vla_server": "rpent/robots/components/pi05_vla_server.py",
        "skill": "LiberoPrimitives.pi0_doubled",
        "not_rldx": True,
        "suite": suite,
        "task": task,
        "seed": seed,
    }
    daemons = []
    try:
        daemons, runtime = _init_runtime(
            args, output, NullDashboardEventSink(), {"env", "vla"}
        )
        env = runtime["env"]
        model = runtime["model"]
        language = env._client.call("env.get_task_language")
        trace = env._client.call("env.safety_trace")
        spec = env.action_specs["default"]
        status = model.status()
        if f"{suite}/L{trace['level']}/" not in trace["bddl_path"]:
            raise RuntimeError(f"env server did not open a safety level path: {trace['bddl_path']}")
        obs = env.last_obs
        from robots.libero.tools import LiberoPrimitives

        primitives = LiberoPrimitives(
            env,
            model,
            sam3_client=None,
            check_cancelled=lambda: None,
        )
        primitives.set_obs(obs)
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
        report.update(
            {
                "language": language,
                "vla_action_chunk": status.get("action_chunk"),
                "action_dim": len(spec["low"]),
                "obs_state_shape": list(np.asarray(obs["states"]).shape),
                "primitive_result": result.data,
                "safety_trace": trace,
                "check_success": trace["check_success"],
                "constraint_positive_steps": trace["constraint_positive_steps"],
                "bddl_path": trace["bddl_path"],
                "video": str(video),
            }
        )
    except Exception:
        report["error"] = traceback.format_exc()
        report["env_server_log"] = _tail(output / "env_server.log")
        report["vla_server_log"] = _tail(output / "vla_server.log")
        raise
    finally:
        for daemon in reversed(daemons):
            try:
                daemon.stop()
            except Exception:
                pass
        out = ROOT / "runs" / "g5_libero" / f"harness_runtime_{suite}_t{task}_s{seed}.json"
        out.write_text(json.dumps(report, indent=2, default=str) + "\n")
        print(json.dumps(report, default=str), flush=True)
    return report


def main() -> None:
    suite = os.environ.get("SAFETY_SUITE", "obstacle_avoidance")
    tasks = [
        int(item)
        for item in os.environ.get("HARNESS_TASKS", "0,5").split(",")
        if item.strip()
    ]
    seed = int(os.environ.get("HARNESS_SEED", "0"))
    for task in tasks:
        run_task(suite, task, seed)


if __name__ == "__main__":
    main()
