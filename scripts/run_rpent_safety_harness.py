#!/usr/bin/env python3
"""One LIBERO-Safety episode through RPent's LIBERO primitive, not rldx.

The skill is ``LiberoPrimitives.pi0_doubled``: a language instruction is sent
to the frozen Pi0.5, and each returned action chunk is stepped on the safety
environment. ``pi0_pick`` is only the grasp heuristic and is not used here.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
HARNESS = ROOT / "vendor" / "RPent-harness"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(HARNESS))

from dash_vla.adapters.libero_safety_harness import SAFETY_SUITES, SafetyHarnessEnv  # noqa: E402


def _select(level: int, level_id: int):
    from libero.libero.benchmark import get_benchmark
    from libero.libero.benchmark.vla_safety_task_map import vla_safety_task_map

    suite_name = os.environ.get("SAFETY_SUITE", "obstacle_avoidance")
    if suite_name not in SAFETY_SUITES:
        raise KeyError(f"{suite_name} is not a LIBERO-Safety suite")
    from robots.libero.robot_spec import LIBERO_SUITE_NAMES

    if suite_name not in LIBERO_SUITE_NAMES:
        raise KeyError(
            f"RPent LIBERO_SUITE_NAMES does not list {suite_name}. "
            "The harness allowlist has not been extended."
        )
    suite = get_benchmark(suite_name)()
    task = suite.get_task_by_level_id(level, level_id)
    expected = vla_safety_task_map[suite_name][level][level_id]
    if task is None or task.name != expected:
        raise KeyError(f"task map mismatch: {expected}")
    bddl = Path(suite.get_task_bddl_file_path_by_level_id(level, level_id))
    language = getattr(task, "language", None) or task.name.replace("_", " ")
    return suite_name, task.name, bddl, str(language)


def _settle(env, raw, steps: int = 15):
    wait = np.zeros(7, dtype=np.float32)
    wait[-1] = -1.0
    for _ in range(steps):
        result = env.step(wait)
        raw = result[0]
    return raw


def _start_model() -> subprocess.Popen:
    python = os.environ["PI05_PYTHON"]
    script = ROOT / "scripts" / "run_pi05_one_episode.py"
    model_env = os.environ.copy()
    model_env["CUDA_VISIBLE_DEVICES"] = os.environ.get("PI05_CUDA_DEVICE", "0")
    model_env["TORCHDYNAMO_DISABLE"] = "1"
    model_env["TORCH_COMPILE_DISABLE"] = "1"
    client_src = ROOT / "third_party" / "openpi" / "packages" / "openpi-client" / "src"
    site = ROOT / ".venv-pi05" / "lib" / "python3.10" / "site-packages"
    model_env["PYTHONPATH"] = f"{client_src}{os.pathsep}{site}"
    model_env["HF_HUB_OFFLINE"] = "1"
    model_env["TRANSFORMERS_OFFLINE"] = "1"
    proc = subprocess.Popen(
        [python, str(script), "model"],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        text=True,
        env=model_env,
    )
    while True:
        line = proc.stdout.readline()
        if not line:
            raise RuntimeError("Pi0.5 harness server exited before MODEL_READY")
        print(line.rstrip(), flush=True)
        if line.strip() == "MODEL_READY":
            return proc
    raise RuntimeError("unreachable")


class HarnessPi05:
    """In-process stand-in for ``Pi05VLAClient.predict``.

    The observation dict is the one ``LiberoPrimitives`` holds. Encoding
    matches ``rpent`` ``_encode_obs_libero``; the weights stay in the Pi0.5
    process.
    """

    def __init__(self, proc: subprocess.Popen, scratch: Path) -> None:
        self._proc = proc
        self._scratch = scratch
        self._calls = 0

    def predict(self, env_obs: dict, options: dict | None = None) -> np.ndarray:
        del options
        self._calls += 1
        obs_path = self._scratch / f"obs_{self._calls}.npz"
        act_path = self._scratch / f"act_{self._calls}.npy"
        np.savez(
            obs_path,
            main_images=np.asarray(env_obs["main_images"]),
            wrist_images=np.asarray(env_obs["wrist_images"]),
            states=np.asarray(env_obs["states"], dtype=np.float32),
            task_descriptions=np.array([str(env_obs.get("task_descriptions") or "")]),
        )
        self._proc.stdin.write(json.dumps({"obs": str(obs_path), "out": str(act_path)}) + "\n")
        self._proc.stdin.flush()
        reply = json.loads(self._proc.stdout.readline())
        if not reply.get("ok"):
            raise RuntimeError(reply)
        actions = np.load(act_path)
        if actions.ndim == 1:
            actions = actions[None]
        return actions

    def close(self) -> None:
        try:
            self._proc.stdin.write(json.dumps({"cmd": "stop"}) + "\n")
            self._proc.stdin.flush()
        except Exception:
            pass
        self._proc.terminate()


def _write_video(path: Path, frames: list) -> None:
    if not frames:
        return
    import cv2

    sample = np.asarray(frames[0])
    height, width = sample.shape[:2]
    writer = cv2.VideoWriter(
        str(path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        20.0,
        (width, height),
    )
    for frame in frames:
        image = np.asarray(frame)
        if image.ndim == 3 and image.shape[-1] == 3:
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        writer.write(image)
    writer.release()


def main() -> None:
    level = int(os.environ.get("SAFETY_LEVEL", "0"))
    level_id = int(os.environ.get("SAFETY_LEVEL_ID", "0"))
    suite_name, task_name, bddl, language = _select(level, level_id)
    benchmark = ROOT / "third_party" / "LIBERO-Safety" / "libero" / "libero"
    init_path = (
        benchmark
        / "init_files"
        / bddl.parent.relative_to(benchmark / "bddl_files")
        / f"{bddl.stem}.pruned_init"
    )
    if "LIBERO-Safety" not in str(bddl):
        raise RuntimeError(f"harness resolved a non-safety BDDL: {bddl}")

    import torch
    from libero.libero.envs import OffScreenRenderEnv

    sim = OffScreenRenderEnv(
        bddl_file_name=str(bddl),
        camera_heights=256,
        camera_widths=256,
    )
    sim.reset()
    loaded = torch.load(init_path, weights_only=False)
    if hasattr(loaded, "detach"):
        loaded = loaded.detach().cpu().numpy()
    loaded = np.asarray(loaded)
    init_state = loaded[0] if loaded.ndim > 1 else loaded
    raw = sim.set_init_state(np.asarray(init_state, dtype=np.float64))
    env = SafetyHarnessEnv(sim, language, max_steps=int(os.environ.get("SAFETY_MAX_STEPS", "300")))
    raw = _settle(sim, raw)
    obs = env.set_settled_obs(raw)

    scratch = ROOT / "runs" / "g5_libero" / f"harness_safety_L{level}_id{level_id}"
    scratch.mkdir(parents=True, exist_ok=True)
    proc = _start_model()
    model = HarnessPi05(proc, scratch)
    success = False
    data: dict = {}
    try:
        from robots.libero.tools import LiberoPrimitives

        primitives = LiberoPrimitives(
            env,  # type: ignore[arg-type]
            model,  # type: ignore[arg-type]
            sam3_client=None,  # pi0_doubled does not segment
            check_cancelled=lambda: None,
        )
        primitives.set_obs(obs)
        primitives.start_recording()
        result = primitives.pi0_doubled(
            language, max_chunks=int(os.environ.get("HARNESS_MAX_CHUNKS", "30"))
        )
        data = result.data if hasattr(result, "data") else dict(result)
        frames = primitives.stop_recording()
        success = bool(sim.check_success())
        try:
            _write_video(scratch / "episode.mp4", frames)
        except Exception as exc:
            print(f"video write failed: {exc}", flush=True)
    finally:
        model.close()
        sim.close()

    report = {
        "harness": "robots.libero.tools.LiberoPrimitives.pi0_doubled",
        "not_rldx": True,
        "suite_allowlist_extended": True,
        "suite": suite_name,
        "level": level,
        "level_local_id": level_id,
        "task_name": task_name,
        "bddl_path": str(bddl),
        "init_state_path": str(init_path),
        "language": language,
        "checkpoint": os.environ.get("PI05_CHECKPOINT"),
        "checkpoint_role": os.environ.get("PI05_CHECKPOINT_ROLE"),
        "primitive_result": data,
        "check_success": success,
        "control_steps": env.steps,
        "constraint_positive_steps": env.positive_steps,
        "constraint_first_positive_step": env.first_positive,
        "constraint_max": {
            "checkrobotcontact": env.max_robot,
            "checkcontact": env.max_contact,
        },
        "official_constraint_cost_last": env.last_cost,
        "video": str(scratch / "episode.mp4"),
        "evaluation_note": "RPent LiberoPrimitives.pi0_doubled on LIBERO-Safety. Not rldx. Not G6.",
    }
    out = ROOT / "runs" / "g5_libero" / f"harness_safety_L{level}_id{level_id}.json"
    out.write_text(json.dumps(report, indent=2, default=str) + "\n")
    print(json.dumps(report), flush=True)


if __name__ == "__main__":
    main()
