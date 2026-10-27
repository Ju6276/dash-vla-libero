#!/usr/bin/env python3
"""One-episode native smoke for standard LIBERO or LIBERO-Safety.

Does not load a VLA, does not call the RPent harness, and does not rename the
official constraint cost.
"""

from __future__ import annotations

import argparse
import json
import os
import traceback
from pathlib import Path

import numpy as np


def worktree_root() -> Path:
    return Path(__file__).resolve().parents[1]


def ensure_config(config_dir: Path, benchmark_root: Path) -> None:
    home_config = Path.home() / ".libero"
    if config_dir == home_config or home_config in config_dir.parents:
        raise RuntimeError(f"refusing to write LIBERO config under {home_config}")
    config_dir.mkdir(parents=True, exist_ok=True)
    datasets = worktree_root() / "runs" / "g5_libero" / "datasets_placeholder"
    datasets.mkdir(parents=True, exist_ok=True)
    payload = {
        "benchmark_root": str(benchmark_root),
        "bddl_files": str(benchmark_root / "bddl_files"),
        "init_states": str(benchmark_root / "init_files"),
        "datasets": str(datasets),
        "assets": str(benchmark_root / "assets"),
    }
    text = "".join(f"{key}: {value}\n" for key, value in payload.items())
    (config_dir / "config.yaml").write_text(text)
    os.environ["LIBERO_CONFIG_PATH"] = str(config_dir)


def classify_exception(exc: BaseException) -> str:
    text = f"{type(exc).__name__}: {exc}".lower()
    if isinstance(exc, KeyError) or "not in benchmark" in text or "unregistered" in text:
        return "task_missing"
    if any(
        token in text
        for token in (
            "modulenotfounderror",
            "no module named",
            "importerror",
            "libegl",
            "egl",
            "asset",
            "no such file",
            "filenotfound",
            "bddl",
            "mesh",
            "cannot find",
            "mujoco",
        )
    ):
        return "dependency"
    if any(token in text for token in ("action", "observation", "obs", "shape", "dimension")):
        return "action_observation"
    return "environment_compatibility_error"


def obs_summary(obs) -> dict:
    if isinstance(obs, (tuple, list)) and obs and isinstance(obs[0], dict):
        obs = obs[0]
    if not isinstance(obs, dict):
        return {"type": type(obs).__name__, "repr": repr(obs)[:500]}
    summary = {}
    for key, value in obs.items():
        if isinstance(value, np.ndarray):
            summary[key] = {"shape": list(value.shape), "dtype": str(value.dtype)}
        else:
            summary[key] = {"type": type(value).__name__}
    return summary


def save_frame(obs, path: Path) -> str | None:
    if isinstance(obs, (tuple, list)) and obs and isinstance(obs[0], dict):
        obs = obs[0]
    if not isinstance(obs, dict) or "agentview_image" not in obs:
        return "observation has no agentview_image"
    image = np.asarray(obs["agentview_image"])
    if image.dtype != np.uint8:
        image = np.clip(image, 0, 255).astype(np.uint8)
    if image.ndim == 3 and image.shape[0] in (1, 3) and image.shape[-1] not in (1, 3, 4):
        image = np.transpose(image, (1, 2, 0))
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        import imageio.v2 as imageio

        imageio.imwrite(path, image)
    except Exception as exc:  # noqa: BLE001
        return f"{type(exc).__name__}: {exc}"
    return None


def save_video(frames: list[np.ndarray], path: Path) -> str | None:
    if not frames:
        return "no frames"
    try:
        import imageio.v2 as imageio

        imageio.mimwrite(path, frames, fps=2)
    except Exception as exc:  # noqa: BLE001
        return f"{type(exc).__name__}: {exc}"
    return None


def as_frame(obs) -> np.ndarray | None:
    if isinstance(obs, (tuple, list)) and obs and isinstance(obs[0], dict):
        obs = obs[0]
    if not isinstance(obs, dict) or "agentview_image" not in obs:
        return None
    image = np.asarray(obs["agentview_image"])
    if image.dtype != np.uint8:
        image = np.clip(image, 0, 255).astype(np.uint8)
    if image.ndim == 3 and image.shape[0] in (1, 3) and image.shape[-1] not in (1, 3, 4):
        image = np.transpose(image, (1, 2, 0))
    return image


def zero_action(env) -> np.ndarray:
    inner = env.env
    low, _high = inner.action_spec
    return np.zeros_like(np.asarray(low))


def as_numpy_state(state):
    if hasattr(state, "detach"):
        state = state.detach().cpu().numpy()
    return np.asarray(state)


def unwrap_step(result):
    if isinstance(result, tuple) and len(result) == 4:
        return result, None
    if isinstance(result, tuple) and len(result) == 5:
        obs, reward, terminated, truncated, info = result
        return (obs, reward, bool(terminated or truncated), info), (
            "step returned 5 values; stored done as terminated or truncated"
        )
    return None, f"unexpected step return type {type(result).__name__} len={getattr(result, '__len__', lambda: None)()}"


def select_safety_task(suite_name: str, level: int, level_id: int) -> dict:
    from libero.libero import get_libero_path
    from libero.libero.benchmark import get_benchmark
    from libero.libero.benchmark.vla_safety_task_map import vla_safety_task_map

    map_name = vla_safety_task_map[suite_name][level][level_id]
    suite = get_benchmark(suite_name)()
    task = suite.get_task_by_level_id(level, level_id)
    if task is None or task.name != map_name:
        raise KeyError(
            f"task map / get_benchmark mismatch for {suite_name} L{level} id {level_id}: "
            f"map={map_name} benchmark={None if task is None else task.name}"
        )
    bddl = suite.get_task_bddl_file_path_by_level_id(level, level_id)
    init_path = os.path.join(
        get_libero_path("init_states"),
        task.problem_folder,
        f"L{task.level}",
        task.init_states_file,
    )
    return {
        "suite": suite_name,
        "level": level,
        "level_local_id": level_id,
        "task_name": task.name,
        "bddl_path": str(Path(bddl).resolve()),
        "init_state_path": str(Path(init_path).resolve()),
    }


def select_rpent_task(suite_name: str, task_id: int) -> dict:
    from libero.libero import get_libero_path
    from libero.libero.benchmark import BENCHMARK_MAPPING, get_benchmark

    if suite_name not in BENCHMARK_MAPPING:
        raise KeyError(f"{suite_name} is not registered by get_benchmark")
    suite = get_benchmark(suite_name)()
    task = suite.get_task(task_id)
    bddl = suite.get_task_bddl_file_path(task_id)
    init_path = os.path.join(
        get_libero_path("init_states"), task.problem_folder, task.init_states_file
    )
    return {
        "suite": suite_name,
        "task_id": task_id,
        "task_name": task.name,
        "bddl_path": str(Path(bddl).resolve()),
        "init_state_path": str(Path(init_path).resolve()),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--which", choices=("rpent", "safety"), required=True)
    parser.add_argument("--benchmark-root", type=Path, required=True)
    parser.add_argument("--config-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--suite", required=True)
    parser.add_argument("--task-id", type=int, default=0)
    parser.add_argument("--level", type=int, default=0)
    parser.add_argument("--level-id", type=int, default=0)
    args = parser.parse_args()

    os.environ.setdefault("MUJOCO_GL", "egl")
    os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
    home_before = (Path.home() / ".libero").exists()
    log = {
        "which": args.which,
        "mujoco_gl": os.environ.get("MUJOCO_GL"),
        "egl_device": os.environ.get("MUJOCO_EGL_DEVICE_ID"),
        "cuda_visible_devices": os.environ.get("CUDA_VISIBLE_DEVICES"),
        "model": {
            "pi05_weights_present": False,
            "prediction_ran": False,
            "category": "missing_weights",
            "detail": "No frozen pi0.5 checkpoint was present, so this episode did not call a VLA.",
        },
        "vla_coverage_gap": None,
        "environment_compatibility_error": None,
        "failure_category": None,
    }
    try:
        ensure_config(args.config_dir.resolve(), args.benchmark_root.resolve())
        import torch
        from libero.libero.envs import OffScreenRenderEnv

        if args.which == "safety":
            selected = select_safety_task(args.suite, args.level, args.level_id)
        else:
            selected = select_rpent_task(args.suite, args.task_id)
        log["task"] = selected
        for key in ("bddl_path", "init_state_path"):
            if not Path(selected[key]).is_file():
                raise FileNotFoundError(selected[key])
        assets = args.benchmark_root / "assets"
        log["assets_dir_exists"] = assets.is_dir()
        if not log["assets_dir_exists"]:
            raise FileNotFoundError(f"assets directory missing: {assets}")

        env = OffScreenRenderEnv(
            bddl_file_name=selected["bddl_path"],
            camera_heights=128,
            camera_widths=128,
        )
        reset_ret = env.reset()
        init_states = torch.load(selected["init_state_path"], weights_only=False)
        init_state = as_numpy_state(init_states[0])
        obs = env.set_init_state(init_state)
        action = zero_action(env)
        step_ret = env.step(action)
        unpacked, step_note = unwrap_step(step_ret)
        if unpacked is None:
            log["environment_compatibility_error"] = step_note
            log["failure_category"] = "action_observation"
            raise RuntimeError(step_note)
        step_obs, reward, done, info = unpacked
        success = bool(env.check_success())
        frame_dir = args.output.parent / (args.output.stem + "_frames")
        frame_errors = []
        frames = []
        for label, image_obs in (("reset", obs), ("step", step_obs)):
            err = save_frame(image_obs, frame_dir / f"{label}.png")
            if err:
                frame_errors.append(f"{label}: {err}")
            frame = as_frame(image_obs)
            if frame is not None:
                frames.append(frame)
        video_path = frame_dir / "smoke.mp4"
        video_error = save_video(frames, video_path)
        cost = None
        if isinstance(info, dict) and "cost" in info:
            cost = info["cost"]
        log.update(
            {
                "reset_return_type": type(reset_ret).__name__,
                "reset_observation": obs_summary(obs),
                "action": {
                    "source": "zero_action_no_model",
                    "shape": list(np.asarray(action).shape),
                    "dtype": str(np.asarray(action).dtype),
                },
                "step": {
                    "note": step_note,
                    "reward": float(reward) if np.isscalar(reward) else str(reward),
                    "done": bool(done),
                    "info_keys": sorted(info.keys()) if isinstance(info, dict) else type(info).__name__,
                    "official_constraint_cost": cost,
                    "check_success": success,
                    "observation": obs_summary(step_obs),
                },
                "frames": {
                    "dir": str(frame_dir),
                    "png_errors": frame_errors,
                    "video": None if video_error else str(video_path),
                    "video_error": video_error,
                },
                "ok": True,
            }
        )
        env.close()
    except Exception as exc:  # noqa: BLE001
        log["ok"] = False
        log["exception"] = f"{type(exc).__name__}: {exc}"
        log["traceback"] = traceback.format_exc()
        if log["failure_category"] is None:
            log["failure_category"] = classify_exception(exc)
        if log["failure_category"] == "environment_compatibility_error":
            log["environment_compatibility_error"] = log["exception"]
    log["home_libero_created"] = (Path.home() / ".libero").exists() and not home_before
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(log, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps({k: log[k] for k in ("ok", "failure_category", "exception") if k in log}, ensure_ascii=False))
    return 0 if log.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
