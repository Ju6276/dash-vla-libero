#!/usr/bin/env python3
"""Group A: the Harness's frozen VLA alone on SafeLIBERO.

Same checkpoint, VLA server, client and env adapter the Harness uses. No
planner, no perception, no memory, no analytic primitives: the official task
sentence goes to ``full_task_vla`` until the scene terminates or the official
step budget (300, or 550 for LIBERO-10) is spent. Collision follows the
SafeLIBERO rule computed in the env process.
"""

from __future__ import annotations

import argparse
import json
import os
import time
import traceback
from pathlib import Path

from rpent.utils.daemon import ProcessDaemon, pick_free_port
from rpent.utils.rpc.client_utils import wait_for_ready
from rpent.utils.rpc.http_rpc import HttpRpcClient
from robots.libero.env_client import LiberoEnvClient
from robots.libero.safelibero_env import OFFICIAL_MAX_STEPS
from robots.libero.tools import LiberoPrimitives

ROOT = Path(os.environ["RPENT_REPO_ROOT"])
CHUNK = int(os.environ.get("PI05_ACTION_CHUNK", "10"))


def start_server(name: str, cmd: list[str], log_path: Path, overrides: dict[str, str]):
    port = pick_free_port()
    daemon = ProcessDaemon(
        name, [*cmd, "--port", str(port)], env_overrides=overrides, log_path=str(log_path)
    )
    daemon.start()
    client = HttpRpcClient(f"http://127.0.0.1:{port}")
    wait_for_ready(client, timeout_s=600.0, daemon=daemon)
    return daemon, client


def env_overrides() -> dict[str, str]:
    robosuite = os.environ["SAFELIBERO_ROBOSUITE_ROOT"]
    return {
        "LIBERO_CONFIG_PATH": os.environ["SAFELIBERO_CONFIG_PATH"],
        "SAFELIBERO_ROOT": os.environ["SAFELIBERO_ROOT"],
        "LIBERO_SAFETY_ROOT": os.environ["LIBERO_SAFETY_ROOT"],
        "MUJOCO_GL": "egl",
        "PYOPENGL_PLATFORM": "egl",
        "PYTHONPATH": os.pathsep.join([str(ROOT), os.environ["SAFELIBERO_ROOT"], robosuite]),
    }


def run_episode(model, suite: str, task: int, episode: int, out: Path) -> dict:
    base = suite.rsplit("_", 1)[0].replace("safelibero_", "")
    budget = OFFICIAL_MAX_STEPS[base]
    row: dict = {"suite": suite, "task": task, "episode": episode, "budget_steps": budget}
    started = time.time()
    env_cmd = [
        os.environ["LIBERO_SAFETY_PYTHON"],
        str(ROOT / "robots/libero/env_server.py"),
        "--suite", suite,
        "--task", str(task),
        "--seed", str(episode),
        "--max-episode-steps", str(budget),
        "--transport", "http",
        "--host", "127.0.0.1",
        "--parent-watch",
    ]
    log = out / "logs" / f"env_{suite}_t{task}_e{episode}.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    daemon = None
    try:
        daemon, client = start_server("env_server", env_cmd, log, env_overrides())
        daemon.subprocess_env.pop("CUDA_VISIBLE_DEVICES", None)
        env = LiberoEnvClient(
            client,
            expected_meta={
                "suite": suite,
                "task": task,
                "seed": episode,
                "max_episode_steps": budget,
            },
        )
        prim = LiberoPrimitives(env, model, None, lambda: None)
        prim.reset()
        language = client.call("env.get_task_language", timeout_s=60.0)
        result = prim.full_task_vla(language, max_chunks=budget // CHUNK).data
        trace = client.call("env.safety_trace", timeout_s=120.0)
        row.update(
            {
                "language": language,
                "chunks_used": result.get("chunks_used"),
                "obstacle": trace["obstacle"],
                "success": bool(trace["success_step"] is not None),
                "success_step": trace["success_step"],
                "collided": bool(trace["collided"]),
                "collide_step": trace["collide_step"],
                "max_displacement_m": trace["max_displacement_m"],
                "control_steps": trace["control_steps"],
                "contact_keys": sorted(
                    {
                        key
                        for record in trace["step_audits"]
                        for key, value in record["audited_cost"].items()
                        if value
                    }
                ),
                "status": "done",
            }
        )
    except Exception as exc:
        row["status"] = "error"
        row["error"] = f"{type(exc).__name__}: {exc}"
        row["traceback"] = traceback.format_exc()[-2000:]
    finally:
        if daemon is not None:
            daemon.stop()
    row["wall_s"] = round(time.time() - started, 1)
    return row


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--cells", nargs="+", required=True, help="suite:task, e.g. safelibero_goal_I:0")
    parser.add_argument("--episodes", type=int, nargs="+", default=[0, 1, 2])
    args = parser.parse_args()
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    results = out / "results.jsonl"
    done = set()
    if results.exists():
        for line in results.read_text().splitlines():
            row = json.loads(line)
            if row.get("status") == "done":
                done.add((row["suite"], row["task"], row["episode"]))

    vla_cmd = [
        os.environ["PI05_PYTHON"],
        str(ROOT / "rpent/robots/components/pi05_vla_server.py"),
        "--embodiment", "libero",
        "--transport", "http",
        "--host", "127.0.0.1",
        "--parent-watch",
        "--cuda-device", os.environ.get("H0_CUDA_DEVICE", "0"),
        "--model-path", os.environ["PI05_CHECKPOINT_PATH"],
        "--norm-stats-path", os.environ["PI05_NORM_STATS_PATH"],
    ]
    vla_env = {
        "PYTHONPATH": os.pathsep.join([str(ROOT), os.environ.get("OPENPI_CLIENT_SRC", "")]),
        "TORCHDYNAMO_DISABLE": "1",
        "TORCH_COMPILE_DISABLE": "1",
        "PI05_EXTRA_SITE": os.environ.get("PI05_EXTRA_SITE", ""),
        "PI05_ACTION_CHUNK": str(CHUNK),
    }
    vla, vla_client = start_server("vla_server", vla_cmd, out / "vla_server.log", vla_env)
    from rpent.robots.components.pi05_vla_client import Pi05VLAClient

    model = Pi05VLAClient(vla_client, embodiment="libero")
    try:
        for cell in args.cells:
            suite, task = cell.split(":")
            for episode in args.episodes:
                if (suite, int(task), episode) in done:
                    continue
                row = run_episode(model, suite, int(task), episode, out)
                with results.open("a") as handle:
                    handle.write(json.dumps(row) + "\n")
                print(
                    f"DONE {suite} t{task} e{episode} status={row['status']} "
                    f"success={row.get('success')} collided={row.get('collided')} "
                    f"steps={row.get('control_steps')} wall={row['wall_s']}s",
                    flush=True,
                )
    finally:
        vla.stop()


if __name__ == "__main__":
    main()
