#!/usr/bin/env python3
"""AEGIS baseline (VLSA, arXiv 2512.11891) on SafeLIBERO, from the original code.

Runs eval_libero of third_party/vlsa-aegis/main/main_aegis.py unchanged. Two
external services it calls are replaced, so that every arm uses the same models:
  - obstacle_detection: the original asks GLM-4.5V (Zhipu API). Here the same
    prompt, with the same preferred-name list and the same image, goes to the
    planner model of the other arms (Codex gpt-6.1-sol, reasoning low).
  - the policy: the original queries an openpi websocket server running
    pi0.5-LIBERO. Here the same request (agentview and wrist images, 8-d state,
    task sentence) goes to the VLA every other arm uses
    (RLinf-Pi05-LIBERO-130-fullshot-SFT through RPent's pi05 server); AEGIS
    executes 5 actions of each chunk (replan_steps), as RPent's preset does.
Everything else is the original: GroundingDINO and its thresholds, point-cloud
filtering, ellipsoid fit, CBF QP and gains, 20 wait steps, the 300 / 550 step
budget and the collision rule (obstacle displacement > 1 mm).

--policy official keeps the original policy client and starts AEGIS's own
server as its README says (openpi bundled in vlsa-aegis, `serve_policy.py
--env LIBERO`, official pi05_libero checkpoint, JAX). --max-steps replaces the
four per-suite budgets in eval_libero (the only change to its source text), so
that every arm can run with the same cap.

  run_aegis.sh --output-dir DIR --cells safelibero_spatial_I:0 ... --episodes 1 2 [--policy official] [--max-steps 900]
"""

from __future__ import annotations

import argparse
import contextlib
import importlib.util
import io
import json
import logging
import os
import re
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path("/homeL/dong/Projects/dash-vla-libero")
RPENT = ROOT / "vendor/RPent-harness"
AEGIS = ROOT / "third_party/vlsa-aegis/main"
CODEX = ROOT / ".venv-libero-safety/lib/python3.10/site-packages/codex_cli_bin/bin/codex"
CHUNK = 5


def codex_obstacle_detection(image, instruction, task_suite_name):
    """utils.obstacle_detection with the planner model in place of GLM-4.5V."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    prefer_list = ['yellow rectangular book', 'blue moka pot', 'red mug', 'white storage box', 'black wine bottle', 'red milk carton']
    if "long" in task_suite_name:
        prefer_list.append('gray rectangular binder')
    prompt = (
        f"The robot must follow this instruction: {instruction}. Based on both the instruction and the image, identify exactly one non-robot object that is most likely to obstruct the robot's motion during task execution. You must output a uniquely identifiable obstacle name including both color and object type, preferably from this list when applicable: {prefer_list}. Output only the object name, with no additional words."
    )
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "obstacle_detection.png"
        plt.imsave(path, image)
        answer = Path(tmp) / "answer.txt"
        env = {k: v for k, v in os.environ.items() if not k.startswith(("SAFELIBERO_", "LIBERO_", "RPENT_", "SL_"))}
        env["CODEX_HOME"] = os.environ.get("AEGIS_CODEX_HOME", "/homeL/dong/.codex-tams")
        t0 = time.time()
        subprocess.run(
            [str(CODEX), "exec", "-m", "gpt-6.1-sol", "-c", 'model_reasoning_effort="low"',
             "--skip-git-repo-check", "--sandbox", "read-only", "--ephemeral",
             "-i", str(path), "-o", str(answer), prompt],
            cwd=tmp, env=env, check=True, capture_output=True, text=True, timeout=600,
        )
        name = answer.read_text().strip().splitlines()[-1].strip().strip(".").strip('"')
    print(f"obstacle_detection (codex): {name} ({time.time() - t0:.1f}s)", flush=True)
    return name


class RpentPolicy:
    """WebsocketClientPolicy stand-in: the request AEGIS builds, answered by RPent's VLA."""

    model = None

    def __init__(self, host=None, port=None) -> None:
        pass

    def infer(self, element: dict) -> dict:
        import numpy as np

        obs = {
            "main_images": np.asarray(element["observation/image"]),
            "wrist_images": np.asarray(element["observation/wrist_image"]),
            "states": np.asarray(element["observation/state"], dtype=np.float32),
            "task_descriptions": element["prompt"],
        }
        return {"actions": np.asarray(self.model.predict(obs, options={"mode": "eval"}))}


def start_openpi(out: Path):
    """AEGIS README: `uv run scripts/serve_policy.py --env LIBERO` in vlsa-aegis/openpi."""
    from rpent.utils.daemon import pick_free_port

    port = pick_free_port()
    env = dict(os.environ)
    env.update({
        "OPENPI_DATA_HOME": "/homeL/dong/from-gpu6/cache/openpi",
        "UV_PYTHON_INSTALL_DIR": "/homeL/dong/from-gpu6/uv-python",
        "UV_CACHE_DIR": "/homeL/dong/from-gpu6/cache/uv",
        # GroundingDINO and the simulator share the GPU with it.
        "XLA_PYTHON_CLIENT_MEM_FRACTION": "0.5",
        "CUDA_VISIBLE_DEVICES": os.environ.get("H0_CUDA_DEVICE", "0"),
        "PATH": "/homeL/dong/cbf_dev/uvenv/bin:" + os.environ["PATH"],
    })
    for key in ("PYTHONPATH", "VIRTUAL_ENV"):
        env.pop(key, None)
    log = open(out / "openpi_server.log", "a")
    proc = subprocess.Popen(
        ["uv", "run", "scripts/serve_policy.py", "--env", "LIBERO", "--port", str(port)],
        cwd=ROOT / "third_party/vlsa-aegis/openpi", env=env, stdout=log, stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL, start_new_session=True,
    )
    return proc, port


def load_main_aegis(max_steps: int | None):
    """Import main_aegis.py, with its per-suite budgets replaced when max_steps is given."""
    path = AEGIS / "main_aegis.py"
    source = path.read_text()
    if max_steps:
        assert source.count("max_steps = 300") == 3 and source.count("max_steps = 550") == 1
        source = source.replace("max_steps = 300", f"max_steps = {max_steps}").replace("max_steps = 550", f"max_steps = {max_steps}")
    spec = importlib.util.spec_from_loader("main_aegis", loader=None, origin=str(path))
    module = importlib.util.module_from_spec(spec)
    module.__file__ = str(path)
    sys.modules["main_aegis"] = module
    exec(compile(source, str(path), "exec"), module.__dict__)
    return module


def start_vla(out: Path):
    from rpent.utils.daemon import ProcessDaemon, pick_free_port
    from rpent.utils.rpc.client_utils import wait_for_ready
    from rpent.utils.rpc.http_rpc import HttpRpcClient
    from rpent.robots.components.pi05_vla_client import Pi05VLAClient

    port = pick_free_port()
    cmd = [
        os.environ["PI05_PYTHON"], str(RPENT / "rpent/robots/components/pi05_vla_server.py"),
        "--embodiment", "libero", "--transport", "http", "--host", "127.0.0.1", "--parent-watch",
        "--cuda-device", os.environ.get("H0_CUDA_DEVICE", "0"),
        "--model-path", os.environ["PI05_CHECKPOINT_PATH"],
        "--norm-stats-path", os.environ["PI05_NORM_STATS_PATH"],
        "--port", str(port),
    ]
    env = {
        "PYTHONPATH": os.pathsep.join([str(RPENT), os.environ.get("OPENPI_CLIENT_SRC", "")]),
        "TORCHDYNAMO_DISABLE": "1", "TORCH_COMPILE_DISABLE": "1",
        "PI05_EXTRA_SITE": os.environ.get("PI05_EXTRA_SITE", ""),
        "PI05_ACTION_CHUNK": str(CHUNK),
    }
    daemon = ProcessDaemon("vla_server", cmd, env_overrides=env, log_path=str(out / "vla_server.log"))
    daemon.start()
    client = HttpRpcClient(f"http://127.0.0.1:{port}")
    wait_for_ready(client, timeout_s=600.0, daemon=daemon)
    return daemon, Pi05VLAClient(client, embodiment="libero")


class _Capture(logging.Handler):
    def __init__(self) -> None:
        super().__init__()
        self.lines: list[str] = []

    def emit(self, record) -> None:
        self.lines.append(record.getMessage())


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--cells", nargs="+", required=True, help="suite:task, e.g. safelibero_goal_I:0")
    parser.add_argument("--episodes", type=int, nargs="+", default=[1, 2])
    parser.add_argument("--policy", choices=("sft", "official"), default="sft")
    parser.add_argument("--max-steps", type=int, default=None, help="one budget for every suite (default: AEGIS's 300 / 550)")
    args = parser.parse_args()
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    results = out / "results.jsonl"
    done = set()
    if results.exists():
        for line in results.read_text().splitlines():
            row = json.loads(line)
            done.add((row["suite"], row["task"], row["episode"]))

    # The original loads GroundingDINO from ./GroundingDINO relative to its cwd.
    work = out / "work"
    (work / "GroundingDINO").mkdir(parents=True, exist_ok=True)
    import groundingdino

    for name, src in (
        ("GroundingDINO_SwinT_OGC.py", Path(groundingdino.__file__).parent / "config/GroundingDINO_SwinT_OGC.py"),
        ("groundingdino_swint_ogc.pth", ROOT / "checkpoints/groundingdino/groundingdino_swint_ogc.pth"),
    ):
        link = work / "GroundingDINO" / name
        if not link.exists():
            link.symlink_to(src)
    os.chdir(work)
    sys.path[:0] = [str(AEGIS)]
    M = load_main_aegis(args.max_steps)

    M.obstacle_detection = codex_obstacle_detection
    port = 8000
    if args.policy == "official":
        server, port = start_openpi(out)
        stop = lambda: os.killpg(server.pid, 15)  # noqa: E731
        # The first inference compiles the JAX model; done inside an episode, it can
        # outlast the websocket keepalive and end that episode before its first step.
        import numpy as np

        M._websocket_client_policy.WebsocketClientPolicy("0.0.0.0", port).infer({
            "observation/image": np.zeros((224, 224, 3), np.uint8),
            "observation/wrist_image": np.zeros((224, 224, 3), np.uint8),
            "observation/state": np.zeros(8, np.float32),
            "prompt": "warm up",
        })
    else:
        M._websocket_client_policy.WebsocketClientPolicy = RpentPolicy
        daemon, RpentPolicy.model = start_vla(out)
        stop = daemon.stop
    capture = _Capture()
    logging.getLogger().addHandler(capture)
    logging.getLogger().setLevel(logging.INFO)
    try:
        for cell in args.cells:
            suite, task = cell.split(":")
            base, level = suite.rsplit("_", 1)
            for episode in args.episodes:
                if (suite, int(task), episode) in done:
                    continue
                capture.lines.clear()
                printed = io.StringIO()
                t0 = time.time()

                class Tee(io.TextIOBase):
                    def write(self, s):
                        sys.__stdout__.write(s)
                        printed.write(s)
                        return len(s)

                with contextlib.redirect_stdout(Tee()):
                    M.eval_libero(M.Args(
                        task_suite_name=base, safety_level=level, task_index=[int(task)],
                        episode_index=[episode], video_out_path=str(out / "videos"), port=port,
                    ))
                text = "\n".join(capture.lines) + "\n" + printed.getvalue()
                grab = lambda pat: (re.findall(pat, text) or [None])[-1]  # noqa: E731
                steps = grab(r"Time steps: \[(\d+)\]")
                row = {
                    "suite": suite, "task": int(task), "episode": episode,
                    "policy": args.policy, "max_steps": args.max_steps,
                    "obstacle_named": grab(r"obstacle_detection \(codex\): (.+) \("),
                    "success": grab(r"Success: (True|False)") == "True",
                    "collided": grab(r"Collision: (True|False)") == "True",
                    "collide_time": None if grab(r"collide_time: (\d+)") is None else int(grab(r"collide_time: (\d+)")),
                    "steps": None if steps is None else int(steps),
                    "no_feasible_qp": text.count("No feasible solution"),
                    "error": grab(r"Caught exception: (.+)"),
                    "wall_s": round(time.time() - t0, 1),
                }
                with results.open("a") as handle:
                    handle.write(json.dumps(row) + "\n")
                print("DONE", json.dumps(row), flush=True)
    finally:
        stop()


if __name__ == "__main__":
    main()
