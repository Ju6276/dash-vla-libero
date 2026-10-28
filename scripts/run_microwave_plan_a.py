"""Longer-budget capability check for the L2 microwave task, seed 0.

Same frozen pi0.5 and official task sentence as the 300-step failure.
Sixty chunks is 600 control steps. Does not overwrite the feasibility pilot.
"""

from __future__ import annotations

import importlib.util
import json
import os
import sys
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("RPENT_REPO_ROOT", str(ROOT / "vendor" / "RPent-harness"))
sys.path.insert(0, str(ROOT / "vendor" / "RPent-harness"))

_spec = importlib.util.spec_from_file_location(
    "run_l50_screen", ROOT / "scripts" / "run_l50_screen.py"
)
l50 = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(l50)

from rpent.dashboard.events import NullDashboardEventSink  # noqa: E402
from robots.libero.robot_spec import _init_runtime  # noqa: E402

MAX_CHUNKS = 60
MAX_STEPS = 600


def _positive_steps(trace: dict) -> list[int]:
    steps = []
    for record in trace.get("step_audits", []):
        if record.get("phase") != "control":
            continue
        if any(record.get("audited_cost", {}).values()):
            steps.append(int(record["step"]))
    return steps


def main() -> None:
    output = ROOT / "runs" / "g5_libero" / "microwave_cap" / "plan_a_t14_s0"
    vla_dir = output / "vla"
    vla_dir.mkdir(parents=True, exist_ok=True)
    daemons, runtime = _init_runtime(
        l50._args("obstacle_avoidance", 14, 0, MAX_STEPS),
        vla_dir,
        NullDashboardEventSink(),
        {"vla"},
    )
    report = {
        "role": "microwave_plan_a_longer_budget",
        "formal_statistic": False,
        "g6_started": False,
        "planner": False,
        "suite": "obstacle_avoidance",
        "task": 14,
        "seed": 0,
        "language_source": "official task sentence",
        "skill": "LiberoPrimitives.pi0_doubled",
        "max_chunks": MAX_CHUNKS,
        "max_episode_steps": MAX_STEPS,
        "compared_with": "ood_feas oa_t14_s0_task_only at 300 steps",
        "execution": l50._execution_config(),
        "policy_rng_state_saved": False,
    }
    env_daemons = []
    try:
        env_daemons, env = l50._open_env("obstacle_avoidance", 14, 0, output, MAX_STEPS)
        language = str(env._client.call("env.get_task_language"))
        from robots.libero.tools import LiberoPrimitives

        primitives = LiberoPrimitives(
            env, runtime["model"], sam3_client=None, check_cancelled=lambda: None
        )
        primitives.set_obs(env.last_obs)
        primitives.start_recording()
        result = primitives.pi0_doubled(language, max_chunks=MAX_CHUNKS)
        trace = env._client.call("env.safety_trace")
        keys = l50._positive_keys(trace)
        success = bool(trace["check_success"])
        try:
            (output / "final_scene.json").write_text(
                json.dumps(env._client.call("env.scene_snapshot"), default=str)
            )
        except Exception as exc:
            report["final_scene_error"] = f"{type(exc).__name__}: {exc}"
        report.update(
            {
                "language": language,
                "chunks_used": result.data.get("chunks_used"),
                "task_success": success,
                "class": l50._classify(success, bool(keys)),
                "control_violation_keys": keys,
                "positive_control_steps": _positive_steps(trace),
                "control_steps": trace["control_steps"],
                "settle_audited_positive_steps": trace["settle_audited_positive_steps"],
                "truncated_by_budget": bool(result.data.get("truncated")),
                "init_index": trace["init_index"],
                "init_state_path": trace["init_state_path"],
                "eef_start": trace.get("eef_start"),
                "eef_end": trace.get("eef_end"),
            }
        )
        (output / "safety_audit.json").write_text(json.dumps(trace, default=str))
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        report["traceback"] = traceback.format_exc()
    finally:
        l50._stop(env_daemons)
        l50._stop(daemons)
    (output / "summary.json").write_text(json.dumps(report, indent=2, default=str))
    print(json.dumps({k: report[k] for k in report if k != "traceback"}, default=str))
    if report.get("error"):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
