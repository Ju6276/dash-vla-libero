"""G5-OOD-1 pilot: official safety π0.5 on two held-out L2 stove scenes.

Same task sentence as the L0 stove baseline, different constraint objects.
Does not overwrite L4, L5.0, L5.2b, or L5.3. Not a formal statistic and
not G6. The per-constraint auditor is read after the policy returns.
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

CASES = (
    (
        "obstacle_avoidance",
        10,
        "L2 stove bowl, held-out sake bottle. Same sentence as L0.",
    ),
    (
        "obstacle_avoidance_human",
        10,
        "L2 stove bowl, held-out hand-held water bottle. Same sentence as L0.",
    ),
)


def run_case(model, suite: str, task: int, seed: int, why: str) -> dict:
    output = ROOT / "runs" / "g5_libero" / "ood1" / f"{suite}_t{task}_s{seed}"
    report = {
        "role": "g5_ood1_pilot",
        "formal_statistic": False,
        "g6_started": False,
        "hypothesis": "L2 held out of LIBERO-Safety fine-tuning",
        "why": why,
        "suite": suite,
        "task": task,
        "seed": seed,
        "skill": "LiberoPrimitives.pi0_doubled",
        "execution": l50._execution_config(),
    }
    daemons = []
    try:
        daemons, env = l50._open_env(suite, task, seed, output, 300)
        language = env._client.call("env.get_task_language")
        from robots.libero.tools import LiberoPrimitives

        primitives = LiberoPrimitives(
            env, model, sam3_client=None, check_cancelled=lambda: None
        )
        primitives.set_obs(env.last_obs)
        primitives.start_recording()
        result = primitives.pi0_doubled(str(language), max_chunks=30)
        trace = env._client.call("env.safety_trace")
        keys = l50._positive_keys(trace)
        success = bool(trace["check_success"])
        report.update(
            {
                "language": language,
                "primitive_result": result.data,
                "task_success": success,
                "class": l50._classify(success, bool(keys)),
                "control_violation_keys": keys,
                "control_steps": trace["control_steps"],
                "settle_audited_positive_steps": trace["settle_audited_positive_steps"],
                "init_index": trace["init_index"],
                "init_state_path": trace["init_state_path"],
                "bddl_path": trace["bddl_path"],
                "eef_start": trace.get("eef_start"),
                "eef_end": trace.get("eef_end"),
            }
        )
        (output / "safety_audit.json").write_text(json.dumps(trace, default=str))
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        report["traceback"] = traceback.format_exc()
    finally:
        l50._stop(daemons)
    (output / "summary.json").write_text(json.dumps(report, indent=2, default=str))
    print(json.dumps({k: report[k] for k in report if k != "primitive_result"}, default=str)[:2000])
    return report


def main() -> None:
    vla_dir = ROOT / "runs" / "g5_libero" / "ood1" / "vla"
    vla_dir.mkdir(parents=True, exist_ok=True)
    daemons, runtime = _init_runtime(
        l50._args("obstacle_avoidance", 10, 0, 300),
        vla_dir,
        NullDashboardEventSink(),
        {"vla"},
    )
    reports = []
    try:
        for suite, task, why in CASES:
            reports.append(run_case(runtime["model"], suite, task, 0, why))
    finally:
        l50._stop(daemons)
    summary = {
        "formal_statistic": False,
        "g6_started": False,
        "cases": [
            {
                "suite": item["suite"],
                "task": item["task"],
                "seed": item["seed"],
                "class": item.get("class"),
                "task_success": item.get("task_success"),
                "control_steps": item.get("control_steps"),
                "control_violation_keys": item.get("control_violation_keys"),
                "error": item.get("error"),
            }
            for item in reports
        ],
    }
    out = ROOT / "runs" / "g5_libero" / "ood1" / "summary.json"
    out.write_text(json.dumps(summary, indent=2) + "\n")
    if any(item.get("error") for item in reports):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
