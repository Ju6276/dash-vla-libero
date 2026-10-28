"""One re-execution of the L2 sake-bottle direct VLA.

Same official sentence, same init index 0, same frozen weights. A new
process reseeds the policy at 0. Does not overwrite the OOD-1 record.
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


def _positive_steps(trace: dict) -> list[int]:
    steps = []
    for record in trace.get("step_audits", []):
        if record.get("phase") != "control":
            continue
        if any(record.get("audited_cost", {}).values()):
            steps.append(int(record["step"]))
    return steps


def main() -> None:
    label = sys.argv[1]
    output = ROOT / "runs" / "g5_libero" / "ood2a" / label
    vla_dir = output / "vla"
    vla_dir.mkdir(parents=True, exist_ok=True)
    daemons, runtime = _init_runtime(
        l50._args("obstacle_avoidance", 10, 0, 300),
        vla_dir,
        NullDashboardEventSink(),
        {"vla"},
    )
    report = {
        "role": "g5_ood2a_reexecution",
        "formal_statistic": False,
        "g6_started": False,
        "label": label,
        "suite": "obstacle_avoidance",
        "task": 10,
        "seed": 0,
        "language_source": "official task sentence",
        "skill": "LiberoPrimitives.pi0_doubled",
        "execution": l50._execution_config(),
        "does_not_overwrite": "runs/g5_libero/ood1",
    }
    env_daemons = []
    try:
        env_daemons, env = l50._open_env("obstacle_avoidance", 10, 0, output, 300)
        language = env._client.call("env.get_task_language")
        from robots.libero.tools import LiberoPrimitives

        primitives = LiberoPrimitives(
            env, runtime["model"], sam3_client=None, check_cancelled=lambda: None
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
                "chunks_used": result.data.get("chunks_used"),
                "task_success": success,
                "class": l50._classify(success, bool(keys)),
                "control_violation_keys": keys,
                "positive_control_steps": _positive_steps(trace),
                "control_steps": trace["control_steps"],
                "settle_audited_positive_steps": trace["settle_audited_positive_steps"],
                "init_index": trace["init_index"],
                "init_state_path": trace["init_state_path"],
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
