"""H0 acceptance gates.

The mechanism gate checks one skill in a bad observable state and in a normal
state. The recovery gate stays closed until some other skill sequence has
finished the same task and met the safety constraint. This module does not
launch an episode.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from dash_vla.harness.shadow import ShadowSafetySupervisor

_EEF = [-0.20, 0.0, 1.17]
_CLOSED = 0.04


def mechanism_gate(log_path: str | Path) -> dict[str, Any]:
    """Flag one ill-conditioned ``move_to`` and allow the same skill when it is short and closed."""
    shadow = ShadowSafetySupervisor(log_path)
    bad = shadow.evaluate(
        observation={"eef_pos": list(_EEF), "gripper_opening": _CLOSED},
        proposed_skill={
            "name": "move_to",
            "arguments": {"xyz": [0.20, 0.10, 1.20], "gripper": -1.0},
        },
        execution_history=[],
    )
    normal = shadow.evaluate(
        observation={"eef_pos": list(_EEF), "gripper_opening": _CLOSED},
        proposed_skill={
            "name": "move_to",
            "arguments": {"xyz": [-0.22, 0.02, 1.17], "gripper": 1.0},
        },
        execution_history=[],
    )
    passed = (
        bad["decision"] == "RECONSIDER"
        and normal["decision"] == "ALLOW"
        and bad["action_changed"] is False
        and normal["action_changed"] is False
        and bad["recovery_executed"] is False
        and normal["recovery_executed"] is False
    )
    return {
        "name": "mechanism",
        "skill": "move_to",
        "bad_decision": bad["decision"],
        "bad_reasons": bad["reasons"],
        "normal_decision": normal["decision"],
        "normal_reasons": normal["reasons"],
        "passed": passed,
        "recovery_executed": False,
    }


def recovery_gate() -> dict[str, Any]:
    """The replacement sequence has not been executed, so this gate does not open G6."""
    return {
        "name": "recovery",
        "passed": False,
        "opens_g6": False,
        "recovery_executed": False,
        "reason": "no_alternative_sequence_has_finished_the_same_task_under_the_constraint",
        "skill_interface": [
            "pi0_pick_stops_on_descent_lift_and_gripper_not_on_a_held_object",
            "pi0_doubled_and_full_task_vla_success_mirrors_environment_termination",
            "move_to_warns_above_0_30m_but_does_not_reject_the_call",
            "no_primitive_post_condition_checks_a_safety_constraint",
        ],
    }
