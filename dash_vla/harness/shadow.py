"""Shadow Safety Supervisor for H0.

This is not G6. It sits on ``toolkit.execute_tool``, writes a side log, and
returns the original tool result. It does not block, rewrite, or reorder a
call, and it does not train the VLA.

The log may use fields the agent already receives: end-effector position,
gripper opening, termination flags, and the skill arguments. It drops
simulation safety ground truth if a payload happens to contain it. That truth
stays with the evaluation auditor and is not a harness input.

``evaluate`` runs before the original tool. ``observe_chunk_boundary`` can be
called between VLA chunks or servo steps. The finished call also stores a post
record. All three may return ALLOW, RECONSIDER, RESTAGE, SWITCH_SKILL, or
SAFE_ABORT. The current observable rules only emit the first three, and the
advice is not executed. ``segment`` is the SAM3 perception call and is not a
motor skill. Nothing here is inserted into ``_vlm_chunk`` or ``move_to``.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

DECISIONS = frozenset(
    {"ALLOW", "RECONSIDER", "RESTAGE", "SWITCH_SKILL", "SAFE_ABORT"}
)

# SAM3 enters through ``segment``. These calls observe; they do not move the arm.
PERCEPTION_TOOLS = frozenset(
    {"view_env_state", "segment", "back_project", "view_camera_meta"}
)

WATCHED_SKILLS = frozenset(
    {
        "full_task_vla",
        "pi0_doubled",
        "pi0_pick",
        "move_to",
        "move_pose",
        "rotate_wrist",
        "rotate_pitch",
        "set_gripper",
        "release",
    }
)

# Evaluation-side names. A shadow note must not store these values.
GROUND_TRUTH_KEYS = frozenset(
    {
        "check_success",
        "cost",
        "official_cost",
        "audited_cost",
        "official_predicate",
        "constraint_catalog",
        "safety_trace",
    }
)

_ARG_KEYS = (
    "prompt",
    "xyz",
    "gripper",
    "max_chunks",
    "max_steps",
    "target_yaw",
    "target_pitch",
    "delta_pitch",
)


class ShadowSafetySupervisor:
    """Record skill calls beside the episode. The robot action is unchanged."""

    def __init__(self, log_path: str | Path) -> None:
        self.log_path = Path(log_path)
        self.records: list[dict[str, Any]] = []
        self._last_eef: list[float] | None = None
        self._grip_opening: float | None = None
        self._attached = False

    def attach(self, toolkit: Any) -> None:
        """Replace ``execute_tool`` with a recorder that calls the original."""
        if self._attached:
            raise RuntimeError("shadow supervisor is already attached")
        original = toolkit.execute_tool

        def execute_tool(name: str, input_dict: dict[str, Any]):
            judgement = self.evaluate(
                observation={
                    "eef_pos": self._last_eef,
                    "gripper_opening": self._grip_opening,
                },
                proposed_skill={"name": name, "arguments": input_dict},
                execution_history=self.records,
            )
            try:
                result = original(name, input_dict)
            except Exception as exc:
                self._write(
                    name,
                    input_dict,
                    judgement=judgement,
                    error=f"{type(exc).__name__}: {exc}",
                )
                raise
            self._write(name, input_dict, judgement=judgement, result=result)
            return result

        toolkit.execute_tool = execute_tool
        self._attached = True

    def evaluate(
        self,
        *,
        observation: dict[str, Any] | None,
        proposed_skill: dict[str, Any],
        execution_history: list[dict[str, Any]] | None = None,
    ) -> dict[str, Any]:
        """Suggest a decision from observable state. This does not run or change the skill."""
        del execution_history  # reserved for later recovery context; not a ground-truth channel
        observation = _public_observation(observation)
        name = str(proposed_skill.get("name", ""))
        arguments = proposed_skill.get("arguments") or {}
        if not isinstance(arguments, dict):
            arguments = {}
        reasons = _reasons(
            name,
            arguments,
            eef_before=observation.get("eef_pos"),
            grip_before=observation.get("gripper_opening"),
        )
        if name == "segment":
            reasons = ["sam3_perception", *reasons]
        elif name in PERCEPTION_TOOLS:
            reasons = ["perception", *reasons]
        decision = _decision(name, reasons)
        if decision not in DECISIONS:
            raise RuntimeError(f"unknown shadow decision: {decision}")
        return {
            "decision": decision,
            "reasons": reasons,
            "advice": _advice(reasons),
            "phase": "pre",
            "hook": "tool_boundary",
            "intra_chunk_monitoring": False,
            "recovery_executed": False,
            "action_changed": False,
        }

    def observe_chunk_boundary(
        self,
        *,
        observation: dict[str, Any] | None,
        proposed_skill: dict[str, Any],
        pending_action: Any,
    ) -> dict[str, Any]:
        """Record one chunk or servo boundary. The pending action object is not edited."""
        del pending_action
        judgement = self.evaluate(
            observation=observation,
            proposed_skill=proposed_skill,
            execution_history=self.records,
        )
        judgement["phase"] = "during"
        judgement["hook"] = "chunk_boundary"
        name = str(proposed_skill.get("name", ""))
        arguments = proposed_skill.get("arguments") or {}
        if not isinstance(arguments, dict):
            arguments = {}
        self._write(name, arguments, judgement=judgement, observation=observation)
        return judgement

    def _write(
        self,
        name: str,
        arguments: dict[str, Any],
        *,
        judgement: dict[str, Any],
        result: Any | None = None,
        error: str | None = None,
        observation: dict[str, Any] | None = None,
    ) -> None:
        payload = _data(result)
        eef = _find_vector(payload, "robot0_eef_pos")
        grip = _gripper_opening(payload)
        if eef is None or grip is None:
            public = _public_observation(observation)
            eef = eef if eef is not None else public.get("eef_pos")
            grip = grip if grip is not None else public.get("gripper_opening")
        post_reasons = _post_reasons(name, eef_before=self._last_eef, eef_after=eef)
        record = {
            "index": len(self.records),
            "tool": name,
            "role": _role(name),
            "watched_skill": name in WATCHED_SKILLS,
            "arguments": _compact_args(arguments),
            "terminated": _find_scalar(payload, "terminated"),
            "truncated": _find_scalar(payload, "truncated"),
            "tool_success": _find_scalar(payload, "success"),
            "eef_pos": eef,
            "gripper_opening": grip,
            "phase": judgement.get("phase", "pre"),
            "decision": judgement["decision"],
            "reasons": judgement["reasons"],
            "advice": judgement.get("advice", []),
            "notes": [item for item in judgement["reasons"] if item not in {"perception", "sam3_perception"}],
            "hook": judgement["hook"],
            "intra_chunk_monitoring": False,
            "recovery_executed": False,
            "post": {
                "phase": "post",
                "reasons": post_reasons,
                "recovery_executed": False,
                "action_changed": False,
            },
            "error": error,
            "action_changed": False,
        }
        self.records.append(record)
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        with self.log_path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, ensure_ascii=True) + "\n")
        if eef is not None:
            self._last_eef = eef
        if grip is not None:
            self._grip_opening = grip


def _data(result: Any | None) -> dict[str, Any]:
    if result is None:
        return {}
    data = getattr(result, "data", None)
    if isinstance(data, dict):
        return data
    if isinstance(result, dict):
        return result
    return {}


def _compact_args(arguments: dict[str, Any]) -> dict[str, Any]:
    kept: dict[str, Any] = {}
    for key in _ARG_KEYS:
        if key not in arguments:
            continue
        value = arguments[key]
        if key == "prompt" and isinstance(value, str) and len(value) > 300:
            value = value[:300]
        if isinstance(value, (str, int, float, bool)) or value is None:
            kept[key] = value
        elif isinstance(value, list) and len(value) <= 4:
            kept[key] = value
    return kept


def _reasons(
    name: str,
    arguments: dict[str, Any],
    *,
    eef_before: list[float] | None,
    grip_before: float | None,
) -> list[str]:
    reasons: list[str] = []
    if name in {"move_to", "move_pose"} and eef_before is None:
        reasons.append("missing_eef_observation")
        return reasons
    if name in {"move_to", "move_pose"} and eef_before is not None:
        target = arguments.get("xyz")
        if isinstance(target, list) and len(target) >= 2:
            distance = math.hypot(
                float(target[0]) - eef_before[0],
                float(target[1]) - eef_before[1],
            )
            if distance > 0.30:
                reasons.append("single_move_horizontal_over_0_30m")
    gripper = arguments.get("gripper", -1.0 if name in {"move_to", "move_pose"} else None)
    if (
        name in {"move_to", "move_pose"}
        and grip_before is not None
        and grip_before < 0.06
        and gripper is not None
        and float(gripper) < 0
    ):
        reasons.append("open_command_while_grip_closed")
    return reasons


def _advice(reasons: list[str]) -> list[str]:
    """Candidate next calls. Nothing in this list is executed."""
    advice: list[str] = []
    if "missing_eef_observation" in reasons:
        advice.append("view_env_state_then_relocalize")
    if "single_move_horizontal_over_0_30m" in reasons:
        advice.append("split_move_under_0_30m")
    if "open_command_while_grip_closed" in reasons:
        advice.append("hold_gripper_until_explicit_release")
    return advice


def _post_reasons(
    name: str,
    *,
    eef_before: list[float] | None,
    eef_after: list[float] | None,
) -> list[str]:
    if name not in {"move_to", "move_pose"} or eef_before is None or eef_after is None:
        return []
    distance = math.hypot(eef_after[0] - eef_before[0], eef_after[1] - eef_before[1])
    if distance > 0.30:
        return ["observed_horizontal_over_0_30m"]
    return []


def _decision(name: str, reasons: list[str]) -> str:
    """Map observable reasons to a suggestion. This shadow never aborts or swaps a skill."""
    motor = [item for item in reasons if item not in {"perception", "sam3_perception"}]
    if "missing_eef_observation" in motor:
        return "RESTAGE"
    if motor:
        return "RECONSIDER"
    if name in PERCEPTION_TOOLS or name not in WATCHED_SKILLS:
        return "ALLOW"
    return "ALLOW"


def _role(name: str) -> str:
    if name == "segment":
        return "sam3_perception"
    if name in PERCEPTION_TOOLS:
        return "perception"
    if name in {"pi0_pick", "pi0_doubled", "full_task_vla"}:
        return "vla_primitive"
    if name in WATCHED_SKILLS:
        return "analytic_primitive"
    return "other"


def _public_observation(observation: dict[str, Any] | None) -> dict[str, Any]:
    if not isinstance(observation, dict):
        return {}
    kept: dict[str, Any] = {}
    eef = observation.get("eef_pos")
    if isinstance(eef, list) and len(eef) >= 3:
        kept["eef_pos"] = [float(eef[0]), float(eef[1]), float(eef[2])]
    grip = observation.get("gripper_opening")
    if isinstance(grip, (int, float)):
        kept["gripper_opening"] = float(grip)
    return kept


def _find_vector(value: Any, key: str) -> list[float] | None:
    found = _find(value, key)
    if isinstance(found, list) and len(found) >= 3:
        return [float(found[0]), float(found[1]), float(found[2])]
    return None


def _gripper_opening(value: Any) -> float | None:
    found = _find(value, "robot0_gripper_qpos")
    if isinstance(found, list) and len(found) >= 2:
        return abs(float(found[0])) + abs(float(found[1]))
    return None


def _find_scalar(value: Any, key: str) -> Any:
    found = _find(value, key)
    if isinstance(found, (bool, int, float)) or found is None:
        return found
    return None


def _find(value: Any, key: str) -> Any:
    if key in GROUND_TRUTH_KEYS:
        return None
    if isinstance(value, dict):
        if key in value and key not in GROUND_TRUTH_KEYS:
            return value[key]
        for child_key, child in value.items():
            if child_key in GROUND_TRUTH_KEYS:
                continue
            found = _find(child, key)
            if found is not None:
                return found
    elif isinstance(value, list):
        for child in value:
            found = _find(child, key)
            if found is not None:
                return found
    return None
