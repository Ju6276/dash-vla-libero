"""Planner side of the DASH-VLA safety filter (robots/libero/cbf_filter.py).

The filter in the env process keeps the hand, fingers and held object away
from every object the planner has not marked. This layer
  - gives the planner mark_object: ground the grasp target or the place with
    SAM 3 and exempt its box from the filter;
  - tells the filter which marked object the gripper holds, using the same rule
    as the B2 layer (a closed gripper 3-60 mm wide holds the nearest target);
  - ends a motion the filter has held back for a whole window and returns why,
    so the planner can choose another route instead of pushing on;
  - adds the filter's interventions to every tool result;
  - logs every stepping call with the filter's record to RPENT_CBF_LOG for the
    memory step (process and episode identify the attempt).
"""

from __future__ import annotations

import json
import os
from typing import Any, Literal

import numpy as np

from robots.libero.tools import _select_segment_artifacts
from rpent.session import EnvState
from rpent.tools import ToolResult, tool

MOTION_TOOLS = ("move_to", "move_pose")
VLA_TOOLS = ("pi0_pick", "pi0_doubled")
STEPPING_TOOLS = MOTION_TOOLS + VLA_TOOLS + ("release", "set_gripper", "execute_action")
HOLDING_OPENING = 0.003  # m; a closed gripper wider than this holds something (as in B2)
HOLDING_MAX = 0.06
HOLD_RADIUS = 0.06       # m; the held object is the marked target this close to the gripper


class CbfBlocked(Exception):
    pass


def _r(values) -> list[float] | None:
    return None if values is None else [round(float(v), 4) for v in values]


class CbfLayer:
    def __init__(self, primitives) -> None:
        self.primitives = primitives
        self.marks: dict[str, dict] = {}
        self.held: str | None = None
        self._log = os.environ.get("RPENT_CBF_LOG")
        # RPENT_CBF=aegis: AEGIS's filter (aegis_cbf.py) keeps the hand off one
        # obstacle named at reset; there is nothing to mark.
        self.marking = os.environ.get("RPENT_CBF") == "1"
        primitives._step_monitor = self._check
        primitives._chunk_monitor = lambda actions: self._check()

    def _call(self, method: str, *args):
        return self.primitives.env._client.call(method, args=args, timeout_s=30.0)

    def clear(self) -> None:
        """Episode reset: the env forgets its marks, so does the layer."""
        self.marks.clear()
        self.held = None

    def _check(self) -> None:
        if self._call("env.cbf_stats", False).get("blocked_now"):
            raise CbfBlocked()

    # ---- tool gate ---------------------------------------------------------

    def gate(self, name: str, handler):
        if name not in STEPPING_TOOLS:
            return handler

        def gated(**kwargs: Any) -> ToolResult:
            primitives = self.primitives
            start = np.asarray(primitives._last_obs_eef_pos, float).copy()
            self._call("env.cbf_stats", True)
            blocked = False
            try:
                result = handler(**kwargs)
            except CbfBlocked:
                blocked = True
                result = None
            stats = self._call("env.cbf_stats", True)
            eef = np.asarray(primitives._last_obs_eef_pos, float)
            summary = {
                "intervened_steps": int(stats["interventions"]),
                "of_steps": int(stats["steps"]),
                "min_clearance_m": None if stats["min_clearance_m"] is None else round(float(stats["min_clearance_m"]), 4),
                "closest_unmarked_point": _r(stats["closest_point"]),
                "closest_part": stats["closest_element"],
            }
            if blocked:
                part = "held object" if stats["closest_element"] == "held" else "hand"
                other = "an unmarked object" if self.marking else "the obstacle"
                advice = (
                    "Do not repeat this motion. Approach from another side, pass higher, or turn the wrist so the hand fits."
                    + (" If that point belongs to the object to grasp or to the place, mark it with mark_object." if self.marking else "")
                )
                result = ToolResult(
                    data={
                        "name": name,
                        "blocked": True,
                        "reason": f"the safety filter held the arm back: the {part} cannot get closer to {other} near {summary['closest_unmarked_point']}",
                        "eef_pos": _r(eef),
                        "safety_filter": summary,
                        "terminated": primitives.env.terminated,
                        "truncated": primitives.env.truncated,
                        "next": advice,
                    }
                )
            elif summary["intervened_steps"]:
                result.data["safety_filter"] = summary
            self._update_held(name, kwargs)
            if self._log:
                record = {
                    "pid": os.getpid(),
                    "episode": primitives._episode,
                    "tool": name,
                    "args": {k: v for k, v in kwargs.items() if k in ("xyz", "prompt", "target_yaw", "gripper")},
                    "eef_start": _r(start),
                    "eef_end": _r(eef),
                    "blocked": blocked,
                    "held": self.held,
                    "marks": sorted(self.marks),
                    **summary,
                }
                with open(self._log, "a", encoding="utf-8") as handle:
                    handle.write(json.dumps(record, default=str) + "\n")
            return result

        return gated

    def _update_held(self, name: str, kwargs: dict) -> None:
        opening = float(self.primitives._last_obs_gripper)
        holding = HOLDING_OPENING < opening < HOLDING_MAX
        held = self.held
        closing = name in ("pi0_pick",) or (name == "set_gripper" and float(kwargs.get("gripper", -1)) > 0)
        if closing and holding:
            held = self._nearest_mark()
        if name == "release" or (name == "set_gripper" and float(kwargs.get("gripper", -1)) < 0) or not holding:
            held = None
        if held != self.held:
            self.held = held
            self._call("env.cbf_hold", held)

    def _nearest_mark(self) -> str | None:
        eef = np.asarray(self.primitives._last_obs_eef_pos, float)
        best, best_d = None, HOLD_RADIUS
        for label, box in self.marks.items():
            if box["role"] != "target":
                continue
            d = float(np.linalg.norm(eef - np.clip(eef, box["min"], box["max"])))
            if d < best_d:
                best, best_d = label, d
        return best

    # ---- tools ---------------------------------------------------------------

    @tool(readonly=True, exclude=("state",))
    def mark_object(
        self,
        label: str,
        role: Literal["target", "place"],
        prompt: str = "",
        camera: Literal["agentview", "wrist"] = "agentview",
        *,
        state: EnvState,
    ) -> ToolResult:
        """Mark the object you will grasp (role='target') or the place you will put it (role='place') so the safety filter lets the hand reach it. The object is grounded with SAM 3 as a world-frame box; marking the same label again replaces its box. Every unmarked object stays protected.

        Args:
            label: Short name for this object, reused when re-marking.
            role: target or place.
            prompt: SAM 3 text prompt (defaults to label).
            camera: Camera artifact to segment (default agentview).
        """
        record = state.get(-1)
        image_name, world_name, _ = _select_segment_artifacts(state, record, camera)
        if image_name is None:
            return ToolResult(data={"error": "no segment artifacts for this step"})
        data = self.primitives._sam3_client.segment(
            state.load_bytes(image_name, step=record.step_idx),
            text_prompt=prompt or label,
            point=None,
            min_score=0.2,
        )
        if not data.found or not isinstance(data.mask, np.ndarray):
            return ToolResult(data={"found": False, "error": data.reason or "no mask"})
        world = state.load(world_name, step=record.step_idx)
        points = world[np.where(data.mask)].astype(np.float64)
        points = points[np.isfinite(points).all(axis=1) & (np.abs(points).sum(axis=1) > 1e-6)]
        if len(points) < 10:
            return ToolResult(data={"found": False, "error": "too few depth points"})
        lo, hi = np.percentile(points, 2, axis=0), np.percentile(points, 98, axis=0)
        self.marks[label] = {"label": label, "role": role, "min": _r(lo), "max": _r(hi)}
        self._call("env.cbf_mark", label, self.marks[label]["min"], self.marks[label]["max"])
        score = None if data.score is None else round(float(data.score), 3)
        return ToolResult(data={"found": True, "score": score, **self.marks[label]})

    @tool(readonly=True)
    def unmark_object(self, label: str) -> ToolResult:
        """Remove a mark: the object becomes protected by the safety filter again.

        Args:
            label: Label given to mark_object.
        """
        if label not in self.marks:
            return ToolResult(data={"error": f"unknown label {label}"})
        if self.held == label:
            return ToolResult(data={"error": f"{label} is held; release it first"})
        self.marks.pop(label)
        self._call("env.cbf_unmark", label)
        return ToolResult(data={"unmarked": label})
