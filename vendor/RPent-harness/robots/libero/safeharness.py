"""SafeHarness obstacle layer, reimplemented from the paper (arXiv 2609.20822).

SafeHarness has no public code. This implements the rules the paper states and
adds only what is needed to run them; each addition is listed below.

Stated in the paper and implemented here:
  - target and obstacle are grounded as boxes by SAM 3 segmentation and kept
    outside the conversation; a box is re-derived when grounded again
  - the planner proposes a route as a polyline of waypoints that ends at a
    pre-grasp or release pose; every segment is checked before it runs, and a
    segment collides if the gripper, or the held object, passes within 12 cm of
    an obstacle or less than 5 cm above it
  - during execution, motion halts when the arm comes closer to an obstacle
    than the checked route predicted, and a stop more than 3 cm short of a
    waypoint is reported as a stall; the planner replans from that pose
  - the frozen VLA is invoked only once the gripper is within 3 cm of the target
  - the contact step that follows a route (the VLA grasp, or lowering the
    payload over the place) is not route-checked; at each contact, when the
    gripper footprint is within 2 cm of an obstacle, the gripper is reoriented
    so its finger opening is tangent to the obstacle and the payload is lowered
    vertically; contact_check reports the approach direction farthest from it

Added so the rules can run (the paper does not specify them):
  - box = 2nd-98th percentile of the mask's world points (depth outliers)
  - gripper footprint: the collision mesh vertices of the Panda hand, fingers
    and wrist link (link7) in the EEF frame, +-4.4 cm along the EEF x-axis and
    +-10.4 cm along the finger opening axis (EEF y), rotated by the wrist yaw
    and taken as its axis-aligned bound; lowest gripper point 1 cm below the
    EEF site. "Within 12 cm" is the horizontal gap between this footprint and
    the obstacle box; "less than 5 cm above" is the gap between the lowest
    gripper or held-object point and the obstacle top
  - a contact step is a near-vertical segment that ends within 3 cm
    (horizontally) of a grounded target or place; "within 3 cm of the target"
    for the VLA is the same horizontal test
  - "closer than predicted" allows 2 cm of servo tracking error
  - escape: a segment that starts inside the clearance zone (e.g. the robot's
    initial pose next to a tall obstacle) passes while neither gap shrinks,
    until it leaves the zone
  - motion and VLA calls wait until at least one obstacle is grounded (the
    paper's first step)
"""

from __future__ import annotations

from typing import Annotated, Any, Literal

import numpy as np
from pydantic import Field

from robots.libero.tools import _select_segment_artifacts
from rpent.session import EnvState
from rpent.tools import ToolResult, tool

HORIZONTAL_CLEARANCE = 0.12
VERTICAL_CLEARANCE = 0.05
STALL_DISTANCE = 0.03
VLA_HANDOFF = 0.03
ADJACENCY = 0.02
# Added so the rules can run (see the module docstring).
HAND_HALF_X = 0.044  # Panda hand, fingers and link7 mesh vertices, EEF x-axis
HAND_HALF_Y = 0.104  # EEF y-axis, the finger opening axis (hand body width)
GRIPPER_BELOW_EEF = 0.01
MONITOR_SLACK = 0.02
SAMPLE_SPACING = 0.005
TANGENT_DONE = 0.05  # rad; the wrist counts as already tangent within this
MOTION_TOOLS = ("move_to", "move_pose")
VLA_TOOLS = ("pi0_pick", "pi0_doubled")


class SafetyHalt(Exception):
    """Raised from the per-step monitor to stop a motion primitive."""


def _wrap_half_turn(angle: float) -> float:
    """Angle difference modulo pi (the parallel gripper is symmetric under a half turn)."""
    return float((angle + np.pi / 2) % np.pi - np.pi / 2)


def _rect_distance(xy, box: dict) -> float:
    """Horizontal distance from a point to the box footprint (0 inside)."""
    lo, hi = np.asarray(box["min"][:2]), np.asarray(box["max"][:2])
    return float(np.linalg.norm(np.maximum(0.0, np.maximum(lo - xy, xy - hi))))


class SafeHarnessLayer:
    def __init__(self, primitives: Any) -> None:
        self.primitives = primitives
        self.boxes: dict[str, dict] = {}
        self.held: str | None = None
        self._monitor_floor: float | None = None
        primitives._step_monitor = self._on_step

    def clear(self) -> None:
        self.boxes.clear()
        self.held = None
        self._monitor_floor = None

    # ---- geometry ----------------------------------------------------------

    def _obstacles(self) -> list[dict]:
        return [b for b in self.boxes.values() if b["role"] == "obstacle"]

    def _targets(self) -> list[dict]:
        return [b for b in self.boxes.values() if b["role"] in ("target", "place")]

    def _carried(self, yaws=None) -> tuple[float, float, float]:
        """Axis-aligned half extents (x, y) and depth below the EEF of gripper plus held object."""
        if yaws is None:
            yaws = (self._wrist_yaw(),)
        half_x = half_y = 0.0
        for yaw in yaws:
            c, s = abs(np.cos(yaw)), abs(np.sin(yaw))
            half_x = max(half_x, c * HAND_HALF_X + s * HAND_HALF_Y)
            half_y = max(half_y, s * HAND_HALF_X + c * HAND_HALF_Y)
        below = GRIPPER_BELOW_EEF
        if self.held and self.held in self.boxes:
            size = np.asarray(self.boxes[self.held]["max"]) - np.asarray(self.boxes[self.held]["min"])
            half_x, half_y = max(half_x, float(size[0]) / 2), max(half_y, float(size[1]) / 2)
            below += float(size[2])
        return half_x, half_y, below

    def separation(self, eef, obstacle: dict, yaws=None) -> tuple[float, float]:
        """(horizontal gap, vertical gap above the obstacle top) for the carried footprint."""
        half_x, half_y, below = self._carried(yaws)
        grown = {
            "min": [obstacle["min"][0] - half_x, obstacle["min"][1] - half_y],
            "max": [obstacle["max"][0] + half_x, obstacle["max"][1] + half_y],
        }
        horizontal = _rect_distance(np.asarray(eef[:2]), grown)
        vertical = float(eef[2]) - below - float(obstacle["max"][2])
        return horizontal, vertical

    def _point_ok(self, eef, obstacle: dict, yaws=None) -> bool:
        horizontal, vertical = self.separation(eef, obstacle, yaws)
        return horizontal >= HORIZONTAL_CLEARANCE or vertical >= VERTICAL_CLEARANCE

    def _is_contact(self, start, end) -> bool:
        """A near-vertical segment ending over a grounded target or place: the contact step."""
        start, end = np.asarray(start), np.asarray(end)
        if np.linalg.norm(end[:2] - start[:2]) > 0.01:
            return False
        return self._contact_target(end) is not None

    def check_segment(self, start, end, yaws=None) -> dict:
        """Check a straight segment; ``yaws`` are the wrist yaws it may take (default: current)."""
        start, end = np.asarray(start, float), np.asarray(end, float)
        if yaws is None:
            yaws = (self._wrist_yaw(),)
        contact = self._is_contact(start, end)
        n = max(2, int(np.linalg.norm(end - start) / SAMPLE_SPACING) + 1)
        # Escape: a segment that starts inside the clearance zone may stay in
        # it while neither gap shrinks, until it leaves the zone.
        escaping = {
            obstacle["label"]: self.separation(start, obstacle, yaws)
            for obstacle in self._obstacles()
            if not self._point_ok(start, obstacle, yaws)
        }
        worst = None
        min_sep = float("inf")
        for t in np.linspace(0.0, 1.0, n):
            point = start + t * (end - start)
            for obstacle in self._obstacles():
                horizontal, vertical = self.separation(point, obstacle, yaws)
                min_sep = min(min_sep, max(horizontal, vertical))
                label = obstacle["label"]
                if contact:
                    continue  # the contact step is governed by the adjacency rule
                if label in escaping:
                    if self._point_ok(point, obstacle, yaws):
                        del escaping[label]
                        continue
                    previous = escaping[label]
                    if horizontal >= previous[0] - 1e-4 and vertical >= previous[1] - 1e-4:
                        escaping[label] = (horizontal, vertical)
                        continue
                if not self._point_ok(point, obstacle, yaws) and worst is None:
                    worst = {
                        "obstacle": obstacle["label"],
                        "at_xyz": [round(float(v), 3) for v in point],
                        "horizontal_gap_m": round(horizontal, 3),
                        "height_above_top_m": round(vertical, 3),
                    }
        return {
            "ok": worst is None,
            "contact_segment": contact,
            "min_separation_m": round(min_sep, 3) if np.isfinite(min_sep) else None,
            "violation": worst,
        }

    # ---- execution hooks ---------------------------------------------------

    def _on_step(self) -> None:
        if self._monitor_floor is None:
            return
        eef = self.primitives._last_obs_eef_pos
        for obstacle in self._obstacles():
            if max(self.separation(eef, obstacle)) < self._monitor_floor:
                raise SafetyHalt(obstacle["label"])

    def gate(self, name: str, handler):
        """Wrap a primitive with the pre-motion check, monitor and VLA hand-off rule."""

        def gated(**kwargs: Any) -> ToolResult:
            refusal = self._refusal(name, kwargs)
            if refusal is not None:
                return ToolResult(data={"name": name, "refused": True, **refusal})
            reoriented = self._reorient_at_contact(name, kwargs)
            plan = None
            if name in MOTION_TOOLS:
                plan = self.check_segment(
                    self.primitives._last_obs_eef_pos, kwargs["xyz"], self._motion_yaws(name, kwargs)
                )
                if plan["min_separation_m"] is not None:
                    self._monitor_floor = plan["min_separation_m"] - MONITOR_SLACK
            try:
                result = handler(**kwargs)
            except SafetyHalt as halt:
                eef = self.primitives._last_obs_eef_pos
                return ToolResult(
                    data={
                        "name": name,
                        "halted": True,
                        "reason": f"arm came closer to obstacle '{halt}' than the verified route predicted",
                        "stop_eef_pos": [round(float(v), 4) for v in eef],
                        "next": "Replan the route from the stopping pose and verify it with check_route.",
                    }
                )
            finally:
                self._monitor_floor = None
            if reoriented is not None:
                result.data["reoriented"] = reoriented
            self._after(name, kwargs, result)
            return result

        return gated

    def _motion_yaws(self, name: str, kwargs: dict) -> tuple[float, ...]:
        """Wrist yaws a primitive passes through: the current one and any commanded one."""
        current = self._wrist_yaw()
        if name == "move_pose" and kwargs.get("target_yaw") is not None:
            return current, float(kwargs["target_yaw"])
        return (current,)

    def _refusal(self, name: str, kwargs: dict) -> dict | None:
        if name not in MOTION_TOOLS + VLA_TOOLS:
            return None
        if not self._obstacles():
            return {
                "error": "no obstacle grounded",
                "next": "Ground the obstacle with ground_box(role='obstacle') before any motion.",
            }
        if name in MOTION_TOOLS:
            check = self.check_segment(
                self.primitives._last_obs_eef_pos, kwargs["xyz"], self._motion_yaws(name, kwargs)
            )
            if not check["ok"]:
                return {
                    "error": "route rejected by obstacle check",
                    "check": check,
                    "next": "Insert waypoints that keep 12 cm horizontal or 5 cm vertical clearance; verify them with check_route.",
                }
            return None
        if self._contact_target(self.primitives._last_obs_eef_pos) is not None:
            return None
        return {
            "error": "VLA hand-off refused",
            "next": "Move the gripper with verified routes to within 3 cm (horizontally) of a grounded target, then call the VLA.",
        }

    def _contact_target(self, xyz) -> dict | None:
        """The grounded target or place within 3 cm of ``xyz`` horizontally."""
        xy = np.asarray(xyz, float)[:2]
        for target in self._targets():
            if _rect_distance(xy, target) <= VLA_HANDOFF:
                return target
        return None

    def _reorient_at_contact(self, name: str, kwargs: dict) -> dict | None:
        """At a contact, turn the finger opening tangent to an obstacle within 2 cm of the footprint."""
        eef = np.asarray(self.primitives._last_obs_eef_pos, float)
        if name in VLA_TOOLS:
            where = eef
        elif name in MOTION_TOOLS and self._is_contact(eef, kwargs["xyz"]):
            where = np.asarray(kwargs["xyz"], float)
        else:
            return None
        yaw = self._wrist_yaw()
        nearest = None
        for obstacle in self._obstacles():
            gap = self.separation(where, obstacle, (yaw,))[0]
            if gap <= ADJACENCY and (nearest is None or gap < nearest[1]):
                nearest = (obstacle, gap)
        if nearest is None:
            return None
        tangent = self._tangent_yaw(where, nearest[0])
        if abs(tangent - yaw) <= TANGENT_DONE:
            return None
        turn = self.primitives.rotate_wrist(target_yaw=tangent).data
        return {
            "obstacle": nearest[0]["label"],
            "footprint_gap_m": round(nearest[1], 3),
            "target_yaw_rad": round(tangent, 3),
            "steps_used": int(turn.get("steps_used") or 0),
        }

    def _tangent_yaw(self, xyz, obstacle: dict) -> float:
        """rotate_wrist yaw that puts the finger opening tangent to the obstacle.

        The opening axis is 90 degrees from the EEF x-axis, so the EEF x-axis
        points away from the obstacle; of the two equivalent yaws, the one
        nearest the current wrist yaw.
        """
        o_centre = (np.asarray(obstacle["min"][:2]) + np.asarray(obstacle["max"][:2])) / 2
        away = np.asarray(xyz, float)[:2] - o_centre
        yaw = self._wrist_yaw()
        return yaw + _wrap_half_turn(float(np.arctan2(away[1], away[0])) - yaw)

    def _wrist_yaw(self) -> float:
        """World yaw of the EEF x-axis, the angle rotate_wrist(target_yaw=...) sets.

        Read from the cached policy state (EEF axis-angle at [3:6]); the step
        monitor calls this every control step, so it must not go through RPC.
        """
        from scipy.spatial.transform import Rotation

        obs = getattr(self.primitives, "_last_obs", None)
        if obs is not None and "states" in obs:
            rotation = Rotation.from_rotvec(np.asarray(obs["states"], float).reshape(-1)[3:6])
        else:
            q = self.primitives.env.raw_obs()["robot0_eef_quat"]
            rotation = Rotation.from_quat([q[0], q[1], q[2], q[3]])
        matrix = rotation.as_matrix()
        return float(np.arctan2(matrix[1, 0], matrix[0, 0]))

    def _after(self, name: str, kwargs: dict, result: ToolResult) -> None:
        data = result.data
        if name in MOTION_TOOLS and data.get("final_dist_m", 0.0) > STALL_DISTANCE:
            data["stalled"] = True
            data["next"] = "Stopped more than 3 cm short of the waypoint. Replan from this pose."
        closed = name == "pi0_pick" and data.get("success")
        closed |= name == "set_gripper" and float(kwargs.get("gripper", -1)) > 0
        if closed:
            self.held = self._nearest_target()
        if name == "release" or (name == "set_gripper" and float(kwargs.get("gripper", -1)) < 0):
            self.held = None

    def _nearest_target(self) -> str | None:
        eef = np.asarray(self.primitives._last_obs_eef_pos)
        best, best_d = None, 0.06
        for box in self.boxes.values():
            if box["role"] != "target":
                continue
            d = _rect_distance(eef[:2], box)
            if d < best_d:
                best, best_d = box["label"], d
        return best

    # ---- tools -------------------------------------------------------------

    @tool(readonly=True, exclude=("state",))
    def ground_box(
        self,
        label: str,
        role: Literal["obstacle", "target", "place"],
        prompt: str = "",
        camera: Literal["agentview", "wrist"] = "agentview",
        *,
        state: EnvState,
    ) -> ToolResult:
        """Ground an object as a world-frame box with SAM 3 and keep it for the obstacle checks: the obstacle as role='obstacle', the object to pick as 'target', the place location as 'place'. Grounding the same label again replaces its box.

        Args:
            label: Short name for this object, reused when re-grounding.
            role: obstacle, target, or place.
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
        self.boxes[label] = {
            "label": label,
            "role": role,
            "min": [round(float(v), 4) for v in lo],
            "max": [round(float(v), 4) for v in hi],
        }
        score = None if data.score is None else round(float(data.score), 3)
        return ToolResult(data={"found": True, "score": score, **self.boxes[label]})

    @tool(readonly=True)
    def check_route(
        self,
        waypoints: Annotated[list[list[float]], Field(min_length=1)],
    ) -> ToolResult:
        """Verify a polyline route from the current gripper pose through world-frame waypoints against every grounded obstacle, for the gripper and any held object. A segment fails if it passes within 12 cm of an obstacle or less than 5 cm above it. The route ends at a pre-grasp or release pose; a final vertical segment onto a grounded target or place is the contact step and is not checked. Each move_to / move_pose is checked again before it runs.

        Args:
            waypoints: List of [x, y, z] waypoints, in execution order.
        """
        start = self.primitives._last_obs_eef_pos
        segments = []
        for waypoint in waypoints:
            segments.append({"to": waypoint, **self.check_segment(start, waypoint)})
            start = waypoint
        return ToolResult(
            data={
                "route_ok": all(s["ok"] for s in segments),
                "held_object": self.held,
                "segments": segments,
            }
        )

    @tool(readonly=True)
    def contact_check(self, label: str) -> ToolResult:
        """Adjacency test at a grounded target or place, for the current wrist yaw: whether the gripper footprint over it is within 2 cm of an obstacle, the approach direction farthest from that obstacle, and the rotate_wrist yaw that puts the finger opening tangent to it. At every contact the harness applies the tangent yaw itself when the footprint is within 2 cm.

        Args:
            label: Grounded target or place label.
        """
        if label not in self.boxes:
            return ToolResult(data={"error": f"unknown label {label}"})
        target = self.boxes[label]
        centre = (np.asarray(target["min"][:2]) + np.asarray(target["max"][:2])) / 2
        where = np.append(centre, target["max"][2])
        yaw = self._wrist_yaw()
        report = []
        for obstacle in self._obstacles():
            gap = self.separation(where, obstacle, (yaw,))[0]
            entry = {"obstacle": obstacle["label"], "footprint_gap_m": round(gap, 3), "adjacent": gap <= ADJACENCY}
            if entry["adjacent"]:
                o_centre = (np.asarray(obstacle["min"][:2]) + np.asarray(obstacle["max"][:2])) / 2
                away = centre - o_centre
                away = away / (np.linalg.norm(away) + 1e-9)
                entry["approach_from_direction_xy"] = [round(float(v), 3) for v in away]
                entry["tangent_yaw_rad"] = round(self._tangent_yaw(where, obstacle), 3)
            report.append(entry)
        return ToolResult(data={"target": label, "wrist_yaw_rad": round(yaw, 3), "obstacles": report})
