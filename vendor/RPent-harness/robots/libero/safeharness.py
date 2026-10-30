"""SafeHarness-style obstacle layer, reimplemented from the paper (arXiv 2609.20822).

SafeHarness has no public code. This follows the rules the paper states and
fills the gaps it leaves open; every filled gap is marked "not in the paper".

Stated in the paper and implemented here:
  - target and obstacle are grounded as boxes by SAM 3 segmentation; the boxes
    live outside the conversation (in this layer, not in the prompt)
  - every motion segment is checked before it runs, for the gripper and, when
    an object is held, for the held object; a segment collides if it passes
    within 12 cm of the obstacle or less than 5 cm above it
  - during execution, motion halts when the arm comes closer to the obstacle
    than predicted, and a stop more than 3 cm short of a waypoint is reported
    as a stall; the planner replans from the stopping pose
  - the frozen VLA is invoked only once the gripper is within 3 cm of the target
  - contact adjacency, tested automatically at each contact (VLA hand-off or
    vertical descent over a target/place): a contact target whose footprint is
    within 2 cm of the obstacle gets an approach direction that keeps clearance
    (farthest from the obstacle), or, if none exists, a gripper yaw tangent to
    the obstacle and a vertical descent

Not in the paper (choices made here):
  - box = 2nd-98th percentile of the mask's world points; repeated grounding of
    the same label (e.g. from the wrist camera) is merged by union unless the
    planner asks to replace it; a wrong box can be deleted (forget_box)
  - "within 12 cm" is horizontal footprint distance; "less than 5 cm above" is
    the gap between the lowest held point and the obstacle top
  - gripper footprint: the collision mesh vertices of the Panda hand, fingers
    and wrist link (link7) in the EEF frame, +-4.4 cm along the EEF x-axis and
    +-10.4 cm along the finger opening axis (EEF y; the hand body is that wide
    3-12 cm above the EEF), rotated by the wrist yaw and taken as its
    axis-aligned bound; a motion that changes yaw uses the union of both
    footprints; lowest gripper point 1 cm below the EEF site. The arm links
    above the wrist are not modelled, as in the paper
  - the contact segment (near-vertical, within 3 cm of a grounded target in xy)
    is exempt from the 12 cm rule but must keep the footprint 1 cm clear of the
    obstacle box; an in-place wrist rotation must keep the same 1 cm or not
    reduce the separation it starts with
  - "within 3 cm of the target" for the VLA: horizontal distance to the target
    box centre <= 3 cm and EEF at most 15 cm above the target top
  - "closer than predicted" means 2 cm below the planned minimum separation
  - escape: a segment starting inside the clearance zone (e.g. the robot's
    initial pose next to a tall obstacle) passes while its separation never
    drops below the starting separation
  - a contact that needs the tangent yaw is refused while the wrist is more
    than 15 degrees off it; the yaw is the rotate_wrist target_yaw (EEF x-axis),
    and the Panda fingers open along the axis 90 degrees from it
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
# Not in the paper.
HAND_HALF_X = 0.044  # Panda hand, fingers and link7 mesh vertices, EEF x-axis
HAND_HALF_Y = 0.104  # EEF y-axis, the finger opening axis (hand body width)
GRIPPER_BELOW_EEF = 0.01
CONTACT_CLEARANCE = 0.01
VLA_MAX_HEIGHT = 0.15
MONITOR_SLACK = 0.02
SAMPLE_SPACING = 0.005
YAW_TOLERANCE = np.deg2rad(15.0)
MOTION_TOOLS = ("move_to", "move_pose")
VLA_TOOLS = ("pi0_pick", "pi0_doubled")
ROTATION_TOOLS = ("rotate_wrist",)


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

    def _point_ok(self, eef, obstacle: dict, contact: bool, yaws=None) -> bool:
        horizontal, vertical = self.separation(eef, obstacle, yaws)
        if contact:
            return horizontal >= CONTACT_CLEARANCE or vertical >= CONTACT_CLEARANCE
        return horizontal >= HORIZONTAL_CLEARANCE or vertical >= VERTICAL_CLEARANCE

    def _is_contact(self, start, end) -> bool:
        start, end = np.asarray(start), np.asarray(end)
        if np.linalg.norm(end[:2] - start[:2]) > 0.01:
            return False
        for target in self._targets():
            centre = (np.asarray(target["min"][:2]) + np.asarray(target["max"][:2])) / 2
            if np.linalg.norm(end[:2] - centre) <= VLA_HANDOFF:
                return True
        return False

    def check_segment(self, start, end, yaws=None) -> dict:
        """Check a straight segment; ``yaws`` are the wrist yaws it may take (default: current)."""
        start, end = np.asarray(start, float), np.asarray(end, float)
        if yaws is None:
            yaws = (self._wrist_yaw(),)
        contact = self._is_contact(start, end)
        n = max(2, int(np.linalg.norm(end - start) / SAMPLE_SPACING) + 1)
        # Escape (not in the paper): a segment that starts inside the clearance
        # zone may stay in it while neither gap shrinks, until it leaves the zone.
        escaping = {
            obstacle["label"]: self.separation(start, obstacle, yaws)
            for obstacle in self._obstacles()
            if not self._point_ok(start, obstacle, contact, yaws)
        }
        worst = None
        min_sep = float("inf")
        for t in np.linspace(0.0, 1.0, n):
            point = start + t * (end - start)
            for obstacle in self._obstacles():
                horizontal, vertical = self.separation(point, obstacle, yaws)
                min_sep = min(min_sep, max(horizontal, vertical))
                label = obstacle["label"]
                if label in escaping:
                    if self._point_ok(point, obstacle, contact, yaws):
                        del escaping[label]
                        continue
                    previous = escaping[label]
                    if horizontal >= previous[0] - 1e-4 and vertical >= previous[1] - 1e-4:
                        escaping[label] = (horizontal, vertical)
                        continue
                if not self._point_ok(point, obstacle, contact, yaws) and worst is None:
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
            self._after(name, kwargs, result)
            return result

        return gated

    def _motion_yaws(self, name: str, kwargs: dict) -> tuple[float, ...]:
        """Wrist yaws a primitive passes through: the current one and any commanded one."""
        current = self._wrist_yaw()
        if name == "move_pose" and kwargs.get("target_yaw") is not None:
            return current, float(kwargs["target_yaw"])
        if name == "rotate_wrist":
            if kwargs.get("target_yaw") is not None:
                return current, float(kwargs["target_yaw"])
            return current, current + float(kwargs.get("delta_yaw") or 0.0)
        return (current,)

    def _rotation_refusal(self, kwargs: dict) -> dict | None:
        """An in-place wrist rotation keeps 1 cm or does not lose separation."""
        eef = self.primitives._last_obs_eef_pos
        yaws = self._motion_yaws("rotate_wrist", kwargs)
        for obstacle in self._obstacles():
            if self._point_ok(eef, obstacle, True, yaws):
                continue
            if max(self.separation(eef, obstacle, yaws)) >= max(self.separation(eef, obstacle, yaws[:1])) - 1e-4:
                continue
            return {
                "error": "wrist rotation refused: the hand would swing closer to an obstacle",
                "obstacle": obstacle["label"],
                "next": "Move away from the obstacle (verified route) before rotating the wrist.",
            }
        return None

    def _refusal(self, name: str, kwargs: dict) -> dict | None:
        if name in ROTATION_TOOLS:
            return self._rotation_refusal(kwargs)
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
            if check["contact_segment"]:
                return self._contact_refusal(self._contact_target(kwargs["xyz"]))
            return None
        eef = np.asarray(self.primitives._last_obs_eef_pos)
        target = self._contact_target(eef)
        if target is not None and float(eef[2]) - float(target["max"][2]) <= VLA_MAX_HEIGHT:
            return self._contact_refusal(target)
        return {
            "error": "VLA hand-off refused",
            "next": "Move the gripper with verified routes to within 3 cm (horizontal) above a grounded target, then call the VLA.",
        }

    def _contact_target(self, xyz) -> dict | None:
        """The grounded target or place whose centre is within 3 cm of ``xyz`` horizontally."""
        xy = np.asarray(xyz, float)[:2]
        for target in self._targets():
            centre = (np.asarray(target["min"][:2]) + np.asarray(target["max"][:2])) / 2
            if np.linalg.norm(xy - centre) <= VLA_HANDOFF:
                return target
        return None

    def _contact_refusal(self, target: dict | None) -> dict | None:
        """Automatic adjacency test at a contact: enforce the tangent yaw when it is required."""
        if target is None:
            return None
        report = self._adjacency(target)
        yaw = self._wrist_yaw()
        for entry in report:
            if "tangent_yaw_rad" in entry and abs(_wrap_half_turn(yaw - entry["tangent_yaw_rad"])) > YAW_TOLERANCE:
                return {
                    "error": "contact refused: an adjacent obstacle leaves no clear approach and the wrist is not tangent to it",
                    "adjacency": report,
                    "wrist_yaw_rad": round(yaw, 3),
                    "next": f"rotate_wrist(target_yaw={entry['tangent_yaw_rad']}), then descend vertically at the target centre.",
                }
        return None

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
        replace: bool = False,
        *,
        state: EnvState,
    ) -> ToolResult:
        """Ground an object as a world-frame box with SAM 3 and keep it for the obstacle checks. Ground every object that is not part of the task but sits near the route as role='obstacle'; the object to pick as 'target'; the place location as 'place'. Grounding the same label again (e.g. from the wrist camera) enlarges the box unless replace=true; use replace=true or forget_box when a mask was wrong.

        Args:
            label: Short name for this object, reused when re-grounding.
            role: obstacle, target, or place.
            prompt: SAM 3 text prompt (defaults to label).
            camera: Camera artifact to segment (default agentview).
            replace: Replace the stored box for this label instead of enlarging it (default false).
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
        if label in self.boxes and not replace:
            lo = np.minimum(lo, self.boxes[label]["min"])
            hi = np.maximum(hi, self.boxes[label]["max"])
        self.boxes[label] = {
            "label": label,
            "role": role,
            "min": [round(float(v), 4) for v in lo],
            "max": [round(float(v), 4) for v in hi],
        }
        score = None if data.score is None else round(float(data.score), 3)
        return ToolResult(data={"found": True, "score": score, **self.boxes[label]})

    @tool(readonly=True)
    def forget_box(self, label: str) -> ToolResult:
        """Delete a grounded box, e.g. after a wrong mask. Motion stays refused while no obstacle is grounded.

        Args:
            label: Grounded label to delete.
        """
        removed = self.boxes.pop(label, None)
        if removed is not None and self.held == label:
            self.held = None
        return ToolResult(data={"removed": removed is not None, "labels": sorted(self.boxes)})

    @tool(readonly=True)
    def check_route(
        self,
        waypoints: Annotated[list[list[float]], Field(min_length=1)],
    ) -> ToolResult:
        """Verify a polyline route from the current gripper pose through world-frame waypoints against every grounded obstacle, for the gripper and any held object. A segment fails if it passes within 12 cm of an obstacle or less than 5 cm above it; a final vertical contact segment over a grounded target only has to stay 2 cm clear. Each move_to / move_pose is checked again before it runs.

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
        """Adjacency test for picking or placing at a grounded target: whether an obstacle is within 2 cm of the target footprint and, if so, the approach direction farthest from the obstacle or the rotate_wrist target_yaw that puts the finger opening tangent to it for a vertical descent. The same test runs automatically at every VLA hand-off and contact descent.

        Args:
            label: Grounded target or place label.
        """
        if label not in self.boxes:
            return ToolResult(data={"error": f"unknown label {label}"})
        return ToolResult(data={"target": label, "obstacles": self._adjacency(self.boxes[label])})

    def _adjacency(self, target: dict) -> list[dict]:
        centre = (np.asarray(target["min"][:2]) + np.asarray(target["max"][:2])) / 2
        half = (np.asarray(target["max"][:2]) - np.asarray(target["min"][:2])) / 2
        report = []
        for obstacle in self._obstacles():
            gap = max(
                0.0,
                float(
                    np.linalg.norm(
                        np.maximum(
                            0.0,
                            np.maximum(
                                np.asarray(obstacle["min"][:2]) - (centre + half),
                                (centre - half) - np.asarray(obstacle["max"][:2]),
                            ),
                        )
                    )
                ),
            )
            o_centre = (np.asarray(obstacle["min"][:2]) + np.asarray(obstacle["max"][:2])) / 2
            away = centre - o_centre
            away = away / (np.linalg.norm(away) + 1e-9)
            entry = {"obstacle": obstacle["label"], "gap_m": round(gap, 3), "adjacent": gap <= ADJACENCY}
            if entry["adjacent"]:
                pre = np.append(centre + away * (half.max() + HAND_HALF_Y + 0.03), target["max"][2] + 0.1)
                # A point test: check_segment's escape rule would pass any
                # zero-length segment that starts inside the clearance zone.
                if all(self._point_ok(pre, other, False) for other in self._obstacles()):
                    entry["approach_from_xyz"] = [round(float(v), 3) for v in pre]
                else:
                    # Finger opening tangent to the obstacle, i.e. perpendicular to the
                    # away vector; the opening axis is 90 degrees from the EEF x-axis, so
                    # the EEF x-axis points along the away vector. Of the two equivalent
                    # yaws, report the one nearest the current wrist yaw.
                    yaw = self._wrist_yaw()
                    tangent = yaw + _wrap_half_turn(float(np.arctan2(away[1], away[0])) - yaw)
                    entry["tangent_yaw_rad"] = round(tangent, 3)
                    entry["then"] = "descend vertically at the target centre"
            report.append(entry)
        return report
