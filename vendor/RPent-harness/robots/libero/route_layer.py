"""SafeHarness obstacle layer, reimplemented from the paper (arXiv 2609.20822).

SafeHarness has no public code. This implements the rules the paper states and
adds only what is needed to run them; each addition is listed below.

Stated in the paper and implemented here (section numbers of the paper):
  - each phase begins with visual grounding: the target and the obstacle are
    grounded as boxes by SAM 3; a box is re-derived when grounded again (3.2)
  - the route is a polyline of waypoints that ends just above the contact
    target, at a pre-grasp or release pose, and is checked before it runs; a
    segment collides if the gripper, or the held object, passes within 12 cm
    of the obstacle or less than 5 cm above it (3.2, 4.1)
  - during execution, motion halts when the arm comes closer to the obstacle
    than the checked route predicted, and a stop more than 3 cm short of a
    waypoint is a stall; the planner replans from that pose (3.2, 4.1)
  - the frozen VLA is invoked only once the gripper is within 3 cm of the
    target (4.1); below the pre-grasp or release pose the gripper opens,
    descends and closes, and this contact is not route-checked (3.3); the
    motion after the grasp belongs to route planning (4.4)
  - adjacency (3.3, 4.1): at each contact, if the gripper footprint at the
    contact target is within 2 cm of the obstacle, the contact must approach
    from a direction that keeps clearance, the one farthest from the obstacle;
    if no direction does, the finger opening is turned tangent to the obstacle
    and the contact is made vertically

Added so the rules can run (the paper does not specify them):
  - box = 2nd-98th percentile of the mask's world points (depth outliers)
  - "the gripper" in the 12 cm / 5 cm check is the gripper point (the EEF site,
    lowest point 1 cm below it); a held object adds its horizontal radius (half
    its larger box side, so wrist turns do not matter) and its height below the
    gripper. A segment fails when the horizontal gap is under 12 cm and the gap
    above the obstacle top is under 5 cm. Turning the wrist in place does not
    move the gripper point, so it needs no check
  - "gripper footprint" (adjacency only): the collision mesh vertices of the
    Panda hand, fingers and wrist link (link7) in the EEF frame, +-4.4 cm along
    the EEF x-axis and +-10.4 cm along the finger opening axis (EEF y), rotated
    by the wrist yaw and taken as its axis-aligned bound
  - the contact step after a route is a vertical descent that starts at the
    end pose of a verified route and ends over a grounded target or place; a
    route may include it as its last segment, and its pre-grasp or release
    pose is then the waypoint before it; "within 3 cm of the target" for the
    VLA is a horizontal test
  - a pi0_pick call ends after the chunk in which its closing gripper settles,
    or once it carries the gripper out of the 3 cm hand-off region; contact
    skills (pi0_doubled) run as upstream
  - adjacency directions: 16 headings from the target centre; along each, the
    contact point is the nearest one (5 mm steps, within the target box) where
    the footprint is more than 2 cm from every obstacle; of these, the heading
    pointing farthest away from the obstacle is selected and suggested; a
    contact is accepted wherever the footprint keeps that clearance (and the
    gripper is within 3 cm of the target). The decision for a target
    is kept within a phase, until the gripper picks up or puts down an object
    or a box moves by more than 2 cm
  - the tangent turn holds the gripper as it is (open before a grasp); a
    planner target_yaw on that contact motion is replaced by the tangent yaw
  - the held object is the target nearest the gripper after a grasp that ends
    with the fingers closed on something (opening above 3 mm); it is never a
    contact target
  - "closer than predicted" allows 2 cm of servo tracking error
  - escape: a segment that starts inside the clearance zone (e.g. the robot's
    initial pose next to a tall obstacle) passes while neither gap shrinks,
    until it leaves the zone
  - grounding as the first step (3.2) is enforced: motion and VLA calls are
    refused until an obstacle is grounded; which object it is stays the
    planner's choice (the paper speaks of the one obstacle and does not say how
    it is identified)

Not implemented: the bound on how often the route-plan image may be read
during a route (3.2). The paper gives no number, and this harness has no
separate route-plan image; the planner sees the same camera views as B0.
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
GRIP_SETTLED = 0.002  # m of finger travel per chunk below which a closing gripper has settled
HOLDING_OPENING = 0.003  # m; a closed gripper wider than this holds something
ROUTE_END_TOLERANCE = 0.01  # m; a descent starts at the checked route's end pose
APPROACH_HEADINGS = 16  # candidate approach directions around the contact target
APPROACH_STEP = 0.005  # m; search step along a heading for the nearest clearing contact point
MOTION_TOOLS = ("move_to", "move_pose")
VLA_TOOLS = ("pi0_pick", "pi0_doubled")


class SafetyHalt(Exception):
    """Raised from the per-step monitor to stop a motion primitive."""


class ContactDone(Exception):
    """Raised after a VLA chunk once the contact step is over, to end the VLA call."""


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
        self._route_end: np.ndarray | None = None
        self._planned_end: np.ndarray | None = None
        self._contact: dict | None = None
        # One adjacency decision per contact target (paper: each contact begins
        # with one adjacency test), kept until its boxes move.
        self._contact_plans: dict[str, dict] = {}
        primitives._step_monitor = self._on_step
        primitives._chunk_monitor = self._on_chunk

    def clear(self) -> None:
        self.boxes.clear()
        self.held = None
        self._monitor_floor = None
        self._route_end = None
        self._planned_end = None
        self._contact = None
        self._contact_plans.clear()

    # ---- geometry ----------------------------------------------------------

    def _obstacles(self) -> list[dict]:
        return [b for b in self.boxes.values() if b["role"] == "obstacle"]

    def _targets(self) -> list[dict]:
        return [b for b in self.boxes.values() if b["role"] in ("target", "place")]

    def _held_extent(self) -> tuple[float, float]:
        """Horizontal radius and height of the held object (rotation-invariant bound)."""
        if self.held and self.held in self.boxes:
            size = np.asarray(self.boxes[self.held]["max"]) - np.asarray(self.boxes[self.held]["min"])
            return float(max(size[0], size[1])) / 2, float(size[2])
        return 0.0, 0.0

    def separation(self, eef, obstacle: dict) -> tuple[float, float]:
        """(horizontal gap, gap above the obstacle top) of the gripper and any held object.

        The gripper is its point (the EEF site, lowest point 1 cm below it); a
        held object adds its horizontal radius and its height below the gripper.
        """
        radius, height = self._held_extent()
        horizontal = max(0.0, _rect_distance(np.asarray(eef[:2], float), obstacle) - radius)
        vertical = float(eef[2]) - GRIPPER_BELOW_EEF - height - float(obstacle["max"][2])
        return horizontal, vertical

    def _point_ok(self, eef, obstacle: dict) -> bool:
        horizontal, vertical = self.separation(eef, obstacle)
        return horizontal >= HORIZONTAL_CLEARANCE or vertical >= VERTICAL_CLEARANCE

    def _footprint_gap(self, xyz, obstacle: dict, yaw: float) -> float:
        """Horizontal gap between the gripper footprint at ``xyz`` (wrist ``yaw``) and the obstacle."""
        c, s = abs(np.cos(yaw)), abs(np.sin(yaw))
        half_x, half_y = c * HAND_HALF_X + s * HAND_HALF_Y, s * HAND_HALF_X + c * HAND_HALF_Y
        grown = {
            "min": [obstacle["min"][0] - half_x, obstacle["min"][1] - half_y],
            "max": [obstacle["max"][0] + half_x, obstacle["max"][1] + half_y],
        }
        return _rect_distance(np.asarray(xyz[:2], float), grown)

    def _is_descent_onto_target(self, start, end) -> bool:
        start, end = np.asarray(start, float), np.asarray(end, float)
        if np.linalg.norm(end[:2] - start[:2]) > 0.01 or end[2] >= start[2]:
            return False
        return self._contact_target(end) is not None

    def _is_contact(self, start, end) -> bool:
        """The contact step: a vertical descent from the end pose of a checked route
        (the pre-grasp or release pose) over a grounded target or place."""
        start = np.asarray(start, float)
        if self._route_end is None or np.linalg.norm(start - self._route_end) > ROUTE_END_TOLERANCE:
            return False
        return self._is_descent_onto_target(start, end)

    def check_segment(self, start, end, contact: bool | None = None) -> dict:
        """Check a straight segment of the gripper (and held object) against every obstacle."""
        start, end = np.asarray(start, float), np.asarray(end, float)
        if contact is None:
            contact = self._is_contact(start, end)
        n = max(2, int(np.linalg.norm(end - start) / SAMPLE_SPACING) + 1)
        # Escape: a segment that starts inside the clearance zone may stay in
        # it while neither gap shrinks, until it leaves the zone.
        escaping = {
            obstacle["label"]: self.separation(start, obstacle)
            for obstacle in self._obstacles()
            if not self._point_ok(start, obstacle)
        }
        worst = None
        min_sep = float("inf")
        for t in np.linspace(0.0, 1.0, n):
            point = start + t * (end - start)
            for obstacle in self._obstacles():
                horizontal, vertical = self.separation(point, obstacle)
                min_sep = min(min_sep, max(horizontal, vertical))
                label = obstacle["label"]
                if contact:
                    continue  # the contact step is governed by the adjacency rule
                if label in escaping:
                    if self._point_ok(point, obstacle):
                        del escaping[label]
                        continue
                    previous = escaping[label]
                    if horizontal >= previous[0] - 1e-4 and vertical >= previous[1] - 1e-4:
                        escaping[label] = (horizontal, vertical)
                        continue
                if not self._point_ok(point, obstacle) and worst is None:
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

    def _on_chunk(self, actions) -> None:
        """End a VLA call once its contact step is over: the gripper has closed and
        settled, or the VLA has carried the gripper out of the 3 cm hand-off region."""
        contact = self._contact
        if contact is None:
            return
        opening = float(self.primitives._last_obs_gripper)
        closing = float(np.asarray(actions, float).reshape(len(actions), -1)[-1, -1]) > 0
        settled = contact["closing"] and closing and abs(opening - contact["opening"]) < GRIP_SETTLED
        contact["closing"], contact["opening"] = closing, opening
        if contact["tool"] == "pi0_pick" and settled:
            raise ContactDone("gripper closed")
        if contact["tool"] == "pi0_pick" and contact["target"] is not None:
            if _rect_distance(np.asarray(self.primitives._last_obs_eef_pos)[:2], contact["target"]) > VLA_HANDOFF:
                raise ContactDone("gripper left the 3 cm contact region")

    def gate(self, name: str, handler):
        """Wrap a primitive with the pre-motion check, monitor and VLA hand-off rule."""

        def gated(**kwargs: Any) -> ToolResult:
            reoriented = None
            vla_target = self._contact_target(self.primitives._last_obs_eef_pos) if name in VLA_TOOLS else None
            refusal = self._refusal(name, kwargs)
            if refusal is None:
                refusal, reoriented = self._contact_rule(name, kwargs)
            if refusal is not None:
                return ToolResult(data={"name": name, "refused": True, **refusal})
            if reoriented is not None and kwargs.get("target_yaw") is not None:
                # The contact keeps the tangent yaw the harness just set.
                kwargs = {**kwargs, "target_yaw": reoriented["target_yaw_rad"]}
                reoriented["planner_target_yaw_replaced"] = True
            plan = None
            if name in MOTION_TOOLS:
                plan = self.check_segment(self.primitives._last_obs_eef_pos, kwargs["xyz"])
                if plan["min_separation_m"] is not None:
                    self._monitor_floor = plan["min_separation_m"] - MONITOR_SLACK
            if name in VLA_TOOLS:
                self._contact = {
                    "tool": name,
                    "target": vla_target,
                    "closing": False,
                    "opening": float(self.primitives._last_obs_gripper),
                }
            try:
                result = handler(**kwargs)
            except ContactDone as done:
                result = self._contact_result(name, kwargs, str(done))
            except SafetyHalt as halt:
                eef = self.primitives._last_obs_eef_pos
                self._route_end = self._planned_end = None
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
                closed_on_contact = bool(self._contact and self._contact["closing"])
                self._contact = None
            if reoriented is not None:
                result.data["reoriented"] = reoriented
            self._after(name, kwargs, result, closed_on_contact)
            return result

        return gated

    def _contact_result(self, name: str, kwargs: dict, reason: str) -> ToolResult:
        """Result of a VLA call the harness ended after its contact step."""
        primitives = self.primitives
        opening = float(primitives._last_obs_gripper)
        return ToolResult(
            data={
                "name": name,
                "instruction": kwargs.get("prompt"),
                "contact_step_done": True,
                "reason": reason,
                "gripper_opening": round(opening, 4),
                "eef_pos": [round(float(v), 4) for v in primitives._last_obs_eef_pos],
                "terminated": primitives.env.terminated,
                "truncated": primitives.env.truncated,
                "next": "The VLA covers only the contact. Lift and carry along a route verified with check_route.",
            }
        )

    def _refusal(self, name: str, kwargs: dict) -> dict | None:
        if name not in MOTION_TOOLS + VLA_TOOLS:
            return None
        if not self._obstacles():
            # Paper 3.2: "The first step of a phase is visual grounding."
            return {
                "error": "nothing grounded as the obstacle",
                "next": "Begin the phase with visual grounding: ground the obstacle with ground_box(role='obstacle') and the contact target.",
            }
        if name in MOTION_TOOLS:
            check = self.check_segment(self.primitives._last_obs_eef_pos, kwargs["xyz"])
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
        """The grounded target or place within 3 cm of ``xyz`` horizontally (not the held object)."""
        xy = np.asarray(xyz, float)[:2]
        for target in self._targets():
            if target["label"] == self.held:
                continue
            if _rect_distance(xy, target) <= VLA_HANDOFF:
                return target
        return None

    def _adjacent(self, xyz, yaw: float) -> tuple[dict, float] | None:
        """The obstacle nearest the gripper footprint at ``xyz`` if within 2 cm."""
        nearest = None
        for obstacle in self._obstacles():
            gap = self._footprint_gap(xyz, obstacle, yaw)
            if gap <= ADJACENCY and (nearest is None or gap < nearest[1]):
                nearest = (obstacle, gap)
        return nearest

    def _approach(self, target: dict, yaw: float) -> dict | None:
        """Adjacency test at the contact target and, if adjacent, the clearing approach.

        A direction is a heading from the target centre; contact from it is made
        at the nearest point along it, within the target box, where the footprint
        is more than 2 cm from every obstacle. Of the directions that have such a
        point, the one pointing farthest away from the adjacent obstacle is selected.
        """
        centre = (np.asarray(target["min"][:2]) + np.asarray(target["max"][:2])) / 2
        top = float(target["max"][2])
        nearest = self._adjacent(np.append(centre, top), yaw)
        if nearest is None:
            return None
        obstacle, gap = nearest
        half = (np.asarray(target["max"][:2]) - np.asarray(target["min"][:2])) / 2
        o_centre = (np.asarray(obstacle["min"][:2]) + np.asarray(obstacle["max"][:2])) / 2
        away = centre - o_centre
        away = away / (np.linalg.norm(away) + 1e-9)
        best = None
        for heading in np.linspace(0.0, 2 * np.pi, APPROACH_HEADINGS, endpoint=False):
            d = np.array([np.cos(heading), np.sin(heading)])
            reach = min(half[i] / abs(d[i]) for i in range(2) if abs(d[i]) > 1e-9)
            point = None
            for offset in np.arange(APPROACH_STEP, reach + 1e-9, APPROACH_STEP):
                candidate = np.append(centre + d * offset, top)
                if self._adjacent(candidate, yaw) is None:
                    point = candidate
                    break
            if point is None:
                continue
            score = float(d @ away)
            if best is None or score > best[1]:
                best = (point, score, d)
        return {
            "obstacle": obstacle,
            "footprint_gap_m": gap,
            "contact_point": None if best is None else best[0],
            "approach_from_direction_xy": None if best is None else best[2],
        }

    def _forget_contact_plans(self, previous: dict | None, box: dict) -> None:
        """Drop adjacency decisions once a box they depend on moved by more than 2 cm."""
        centre = lambda b: (np.asarray(b["min"]) + np.asarray(b["max"])) / 2  # noqa: E731
        size = lambda b: np.asarray(b["max"]) - np.asarray(b["min"])  # noqa: E731
        moved = (
            previous is None
            or previous["role"] != box["role"]
            or np.abs(centre(previous) - centre(box)).max() > ADJACENCY
            or np.abs(size(previous) - size(box)).max() > ADJACENCY
        )
        if not moved:
            return
        if box["role"] == "obstacle" or (previous is not None and previous["role"] == "obstacle"):
            self._contact_plans.clear()
        else:
            self._contact_plans.pop(box["label"], None)

    def _contact_rule(self, name: str, kwargs: dict) -> tuple[dict | None, dict | None]:
        """Paper adjacency rule at a contact: (refusal, reorientation).

        Not adjacent: any contact. Adjacent with a clearing direction: the contact
        must be made at that direction's contact point. No clearing direction:
        the finger opening is turned tangent to the obstacle before a vertical
        contact.
        """
        eef = np.asarray(self.primitives._last_obs_eef_pos, float)
        if name in VLA_TOOLS:
            where = eef
        elif name in MOTION_TOOLS and self._is_contact(eef, kwargs["xyz"]):
            where = np.asarray(kwargs["xyz"], float)
        else:
            return None, None
        target = self._contact_target(where)
        if target is None:
            return None, None
        yaw = self._wrist_yaw()
        approach = self._contact_plans.get(target["label"])
        if approach is None:
            approach = self._approach(target, yaw) or {"adjacent": False}
            self._contact_plans[target["label"]] = approach
        if not approach.get("adjacent", True):
            return None, None
        obstacle = approach["obstacle"]
        point = approach["contact_point"]
        if point is not None:
            # A clearing direction exists: the contact must be made where the
            # footprint keeps clearance; the selected point is the suggestion.
            if self._adjacent(where, yaw) is None:
                return None, None
            return {
                "error": f"contact at '{target['label']}' is adjacent to obstacle '{obstacle['label']}'",
                "approach_from_direction_xy": [round(float(v), 3) for v in approach["approach_from_direction_xy"]],
                "contact_point_xy": [round(float(v), 3) for v in point[:2]],
                "next": "Make the contact from this direction: route to above contact_point_xy, verify it with check_route, then contact there.",
            }, None
        tangent = self._tangent_yaw(where, obstacle)
        if abs(_wrap_half_turn(tangent - yaw)) <= TANGENT_DONE:
            return None, None
        # Hold the gripper as it is: open before a grasp, closed over a place.
        turn = self.primitives.rotate_wrist(target_yaw=tangent, gripper=1.0 if self.held else -1.0).data
        return None, {
            "obstacle": obstacle["label"],
            "footprint_gap_m": round(approach["footprint_gap_m"], 3),
            "no_clearing_direction": True,
            "target_yaw_rad": round(tangent, 3),
            "converged": turn.get("converged"),
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

    def _after(self, name: str, kwargs: dict, result: ToolResult, closed_on_contact: bool = False) -> None:
        data = result.data
        if name in MOTION_TOOLS and data.get("final_dist_m", 0.0) > STALL_DISTANCE:
            data["stalled"] = True
            data["next"] = "Stopped more than 3 cm short of the waypoint. Replan from this pose."
        if name in MOTION_TOOLS:
            # Reaching the last waypoint of a verified route puts the gripper at
            # the pre-grasp or release pose, where the contact step may begin.
            reached = (
                self._planned_end is not None
                and not data.get("halted")
                and not data.get("stalled")
                and np.linalg.norm(np.asarray(kwargs["xyz"], float) - self._planned_end) < 1e-3
            )
            self._route_end = np.asarray(self.primitives._last_obs_eef_pos, float) if reached else None
            if reached:
                self._planned_end = None
        opening = float(self.primitives._last_obs_gripper)
        holding = HOLDING_OPENING < opening < 0.06
        held_before = self.held
        # A grasp that closed on something holds the target, whatever success
        # the VLA reported; contact skills (pi0_doubled) do not pick objects.
        if name == "pi0_pick" and closed_on_contact and holding:
            self.held = self._nearest_target()
        if name == "set_gripper" and float(kwargs.get("gripper", -1)) > 0 and holding:
            self.held = self._nearest_target()
        if name == "release" or (name == "set_gripper" and float(kwargs.get("gripper", -1)) < 0):
            self.held = None
        if self.held is not None and opening <= HOLDING_OPENING:
            self.held = None  # the fingers closed on nothing or the object slipped out
        if self.held != held_before:
            # A new phase (empty or loaded gripper): each contact is tested anew.
            self._contact_plans.clear()

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
        previous = self.boxes.get(label)
        self.boxes[label] = {
            "label": label,
            "role": role,
            "min": [round(float(v), 4) for v in lo],
            "max": [round(float(v), 4) for v in hi],
        }
        self._forget_contact_plans(previous, self.boxes[label])
        score = None if data.score is None else round(float(data.score), 3)
        return ToolResult(data={"found": True, "score": score, **self.boxes[label]})

    @tool(readonly=True)
    def check_route(
        self,
        waypoints: Annotated[list[list[float]], Field(min_length=1)],
    ) -> ToolResult:
        """Verify a polyline route from the current gripper pose through world-frame waypoints against every grounded obstacle, for the gripper and any held object. A segment fails where it is within 12 cm of an obstacle horizontally and less than 5 cm above its top. The route ends at a pre-grasp or release pose; a final vertical segment onto a grounded target or place is the contact step and is not checked. Each move_to / move_pose is checked again before it runs.

        Args:
            waypoints: List of [x, y, z] waypoints, in execution order.
        """
        start = np.asarray(self.primitives._last_obs_eef_pos, float)
        points = [start, *(np.asarray(w, float) for w in waypoints)]
        # A final vertical descent onto a grounded target or place is the contact
        # step below the pre-grasp or release pose (the point before it).
        contact_last = len(points) > 2 and self._is_descent_onto_target(points[-2], points[-1])
        segments = []
        for index, waypoint in enumerate(waypoints):
            last = index == len(waypoints) - 1
            contact = True if (last and contact_last) else None
            segments.append({"to": waypoint, **self.check_segment(points[index], waypoint, contact=contact)})
        route_ok = all(s["ok"] for s in segments)
        end = points[-2] if contact_last else points[-1]
        self._planned_end = end if route_ok else None
        return ToolResult(
            data={
                "route_ok": route_ok,
                "held_object": self.held,
                "segments": segments,
            }
        )

    @tool(readonly=True)
    def contact_check(self, label: str) -> ToolResult:
        """Adjacency test at a grounded target or place for the current wrist yaw: whether the gripper footprint at it is within 2 cm of an obstacle and, if so, the approach direction that keeps clearance and is farthest from the obstacle, with its contact point. If no direction clears, the harness turns the finger opening tangent to the obstacle itself at the contact, which is then made vertically.

        Args:
            label: Grounded target or place label.
        """
        if label not in self.boxes:
            return ToolResult(data={"error": f"unknown label {label}"})
        yaw = self._wrist_yaw()
        approach = self._contact_plans.get(label)
        if approach is None:
            approach = self._approach(self.boxes[label], yaw) or {"adjacent": False}
            self._contact_plans[label] = approach
        adjacent = approach.get("adjacent", True)
        report = {"target": label, "wrist_yaw_rad": round(yaw, 3), "adjacent": adjacent}
        if adjacent:
            report["obstacle"] = approach["obstacle"]["label"]
            report["footprint_gap_m"] = round(approach["footprint_gap_m"], 3)
            if approach["contact_point"] is not None:
                report["approach_from_direction_xy"] = [
                    round(float(v), 3) for v in approach["approach_from_direction_xy"]
                ]
                report["contact_point_xy"] = [round(float(v), 3) for v in approach["contact_point"][:2]]
            else:
                report["no_clearing_direction"] = True
        return ToolResult(data=report)
