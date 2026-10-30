"""Unit checks of the B2 layer with a stub robot (no simulator, no planner)."""
import math
import numpy as np
from robots.libero import route_layer as rl
from rpent.tools import ToolResult


class Env:
    terminated = truncated = False


class Prims:
    def __init__(self):
        self._last_obs_eef_pos = np.array([0.0, 0.0, 1.2])
        self._last_obs_gripper = 0.08
        self._last_obs = {"states": np.array([0, 0, 1.2, math.pi, 0, 0, 0.04, -0.04], dtype=float)}  # wrist yaw 0
        self.env = Env()
        self.rotations = []

    def rotate_wrist(self, target_yaw, gripper):
        self.rotations.append((target_yaw, gripper))
        self.set_yaw(target_yaw)
        return ToolResult(data={"converged": True, "steps_used": 7})

    def set_yaw(self, yaw):
        from scipy.spatial.transform import Rotation
        rot = Rotation.from_euler("z", yaw) * Rotation.from_euler("x", math.pi)  # gripper down
        self._last_obs["states"][3:6] = rot.as_rotvec()


def box(label, role, lo, hi):
    return {"label": label, "role": role, "min": lo, "max": hi}


p = Prims()
layer = rl.SafeHarnessLayer(p)
layer.boxes["bowl"] = box("bowl", "target", [-0.05, -0.05, 0.90], [0.05, 0.05, 0.95])
layer.boxes["pot"] = box("pot", "obstacle", [0.06, -0.06, 0.90], [0.16, 0.06, 1.05])
yaw = layer._wrist_yaw()

# 1. Grounding first: no obstacle grounded -> motion and VLA refused.
saved = layer.boxes.pop("pot")
assert layer._refusal("move_to", {"xyz": [0, 0, 1.3]})["error"].startswith("nothing grounded")
assert layer._refusal("pi0_pick", {"prompt": "x"})["error"].startswith("nothing grounded")
layer.boxes["pot"] = saved
print("1 grounding first ok")

# 2. Clearance uses the gripper point (+ held object): 11 cm fails, 13 cm passes (low z).
assert not layer._point_ok([-0.05, 0.0, 0.95], layer.boxes["pot"])  # 11 cm, below the top
assert layer._point_ok([-0.07, 0.0, 0.95], layer.boxes["pot"])  # 13 cm
assert layer._point_ok([0.10, 0.0, 1.11], layer.boxes["pot"])  # 5 cm above the top (+1 cm gripper)
layer.boxes["mug"] = box("mug", "target", [-0.4, -0.04, 0.9], [-0.32, 0.04, 1.0])
layer.held = "mug"  # radius 4 cm, height 10 cm
assert not layer._point_ok([-0.07, 0.0, 0.95], layer.boxes["pot"])  # 13 - 4 = 9 cm
assert not layer._point_ok([0.10, 0.0, 1.11], layer.boxes["pot"])  # held object hangs below
layer.held = None
del layer.boxes["mug"]
print("2 point clearance ok")

# 3. Contact exemption only from the end of a verified route.
pre = np.array([0.0, 0.0, 0.98])
assert not layer._is_contact(pre, [0.0, 0.0, 0.93])
layer._route_end = pre.copy()
assert layer._is_contact(pre, [0.0, 0.0, 0.93])
assert not layer._is_contact(pre + [0, 0, 0.2], [0.0, 0.0, 0.93])
assert not layer._is_contact(pre, [0.0, 0.0, 1.05])
layer._route_end = None
print("3 contact exemption ok")

# 4. Adjacency: footprint at the bowl centre is 1.6 cm from the pot; a clearing direction
#    exists away from the pot; the suggested point is the nearest clearing one, not a corner.
a = layer._approach(layer.boxes["bowl"], yaw)
print("   direction", np.round(a["approach_from_direction_xy"], 2), "point", np.round(a["contact_point"], 3))
assert a["approach_from_direction_xy"][0] < -0.9 and abs(a["contact_point"][0]) < 0.02
p._last_obs_eef_pos = np.array([0.03, 0.0, 0.98])  # 3.5 cm from the selected point, toward the pot
refusal, turn = layer._contact_rule("pi0_pick", {"prompt": "x"})
assert refusal is not None and turn is None, (refusal, turn)
p._last_obs_eef_pos = np.array([0.02, 0.0, 0.98])  # 2.5 cm from it: within the stall tolerance
refusal, turn = layer._contact_rule("pi0_pick", {"prompt": "x"})
assert refusal is None and turn is None, (refusal, turn)
print("4 adjacency direction ok")

# 5. No clearing direction -> tangent turn with the gripper held open; kept within the phase.
layer._contact_plans.clear()
layer.boxes["w1"] = box("w1", "obstacle", [-0.16, -0.06, 0.90], [-0.06, 0.06, 1.05])
layer.boxes["w2"] = box("w2", "obstacle", [-0.06, 0.115, 0.90], [0.06, 0.22, 1.05])
layer.boxes["w3"] = box("w3", "obstacle", [-0.06, -0.22, 0.90], [0.06, -0.12, 1.05])
p._last_obs_eef_pos = np.array([0.0, 0.0, 0.98])
refusal, turn = layer._contact_rule("pi0_pick", {"prompt": "x"})
assert refusal is None and turn["no_clearing_direction"], (refusal, turn)
assert p.rotations[-1][1] == -1.0
refusal, turn = layer._contact_rule("pi0_pick", {"prompt": "x"})
assert refusal is None
print("5 tangent fallback ok", turn)

# 6. The planner's target_yaw on a contact descent is replaced by the tangent yaw.
seen = {}
def handler(**kw):
    seen.update(kw)
    return ToolResult(data={"final_dist_m": 0.001})
layer._contact_plans.clear()
p.rotations.clear()
layer._route_end = np.array([0.0, 0.0, 0.98])
p._last_obs_eef_pos = np.array([0.0, 0.0, 0.98])
out = layer.gate("move_to", handler)(xyz=[0.0, 0.0, 0.93], target_yaw=0.0)
assert not out.data.get("refused"), out.data
assert p.rotations and abs(seen["target_yaw"] - p.rotations[-1][0]) < 1e-2, (seen, p.rotations)
assert out.data["reoriented"]["planner_target_yaw_replaced"]
for k in ("w1", "w2", "w3"):
    del layer.boxes[k]
layer._contact_plans.clear()
p.set_yaw(0.0)
print("6 tangent yaw kept ok")

# 7. pi0_pick ends when the closing gripper settles or leaves the region; pi0_doubled does not
#    stop on closing; a missing target does not crash.
close = np.zeros((5, 7)); close[:, -1] = 1.0
layer._contact = {"tool": "pi0_pick", "target": layer.boxes["bowl"], "closing": False, "opening": 0.08}
p._last_obs_eef_pos = np.array([0.0, 0.0, 0.93]); p._last_obs_gripper = 0.03
layer._on_chunk(close)
p._last_obs_gripper = 0.0295
try:
    layer._on_chunk(close); raise AssertionError("should end")
except rl.ContactDone as e:
    print("   contact done:", e)
layer._contact = {"tool": "pi0_pick", "target": layer.boxes["bowl"], "closing": False, "opening": 0.08}
p._last_obs_eef_pos = np.array([0.15, 0.0, 1.0])
try:
    layer._on_chunk(np.zeros((5, 7))); raise AssertionError("should end")
except rl.ContactDone as e:
    print("   contact done:", e)
layer._contact = {"tool": "pi0_doubled", "target": layer.boxes["bowl"], "closing": True, "opening": 0.03}
layer._on_chunk(close); layer._on_chunk(close)
layer._contact = {"tool": "pi0_pick", "target": None, "closing": False, "opening": 0.08}
layer._on_chunk(np.zeros((5, 7)))
layer._contact = None
print("7 VLA bound ok")

# 8. Held object: only a grasp that closed on something; dropped when the fingers are empty;
#    contact plans reset when the held object changes.
p._last_obs_eef_pos = np.array([0.0, 0.0, 0.93])
layer._contact_plans["x"] = {"adjacent": False}
p._last_obs_gripper = 0.03
layer._after("pi0_pick", {}, ToolResult(data={"success": False}), closed_on_contact=True)
assert layer.held == "bowl" and not layer._contact_plans
assert layer._contact_target([0.0, 0.0, 0.98]) is None  # held bowl is no contact target
p._last_obs_gripper = 0.001
layer._after("move_to", {"xyz": [0, 0, 1]}, ToolResult(data={"final_dist_m": 0.0}))
assert layer.held is None
layer._after("set_gripper", {"gripper": 1.0}, ToolResult(data={}))
assert layer.held is None, "closing on nothing holds nothing"
p._last_obs_gripper = 0.03
layer._after("pi0_pick", {}, ToolResult(data={}), closed_on_contact=False)
assert layer.held is None, "a grasp that never closed holds nothing"
print("8 held tracking ok")

# 9. A route whose last segment descends onto the target: that segment is the contact; the
#    pre-grasp before it must clear the obstacle; the descent then passes the motion gate.
p._last_obs_gripper = 0.08
p._last_obs_eef_pos = np.array([-0.3, 0.0, 1.2])
high = [0.0, 0.0, 1.11]
r = layer.check_route(waypoints=[[0.0, 0.0, 1.2], high, [0.0, 0.0, 0.97]])
print("   route", r.data["route_ok"], [(s["ok"], s["contact_segment"]) for s in r.data["segments"]])
assert r.data["route_ok"] and r.data["segments"][-1]["contact_segment"]
p._last_obs_eef_pos = np.array(high)
layer._after("move_to", {"xyz": high}, ToolResult(data={"final_dist_m": 0.002}))
assert layer._route_end is not None
assert layer._refusal("move_to", {"xyz": [0.0, 0.0, 0.97]}) is None
layer._route_end = None
assert layer._refusal("move_to", {"xyz": [0.0, 0.0, 0.97]}) is not None
print("9 route contact ok")

# 10. A cached decision is dropped only when a box moves > 2 cm.
layer._contact_plans.clear()
p._last_obs_eef_pos = np.array([0.0, 0.0, 0.98])
layer._contact_rule("pi0_pick", {"prompt": "x"})
assert "bowl" in layer._contact_plans
shift = lambda b, d: {**b, "min": [v + d for v in b["min"]], "max": [v + d for v in b["max"]]}
layer._forget_contact_plans(layer.boxes["bowl"], shift(layer.boxes["bowl"], 0.005))
assert "bowl" in layer._contact_plans
layer._forget_contact_plans(layer.boxes["bowl"], shift(layer.boxes["bowl"], 0.03))
assert "bowl" not in layer._contact_plans
print("10 cache ok")

# 11. A halted motion clears the route state.
def halting(**kw):
    raise rl.SafetyHalt("pot")
layer._route_end = np.array([0, 0, 1.0]); layer._planned_end = np.array([0, 0, 1.0])
p._last_obs_eef_pos = np.array([-0.3, 0.0, 1.2])
out = layer.gate("move_to", halting)(xyz=[-0.3, 0.1, 1.2])
assert out.data["halted"] and layer._route_end is None and layer._planned_end is None
print("11 halt state ok")

# 12. Escape uses signed gaps: a held object sweeping across the obstacle is refused.
layer.boxes["mug"] = box("mug", "target", [-0.4, -0.04, 0.9], [-0.32, 0.04, 1.0])
layer.held = "mug"
assert not layer.check_segment([0.04, 0.0, 0.95], [0.20, 0.0, 0.95])["ok"], "sweep through the pot"
assert layer.check_segment([0.04, 0.0, 0.95], [-0.20, 0.0, 0.95])["ok"], "moving away escapes"
layer.held = None; del layer.boxes["mug"]
print("12 signed escape ok")

# 13. Adjacency is re-tested at the current yaw at each contact (no stale "not adjacent").
layer._contact_plans.clear(); p.set_yaw(math.pi / 2)
p._last_obs_eef_pos = np.array([0.0, 0.0, 0.98])
layer.boxes["pot"] = box("pot", "obstacle", [0.0, 0.10, 0.90], [0.10, 0.20, 1.05])  # pot on +y side
r = layer.contact_check("bowl").data
print("   at yaw 90:", r)
p.set_yaw(0.0)
refusal, turn = layer._contact_rule("pi0_pick", {"prompt": "x"})
print("   at yaw 0:", refusal and refusal.get("error"), turn)
assert (refusal is not None) or (turn is not None), "the long hand axis now points at the pot"
layer.boxes["pot"] = box("pot", "obstacle", [0.06, -0.06, 0.90], [0.16, 0.06, 1.05])
layer._contact_plans.clear(); p.set_yaw(0.0); p.rotations.clear()
print("13 per-contact retest ok")

# 14. Contact descent after arriving 1.1-2.5 cm off the pre-grasp is still the contact.
layer._route_end = np.array([-0.01, 0.0, 1.11])
assert layer._is_contact([-0.01, 0.022, 1.11], [-0.01, 0.0, 0.97]), "arrival 2.2 cm off"
assert not layer._is_contact([-0.01, 0.04, 1.11], [-0.01, 0.0, 0.97]), "4 cm off is not at the pose"
assert not layer._is_contact([-0.01, 0.0, 1.11], [0.02, 0.0, 0.97]), "not vertical below the pose"
layer._route_end = None
print("14 descent tolerance ok")

# 15. Held object is dropped when the fingers open.
layer.held = "bowl"; p._last_obs_gripper = 0.08
layer._after("move_to", {"xyz": [0, 0, 1.2]}, ToolResult(data={"final_dist_m": 0.0}))
assert layer.held is None
print("15 held cleared on open ok")

# 16. Release over an adjacent place is under the adjacency rule; route state ends with a VLA call.
layer.boxes["plate"] = box("plate", "place", [-0.05, -0.05, 0.90], [0.05, 0.05, 0.91])
del layer.boxes["bowl"]
layer._contact_plans.clear()
layer.boxes["cup"] = box("cup", "target", [-0.4, -0.02, 0.9], [-0.36, 0.02, 0.95])
layer.held = "cup"
p._last_obs_eef_pos = np.array([0.04, 0.0, 1.0])  # 4.5 cm from the selected point
refusal, turn = layer._contact_rule("release", {})
assert refusal is not None, "release next to the pot must follow the clearing direction"
layer.held = None
layer._route_end = np.array([0, 0, 1.1]); layer._planned_end = np.array([0, 0, 1.1])
p._last_obs_gripper = 0.08
layer._after("pi0_pick", {}, ToolResult(data={}), closed_on_contact=False)
assert layer._route_end is None and layer._planned_end is None
print("16 release rule and phase end ok")

# 17. R1: opening an empty gripper over a target adjacent to the obstacle is no contact.
layer.held = None
p._last_obs_eef_pos = np.array([0.0, 0.0, 1.0])
assert layer._contact_rule("release", {}) == (None, None)
print("17 empty release ok")

# 18. R2: a cached point that no longer clears at the current yaw is recomputed.
layer.boxes["bowl"] = box("bowl", "target", [-0.05, -0.05, 0.90], [0.05, 0.05, 0.95])
plate = layer.boxes.pop("plate")  # same footprint as the bowl; test the bowl alone
layer._contact_plans.clear(); p.set_yaw(0.0)
first = layer.contact_check("bowl").data
p.set_yaw(math.pi / 2)
p._last_obs_eef_pos = np.array([*first["contact_point_xy"], 0.98])
refusal, turn = layer._contact_rule("pi0_pick", {"prompt": "x"})
second = layer._contact_plans["bowl"]
print("   yaw 0 point", first["contact_point_xy"], "-> yaw 90:", None if second["contact_point"] is None else np.round(second["contact_point"][:2], 3), "turn", turn is not None)
assert refusal is None and turn is not None and second["contact_point"] is None, (refusal, turn)
assert layer._contact_rule("pi0_pick", {"prompt": "x"}) == (None, None), "the turned contact proceeds"
p.set_yaw(0.0); layer._contact_plans.clear(); layer.boxes["plate"] = plate; del layer.boxes["bowl"]
print("18 stale point recomputed ok")

# 19. R3: a release a few mm off the selected point (toward the pot) is accepted.
layer.held = "cup"
a = layer._approach(layer.boxes["plate"], 0.0)
pt = a["contact_point"]
for off in (0.004, 0.008):
    p._last_obs_eef_pos = np.array([pt[0] + off, pt[1], 1.0])
    assert layer._contact_rule("release", {})[0] is None, off
layer.held = None
print("19 servo error tolerated ok")
print("ALL OK")
