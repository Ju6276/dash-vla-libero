"""Depth-point-cloud control barrier function on the commanded gripper motion.

Runs in the env process and filters every control action before it reaches the
simulator, whether the planner or the VLA produced it. Each step it rebuilds the
obstacle set from the current depth images, so a moved or newly appearing
object is seen at once; no obstacle is chosen in advance.

Obstacle points: a depth image of the current state from each camera, taken by
casting one ray per pixel from the camera (mj_multiRay; the same points a depth
camera returns, without the GL context that robosuite's hard reset rebuilds),
minus the robot and the support surfaces (table, floor, arena), minus the
objects the planner marked as the grasp target or the place, and minus the held
object. Robot and support points are known from the geom each ray hits; on a
real robot this is the usual self-filter and plane removal. Target and place
are boxes the planner grounded with SAM 3; when a box is marked, the object
that has most of its points inside it is taken as the marked object and is
followed from then on by the object id of the ray hits (on a real robot, a
tracked segmentation mask), so a marked object that the VLA nudges or the
gripper carries stays exempt. Its box stays exempt too.

Protected body: the oriented bounding boxes of the collision geoms of the hand,
the fingers and the last two arm links at their current poses, and the held
object: the box around its current points, or its marked box moved with the
gripper since the grasp while it is out of view.

Barrier: for each protected element e, h_e = d_e - MARGIN with d_e its distance
to the nearest obstacle point and n_e the unit vector from that point to e. The
OSC controller moves the gripper by about a quarter of the commanded
displacement per step and lags (measured: next displacement ~ LAG * current
displacement + GAIN * command, coasting about BRAKE times one step after the
command stops). The predicted next displacement, extended by the braking
distance, may use at most GAMMA of the clearance:
    (1 + BRAKE) * n_e . (LAG * v + GAIN * u) >= -GAMMA * h_e.
The filtered command u is the closest to the nominal one under these
constraints (small QP, Hildreth's method). Rotation and gripper commands pass
unchanged.
"""

from __future__ import annotations

from collections import deque

import numpy as np

MARGIN = 0.015          # clearance kept between the protected body and any obstacle point (m)
GAMMA = 0.5             # share of the clearance the next step plus braking may use
LAG = 0.5               # measured OSC response, see module docstring
GAIN = 0.125
BRAKE = 1.5
EXEMPT_PAD = 0.01       # marked boxes are grown by this much before their points are dropped
MARK_SHARE = 0.5        # an object is the marked one if this share of its points lies in the box
MARK_MIN_POINTS = 5
RESOLUTION = 128
CAMERAS = ("agentview", "robot0_eye_in_hand")
STRIDE = 2              # one ray per STRIDE x STRIDE pixels: 64 x 64 rays per camera
RAY_GROUPS = np.array([1, 1, 1, 0, 0, 0], dtype=np.uint8)  # geom groups a camera shows
RAY_CUTOFF = 3.0        # m
NEAREST = 3             # constraints per protected element (nearest distinct points)
BLOCK_WINDOW = 15       # steps over which progress is judged
BLOCK_PROGRESS = 0.25   # blocked: the gripper covered less than this share of the nominal motion
BLOCK_INTERVENED = 0.8  # ... while the filter changed at least this share of the steps
MIN_COMMAND = 2e-3      # mean nominal command (m per step) below which nothing is judged blocked
ROBOT_PREFIXES = ("robot0_", "gripper0_", "mount0_")
SUPPORT_BODIES = ("table", "floor", "world")
PROTECTED_BODIES = (
    "gripper0_right_gripper", "gripper0_leftfinger", "gripper0_rightfinger",
    "gripper0_finger_joint1_tip", "gripper0_finger_joint2_tip", "robot0_link7", "robot0_link6",
)


def _box_distance(points: np.ndarray, lo: np.ndarray, hi: np.ndarray):
    """Distance from each point to an axis-aligned box and the closest box point."""
    closest = np.clip(points, lo, hi)
    return np.linalg.norm(points - closest, axis=1), closest


def _hildreth(u0: np.ndarray, rows: np.ndarray, rhs: np.ndarray, iters: int = 60) -> np.ndarray:
    """argmin |u - u0|^2 s.t. rows @ u >= rhs (dual coordinate ascent)."""
    keep = (rows * rows).sum(axis=1) > 1e-12
    rows, rhs = rows[keep], rhs[keep]
    if len(rows) == 0:
        return u0
    lam = np.zeros(len(rows))
    norms = np.maximum((rows * rows).sum(axis=1), 1e-12)
    u = u0.copy()
    for _ in range(iters):
        changed = False
        for i in range(len(rows)):
            step = (rhs[i] - rows[i] @ u) / norms[i]
            new = max(0.0, lam[i] + step)
            if new != lam[i]:
                u = u + (new - lam[i]) * rows[i]
                lam[i] = new
                changed = True
        if not changed:
            break
    return u


class DepthCbf:
    def __init__(self, sim, output_max: float) -> None:
        import mujoco

        self._mj = mujoco
        self.scale = float(output_max)
        self.exempt: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self.exempt_bodies: dict[str, set[int]] = {}
        self.held: dict | None = None
        self.bind(sim)
        self.reset_episode()

    def bind(self, sim) -> None:
        """Attach to the current simulator. robosuite's hard reset builds a new
        MjSim on every episode reset, so the env rebinds after each reset."""
        mujoco = self._mj
        self._sim = sim
        self.model = sim.model._model
        self.data = sim.data._data
        m = self.model
        body = lambda name: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)  # noqa: E731
        names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, int(m.geom_bodyid[g])) or "" for g in range(m.ngeom)]
        self._drop_geom = np.array(
            [n.startswith(ROBOT_PREFIXES) or n in SUPPORT_BODIES for n in names], dtype=bool
        )
        protected = {body(b) for b in PROTECTED_BODIES}
        self._protected = [
            g for g in range(m.ngeom)
            if int(m.geom_bodyid[g]) in protected and (m.geom_contype[g] or m.geom_conaffinity[g])
        ]
        self._site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "gripper0_grip_site")
        self._cams = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, c) for c in CAMERAS]
        self._last_grip = None

    # ---- configuration from the harness ---------------------------------

    def reset_episode(self) -> None:
        self.exempt = {}
        self.exempt_bodies = {}
        self.held = None
        self._window = deque(maxlen=BLOCK_WINDOW)
        self._last_grip = None
        self.totals = {"steps": 0, "interventions": 0, "blocked_steps": 0}
        self.reset_stats()

    def reset_stats(self) -> None:
        """Start a new tool call: its blocking is judged on its own motion only."""
        self._window.clear()
        self.stats = {
            "steps": 0,
            "interventions": 0,
            "max_correction_m": 0.0,
            "min_clearance_m": None,
            "closest_point": None,
            "closest_element": None,
            "blocked": False,
            "blocked_point": None,
        }

    def set_exempt(self, label: str, lo, hi) -> None:
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        self.exempt[label] = (lo, hi)
        points, bodies = self._cloud()
        inside = np.all((points >= lo - EXEMPT_PAD) & (points <= hi + EXEMPT_PAD), axis=1)
        chosen = set()
        for b in np.unique(bodies[inside]):
            mine = bodies == b
            if inside[mine].sum() >= MARK_MIN_POINTS and inside[mine].mean() >= MARK_SHARE:
                chosen.add(int(b))
        self.exempt_bodies[label] = chosen
        if self.held is not None and self.held["label"] == label:
            self.set_held(label)

    def clear_exempt(self, label: str) -> None:
        self.exempt.pop(label, None)
        self.exempt_bodies.pop(label, None)

    def set_held(self, label: str | None) -> None:
        """Attach a marked object to the gripper (None releases it)."""
        if label is None or label not in self.exempt:
            self.held = None
            return
        lo, hi = self.exempt[label]
        self.held = {"label": label, "lo": lo, "hi": hi, "grip": self._grip_pos().copy(),
                     "bodies": set(self.exempt_bodies.get(label, set()))}

    # ---- geometry -------------------------------------------------------

    def _grip_pos(self) -> np.ndarray:
        return np.array(self.data.site_xpos[self._site])

    def _held_box(self, points=None, bodies=None):
        """Box of the held object: around its current points, or its marked box
        moved with the gripper while too few of its points are in view."""
        if self.held is None:
            return None
        if points is not None and self.held["bodies"]:
            mine = np.isin(bodies, list(self.held["bodies"]))
            if mine.sum() >= 10:
                return points[mine].min(axis=0), points[mine].max(axis=0)
        shift = self._grip_pos() - self.held["grip"]
        return self.held["lo"] + shift, self.held["hi"] + shift

    def _elements(self, held=None) -> list[tuple[str, np.ndarray, np.ndarray, np.ndarray]]:
        """(name, centre, rotation, half sizes) of every protected box."""
        m, d = self.model, self.data
        out = []
        for g in self._protected:
            rot = np.array(d.geom_xmat[g]).reshape(3, 3)
            aabb = np.array(m.geom_aabb[g])
            out.append(("robot", np.array(d.geom_xpos[g]) + rot @ aabb[:3], rot, aabb[3:]))
        if held is not None:
            lo, hi = held
            out.append(("held", (lo + hi) / 2, np.eye(3), (hi - lo) / 2))
        return out

    def _rays(self, cam: int) -> tuple[np.ndarray, np.ndarray]:
        """World points and hit geoms of one depth image (RESOLUTION / STRIDE pixels a side)."""
        m, d, n = self.model, self.data, RESOLUTION
        f = 0.5 * n / np.tan(np.deg2rad(m.cam_fovy[cam]) / 2)
        pix = np.arange(0, n, STRIDE) + STRIDE / 2
        u, v = np.meshgrid(pix, pix)
        # MuJoCo camera frame: x right, y up, looking along -z; image rows go down.
        local = np.stack([(u - n / 2) / f, -(v - n / 2) / f, -np.ones_like(u)], axis=-1).reshape(-1, 3)
        rot = np.array(d.cam_xmat[cam]).reshape(3, 3)
        dirs = local @ rot.T
        dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
        origin = np.array(d.cam_xpos[cam])
        geom = np.full(len(dirs), -1, dtype=np.int32)
        dist = np.full(len(dirs), -1.0)
        self._mj.mj_multiRay(m, d, origin, dirs.reshape(-1), RAY_GROUPS, 1, -1, geom, dist, len(dirs), RAY_CUTOFF)
        return origin + dirs * dist[:, None], geom

    def _cloud(self) -> tuple[np.ndarray, np.ndarray]:
        """All non-robot, non-support points of the current depth images and their bodies."""
        clouds, ids = [], []
        for cam in self._cams:
            points, geom = self._rays(cam)
            keep = geom >= 0
            keep[keep] = ~self._drop_geom[geom[keep]]
            clouds.append(points[keep])
            ids.append(geom[keep])
        points = np.concatenate(clouds) if clouds else np.zeros((0, 3))
        geoms = np.concatenate(ids) if ids else np.zeros(0, int)
        return points, np.asarray(self.model.geom_bodyid)[geoms].astype(int)

    def obstacle_points(self, with_bodies: bool = False):
        """Points of every unmarked object, and the held object's box."""
        points, bodies = self._cloud()
        held = self._held_box(points, bodies)
        drop = np.zeros(len(points), dtype=bool)
        boxes = list(self.exempt.values()) + ([held] if held is not None else [])
        for lo, hi in boxes:
            drop |= np.all((points >= lo - EXEMPT_PAD) & (points <= hi + EXEMPT_PAD), axis=1)
        marked = set().union(*self.exempt_bodies.values()) if self.exempt_bodies else set()
        if marked:
            drop |= np.isin(bodies, list(marked))
        if with_bodies:
            return points[~drop], bodies[~drop], held
        return points[~drop], held

    # ---- filter ---------------------------------------------------------

    def filter(self, action: np.ndarray) -> np.ndarray:
        action = np.asarray(action, dtype=np.float32).copy()
        u0 = action[:3].astype(float) * self.scale
        grip = self._grip_pos()
        v = np.zeros(3) if self._last_grip is None else grip - self._last_grip
        self._last_grip = grip
        points, held = self.obstacle_points()
        rows, rhs = [], []
        clearance, closest, element = None, None, None
        for name, centre, rot, half in self._elements(held) if len(points) else []:
            local = (points - centre) @ rot
            near = centre + np.clip(local, -half, half) @ rot.T
            vec = near - points
            dist = np.linalg.norm(vec, axis=1)
            order = np.argsort(dist)[: NEAREST * 8]
            used = []
            for i in order:
                # A point inside the box has no surface direction: push straight out from the centre.
                n = vec[i] if dist[i] > 1e-4 else centre - points[i]
                n = n / max(float(np.linalg.norm(n)), 1e-9)
                if any(float(n @ k) > 0.95 for k in used):
                    continue
                used.append(n)
                h = float(dist[i]) - MARGIN
                # (1 + BRAKE) * n.(LAG v + GAIN u) >= -GAMMA h, as a row on u
                rows.append((1 + BRAKE) * GAIN * n)
                rhs.append(-GAMMA * h - (1 + BRAKE) * LAG * float(n @ v))
                if clearance is None or dist[i] < clearance:
                    clearance, closest, element = float(dist[i]), points[i], name
                if len(used) >= NEAREST:
                    break
        u = _hildreth(u0, np.array(rows).reshape(-1, 3), np.array(rhs)) if rows else u0
        correction = float(np.linalg.norm(u - u0))
        self._window.append((grip, float(np.linalg.norm(u0)), correction > 0.3 * float(np.linalg.norm(u0)) + 1e-4))
        s = self.stats
        s["steps"] += 1
        if correction > 1e-4:
            s["interventions"] += 1
            s["max_correction_m"] = max(s["max_correction_m"], correction)
        if clearance is not None and (s["min_clearance_m"] is None or clearance < s["min_clearance_m"]):
            s["min_clearance_m"] = clearance
            s["closest_point"] = [round(float(v), 4) for v in closest]
            s["closest_element"] = element
        self.totals["steps"] += 1
        self.totals["interventions"] += int(correction > 1e-4)
        if self.blocked():
            self.totals["blocked_steps"] += 1
            s["blocked"] = True
            s["blocked_point"] = s["closest_point"]
        action[:3] = np.clip(u / self.scale, -1.0, 1.0)
        return action

    def blocked(self) -> bool:
        """The filter has held the gripper back for a whole window: it is not getting closer."""
        if len(self._window) < BLOCK_WINDOW:
            return False
        commands = [c for _, c, _ in self._window]
        if np.mean(commands) < MIN_COMMAND:
            return False
        nominal = GAIN / (1 - LAG) * float(np.sum(commands))  # steady-state OSC travel
        moved = float(np.linalg.norm(self._window[-1][0] - self._window[0][0]))
        intervened = np.mean([i for _, _, i in self._window])
        return moved < BLOCK_PROGRESS * nominal and intervened >= BLOCK_INTERVENED

    def close(self) -> None:
        pass
