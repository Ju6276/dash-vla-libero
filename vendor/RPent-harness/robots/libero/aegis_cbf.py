"""AEGIS's control barrier function (VLSA, arXiv 2512.11891) under the harness.

The safety layer of third_party/vlsa-aegis/main/main_aegis.py, moved from its
VLA-only loop into the env process so that it filters every control action,
the planner's motions and the VLA's alike:
  - after the settle steps of each reset, the obstacle is named, detected and
    fitted with an ellipsoid once (scripts/safelibero/aegis_perceive.py, in
    .venv-aegis, from agentview and backview rendered at 1024 x 1024);
  - every step, the end-effector ellipsoid (pose from the last observation)
    and the obstacle ellipsoid give one CBF constraint on the end-effector
    twist and the separating direction z; the weighted QP is solved and z is
    integrated, with AEGIS's gains.
The CBF functions are copied from utils.py. The QP has a single linear
constraint, so its solution is written in closed form instead of calling
OSQP; it is the same minimiser.

RPENT_CBF_LINKS (e.g. "robot0_link5,robot0_link6") adds one constraint per
listed arm link: the link's collision mesh is enclosed in an ellipsoid (fitted
once from the model), and the same h and coefficients are taken against the
obstacle, each link with its own separating direction. The link's twist is
mapped from the commanded end-effector twist through the arm Jacobians,
J_link M^-1 J_ee' (J_ee M^-1 J_ee')^-1, the dynamically consistent inverse
the OSC itself uses (the controller's null-space posture term is ignored), with the
commanded rotation in the controller's own units (output_max for position and
orientation differ by 10). With more
than one constraint the QP is solved exactly by enumerating active sets.
Without the variable the filter is AEGIS's.

For the harness's feedback (robots/libero/cbf_layer.py) the filter keeps the
same per-call statistics and blocked rule as cbf_filter.DepthCbf. Marking and
holding do nothing here: AEGIS protects one obstacle and nothing else.
"""

from __future__ import annotations

import json
import os
import subprocess
import tempfile
from collections import deque
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

from robots.libero.cbf_filter import BLOCK_INTERVENED, BLOCK_PROGRESS, BLOCK_WINDOW, GAIN, LAG, MIN_COMMAND

ROOT = Path("/homeL/dong/Projects/dash-vla-libero")
RESOLUTION = 1024  # LIBERO_ENV_RESOLUTION of main_aegis.py
VIEWS = ("agentview", "backview")
DT = 0.05
EEF_OFFSET = np.array([0, 0, -0.08])
W_DIAG = np.array([1.0 / 25] * 6 + [1.0] * 3)


# ---- utils.py (vlsa-aegis), copied with its comments dropped ---------------

def vector_hat(v):
    """hat operator: R^3 -> so(3)"""
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def project_matrix(z):
    """Compute (I - z z^T)."""
    z = z / (np.linalg.norm(z) + 1e-12)
    return np.eye(3) - np.outer(z, z)


def compute_h_ij(p_i, Q_i_diag, R_i,
                 p_j, Q_j_diag, R_j,
                 z_ij):
    Q_i = np.diag(Q_i_diag)
    Q_j = np.diag(Q_j_diag)
    Qbar_i = R_i @ Q_i @ R_i.T
    Qbar_j = R_j @ Q_j @ R_j.T
    Qbar_i_inv = np.linalg.inv(Qbar_i)
    z = z_ij / np.linalg.norm(z_ij)
    term1 = np.linalg.norm(Qbar_j @ Qbar_i_inv @ z)
    term2 = (p_j - p_i).T @ Qbar_i_inv @ z
    denom = np.linalg.norm(Qbar_i_inv @ z)
    h_ij = (-term1 + term2 - 1.0) / denom
    return h_ij


def compute_h_coeffs_3d(p_i, Q_i_diag, R_i,
                        p_j, Q_j_diag, R_j,
                        z,
                        eps=1e-10):
    Q_i = np.diag(Q_i_diag); Q_j = np.diag(Q_j_diag)
    Qbar_i = R_i @ Q_i @ R_i.T
    Qbar_j = R_j @ Q_j @ R_j.T
    Qbar_i_inv = np.linalg.inv(Qbar_i)
    Qbar_i_inv2 = Qbar_i_inv @ Qbar_i_inv
    Qbar_j2 = Qbar_j @ Qbar_j

    z = z / (np.linalg.norm(z)+eps)
    a_vec = Qbar_i_inv @ z
    denom = np.linalg.norm(a_vec) + eps
    b_vec = Qbar_j @ a_vec
    term1 = np.linalg.norm(b_vec) + eps
    sigma = term1 * denom + eps
    rho = (1.0 - (p_j - p_i).T @ a_vec + term1)

    eta_row = - (1.0 / denom) * (z.T @ Qbar_i_inv)
    term_mu_1 = (rho / (denom**3 + eps)) * (z.T @ Qbar_i_inv2)
    term_mu_2 = (1.0 / denom) * ((p_j - p_i).T @ Qbar_i_inv)
    term_mu_3 = (1.0 / sigma) * (z.T @ Qbar_i_inv @ Qbar_j2 @ Qbar_i_inv)
    mu_row = term_mu_1 + term_mu_2 - term_mu_3

    tmp1 = z.T @ Qbar_i_inv2 @ vector_hat(z)
    left_vec = (z.T @ Qbar_i_inv @ Qbar_j2)
    Ja_vec = vector_hat(a_vec)
    tmp2 = left_vec @ (Ja_vec - Qbar_i_inv @ vector_hat(z))
    part_a = (p_j - p_i).T @ Qbar_i_inv @ vector_hat(z)
    part_b = z.T @ Qbar_i_inv @ vector_hat(p_j - p_i)
    tmp3 = part_a + part_b
    zeta_tilde = rho * (1.0 / (denom**3 + eps)) * tmp1 + (1.0 / sigma) * tmp2 + (1.0 / denom) * tmp3

    a_v = (eta_row @ R_i).ravel()
    a_omega = zeta_tilde @ R_i
    a_uz = (mu_row @ project_matrix(z)).ravel()
    h = compute_h_ij(p_i, Q_i_diag, R_i, p_j, Q_j_diag, R_j, z)
    return a_v, a_omega, a_uz, h, mu_row


# ---- arm links (RPENT_CBF_LINKS) ---------------------------------------------

def enclosing_ellipsoid(points: np.ndarray):
    """Centre, rotation and semi-axes of the principal-axis ellipsoid scaled to just enclose the points."""
    centre = points.mean(axis=0)
    _, _, axes = np.linalg.svd(points - centre, full_matrices=False)
    rot = axes.T
    if np.linalg.det(rot) < 0:
        rot[:, 2] *= -1
    local = (points - centre) @ rot
    half = np.maximum(np.abs(local).max(axis=0), 1e-3)
    scale = float(np.sqrt(((local / half) ** 2).sum(axis=1).max()))  # smallest scaling that encloses every vertex
    return centre, rot, half * scale


def link_ellipsoids(model, body: str, parts: int = 1):
    """Ellipsoids (body frame) around the body's collision meshes; parts > 1 cuts the mesh
    into that many equal slices along its long axis, one ellipsoid each."""
    import mujoco

    index = model.body_name2id(body)
    points = []
    for g in range(model.ngeom):
        if model.geom_bodyid[g] != index or model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0:
            continue
        mesh = model.geom_dataid[g]
        if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or mesh < 0:
            continue
        start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        rot = R.from_quat(np.roll(model.geom_quat[g], -1)).as_matrix()
        points.append(np.asarray(model.geom_pos[g]) + np.asarray(model.mesh_vert[start:start + count]) @ rot.T)
    points = np.vstack(points)
    if parts == 1:
        return [enclosing_ellipsoid(points)]
    centre, rot, _ = enclosing_ellipsoid(points)
    along = (points - centre) @ rot[:, 0]
    order = np.argsort(along)
    return [enclosing_ellipsoid(points[chunk]) for chunk in np.array_split(order, parts)]


def solve_qp(u_ref, W, A, b):
    """min (u - u_ref)' W (u - u_ref) s.t. A u + b >= 0, W diagonal, few rows: exact by active sets."""
    if np.all(A @ u_ref + b >= -1e-12):
        return u_ref
    Winv = 1.0 / W
    best = None
    rows = range(len(b))
    for mask in range(1, 1 << len(b)):
        S = [i for i in rows if mask >> i & 1]
        AS = A[S]
        lam = -np.linalg.pinv((AS * Winv) @ AS.T) @ (AS @ u_ref + b[S])
        if np.any(lam < -1e-9):
            continue
        u = u_ref + Winv * (AS.T @ lam)
        if np.any(A @ u + b < -1e-7):
            continue
        cost = float(((u - u_ref) ** 2 * W).sum())
        if best is None or cost < best[0]:
            best = (cost, u)
    if best is None:  # no feasible active set (conflicting rows): least-squares on all of them
        lam = -np.linalg.pinv((A * Winv) @ A.T) @ (A @ u_ref + b)
        return u_ref + Winv * (A.T @ np.maximum(lam, 0))
    return best[1]


# ---- filter ----------------------------------------------------------------

class AegisCbf:
    def __init__(self, sim, output_max: float, ori_output_max: float = 0.5) -> None:
        self.scale = float(output_max)
        # The OSC scales a unit action to output_max metres or ori_output_max radians per step.
        # AEGIS's hand row treats both alike; the link rows weight rotation by the ratio,
        # since turning the wrist sweeps the forearm much further than it moves the hand.
        self.ori_ratio = float(ori_output_max) / float(output_max)
        self.held = None
        self.bind(sim)
        self.reset_episode()

    def bind(self, sim) -> None:
        self._sim = sim
        # "robot0_link5:2,robot0_link6": body[:slices]; each slice is one ellipsoid and one constraint
        model = sim.model
        self.links = {}
        for entry in filter(None, os.environ.get("RPENT_CBF_LINKS", "").split(",")):
            body, _, parts = entry.partition(":")
            shapes = link_ellipsoids(model, body, int(parts or 1))
            for i, (centre, rot, half) in enumerate(shapes):
                self.links[body if len(shapes) == 1 else f"{body}#{i}"] = (body, centre, rot, half)
        if self.links:
            arm = [model.joint_name2id(f"robot0_joint{i}") for i in range(1, 8)]
            self._dofs = [int(model.jnt_dofadr[j]) for j in arm]

    def reset_episode(self) -> None:
        self.obs = None
        self.active = False
        self.obstacle: dict = {}
        self._window = deque(maxlen=BLOCK_WINDOW)
        self.totals = {"steps": 0, "interventions": 0, "blocked_steps": 0}
        self.last = None
        self.reset_stats()

    def reset_stats(self) -> None:
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

    # AEGIS protects one obstacle: marks and the held object change nothing.
    def set_exempt(self, label: str, lo, hi) -> None:
        pass

    def clear_exempt(self, label: str) -> None:
        pass

    def set_held(self, label) -> None:
        pass

    def observe(self, raw: dict) -> None:
        """The observation after the last step: AEGIS takes the end-effector pose from it."""
        self.obs = raw

    def _eef(self):
        R1 = R.from_quat(self.obs["robot0_eef_quat"]).as_matrix()
        return np.asarray(self.obs["robot0_eef_pos"], float) + R1 @ EEF_OFFSET, R1

    def _link_pose(self, name: str):
        body, centre, rot, _ = self.links[name]
        data = self._sim.data
        xmat = np.asarray(data.get_body_xmat(body), float)
        return np.asarray(data.get_body_xpos(body), float) + xmat @ centre, xmat @ rot

    def _link_map(self, name: str, p1: np.ndarray) -> np.ndarray:
        """6x6 map from the end-effector twist at p1 (world) to the link ellipsoid's twist at its centre (world)."""
        data = self._sim.data
        d = self._dofs

        def jac(body, point):
            jp = np.asarray(data.get_body_jacp(body), float).reshape(3, -1)[:, d]
            jr = np.asarray(data.get_body_jacr(body), float).reshape(3, -1)[:, d]
            r = point - np.asarray(data.get_body_xpos(body), float)
            return np.vstack([jp - vector_hat(r) @ jr, jr])

        pL, _ = self._link_pose(name)
        J_ee = jac("robot0_right_hand", p1)
        J_link = jac(self.links[name][0], pL)
        if os.environ.get("RPENT_CBF_LINK_MAP") == "pinv":
            return J_link @ np.linalg.pinv(J_ee, rcond=1e-3)
        # The OSC moves the joints along the dynamically consistent inverse M^-1 J' (J M^-1 J')^-1.
        import mujoco

        model, raw = self._sim.model._model, self._sim.data._data
        full = np.zeros((model.nv, model.nv))
        mujoco.mj_fullM(model, full, raw.qM)
        M_inv = np.linalg.inv(full[np.ix_(d, d)])
        lam = np.linalg.pinv(J_ee @ M_inv @ J_ee.T, rcond=1e-3)
        return J_link @ M_inv @ J_ee.T @ lam

    def perceive(self, raw: dict, language: str, suite: str, work: Path) -> dict:
        """Name, detect and fit the obstacle once, as main_aegis.py does after its wait steps."""
        from robosuite.utils.camera_utils import (
            get_camera_extrinsic_matrix,
            get_camera_intrinsic_matrix,
            get_real_depth_map,
        )

        self.observe(raw)
        sim = self._sim
        arrays = {"suite": suite, "language": language}
        for view in VIEWS:
            # The env's camera observation is the raw render; AEGIS turns it by 180 degrees.
            rgb, depth = sim.render(camera_name=view, width=RESOLUTION, height=RESOLUTION, depth=True)
            arrays[f"{view}_image"] = np.ascontiguousarray(rgb[::-1, ::-1])
            arrays[f"{view}_depth"] = np.ascontiguousarray(get_real_depth_map(sim, depth[..., None])[::-1, ::-1])
            arrays[f"{view}_K_inv"] = np.linalg.inv(get_camera_intrinsic_matrix(sim, view, RESOLUTION, RESOLUTION))
            arrays[f"{view}_T"] = get_camera_extrinsic_matrix(sim, view)
        work.mkdir(parents=True, exist_ok=True)
        src, dst = work / "views.npz", work / "obstacle.json"
        np.savez(src, **arrays)
        env = dict(os.environ)
        env["PYTHONPATH"] = os.pathsep.join(
            [str(ROOT / "third_party/vlsa-aegis/main"), os.environ.get("SAFELIBERO_ROBOSUITE_ROOT", "")]
        )
        env["HF_HOME"] = str(ROOT / "checkpoints/hf_aegis")  # GroundingDINO's bert-base-uncased lives here
        env["TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD"] = "1"
        proc = subprocess.run(
            [str(ROOT / ".venv-aegis/bin/python"), str(ROOT / "scripts/safelibero/aegis_perceive.py"), str(src), str(dst), str(work)],
            env=env, stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=1200,
        )
        (work / "perceive.log").write_text(proc.stdout + proc.stderr)
        if proc.returncode != 0 or not dst.exists():
            raise RuntimeError(f"aegis_perceive failed ({proc.returncode}), see {work / 'perceive.log'}")
        result = json.loads(dst.read_text())
        self.obstacle = result
        self.active = bool(result["active"])
        if self.active:
            p1, _ = self._eef()
            self.p2 = np.asarray(result["p2"], float)
            self.R2 = np.asarray(result["R2"], float)
            self.Q2 = np.asarray(result["Q2"], float)
            if "orange juice" in language or "milk" in language or "alphabet soup" in language:
                self.Q1 = np.array([0.06, 0.12, 0.2])
            else:
                self.Q1 = np.array([0.06, 0.12, 0.11])
            self.z = (self.p2 - p1) / np.linalg.norm(self.p2 - p1)
            self.z_links = {}
            for name in self.links:
                pL, _ = self._link_pose(name)
                self.z_links[name] = (self.p2 - pL) / np.linalg.norm(self.p2 - pL)
        return result

    def filter(self, action: np.ndarray) -> np.ndarray:
        action = np.asarray(action, dtype=np.float32).copy()
        s = self.stats
        s["steps"] += 1
        self.totals["steps"] += 1
        self.last = None
        if not self.active or self.obs is None:
            return action
        p1, R1 = self._eef()
        a_v, a_omega, a_uz, h, mu_row = compute_h_coeffs_3d(p1, self.Q1, R1, self.p2, self.Q2, self.R2, self.z)
        u_ref = np.hstack([5 * (R1.T @ action[:3]), 5 * action[3:6], 10 * mu_row]).astype(float)
        a = np.hstack([0.2 * a_v, 0.2 * a_omega, a_uz])
        # min (u - u_ref)' W (u - u_ref)  s.t.  a.u + 10 h >= 0
        slack = float(a @ u_ref + 10 * h)
        h_links = {}
        if not self.links:
            u = u_ref if slack >= 0 else u_ref - slack / float(a @ (a / W_DIAG)) * (a / W_DIAG)
        else:
            # u = [u_v, u_omega, u_z, u_z of each link]; the commanded world twist is (0.2 R1 u_v, 0.2 u_omega)
            n = len(self.links)
            rows, b = [np.hstack([a, np.zeros(3 * n)])], [10 * h]
            refs = [u_ref]
            for k, name in enumerate(self.links):
                pL, RL = self._link_pose(name)
                QL = self.links[name][3]
                lv, lw, luz, hL, muL = compute_h_coeffs_3d(pL, QL, RL, self.p2, self.Q2, self.R2, self.z_links[name])
                # lv, lw act on the link's body-frame twist; to world, then through the Jacobian map
                g = np.hstack([lv @ RL.T, lw @ RL.T]) @ self._link_map(name, p1)
                row = np.zeros(9 + 3 * n)
                row[:3] = 0.2 * g[:3] @ R1
                row[3:6] = 0.2 * self.ori_ratio * g[3:]
                row[9 + 3 * k:12 + 3 * k] = luz
                rows.append(row)
                b.append(10 * hL)
                refs.append(10 * muL)
                h_links[name] = float(hL)
            A, b = np.vstack(rows), np.asarray(b)
            W = np.hstack([W_DIAG, np.ones(3 * n)])
            # Turning the wrist is a cheap fix on paper for a link row (rotation is weighted by ori_ratio),
            # but the OSC tracks it loosely; RPENT_CBF_LINK_ROT_WEIGHT makes the QP correct by translation instead.
            W[3:6] *= float(os.environ.get("RPENT_CBF_LINK_ROT_WEIGHT", "25"))
            u_full = solve_qp(np.hstack(refs).astype(float), W, A, b)
            slack = min(slack, float(np.min(A @ np.hstack(refs) + b)))
            for k, name in enumerate(self.links):
                zL = self.z_links[name] + project_matrix(self.z_links[name]) @ u_full[9 + 3 * k:12 + 3 * k] * DT
                self.z_links[name] = zL / np.linalg.norm(zL)
            u = u_full[:9]
        u_v, u_omega, u_z = u[:3], u[3:6], u[6:]
        dz = project_matrix(self.z) @ u_z
        self.z = self.z + dz * DT
        self.z = self.z / np.linalg.norm(self.z)
        out = np.zeros(7, dtype=np.float32)
        out[:3] = 0.2 * R1 @ u_v
        out[3:6] = 0.2 * u_omega
        out[6] = action[6]

        intervened = slack < 0
        self.last = {"h": float(h), "intervened": bool(intervened), "h_links": h_links}
        correction = float(np.linalg.norm(out[:3] - action[:3])) * self.scale
        grip = np.asarray(self.obs["robot0_eef_pos"], float)
        self._window.append((grip, float(np.linalg.norm(action[:3])) * self.scale, intervened))
        if intervened:
            s["interventions"] += 1
            self.totals["interventions"] += 1
            s["max_correction_m"] = max(s["max_correction_m"], correction)
        if s["min_clearance_m"] is None or float(h) < s["min_clearance_m"]:
            s["min_clearance_m"] = float(h)
            s["closest_point"] = [round(float(v), 4) for v in self.p2]
            s["closest_element"] = "hand"
        if self.blocked():
            self.totals["blocked_steps"] += 1
            s["blocked"] = True
            s["blocked_point"] = s["closest_point"]
        return out

    def blocked(self) -> bool:
        """Same rule as DepthCbf.blocked: held back for a whole window without progress."""
        if len(self._window) < BLOCK_WINDOW:
            return False
        commands = [c for _, c, _ in self._window]
        if np.mean(commands) < MIN_COMMAND:
            return False
        nominal = GAIN / (1 - LAG) * float(np.sum(commands))
        moved = float(np.linalg.norm(self._window[-1][0] - self._window[0][0]))
        intervened = np.mean([i for _, _, i in self._window])
        return moved < BLOCK_PROGRESS * nominal and intervened >= BLOCK_INTERVENED

    def step_record(self, step: int, robot_contacts: list[str]) -> list:
        """One line of the per-step log: [step, h, intervened, level, body, robot geoms touching the obstacle(, lowest link h)].

        h and intervened are the filter's at this step (None before perception).
        level is min over robot and gripper body origins of |diag(1/Q2) R2' (x - p2)|,
        1 on the obstacle ellipsoid; it is not used by the filter, only logged to
        tell whether a link other than the hand came close.
        """
        h = intervened = level = body = None
        if self.last is not None:
            h, intervened = round(self.last["h"], 4), int(self.last["intervened"])
        if self.active:
            model, data = self._sim.model, self._sim.data
            if not hasattr(self, "_bodies"):
                self._bodies = [(n, model.body_name2id(n)) for n in model.body_names
                                if n.startswith(("robot0_", "gripper0_"))]
            best = None
            for name, index in self._bodies:
                x = self.R2.T @ (np.asarray(data.body_xpos[index]) - self.p2)
                value = float(np.linalg.norm(x / self.Q2))
                if best is None or value < best[0]:
                    best = (value, name)
            level, body = round(best[0], 3), best[1]
        record = [step, h, intervened, level, body, robot_contacts]
        if self.links and self.last is not None and self.last["h_links"]:
            record.append(round(min(self.last["h_links"].values()), 4))  # lowest link h (RPENT_CBF_LINKS)
        return record

    def close(self) -> None:
        pass
