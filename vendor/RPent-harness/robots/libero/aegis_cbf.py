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


# ---- filter ----------------------------------------------------------------

class AegisCbf:
    def __init__(self, sim, output_max: float) -> None:
        self.scale = float(output_max)
        self.held = None
        self.bind(sim)
        self.reset_episode()

    def bind(self, sim) -> None:
        self._sim = sim

    def reset_episode(self) -> None:
        self.obs = None
        self.active = False
        self.obstacle: dict = {}
        self._window = deque(maxlen=BLOCK_WINDOW)
        self.totals = {"steps": 0, "interventions": 0, "blocked_steps": 0}
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
        return result

    def filter(self, action: np.ndarray) -> np.ndarray:
        action = np.asarray(action, dtype=np.float32).copy()
        s = self.stats
        s["steps"] += 1
        self.totals["steps"] += 1
        if not self.active or self.obs is None:
            return action
        p1, R1 = self._eef()
        a_v, a_omega, a_uz, h, mu_row = compute_h_coeffs_3d(p1, self.Q1, R1, self.p2, self.Q2, self.R2, self.z)
        u_ref = np.hstack([5 * (R1.T @ action[:3]), 5 * action[3:6], 10 * mu_row]).astype(float)
        a = np.hstack([0.2 * a_v, 0.2 * a_omega, a_uz])
        # min (u - u_ref)' W (u - u_ref)  s.t.  a.u + 10 h >= 0
        slack = float(a @ u_ref + 10 * h)
        u = u_ref if slack >= 0 else u_ref - slack / float(a @ (a / W_DIAG)) * (a / W_DIAG)
        u_v, u_omega, u_z = u[:3], u[3:6], u[6:]
        dz = project_matrix(self.z) @ u_z
        self.z = self.z + dz * DT
        self.z = self.z / np.linalg.norm(self.z)
        out = np.zeros(7, dtype=np.float32)
        out[:3] = 0.2 * R1 @ u_v
        out[3:6] = 0.2 * u_omega
        out[6] = action[6]

        intervened = slack < 0
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

    def close(self) -> None:
        pass
