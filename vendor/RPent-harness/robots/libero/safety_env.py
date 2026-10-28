"""LIBERO-Safety scene behind the same batched surface ``LiberoEnvFacade`` uses.

RPent's stock ``make_env`` asks rlinf for ``get_task_init_states(task_id)`` and
builds the BDDL path without a level directory. LIBERO-Safety requires
``(level, level_local_id)`` for both. This object stays inside ``env_server``
so the agent still talks to ``LiberoEnvClient`` over RPC.
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
from pathlib import Path

import numpy as np

SAFETY_SUITES = (
    "affordance",
    "human_safety",
    "obstacle_avoidance",
    "obstacle_avoidance_human",
    "reasoning_safety",
)


def safety_task_identity(suite_name: str, task_id: int) -> dict[str, str | int]:
    """Suite, level, task name, and language for one flat safety task id.

    The lookup runs in a child interpreter so the planner process does not
    import LIBERO. The result is task identity, not a constraint catalog.
    """
    if suite_name not in SAFETY_SUITES:
        raise KeyError(f"{suite_name} is not a LIBERO-Safety suite")
    if not os.environ.get("LIBERO_SAFETY_ROOT"):
        raise RuntimeError("LIBERO_SAFETY_ROOT is required")
    if not os.environ.get("LIBERO_CONFIG_PATH"):
        raise RuntimeError("LIBERO_CONFIG_PATH is required")
    code = r"""
import json, os, sys
from pathlib import Path
root = os.environ["LIBERO_SAFETY_ROOT"]
robosuite = os.environ.get(
    "LIBERO_ROBOSUITE_ROOT",
    str(Path(root) / "third_party" / "robosuite-1.4"),
)
sys.path[:0] = [root, robosuite]
from libero.libero.benchmark import get_benchmark
suite_name = os.environ["SAFETY_ID_SUITE"]
task_id = int(os.environ["SAFETY_ID_TASK"])
suite = get_benchmark(suite_name)()
per_level = suite.get_num_tasks_by_level(0)
if per_level <= 0 or suite.n_tasks % per_level != 0:
    raise SystemExit(f"{suite_name} has no even level split")
if not 0 <= task_id < suite.n_tasks:
    raise SystemExit(f"task_id out of range: {task_id}")
level = task_id // per_level
level_id = task_id % per_level
task = suite.get_task(task_id)
language = getattr(task, "language", None) or task.name.replace("_", " ")
print(json.dumps({
    "suite": suite_name,
    "flat_task_id": task_id,
    "level": int(level),
    "level_id": int(level_id),
    "task_name": task.name,
    "language": str(language),
}))
"""
    env = os.environ.copy()
    env["SAFETY_ID_SUITE"] = suite_name
    env["SAFETY_ID_TASK"] = str(task_id)
    completed = subprocess.run(
        [sys.executable, "-c", code],
        env=env,
        check=False,
        capture_output=True,
        text=True,
    )
    payload = ""
    for line in reversed(completed.stdout.splitlines()):
        if line.startswith("{"):
            payload = line
            break
    if completed.returncode != 0 or not payload:
        detail = (completed.stderr or completed.stdout).strip()
        raise RuntimeError(
            f"safety task identity failed for {suite_name} task {task_id}: {detail}"
        )
    data = json.loads(payload)
    return {
        "suite": str(data["suite"]),
        "flat_task_id": int(data["flat_task_id"]),
        "level": int(data["level"]),
        "level_id": int(data["level_id"]),
        "task_name": str(data["task_name"]),
        "language": str(data["language"]),
    }


def quat2axisangle(quat: np.ndarray) -> np.ndarray:
    quat = np.asarray(quat, dtype=np.float64).copy()
    quat[3] = float(np.clip(quat[3], -1.0, 1.0))
    den = math.sqrt(max(0.0, 1.0 - quat[3] * quat[3]))
    if math.isclose(den, 0.0):
        return np.zeros(3, dtype=np.float32)
    return ((quat[:3] * 2.0 * math.acos(quat[3])) / den).astype(np.float32)


def _constraint_key(constraint) -> str:
    return "::".join(str(item) for item in constraint)


def _robot_geom_names(domain) -> list[str]:
    names = []
    sim = domain.sim
    for geom_index in range(sim.model.ngeom):
        name = sim.model.geom_id2name(geom_index)
        if not name or int(sim.model.geom_group[geom_index]) != 0:
            continue
        if "gripper" in name or "robot" in name:
            names.append(name)
    return names


def read_constraint_audit(domain) -> dict:
    """Read every BDDL constraint without writing simulation state.

    ``official_predicate`` is one call of the benchmark predicate per constraint.
    ``audited_cost`` keeps that value for object-object contact. Robot contact
    uses geom names: the benchmark stores geom ids and then tests names against
    that list, so its own predicate stays at 0 during a real collision.
    """
    geom_names = _robot_geom_names(domain)
    official_predicate = {}
    audited_cost = {}
    catalog = []
    for constraint in domain.parsed_problem["constraints"]:
        key = _constraint_key(constraint)
        predicate = str(constraint[0])
        arguments = [str(item) for item in constraint[1:]]
        raw = int(bool(domain._eval_predicate(constraint)))
        official_predicate[key] = raw
        if predicate == "checkrobotcontact" and len(arguments) == 1 and geom_names:
            audited = int(
                bool(domain.check_contact(geom_names, domain.get_object(arguments[0])))
            )
        else:
            audited = raw
        audited_cost[key] = audited
        catalog.append({"key": key, "predicate": predicate, "arguments": arguments})
    return {
        "catalog": catalog,
        "official_predicate": official_predicate,
        "audited_cost": audited_cost,
        "robot_geom_names": geom_names,
    }


def _batch_obs(raw: dict, language: str) -> dict:
    image = np.ascontiguousarray(np.asarray(raw["agentview_image"])[::-1, ::-1])
    wrist = np.ascontiguousarray(np.asarray(raw["robot0_eye_in_hand_image"])[::-1, ::-1])
    state = np.concatenate(
        [
            np.asarray(raw["robot0_eef_pos"], dtype=np.float32),
            quat2axisangle(raw["robot0_eef_quat"]),
            np.asarray(raw["robot0_gripper_qpos"], dtype=np.float32),
        ]
    ).astype(np.float32)
    return {
        "main_images": image[None],
        "wrist_images": wrist[None],
        "extra_view_images": None,
        "states": state[None],
        "task_descriptions": [str(language)],
    }


class _Closer:
    def __init__(self, env: SafetyRpcEnv) -> None:
        self._env = env

    def close(self) -> None:
        self._env.close()


class SafetyRpcEnv:
    """One safety task, flattened the way RPent addresses ``--suite --task``."""

    harness_native = True

    def __init__(
        self,
        suite_name: str,
        task_id: int,
        seed: int,
        max_episode_steps: int,
        *,
        audit: bool = True,
    ) -> None:
        if suite_name not in SAFETY_SUITES:
            raise KeyError(f"{suite_name} is not a LIBERO-Safety suite")
        import torch

        torch.cuda.init()
        from libero.libero.benchmark import get_benchmark
        from libero.libero.envs import OffScreenRenderEnv

        suite = get_benchmark(suite_name)()
        if not 0 <= task_id < suite.n_tasks:
            raise ValueError(
                f"task_id must be in [0, {suite.n_tasks}) for suite {suite_name!r}; got {task_id}"
            )
        per_level = suite.get_num_tasks_by_level(0)
        if per_level <= 0 or suite.n_tasks % per_level != 0:
            raise RuntimeError(f"{suite_name} has no even level split")
        level = task_id // per_level
        level_id = task_id % per_level
        task = suite.get_task(task_id)
        leveled = suite.get_task_by_level_id(level, level_id)
        if leveled is None or leveled.name != task.name or int(task.level) != level:
            raise RuntimeError(
                f"flat task {task_id} does not match level {level} id {level_id}"
            )
        bddl = Path(suite.get_task_bddl_file_path_by_level_id(level, level_id))
        if f"{suite_name}/L{level}/" not in bddl.as_posix():
            raise RuntimeError(f"safety BDDL is missing its level directory: {bddl}")
        bddl_root = next(parent for parent in bddl.parents if parent.name == "bddl_files")
        benchmark = bddl_root.parent
        init_path = (
            benchmark
            / "init_files"
            / bddl.parent.relative_to(bddl_root)
            / f"{bddl.stem}.pruned_init"
        )
        if not init_path.is_file():
            raise FileNotFoundError(init_path)
        language = getattr(task, "language", None) or task.name.replace("_", " ")
        self._init_bank = _load_inits(init_path)
        self.suite_name = suite_name
        self.task_id = int(task_id)
        self.level = int(level)
        self.level_id = int(level_id)
        self.task_name = task.name
        self.language = str(language)
        self.bddl_path = str(bddl)
        self.init_path = str(init_path)
        self.seed = int(seed)
        self.audit = bool(audit)
        self.init_bank_size = int(len(self._init_bank))
        self.max_episode_steps = int(max_episode_steps)
        self._sim = OffScreenRenderEnv(
            bddl_file_name=str(bddl),
            camera_heights=256,
            camera_widths=256,
            camera_depths=True,
        )
        self.env = _Closer(self)
        self.terminated = False
        self.truncated = False
        self.steps = 0
        self.positive_steps = 0
        self.first_positive = None
        self.max_robot = 0
        self.max_contact = 0
        self.last_cost: dict = {}
        self.settle_steps = 0
        self.first_action = None
        self.last_action = None
        self.action_absmax: list[float] = []
        self.eef_start = None
        self.eef_end = None
        self.image_mean_first = None
        self.image_mean_last = None
        self.step_audits: list[dict] = []
        self._raw = None
        self._batched = None
        self.task_descriptions = [self.language]
        self.current_raw_obs = [None]
        self.reset()

    def _mujoco_sim(self):
        return self._sim.env.sim

    def get_camera_meta(
        self, camera_name: str = "agentview", height: int = 256, width: int = 256
    ) -> dict:
        """Read intrinsics, cam-to-world, and depth planes without stepping."""
        from robosuite.utils import camera_utils

        sim = self._mujoco_sim()
        intrinsic = camera_utils.get_camera_intrinsic_matrix(
            sim, camera_name, int(height), int(width)
        )
        extrinsic = camera_utils.get_camera_extrinsic_matrix(sim, camera_name)
        extent = float(sim.model.stat.extent)
        near = float(sim.model.vis.map.znear) * extent
        far = float(sim.model.vis.map.zfar) * extent
        return {
            "camera_name": camera_name,
            "height": int(height),
            "width": int(width),
            "intrinsic_K": intrinsic.tolist(),
            "extrinsic_cam2world": extrinsic.tolist(),
            "depth_near": near,
            "depth_far": far,
        }

    def render_camera(
        self,
        camera_name: str = "agentview",
        height: int = 1024,
        width: int = 1024,
        depth: bool = False,
    ):
        """Render one camera on the environment process that owns the GL context."""
        return self._mujoco_sim().render(
            width=int(width),
            height=int(height),
            camera_name=camera_name,
            depth=bool(depth),
        )

    def safety_trace(self) -> dict:
        return {
            "suite": self.suite_name,
            "flat_task_id": self.task_id,
            "level": self.level,
            "level_local_id": self.level_id,
            "task_name": self.task_name,
            "language": self.language,
            "bddl_path": self.bddl_path,
            "init_state_path": self.init_path,
            "seed": self.seed,
            "init_index": self.seed % self.init_bank_size,
            "init_bank_size": self.init_bank_size,
            "auditor": {
                "where": "env_process",
                "include_in_baseline": True,
                "key": "predicate::arguments",
                "robot_contact": "geom names in group 0 whose names contain gripper or robot",
            },
            "control_steps": self.steps,
            "settle_steps": self.settle_steps,
            "first_action": self.first_action,
            "last_action": self.last_action,
            "action_absmax": self.action_absmax,
            "eef_start": self.eef_start,
            "eef_end": self.eef_end,
            "image_mean_first": self.image_mean_first,
            "image_mean_last": self.image_mean_last,
            "constraint_positive_steps": self.positive_steps,
            "constraint_first_positive_step": self.first_positive,
            "constraint_max": {
                "checkrobotcontact": self.max_robot,
                "checkcontact": self.max_contact,
            },
            "official_constraint_cost_last": self.last_cost,
            "check_success": bool(self._sim.check_success()),
            "constraint_catalog": [
                [str(item) for item in constraint]
                for constraint in self._sim.env.parsed_problem["constraints"]
            ],
            "step_audits": self.step_audits,
            "audited_positive_steps": sum(
                1
                for record in self.step_audits
                if record["phase"] == "control" and any(record["audited_cost"].values())
            ),
            "settle_audited_positive_steps": sum(
                1
                for record in self.step_audits
                if record["phase"] == "settle" and any(record["audited_cost"].values())
            ),
            "success_transition": next(
                (
                    record
                    for record in self.step_audits
                    if record["phase"] == "control" and record["success"]
                ),
                None,
            ),
        }

    def scene_snapshot(self) -> dict:
        """Read object poses and robot contact. This does not step the simulator."""
        domain = self._sim.env
        sim = domain.sim
        reading = read_constraint_audit(domain)
        names: list[str] = []
        for source in ("objects_dict", "fixtures_dict"):
            names.extend(getattr(domain, source, {}) or {})
        geom_names = reading["robot_geom_names"]
        body_pos = {}
        robot_contact = {}
        for name in names:
            try:
                position = sim.data.body_xpos[domain.obj_body_id[name]]
                body_pos[name] = [float(value) for value in position[:3]]
            except Exception as exc:
                body_pos[name] = None
                robot_contact[name] = f"{type(exc).__name__}: {exc}"
                continue
            try:
                robot_contact[name] = int(
                    bool(domain.check_contact(geom_names, domain.get_object(name)))
                )
            except Exception as exc:
                robot_contact[name] = f"{type(exc).__name__}: {exc}"
        raw = self._raw or {}
        eef = raw.get("robot0_eef_pos")
        grip = raw.get("robot0_gripper_qpos")
        return {
            "check_success": bool(self._sim.check_success()),
            "eef_pos": (
                [float(value) for value in np.asarray(eef).reshape(-1)[:3]]
                if eef is not None
                else None
            ),
            "gripper_qpos": (
                [float(value) for value in np.asarray(grip).reshape(-1)[:2]]
                if grip is not None
                else None
            ),
            "body_pos": body_pos,
            "robot_contact": robot_contact,
            "audited_cost": reading["audited_cost"],
            "official_predicate": reading["official_predicate"],
        }

    def harness_action_spec(self) -> dict:
        from rpent.robots.components.action_spec import box_action_spec

        spec = getattr(self._sim, "action_spec", None)
        if spec is not None and hasattr(spec, "shape"):
            low, high = np.asarray(spec.low, dtype=float), np.asarray(spec.high, dtype=float)
        else:
            low = np.full(7, -1.0)
            high = np.full(7, 1.0)
        return {
            "default": box_action_spec(
                low.reshape(-1),
                high.reshape(-1),
                "LIBERO-Safety OSC action, shape (7,), gripper -1 opens and +1 closes.",
            )
        }

    def reset(self):
        self.terminated = False
        self.truncated = False
        self.steps = 0
        self.positive_steps = 0
        self.first_positive = None
        self.max_robot = 0
        self.max_contact = 0
        self.last_cost = {}
        self.settle_steps = 0
        self.first_action = None
        self.last_action = None
        self.action_absmax = []
        self.eef_start = None
        self.eef_end = None
        self.image_mean_first = None
        self.image_mean_last = None
        self.step_audits = []
        self._sim.reset()
        index = self.seed % len(self._init_bank)
        raw = self._sim.set_init_state(np.asarray(self._init_bank[index], dtype=np.float64))
        wait = np.zeros(7, dtype=np.float32)
        wait[-1] = -1.0
        for _ in range(15):
            raw = self._consume(wait, count_control=False)[0]
            self.settle_steps += 1
        self._remember(raw)
        return self._batched, {}

    def step(self, action):
        if self.terminated or self.truncated:
            raise RuntimeError("step called after the safety episode finished")
        raw, reward, terminated, truncated, info = self._consume(
            _flat_action(action), count_control=True
        )
        self._remember(raw)
        return (
            self._batched,
            np.asarray([reward], dtype=np.float32),
            np.asarray([terminated]),
            np.asarray([truncated]),
            info,
        )

    def chunk_step(self, actions):
        rows = np.asarray(actions, dtype=np.float32)
        if rows.ndim == 3:
            rows = rows[0]
        obs_list = []
        rewards = []
        terms = []
        truncs = []
        info: dict = {}
        for row in rows:
            if self.terminated or self.truncated:
                obs_list.append(self._batched)
                rewards.append(0.0)
                terms.append(bool(self.terminated))
                truncs.append(bool(self.truncated))
                continue
            _obs, reward, terminated, truncated, info = self.step(row)
            obs_list.append(self._batched)
            rewards.append(float(np.asarray(reward).reshape(-1)[0]))
            terms.append(bool(np.asarray(terminated).reshape(-1)[0]))
            truncs.append(bool(np.asarray(truncated).reshape(-1)[0]))
        return (
            obs_list,
            np.asarray(rewards, dtype=np.float32)[None],
            np.asarray(terms)[None],
            np.asarray(truncs)[None],
            info,
        )

    def close(self) -> None:
        self._dump_auditor_trace()
        if self._sim is not None:
            self._sim.close()
            self._sim = None

    def _dump_auditor_trace(self) -> None:
        """Write the per-step auditor record beside the run, not into the prompt."""
        path = os.environ.get("RPENT_SAFETY_TRACE_PATH", "")
        if not path or self._sim is None:
            return
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(self.safety_trace(), default=str))

    def _remember(self, raw: dict) -> None:
        self._raw = raw
        self._batched = _batch_obs(raw, self.language)
        self.current_raw_obs = [raw]
        self.task_descriptions = [self.language]

    def _consume(self, action: np.ndarray, *, count_control: bool):
        if count_control and self.eef_start is None and self._raw is not None:
            self.eef_start = [
                float(x) for x in np.asarray(self._raw["robot0_eef_pos"]).reshape(-1)[:3]
            ]
        result = self._sim.step(np.asarray(action, dtype=np.float32))
        if len(result) == 5:
            raw, reward, terminated, truncated, info = result
            done = bool(terminated or truncated)
        else:
            raw, reward, done, info = result
            terminated, truncated = bool(done), False
        info = info if isinstance(info, dict) else {}
        success = bool(self._sim.check_success())
        cost = info.get("cost") if isinstance(info.get("cost"), dict) else {}
        if count_control:
            self.steps += 1
            self.eef_end = [float(x) for x in np.asarray(raw["robot0_eef_pos"]).reshape(-1)[:3]]
            if self.first_action is None:
                self.first_action = [float(x) for x in action]
            self.last_action = [float(x) for x in action]
            image = np.asarray(raw["agentview_image"])
            mean = float(image.mean())
            if self.image_mean_first is None:
                self.image_mean_first = mean
            self.image_mean_last = mean
            if self.steps % 10 == 1:
                self.action_absmax.append(float(np.max(np.abs(action))))
            if cost:
                robot_cost = int(cost.get("checkrobotcontact", 0) or 0)
                contact_cost = int(cost.get("checkcontact", 0) or 0)
                self.max_robot = max(self.max_robot, robot_cost)
                self.max_contact = max(self.max_contact, contact_cost)
                self.last_cost = dict(cost)
                if robot_cost or contact_cost:
                    self.positive_steps += 1
                    if self.first_positive is None:
                        self.first_positive = self.steps
            if success:
                self.terminated = True
            elif done or self.steps >= self.max_episode_steps:
                self.truncated = True
        phase = "control" if count_control else "settle"
        index = self.steps if count_control else self.settle_steps + 1
        self._record_constraints(cost, success, phase, index)
        return raw, reward, self.terminated, self.truncated, info

    def _record_constraints(
        self, official_cost: dict, success: bool, phase: str, index: int
    ) -> None:
        if not self.audit:
            return
        reading = read_constraint_audit(self._sim.env)
        self.step_audits.append(
            {
                "phase": phase,
                "step": index,
                "success": bool(success),
                "official_cost": dict(official_cost),
                "official_predicate": reading["official_predicate"],
                "audited_cost": reading["audited_cost"],
            }
        )


def _flat_action(action) -> np.ndarray:
    values = np.asarray(action, dtype=np.float32).reshape(-1)
    if values.shape != (7,):
        raise ValueError(f"LIBERO harness action must be shape (7,), got {values.shape}")
    return values


def _load_inits(path: Path) -> np.ndarray:
    import torch

    loaded = torch.load(path, weights_only=False)
    if hasattr(loaded, "detach"):
        loaded = loaded.detach().cpu().numpy()
    loaded = np.asarray(loaded)
    if loaded.ndim == 1:
        loaded = loaded[None]
    return loaded
