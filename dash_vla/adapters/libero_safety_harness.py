"""LIBERO-Safety scene with the observation contract RPent's LIBERO harness uses.

RPent's ``LiberoPrimitives`` expects one environment observation dict:
``main_images``, ``wrist_images``, ``states``, ``task_descriptions``.
Images are already rotated 180 degrees, matching ``rlinf`` ``get_libero_image``.
The primitive then calls ``model.predict`` and ``env.chunk_step``.
"""

from __future__ import annotations

import math

import numpy as np

SAFETY_SUITES = (
    "affordance",
    "human_safety",
    "obstacle_avoidance",
    "obstacle_avoidance_human",
    "reasoning_safety",
)


def quat2axisangle(quat: np.ndarray) -> np.ndarray:
    quat = np.asarray(quat, dtype=np.float64).copy()
    quat[3] = float(np.clip(quat[3], -1.0, 1.0))
    den = math.sqrt(max(0.0, 1.0 - quat[3] * quat[3]))
    if math.isclose(den, 0.0):
        return np.zeros(3, dtype=np.float32)
    return ((quat[:3] * 2.0 * math.acos(quat[3])) / den).astype(np.float32)


def harness_obs(raw: dict, language: str) -> dict:
    image = np.asarray(raw["agentview_image"])[::-1, ::-1]
    wrist = np.asarray(raw["robot0_eye_in_hand_image"])[::-1, ::-1]
    state = np.concatenate(
        [
            np.asarray(raw["robot0_eef_pos"], dtype=np.float32),
            quat2axisangle(raw["robot0_eef_quat"]),
            np.asarray(raw["robot0_gripper_qpos"], dtype=np.float32),
        ]
    ).astype(np.float32)
    return {
        "main_images": np.ascontiguousarray(image),
        "wrist_images": np.ascontiguousarray(wrist),
        "states": state,
        "task_descriptions": str(language),
    }


class SafetyHarnessEnv:
    """Single-env surface used by ``LiberoPrimitives.chunk_step`` / ``step``.

    Official ``info['cost']`` is recorded on every step whose goal is still
    false. LIBERO-Safety clears that dict once ``check_success()`` is true.
    """

    def __init__(self, env, language: str, *, max_steps: int = 300) -> None:
        self._env = env
        self._language = language
        self._max_steps = max_steps
        self.terminated = False
        self.truncated = False
        self.return_all_frames = False
        self.steps = 0
        self.positive_steps = 0
        self.first_positive = None
        self.max_robot = 0
        self.max_contact = 0
        self.last_cost: dict | None = None
        self._last_obs: dict | None = None

    def validate_action(self, values) -> np.ndarray:
        action = np.asarray(values, dtype=np.float32).reshape(-1)
        if action.shape != (7,):
            raise ValueError(f"LIBERO harness action must be shape (7,), got {action.shape}")
        return action

    def _mark(self, info: dict, done: bool) -> None:
        success = bool(self._env.check_success())
        cost = info.get("cost") if isinstance(info, dict) else None
        if isinstance(cost, dict) and cost:
            robot_cost = int(cost.get("checkrobotcontact", 0) or 0)
            contact_cost = int(cost.get("checkcontact", 0) or 0)
            self.max_robot = max(self.max_robot, robot_cost)
            self.max_contact = max(self.max_contact, contact_cost)
            self.last_cost = cost
            if robot_cost or contact_cost:
                self.positive_steps += 1
                if self.first_positive is None:
                    self.first_positive = self.steps
        if success:
            self.terminated = True
        elif done:
            self.truncated = True
        if self.steps >= self._max_steps and not self.terminated:
            self.truncated = True

    def reset(self):
        self.terminated = False
        self.truncated = False
        self.steps = 0
        raw = self._env.reset()
        if isinstance(raw, tuple):
            raw = raw[0]
        self._last_obs = harness_obs(raw, self._language)
        return self._last_obs, {}

    def set_settled_obs(self, raw: dict) -> dict:
        self._last_obs = harness_obs(raw, self._language)
        return self._last_obs

    def step(self, action):
        if self.terminated or self.truncated:
            raise RuntimeError("step called after the harness episode finished")
        result = self._env.step(self.validate_action(action))
        if len(result) == 5:
            raw, reward, terminated, truncated, info = result
            done = bool(terminated or truncated)
        else:
            raw, reward, done, info = result
        self.steps += 1
        self._mark(info if isinstance(info, dict) else {}, bool(done))
        self._last_obs = harness_obs(raw, self._language)
        return (
            self._last_obs,
            reward,
            np.asarray(self.terminated),
            np.asarray(self.truncated),
            info if isinstance(info, dict) else {},
        )

    def chunk_step(self, actions, return_all_frames: bool | None = None):
        if return_all_frames is None:
            return_all_frames = self.return_all_frames
        obs_list = []
        rewards = []
        terms = []
        truncs = []
        infos = []
        for action in np.asarray(actions):
            if self.terminated or self.truncated:
                obs_list.append(self._last_obs)
                rewards.append(0.0)
                terms.append(self.terminated)
                truncs.append(self.truncated)
                infos.append({})
                continue
            obs, reward, term, trunc, info = self.step(action)
            obs_list.append(obs)
            rewards.append(reward)
            terms.append(bool(np.asarray(term)))
            truncs.append(bool(np.asarray(trunc)))
            infos.append(info)
        obs_field = obs_list if return_all_frames else obs_list[-1]
        return (
            obs_field,
            np.asarray(rewards, dtype=np.float32),
            np.asarray(terms),
            np.asarray(truncs),
            infos[-1] if infos else {},
        )
