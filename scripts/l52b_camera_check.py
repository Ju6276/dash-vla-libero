"""Check SafetyRpcEnv camera RPC without loading a VLA or a planner."""

from __future__ import annotations

import os

import numpy as np

os.environ.pop("MUJOCO_EGL_DEVICE_ID", None)

from robots.libero.safety_env import SafetyRpcEnv
from robots.libero.tools import _metric_depth, _world_from_depth


def _qpos(env: SafetyRpcEnv) -> np.ndarray:
    return np.asarray(env._mujoco_sim().data.qpos).copy()


def _nearest(world: np.ndarray, point: np.ndarray) -> float:
    flat = world.reshape(-1, 3)
    valid = np.isfinite(flat).all(axis=1) & (np.linalg.norm(flat, axis=1) > 1e-6)
    if not np.any(valid):
        return float("inf")
    return float(np.linalg.norm(flat[valid] - point, axis=1).min())


def main() -> None:
    env = SafetyRpcEnv("obstacle_avoidance", 0, 0, 30)
    before = _qpos(env)
    meta = env.get_camera_meta("agentview", 256, 256)
    rgb, depth = env.render_camera("agentview", 256, 256, depth=True)
    after = _qpos(env)
    raw = env._raw
    image = np.asarray(raw["agentview_image"])
    wrist = np.asarray(raw["robot0_eye_in_hand_image"])
    raw_depth = raw.get("agentview_depth")
    print(
        "meta",
        meta["camera_name"],
        meta["height"],
        round(meta["depth_near"], 4),
        round(meta["depth_far"], 4),
        "K",
        np.round(meta["intrinsic_K"], 3).tolist(),
    )
    print(
        "qpos_delta",
        float(np.max(np.abs(after - before))),
        "rgb",
        tuple(np.asarray(rgb).shape),
        float(np.asarray(rgb).mean()),
        "depth",
        tuple(np.asarray(depth).shape),
        float(np.nanmin(depth)),
        float(np.nanmax(depth)),
    )
    print(
        "raw_image",
        tuple(image.shape),
        round(float(image.mean()), 3),
        "wrist",
        tuple(wrist.shape),
        "raw_depth",
        None if raw_depth is None else tuple(np.asarray(raw_depth).shape),
    )
    metric = _metric_depth(np.asarray(depth)[::-1], meta)
    world = _world_from_depth(metric, meta)
    names = ("plate_1", "akita_black_bowl_1", "bottle_of_perfume_1")
    domain = env._sim.env
    for name in names:
        point = np.asarray(domain.sim.data.body_xpos[domain.obj_body_id[name]][:3])
        print(name, [round(float(v), 4) for v in point], "nearest_m", round(_nearest(world, point), 4))
    action = np.zeros(7, dtype=np.float32)
    action[0] = 0.8
    for _ in range(8):
        env.step(action)
    image_after = np.asarray(env._raw["agentview_image"])
    rgb_after, _ = env.render_camera("agentview", 256, 256, depth=True)
    print(
        "moved_raw_mean",
        round(float(image.mean()), 3),
        round(float(image_after.mean()), 3),
        "delta",
        round(float(image_after.mean() - image.mean()), 3),
        "max_abs",
        int(np.max(np.abs(image_after.astype(np.int16) - image.astype(np.int16)))),
        "render_delta",
        round(float(np.asarray(rgb_after).mean() - np.asarray(rgb).mean()), 3),
        "eef",
        [round(float(v), 3) for v in env.eef_end],
    )
    wrist_meta = env.get_camera_meta("robot0_eye_in_hand", 256, 256)
    wrist_rgb, wrist_depth = env.render_camera("robot0_eye_in_hand", 256, 256, depth=True)
    print(
        "wrist_meta",
        wrist_meta["camera_name"],
        round(wrist_meta["depth_near"], 4),
        "rgb",
        tuple(np.asarray(wrist_rgb).shape),
        "depth",
        tuple(np.asarray(wrist_depth).shape),
    )
    env.close()


if __name__ == "__main__":
    main()
