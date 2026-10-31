#!/usr/bin/env python3
"""AEGIS's obstacle perception for one episode, run by robots/libero/aegis_cbf.py.

The env process renders agentview and backview at AEGIS's resolution after the
settle steps and saves the images, the real depth maps and the camera matrices.
This script (in .venv-aegis, which has GroundingDINO, open3d and cvxpy) then does
what eval_libero of main_aegis.py does before its control loop:
  - names the obstacle with the planner model (run_aegis.codex_obstacle_detection,
    the AEGIS prompt in place of GLM-4.5V, as in the AEGIS arm);
  - GroundingDINO on both views and back-projection of the first box
    (point_cloud below is utils.get_point_cloud with the three lines that read
    env.sim given their results);
  - utils.filtering_points and utils.fit_ellipse, unchanged.

  aegis_perceive.py IN.npz OUT.json WORKDIR
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path("/homeL/dong/Projects/dash-vla-libero")
sys.path[:0] = [str(ROOT / "third_party/vlsa-aegis/main"), str(ROOT / "scripts/safelibero")]


def point_cloud(image, depth, K_inv, T_cam_to_world, view, TEXT_PROMPT, model, save_path):
    """utils.get_point_cloud; depth is already the real depth map, K_inv and T come from the env."""
    from groundingdino.util.inference import load_image, predict, annotate
    DEVICE = "cuda"
    BOX_TRESHOLD = 0.35
    TEXT_TRESHOLD = 0.25
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.imsave(str(save_path / f"{view}.png"), image)
    import cv2
    IMAGE_PATH = str(save_path / f"{view}.png")
    image_source, image_cv = load_image(IMAGE_PATH)
    boxes, logits, phrases = predict(
        model=model,
        image=image_cv,
        caption=TEXT_PROMPT,
        box_threshold=BOX_TRESHOLD,
        text_threshold=TEXT_TRESHOLD,
        device=DEVICE,
    )
    annotated_frame = annotate(image_source=image_source, boxes=boxes, logits=logits, phrases=phrases)
    cv2.imwrite(str(save_path / f"annotated_ {view}_image.jpg"), annotated_frame)
    from groundingdino.util.box_ops import box_cxcywh_to_xyxy
    import torch
    image = image[::-1, ::-1]
    depth = depth[::-1, ::-1].squeeze()
    if boxes.shape[0] > 0:
        h, w, _ = image.shape
        boxes_xyxy = box_cxcywh_to_xyxy(boxes)
        size_tensor = torch.tensor([w, h, w, h], device=boxes.device)
        pixel_boxes = boxes_xyxy * size_tensor
        first_box = pixel_boxes[0].cpu().numpy().astype(int)
        x1, y1, x2, y2 = first_box
        xmin = w - 1 - x2
        xmax = w - 1 - x1
        ymin = h - 1 - y2
        ymax = h - 1 - y1
        xmin = max(0, xmin)
        ymin = max(0, ymin)
        xmax = min(w, xmax)
        ymax = min(h, ymax)
        cropped_image_rgb = image[ymin:ymax, xmin:xmax]
        cropped_image_depth = depth[ymin:ymax, xmin:xmax]
        if cropped_image_rgb.size == 0:
            print(f"Crop failed, invalid bounding box coordinates: [{xmin}, {ymin}, {xmax}, {ymax}]")
            return np.array([[]])
    else:
        print("GroundingDINO detected no objects, unable to crop.")
        return np.array([[]])
    h_full, w_full = image.shape[0], image.shape[1]
    v_full, u_full = np.indices((h_full, w_full))
    v_full = (h_full - 1) - v_full
    cropped_u = u_full[ymin:ymax, xmin:xmax]
    cropped_v = v_full[ymin:ymax, xmin:xmax]
    u_flat = cropped_u.flatten()
    v_flat = cropped_v.flatten()
    depth_flat = cropped_image_depth.flatten()
    pixels_homo = np.stack([u_flat, v_flat, np.ones_like(u_flat)], axis=0)
    points_cam = K_inv @ pixels_homo * depth_flat
    points_cam_homo = np.vstack([points_cam, np.ones_like(depth_flat)])
    points_world_homo = T_cam_to_world @ points_cam_homo
    return points_world_homo[:3, :].T


def main() -> None:
    src, dst, work = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
    work.mkdir(parents=True, exist_ok=True)
    data = np.load(src)
    suite = str(data["suite"])
    language = str(data["language"])

    from run_aegis import codex_obstacle_detection
    from utils import filtering_points, fit_ellipse
    from groundingdino.util.inference import load_model

    name = codex_obstacle_detection(data["agentview_image"], language, suite)
    import groundingdino

    model = load_model(
        str(Path(groundingdino.__file__).parent / "config/GroundingDINO_SwinT_OGC.py"),
        str(ROOT / "checkpoints/groundingdino/groundingdino_swint_ogc.pth"),
    )
    views = []
    for view in ("agentview", "backview"):
        pts = point_cloud(
            data[f"{view}_image"], data[f"{view}_depth"], data[f"{view}_K_inv"], data[f"{view}_T"],
            view, name, model, work,
        )
        views.append(pts)
    a, b = views
    if a.shape[1] > 0 and b.shape[1] > 0:
        full = np.vstack([a, b])
    elif a.shape[1] == 0 and b.shape[1] > 0:
        full = b
    elif a.shape[1] > 0 and b.shape[1] == 0:
        full = a
    else:
        full = np.array([[]])
    out = {"obstacle_named": name, "active": False, "points": 0}
    if full.size:
        filtered = filtering_points(full, suite)
        out["points"] = int(filtered.shape[0])
        if filtered.shape[0] > 0:
            p2, R2, Q2 = fit_ellipse(filtered, plot=True, save_path=work)
            out.update(active=True, p2=np.asarray(p2).tolist(), R2=np.asarray(R2).tolist(), Q2=np.asarray(Q2).tolist())
    dst.write_text(json.dumps(out))
    print("aegis_perceive", json.dumps(out), flush=True)


if __name__ == "__main__":
    os.environ.setdefault("TOKENIZERS_PARALLELISM", "false")
    main()
