"""Offline check for a scene hazard inventory (DASH module 1).

Instead of letting the planner pick which objects to ground as obstacles,
ground every planner-visible object the task language does not name, with
SAM 3 on the frame ground_box uses, and compare with ground truth: is the
SafeLIBERO obstacle in the inventory, and how accurate is its box?

Boxes are built exactly as ground_box builds them (2nd-98th percentile of the
mask's world points). Ground truth (scene.json gt_boxes) is used only to score.

Usage (Pi0.5 venv, SAM3_CHECKPOINT_PATH set):
  hazard_inventory.py SCENES_DIR [--out summary.json]
"""

from __future__ import annotations

import argparse
import base64
import io
import json
import os
import re
import sys
from pathlib import Path

import importlib.util

import numpy as np
from PIL import Image


def load_sam3_facade():
    """Load sam3_server.py by path, as RPent runs it, without rpent.robots/__init__."""
    path = Path(os.environ["RPENT_REPO_ROOT"]) / "rpent" / "robots" / "components" / "sam3_server.py"
    spec = importlib.util.spec_from_file_location("sam3_server", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules["sam3_server"] = module  # dataclasses look the module up here
    spec.loader.exec_module(module)
    return module.Sam3Facade


def words(text: str) -> set[str]:
    return set(re.findall(r"[a-z]+", text.lower()))


def base_name(name: str) -> str:
    """``akita_black_bowl_2`` -> ``akita_black_bowl``."""
    return re.sub(r"_\d+$", "", name)


def task_named(name: str, language: str) -> bool:
    """The object's noun (last word of its name) appears in the task language."""
    return base_name(name).split("_")[-1] in words(language)


def box_from_mask(mask: np.ndarray, world: np.ndarray) -> dict | None:
    points = world[np.where(mask)].astype(np.float64)
    points = points[np.isfinite(points).all(axis=1) & (np.abs(points).sum(axis=1) > 1e-6)]
    if len(points) < 10:
        return None
    lo, hi = np.percentile(points, 2, axis=0), np.percentile(points, 98, axis=0)
    return {"min": lo.round(4).tolist(), "max": hi.round(4).tolist()}


def iou3(a: dict, b: dict) -> float:
    lo = np.maximum(a["min"], b["min"])
    hi = np.minimum(a["max"], b["max"])
    inter = float(np.prod(np.clip(hi - lo, 0, None)))
    vol = lambda box: float(np.prod(np.subtract(box["max"], box["min"])))  # noqa: E731
    return inter / (vol(a) + vol(b) - inter + 1e-12)


def centre(box: dict) -> np.ndarray:
    return (np.asarray(box["min"]) + np.asarray(box["max"])) / 2


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("scenes", type=Path)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    sam = load_sam3_facade()(os.environ["SAM3_CHECKPOINT_PATH"])
    rows = []
    for scene_dir in sorted(p for p in args.scenes.iterdir() if (p / "scene.json").is_file()):
        scene = json.loads((scene_dir / "scene.json").read_text())
        image_b64 = base64.b64encode((scene_dir / "image.png").read_bytes()).decode()
        world = np.load(scene_dir / "world.npz")["world"]
        hazards = {}
        for name in scene["objects"]:
            if task_named(name, scene["language"]):
                continue
            prompt = base_name(name).replace("_", " ")
            result = sam.segment(image_b64, text_prompt=prompt, min_score=0.2)
            box = None
            if result.get("found"):
                mask = np.asarray(Image.open(io.BytesIO(base64.b64decode(result["mask_png_base64"])))) > 0
                box = box_from_mask(mask, world)
            hazards[name] = {"prompt": prompt, "score": result.get("score"), "box": box}
        obstacle = scene["obstacle"]
        gt = scene["gt_boxes"].get(obstacle)
        found = hazards.get(obstacle, {}).get("box")
        row = {
            "cell": scene["cell"],
            "language": scene["language"],
            "obstacle": obstacle,
            "n_objects": len(scene["objects"]),
            "n_hazards": len(hazards),
            "obstacle_in_inventory": obstacle in hazards,
            "obstacle_grounded": found is not None,
            "obstacle_score": hazards.get(obstacle, {}).get("score"),
            "centre_error_xy_m": None,
            "top_error_m": None,
            "iou": None,
            "missed_hazards": sorted(n for n, h in hazards.items() if h["box"] is None),
        }
        if found is not None and gt:
            row["centre_error_xy_m"] = round(float(np.linalg.norm(centre(found)[:2] - centre(gt)[:2])), 4)
            row["top_error_m"] = round(float(found["max"][2] - gt["max"][2]), 4)
            row["iou"] = round(iou3(found, gt), 3)
        rows.append(row)
        print(
            f"{row['cell']:28s} {obstacle or '-':28s} inv={row['obstacle_in_inventory']!s:5s} "
            f"grounded={row['obstacle_grounded']!s:5s} xy_err={row['centre_error_xy_m']} "
            f"top_err={row['top_error_m']} iou={row['iou']} hazards={row['n_hazards']}/{row['n_objects']}",
            flush=True,
        )
    n = len(rows)
    errors = [r["centre_error_xy_m"] for r in rows if r["centre_error_xy_m"] is not None]
    summary = {
        "cells": n,
        "obstacle_in_inventory": sum(r["obstacle_in_inventory"] for r in rows),
        "obstacle_grounded": sum(r["obstacle_grounded"] for r in rows),
        "obstacle_within_5cm": sum(1 for e in errors if e <= 0.05),
        "median_centre_error_xy_m": round(float(np.median(errors)), 4) if errors else None,
        "mean_hazards_per_scene": round(float(np.mean([r["n_hazards"] for r in rows])), 2) if rows else None,
    }
    print(json.dumps(summary, indent=1))
    if args.out:
        args.out.write_text(json.dumps({"summary": summary, "rows": rows}, indent=1))


if __name__ == "__main__":
    main()
