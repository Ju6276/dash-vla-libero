"""Build one memory copy for the memory control experiment (report 05).

A scene's contribution to an RPent memory corpus is everything its explore run
merged: the task-family note, the task-specific trace (json and recipe), global
notes whose only evidence is that explore cell, and its _internal/merged dir.
Existing global notes are not modified by a merge in p2 (checked against the
published base memory), so removing these files removes the scene's experience
and nothing else.

  build_memctl.py SRC OUT --scene spatial_I                  # SRC minus the scene
  build_memctl.py SRC OUT --scene spatial_I --swap-from MEM  # scene part taken from MEM
  build_memctl.py SRC OUT                                    # plain copy

The index MEMORY.md is regenerated with upstream MemoryManager.rebuild_index.
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "vendor/RPent-harness"))
from rpent.memory.manager import MemoryManager  # noqa: E402


def scene_files(mem: Path, scene: str) -> list[Path]:
    """Files the explore of scene (e.g. 'spatial_I') merged into mem."""
    cell = f"scene_{scene}_t0_s0"
    regime = scene.rsplit("_", 1)[1]
    files = [
        mem / "task-family" / f"task-family_libero_scene_{scene}_{regime}_t0.md",
        mem / "task-specific" / f"{cell}.json",
        mem / "task-specific" / f"{cell}_recipe.jsonl",
        mem / "_internal" / "merged" / cell,
    ]
    for note in (mem / "global").glob("*.md"):
        text = note.read_text(errors="replace")
        end = text.find("\n---", 3)
        meta = yaml.safe_load(text[3:end]) if text.startswith("---") and end > 0 else {}
        cells = ((meta or {}).get("evidence") or {}).get("cells") or []
        if cells == [cell]:
            files.append(note)
    return [f for f in files if f.exists()]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("src", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--scene")
    ap.add_argument("--swap-from", type=Path)
    args = ap.parse_args()
    if args.out.exists():
        sys.exit(f"{args.out} exists")
    shutil.copytree(args.src, args.out)
    if args.scene:
        for f in scene_files(args.out, args.scene):
            shutil.rmtree(f) if f.is_dir() else f.unlink()
            print(f"removed {f.relative_to(args.out)}")
        if args.swap_from:
            for f in scene_files(args.swap_from, args.scene):
                dest = args.out / f.relative_to(args.swap_from)
                dest.parent.mkdir(parents=True, exist_ok=True)
                shutil.copytree(f, dest) if f.is_dir() else shutil.copy2(f, dest)
                print(f"added {dest.relative_to(args.out)}")
    # Every condition gets an index built the same way.
    MemoryManager(args.out).rebuild_index()


if __name__ == "__main__":
    main()
