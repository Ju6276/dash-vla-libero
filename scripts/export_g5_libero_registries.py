#!/usr/bin/env python3
"""Export G5-L0 registries from the installed LIBERO packages.

Read-only with respect to RPent and LIBERO-Safety sources. The task list comes
from get_benchmark() plus the upstream task map, not from a guessed index.
"""

from __future__ import annotations

import argparse
import ast
import json
import os
import sys
from pathlib import Path


SAFETY_SUITES = (
    "affordance",
    "human_safety",
    "obstacle_avoidance",
    "obstacle_avoidance_human",
    "reasoning_safety",
)


def worktree_root() -> Path:
    return Path(__file__).resolve().parents[1]


def ensure_isolated_config(config_dir: Path, benchmark_root: Path) -> None:
    """Write a worktree-local LIBERO config before importing libero.

    Upstream import creates ~/.libero/config.yaml and may prompt on stdin when
    that file is missing. This function never writes outside config_dir.
    """
    home_config = Path.home() / ".libero"
    if config_dir == home_config or home_config in config_dir.parents:
        raise RuntimeError(f"refusing to write LIBERO config under {home_config}")
    config_dir.mkdir(parents=True, exist_ok=True)
    datasets = worktree_root() / "runs" / "g5_libero" / "datasets_placeholder"
    datasets.mkdir(parents=True, exist_ok=True)
    payload = {
        "benchmark_root": str(benchmark_root),
        "bddl_files": str(benchmark_root / "bddl_files"),
        "init_states": str(benchmark_root / "init_files"),
        "datasets": str(datasets),
        "assets": str(benchmark_root / "assets"),
    }
    text = "".join(f"{key}: {value}\n" for key, value in payload.items())
    (config_dir / "config.yaml").write_text(text)
    os.environ["LIBERO_CONFIG_PATH"] = str(config_dir)


def load_suite_names(robot_spec: Path) -> list[str]:
    tree = ast.parse(robot_spec.read_text())
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if isinstance(target, ast.Name) and target.id == "LIBERO_SUITE_NAMES":
                value = ast.literal_eval(node.value)
                if not isinstance(value, (list, tuple)) or not all(
                    isinstance(item, str) for item in value
                ):
                    raise TypeError("LIBERO_SUITE_NAMES is not a tuple of strings")
                return list(value)
    raise RuntimeError(f"LIBERO_SUITE_NAMES not found in {robot_spec}")


def rel_or_abs(path: str) -> str:
    return str(Path(path).resolve())


def export_rpent(robot_spec: Path, out_path: Path) -> dict:
    from libero.libero import get_libero_path
    from libero.libero.benchmark import BENCHMARK_MAPPING, get_benchmark

    allowlist = load_suite_names(robot_spec)
    registered = sorted(BENCHMARK_MAPPING)
    safety_in_standard = [name for name in SAFETY_SUITES if name in BENCHMARK_MAPPING]
    entries = []
    unregistered = []
    for suite_name in allowlist:
        if suite_name not in BENCHMARK_MAPPING:
            unregistered.append(suite_name)
            continue
        suite = get_benchmark(suite_name)()
        for task_id in range(suite.n_tasks):
            task = suite.get_task(task_id)
            bddl_path = rel_or_abs(suite.get_task_bddl_file_path(task_id))
            init_path = rel_or_abs(
                os.path.join(
                    get_libero_path("init_states"),
                    task.problem_folder,
                    task.init_states_file,
                )
            )
            entries.append(
                {
                    "suite": suite_name,
                    "task_id": task_id,
                    "task_name": task.name,
                    "language": task.language,
                    "problem": task.problem,
                    "problem_folder": task.problem_folder,
                    "bddl_file": task.bddl_file,
                    "bddl_path": bddl_path,
                    "bddl_exists": Path(bddl_path).is_file(),
                    "init_states_file": task.init_states_file,
                    "init_state_path": init_path,
                    "init_state_exists": Path(init_path).is_file(),
                }
            )
    outside = [name for name in registered if name not in allowlist]
    payload = {
        "kind": "rpent_standard_libero_registry",
        "authority": [
            "robots/libero/robot_spec.py LIBERO_SUITE_NAMES",
            "libero.libero.benchmark.get_benchmark",
            "libero.libero.benchmark.libero_suite_task_map",
        ],
        "robot_spec_path": str(robot_spec),
        "allowlist": allowlist,
        "benchmark_mapping_keys": registered,
        "safety_suites_in_standard_benchmark": safety_in_standard,
        "unregistered_allowlist_names": unregistered,
        "registered_outside_allowlist": outside,
        "n_registered_allowlist_suites": len(allowlist) - len(unregistered),
        "n_tasks_exported": len(entries),
        "missing_bddl": [row["bddl_path"] for row in entries if not row["bddl_exists"]],
        "missing_init_states": [
            row["init_state_path"] for row in entries if not row["init_state_exists"]
        ],
        "entries": entries,
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")
    return payload


def export_safety(out_path: Path) -> dict:
    from libero.libero import get_libero_path
    from libero.libero.benchmark import BENCHMARK_MAPPING, get_benchmark
    from libero.libero.benchmark.vla_safety_task_map import vla_safety_task_map

    entries = []
    discrepancies = []
    suite_counts = {}
    for suite_name in SAFETY_SUITES:
        if suite_name not in vla_safety_task_map:
            discrepancies.append(
                {"suite": suite_name, "error": "missing_from_vla_safety_task_map"}
            )
            continue
        if suite_name not in BENCHMARK_MAPPING:
            discrepancies.append(
                {"suite": suite_name, "error": "missing_from_get_benchmark"}
            )
            continue
        suite = get_benchmark(suite_name)()
        level_keys = sorted(vla_safety_task_map[suite_name])
        per_level = {}
        for level in level_keys:
            map_names = list(vla_safety_task_map[suite_name][level])
            per_level[str(level)] = len(map_names)
            official_n = suite.get_num_tasks_by_level(level)
            if official_n != len(map_names):
                discrepancies.append(
                    {
                        "suite": suite_name,
                        "level": level,
                        "error": "count_mismatch",
                        "task_map": len(map_names),
                        "get_benchmark": official_n,
                    }
                )
            for level_id, map_name in enumerate(map_names):
                task = suite.get_task_by_level_id(level, level_id)
                if task is None:
                    discrepancies.append(
                        {
                            "suite": suite_name,
                            "level": level,
                            "level_local_id": level_id,
                            "error": "get_task_by_level_id_returned_none",
                            "map_task_name": map_name,
                        }
                    )
                    continue
                bddl_path = suite.get_task_bddl_file_path_by_level_id(level, level_id)
                init_path = os.path.join(
                    get_libero_path("init_states"),
                    task.problem_folder,
                    f"L{task.level}",
                    task.init_states_file,
                )
                bddl_resolved = rel_or_abs(bddl_path) if bddl_path else None
                init_resolved = rel_or_abs(init_path)
                entries.append(
                    {
                        "suite": suite_name,
                        "level": int(task.level),
                        "level_local_id": int(task.level_id),
                        "task_name": task.name,
                        "map_task_name": map_name,
                        "name_matches_task_map": task.name == map_name,
                        "language": task.language,
                        "problem": task.problem,
                        "problem_folder": task.problem_folder,
                        "bddl_file": task.bddl_file,
                        "bddl_path": bddl_resolved,
                        "bddl_exists": bool(bddl_resolved) and Path(bddl_resolved).is_file(),
                        "init_states_file": task.init_states_file,
                        "init_state_path": init_resolved,
                        "init_state_exists": Path(init_resolved).is_file(),
                    }
                )
                if task.name != map_name or task.level_id != level_id or task.level != level:
                    discrepancies.append(
                        {
                            "suite": suite_name,
                            "level": level,
                            "level_local_id": level_id,
                            "error": "task_identity_mismatch",
                            "map_task_name": map_name,
                            "benchmark_task_name": task.name,
                            "benchmark_level": task.level,
                            "benchmark_level_id": task.level_id,
                        }
                    )
        suite_counts[suite_name] = {
            "n_level_tasks": sum(per_level.values()),
            "by_level": per_level,
            "n_unique_task_names": len(
                {
                    row["task_name"]
                    for row in entries
                    if row["suite"] == suite_name
                }
            ),
        }
    unique_names = sorted({row["task_name"] for row in entries})
    payload = {
        "kind": "libero_safety_level_task_registry",
        "note": (
            "These rows are level-task configurations. "
            "Different levels may reuse a semantic task name, so the row count "
            "is not a count of mutually distinct task names."
        ),
        "authority": [
            "libero.libero.benchmark.vla_safety_task_map.vla_safety_task_map",
            "libero.libero.benchmark.get_benchmark",
        ],
        "suites_requested": list(SAFETY_SUITES),
        "suite_counts": suite_counts,
        "n_level_tasks": len(entries),
        "n_unique_task_names": len(unique_names),
        "expected_level_tasks": 75,
        "registry_rows_aligned": len(entries) == 75 and not discrepancies,
        "files_complete": not any(not row["bddl_exists"] or not row["init_state_exists"] for row in entries),
        "complete": (
            len(entries) == 75
            and not discrepancies
            and all(row["bddl_exists"] and row["init_state_exists"] for row in entries)
        ),
        "discrepancies": discrepancies,
        "missing_bddl": [row["bddl_path"] for row in entries if not row["bddl_exists"]],
        "missing_init_states": [
            row["init_state_path"] for row in entries if not row["init_state_exists"]
        ],
        "entries": entries,
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")
    return payload


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--which", choices=("rpent", "safety"), required=True)
    parser.add_argument("--benchmark-root", type=Path, required=True)
    parser.add_argument("--config-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--robot-spec", type=Path)
    args = parser.parse_args()
    home_before = (Path.home() / ".libero").exists()
    ensure_isolated_config(args.config_dir.resolve(), args.benchmark_root.resolve())
    if args.which == "rpent":
        if args.robot_spec is None:
            parser.error("--robot-spec is required for --which rpent")
        payload = export_rpent(args.robot_spec, args.output)
        print(
            json.dumps(
                {
                    "output": str(args.output),
                    "allowlist": len(payload["allowlist"]),
                    "unregistered": payload["unregistered_allowlist_names"],
                    "n_tasks_exported": payload["n_tasks_exported"],
                    "safety_suites_in_standard_benchmark": payload[
                        "safety_suites_in_standard_benchmark"
                    ],
                },
                ensure_ascii=False,
            )
        )
    else:
        payload = export_safety(args.output)
        print(
            json.dumps(
                {
                    "output": str(args.output),
                    "n_level_tasks": payload["n_level_tasks"],
                    "n_unique_task_names": payload["n_unique_task_names"],
                    "complete": payload["complete"],
                    "suite_counts": payload["suite_counts"],
                    "n_missing_init": len(payload["missing_init_states"]),
                    "n_discrepancies": len(payload["discrepancies"]),
                },
                ensure_ascii=False,
            )
        )
    home_after = (Path.home() / ".libero").exists()
    if home_after and not home_before:
        print("ERROR: import created ~/.libero", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
