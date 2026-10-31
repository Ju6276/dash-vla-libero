"""Write the safety filter's record of an explore run into the scene's memory.

After a DASH-VLA explore run has merged its notes, this appends to the scene's
task-family note what the safety filter did: the calls of the successful
(final) attempt that the filter changed, with the closest protected point, and
the approaches it blocked in earlier attempts. The planner reads the note when
it evaluates the scene, so the next run can keep away from those points from
the start instead of relying on the filter again.

  cbf_memory.py RUN_OUT_DIR MEMORY_DIR CELL      (CELL e.g. scene_spatial_I_t0_s0)
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

HEADER = "## Safety filter record"
# AEGIS's filter (RPENT_CBF=aegis) reports the obstacle ellipsoid's centre and the ellipsoid gap.
POINT = "the obstacle ellipsoid centred at" if os.environ.get("RPENT_CBF") == "aegis" else "the unmarked point"


def cm(values) -> str:
    return "[" + ", ".join(f"{float(v):.2f}" for v in values) + "]" if values else "?"


def line(event: dict) -> str:
    where = f"{event['tool']}"
    args = event.get("args") or {}
    if "xyz" in args:
        where += f" to {cm(args['xyz'])}"
    elif "prompt" in args:
        where += f" \"{args['prompt']}\""
    where += f" from {cm(event['eef_start'])}"
    part = "held object" if event.get("closest_part") == "held" else "hand"
    clear = event.get("min_clearance_m")
    clear = "?" if clear is None else f"{100 * clear:.1f} cm"
    if event["blocked"]:
        return f"- {where}: blocked; the {part} could not get closer than {clear} to {POINT} {cm(event['closest_unmarked_point'])}."
    return (
        f"- {where}: changed on {event['intervened_steps']} of {event['of_steps']} steps; "
        f"the {part} came within {clear} of {POINT} {cm(event['closest_unmarked_point'])}."
    )


def main() -> None:
    out, mem, cell = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]
    log = out / "cbf_interventions.jsonl"
    events = [json.loads(l) for l in log.read_text().splitlines() if l.strip()] if log.exists() else []
    attempts: list[list[dict]] = []
    for e in events:
        key = (e["pid"], e["episode"])
        if not attempts or (attempts[-1][0]["pid"], attempts[-1][0]["episode"]) != key:
            attempts.append([])
        attempts[-1].append(e)
    final = attempts[-1] if attempts else []
    changed = [e for e in final if e["blocked"] or e["intervened_steps"]]
    earlier = [e for a in attempts[:-1] for e in a if e["blocked"]]
    suite, task = cell.removeprefix("scene_").rsplit("_t", 1)[0], cell.rsplit("_t", 1)[1].split("_s")[0]
    note = mem / "task-family" / f"task-family_libero_scene_{suite}_{suite.rsplit('_', 1)[1]}_t{task}.md"
    if not note.exists():
        # An unsolved explore merges no task-family note; there is nothing to annotate.
        print(f"cbf_memory: no task-family note {note.name}; skipped")
        return
    text = note.read_text()
    if HEADER in text:
        text = text[: text.index(HEADER)].rstrip() + "\n"
    body = [HEADER, "", "Written automatically from the safety filter's log of the explore run."]
    if changed:
        body += ["", "In the successful attempt the filter changed these calls. Plan the next run's motions to keep farther from these points, so the filter does not have to act:"]
        body += [line(e) for e in changed]
    else:
        body += ["", "In the successful attempt the filter changed no motion."]
    if earlier:
        body += ["", "Approaches the filter blocked in earlier attempts (do not repeat them):"]
        body += [line(e) for e in earlier]
    note.write_text(text.rstrip() + "\n\n" + "\n".join(body) + "\n")
    print(f"cbf_memory: {note.name}: {len(changed)} changed calls, {len(earlier)} earlier blocked approaches")


if __name__ == "__main__":
    main()
