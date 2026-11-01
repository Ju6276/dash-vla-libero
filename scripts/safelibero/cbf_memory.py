"""Write the safety filter's record of an explore run into the scene's memory.

After a DASH-VLA explore run has merged its notes, this appends to the scene's
task-family note what the safety filter did: the calls of the successful
(final) attempt that the filter changed, with the closest protected point, and
the approaches it blocked in earlier attempts. The planner reads the note when
it evaluates the scene, so the next run can keep away from those points from
the start instead of relying on the filter again.

With SL_CBF_MEMORY=2 (AEGIS filter) the record is written relative to the
obstacle instead of in table coordinates, since the obstacle and the objects
move between episodes: each call's start is given as an offset from the
obstacle ellipsoid's centre, and the hand-overs to the VLA are summarised as
the horizontal distance at which the filter did and did not have to act.

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
RELATIVE = os.environ.get("RPENT_CBF") == "aegis" and os.environ.get("SL_CBF_MEMORY") == "2"
VLA_TOOLS = ("pi0_pick", "pi0_doubled")


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


def offset(event: dict) -> tuple[list[float], float]:
    """Start of the call relative to the obstacle centre, and its horizontal distance."""
    start, centre = event["eef_start"], event["closest_unmarked_point"]
    d = [float(a) - float(b) for a, b in zip(start, centre)]
    return d, (d[0] ** 2 + d[1] ** 2) ** 0.5


def relative_line(event: dict) -> str:
    where = event["tool"]
    prompt = (event.get("args") or {}).get("prompt")
    if prompt:
        where += f" \"{prompt}\""
    d, xy = offset(event)
    start = f"started {xy:.2f} m from the obstacle's centre horizontally (offset {cm(d)} from it)"
    if event["blocked"]:
        return f"- {where}: {start}; blocked by the filter."
    return f"- {where}: {start}; changed on {event['intervened_steps']} of {event['of_steps']} steps."


def relative_body(final: list[dict], earlier: list[dict]) -> list[str]:
    body = ["", "Written automatically from the safety filter's log of the explore run. "
            "Positions are relative to the obstacle, which may sit elsewhere next time."]
    picks = [e for e in final if e["tool"] in VLA_TOOLS and e.get("closest_unmarked_point")]
    if picks:
        hit = [offset(e)[1] for e in picks if e["blocked"] or e["intervened_steps"]]
        clean = [offset(e)[1] for e in picks if not (e["blocked"] or e["intervened_steps"])]
        body += ["", "Hand-overs to the VLA in the successful attempt:"]
        body += [relative_line(e) for e in picks]
        if hit:
            body += [f"The filter had to act on VLA grasps that started up to {max(hit):.2f} m from the obstacle's centre. "
                     "Start the next grasps farther than that, on the side of the target away from the obstacle."]
        elif clean:
            body += [f"The filter did not act on VLA grasps that started {min(clean):.2f} m or more from the obstacle's centre."]
    moves = [e for e in final if e["tool"] not in VLA_TOOLS and (e["blocked"] or e["intervened_steps"])]
    if moves:
        body += ["", "Other motions the filter changed:"]
        body += [relative_line(e) for e in moves]
    if earlier:
        body += ["", "Approaches the filter blocked in earlier attempts (do not repeat them):"]
        body += [relative_line(e) for e in earlier]
    return body


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
    if RELATIVE:
        note.write_text(text.rstrip() + "\n\n" + "\n".join([HEADER] + relative_body(final, earlier)) + "\n")
        print(f"cbf_memory: {note.name}: relative record, {len(final)} calls")
        return
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
