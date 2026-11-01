#!/usr/bin/env python3
"""Does the safety filter's intervention signal precede collisions?

Offline, from runs already done. For each tool call of the CBF arms it joins
  - cbf_interventions.jsonl (run dir): intervened_steps, of_steps, min_clearance_m
    (for RPENT_CBF=aegis this is the smallest barrier value h, not a distance)
  - tool_steps.jsonl (audit dir): start_step, end_step of each call
  - safety_trace.json (audit dir): collide_step
and reports, per tool, calls that contain the first collision against calls
that do not, plus how the call before a colliding call looked.

  analyze_cbf_signal.py ARM [ARM ...] [--out OUT.json]
"""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

ROOT = Path("/homeL/dong/Projects/dash-vla-libero")


def rows(path: Path) -> list[dict]:
    return [json.loads(l) for l in path.read_text().splitlines() if l.strip()] if path.exists() else []


def episode_calls(run: Path, audit: Path) -> tuple[list[dict], dict] | None:
    trace = audit / "safety_trace.json"
    if not trace.exists():
        return None
    t = json.loads(trace.read_text())
    steps = rows(audit / "tool_steps.jsonl")
    inter = rows(run / "cbf_interventions.jsonl")
    # the final attempt is the one the trace describes
    if steps:
        ep = steps[-1]["episode"]
        steps = [s for s in steps if s["episode"] == ep]
        inter = [r for r in inter if r.get("episode") == ep]
    logged = {r["tool"] for r in inter}
    steps = [s for s in steps if s["tool"] in logged]
    if len(steps) != len(inter) or any(s["tool"] != r["tool"] for s, r in zip(steps, inter)):
        return [], {"mismatch": (len(steps), len(inter)), **t}
    calls = []
    for s, r in zip(steps, inter):
        n = max(int(r["of_steps"]), 1)
        calls.append({
            "tool": s["tool"],
            "start": s["start_step"],
            "end": s["end_step"],
            "rate": int(r["intervened_steps"]) / n,
            "intervened": int(r["intervened_steps"]),
            "h_min": r["min_clearance_m"],
            "blocked": bool(r.get("blocked")),
        })
    return calls, t


def label(calls: list[dict], collide: int | None) -> None:
    for c in calls:
        c["collides"] = collide is not None and c["start"] < collide <= c["end"]
        c["after"] = collide is not None and c["start"] >= collide


def describe(xs: list[float]) -> str:
    if not xs:
        return "n=0"
    xs = sorted(xs)
    q = lambda p: xs[min(len(xs) - 1, int(p * len(xs)))]
    return f"n={len(xs)} median={q(0.5):.3f} q25={q(0.25):.3f} q75={q(0.75):.3f} min={xs[0]:.3f} max={xs[-1]:.3f}"


def auc(pos: list[float], neg: list[float]) -> float | None:
    """P(score of a colliding call > score of a clean call), ties count half."""
    if not pos or not neg:
        return None
    wins = sum((p > n) + 0.5 * (p == n) for p in pos for n in neg)
    return wins / (len(pos) * len(neg))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("arms", nargs="+")
    ap.add_argument("--out")
    args = ap.parse_args()

    by_tool = defaultdict(lambda: {"hit": [], "clean": []})
    preceding = []
    episodes = []
    skipped = []
    for arm in args.arms:
        for phase in ("explore", "eval"):
            for run in sorted((ROOT / "runs/p2" / arm / phase).glob("scene_*")):
                got = episode_calls(run, ROOT / "runs/p2_audit" / arm / phase / run.name)
                if got is None:
                    skipped.append(f"{arm}/{phase}/{run.name}: no trace")
                    continue
                calls, t = got
                if "mismatch" in t:
                    skipped.append(f"{arm}/{phase}/{run.name}: calls {t['mismatch']}")
                    continue
                collide = t["collide_step"] if t["collided"] else None
                label(calls, collide)
                ep = {"arm": arm, "phase": phase, "run": run.name, "collide_step": collide,
                      "pick_intervened": sum(c["intervened"] for c in calls if c["tool"] == "pi0_pick"),
                      "pick_calls": sum(c["tool"] == "pi0_pick" for c in calls)}
                for i, c in enumerate(calls):
                    if c["after"]:
                        continue
                    by_tool[c["tool"]]["hit" if c["collides"] else "clean"].append(c)
                    if c["collides"]:
                        before = [p for p in calls[:i] if p["tool"] == "pi0_pick"]
                        ep["colliding_call"] = {**c, "steps_into_call": collide - c["start"]}
                        ep["previous_pick"] = before[-1] if before else None
                        preceding.append(ep)
                episodes.append(ep)

    print(f"episodes={len(episodes)} skipped={len(skipped)}")
    for s in skipped:
        print("  skipped", s)
    report = {"tools": {}, "colliding": preceding}
    for tool, d in sorted(by_tool.items()):
        hit, clean = d["hit"], d["clean"]
        hr = [c["rate"] for c in hit]
        cr = [c["rate"] for c in clean]
        hh = [c["h_min"] for c in hit if c["h_min"] is not None]
        ch = [c["h_min"] for c in clean if c["h_min"] is not None]
        print(f"\n[{tool}] calls with collision {len(hit)}, without {len(clean)}")
        print("  intervention rate  hit  ", describe(hr))
        print("  intervention rate  clean", describe(cr))
        print("  h_min              hit  ", describe(hh))
        print("  h_min              clean", describe(ch))
        a_rate = auc(hr, cr)
        a_h = auc([-x for x in hh], [-x for x in ch])
        print(f"  AUC rate={a_rate} AUC(-h_min)={a_h}")
        for thr in (0.1, 0.2, 0.3, 0.5):
            tp = sum(r >= thr for r in hr)
            fp = sum(r >= thr for r in cr)
            print(f"  rate>={thr}: flags {tp}/{len(hr)} colliding calls, {fp}/{len(cr)} clean calls")
        report["tools"][tool] = {"hit": len(hit), "clean": len(clean), "auc_rate": a_rate, "auc_neg_h": a_h}

    print("\n[colliding calls]")
    for ep in preceding:
        c, p = ep["colliding_call"], ep["previous_pick"]
        prev = "none" if p is None else f"rate={p['rate']:.2f} h={p['h_min']}"
        print(f"  {ep['arm']:22s} {ep['phase']:7s} {ep['run']:28s} {c['tool']:9s} "
              f"rate={c['rate']:.2f} h={c['h_min']} into={c['steps_into_call']}/{c['end'] - c['start']} "
              f"prev_pick: {prev}")

    hit_eps = [e for e in episodes if e["collide_step"] is not None]
    clean_eps = [e for e in episodes if e["collide_step"] is None]
    print("\n[episodes] pi0_pick intervened steps per episode")
    print("  collided", describe([e["pick_intervened"] for e in hit_eps]))
    print("  clean   ", describe([e["pick_intervened"] for e in clean_eps]))
    if args.out:
        Path(args.out).write_text(json.dumps(report, indent=1, default=str))


if __name__ == "__main__":
    main()
