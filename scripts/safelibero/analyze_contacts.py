#!/usr/bin/env python3
"""Which robot geoms touched the obstacle in the harness runs with the per-step filter log (report 13).

Reads runs/p2_audit/aegiscbf_*/eval/*/safety_trace.json (cbf_steps); run from the repo root."""
import json,glob,collections
first=collections.Counter(); anyc=collections.Counter(); closest=collections.Counter(); n=0
for f in sorted(glob.glob("runs/p2_audit/aegiscbf_*/eval/*/safety_trace.json")):
    t=json.load(open(f)); c=t.get("cbf_steps") or []
    if not c: continue
    n+=1
    hit=[x for x in c if x[5]]
    if not hit: continue
    arm=f.split("/")[2].replace("aegiscbf_",""); sc=f.split("/")[4]
    geoms=collections.Counter(g for x in hit for g in x[5])
    print(arm, sc, "collided",t.get("collided"), "first_contact_step",hit[0][0], hit[0][5], "closest_body",hit[0][4], "contact_steps",len(hit), dict(geoms.most_common(4)))
    for g in hit[0][5]: first[g]+=1
    for g in geoms: anyc[g]+=1
    closest[hit[0][4]]+=1
print("episodes with per-step log",n)
print("first contact geoms",first.most_common()); print("any contact geoms",anyc.most_common()); print("closest body at first contact",closest.most_common())
