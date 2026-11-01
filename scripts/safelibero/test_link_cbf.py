"""Checks of the link constraints in aegis_cbf.py, on robosuite Lift (same Panda and OSC as LIBERO), no rendering."""
import os, sys
import numpy as np
sys.path.insert(0, "/homeL/dong/Projects/dash-vla-libero/vendor/RPent-harness")
LINKS = os.environ.get("TEST_LINKS", "robot0_link5:2,robot0_link6")
os.environ["RPENT_CBF_LINKS"] = LINKS
import robosuite as suite
from robots.libero import aegis_cbf as A

def make():
    return suite.make("Lift", robots="Panda", controller_configs=suite.load_controller_config(default_controller="OSC_POSE"),
                      has_renderer=False, has_offscreen_renderer=False, use_camera_obs=False, control_freq=20)

env = make(); obs = env.reset(); sim = env.sim
f = A.AegisCbf(sim, 0.05)
# 1. ellipsoids
for n, (body, c, r, q) in f.links.items():
    print("ellipsoid", n, "centre", np.round(c, 3), "semi-axes", np.round(q, 3))
# 2. Jacobian map: predicted vs finite-difference link-centre motion
errs = []
for trial in range(20):
    f.observe(obs)
    p1, R1 = f._eef()
    before = {n: f._link_pose(n)[0] for n in f.links}
    M = {n: f._link_map(n, p1) for n in f.links}
    act = np.zeros(7); act[:6] = np.random.uniform(-0.5, 0.5, 6)
    obs, *_ = env.step(act)
    f.observe(obs)
    p1b, R1b = f._eef()
    dv = p1b - p1
    dw = A.R.from_matrix(R1b @ R1.T).as_rotvec()
    for n in f.links:
        pred = (M[n] @ np.hstack([dv, dw]))[:3]
        real = f._link_pose(n)[0] - before[n]
        errs.append(np.linalg.norm(pred - real) / (np.linalg.norm(real) + 1e-9))
print("jacobian map relative error median %.3f p90 %.3f" % (np.median(errs), np.percentile(errs, 90)))

def body_vertices(model, body):
    import mujoco
    index = model.body_name2id(body); pts = []
    for g in range(model.ngeom):
        if model.geom_bodyid[g] != index or model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0:
            continue
        m = model.geom_dataid[g]; a, k = model.mesh_vertadr[m], model.mesh_vertnum[m]
        rot = A.R.from_quat(np.roll(model.geom_quat[g], -1)).as_matrix()
        pts.append(np.asarray(model.geom_pos[g]) + np.asarray(model.mesh_vert[a:a + k]) @ rot.T)
    return np.vstack(pts) if pts else None

VERTS = {n: v for n in ("robot0_link5", "robot0_link6", "robot0_link7", "gripper0_right_gripper") if (v := body_vertices(sim.model, n)) is not None}

# 3. closed loop: a virtual obstacle in the path of link6, EE-only filter vs with links
def levels(sim, p2, Q2):
    """Lowest level over each body's collision-mesh vertices: < 1 means a vertex is inside the obstacle."""
    out = {}
    for n, v in VERTS.items():
        xm = np.asarray(sim.data.get_body_xmat(n)); x = np.asarray(sim.data.get_body_xpos(n)) + v @ xm.T
        out[n] = float(np.linalg.norm((x - p2) / Q2, axis=1).min())
    return out


def run(links, act, direction, steps, clearance=1.3, Q2=(0.05, 0.05, 0.05)):
    """Obstacle sphere placed along `direction` from link6's ellipsoid centre, as close as possible
    while every vertex of link5 / link6 / link7 / gripper stays at level >= clearance; then `act` for `steps`."""
    env = make(); obs = env.reset()
    os.environ["RPENT_CBF_LINKS"] = "robot0_link6"
    probe = A.AegisCbf(env.sim, 0.05)
    p6 = probe._link_pose("robot0_link6")[0]
    os.environ["RPENT_CBF_LINKS"] = links
    f = A.AegisCbf(env.sim, 0.05); f.observe(obs)
    if os.environ.get("TEST_ORI_RATIO"):  # e.g. 1: the first version, rotation in position units
        f.ori_ratio = float(os.environ["TEST_ORI_RATIO"])
    Q2 = np.array(Q2); u = np.array(direction, float); u /= np.linalg.norm(u)
    for d in np.arange(0.05, 0.6, 0.005):
        if min(levels(env.sim, p6 + d * u, Q2).values()) >= clearance:
            break
    f.p2 = p6 + d * u; f.Q2 = Q2; f.R2 = np.eye(3)
    f.Q1 = np.array([0.06, 0.12, 0.11]); f.active = True
    p1, _ = f._eef(); f.z = (f.p2 - p1) / np.linalg.norm(f.p2 - p1)
    f.z_links = {n: (f.p2 - f._link_pose(n)[0]) / np.linalg.norm(f.p2 - f._link_pose(n)[0]) for n in f.links}
    worst = levels(env.sim, f.p2, Q2); inter = 0; hmin = None
    for t in range(steps):
        out = f.filter(np.array(act, float)); inter += f.last["intervened"]
        if f.last["h_links"]:
            hmin = min([hmin if hmin is not None else 9] + list(f.last["h_links"].values()))
        obs, *_ = env.step(out); f.observe(obs)
        for n, v in levels(env.sim, f.p2, Q2).items():
            worst[n] = min(worst[n], v)
    tag = "links" if links else "hand only"
    hs = "" if hmin is None else f" min link h {hmin:+.3f}"
    print(f"  {tag:9s} interventions {inter:3d}  min vertex level " + " ".join(f"{k.split('_')[-1]}={v:.2f}" for k, v in worst.items()) + hs)
    return min(worst.values())


cases = [
    ("translate -y", (0, -0.6, 0, 0, 0, 0, -1.0), (0, -1, 0), 80),
    ("translate +x", (0.6, 0, 0, 0, 0, 0, -1.0), (1, 0, 0), 80),
    ("translate -z", (0, 0, -0.6, 0, 0, 0, -1.0), (0, 0, -1), 80),
]
for pitch in (0.8, -0.8):
    for name, direction in (("up", (0, 0, 1)), ("front", (1, 0, 0)), ("back", (-1, 0, 0)), ("side", (0, 1, 0))):
        cases.append((f"pitch {pitch:+.1f}, obstacle {name}", (0, 0, 0, pitch, 0, 0, -1.0), direction, 40))
cases.append(("yaw +0.8, obstacle side", (0, 0, 0, 0, 0, 0.8, -1.0), (0, 1, 0), 40))
cases.append(("pitch +0.6 and -y, obstacle side", (0, -0.4, 0, 0.6, 0, 0, -1.0), (0, -1, 0), 60))
TRIALS = int(os.environ.get("TEST_TRIALS", "5"))
summary = []
for name, act, direction, steps in cases:
    print(name)
    a, b = [], []
    for trial in range(TRIALS):
        np.random.seed(trial)
        a.append(run("", act, direction, steps))
        np.random.seed(trial)
        b.append(run(LINKS, act, direction, steps))
    summary.append((name, a, b))
print(f"SUMMARY over {TRIALS} starts: penetrations (lowest vertex level < 1) and median lowest level")
tot = [0, 0]
for name, a, b in summary:
    pa, pb = sum(x < 1 for x in a), sum(x < 1 for x in b)
    tot[0] += pa; tot[1] += pb
    print(f"  {name:36s} hand only {pa}/{TRIALS} ({np.median(a):.2f})   links {pb}/{TRIALS} ({np.median(b):.2f})")
print(f"  {'total':36s} hand only {tot[0]}/{TRIALS * len(summary)}   links {tot[1]}/{TRIALS * len(summary)}")
