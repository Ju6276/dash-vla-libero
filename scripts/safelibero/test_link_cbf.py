"""Checks of the link constraints in aegis_cbf.py, on robosuite Lift (same Panda and OSC as LIBERO), no rendering."""
import os, sys
import numpy as np
sys.path.insert(0, "/homeL/dong/Projects/dash-vla-libero/vendor/RPent-harness")
os.environ["RPENT_CBF_LINKS"] = "robot0_link5,robot0_link6"
import robosuite as suite
from robots.libero import aegis_cbf as A

def make():
    return suite.make("Lift", robots="Panda", controller_configs=suite.load_controller_config(default_controller="OSC_POSE"),
                      has_renderer=False, has_offscreen_renderer=False, use_camera_obs=False, control_freq=20)

env = make(); obs = env.reset(); sim = env.sim
f = A.AegisCbf(sim, 0.05)
# 1. ellipsoids
for n, (c, r, q) in f.links.items():
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

# 3. closed loop: a virtual obstacle in the path of link6, EE-only filter vs with links
def run(links):
    env = make(); obs = env.reset()
    os.environ["RPENT_CBF_LINKS"] = links
    f = A.AegisCbf(env.sim, 0.05); f.observe(obs)
    # obstacle ellipsoid 12 cm ahead of link6 along -y, low enough that the hand passes above it
    pL6 = f._link_pose("robot0_link6")[0] if f.links else None
    probe = A.AegisCbf.__new__(A.AegisCbf); os.environ["RPENT_CBF_LINKS"] = "robot0_link6"; probe.bind(env.sim)
    p6 = probe._link_pose("robot0_link6")[0]
    os.environ["RPENT_CBF_LINKS"] = links
    f.p2 = p6 + np.array([0.0, -0.15, 0.0]); f.Q2 = np.array([0.05, 0.05, 0.05]); f.R2 = np.eye(3)
    f.Q1 = np.array([0.06, 0.12, 0.11]); f.active = True
    p1, _ = f._eef(); f.z = (f.p2 - p1) / np.linalg.norm(f.p2 - p1)
    f.z_links = {n: (f.p2 - f._link_pose(n)[0]) / np.linalg.norm(f.p2 - f._link_pose(n)[0]) for n in f.links}
    worst = {}; inter = 0
    for t in range(80):
        act = np.array([0, -0.6, 0, 0, 0, 0, -1.0])
        out = f.filter(act); inter += f.last["intervened"]
        obs, *_ = env.step(out); f.observe(obs)
        for n in ("robot0_link5", "robot0_link6", "robot0_link7", "robot0_right_hand"):
            # level of the body's collision vertices: < 1 means inside the obstacle ellipsoid
            c, r, q = A.link_ellipsoid(env.sim.model, n) if n != "robot0_right_hand" else (np.zeros(3), np.eye(3), np.ones(3) * 0.03)
            xm = np.asarray(env.sim.data.get_body_xmat(n)); x = np.asarray(env.sim.data.get_body_xpos(n)) + xm @ c
            lvl = float(np.linalg.norm((x - f.p2) / f.Q2))
            worst[n] = min(worst.get(n, 9), lvl)
    ee = obs["robot0_eef_pos"]
    print(f"links={links or 'none':28s} interventions {inter:3d}  min centre level " + " ".join(f"{k[7:]}={v:.2f}" for k, v in worst.items()), "ee_end", np.round(ee, 3))
run("")
run("robot0_link5,robot0_link6")
