import sys, numpy as np
from robots.libero.env_server import make_env
for suite, task in [("safelibero_spatial_I", 0), ("obstacle_avoidance", 0)]:
    try:
        env = make_env(task, 1, suite, 300)
    except Exception as e:
        print(suite, "ERR", type(e).__name__, e); continue
    raw = env._raw if getattr(env, "_raw", None) is not None else None
    def eef():
        return np.asarray(env._sim.env.sim.data.site_xpos[env._sim.env.robots[0].eef_site_id]).copy()
    p0 = eef()
    for a in ([0.5,0,0,0,0,0,-1], [0,0.5,0,0,0,0,-1], [0,0,0,0,0,0,-1]):
        start = eef()
        for _ in range(10):
            env.step(np.asarray(a, dtype=np.float32)[None])
        print(suite, a[:3], "delta", (eef() - start).round(3))
    print(suite, "controller", type(env._sim.env.robots[0].controller).__name__,
          getattr(env._sim.env.robots[0].controller, "use_delta", None),
          getattr(env._sim.env.robots[0].controller, "input_max", None),
          getattr(env._sim.env.robots[0].controller, "output_max", None),
          "control_freq", env._sim.env.control_freq)
    env.close()
