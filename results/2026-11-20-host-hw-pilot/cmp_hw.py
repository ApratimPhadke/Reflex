import re, sys, numpy as np
sys.path.insert(0, sys.argv[1]); sys.argv = sys.argv[:2] + sys.argv[2:]
import train_policy as T
exec(open(sys.argv[2]).read(), globals()) if False else exec(open(sys.argv[2]).read().split("w = load")[0].split("import train_policy as T")[1])   # reuse load()
hw = load(sys.argv[3], "rx_polhw")
def track(policy_fn, label):
    orig = T.policy
    if policy_fn is not None: T.policy = policy_fn
    c, vpm, lost = T.rollout_hw(hw[None, :], np.random.default_rng(1), episodes=64, secs=60.0, stats=True)
    T.policy = orig
    print(f"{label:28s} cost {c[0]:.4f}  vetoes/min {vpm[0]:.2f}  lost {lost[0]}/64")
track(None, "hardware pilot")
track(lambda w, x, v, sp: T.safe_u(x, v, sp, T.HW_T_DELAY), "fallback law as the pilot")
