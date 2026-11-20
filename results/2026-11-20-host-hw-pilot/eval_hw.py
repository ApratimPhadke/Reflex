# evaluate a weights header under rollout_hw: square-wave-only and mixed setpoints
import re, sys, numpy as np
sys.path.insert(0, sys.argv[1])
import train_policy as T
def load(path, prefix):
    s = open(path).read()
    nums = lambda name: [float(q) for q in re.findall(r'-?[0-9.]+(?:e-?[0-9]+)?', s[s.index(name):].split(';')[0].split('=')[1])]
    w1 = nums(prefix + "_w1["); b1 = nums(prefix + "_b1["); w2 = nums(prefix + "_w2[")
    b2 = float(re.search(prefix.upper() + r'_B2 (-?[0-9.e-]+)f', s).group(1))
    return np.array(w1 + b1 + w2 + [b2])
w = load(sys.argv[2], sys.argv[3])
c, vpm, lost = T.rollout_hw(w[None, :], np.random.default_rng(1), episodes=64, secs=60.0, stats=True)
print(f"{sys.argv[2].split('/')[-1]}: cost {c[0]:.4f}  vetoes/min {vpm[0]:.2f}  episodes lost {lost[0]}/64")
