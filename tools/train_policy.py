#!/usr/bin/env python3
"""Train the Reflex neural pilot and export it as a C header.

The pilot is a 3-12-1 tanh MLP. It is first fitted to an aggressive PD law
(behaviour cloning) and then improved with the cross-entropy method on a
vectorised copy of the ball-and-beam simulator in src/core/rx_core.c.

The pilot is deliberately NOT trusted: it is rewarded for speed, and nothing
in training guarantees it stays inside the safe envelope. That guarantee is the
guardian's job.

usage: python3 tools/train_policy.py [--iters 40] [--out src/core/policy_weights.h]
"""
import argparse
import numpy as np

# must match rx_cfg_default() in src/core/rx_core.c
HALF_LEN, EDGE, K_ACC, TH_MAX, SLEW = 0.20, 0.010, 5.886, 0.21, 6.0
T_DELAY, ETA, EST_TAU = 0.08, 0.7, 0.03
NI, NH = 3, 12


def unpack(w):
    """w: (P, nparams) -> W1 (P,NH,NI), b1 (P,NH), w2 (P,NH), b2 (P,)"""
    i = 0
    W1 = w[:, i:i + NH * NI].reshape(-1, NH, NI); i += NH * NI
    b1 = w[:, i:i + NH]; i += NH
    w2 = w[:, i:i + NH]; i += NH
    b2 = w[:, i]
    return W1, b1, w2, b2


NPARAM = NH * NI + NH + NH + 1


def policy(w, x, v, sp):
    """x, v, sp: (P, E). returns u (P, E)"""
    W1, b1, w2, b2 = unpack(w)
    f = np.stack([(sp - x) / 0.1, v / 0.3, x / 0.2], axis=-1)           # (P,E,3)
    h = np.tanh(np.einsum('pji,pei->pej', W1, f) + b1[:, None, :])        # (P,E,NH)
    o = np.einsum('pj,pej->pe', w2, h) + b2[:, None]
    return TH_MAX * np.tanh(o)


def margin(x, v):
    """Two-sided recoverable-set margin, same formula as rx_margin() in rx_core.c."""
    a_max = K_ACC * np.sin(TH_MAX)

    def side(s):
        xs, vs = s * x, s * v
        xd = xs + vs * T_DELAY + 0.5 * a_max * T_DELAY ** 2
        vd = vs + a_max * T_DELAY
        reach = np.maximum(xs, xd)
        reach = np.where(vd > 0, np.maximum(reach, xd + vd ** 2 / (2 * ETA * a_max)), reach)
        return (HALF_LEN - EDGE) - reach

    return np.minimum(side(1.0), side(-1.0))


def rollout(w, rng, episodes=24, hz=250.0, secs=6.0):
    """Returns cost per population member."""
    P = w.shape[0]
    dt = 1.0 / hz
    n = int(secs * hz)
    x = np.tile(rng.uniform(-0.10, 0.10, (1, episodes)), (P, 1))
    v = np.tile(rng.uniform(-0.05, 0.05, (1, episodes)), (P, 1))
    th = np.zeros((P, episodes))
    sp = np.tile(rng.uniform(-0.10, 0.10, (1, episodes)), (P, 1))
    xe, ve = x.copy(), np.zeros_like(x)
    k_scale = np.tile(rng.uniform(0.85, 1.15, (1, episodes)), (P, 1))     # domain randomisation
    slew = np.tile(rng.uniform(4.0, 8.0, (1, episodes)), (P, 1))
    thf = np.exp(-dt / EST_TAU)
    alpha, beta = 1 - thf * thf, (1 - thf) ** 2
    cost = np.zeros(P)
    dead = np.zeros((P, episodes), bool)
    switch = rng.integers(int(1.5 * hz), int(3.0 * hz), episodes)
    for t in range(n):
        flip = (t % switch) == (switch - 1)
        if flip.any():
            newsp = rng.uniform(-0.10, 0.10, episodes)
            sp = np.where(flip[None, :], newsp[None, :], sp)
        u = policy(w, xe, ve, sp)
        for _ in range(2):                                               # 2 physics substeps
            h = dt / 2
            th += np.clip(u - th, -slew * h, slew * h)
            a = K_ACC * k_scale * np.sin(th) - 0.15 * v
            v += a * h
            x += v * h
        z = x + rng.normal(0, 0.0015, (1, episodes))
        xp = xe + ve * dt
        r = z - xp
        xe = xp + alpha * r
        ve = ve + beta * r / dt
        dead |= np.abs(x) > HALF_LEN
        m = margin(x, v)
        c = (x - sp) ** 2 * 100 + 0.05 * u ** 2 + 4.0 * np.clip(0.035 - m, 0, None) * 100
        cost += np.where(dead, 50.0, c).mean(axis=1)
    return cost / n


def clone_pd(rng, steps=4000, lr=0.02):
    """Fit the MLP to an aggressive PD law so CEM starts from something that balances."""
    wn, zeta = 5.0, 0.85
    w = rng.normal(0, 0.3, (1, NPARAM))
    m = np.zeros_like(w); s = np.zeros_like(w)
    for it in range(1, steps + 1):
        x = rng.uniform(-0.18, 0.18, 512); v = rng.uniform(-0.4, 0.4, 512); sp = rng.uniform(-0.12, 0.12, 512)
        a = wn * wn * (sp - x) - 2 * zeta * wn * v
        tgt = np.arcsin(np.clip(a / K_ACC, -np.sin(TH_MAX), np.sin(TH_MAX)))
        W1, b1, w2, b2 = unpack(w)
        f = np.stack([(sp - x) / 0.1, v / 0.3, x / 0.2], axis=-1)       # (B,3)
        hpre = f @ W1[0].T + b1[0]; h = np.tanh(hpre)
        o = h @ w2[0] + b2[0]; y = TH_MAX * np.tanh(o)
        dy = 2 * (y - tgt) / len(x)
        do = dy * TH_MAX * (1 - np.tanh(o) ** 2)
        gw2 = h.T @ do; gb2 = do.sum()
        dh = np.outer(do, w2[0]) * (1 - h ** 2)
        gW1 = dh.T @ f; gb1 = dh.sum(0)
        g = np.concatenate([gW1.ravel(), gb1, gw2, [gb2]])[None, :] * 1e3
        m = 0.9 * m + 0.1 * g; s = 0.999 * s + 0.001 * g * g
        w -= lr * (m / (1 - 0.9 ** it)) / (np.sqrt(s / (1 - 0.999 ** it)) + 1e-8)
    return w[0]


def export(w, path, score):
    W1, b1, w2, b2 = unpack(w[None, :])
    fl = lambda a: ", ".join(f"{float(q):.7g}f" for q in a)
    with open(path, "w") as f:
        f.write("/* GENERATED by tools/train_policy.py - do not edit by hand.\n")
        f.write(f" * 3-{NH}-1 tanh MLP, behaviour-cloned from PD then tuned with CEM. eval cost {score:.4f} */\n")
        f.write("#ifndef RX_POLICY_WEIGHTS_H\n#define RX_POLICY_WEIGHTS_H\n")
        f.write(f"#define RX_POL_IN {NI}\n#define RX_POL_H {NH}\n")
        f.write(f"static const float rx_pol_w1[RX_POL_H][RX_POL_IN] = {{\n")
        for j in range(NH):
            f.write(f"    {{ {fl(W1[0, j])} }},\n")
        f.write("};\n")
        f.write(f"static const float rx_pol_b1[RX_POL_H] = {{ {fl(b1[0])} }};\n")
        f.write(f"static const float rx_pol_w2[RX_POL_H] = {{ {fl(w2[0])} }};\n")
        f.write(f"#define RX_POL_B2 {float(b2[0]):.7g}f\n#endif\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=40)
    ap.add_argument("--pop", type=int, default=48)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--out", default="src/core/policy_weights.h")
    a = ap.parse_args()
    rng = np.random.default_rng(a.seed)

    mean = clone_pd(rng)
    print(f"cloned PD      cost {rollout(mean[None, :], np.random.default_rng(1))[0]:.4f}")
    sigma = np.full(NPARAM, 0.08)
    for it in range(a.iters):
        pop = mean + sigma * rng.normal(size=(a.pop, NPARAM))
        pop[0] = mean
        cost = rollout(pop, np.random.default_rng(1000 + it))
        elite = pop[np.argsort(cost)[: a.pop // 6]]
        mean = 0.5 * mean + 0.5 * elite.mean(0)
        sigma = 0.7 * sigma + 0.3 * elite.std(0) + 0.002
        if it % 5 == 0 or it == a.iters - 1:
            print(f"iter {it:3d}  best {cost.min():.4f}  mean-of-elite {np.sort(cost)[: a.pop // 6].mean():.4f}")
    score = rollout(mean[None, :], np.random.default_rng(1), episodes=64)[0]
    print(f"final          cost {score:.4f}")
    export(mean, a.out, score)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
