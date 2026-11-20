#!/usr/bin/env python3
"""Train the Reflex neural pilot and export it as a C header.

The pilot is a 3-12-1 tanh MLP. It is first fitted to an aggressive PD law
(behaviour cloning) and then improved with the cross-entropy method on a
vectorised copy of the ball-and-beam simulator in src/core/rx_core.c.

The pilot is deliberately NOT trusted: it is rewarded for speed, and nothing
in training guarantees it stays inside the safe envelope. That guarantee is the
guardian's job.

usage: python3 tools/train_policy.py [--iters 40] [--out src/core/policy_weights.h]
       python3 tools/train_policy.py --hardware     (-> src/core/policy_weights_hw.h)
       python3 tools/train_policy.py --plant joint  (-> src/core/policy_weights_joint.h)

--hardware trains the pilot the serial backend flies: 50 Hz and rx_cfg_hardware()
(longer actuator delay, slower estimator), a noisy sensor, one tick of pilot latency,
and the guardian itself in the loop: every command is judged with the same one-step
envelope check as rx_guard_step(), a vetoed pilot is replaced by rx_safe_u() for the
hold time, and each veto costs a lot. The pilot is still untrusted; this only makes
it a pilot the guardian rarely has to overrule on the real rig.
"""
import argparse
import numpy as np

# must match rx_cfg_default() in src/core/rx_core.c
HALF_LEN, EDGE, K_ACC, TH_MAX, SLEW = 0.20, 0.010, 5.886, 0.21, 6.0
T_DELAY, ETA, EST_TAU = 0.08, 0.7, 0.03
WN, ZETA, HOLD_S, INNER = 2.5, 1.0, 0.5, 0.03
# must match rx_cfg_hardware()
HW_HZ, HW_T_DELAY, HW_EST_TAU = 50.0, 0.12, 0.08
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


def net_raw(w, e, vn, xn):
    W1, b1, w2, b2 = unpack(w)
    f = np.stack([e, vn, xn], axis=-1)                                     # (P,E,3)
    h = np.tanh(np.einsum('pji,pei->pej', W1, f) + b1[:, None, :])        # (P,E,NH)
    return np.einsum('pj,pej->pe', w2, h) + b2[:, None]


def policy(w, x, v, sp):
    """x, v, sp: (P, E). returns u (P, E). Equilibrium-corrected like rx_policy_with():
    u = net(e, v, x) - net(0, 0, x), so the ball at rest on the setpoint gets exactly zero tilt."""
    z = np.zeros_like(x)
    o = net_raw(w, (sp - x) / 0.1, v / 0.3, x / 0.2) - net_raw(w, z, z, x / 0.2)
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


def margin_cfg(x, v, t_delay):
    """rx_margin() for an arbitrary t_delay. Returns (min margin, right margin, left margin)."""
    a_max = K_ACC * np.sin(TH_MAX)

    def side(s):
        xs, vs = s * x, s * v
        xd = xs + vs * t_delay + 0.5 * a_max * t_delay ** 2
        vd = vs + a_max * t_delay
        reach = np.maximum(xs, xd)
        reach = np.where(vd > 0, np.maximum(reach, xd + vd ** 2 / (2 * ETA * a_max)), reach)
        return (HALF_LEN - EDGE) - reach

    r, l = side(1.0), side(-1.0)
    return np.minimum(r, l), r, l


def safe_u(x, v, sp, t_delay):
    """rx_safe_u(): PD fallback, full push away from an end when close to the limit."""
    _, r, l = margin_cfg(x, v, t_delay)
    sm = np.sin(TH_MAX)
    u = np.arcsin(np.clip((WN * WN * (sp - x) - 2 * ZETA * WN * v) / K_ACC, -sm, sm))
    near = (r < 0.01) | (l < 0.01)
    return np.where(near, np.where(r < l, -TH_MAX, TH_MAX), u)


def rollout_hw(w, rng, episodes=24, secs=10.0, substeps=20, stats=False):
    """Hardware-config rollout with the guardian in the loop. Returns cost per population member
    (and, with stats=True, vetoes per minute and balls lost per member)."""
    P = w.shape[0]
    hz = HW_HZ
    dt = 1.0 / hz
    n = int(secs * hz)
    hold_ticks = int(HOLD_S * hz)
    x = np.tile(rng.uniform(-0.10, 0.10, (1, episodes)), (P, 1))
    v = np.tile(rng.uniform(-0.05, 0.05, (1, episodes)), (P, 1))
    th = np.zeros((P, episodes))
    k_scale = np.tile(rng.uniform(0.85, 1.15, (1, episodes)), (P, 1))     # domain randomisation
    slew = np.tile(rng.uniform(4.0, 8.0, (1, episodes)), (P, 1))
    noise = np.tile(rng.uniform(0.001, 0.003, (1, episodes)), (P, 1))     # sensor noise, std in m
    # setpoints: half the episodes fly the demo's +/-80 mm square wave (4 s half period, random
    # phase), the other half random targets every 1.5-3 s
    square = rng.random(episodes) < 0.5
    phase = rng.integers(0, int(4 * hz), episodes)
    switch = rng.integers(int(1.5 * hz), int(3.0 * hz), episodes)
    sp_rand = rng.uniform(-0.10, 0.10, episodes)
    thf = np.exp(-dt / HW_EST_TAU)
    alpha, beta = 1 - thf * thf, (1 - thf) ** 2
    xe, ve = x.copy(), np.zeros_like(x)
    u_prop = np.zeros_like(x)                  # the pilot answers one tick late
    ai = np.zeros((P, episodes), bool)          # guardian mode: the pilot has to earn control
    hold = np.full((P, episodes), hold_ticks)
    dead = np.zeros((P, episodes), bool)
    cost = np.zeros(P)
    vetoes = np.zeros(P)
    hl = HALF_LEN - EDGE
    for t in range(n):
        flip = (t % switch) == (switch - 1)
        sp_rand = np.where(flip, rng.uniform(-0.10, 0.10, episodes), sp_rand)
        sq = np.where(((t + phase) // int(4 * hz)) % 2 == 0, 0.08, -0.08)
        sp = np.broadcast_to(np.where(square, sq, sp_rand)[None, :], x.shape)
        # sensor + guardian estimator
        z = x + noise * rng.normal(size=(1, episodes))
        xp = xe + ve * dt
        r = z - xp
        xe = xp + alpha * r
        ve = ve + beta * r / dt
        # guardian: one-step envelope check of the proposal (rx_guard_step, hardware cfg)
        h = max(dt, 0.02)
        a = K_ACC * np.sin(u_prop)
        veto = margin_cfg(xe + ve * h + 0.5 * a * h * h, ve + a * h, HW_T_DELAY)[0] <= 0
        vetoes += (veto & ai & ~dead).sum(axis=1) / episodes
        m_now = margin_cfg(xe, ve, HW_T_DELAY)[0]
        back = ~veto & ~ai & (hold == 0) & (m_now >= INNER)
        hold = np.where(veto, hold_ticks, np.where(~ai & (hold > 0), hold - 1, hold))
        newly_safe = veto & ai
        ai = np.where(veto, False, ai | back)
        u = np.where(ai, u_prop, safe_u(xe, ve, sp, HW_T_DELAY))
        for _ in range(substeps):
            hs = dt / substeps
            th += np.clip(u - th, -slew * hs, slew * hs)
            acc = K_ACC * k_scale * np.sin(th) - 0.15 * v
            v += acc * hs
            x += v * hs
        dead |= np.abs(x) > HALF_LEN
        # the pilot sees this tick's estimate and answers for the next tick
        u_prop = policy(w, xe, ve, sp)
        c = (x - sp) ** 2 * 100 + 0.05 * u_prop ** 2 + 0.5 * (~ai) + 40.0 * newly_safe
        c = c + 4.0 * np.clip(0.035 - margin_cfg(x, v, HW_T_DELAY)[0], 0, None) * 100
        cost += np.where(dead, 50.0, c).mean(axis=1)
    if stats:
        return cost / n, vetoes / (secs / 60.0), dead.sum(axis=1)
    return cost / n


def clone_pd(rng, steps=4000, lr=0.02, wn=5.0, zeta=0.85):
    """Fit the MLP to a PD law so CEM starts from something that balances
    (aggressive by default; --hardware starts from a gentler one)."""
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


def export(w, path, score, prefix="rx_pol", guard="RX_POLICY_WEIGHTS_H", note="behaviour-cloned from PD then tuned with CEM"):
    W1, b1, w2, b2 = unpack(w[None, :])
    fl = lambda a: ", ".join(f"{float(q):.7g}f" for q in a)
    P = prefix.upper()
    with open(path, "w") as f:
        f.write("/* GENERATED by tools/train_policy.py - do not edit by hand.\n")
        f.write(f" * 3-{NH}-1 tanh MLP, {note}. eval cost {score:.4f} */\n")
        f.write(f"#ifndef {guard}\n#define {guard}\n")
        f.write(f"#define {P}_IN {NI}\n#define {P}_H {NH}\n")
        f.write(f"static const float {prefix}_w1[{P}_H][{P}_IN] = {{\n")
        for j in range(NH):
            f.write(f"    {{ {fl(W1[0, j])} }},\n")
        f.write("};\n")
        f.write(f"static const float {prefix}_b1[{P}_H] = {{ {fl(b1[0])} }};\n")
        f.write(f"static const float {prefix}_w2[{P}_H] = {{ {fl(w2[0])} }};\n")
        f.write(f"#define {P}_B2 {float(b2[0]):.7g}f\n#endif\n")


def train_hw(a):
    rng = np.random.default_rng(a.seed)
    mean = clone_pd(rng, wn=3.0, zeta=1.0)             # close to the fallback law: starts inside the envelope
    c, vpm, lost = rollout_hw(mean[None, :], np.random.default_rng(1), stats=True)
    print(f"cloned PD      cost {c[0]:.4f}  vetoes/min {vpm[0]:.2f}  lost {lost[0]}")
    sigma = np.full(NPARAM, 0.06)
    for it in range(a.iters):
        pop = mean + sigma * rng.normal(size=(a.pop, NPARAM))
        pop[0] = mean
        cost = rollout_hw(pop, np.random.default_rng(1000 + it))
        elite = pop[np.argsort(cost)[: a.pop // 6]]
        mean = 0.5 * mean + 0.5 * elite.mean(0)
        sigma = 0.7 * sigma + 0.3 * elite.std(0) + 0.002
        if it % 5 == 0 or it == a.iters - 1:
            print(f"iter {it:3d}  best {cost.min():.4f}  mean-of-elite {np.sort(cost)[: a.pop // 6].mean():.4f}")
    c, vpm, lost = rollout_hw(mean[None, :], np.random.default_rng(1), episodes=64, secs=60.0, stats=True)
    print(f"final          cost {c[0]:.4f}  vetoes/min {vpm[0]:.2f}  lost {lost[0]} (64 episodes x 60 s, held-out seed)")
    out = a.out or "src/core/policy_weights_hw.h"
    export(mean, out, c[0], prefix="rx_polhw", guard="RX_POLICY_WEIGHTS_HW_H",
           note=f"hardware config at {HW_HZ:.0f} Hz with the guardian in the loop; vetoes/min {vpm[0]:.2f}")
    print("wrote", out)


# ======================================================================= robot joint (--plant joint)
# must match rx_cfg_joint() in src/core/rx_core.c
J_I, J_MGL, J_B, J_TMAX, J_SLEW = 0.20, 1.5, 0.05, 4.0, 200.0
J_LIM, J_EDGE, J_TD, J_ETA, J_WN, J_ZETA, J_EST_TAU = 1.2, 0.05, 0.05, 0.7, 5.0, 1.0, 0.01
J_HOLD, J_INNER, J_SPLIM = 0.5, 0.1, 0.8
J_SE, J_SV, J_SX = 0.3, 3.0, 1.0                     # input scales, as NETS[RX_NET_JOINT]


def policy_joint(w, x, v, sp):
    z = np.zeros_like(x)
    o = net_raw(w, (sp - x) / J_SE, v / J_SV, x / J_SX) - net_raw(w, z, z, x / J_SX)
    return np.clip(J_MGL * np.sin(x) + J_TMAX * np.tanh(o), -J_TMAX, J_TMAX)


def joint_margin(x, v):
    a_max = (J_TMAX + J_MGL) / J_I
    a_brk = J_ETA * (J_TMAX - J_MGL) / J_I

    def side(s):
        xs, vs = s * x, s * v
        xd = xs + vs * J_TD + 0.5 * a_max * J_TD ** 2
        vd = vs + a_max * J_TD
        reach = np.maximum(xs, xd)
        reach = np.where(vd > 0, np.maximum(reach, xd + vd ** 2 / (2 * a_brk)), reach)
        return (J_LIM - J_EDGE) - reach

    r, l = side(1.0), side(-1.0)
    return np.minimum(r, l), r, l


def joint_safe_u(x, v, sp):
    _, r, l = joint_margin(x, v)
    tau = np.clip(J_I * (J_WN * J_WN * (sp - x) - 2 * J_ZETA * J_WN * v) + J_MGL * np.sin(x), -J_TMAX, J_TMAX)
    near = (r < 0.02) | (l < 0.02)
    return np.where(near, np.where(r < l, -J_TMAX, J_TMAX), tau)


def rollout_joint(w, rng, episodes=24, hz=250.0, secs=6.0, stats=False, policy_fn=None):
    """Joint plant with the guardian in the loop (one-step envelope check, hold, fallback)."""
    pol = policy_fn or policy_joint
    P = w.shape[0]
    dt = 1.0 / hz
    n = int(secs * hz)
    sub = max(1, int(round(dt / 0.0005)))
    hold_ticks = int(J_HOLD * hz)
    x = np.tile(rng.uniform(-0.5, 0.5, (1, episodes)), (P, 1))
    v = np.zeros((P, episodes))
    tq = np.zeros((P, episodes))
    i_scale = np.tile(rng.uniform(0.85, 1.15, (1, episodes)), (P, 1))     # domain randomisation
    g_scale = np.tile(rng.uniform(0.85, 1.15, (1, episodes)), (P, 1))
    square = rng.random(episodes) < 0.5
    phase = rng.integers(0, int(3 * hz), episodes)
    switch = rng.integers(int(1.0 * hz), int(2.5 * hz), episodes)
    sp_rand = rng.uniform(-J_SPLIM, J_SPLIM, episodes)
    thf = np.exp(-dt / J_EST_TAU)
    alpha, beta = 1 - thf * thf, (1 - thf) ** 2
    xe, ve = x.copy(), np.zeros_like(x)
    u_prop = np.zeros_like(x)
    ai = np.zeros((P, episodes), bool)
    hold = np.full((P, episodes), hold_ticks)
    dead = np.zeros((P, episodes), bool)
    cost = np.zeros(P); vetoes = np.zeros(P)
    h = max(dt, 0.02)
    for t in range(n):
        flip = (t % switch) == (switch - 1)
        sp_rand = np.where(flip, rng.uniform(-J_SPLIM, J_SPLIM, episodes), sp_rand)
        sq = np.where(((t + phase) // int(3 * hz)) % 2 == 0, 0.6, -0.6)
        sp = np.broadcast_to(np.where(square, sq, sp_rand)[None, :], x.shape)
        z = x + 0.0015 * rng.normal(size=(1, episodes))
        xp = xe + ve * dt
        r = z - xp
        xe = xp + alpha * r
        ve = ve + beta * r / dt
        a = (u_prop - J_MGL * np.sin(xe) - J_B * ve) / J_I
        veto = joint_margin(xe + ve * h + 0.5 * a * h * h, ve + a * h)[0] <= 0
        vetoes += (veto & ai & ~dead).sum(axis=1) / episodes
        m_now = joint_margin(xe, ve)[0]
        back = ~veto & ~ai & (hold == 0) & (m_now >= J_INNER)
        hold = np.where(veto, hold_ticks, np.where(~ai & (hold > 0), hold - 1, hold))
        newly_safe = veto & ai
        ai = np.where(veto, False, ai | back)
        u = np.where(ai, u_prop, joint_safe_u(xe, ve, sp))
        for _ in range(sub):
            hs = dt / sub
            tq += np.clip(u - tq, -J_SLEW * hs, J_SLEW * hs)
            acc = (tq - J_MGL * g_scale * np.sin(x) - J_B * v) / (J_I * i_scale)
            v += acc * hs
            x += v * hs
        dead |= np.abs(x) > J_LIM
        u_prop = pol(w, xe, ve, sp)
        c = ((x - sp) / 0.3) ** 2 + 0.01 * (u_prop / J_TMAX) ** 2 + 0.5 * (~ai) + 40.0 * newly_safe
        c = c + 40.0 * np.clip(0.1 - joint_margin(x, v)[0], 0, None)
        cost += np.where(dead, 50.0, c).mean(axis=1)
    if stats:
        return cost / n, vetoes / (secs / 60.0), dead.sum(axis=1)
    return cost / n


def clone_joint(rng, steps=4000, lr=0.02, wn=7.0, zeta=0.9):
    """Fit the network's raw output to a gravity-compensated PD law, so CEM starts from something that works."""
    w = rng.normal(0, 0.3, (1, NPARAM))
    m = np.zeros_like(w); s = np.zeros_like(w)
    for it in range(1, steps + 1):
        x = rng.uniform(-1.0, 1.0, 512); v = rng.uniform(-3, 3, 512); sp = rng.uniform(-J_SPLIM, J_SPLIM, 512)
        resid = J_I * (wn * wn * (sp - x) - 2 * zeta * wn * v)              # torque on top of gravity compensation
        tgt = np.arctanh(np.clip(resid / J_TMAX, -0.995, 0.995))
        W1, b1, w2, b2 = unpack(w)
        f = np.stack([(sp - x) / J_SE, v / J_SV, x / J_SX], axis=-1)
        hpre = f @ W1[0].T + b1[0]; hh = np.tanh(hpre)
        o = hh @ w2[0] + b2[0]
        do = 2 * (o - tgt) / len(x)
        gw2 = hh.T @ do; gb2 = do.sum()
        dh = np.outer(do, w2[0]) * (1 - hh ** 2)
        gW1 = dh.T @ f; gb1 = dh.sum(0)
        g = np.concatenate([gW1.ravel(), gb1, gw2, [gb2]])[None, :] * 1e2
        m = 0.9 * m + 0.1 * g; s = 0.999 * s + 0.001 * g * g
        w -= lr * (m / (1 - 0.9 ** it)) / (np.sqrt(s / (1 - 0.999 ** it)) + 1e-8)
    return w[0]


def train_joint(a):
    rng = np.random.default_rng(a.seed)
    mean = clone_joint(rng)
    c, vpm, lost = rollout_joint(mean[None, :], np.random.default_rng(1), stats=True)
    print(f"cloned PD      cost {c[0]:.4f}  vetoes/min {vpm[0]:.2f}  lost {lost[0]}")
    sigma = np.full(NPARAM, 0.06)
    for it in range(a.iters):
        pop = mean + sigma * rng.normal(size=(a.pop, NPARAM))
        pop[0] = mean
        cost = rollout_joint(pop, np.random.default_rng(1000 + it))
        elite = pop[np.argsort(cost)[: a.pop // 6]]
        mean = 0.5 * mean + 0.5 * elite.mean(0)
        sigma = 0.7 * sigma + 0.3 * elite.std(0) + 0.002
        if it % 5 == 0 or it == a.iters - 1:
            print(f"iter {it:3d}  best {cost.min():.4f}  mean-of-elite {np.sort(cost)[: a.pop // 6].mean():.4f}")
    c, vpm, lost = rollout_joint(mean[None, :], np.random.default_rng(1), episodes=64, secs=30.0, stats=True)
    cf, vf, lf = rollout_joint(mean[None, :], np.random.default_rng(1), episodes=64, secs=30.0, stats=True,
                               policy_fn=lambda w, x, v, sp: joint_safe_u(x, v, sp))
    print(f"final          cost {c[0]:.4f}  vetoes/min {vpm[0]:.2f}  lost {lost[0]} (64 episodes x 30 s, held-out seed)")
    print(f"fallback law   cost {cf[0]:.4f}  vetoes/min {vf[0]:.2f}  lost {lf[0]} (same episodes, flown as the pilot)")
    out = a.out or "src/core/policy_weights_joint.h"
    export(mean, out, c[0], prefix="rx_poljt", guard="RX_POLICY_WEIGHTS_JOINT_H",
           note=f"robot joint (rx_cfg_joint) with the guardian in the loop; vetoes/min {vpm[0]:.2f}")
    print("wrote", out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--iters", type=int, default=40)
    ap.add_argument("--pop", type=int, default=48)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--out", default=None)
    ap.add_argument("--hardware", action="store_true", help="train for the serial backend (50 Hz, rx_cfg_hardware)")
    ap.add_argument("--plant", choices=("beam", "joint"), default="beam", help="joint: the robot-joint plant (rx_cfg_joint)")
    a = ap.parse_args()
    if a.plant == "joint":
        return train_joint(a)
    if a.hardware:
        return train_hw(a)
    a.out = a.out or "src/core/policy_weights.h"
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
