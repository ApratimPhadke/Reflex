/* Reflex safety kernel. See rx_core.h.
 * SPDX-License-Identifier: MIT */
#include <math.h>
#include <string.h>
#include "rx_core.h"

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void rx_cfg_default(rx_cfg_t *c, float hz)
{
    memset(c, 0, sizeof *c);
    c->hz = hz;              c->dt = 1.0f / hz;
    c->half_len = 0.20f;     c->edge_margin = 0.010f;
    c->k_acc = 5.886f;       /* hollow sphere rolling on an incline: (3/5) g */
    c->theta_max = 0.21f;    /* ~12 degrees */
    c->slew = 6.0f;
    c->t_delay = 0.08f;
    c->brake_eta = 0.7f;
    c->wn = 2.5f;            c->zeta = 1.0f;
    c->est_tau = 0.03f;
    c->max_age_s = 0.005f;
    c->hold_s = 0.5f;
    c->inner_margin = 0.03f;
    c->sp_limit = 0.12f;
    c->strike_limit = 3;     c->strike_window_s = 10.0f;
}

void rx_cfg_hardware(rx_cfg_t *c)
{
    c->max_age_s = 2.5f / c->hz;     /* at 50 Hz one period is already 20 ms */
    c->t_delay = 0.12f;              /* sensor integration time + servo response */
    c->est_tau = 0.08f;              /* a time-of-flight sensor is several mm noisy */
}

/* Physical numbers only; the envelope maths, brake_eta, margins and strike logic are the same as for every rig.
 *   theta_max: limited by the servo travel around level (step2-range-2): the smaller side, so it is never
 *              more than the beam can do in either direction.
 *   t_delay:   measurement 1 tick old (20 ms) + median of 3 (20 ms) + estimator (est_tau, 40 ms) + one full
 *              reversal of the beam at the backend's slew limit (reversal_s).
 *   est_tau:   0.04 s: the HC-SR04 is 0.1 mm noisy with the ball still (step3-still), so less smoothing.
 *   sp_limit:  setpoints within +-15 mm: the ball is seen over +-42 mm only. */
void rx_cfg_beam_direct(rx_cfg_t *c, float theta_max, float reversal_s)
{
    c->theta_max = theta_max;
    c->est_tau = 0.04f;
    c->t_delay = 0.02f + 0.02f + 0.04f + reversal_s;
    c->sp_limit = 0.015f;
}

/* One revolute joint lifting a link against gravity, driven by a torque-limited motor, between hard angle
 * stops: a stand-in for an exoskeleton knee or a surgical-arm joint. Simulation only.
 *   x'' = (tau - mgl sin x - b x') / I,   |tau| <= theta_max,   |x| <= half_len
 * Sized so the worst-case push-then-brake travel (about 0.14 rad) leaves the setpoint range recoverable. */
void rx_cfg_joint(rx_cfg_t *c, float hz)
{
    memset(c, 0, sizeof *c);
    c->plant = RX_PLANT_JOINT;
    c->hz = hz;              c->dt = 1.0f / hz;
    c->inertia = 0.20f;      /* kg m^2: e.g. 0.5 kg link with its centre of mass 0.3 m out, plus the rotor */
    c->mgl = 1.5f;           /* N m: gravity torque with the link horizontal */
    c->damping = 0.05f;      /* N m s/rad */
    c->theta_max = 4.0f;     /* N m: motor torque limit (must exceed mgl to hold the link anywhere) */
    c->slew = 200.0f;        /* N m/s: current-loop torque rise; a full reversal takes 2*4/200 = 40 ms */
    c->half_len = 1.2f;      /* rad: hard stops at +-69 deg */
    c->edge_margin = 0.05f;  /* rad: keep-out strip before each stop (~3 deg) */
    c->t_delay = 0.05f;      /* s: torque reversal (40 ms) + one estimator lag */
    c->brake_eta = 0.7f;
    c->wn = 5.0f;            c->zeta = 1.0f;
    c->est_tau = 0.01f;      /* an encoder is far less noisy than a time-of-flight sensor */
    c->max_age_s = 0.005f;
    c->hold_s = 0.5f;
    c->inner_margin = 0.1f;  /* rad */
    c->sp_limit = 0.8f;      /* rad */
    c->strike_limit = 3;     c->strike_window_s = 10.0f;
}

void rx_est_update(rx_est_t *e, const rx_cfg_t *c, float z)
{
    if (!e->init) { e->x = z; e->v = 0; e->init = 1; return; }
    float th = expf(-c->dt / c->est_tau);       /* critically damped g-h filter */
    float alpha = 1.0f - th * th, beta = (1.0f - th) * (1.0f - th);
    float xp = e->x + e->v * c->dt;
    float r = z - xp;
    e->x = xp + alpha * r;
    e->v = e->v + beta * r / c->dt;
}

void rx_est_predict(rx_est_t *e, const rx_cfg_t *c)
{
    if (e->init) e->x += e->v * c->dt;
}

/* ================================================================== ball and beam
 * Margin toward ONE end of the beam (s = +1 right, -1 left).
 * Worst case assumed: for t_delay the actuator pushes the ball toward that end at full authority,
 * after that braking achieves only brake_eta of the ideal deceleration. */
static float beam_margin_side(const rx_cfg_t *c, float x, float v, float s)
{
    float a_max = c->k_acc * sinf(c->theta_max);
    float a_brk = c->brake_eta * a_max;
    float xs = s * x, vs = s * v;                           /* position / velocity toward this end */
    float xd = xs + vs * c->t_delay + 0.5f * a_max * c->t_delay * c->t_delay;
    float vd = vs + a_max * c->t_delay;
    float reach = xd > xs ? xd : xs;                        /* furthest point during the delay */
    if (vd > 0) { float stop = xd + vd * vd / (2.0f * a_brk); if (stop > reach) reach = stop; }
    return (c->half_len - c->edge_margin) - reach;
}

static float beam_safe_u(const rx_cfg_t *c, float x, float v, float sp)
{
    float r = beam_margin_side(c, x, v, 1.0f), l = beam_margin_side(c, x, v, -1.0f);
    if (r < 0.01f || l < 0.01f)                             /* close to a limit: push away from it, full authority */
        return (r < l) ? -c->theta_max : c->theta_max;
    float a = c->wn * c->wn * (sp - x) - 2.0f * c->zeta * c->wn * v;
    float sm = sinf(c->theta_max);
    return asinf(clampf(a / c->k_acc, -sm, sm));
}

/* the guardian's one-step lookahead model: ball acceleration for a commanded beam angle */
static float beam_accel(const rx_cfg_t *c, float x, float v, float u) { (void)x; (void)v; return c->k_acc * sinf(u); }
static float beam_u_eq(const rx_cfg_t *c, float x) { (void)c; (void)x; return 0.0f; }   /* a level beam holds the ball */

static void beam_step(rx_plant_t *p, const rx_cfg_t *c, float u_cmd, double dt)
{
    int n = (int)ceil(dt / 0.0005);
    if (n < 1) n = 1;
    if (n > 20000) n = 20000;
    double h = dt / n;
    double u = isfinite(u_cmd) ? u_cmd : p->theta;   /* a servo fed garbage holds position */
    if (u > c->theta_max) u = c->theta_max;
    if (u < -c->theta_max) u = -c->theta_max;
    for (int i = 0; i < n; i++) {
        double d = u - p->theta, lim = c->slew * h;
        p->theta += d > lim ? lim : (d < -lim ? -lim : d);
        double a = c->k_acc * sin(p->theta) - 0.15 * p->v;   /* small rolling loss */
        p->v += a * h;
        p->x += p->v * h;
    }
}

/* ================================================================== robot joint
 * Same worst case as the beam, with the joint's own accelerations: for t_delay the motor pushes toward the
 * stop at full torque with gravity helping as much as it ever can (|sin x| <= 1), then brakes at full torque
 * against gravity at its worst, and we rely on only brake_eta of that. Viscous friction is ignored, which
 * only makes the bound more conservative (it always opposes motion). */
static float joint_margin_side(const rx_cfg_t *c, float x, float v, float s)
{
    float a_max = (c->theta_max + c->mgl) / c->inertia;
    float a_brk = c->brake_eta * (c->theta_max - c->mgl) / c->inertia;
    float xs = s * x, vs = s * v;
    float xd = xs + vs * c->t_delay + 0.5f * a_max * c->t_delay * c->t_delay;
    float vd = vs + a_max * c->t_delay;
    float reach = xd > xs ? xd : xs;
    if (vd > 0) { float stop = xd + vd * vd / (2.0f * a_brk); if (stop > reach) reach = stop; }
    return (c->half_len - c->edge_margin) - reach;
}

static float joint_safe_u(const rx_cfg_t *c, float x, float v, float sp)
{
    float r = joint_margin_side(c, x, v, 1.0f), l = joint_margin_side(c, x, v, -1.0f);
    if (r < 0.02f || l < 0.02f)                             /* close to a stop: full torque away from it */
        return (r < l) ? -c->theta_max : c->theta_max;
    /* PD on the angle with gravity compensation, saturated at the torque limit */
    float tau = c->inertia * (c->wn * c->wn * (sp - x) - 2.0f * c->zeta * c->wn * v) + c->mgl * sinf(x);
    return clampf(tau, -c->theta_max, c->theta_max);
}

static float joint_accel(const rx_cfg_t *c, float x, float v, float u) { return (u - c->mgl * sinf(x) - c->damping * v) / c->inertia; }
static float joint_u_eq(const rx_cfg_t *c, float x) { return c->mgl * sinf(x); }   /* torque that holds the link at x */

static void joint_step(rx_plant_t *p, const rx_cfg_t *c, float u_cmd, double dt)
{
    int n = (int)ceil(dt / 0.0005);
    if (n < 1) n = 1;
    if (n > 20000) n = 20000;
    double h = dt / n;
    double u = isfinite(u_cmd) ? u_cmd : p->theta;   /* a drive fed garbage holds its torque */
    if (u > c->theta_max) u = c->theta_max;
    if (u < -c->theta_max) u = -c->theta_max;
    const double inv_i = 1.0 / c->inertia, lim = c->slew * h;
    for (int i = 0; i < n; i++) {
        double d = u - p->theta;                     /* p->theta = applied torque, rising at the slew rate */
        p->theta += d > lim ? lim : (d < -lim ? -lim : d);
        double a = (p->theta - c->mgl * sin(p->x) - c->damping * p->v) * inv_i;
        p->v += a * h;
        p->x += p->v * h;
    }
}

/* ================================================================== the plant interface */
static const rx_plant_ops_t PLANTS[2] = {
    { "beam",  "mm",  1000.0f,     beam_accel,  beam_margin_side,  beam_safe_u,  beam_step,  beam_u_eq },
    { "joint", "deg", 57.2957795f, joint_accel, joint_margin_side, joint_safe_u, joint_step, joint_u_eq },
};

const rx_plant_ops_t *rx_plant_ops(int plant) { return &PLANTS[plant == RX_PLANT_JOINT ? 1 : 0]; }

float rx_margin(const rx_cfg_t *c, float x, float v)
{
    const rx_plant_ops_t *P = rx_plant_ops(c->plant);
    float r = P->margin_side(c, x, v, 1.0f), l = P->margin_side(c, x, v, -1.0f);
    return r < l ? r : l;
}

float rx_safe_u(const rx_cfg_t *c, float x, float v, float sp) { return rx_plant_ops(c->plant)->safe_u(c, x, v, sp); }

void rx_plant_step(rx_plant_t *p, const rx_cfg_t *c, float u_cmd, double dt) { rx_plant_ops(c->plant)->step(p, c, u_cmd, dt); }

void rx_guard_init(rx_guard_t *g)
{
    memset(g, 0, sizeof *g);
    g->mode = RX_MODE_SAFE;                       /* the pilot has to earn control */
    g->reason = RX_R_NO_PILOT;
}

void rx_guard_step(rx_guard_t *g, const rx_cfg_t *c, float x, float v, float sp,
                   const rx_guard_in_t *in, rx_guard_out_t *out)
{
    sp = clampf(sp, -c->sp_limit, c->sp_limit);
    out->intervened = 0;
    out->want_restart = 0;
    out->margin = rx_margin(c, x, v);

    if (!in->guard_enabled) {                     /* demonstration only: what happens without Reflex */
        float u = g->u_last;
        if (in->pilot_present && in->have_prop && isfinite(in->u_ai))
            u = clampf(in->u_ai, -c->theta_max, c->theta_max);
        g->mode = RX_MODE_BYPASS; g->reason = RX_R_NONE; g->hold_left = 0;
        g->u_last = out->u_out = u;
        return;
    }
    if (g->mode == RX_MODE_BYPASS) { g->mode = RX_MODE_SAFE; g->hold_left = (uint32_t)(c->hold_s * c->hz); }

    uint32_t max_age = (uint32_t)(c->max_age_s * c->hz + 0.5f);
    if (max_age < 2) max_age = 2;

    /* 1. judge the proposal */
    uint8_t verdict = RX_R_NONE;
    if (g->quarantined)                              verdict = RX_R_QUARANTINE;
    else if (!in->pilot_present)                     verdict = RX_R_NO_PILOT;
    else if (!in->have_prop || in->age_ticks > max_age) verdict = RX_R_STALE;
    else if (!isfinite(in->u_ai))                    verdict = RX_R_NONFINITE;
    else if (fabsf(in->u_ai) > c->theta_max * 1.001f) verdict = RX_R_RANGE;
    else {
        /* one-step lookahead: where would this command put us, and could we still stop from there? */
        float h = c->dt > 0.02f ? c->dt : 0.02f;
        float a = rx_plant_ops(c->plant)->accel(c, x, v, in->u_ai);
        float v1 = v + a * h;
        float x1 = x + v * h + 0.5f * a * h * h;
        if (rx_margin(c, x1, v1) <= 0.0f)            verdict = RX_R_ENVELOPE;
    }

    /* 2. mode logic */
    if (g->strike_age > 0 && --g->strike_age == 0) g->strikes = 0;

    if (verdict != RX_R_NONE) {
        if (g->mode == RX_MODE_AI) {
            out->intervened = 1;
            if (verdict == RX_R_NONFINITE || verdict == RX_R_RANGE || verdict == RX_R_ENVELOPE) {
                g->strikes++;
                g->strike_age = (uint32_t)(c->strike_window_s * c->hz);
                if (g->strikes >= c->strike_limit) { out->want_restart = 1; g->strikes = 0; }
            }
        }
        g->mode = RX_MODE_SAFE;
        g->reason = verdict;
        g->hold_left = (uint32_t)(c->hold_s * c->hz);
    } else if (g->mode == RX_MODE_SAFE) {
        if (g->hold_left > 0) { g->hold_left--; g->reason = RX_R_HOLD; }
        else if (out->margin < c->inner_margin) g->reason = RX_R_HOLD;
        else { g->mode = RX_MODE_AI; g->reason = RX_R_NONE; }
    }

    /* 3. choose the command */
    out->u_out = (g->mode == RX_MODE_AI) ? in->u_ai : rx_safe_u(c, x, v, sp);
    g->u_last = out->u_out;
}

#include "policy_weights.h"
#include "policy_weights_hw.h"
#include "policy_weights_joint.h"

_Static_assert(RX_POL_IN == RX_NET_IN && RX_POL_H == RX_NET_H, "policy_weights.h does not match RX_NET_*");
_Static_assert(RX_POLHW_IN == RX_NET_IN && RX_POLHW_H == RX_NET_H, "policy_weights_hw.h does not match RX_NET_*");
_Static_assert(RX_POLJT_IN == RX_NET_IN && RX_POLJT_H == RX_NET_H, "policy_weights_joint.h does not match RX_NET_*");

/* input scales: beam (sp - x) / 0.1 m, v / 0.3 m/s, x / 0.2 m; joint 0.3 rad, 3 rad/s, 1 rad */
static const rx_net_t NETS[3] = {
    { rx_pol_w1,   rx_pol_b1,   rx_pol_w2,   RX_POL_B2,   0.1f, 0.3f, 0.2f },
    { rx_polhw_w1, rx_polhw_b1, rx_polhw_w2, RX_POLHW_B2, 0.1f, 0.3f, 0.2f },
    { rx_poljt_w1, rx_poljt_b1, rx_poljt_w2, RX_POLJT_B2, 0.3f, 3.0f, 1.0f },
};

const rx_net_t *rx_policy_net(int which)
{
    return &NETS[(which == RX_NET_HW || which == RX_NET_JOINT) ? which : RX_NET_SIM];
}

static float net_raw(const rx_net_t *n, float e, float vn, float xn)
{
    const float in[RX_NET_IN] = { e, vn, xn };
    float o = n->b2;
    for (int j = 0; j < RX_NET_H; j++) {
        float s = n->b1[j];
        for (int i = 0; i < RX_NET_IN; i++) s += n->w1[j][i] * in[i];
        o += n->w2[j] * tanhf(s);
    }
    return o;
}

/* Equilibrium-corrected: u = u_eq(x) + theta_max * tanh(net(e, v, x) - net(0, 0, x)). At rest on the setpoint
 * the command is exactly the holding command (a level beam; the gravity torque of the joint), whatever the
 * weights, so the pilot cannot learn a steady-state bias. rxctl bench found one before this
 * (results/2026-11-20-host-bench/): the frozen beam pilots stopped 13-23 % short. */
float rx_policy_with(const rx_cfg_t *c, const rx_net_t *n, float x, float v, float sp)
{
    float o = net_raw(n, (sp - x) / n->s_e, v / n->s_v, x / n->s_x) - net_raw(n, 0.0f, 0.0f, x / n->s_x);
    float u = rx_plant_ops(c->plant)->u_eq(c, x) + c->theta_max * tanhf(o);
    return clampf(u, -c->theta_max, c->theta_max);
}

float rx_policy(const rx_cfg_t *c, float x, float v, float sp)
{
    return rx_policy_with(c, &NETS[RX_NET_SIM], x, v, sp);
}
