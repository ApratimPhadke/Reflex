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

/* Margin toward ONE end of the beam (s = +1 right, -1 left).
 * Worst case assumed: for t_delay the actuator pushes the ball toward that end at full authority,
 * after that braking achieves only brake_eta of the ideal deceleration. */
static float margin_side(const rx_cfg_t *c, float x, float v, float s)
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

float rx_margin(const rx_cfg_t *c, float x, float v)
{
    float r = margin_side(c, x, v, 1.0f), l = margin_side(c, x, v, -1.0f);
    return r < l ? r : l;
}

float rx_safe_u(const rx_cfg_t *c, float x, float v, float sp)
{
    float r = margin_side(c, x, v, 1.0f), l = margin_side(c, x, v, -1.0f);
    if (r < 0.01f || l < 0.01f)                             /* close to a limit: push away from it, full authority */
        return (r < l) ? -c->theta_max : c->theta_max;
    float a = c->wn * c->wn * (sp - x) - 2.0f * c->zeta * c->wn * v;
    float sm = sinf(c->theta_max);
    return asinf(clampf(a / c->k_acc, -sm, sm));
}

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
        float a = c->k_acc * sinf(in->u_ai);
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

void rx_plant_step(rx_plant_t *p, const rx_cfg_t *c, float u_cmd, double dt)
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

#include "policy_weights.h"

float rx_policy(const rx_cfg_t *c, float x, float v, float sp)
{
    float in[RX_POL_IN] = { (sp - x) / 0.1f, v / 0.3f, x / 0.2f };
    float o = RX_POL_B2;
    for (int j = 0; j < RX_POL_H; j++) {
        float s = rx_pol_b1[j];
        for (int i = 0; i < RX_POL_IN; i++) s += rx_pol_w1[j][i] * in[i];
        o += rx_pol_w2[j] * tanhf(s);
    }
    return c->theta_max * tanhf(o);
}
