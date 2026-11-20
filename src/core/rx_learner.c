/* See rx_learner.h. SPDX-License-Identifier: MIT */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "rx_learner.h"

#define L_MAGIC    0x524c5231u     /* free-form network */
#define L_MAGIC_EQ 0x524c5232u     /* equilibrium-corrected network */
/* Learning rate. 0.02 failed to learn within 15 simulated minutes from an untrained network on 11 of 48
 * random seeds (rx_simtest's learning test, results/2026-11-20-host-bench/learner-seeds-*.txt);
 * 0.0075 failed on none, with a lower median and worst best cost. */
#ifndef LR0
#define LR0 0.0075f
#endif

static float frand(rx_learner_t *L)
{
    L->rng ^= L->rng << 13; L->rng ^= L->rng >> 7; L->rng ^= L->rng << 17;
    return (float)((L->rng >> 11) / 9007199254740992.0);
}
static float nrand(rx_learner_t *L)
{
    float s = 0;
    for (int i = 0; i < 6; i++) s += frand(L);
    return (s - 3.0f) * 1.41421f;                 /* ~N(0,1) */
}

/* raw network output for the three normalised inputs */
static float net_raw(const float *w, float e, float vn, float xn)
{
    const float in[RX_L_IN] = { e, vn, xn };
    const float *W1 = w, *b1 = w + RX_L_H * RX_L_IN, *w2 = b1 + RX_L_H;
    float o = w2[RX_L_H];
    for (int j = 0; j < RX_L_H; j++) {
        float s = b1[j];
        for (int i = 0; i < RX_L_IN; i++) s += W1[j * RX_L_IN + i] * in[i];
        o += w2[j] * tanhf(s);
    }
    return o;
}

/* eq = 1: the same equilibrium-corrected form as rx_policy_with() (zero command with the ball at rest on the
 * setpoint), used when learning starts from the frozen weights, which are trained in that form.
 * eq = 0: the free form, used when learning from scratch. Measured with rx_simtest (15 simulated minutes
 * from an untrained network): the free form reaches a lower best cost, the corrected one starts better. */
static float forward(const float *w, int eq, const rx_cfg_t *c, float x, float v, float sp)
{
    float o = net_raw(w, (sp - x) / 0.1f, v / 0.3f, x / 0.2f);
    if (eq) o -= net_raw(w, 0.0f, 0.0f, x / 0.2f);
    return c->theta_max * tanhf(o);
}

static void new_direction(rx_learner_t *L)
{
    for (int i = 0; i < RX_L_NP; i++) {
        L->eps[i] = nrand(L);
        L->cur[i] = L->mean[i] + L->sigma * L->eps[i];
    }
    L->phase = 0;
}

void rx_learner_init(rx_learner_t *L, uint64_t seed, const rx_net_t *pretrained)
{
    memset(L, 0, sizeof *L);
    L->rng = seed ? seed : 0x243F6A8885A308D3ull;
    L->sigma = 0.12f; L->lr = LR0;
    L->cost_avg = -1.0f; L->best_cost = -1.0f;
    float *W1 = L->mean, *b1 = W1 + RX_L_H * RX_L_IN, *w2 = b1 + RX_L_H;
    if (pretrained && RX_NET_H == RX_L_H && RX_NET_IN == RX_L_IN) {
        for (int j = 0; j < RX_L_H; j++) {
            for (int i = 0; i < RX_L_IN; i++) W1[j * RX_L_IN + i] = pretrained->w1[j][i];
            b1[j] = pretrained->b1[j]; w2[j] = pretrained->w2[j];
        }
        w2[RX_L_H] = pretrained->b2;
        L->sigma = 0.04f; L->lr = LR0 * 0.4f;
        L->eq_corrected = 1;
    } else {
        /* from scratch: random hidden layer, silent output layer -> the network starts by doing nothing */
        for (int i = 0; i < RX_L_H * RX_L_IN + RX_L_H; i++) L->mean[i] = 0.6f * nrand(L);
    }
    new_direction(L);
}

float rx_learner_act(rx_learner_t *L, const rx_cfg_t *c, const rx_state_t *st)
{
    if (!st->train) {                             /* exploration switched off: fly the best weights found */
        L->have_sp = 0; L->acc = 0; L->n = 0; L->safe_ticks = 0;
        if (L->phase != 0) new_direction(L);      /* drop the half-finished pair */
        return forward(L->best_cost >= 0 ? L->best : L->mean, L->eq_corrected, c, st->x, st->v, st->sp);
    }
    /* an episode is one setpoint segment: it ends when the setpoint jumps */
    if (L->have_sp && st->sp != L->last_sp && L->n > 20) {
        float cost = (float)(L->acc / L->n);
        L->episode++;
        L->cost_last = cost;
        L->safe_ticks_last = L->safe_ticks;
        if (L->phase == 0) {
            L->cost_plus = cost;
            for (int i = 0; i < RX_L_NP; i++) L->cur[i] = L->mean[i] - L->sigma * L->eps[i];
            L->phase = 1;
        } else {
            /* Two-point gradient estimate along eps. The step is proportional to the measured
             * cost difference, so it shrinks by itself as the two candidates become equally good. */
            float d = L->cost_plus - cost, pair = 0.5f * (L->cost_plus + cost);
            if (d > 1.0f) d = 1.0f;
            if (d < -1.0f) d = -1.0f;
            L->cost_avg = L->cost_avg < 0 ? pair : 0.85f * L->cost_avg + 0.15f * pair;
            for (int i = 0; i < RX_L_NP; i++) {
                L->mom[i] = 0.5f * L->mom[i] + 0.5f * d * L->eps[i];
                L->mean[i] -= L->lr * L->mom[i] / L->sigma;
            }
            /* elitism: remember the best weights seen and fall back to them if learning diverges */
            if (L->best_cost < 0 || L->cost_avg < L->best_cost) {
                L->best_cost = L->cost_avg; memcpy(L->best, L->mean, sizeof L->best); L->worse = 0;
            } else if (L->cost_avg > 1.6f * L->best_cost + 0.05f && ++L->worse >= 6) {
                memcpy(L->mean, L->best, sizeof L->best); memset(L->mom, 0, sizeof L->mom);
                L->cost_avg = L->best_cost * 1.2f; L->worse = 0;
            }
            /* explore less as the policy gets good */
            float target = 0.03f + 0.09f * fminf(1.0f, L->cost_avg / 1.5f);
            L->sigma = 0.9f * L->sigma + 0.1f * target;
            new_direction(L);
        }
        L->acc = 0; L->n = 0; L->safe_ticks = 0;
    }
    L->last_sp = st->sp; L->have_sp = 1;

    /* cost: squared tracking error (in units of 10 cm) plus a heavy price for needing the guardian */
    float e = (st->sp - st->x) / 0.1f, ce = e * e;
    if (ce > 4.0f) ce = 4.0f;
    int safe = st->mode != RX_MODE_AI;
    L->acc += ce + (safe ? 2.0 : 0.0);
    L->safe_ticks += (uint32_t)safe;
    L->n++;

    return forward(L->cur, L->eq_corrected, c, st->x, st->v, st->sp);
}

int rx_learner_save(const rx_learner_t *L, const char *path)
{
    char tmp[512];
    snprintf(tmp, sizeof tmp, "%s.tmp", path);
    FILE *f = fopen(tmp, "wb");
    if (!f) return -1;
    uint32_t hdr[3] = { L->eq_corrected ? L_MAGIC_EQ : L_MAGIC, RX_L_NP, L->episode };
    int ok = fwrite(hdr, sizeof hdr, 1, f) == 1 && fwrite(L->mean, sizeof L->mean, 1, f) == 1
          && fwrite(&L->sigma, sizeof L->sigma, 1, f) == 1 && fwrite(&L->cost_avg, sizeof L->cost_avg, 1, f) == 1;
    fclose(f);
    return ok && rename(tmp, path) == 0 ? 0 : -1;          /* atomic: a crash never leaves half a file */
}

int rx_learner_load(rx_learner_t *L, const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) return -1;
    uint32_t hdr[3]; float mean[RX_L_NP], sigma, avg;
    int ok = fread(hdr, sizeof hdr, 1, f) == 1 && (hdr[0] == L_MAGIC || hdr[0] == L_MAGIC_EQ) && hdr[1] == RX_L_NP
          && fread(mean, sizeof mean, 1, f) == 1 && fread(&sigma, sizeof sigma, 1, f) == 1 && fread(&avg, sizeof avg, 1, f) == 1;
    fclose(f);
    if (!ok) return -1;
    memcpy(L->mean, mean, sizeof mean);
    L->sigma = sigma; L->cost_avg = avg; L->episode = hdr[2]; L->eq_corrected = hdr[0] == L_MAGIC_EQ;
    memset(L->mom, 0, sizeof L->mom);
    new_direction(L);
    return 0;
}
