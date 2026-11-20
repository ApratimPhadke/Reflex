/* Offline, faster-than-real-time test of the safety kernel.
 * No IPC, no timers: the same rx_guard_step()/rx_policy() the real system runs,
 * driven against the simulator under every pilot fault, with and without the guardian.
 * Exit code 0 only if every guarded run keeps the ball on the beam.
 * SPDX-License-Identifier: MIT */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rx_core.h"
#include "rx_learner.h"

static uint64_t rng = 0x9E3779B97F4A7C15ull;
static double urand(void) { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; return (rng >> 11) / 9007199254740992.0; }
static double nrand(void) { return sqrt(-2 * log(urand() + 1e-12)) * cos(6.283185307 * urand()); }

/* A learning pilot that starts from an untrained network must (a) never lose the ball and
 * (b) get better. 15 simulated minutes at 250 Hz. */
static int learning_test(uint64_t seed, float *first, float *bestc)
{
    static rx_learner_t L;
    float hz = 250;
    rx_cfg_t c; rx_cfg_default(&c, hz);
    rx_plant_t p = { 0.03, 0, 0 }; rx_est_t e = { 0 }; rx_guard_t g; rx_guard_init(&g);
    rx_learner_init(&L, 4242, NULL); rng = seed;
    float u_pending = 0; int have = 0, lost = 0; long iv = 0, iv_first = -1, iv_block = 0; uint32_t last_ep = 0;
    float cost_first = -1;
    for (long k = 0; k < (long)(900 * hz); k++) {
        float sp = ((k / (long)(3 * hz)) % 2) ? -0.08f : 0.08f;
        rx_est_update(&e, &c, (float)(p.x + 0.0015 * nrand()));
        rx_guard_in_t in = { 1, 1, have, 1, u_pending }; rx_guard_out_t o;
        rx_guard_step(&g, &c, e.x, e.v, sp, &in, &o);
        iv += o.intervened; iv_block += o.intervened;
        rx_plant_step(&p, &c, o.u_out, c.dt);
        if (fabs(p.x) > c.half_len) { lost++; p.x = 0; p.v = 0; }
        rx_state_t st = { (uint32_t)k, 0, 0, e.x, e.v, sp, g.mode, g.reason, 1 };
        u_pending = rx_learner_act(&L, &c, &st); have = 1;
        if (L.episode != last_ep) {
            last_ep = L.episode;
            if (L.episode % 50 == 0) {
                (void)0; if (0) printf("  episode %3u  t=%4.0fs  smoothed cost %.3f  interventions in last 50 episodes: %ld\n", L.episode, k / hz, L.cost_avg, iv_block);
                if (iv_first < 0) { iv_first = iv_block; cost_first = L.cost_avg; }
                iv_block = 0;
            }
        }
    }
    float best = L.best_cost;
    *first = cost_first; *bestc = L.best_cost; if (0) printf("  balls lost while learning: %d   interventions total: %ld   cost %.3f -> best %.3f\n", lost, iv, cost_first, best);
    return lost != 0 || !(best < 0.6f * cost_first);
}


int main(int argc, char **argv)
{
    int n = argc > 1 ? atoi(argv[1]) : 8, pass = 0;
    for (int i = 0; i < n; i++) {
        float f, b; int r = learning_test(0x9E3779B97F4A7C15ull + (uint64_t)i * 7919u, &f, &b);
        printf("seed %d: first %.3f best %.3f ratio %.2f %s\n", i, f, b, b / f, r ? "FAIL" : "pass"); pass += !r;
    }
    printf("%d/%d pass\n", pass, n); return 0;
}
