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

/* min_margin = closest the ball got to either end of the beam after the fault started */
typedef struct { int lost; int interventions; double rms_err, min_margin, first_detect_ms; int restarts; } result_t;

/* hw = 1: the serial backend's view: rx_cfg_hardware(), the hardware-trained network and a sensor with
 * +/-3 mm uniform noise like the emulator firmware (the trainer uses Gaussian noise: this is held out) */
static result_t run_cfg(float hz, int fault, int guard, double secs, int hw, int net_id)
{
    rx_cfg_t c; rx_cfg_default(&c, hz);
    if (hw == 1) rx_cfg_hardware(&c);
    if (hw == 2) rx_cfg_joint(&c, hz);                     /* the robot joint: angles in rad, torque in N m */
    const rx_net_t *net = rx_policy_net(net_id);
    rx_plant_t p = { hw == 2 ? 0.05 : 0.03, 0, 0 };
    const float sp_amp = hw == 2 ? 0.6f : 0.08f;
    rx_est_t e = { 0 };
    rx_guard_t g; rx_guard_init(&g);
    result_t r = { 0, 0, 0, 1e9, -1, 0 };
    int n = (int)(secs * hz), fault_at = (int)(4.0 * hz);
    float u_pending = 0; int have = 0, pilot_alive = 1;
    double se = 0; int sn = 0;
    int slow_ctr = 0;
    for (int k = 0; k < n; k++) {
        float sp = ((k / (int)(4 * hz)) % 2) ? -sp_amp : sp_amp;
        float z = (float)(p.x + (hw == 1 ? 0.003 * (2 * urand() - 1) : 0.0015 * nrand()));
        rx_est_update(&e, &c, z);
        int f = (k >= fault_at) ? fault : RX_F_NONE;

        rx_guard_in_t in = { guard, pilot_alive, have, 1, u_pending };
        if (f == RX_F_SLOW) { in.age_ticks = 1 + (uint32_t)(slow_ctr); }   /* proposal keeps getting older */
        if (f == RX_F_SPIN || f == RX_F_COUNT /* crash */) { in.pilot_present = (f == RX_F_SPIN); in.have_prop = (f == RX_F_SPIN); in.age_ticks = 1000; }
        rx_guard_out_t o;
        rx_guard_step(&g, &c, e.x, e.v, sp, &in, &o);
        if (o.intervened) { r.interventions++; if (r.first_detect_ms < 0 && k >= fault_at) r.first_detect_ms = (k - fault_at) * 1000.0 / hz; }
        if (o.want_restart) r.restarts++;
        rx_plant_step(&p, &c, o.u_out, c.dt);
        if (k >= fault_at && c.half_len - fabs(p.x) < r.min_margin) r.min_margin = c.half_len - fabs(p.x);
        if (fabs(p.x) > c.half_len) { r.lost = 1; break; }
        if (k < fault_at && k > (int)hz) { se += (p.x - sp) * (p.x - sp); sn++; }

        /* the pilot computes the next proposal from this tick's estimate */
        float u = rx_policy_with(&c, net, e.x, e.v, sp);
        switch (f) {
        case RX_F_NAN:   u = NAN; break;
        case RX_F_FLIP:  u = -u; break;
        case RX_F_STUCK: u = c.theta_max; break;
        case RX_F_HUGE:  u = 40.0f; break;
        case RX_F_SLOW:  slow_ctr++; if (slow_ctr % (int)(0.05f * hz + 1) != 0) u = u_pending; else slow_ctr = 0; break;
        default: break;
        }
        u_pending = u; have = 1;
    }
    r.rms_err = sn ? sqrt(se / sn) : 0;
    return r;
}

static result_t run(float hz, int fault, int guard, double secs) { return run_cfg(hz, fault, guard, secs, 0, RX_NET_SIM); }

/* Monte Carlo check of the recoverable-set claim: pick random states the kernel calls recoverable,
 * let an adversary command full tilt in the worst direction for one more tick, then hand over to
 * the fallback controller. The ball must never enter the keep-out strip. Plant parameters are
 * perturbed to show how much model error the margins absorb. */
static int envelope_mc(float hz, double k_scale, double slew_scale, long samples, long *tested)
{
    rx_cfg_t c; rx_cfg_default(&c, hz);
    rx_cfg_t world = c; world.k_acc *= (float)k_scale; world.slew *= (float)slew_scale;
    long viol = 0, n = 0;
    for (long i = 0; i < samples; i++) {
        rx_plant_t p = { (urand() * 2 - 1) * c.half_len, (urand() * 2 - 1) * 0.6, (urand() * 2 - 1) * c.theta_max };
        if (rx_margin(&c, (float)p.x, (float)p.v) <= 0) continue;
        n++;
        float s = p.v > 0 ? 1.0f : (p.v < 0 ? -1.0f : (p.x >= 0 ? 1.0f : -1.0f));
        float sp = (float)((urand() * 2 - 1) * c.sp_limit);
        rx_plant_step(&p, &world, s * c.theta_max, c.dt);                    /* the adversary's last tick */
        int bad = 0;
        for (int k = 0; k < (int)(3 * hz) && !bad; k++) {
            rx_plant_step(&p, &world, rx_safe_u(&c, (float)p.x, (float)p.v, sp), c.dt);
            if (fabs(p.x) > c.half_len - c.edge_margin) bad = 1;
        }
        viol += bad;
    }
    *tested = n;
    return (int)viol;
}

/* The same Monte Carlo check for the robot joint: random states the kernel calls recoverable, one more tick
 * of full torque in the worst direction, then the fallback only, on a joint whose inertia, gravity load and
 * torque slew differ from the model. Must never enter the keep-out strip before either stop. */
static int envelope_mc_joint(float hz, double i_scale, double g_scale, double slew_scale, long samples, long *tested)
{
    rx_cfg_t c; rx_cfg_joint(&c, hz);
    rx_cfg_t world = c; world.inertia *= (float)i_scale; world.mgl *= (float)g_scale; world.slew *= (float)slew_scale;
    long viol = 0, n = 0;
    for (long i = 0; i < samples; i++) {
        rx_plant_t p = { (urand() * 2 - 1) * c.half_len, (urand() * 2 - 1) * 4.0, (urand() * 2 - 1) * c.theta_max };
        if (rx_margin(&c, (float)p.x, (float)p.v) <= 0) continue;
        n++;
        float s = p.v > 0 ? 1.0f : (p.v < 0 ? -1.0f : (p.x >= 0 ? 1.0f : -1.0f));
        float sp = (float)((urand() * 2 - 1) * c.sp_limit);
        rx_plant_step(&p, &world, s * c.theta_max, c.dt);                    /* the adversary's last tick */
        int bad = 0;
        for (int k = 0; k < (int)(3 * hz) && !bad; k++) {
            rx_plant_step(&p, &world, rx_safe_u(&c, (float)p.x, (float)p.v, sp), c.dt);
            if (fabs(p.x) > c.half_len - c.edge_margin) bad = 1;
        }
        viol += bad;
    }
    *tested = n;
    return (int)viol;
}

/* A learning pilot that starts from an untrained network must (a) never lose the ball and
 * (b) get better. 15 simulated minutes at 250 Hz. */
static int learning_test(void)
{
    static rx_learner_t L;
    float hz = 250;
    rx_cfg_t c; rx_cfg_default(&c, hz);
    rx_plant_t p = { 0.03, 0, 0 }; rx_est_t e = { 0 }; rx_guard_t g; rx_guard_init(&g);
    rx_learner_init(&L, 4242, NULL);
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
                printf("  episode %3u  t=%4.0fs  smoothed cost %.3f  interventions in last 50 episodes: %ld\n", L.episode, k / hz, L.cost_avg, iv_block);
                if (iv_first < 0) { iv_first = iv_block; cost_first = L.cost_avg; }
                iv_block = 0;
            }
        }
    }
    float best = L.best_cost;
    printf("  balls lost while learning: %d   interventions total: %ld   cost %.3f -> best %.3f\n", lost, iv, cost_first, best);
    return lost != 0 || !(best < 0.6f * cost_first);
}

int main(void)
{
    static const char *names[] = { "none", "nan", "flip", "stuck", "huge", "slow", "spin(hang)", "crash" };
    float rates[] = { 1000.0f, 250.0f, 50.0f };
    int fail = 0;
    for (unsigned ri = 0; ri < 3; ri++) {
        float hz = rates[ri];
        printf("\n== control rate %.0f Hz ==\n", hz);
        printf("%-12s | %-28s | %-22s\n", "pilot fault", "WITH guardian", "WITHOUT guardian");
        printf("%-12s | %-6s %-7s %-6s %-6s | %-6s\n", "", "ball", "detect", "edge", "interv", "ball");
        for (int f = 0; f <= RX_F_COUNT; f++) {
            result_t a = run(hz, f, 1, 20.0), b = run(hz, f, 0, 20.0);
            char det[24] = "-", mar[24] = "-";
            if (a.first_detect_ms >= 0) snprintf(det, sizeof det, "%.0fms", a.first_detect_ms);
            if (f) snprintf(mar, sizeof mar, "%.1fcm", a.min_margin * 100);
            printf("%-12s | %-6s %-7s %-6s %-6d | %-6s\n", names[f], a.lost ? "LOST" : "kept", det, mar,
                   a.interventions, b.lost ? "LOST" : "kept");
            if (a.lost || (f && a.min_margin < 0.010)) fail = 1;   /* must also respect the 1 cm keep-out strip */
            if (f == 0) {
                printf("%-12s   nominal tracking RMS error %.1f mm, interventions %d\n", "", a.rms_err * 1000, a.interventions);
                if (a.interventions > 1) { printf("FAIL: guardian interferes with a healthy pilot\n"); fail = 1; }
            }
        }
    }
    printf("\n== recoverable-set Monte Carlo (adversary gets one more tick, then fallback only) ==\n");
    struct { double k, slew; const char *what; int must_pass; } mc[] = {
        { 1.00, 1.00, "nominal plant", 1 }, { 1.15, 1.00, "ball 15% livelier than modelled", 1 },
        { 0.85, 1.00, "ball 15% more sluggish", 1 }, { 1.00, 0.80, "servo 20% slower than modelled", 1 },
        { 1.15, 0.80, "livelier ball AND slower servo", 0 } };
    for (unsigned i = 0; i < sizeof mc / sizeof mc[0]; i++) {
        long n; int v = envelope_mc(250.0f, mc[i].k, mc[i].slew, 200000, &n);
        printf("  %-34s %ld recoverable states tested, %d violations%s\n", mc[i].what, n, v, mc[i].must_pass ? "" : "  (informational)");
        if (v && mc[i].must_pass) fail = 1;
    }
    printf("\n== online learning from an untrained network, guardian in the loop ==\n");
    if (learning_test()) { printf("FAIL: learner lost the ball or did not improve\n"); fail = 1; }

    /* The serial backend runs rx_cfg_hardware() at 50 Hz. A pilot trained for the simulator is vetoed at
     * nearly every setpoint step there and strikes out (seen on the Pi 4, results/2026-11-20-pi4-emulated-io/).
     * The guardian is right to veto it; the hardware-trained pilot must not need it. */
    printf("\n== serial backend config: 50 Hz, rx_cfg_hardware(), +/-3 mm sensor, hardware-trained pilot ==\n");
    printf("%-12s | %-28s | %-22s\n", "pilot fault", "WITH guardian", "WITHOUT guardian");
    printf("%-12s | %-6s %-7s %-6s %-6s | %-6s\n", "", "ball", "detect", "edge", "interv", "ball");
    for (int f = 1; f <= RX_F_COUNT; f++) {
        result_t a = run_cfg(50.0f, f, 1, 20.0, 1, RX_NET_HW), b = run_cfg(50.0f, f, 0, 20.0, 1, RX_NET_HW);
        char det[24] = "-";
        if (a.first_detect_ms >= 0) snprintf(det, sizeof det, "%.0fms", a.first_detect_ms);
        printf("%-12s | %-6s %-7s %-5.1fcm %-6d | %-6s\n", names[f], a.lost ? "LOST" : "kept", det, a.min_margin * 100,
               a.interventions, b.lost ? "LOST" : "kept");
        if (a.lost || a.min_margin < 0.010) fail = 1;
    }
    {
        result_t hwp = run_cfg(50.0f, RX_F_NONE, 1, 120.0, 1, RX_NET_HW);
        result_t sim = run_cfg(50.0f, RX_F_NONE, 1, 120.0, 1, RX_NET_SIM);
        printf("  healthy, 120 s of +/-80 mm steps:  hardware pilot %d interventions, %d strike-outs, RMS %.1f mm"
               "   |   simulator pilot (informational) %d interventions, %d strike-outs\n",
               hwp.interventions, hwp.restarts, hwp.rms_err * 1000, sim.interventions, sim.restarts);
        if (hwp.lost || hwp.restarts > 0 || hwp.interventions > 1) {
            printf("FAIL: guardian has to overrule the healthy hardware pilot\n"); fail = 1;
        }
    }

    /* The guardian is plant-independent: the same kernel, a different plant table (rx_plant_ops). */
    printf("\n== second plant: robot joint with gravity load (rx_cfg_joint), frozen joint pilot ==\n");
    for (unsigned ri = 0; ri < 2; ri++) {
        float hz = ri ? 250.0f : 1000.0f;
        rx_cfg_t jc; rx_cfg_joint(&jc, hz);
        printf("-- %.0f Hz: angle limit %.2f rad, keep-out %.2f rad, torque limit %.1f N m --\n", hz, jc.half_len, jc.edge_margin, jc.theta_max);
        printf("%-12s | %-30s | %-22s\n", "pilot fault", "WITH guardian", "WITHOUT guardian");
        printf("%-12s | %-6s %-7s %-8s %-6s | %-6s\n", "", "joint", "detect", "to stop", "interv", "joint");
        for (int f = 0; f <= RX_F_COUNT; f++) {
            result_t a = run_cfg(hz, f, 1, 20.0, 2, RX_NET_JOINT), b = run_cfg(hz, f, 0, 20.0, 2, RX_NET_JOINT);
            char det[24] = "-", mar[24] = "-";
            if (a.first_detect_ms >= 0) snprintf(det, sizeof det, "%.0fms", a.first_detect_ms);
            if (f) snprintf(mar, sizeof mar, "%.1fdeg", a.min_margin * 57.29578);
            printf("%-12s | %-6s %-7s %-8s %-6d | %-6s\n", names[f], a.lost ? "HIT" : "kept", det, mar,
                   a.interventions, b.lost ? "HIT" : "kept");
            if (a.lost || (f && a.min_margin < jc.edge_margin)) fail = 1;   /* never into the keep-out strip */
            if (f == 0) {
                printf("%-12s   nominal tracking RMS error %.1f deg, interventions %d, strike-outs %d\n", "",
                       a.rms_err * 57.29578, a.interventions, a.restarts);
                if (a.restarts > 0) { printf("FAIL: the healthy joint pilot struck out\n"); fail = 1; }
            }
        }
    }
    printf("\n== robot joint: recoverable-set Monte Carlo (adversary gets one more tick, then fallback only) ==\n");
    struct { double i, g, slew; const char *what; int must_pass; } jm[] = {
        { 1.00, 1.00, 1.00, "nominal joint", 1 }, { 0.85, 1.00, 1.00, "inertia 15% lower than modelled", 1 },
        { 1.15, 1.00, 1.00, "inertia 15% higher", 1 }, { 1.00, 1.15, 1.00, "gravity load 15% higher", 1 },
        { 1.00, 1.00, 0.80, "torque rise 20% slower", 1 }, { 0.85, 1.15, 0.80, "all three at once", 0 } };
    for (unsigned i = 0; i < sizeof jm / sizeof jm[0]; i++) {
        long n; int v = envelope_mc_joint(1000.0f, jm[i].i, jm[i].g, jm[i].slew, 200000, &n);
        printf("  %-34s %ld recoverable states tested, %d violations%s\n", jm[i].what, n, v, jm[i].must_pass ? "" : "  (informational)");
        if (v && jm[i].must_pass) fail = 1;
    }

    printf("\n%s\n", fail ? "SIMTEST FAILED" : "SIMTEST PASSED");
    return fail;
}
