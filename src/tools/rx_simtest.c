/* Offline, faster-than-real-time test of the safety kernel.
 * No IPC, no timers: the same rx_guard_step()/rx_policy() the real system runs,
 * driven against the simulator under every pilot fault, with and without the guardian.
 * Exit code 0 only if every guarded run keeps the ball on the beam.
 * SPDX-License-Identifier: MIT */
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rx_core.h"
#include "rx_learner.h"
#include "rx_guest.h"
#include "rx_sonar.h"

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
 * torque slew differ from the model. Must never enter the keep-out strip before either stop.
 * Each case has its own random stream so the cases can run in parallel (they are independent). */
typedef struct { float hz; double i_scale, g_scale, slew_scale; long samples; uint64_t seed; long tested; int viol; } jmc_t;

static void *envelope_mc_joint(void *arg)
{
    jmc_t *J = arg;
    uint64_t r = J->seed;
#define JRAND() (r ^= r << 13, r ^= r >> 7, r ^= r << 17, (double)(r >> 11) / 9007199254740992.0)
    rx_cfg_t c; rx_cfg_joint(&c, J->hz);
    rx_cfg_t world = c; world.inertia *= (float)J->i_scale; world.mgl *= (float)J->g_scale; world.slew *= (float)J->slew_scale;
    long viol = 0, n = 0;
    for (long i = 0; i < J->samples; i++) {
        double x0 = (JRAND() * 2 - 1) * c.half_len, v0 = (JRAND() * 2 - 1) * 4.0, t0 = (JRAND() * 2 - 1) * c.theta_max;
        rx_plant_t p = { x0, v0, t0 };
        if (rx_margin(&c, (float)p.x, (float)p.v) <= 0) continue;
        n++;
        float s = p.v > 0 ? 1.0f : (p.v < 0 ? -1.0f : (p.x >= 0 ? 1.0f : -1.0f));
        float sp = (float)((JRAND() * 2 - 1) * c.sp_limit);
        rx_plant_step(&p, &world, s * c.theta_max, c.dt);                    /* the adversary's last tick */
        int bad = 0;
        for (int k = 0; k < (int)(3 * J->hz) && !bad; k++) {
            rx_plant_step(&p, &world, rx_safe_u(&c, (float)p.x, (float)p.v, sp), c.dt);
            if (fabs(p.x) > c.half_len - c.edge_margin) bad = 1;
        }
        viol += bad;
    }
#undef JRAND
    J->tested = n; J->viol = (int)viol;
    return NULL;
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

/* Guest supervision (pilot_transport = udp): the restart limiter of rx_guest.c in simulated time. */
static int guest_test(void)
{
    int bad = 0, why = 0; unsigned a;
#define GCHECK(c, ...) do { if (!(c)) { printf("  FAIL: " __VA_ARGS__); printf("\n"); bad = 1; } } while (0)
    const double dt = 0.01;
    rx_guest_t g;

    /* healthy, then silent: down after the timeout, restarted at once, then not again for 15 s */
    rx_guest_init(&g, 1.0f, 15.0f, 0.0);
    a = rx_guest_step(&g, 0.5, 1, 0, &why);
    GCHECK(a == RX_GA_UP && g.state == RX_GUEST_UP, "first valid proposal did not bring the guest up (acts %u)", a);
    double t = 0.5, t_down = -1, t_restart = -1;
    unsigned nrest = 0;
    for (t = 0.5 + dt; t < 40.0; t += dt) {
        a = rx_guest_step(&g, t, 0, 0, &why);
        if ((a & RX_GA_DOWN) && t_down < 0) t_down = t;
        if (a & RX_GA_RESTART) { if (t_restart < 0) { t_restart = t; GCHECK(why == RX_GR_TIMEOUT, "restart reason %d, want timeout", why); } nrest++; }
    }
    GCHECK(t_down > 1.5 - 1e-6 && t_down < 1.5 + 2 * dt, "down at %.2f s, want 1.5 s (1 s after the last proposal)", t_down);
    GCHECK(fabs(t_restart - t_down) < 1e-9, "first restart at %.2f s, not at the down transition %.2f s", t_restart, t_down);
    GCHECK(nrest == 3, "%u restarts in 40 s of silence, want 3 (at 1.5, 16.5, 31.5 s)", nrest);
    printf("  silent guest: down at %.2f s, restarts in the first 40 s: %u (15 s apart)\n", t_down, nrest);

    /* never comes back: 5 restarts, then quarantine, then nothing for an hour */
    rx_guest_init(&g, 1.0f, 15.0f, 0.0);
    rx_guest_step(&g, 0.1, 1, 0, &why);
    nrest = 0; double t_q = -1, t_first = -1;
    for (t = 0.1 + dt; t < 3600.0; t += dt) {
        a = rx_guest_step(&g, t, 0, 0, &why);
        if ((a & RX_GA_RESTART) && !nrest++) t_first = t;
        if (a & RX_GA_QUARANTINE) { GCHECK(t_q < 0, "quarantined twice"); t_q = t; }
    }
    GCHECK(nrest == RX_GUEST_MAX_RESTARTS && g.state == RX_GUEST_QUARANTINE, "dead guest: %u restarts, state %u", nrest, g.state);
    GCHECK(fabs(t_q - t_first - 5 * RX_GUEST_MIN_GAP_S) < 6 * dt,   /* each restart lands within a tick of its slot */ "quarantined %.2f s after the first restart, want at the 6th slot (75 s)", t_q - t_first);
    printf("  dead guest: %u restarts, quarantined at %.1f s, no restart in the hour after\n", nrest, t_q);
    rx_guest_release(&g, 3600.0);
    a = rx_guest_step(&g, 3600.0 + dt, 0, 0, &why);
    GCHECK((a & RX_GA_RESTART) && g.state == RX_GUEST_DOWN, "after release a down guest was not restarted at once (acts %u)", a);
    a = rx_guest_step(&g, 3601.0, 1, 0, &why);
    GCHECK((a & RX_GA_UP) && g.state == RX_GUEST_UP, "released guest did not come up");

    /* a guest that boots slowly: guest_boot_s = 40 spaces the restarts 40 s apart */
    rx_guest_init(&g, 1.0f, 40.0f, 0.0);
    nrest = 0; double last = -1, mingap = 1e9;
    for (t = dt; t < 300.0; t += dt) {
        a = rx_guest_step(&g, t, 0, 0, &why);
        if (a & RX_GA_RESTART) {
            if (!nrest) GCHECK(why == RX_GR_NEVER_UP && t > 40.0 - 1e-6 && t < 40.0 + 2 * dt, "never-up guest restarted at %.2f s reason %d, want 40 s never-up", t, why);
            if (last >= 0 && t - last < mingap) mingap = t - last;
            last = t; nrest++;
        }
    }
    GCHECK(mingap > 40.0 - 2 * dt, "slow-boot restarts %.2f s apart, want >= 40 s", mingap);

    /* strikes while up: one restart, then the limiter holds the next ones off for 15 s */
    rx_guest_init(&g, 1.0f, 15.0f, 0.0);
    rx_guest_step(&g, 0.1, 1, 0, &why);
    a = rx_guest_step(&g, 5.0, 1, 1, &why);
    GCHECK((a & RX_GA_RESTART) && why == RX_GR_STRIKES, "strike-out did not restart the guest (acts %u, reason %d)", a, why);
    a = rx_guest_step(&g, 10.0, 1, 1, &why);
    GCHECK(!(a & RX_GA_RESTART), "second strike-out 5 s later restarted the guest again");
    a = rx_guest_step(&g, 20.1, 1, 1, &why);
    GCHECK(a & RX_GA_RESTART, "strike-out 15 s later was not allowed");

    /* random weather: guest flaps, strikes, comes and goes for 6 simulated hours. At no point may two restarts
     * be less than 15 s apart or more than 5 fall in any 10-minute window. */
    uint64_t r = 0x5DEECE66Dull; double hist[4096]; int nh = 0, total = 0, quarantines = 0;
    for (int run = 0; run < 20 && !bad; run++) {
        rx_guest_init(&g, 0.2f + 2.0f * (float)urand(), 1.0f + 60.0f * (float)urand(), 0.0);
        int alive = 1; double flip_at = 0; nh = 0;            /* hist: restarts since the last release */
        for (t = dt; t < 6 * 3600.0 && !bad; t += dt) {
            if (t >= flip_at) { alive = !alive; r = r * 6364136223846793005ull + 1; flip_at = t + 0.5 + (double)(r >> 40) / (double)(1ull << 24) * 120.0; }
            int strike = alive && urand() < 0.0005;
            a = rx_guest_step(&g, t, alive, strike, &why);
            if (a & RX_GA_RESTART) {
                GCHECK(!nh || t - hist[nh - 1] >= RX_GUEST_MIN_GAP_S - 1e-9, "run %d: restarts %.2f s apart", run, t - hist[nh - 1]);
                GCHECK(nh < RX_GUEST_MAX_RESTARTS || t - hist[nh - RX_GUEST_MAX_RESTARTS] >= RX_GUEST_WINDOW_S - 1e-9,
                       "run %d: 6 restarts within %.1f s", run, t - hist[nh - RX_GUEST_MAX_RESTARTS]);
                if (nh < 4096) hist[nh++] = t;
                total++;
                alive = 0; flip_at = t + 1.0 + urand() * 30.0;    /* a restart takes the guest down for a while */
            }
            if (a & RX_GA_QUARANTINE) {
                int k = 0; for (int i = 0; i < nh; i++) if (t - hist[i] < RX_GUEST_WINDOW_S) k++;
                GCHECK(k == RX_GUEST_MAX_RESTARTS, "run %d: quarantined with %d restarts in the window, want exactly 5", run, k);
                quarantines++;
                rx_guest_release(&g, t); nh = 0;                     /* an operator lets it back in */
            }
        }
    }
    printf("  random flapping guests, 20 x 6 h: %d restarts, %d quarantines; never < 15 s apart, never > 5 in 10 min\n", total, quarantines);
#undef GCHECK
    return bad;
}

/* Beam-direct rig (asymmetric servo authority): the AI gets theta_max both ways, the fallback alone may use
 * theta_fb_pos toward +x, and the margin assumes the largest command of the last t_delay for the push.
 * The Monte Carlo below is sharp enough to need that: with the push peak ignored it finds 7 / 139 / 79 states
 * that enter the keep-out strip (checked 2026-11-25), with it none.
 * Numbers of the rig (results/2026-11-24-beam-direct): 19.2 / 46.6 mrad, 9.88 us/mrad, 60 us per tick slew,
 * half length 42 mm, 50 Hz. */
static void bd_cfg(rx_cfg_t *c)
{
    rx_cfg_default(c, 50.0f); rx_cfg_hardware(c);
    c->half_len = 0.042f;
    const float th = 0.0192f, fb = 0.0466f, slew_rad_s = 60.0f / 9.88f / 1000.0f / 0.02f;
    rx_cfg_beam_direct(c, th, fb, (fb + th) / slew_rad_s);
    c->slew = slew_rad_s;
}
static float hist_peak(const float *h, int n, int k)
{
    float pk = 0; for (int i = 0; i < k && i < n; i++) if (h[(n - 1 - i) % 64] > pk) pk = h[(n - 1 - i) % 64];
    return pk;
}
static int beam_direct_test(void)
{
    printf("== beam-direct: asymmetric authority (AI +-19.2 mrad, fallback up to +46.6 mrad toward +x) ==\n");
    rx_cfg_t c; bd_cfg(&c);
    int bad = 0, K = (int)(c.t_delay * c.hz) + 2;
    printf("  t_delay %.0f ms, slew %.3f rad/s, theta_max %.1f mrad, theta_fb_pos %.1f mrad\n", c.t_delay * 1e3, c.slew, c.theta_max * 1e3, c.theta_fb_pos * 1e3);
    /* 1. Monte Carlo: recoverable states (with their command history), one adversary tick, then the fallback */
    const double ks[3] = { 1.0, 0.8, 1.2 }, ss[3] = { 1.0, 0.8, 1.0 };
    for (int m = 0; m < 3; m++) {
        rx_cfg_t world = c; world.k_acc *= (float)ks[m]; world.slew *= (float)ss[m];
        long n = 0, viol = 0;
        for (long i = 0; i < 200000; i++) {
            float fbp = c.theta_fb_pos, th = c.theta_max;
            rx_plant_t p = { (urand() * 2 - 1) * c.half_len, (urand() * 2 - 1) * 0.15, -th + urand() * (fbp + th) };
            if (urand() < 0.3) { p.theta = fbp; p.x = urand() * c.half_len; p.v = urand() * 0.15; }   /* the corner: pushing hard toward +x */
            float pk = (float)(p.theta > 0 ? p.theta : 0); if (urand() < 0.3) pk += (float)urand() * (fbp - pk);
            if (rx_margin_applied(&c, (float)p.x, (float)p.v, pk) <= 0) continue;
            n++;
            float h[64]; int hn = 0; for (int j = 0; j < K; j++) h[hn++ % 64] = pk;
            float sd = p.v > 0 ? 1.0f : (p.v < 0 ? -1.0f : (p.x >= 0 ? 1.0f : -1.0f));
            float sp = (float)((urand() * 2 - 1) * c.sp_limit);
            /* the fallback sees the ball 2 ticks late (sensor 1 tick old + median of 3), through the estimator */
            double dx[3] = { p.x, p.x, p.x };
            rx_est_t e = { (float)p.x, (float)p.v, 1 };
            float ua = sd * th; rx_plant_step(&p, &world, ua, c.dt); h[hn++ % 64] = ua;   /* the adversary's last tick */
            int b = 0;
            for (int k = 0; k < (int)(4 * c.hz) && !b; k++) {
                dx[k % 3] = p.x;
                rx_est_update(&e, &c, (float)dx[(k + 1) % 3]);                             /* 2 ticks old */
                float u = rx_safe_u_applied(&c, e.x, e.v, sp, hist_peak(h, hn, K));
                rx_plant_step(&p, &world, u, c.dt); h[hn++ % 64] = u;
                if (fabs(p.x) > c.half_len - c.edge_margin) b = 1;
            }
            viol += b;
        }
        printf("  envelope Monte Carlo, k_acc x%.1f slew x%.1f: %ld recoverable states, %ld entered the keep-out strip\n", ks[m], ss[m], n, viol);
        if (viol) bad = 1;
    }
    /* 2. hostile AI under the guard: random and stuck commands in its range, random setpoints, 20 minutes */
    {
        rx_plant_t p = { 0, 0, 0 }; rx_est_t e = { 0 }; rx_guard_t g; rx_guard_init(&g);
        double worst = 1e9; int fb_used = 0; float u_ai = 0;
        for (long k = 0; k < (long)(1200 * c.hz); k++) {
            float sp = (float)(((k / (long)(5 * c.hz)) % 3 - 1) * c.sp_limit);
            rx_est_update(&e, &c, (float)(p.x + 0.0001 * nrand()));
            long ph = (k / (long)(2 * c.hz)) % 4;
            if (ph == 0) u_ai = (float)((urand() * 2 - 1) * c.theta_max);
            else if (ph == 1) u_ai = c.theta_max; else if (ph == 2) u_ai = -c.theta_max;
            else u_ai = rx_safe_u(&c, e.x, e.v, sp);
            rx_guard_in_t in = { 1, 1, 1, 1, u_ai }; rx_guard_out_t o;
            rx_guard_step(&g, &c, e.x, e.v, sp, &in, &o);
            if (o.u_out > c.theta_max * 1.001f) fb_used++;
            rx_plant_step(&p, &c, o.u_out, c.dt);
            double edge = c.half_len - c.edge_margin - fabs(p.x); if (edge < worst) worst = edge;
        }
        printf("  hostile AI under the guard, 20 min: closest to the keep-out strip %.1f mm, fallback used > theta_max on %d ticks\n", worst * 1e3, fb_used);
        if (worst < 0) bad = 1;
    }
    /* 3. from near the hinge end at rest, fallback only: off the end with the extra authority, settle at 0 */
    {
        rx_plant_t p = { -0.030, 0, 0 }; rx_est_t e = { 0 }; rx_guard_t g; rx_guard_init(&g);
        double worst = 1e9; int fb_used = 0;
        for (int k = 0; k < (int)(15 * c.hz); k++) {
            rx_est_update(&e, &c, (float)p.x);
            rx_guard_in_t in = { 1, 0, 0, 1, 0 }; rx_guard_out_t o;
            rx_guard_step(&g, &c, e.x, e.v, 0.0f, &in, &o);
            if (o.u_out > c.theta_max * 1.001f) fb_used++;
            rx_plant_step(&p, &c, o.u_out, c.dt);
            double edge = c.half_len - c.edge_margin - fabs(p.x); if (edge < worst) worst = edge;
        }
        printf("  from x = -30 mm at rest: after 15 s x = %+.1f mm, closest to the strip %.1f mm, extra authority on %d ticks\n", p.x * 1e3, worst * 1e3, fb_used);
        if (worst < 0 || fabs(p.x) > 0.003) bad = 1;
    }
    return bad;
}

/* Ultrasonic ball filter (beam-direct): a ball swinging +-40 mm at 0.3 Hz seen at 50 Hz with 0.3 mm noise,
 * 5 % dropouts, 3 % spikes (the room at 625 mm, a ghost at 167 mm, as on the rig in step3-ballmap-2), and a
 * 0.5 s stretch out of sight while it moves. No spike may get through; the position must track; the ball must
 * be re-acquired within 3 ticks of coming back; the guardian's estimator then gives the velocity. */
static int sonar_test(void)
{
    printf("== ultrasonic ball filter (beam-direct): range gate, spike gate, median of 3 ==\n");
    rx_sonar_cfg_t sc; rx_sonar_cfg_default(&sc);
    rx_sonar_filt_t f; rx_sonar_init(&f, &sc);
    rx_cfg_t c; rx_cfg_default(&c, 50.0f); c.est_tau = 0.04f;
    rx_est_t e = { 0, 0, 0 };
    uint64_t r = 0x9E3779B97F4A7C15ull;
    const float dt = 0.02f; int bad = 0, n_spk = 0, gaps_end = -1, reacq_tick = -1;
    double err_max = 0, err2 = 0, verr2 = 0; int nerr = 0, nverr = 0;
    for (int i = 0; i < 3000; i++) {                       /* 60 s */
        double t = i * dt, x = 106.0 + 40.0 * sin(2 * M_PI * 0.3 * t), v = 40.0 * 2 * M_PI * 0.3 * cos(2 * M_PI * 0.3 * t);
        r = r * 6364136223846793005ull + 1442695040888963407ull; double u = (double)(r >> 11) / 9007199254740992.0;
        r = r * 6364136223846793005ull + 1442695040888963407ull; double u2 = (double)(r >> 11) / 9007199254740992.0;
        float raw = (float)(x + 0.6 * (u2 - 0.5));
        int spike = 0, hidden = i >= 1500 && i < 1525;     /* out of sight for 0.5 s */
        if (hidden) raw = 640.0f;
        else if (u < 0.05) raw = -1.0f;
        else if (u < 0.065) { raw = 625.0f; spike = 1; }
        else if (u < 0.08) { raw = 167.0f; spike = 1; }
        if (i == 1525) gaps_end = i;
        float z;
        if (rx_sonar_step(&f, raw, dt, &z)) {
            if (spike && fabsf(z - raw) < 1e-3f && fabs(x - raw) > 20) { n_spk++; }
            double er = fabs(z - x);
            if (i > 10) { err_max = er > err_max ? er : err_max; err2 += er * er; nerr++; }
            if (gaps_end >= 0 && reacq_tick < 0) reacq_tick = i - gaps_end;
            rx_est_update(&e, &c, z * 1e-3f);
        } else rx_est_predict(&e, &c);
        if (i > 100 && !hidden && !(i >= 1525 && i < 1560)) { double ve = e.v * 1e3 - v; verr2 += ve * ve; nverr++; }
    }
    double rms = sqrt(err2 / nerr), vrms = sqrt(verr2 / nverr);
    printf("  %u readings: %u gated, %u spikes refused, %u re-acquisitions, %u accepted\n", f.n_in, f.n_gated, f.n_spikes, f.n_reacq, f.n_ok);
    printf("  position error rms %.2f mm, max %.2f mm; spikes passed %d; re-acquired %d ticks after coming back; velocity error rms %.1f mm/s\n",
           rms, err_max, n_spk, reacq_tick, vrms);
    if (n_spk) { printf("  FAIL: %d spikes passed the filter\n", n_spk); bad = 1; }
    if (err_max > 6.0) { printf("  FAIL: position error up to %.1f mm (want < 6)\n", err_max); bad = 1; }
    if (reacq_tick < 0 || reacq_tick > 3) { printf("  FAIL: re-acquired after %d ticks (want <= 3)\n", reacq_tick); bad = 1; }
    if (vrms > 25.0) { printf("  FAIL: velocity error rms %.1f mm/s (want < 25)\n", vrms); bad = 1; }
    return bad;
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
    enum { NJ = sizeof jm / sizeof jm[0] };
    jmc_t J[NJ]; pthread_t th[NJ]; int started[NJ];
    for (unsigned i = 0; i < NJ; i++) {               /* 250 Hz like the beam: a 4 ms adversary tick is harsher than 1 ms */
        J[i] = (jmc_t){ 250.0f, jm[i].i, jm[i].g, jm[i].slew, 200000, 0x9E3779B97F4A7C15ull ^ (0xD1B54A32D192ED03ull * (i + 1)), 0, 0 };
        started[i] = pthread_create(&th[i], NULL, envelope_mc_joint, &J[i]) == 0;
        if (!started[i]) envelope_mc_joint(&J[i]);  /* no threads: run it here */
    }
    for (unsigned i = 0; i < NJ; i++) {
        if (started[i]) pthread_join(th[i], NULL);
        printf("  %-34s %ld recoverable states tested, %d violations%s\n", jm[i].what, J[i].tested, J[i].viol, jm[i].must_pass ? "" : "  (informational)");
        if (J[i].viol && jm[i].must_pass) fail = 1;
    }

    printf("\n== guest supervision (pilot over UDP): restart limiter, simulated time ==\n");
    if (guest_test()) fail = 1;
    if (sonar_test()) fail = 1;
    if (beam_direct_test()) fail = 1;

    printf("\n%s\n", fail ? "SIMTEST FAILED" : "SIMTEST PASSED");
    return fail;
}
