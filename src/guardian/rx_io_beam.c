/* Plant I/O: the beam-direct rig. The Pi 4 drives the servo itself (hardware PWM through the QSTI GPIO manager)
 * and reads the ball with an HC-SR04, current and supply with an INA219, beam tilt with an MPU-6500, and an
 * e-stop button. No microcontroller. SPDX-License-Identifier: MIT
 *
 * Invariant 1 holds: every blocking call (MsgSend to the GPIO manager, the echo wait, I2C devctl) runs on one
 * helper thread. The control thread's write() only stores the command and posts a semaphore; read() copies the
 * newest measurement out of a seqlock mailbox and never waits (it gives up after 3 torn reads and reuses the
 * previous one).
 *
 * Helper, once per control tick (woken by write()):
 *   1. pulse = command, slew-limited to SLEW_US per tick and clamped to servo_min_us..servo_max_us; the e-stop
 *      and the watchdog (no tick for 150 ms: the control thread is stuck) both command level_us;
 *   2. one HC-SR04 ping, rx_sonar filter (range gate from reflex.conf, spike gate, median of 3);
 *   3. INA219: more than SERVO_TRIP_A on 3 consecutive ticks = servo limp, latched (the servo shares the Pi's
 *      5 V: results/2026-11-24-beam-direct/step2-*);
 *   4. MPU tilt (logging only), button (e-stop, latched until rxctl release), status LED;
 *   5. publish, and one row to beam.csv (the trial logs of step 4).
 * Test disturbance (RX_BEAM_KICK="mrad:ms", step 4 trials only): one second after the ball is first seen, the
 * helper holds the beam at that tilt for that long whatever the guardian commands, like a hand tilting the beam,
 * still slew-limited and inside the servo limits; then the guardian has the beam again. Logged (kick column). 
 * The measurement the control thread reads is the one taken right after the previous command: 1 tick old. */
#include <errno.h>
#include <math.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rx_io.h"
#include "rx_shm.h"

#ifdef __QNXNTO__
#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include "rx_rpi.h"
#include "rx_sonar.h"

#define PIN_SERVO 18
#define PIN_TRIG 23
#define PIN_ECHO 24
#define PIN_BUTTON 17
#define PIN_LED 22
#define PWM_RANGE 4000                /* 5 us per count at 50 Hz; the GPIO manager refuses ranges above 4095 */
#define SLEW_US RX_BEAM_SLEW_US       /* per tick: 3000 us/s at 50 Hz, about the MG995's own speed */
#define SERVO_TRIP_A 1.5f

typedef struct {
    _Atomic uint32_t seq;             /* seqlock: odd while the helper writes */
    float x; int valid; uint32_t n;   /* filtered ball position [m] (NaN when none) and a counter of ticks */
    float raw_mm, amp, volt, tilt_deg, filt_mm; int applied_us, target_us;
} snap_t;

typedef struct {
    const rx_cfg_t *cfg; rx_conf_t conf; rx_shm_t *shm;
    pthread_t th; int th_ok; sem_t tick; _Atomic int stop;
    _Atomic int cmd_us, estop, estop_clear, trip, ticks, kick_done, hold_us;   /* hold_us: operator tilt, 0 = none */
    snap_t snap;
    int i2c; rx_sonar_t so; rx_sonar_filt_t filt; FILE *log;
    /* control-thread side */
    uint32_t last_n; int gap; float last_x; int was_valid, lost_counted, told_estop, told_trip;
    int start_us;
} beam_t;

static uint64_t mono_ns(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return (uint64_t)t.tv_sec * 1000000000ull + (uint64_t)t.tv_nsec; }

static void publish(beam_t *b, const snap_t *v)
{
    uint32_t s = atomic_load_explicit(&b->snap.seq, memory_order_relaxed);
    atomic_store_explicit(&b->snap.seq, s + 1, memory_order_relaxed);
    atomic_thread_fence(memory_order_release);
    b->snap.x = v->x; b->snap.valid = v->valid; b->snap.n = v->n; b->snap.raw_mm = v->raw_mm;
    b->snap.amp = v->amp; b->snap.volt = v->volt; b->snap.tilt_deg = v->tilt_deg; b->snap.applied_us = v->applied_us;
    b->snap.filt_mm = v->filt_mm; b->snap.target_us = v->target_us;
    atomic_thread_fence(memory_order_release);
    atomic_store_explicit(&b->snap.seq, s + 2, memory_order_release);
}

static int snapshot(beam_t *b, snap_t *out)
{
    for (int i = 0; i < 3; i++) {
        uint32_t s1 = atomic_load_explicit(&b->snap.seq, memory_order_acquire);
        if (s1 & 1) continue;
        snap_t v = { 0 };
        v.x = b->snap.x; v.valid = b->snap.valid; v.n = b->snap.n; v.raw_mm = b->snap.raw_mm;
        v.amp = b->snap.amp; v.volt = b->snap.volt; v.tilt_deg = b->snap.tilt_deg; v.applied_us = b->snap.applied_us;
        v.filt_mm = b->snap.filt_mm; v.target_us = b->snap.target_us;
        atomic_thread_fence(memory_order_acquire);
        if (atomic_load_explicit(&b->snap.seq, memory_order_relaxed) == s1) { *out = v; return 1; }
    }
    return 0;
}

static void *helper(void *arg)
{
    beam_t *b = arg;
    const float dt = b->cfg->dt;
    int applied = b->start_us, over = 0, low = 0;
    uint32_t n = 0; uint64_t t0 = mono_ns();
    float tilt = NAN;
    int kick_left = -1, kick_us = 0, seen_ticks = 0;                       /* -1: no kick configured */
    const char *ke = getenv("RX_BEAM_KICK");
    float kmrad = 0, kms = 0;
    if (ke && sscanf(ke, "%f:%f", &kmrad, &kms) == 2 && kms > 0 && kms <= 3000 && fabsf(kmrad) <= 50) {
        kick_left = (int)(kms / 1000.0f / dt + 0.5f);
        kick_us = b->conf.level_us + (int)lrintf((float)b->conf.servo_dir * kmrad * b->conf.us_per_mrad);
    }
    int kicking = 0;
    while (!atomic_load(&b->stop)) {
        struct timespec ts; clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += 150000000L; if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
        int ticked = sem_timedwait(&b->tick, &ts) == 0;
        if (atomic_load(&b->stop)) break;
        int target = ticked ? atomic_load(&b->cmd_us) : b->conf.level_us;    /* watchdog: level */
        if (atomic_exchange(&b->estop_clear, 0)) atomic_store(&b->estop, 0);
        int estop = atomic_load(&b->estop);
        int hold = atomic_load(&b->hold_us);
        if (hold) target = hold;                                               /* operator tilt hold */
        if (estop) target = b->conf.level_us;
        kicking = 0;
        if (kick_left > 0 && seen_ticks >= (int)(1.0f / dt) && !estop) { target = kick_us; kick_left--; kicking = 1;
            if (kick_left == 0) atomic_store(&b->kick_done, 1); }
        int d = target - applied;
        applied += d > SLEW_US ? SLEW_US : (d < -SLEW_US ? -SLEW_US : d);
        if (applied < b->conf.servo_min_us) applied = b->conf.servo_min_us;
        if (applied > b->conf.servo_max_us) applied = b->conf.servo_max_us;
        int trip = atomic_load(&b->trip);
        rx_gpio_pwm_duty(PIN_SERVO, trip ? 0u : (unsigned)((applied * PWM_RANGE + 10000) / 20000));

        float raw = rx_sonar_measure(&b->so, 6, NULL), z = NAN;
        int ok = rx_sonar_step(&b->filt, raw, dt, &z);
        if (ok || seen_ticks) seen_ticks++;

        float a = 0, v = 0;
        if (rx_ina_read(b->i2c, 0x40, &v, NULL, &a, NULL) == 0) {
            over = fabsf(a) > SERVO_TRIP_A ? over + 1 : 0;
            if (over >= 3 && !trip) { atomic_store(&b->trip, (int)lrintf(fabsf(a) * 1000)); rx_gpio_pwm_duty(PIN_SERVO, 0); }
        }
        rx_mpu_t m;
        if ((n & 1) == 0 && rx_mpu_read(b->i2c, 0x68, &m) == 0) tilt = atan2f(-m.ax, m.az) * 57.29578f;
        int btn = rx_gpio_read(PIN_BUTTON);
        low = btn == 0 ? low + 1 : 0;
        if (low == 2) atomic_store(&b->estop, 1);
        rx_gpio_write(PIN_LED, estop || trip ? (int)(n / 5) & 1 : 1);          /* blinking = e-stop or trip */

        snap_t s = { 0 };
        s.x = ok ? (float)b->conf.sensor_dir * (z - (float)b->conf.center_mm) * 1e-3f : NAN;
        s.valid = ok; s.n = ++n; s.raw_mm = raw; s.amp = a; s.volt = v; s.tilt_deg = tilt; s.applied_us = applied;
        s.filt_mm = ok ? z : NAN; s.target_us = target;
        publish(b, &s);
        if (b->log) fprintf(b->log, "%.3f,%.1f,%.1f,%d,%d,%d,%.0f,%.3f,%.2f,%d,%d,%d,%d\n", (mono_ns() - t0) * 1e-9, raw, ok ? z : -1.0f,
                            ok, target, trip ? 0 : applied, a * 1000, v, tilt, estop, trip ? 1 : 0, ticked, kicking);
    }
    rx_gpio_pwm_duty(PIN_SERVO, 0);                                             /* closing: servo limp */
    rx_gpio_write(PIN_LED, 0);
    return NULL;
}

static float beam_read(rx_io_t *io, float *x_true, int *lost, int *fresh)
{
    beam_t *b = io->priv;
    *x_true = NAN; *fresh = 0;
    snap_t s;
    int have = snapshot(b, &s);
    if (have && s.n != b->last_n) {
        b->last_n = s.n; *fresh = 1;
        /* dashboard diagnostics (display only) */
        rx_shm_t *m = b->shm;
        m->beam_raw_mm = s.raw_mm; m->beam_filt_mm = s.filt_mm; m->beam_amp_ma = s.amp * 1000.0f; m->beam_supply_v = s.volt;
        m->beam_tilt_deg = s.tilt_deg; m->beam_hit_pct = 0.98f * m->beam_hit_pct + (s.valid ? 2.0f : 0.0f);
        atomic_store(&m->beam_applied_us, (uint32_t)s.applied_us); atomic_store(&m->beam_target_us, (uint32_t)s.target_us);
        atomic_store(&m->beam_estop, (uint32_t)atomic_load(&b->estop)); atomic_store(&m->beam_trip, (uint32_t)(atomic_load(&b->trip) != 0));
        {   /* graph history */
            uint64_t h = atomic_load_explicit(&m->beam_head, memory_order_relaxed);
            __typeof__(m->beam_hist[0]) *e = &m->beam_hist[h % 1024];
            atomic_store_explicit(&e->seq, 0, memory_order_relaxed);
            e->raw_mm = s.raw_mm; e->filt_mm = s.filt_mm; e->amp_ma = s.amp * 1000.0f; e->volt = s.volt; e->tilt_deg = s.tilt_deg;
            e->applied_us = (uint16_t)s.applied_us; e->target_us = (uint16_t)s.target_us; e->accepted = (uint8_t)s.valid;
            e->hold = (uint8_t)(atomic_load(&b->shm->beam_hold) != 0);
            atomic_store_explicit(&e->seq, (uint32_t)h + 1, memory_order_release);
            atomic_store_explicit(&m->beam_head, h + 1, memory_order_release);
        }
        if (s.valid) { b->gap = 0; b->last_x = s.x; } else b->gap++;
        /* evaluation only (the guardian's decisions never use it): the ball reached the end of the beam it can
         * be seen on, or was last seen near an end and then vanished for 0.5 s */
        int at_end = s.valid && fabsf(s.x) >= b->cfg->half_len;
        int vanished = !s.valid && b->gap == 25 && fabsf(b->last_x) > b->cfg->half_len - 0.012f;
        if ((at_end || vanished) && !b->lost_counted) {
            b->lost_counted = 1;
            atomic_fetch_add(&b->shm->balls_lost, 1);
            rx_event_push(b->shm, 0, RX_E_BALL_LOST, (int32_t)lrintf(b->last_x * 1000.0f));
        }
        if (s.valid && fabsf(s.x) < b->cfg->half_len - 0.02f) b->lost_counted = 0;
    }
    /* operator tilt hold, from the shared memory (rxctl / dashboard "tilt <mrad>" / "tilt off") */
    if (atomic_load(&b->shm->beam_hold)) {
        float mr = atomic_load(&b->shm->beam_hold_mrad10) * 0.1f;
        int p = b->conf.level_us + (int)lrintf((float)b->conf.servo_dir * mr * b->conf.us_per_mrad);
        if (p < b->conf.servo_min_us) p = b->conf.servo_min_us;
        if (p > b->conf.servo_max_us) p = b->conf.servo_max_us;
        atomic_store(&b->hold_us, p);
    } else atomic_store(&b->hold_us, 0);
    int t = atomic_load(&b->trip);
    if (t && !b->told_trip) { b->told_trip = 1; rx_event_push(b->shm, 0, RX_E_SERVO_TRIP, t); }
    if (atomic_load(&b->estop) && !b->told_estop) { b->told_estop = 1; rx_event_push(b->shm, 0, RX_E_ESTOP, 0); }
    if (atomic_exchange(&b->kick_done, 0)) rx_event_push(b->shm, 0, RX_E_KICK, 0);
    *lost = b->gap >= RX_SONAR_LOST_TICKS || !have;
    return *lost ? NAN : b->last_x;
}

static void beam_write(rx_io_t *io, float u)
{
    beam_t *b = io->priv;
    float lim = b->cfg->theta_max;                                  /* never ask for more than the kernel allows */
    if (isfinite(u)) u = u > lim ? lim : (u < -lim ? -lim : u);
    int p = rx_conf_pulse(&b->conf, u);
    if (p < b->conf.servo_min_us) p = b->conf.servo_min_us;
    if (p > b->conf.servo_max_us) p = b->conf.servo_max_us;
    atomic_store(&b->cmd_us, p);
    sem_post(&b->tick);
}

static int beam_estop(rx_io_t *io) { beam_t *b = io->priv; return atomic_load(&b->estop) || atomic_load(&b->trip); }
static void beam_release(rx_io_t *io)
{
    beam_t *b = io->priv;
    atomic_store(&b->estop_clear, 1);
    b->told_estop = 0;                                              /* a servo trip stays latched until restart */
}

static void beam_close(rx_io_t *io)
{
    beam_t *b = io->priv;
    atomic_store(&b->stop, 1); sem_post(&b->tick);
    if (b->th_ok) pthread_join(b->th, NULL);
    if (b->log) fclose(b->log);
    free(b); free(io);
}

rx_io_t *rx_io_beam_open(const rx_cfg_t *cfg, rx_shm_t *shm, const rx_conf_t *conf, const char *log_path)
{
    if (!conf->servo_min_us) { fprintf(stderr, "[beam] reflex.conf has no servo_min_us/servo_max_us (rxhw servo-cal / range)\n"); return NULL; }
    beam_t *b = calloc(1, sizeof *b);
    rx_io_t *io = calloc(1, sizeof *io);
    if (!b || !io) { free(b); free(io); return NULL; }
    b->cfg = cfg; b->conf = *conf; b->shm = shm; b->last_x = 0;
    b->gap = RX_SONAR_LOST_TICKS;                                   /* nothing measured yet = lost (beam level) */
    atomic_store(&shm->beam_present, 1); atomic_store(&shm->beam_level_us, (uint32_t)conf->level_us);
    atomic_store(&shm->beam_servo_dir, conf->servo_dir); shm->beam_us_per_mrad = conf->us_per_mrad;
    /* RX_BEAM_HOLD=mrad: start held at that tilt (no jump when the servo is already there), until "tilt off" */
    const char *he = getenv("RX_BEAM_HOLD"); float hm = 0;
    int start_us = conf->level_us;
    if (he && sscanf(he, "%f", &hm) == 1 && fabsf(hm) <= 60) {
        atomic_store(&shm->beam_hold_mrad10, (int32_t)lrintf(hm * 10)); atomic_store(&shm->beam_hold, 1);
        start_us = conf->level_us + (int)lrintf((float)conf->servo_dir * hm * conf->us_per_mrad);
        if (start_us < conf->servo_min_us) start_us = conf->servo_min_us;
        if (start_us > conf->servo_max_us) start_us = conf->servo_max_us;
        atomic_store(&b->hold_us, start_us);
    }
    b->start_us = start_us;
    atomic_store(&b->cmd_us, conf->level_us);
    if (rx_gpio_open() != 0) { perror("[beam] /dev/gpio/msg"); goto fail; }
    if ((b->i2c = rx_i2c_open(1)) < 0) { perror("[beam] /dev/i2c1"); goto fail; }
    uint8_t who = 0;
    if (rx_mpu_init(b->i2c, 0x68, &who) != 0) fprintf(stderr, "[beam] MPU at 0x68 not answering: tilt not logged\n");
    if (rx_sonar_open(&b->so, PIN_TRIG, PIN_ECHO) != 0) { perror("[beam] HC-SR04"); goto fail; }
    rx_gpio_input(PIN_BUTTON, 1); rx_gpio_output(PIN_LED, 0);
    rx_sonar_cfg_t sc; rx_sonar_cfg_default(&sc);
    sc.lo_mm = (float)conf->range_min_mm; sc.hi_mm = (float)conf->range_max_mm;
    rx_sonar_init(&b->filt, &sc);
    /* start at level: the beam rests near it after the calibration tools; the slew limit takes care of the rest */
    if (rx_gpio_pwm_setup(PIN_SERVO, 50, PWM_RANGE) != 0 ||
        rx_gpio_pwm_duty(PIN_SERVO, (unsigned)((b->start_us * PWM_RANGE + 10000) / 20000)) != 0) { perror("[beam] PWM"); goto fail; }
    if (sem_init(&b->tick, 0, 0) != 0) goto fail;
    if (log_path && (b->log = fopen(log_path, "w")))
        fprintf(b->log, "t_s,raw_mm,filt_mm,accepted,target_us,applied_us,current_ma,supply_v,mpu_tilt_deg,estop,trip,ticked,kick\n");
    if (pthread_create(&b->th, NULL, helper, b) != 0) { perror("[beam] helper"); goto fail; }
    b->th_ok = 1;
    io->read = beam_read; io->write = beam_write; io->close = beam_close; io->name = "beam-direct"; io->priv = b;
    io->estop = beam_estop; io->release = beam_release;
    return io;
fail:
    rx_gpio_pwm_duty(PIN_SERVO, 0);
    free(b); free(io);
    return NULL;
}

#else   /* Linux: the rig is a Pi under QNX */
rx_io_t *rx_io_beam_open(const rx_cfg_t *cfg, rx_shm_t *shm, const rx_conf_t *conf, const char *log_path)
{
    (void)cfg; (void)shm; (void)conf; (void)log_path;
    fprintf(stderr, "[beam] the beam-direct backend runs on the Pi under QNX only\n");
    return NULL;
}
#endif
