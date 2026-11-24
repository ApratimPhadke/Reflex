/* rxhw - bring-up and checks for the beam-direct rig on a Pi 4 under QNX (no microcontroller).
 * SPDX-License-Identifier: MIT
 *
 *   rxhw scan                  every sensor once, PASS/FAIL with the value. Never drives the servo.
 *   rxhw watch <secs>          sample tilt (both MPU6050), HC-SR04 and the button for <secs> while a person
 *                              tilts the beam, moves a hand in front of the sensor, presses the button;
 *                              prints the ranges seen and PASS/FAIL for each hand check
 *   rxhw led <n>               blink the status LED n times (a person confirms)
 *   rxhw sonar <n> [ms]        n distance readings, one every ms (default 50): median, spread, valid count
 *   options: --bus N (I2C bus, default: the one where 0x40/0x68 answer)
 *
 * Wiring (BCM numbers): servo PWM GPIO18, HC-SR04 trig GPIO23 / echo GPIO24 (1k/2k divider),
 * button GPIO17 to GND (internal pull-up), LED GPIO22. I2C: INA219 0x40, MPU6050 0x68 (beam), 0x69 (spare). */
#include <errno.h>
#include <signal.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "rx_rpi.h"
#include "rx_conf.h"
#include "rx_core.h"
#include "rx_sonar.h"
#ifndef RX_GIT
#define RX_GIT "unknown"
#endif

#define PIN_SERVO  18
#define PIN_TRIG   23
#define PIN_ECHO   24
#define PIN_BUTTON 17
#define PIN_LED    22
#define A_INA  0x40
#define A_MPU1 0x68
#define A_MPU2 0x69

static int fails, want_mpu2;           /* the spare MPU (0x69) is only checked with --mpu2 */
static void res(int ok, const char *what, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
static void res(int ok, const char *what, const char *fmt, ...)
{
    char b[256]; va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    printf("%s  %-22s %s\n", ok ? "PASS" : "FAIL", what, b);
    if (!ok) fails++;
}

static double now_s(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
static float rad2deg(float r) { return r * 57.29578f; }
/* tilt about each horizontal axis from gravity: which one is the beam's depends on how the board is taped on */
static float tilt_x(const rx_mpu_t *m) { return rad2deg(atan2f(m->ax, sqrtf(m->ay * m->ay + m->az * m->az))); }
static float tilt_y(const rx_mpu_t *m) { return rad2deg(atan2f(m->ay, sqrtf(m->ax * m->ax + m->az * m->az))); }

static int pick_bus(int want)
{
    if (want >= 0) return want;
    for (int b = 0; b < 2; b++) {
        int fd = rx_i2c_open(b); if (fd < 0) continue;
        int hit = rx_i2c_probe(fd, A_INA) || rx_i2c_probe(fd, A_MPU1);
        close(fd);
        if (hit) return b;
    }
    return 1;
}

static int cmp_f(const void *a, const void *b) { float x = *(const float *)a, y = *(const float *)b; return (x > y) - (x < y); }

static int cmd_scan(int bus)
{
    printf("rxhw scan, build %s (servo is not driven)\n", RX_GIT);
    /* GPIO manager */
    int g = rx_gpio_open();
    res(g == 0, "GPIO manager", g == 0 ? "/dev/gpio/msg open" : "/dev/gpio/msg: %s (is rpi_gpio running? are you in group gpio?)", strerror(errno));
    /* I2C */
    int fd = rx_i2c_open(bus);
    res(fd >= 0, "I2C bus", fd >= 0 ? "/dev/i2c%d open" : "/dev/i2c%d: %s", bus, strerror(errno));
    if (fd >= 0) {
        char list[400] = ""; size_t k = 0;
        for (int a = 0x03; a <= 0x77; a++) if (rx_i2c_probe(fd, (uint8_t)a)) k += (size_t)snprintf(list + k, sizeof list - k, " 0x%02x", a);
        printf("      i2c%d answers:%s\n", bus, k ? list : " (nothing)");
        const struct { uint8_t a; const char *n; } want[] = { { A_INA, "INA219" }, { A_MPU1, "MPU6050 #1 (beam)" }, { A_MPU2, "MPU6050 #2 (spare)" } };
        for (unsigned i = 0; i < 3; i++) {
            char w[40]; snprintf(w, sizeof w, "I2C 0x%02x", want[i].a);
            int ok = rx_i2c_probe(fd, want[i].a);
            if (!ok && want[i].a == A_MPU2 && !want_mpu2) { printf("SKIP  %-22s %s not fitted (check with --mpu2)\n", w, want[i].n); continue; }
            res(ok, w, "%s %s", want[i].n, ok ? "acknowledges" : "does not answer");
        }
        for (int i = 0; i < 2; i++) {
            uint8_t a = i ? A_MPU2 : A_MPU1, who = 0; rx_mpu_t m;
            char w[40]; snprintf(w, sizeof w, "MPU6050 #%d tilt", i + 1);
            if (i == 1 && !want_mpu2 && !rx_i2c_probe(fd, A_MPU2)) continue;
            if (rx_mpu_init(fd, a, &who) != 0 || rx_mpu_read(fd, a, &m) != 0) { res(0, w, "0x%02x: no data (%s)", a, strerror(errno)); continue; }
            float gmag = sqrtf(m.ax * m.ax + m.ay * m.ay + m.az * m.az);
            if (who == 0x70 || who == 0x71 || who == 0x73)              /* MPU-6500 family: 333.87 LSB/C, 21 C offset */
                m.temp_c = (m.temp_c - 36.53f) * 340.0f / 333.87f + 21.0f;
            int ok = gmag > 0.8f && gmag < 1.2f && m.temp_c > -10 && m.temp_c < 70;
            res(ok, w, "who=0x%02x |g|=%.3f  tilt_x=%+.1f deg tilt_y=%+.1f deg  gyro=(%+.1f %+.1f %+.1f) deg/s  %.1f C",
                who, gmag, tilt_x(&m), tilt_y(&m), m.gx, m.gy, m.gz, m.temp_c);
        }
        float bv, sh, cur; uint16_t cfg;
        if (rx_ina_read(fd, A_INA, &bv, &sh, &cur, &cfg) != 0) res(0, "INA219", "no data (%s)", strerror(errno));
        else {
            res(cfg != 0 && cfg != 0xFFFF, "INA219 config", "0x%04x (power-on default 0x399f)", cfg);
            res(bv > 4.3f && bv < 6.0f, "INA219 servo supply", "%.3f V", bv);
            res(fabsf(cur) < 0.5f, "INA219 current", "%.0f mA (shunt %.2f mV; servo not driven, expect near 0)", cur * 1000, sh);
        }
        close(fd);
    }
    if (g != 0) { printf("\n%d check(s) failed\n", fails); return 1; }
    /* servo pin: only looked at */
    int f = rx_gpio_func(PIN_SERVO);
    printf("      GPIO18 (servo) function code %d: left alone\n", f);
    /* button */
    if (rx_gpio_input(PIN_BUTTON, 1) != 0) res(0, "button GPIO17", "setup: %s", strerror(errno));
    else { usleep(2000); int l = rx_gpio_read(PIN_BUTTON); res(l == 1, "button GPIO17", "level %d (%s)", l, l == 1 ? "released, pull-up" : l == 0 ? "LOW: pressed, or shorted to GND" : "read error"); }
    /* LED: three blinks; a person confirms */
    if (rx_gpio_output(PIN_LED, 0) != 0) res(0, "LED GPIO22", "setup: %s", strerror(errno));
    else { for (int i = 0; i < 3; i++) { rx_gpio_write(PIN_LED, 1); usleep(200000); rx_gpio_write(PIN_LED, 0); usleep(200000); }
           printf("      LED GPIO22 blinked 3 times (confirm by eye)\n"); }
    /* HC-SR04 */
    rx_sonar_t s;
    if (rx_sonar_open(&s, PIN_TRIG, PIN_ECHO) != 0) res(0, "HC-SR04", "setup: %s", strerror(errno));
    else {
        int idle = rx_gpio_read(PIN_ECHO);
        float d[15]; int n = 0;
        for (int i = 0; i < 15; i++) { float us; float mm = rx_sonar_measure(&s, 40, &us); if (mm > 0) d[n++] = mm; usleep(60000); }
        if (n) qsort(d, (size_t)n, sizeof d[0], cmp_f);
        if (n) res(idle == 0 && n >= 12, "HC-SR04 distance", "median %.0f mm (min %.0f, max %.0f), %d/15 echoes, echo idle level %d",
                   d[n / 2], d[0], d[n - 1], n, idle);
        else res(0, "HC-SR04 distance", "no echo in 15 tries, echo idle level %d (%s)", idle,
                 idle == 1 ? "echo stuck HIGH: check the divider and the echo wire on GPIO24" : "check trig on GPIO23, 5 V and GND at the sensor");
    }
    printf("\n%s: %d check(s) failed\n", fails ? "NOT READY" : "ALL PASS", fails);
    return fails ? 1 : 0;
}

static int cmd_watch(int bus, double secs)
{
    int fd = rx_i2c_open(bus);
    if (fd < 0 || rx_gpio_open() != 0) { perror("rxhw watch"); return 1; }
    int have2 = rx_i2c_probe(fd, A_MPU2);
    rx_mpu_init(fd, A_MPU1, NULL); if (have2) rx_mpu_init(fd, A_MPU2, NULL);
    rx_gpio_input(PIN_BUTTON, 1);
    rx_gpio_output(PIN_LED, 0);                                   /* the LED blinks while the window is open */
    rx_sonar_t s; int sonar = rx_sonar_open(&s, PIN_TRIG, PIN_ECHO) == 0;
    float tmin[2][2] = { { 1e9, 1e9 }, { 1e9, 1e9 } }, tmax[2][2] = { { -1e9, -1e9 }, { -1e9, -1e9 } };
    float dmin = 1e9, dmax = -1e9; int dn = 0, dtot = 0, presses = 0, last_btn = 1;
    double t0 = now_s(), next_print = t0;
    printf("rxhw watch %.0f s: tilt the beam, move a hand in front of the HC-SR04, press the button\n", secs); fflush(stdout);
    while (now_s() - t0 < secs) {
        rx_mpu_t m[2]; int ok[2];
        for (int i = 0; i < 2; i++) {
            ok[i] = (i == 0 || have2) && rx_mpu_read(fd, i ? A_MPU2 : A_MPU1, &m[i]) == 0;
            if (!ok[i]) continue;
            float tx = tilt_x(&m[i]), ty = tilt_y(&m[i]);
            tmin[i][0] = fminf(tmin[i][0], tx); tmax[i][0] = fmaxf(tmax[i][0], tx);
            tmin[i][1] = fminf(tmin[i][1], ty); tmax[i][1] = fmaxf(tmax[i][1], ty);
        }
        float mm = -1;
        if (sonar) { mm = rx_sonar_measure(&s, 40, NULL); dtot++; if (mm > 0) { dn++; if (mm < dmin) dmin = mm; if (mm > dmax) dmax = mm; } }
        int b = rx_gpio_read(PIN_BUTTON);
        if (b == 0 && last_btn == 1) presses++;
        if (b >= 0) last_btn = b;
        if (now_s() >= next_print) {
            printf("t=%5.1f  mpu1 x=%+6.1f y=%+6.1f  mpu2 x=%+6.1f y=%+6.1f  dist=%6.0f mm  button=%s  presses=%d\n", now_s() - t0,
                   ok[0] ? tilt_x(&m[0]) : NAN, ok[0] ? tilt_y(&m[0]) : NAN, ok[1] ? tilt_x(&m[1]) : NAN, ok[1] ? tilt_y(&m[1]) : NAN,
                   mm, b == 0 ? "DOWN" : b == 1 ? "up" : "?", presses);
            fflush(stdout); next_print += 0.5;
        }
        rx_gpio_write(PIN_LED, ((int)((now_s() - t0) * 2)) & 1);
        usleep(50000);
    }
    rx_gpio_write(PIN_LED, 0);
    printf("\nsummary over %.0f s\n", secs);
    for (int i = 0; i < (have2 || want_mpu2 ? 2 : 1); i++) {
        float rx_ = tmax[i][0] - tmin[i][0], ry = tmax[i][1] - tmin[i][1], r = rx_ > ry ? rx_ : ry;
        char w[40]; snprintf(w, sizeof w, "MPU6050 #%d moved", i + 1);
        res(r >= 5.0f, w, "tilt_x %+.1f..%+.1f, tilt_y %+.1f..%+.1f deg (largest swing %.1f deg on %s; need >= 5)",
            tmin[i][0], tmax[i][0], tmin[i][1], tmax[i][1], r, rx_ > ry ? "x" : "y");
    }
    res(dn > 0 && dmax - dmin >= 50.0f, "HC-SR04 hand", "%.0f..%.0f mm, %d/%d echoes (need a change >= 50 mm)", dn ? dmin : 0.0f, dn ? dmax : 0.0f, dn, dtot);
    res(presses >= 1, "button press", "%d press(es) seen", presses);
    return fails ? 1 : 0;
}

/* register dump and raw samples of one MPU (diagnosing clones, sleep, address clashes) */
static int cmd_mpu(int bus, int addr)
{
    int fd = rx_i2c_open(bus);
    if (fd < 0) { perror("rxhw mpu"); return 1; }
    uint8_t r[] = { 0x75, 0x6B, 0x6C, 0x1A, 0x1B, 0x1C, 0x1D, 0x19, 0x37, 0x6A };
    printf("i2c%d 0x%02x before init:", bus, addr);
    for (unsigned i = 0; i < sizeof r; i++) { uint8_t v = 0; int e = rx_i2c_read(fd, (uint8_t)addr, r[i], &v, 1); printf(" [%02x]=%s%02x", r[i], e ? "ERR" : "", v); }
    uint8_t who = 0; int e = rx_mpu_init(fd, (uint8_t)addr, &who);
    printf("\ninit %s, who 0x%02x; after 200 ms:", e ? "FAILED" : "ok", who);
    usleep(200000);
    for (unsigned i = 0; i < sizeof r; i++) { uint8_t v = 0; int e2 = rx_i2c_read(fd, (uint8_t)addr, r[i], &v, 1); printf(" [%02x]=%s%02x", r[i], e2 ? "ERR" : "", v); }
    printf("\n");
    for (int k = 0; k < 8; k++) {
        uint8_t b[14] = { 0 };
        if (rx_i2c_read(fd, (uint8_t)addr, 0x3B, b, 14) != 0) { printf("read error %s\n", strerror(errno)); continue; }
        printf("raw:"); for (int i = 0; i < 14; i++) printf(" %02x", b[i]);
        rx_mpu_t m; rx_mpu_read(fd, (uint8_t)addr, &m);
        printf("   a=(%+.3f %+.3f %+.3f) g  gyro=(%+.2f %+.2f %+.2f)\n", m.ax, m.ay, m.az, m.gx, m.gy, m.gz);
        usleep(100000);
    }
    return 0;
}

/* ------------------------------------------------------------------ servo range
 * Moves the servo, so only on request. PWM with the settings of QNX's servo sample (50 Hz, range 1024: one
 * count = 19.53 us, about 2 degrees of servo). Every wait polls the INA219 every 20 ms and cuts the pulses
 * (servo limp) at once above SERVO_TRIP_A. Steps of one count from the centre, out to where the beam stops
 * tilting further, the holding current jumps, the tilt passes 14 degrees, or the pulse reaches 1000/2000 us. */
#define SERVO_TRIP_A 1.5f
#define PWM_RANGE 1024
static int s_fd = -1, s_tripped;
static float s_peak_a, s_min_v = 99;
static float s_lo_us = 1000, s_hi_us = 2000;                  /* pulse bounds of the sweep */
static unsigned us2c(float us) { return (unsigned)(us * PWM_RANGE / 20000.0f + 0.5f); }
static float c2us(unsigned c) { return c * 20000.0f / PWM_RANGE; }
static void servo_limp(void) { rx_gpio_pwm_duty(PIN_SERVO, 0); }
static int wait_watch(int ms, float *avg_a)                      /* 0 = ok, -1 = tripped */
{
    double t0 = now_s(), sum = 0; int n = 0;
    do {
        float a = 0, v = 0;
        if (rx_ina_read(s_fd, A_INA, &v, NULL, &a, NULL) != 0) { servo_limp(); s_tripped = 1; printf("TRIP: INA219 read failed\n"); return -1; }
        a = fabsf(a); sum += a; n++;
        if (v < s_min_v) s_min_v = v;
        if (a > s_peak_a) s_peak_a = a;
        if (a > SERVO_TRIP_A) { servo_limp(); s_tripped = 1; printf("TRIP: %.2f A > %.1f A, servo limp\n", a, SERVO_TRIP_A); fflush(stdout); return -1; }
        usleep(20000);
    } while ((now_s() - t0) * 1000 < ms);
    if (avg_a) *avg_a = (float)(sum / n);
    return 0;
}
static int gravity(float g[3])                                   /* mean unit gravity vector over ~200 ms */
{
    double s3[3] = { 0, 0, 0 }; int n = 0;
    for (int i = 0; i < 10; i++) {
        rx_mpu_t m;
        if (rx_mpu_read(s_fd, A_MPU1, &m) == 0) { s3[0] += m.ax; s3[1] += m.ay; s3[2] += m.az; n++; }
        if (wait_watch(20, NULL) != 0) return -1;
    }
    if (!n) return -1;
    double l = sqrt(s3[0] * s3[0] + s3[1] * s3[1] + s3[2] * s3[2]);
    for (int k = 0; k < 3; k++) g[k] = (float)(s3[k] / l);
    return 0;
}
static float angle_deg(const float a[3], const float b[3])
{
    float d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    return rad2deg(acosf(d > 1 ? 1 : d < -1 ? -1 : d));
}
typedef struct { unsigned c; float g[3], amp, dang; } step_t;

static int sweep(int dir, unsigned c0, const float g0[3], step_t *st, int max, unsigned *limit, const char **why)
{
    int n = 0, flat = 0; float prev = 0, hold0 = -1;
    unsigned c = c0, lo = us2c(s_lo_us), hi = us2c(s_hi_us);
    *why = "pulse bound";
    for (;;) {
        unsigned nc = c + (unsigned)dir;
        if (nc < lo || nc > hi) { *limit = c; break; }
        c = nc;
        if (rx_gpio_pwm_duty(PIN_SERVO, c) != 0) { servo_limp(); printf("PWM duty failed: %s\n", strerror(errno)); return -1; }
        float amp;
        if (wait_watch(350, &amp) != 0) return -1;
        float g[3]; if (gravity(g) != 0) return -1;
        float ang = angle_deg(g, g0);
        step_t *s = &st[n]; s->c = c; memcpy(s->g, g, sizeof g); s->amp = amp; s->dang = ang - prev;
        printf("  %4.0f us  tilt from centre %5.2f deg (step %+.2f)  hold %4.0f mA  peak so far %4.0f mA  min supply %.2f V\n",
               c2us(c), ang, s->dang, amp * 1000, s_peak_a * 1000, s_min_v); fflush(stdout);
        if (n == 2) hold0 = (st[0].amp + st[1].amp + st[2].amp) / 3;
        n++;
        /* stopped tilting: under 0.15 deg gained over the last 4 steps (about 0.5 deg expected) */
        (void)flat;
        if (n > 4) {
            float gain = 0; for (int j = n - 4; j < n; j++) gain += st[j].dang;
            if (gain < 0.15f) { *limit = st[n - 5].c; *why = "beam stopped tilting further"; break; }
        }
        if (hold0 >= 0 && amp > 0.35f && amp > 4 * hold0) { *limit = st[n - 2].c; *why = "holding current jumped (binding?)"; break; }
        if (ang > 14.0f) { *limit = st[n - 2].c; *why = "tilt past 14 deg"; break; }
        if (n >= max) { *limit = c; *why = "step budget"; break; }
        prev = ang;
    }
    return n;
}
static int move_slow(unsigned from, unsigned to)                 /* one count per 150 ms */
{
    unsigned c = from;
    while (c != to) {
        c = c < to ? c + 1 : c - 1;
        if (rx_gpio_pwm_duty(PIN_SERVO, c) != 0) return -1;
        if (wait_watch(150, NULL) != 0) return -1;
    }
    return 0;
}

static int cmd_servo_cal(int bus)
{
    s_fd = rx_i2c_open(bus);
    if (s_fd < 0 || rx_gpio_open() != 0) { perror("rxhw servo-cal"); return 1; }
    uint8_t who; if (rx_mpu_init(s_fd, A_MPU1, &who) != 0) { perror("MPU"); return 1; }
    float g_rest[3]; if (gravity(g_rest) != 0) { printf("no MPU data\n"); return 1; }
    printf("servo limp: g = (%+.3f %+.3f %+.3f)\n", g_rest[0], g_rest[1], g_rest[2]);
    unsigned c0 = us2c(1500);
    if (rx_gpio_pwm_setup(PIN_SERVO, 50, PWM_RANGE) != 0 || rx_gpio_pwm_duty(PIN_SERVO, c0) != 0) {
        servo_limp(); printf("PWM setup failed: %s\n", strerror(errno)); return 1;
    }
    printf("PWM on: 50 Hz, range %d, centre %u counts = %.0f us (the servo goes there at its own speed)\n", PWM_RANGE, c0, c2us(c0)); fflush(stdout);
    float amp; if (wait_watch(1500, &amp) != 0) { servo_limp(); return 1; }
    float g0[3]; if (gravity(g0) != 0) { servo_limp(); return 1; }
    printf("centre: g = (%+.3f %+.3f %+.3f), %.0f mA, moved %.1f deg from the limp rest\n", g0[0], g0[1], g0[2], amp * 1000, angle_deg(g0, g_rest));
    static step_t up[120], dn[120]; unsigned lim_up = c0, lim_dn = c0; const char *wu = "", *wd = "";
    printf("sweep up\n");
    int nu = sweep(+1, c0, g0, up, 120, &lim_up, &wu);
    if (nu < 0) { servo_limp(); return 1; }
    printf("  -> upper limit %.0f us (%s)\n", c2us(lim_up), wu);
    if (move_slow(up[nu - 1].c, c0) != 0) { servo_limp(); return 1; }
    printf("sweep down\n");
    int nd = sweep(-1, c0, g0, dn, 120, &lim_dn, &wd);
    if (nd < 0) { servo_limp(); return 1; }
    printf("  -> lower limit %.0f us (%s)\n", c2us(lim_dn), wd);
    if (move_slow(dn[nd - 1].c, c0) != 0) { servo_limp(); return 1; }

    /* hinge axis h: gravity turns about it. Beam axis b = h x z (MPU taped flat on the beam), tilt = asin(g.b) */
    const float *ga = up[nu > 1 ? nu - 1 : 0].g, *gb = dn[nd > 1 ? nd - 1 : 0].g;
    float h[3] = { ga[1] * gb[2] - ga[2] * gb[1], ga[2] * gb[0] - ga[0] * gb[2], ga[0] * gb[1] - ga[1] * gb[0] };
    float hl = sqrtf(h[0] * h[0] + h[1] * h[1] + h[2] * h[2]);
    if (hl < 1e-3f) { printf("the beam did not tilt: cannot find its axis\n"); servo_limp(); return 1; }
    for (int k = 0; k < 3; k++) h[k] /= hl;
    float b[3] = { h[1], -h[0], 0 };                              /* h x (0,0,1) */
    float bl = sqrtf(b[0] * b[0] + b[1] * b[1]); b[0] /= bl; b[1] /= bl;
    float tu = rad2deg(asinf(up[nu > 1 ? nu - 1 : 0].g[0] * b[0] + up[nu > 1 ? nu - 1 : 0].g[1] * b[1]));
    if (tu < 0) { b[0] = -b[0]; b[1] = -b[1]; }                  /* positive tilt = larger pulse */
    printf("\nhinge axis (sensor frame) (%+.3f %+.3f %+.3f); beam axis (%+.3f %+.3f 0)\n", h[0], h[1], h[2], b[0], b[1]);
    printf("pulse_us,tilt_deg,hold_mA\n");
    float tmin = 1e9, tmax = -1e9, lvl = NAN; float pt = NAN; unsigned pc = 0;
    /* walk all samples in pulse order: down sweep reversed, centre, up sweep */
    for (int pass = 0; pass < 2; pass++) {
        int cnt = pass ? nu : nd;
        for (int j = 0; j < cnt; j++) {
            const step_t *s = pass ? &up[j] : &dn[cnt - 1 - j];
            float t = rad2deg(asinf(s->g[0] * b[0] + s->g[1] * b[1] + 0 * s->g[2]));
            if (s->c >= lim_dn && s->c <= lim_up) { tmin = fminf(tmin, t); tmax = fmaxf(tmax, t); }
            printf("%.0f,%.2f,%.0f\n", c2us(s->c), t, s->amp * 1000);
            if (!isnan(pt) && ((pt <= 0 && t >= 0) || (pt >= 0 && t <= 0)) && t != pt) lvl = c2us(pc) + (c2us(s->c) - c2us(pc)) * (0 - pt) / (t - pt);
            pt = t; pc = s->c;
            if (pass == 0 && j == cnt - 1) {                      /* the centre sits between the sweeps */
                float tc = rad2deg(asinf(g0[0] * b[0] + g0[1] * b[1]));
                printf("%.0f,%.2f,centre\n", c2us(c0), tc);
                if ((pt <= 0 && tc >= 0) || (pt >= 0 && tc <= 0)) if (tc != pt) lvl = c2us(pc) + (c2us(c0) - c2us(pc)) * (0 - pt) / (tc - pt);
                pt = tc; pc = c0;
            }
        }
    }
    printf("\nRESULT limits %.0f..%.0f us; beam tilt over that range %+.1f..%+.1f deg (MPU taken as flat on the beam); "
           "level (tilt 0) at %.0f us; peak current %.0f mA, min supply %.2f V%s\n", c2us(lim_dn), c2us(lim_up), tmin, tmax, lvl, s_peak_a * 1000, s_min_v,
           s_tripped ? " TRIPPED" : "");
    if (!isnan(lvl)) {
        if (move_slow(c0, us2c(lvl)) == 0) { wait_watch(1000, NULL); float g[3]; gravity(g);
            printf("at %.0f us: tilt %.2f deg\n", c2us(us2c(lvl)), rad2deg(asinf(g[0] * b[0] + g[1] * b[1]))); }
    }
    servo_limp();
    printf("servo limp\n");
    return 0;
}

/* Level the beam with the MPU as the only feedback, never outside servo_min_us..servo_max_us of reflex.conf.
 * Beam tilt here = rotation about the MPU's y axis (the hinge axis the sweeps found: (0, -0.99, -0.13)). */
static float beam_tilt(const float g[3]) { return rad2deg(atan2f(-g[0], g[2])); }
static int cmd_servo_level(int bus, const char *conf_path, double hold_s)
{
    rx_conf_t cf; char err[200];
    if (rx_conf_load(&cf, conf_path, err, sizeof err) != 0) { printf("%s\n", err); return 1; }
    if (!cf.servo_min_us) { printf("%s has no servo_min_us/servo_max_us: run servo-cal first\n", conf_path); return 1; }
    unsigned lo = us2c((float)cf.servo_min_us), hi = us2c((float)cf.servo_max_us);
    while (c2us(lo) < cf.servo_min_us) lo++;
    while (c2us(hi) > cf.servo_max_us) hi--;
    s_fd = rx_i2c_open(bus);
    if (s_fd < 0 || rx_gpio_open() != 0 || rx_mpu_init(s_fd, A_MPU1, NULL) != 0) { perror("rxhw servo-level"); return 1; }
    unsigned c = us2c(1500); if (c < lo) c = lo; if (c > hi) c = hi;
    printf("limits %.0f..%.0f us from %s; start %.0f us\n", c2us(lo), c2us(hi), conf_path, c2us(c));
    if (rx_gpio_pwm_setup(PIN_SERVO, 50, PWM_RANGE) != 0 || rx_gpio_pwm_duty(PIN_SERVO, c) != 0) { servo_limp(); perror("PWM"); return 1; }
    if (wait_watch(1500, NULL) != 0) { servo_limp(); return 1; }
    float g[3]; if (gravity(g) != 0) { servo_limp(); return 1; }
    float t = beam_tilt(g), best_t = t; unsigned best_c = c;
    printf("  %4.0f us  tilt %+6.2f deg\n", c2us(c), t); fflush(stdout);
    /* direction from the sweeps (step2-servo-cal-*): a larger pulse lifts the servo end and lowers the tilt.
     * A one-count probe is not used: its 0.1 deg effect is within the MPU noise. */
    int dir = 0;
    if (!dir) dir = (t > 0) ? 1 : -1;                              /* sweep data: a larger pulse lowers the tilt */
    for (;;) {
        unsigned nc = c + (unsigned)dir;
        if (nc < lo || nc > hi) { printf("  reached the saved limit %.0f us\n", c2us(c)); break; }
        c = nc; rx_gpio_pwm_duty(PIN_SERVO, c);
        if (wait_watch(300, NULL) != 0) { servo_limp(); return 1; }
        if (gravity(g) != 0) { servo_limp(); return 1; }
        t = beam_tilt(g);
        printf("  %4.0f us  tilt %+6.2f deg\n", c2us(c), t); fflush(stdout);
        if (fabsf(t) < fabsf(best_t)) { best_t = t; best_c = c; }
        if (fabsf(t) < 0.2f) break;
        if ((t > 0) != (best_t > 0) || fabsf(t) > fabsf(best_t) + 0.3f) break;   /* crossed level or got worse */
    }
    move_slow(c, best_c);
    if (wait_watch(800, NULL) != 0) { servo_limp(); return 1; }
    gravity(g);
    printf("\nRESULT most level reachable: %.0f us, tilt %+.2f deg (MPU, sign: + = hinge/sensor end higher)%s\n",
           c2us(best_c), beam_tilt(g), best_c == lo || best_c == hi ? " AT THE SAVED LIMIT: level is not reachable" : "");
    printf("holding %.0f s for a look...\n", hold_s); fflush(stdout);
    double t0 = now_s(); float pk = 0;
    while (now_s() - t0 < hold_s) { float a; if (wait_watch(500, &a) != 0) return 1; if (a > pk) pk = a; }
    servo_limp();
    printf("servo limp (holding current peak avg %.0f mA, peak %.0f mA, min supply %.2f V)\n", pk * 1000, s_peak_a * 1000, s_min_v);
    return 0;
}

/* ------------------------------------------------------------------ jog: smooth ramp, the person marks level
 * PWM at 5 us resolution (range 4000 at 50 Hz; the QSTI GPIO manager refuses ranges above 4095). Starts at the pulse the beam already rests at (from the MPU
 * and the sweep's tilt map) so there is no jump, then ramps at rate_us_s towards stop_us. The person presses
 * the e-stop button when the beam looks horizontal: the pulse freezes there and is held; a second press (or
 * SIGTERM) makes the servo limp. Stops by itself: 1.5 A, holding current > 0.8 A for 0.5 s, the beam not
 * rising any more (< 0.2 deg over the last 60 us of travel past the saved limit), the 2400 us ceiling. */
#define JOG_RANGE 4000
#define JOG_US_PER_COUNT (20000.0f / JOG_RANGE)
static volatile sig_atomic_t jog_quit;
static void on_term(int s) { (void)s; jog_quit = 1; }
static void jlimp(void) { rx_gpio_pwm_duty(PIN_SERVO, 0); }
static int cmd_jog(int bus, const char *conf_path, float stop_us, float rate)
{
    rx_conf_t cf; char err[200];
    if (rx_conf_load(&cf, conf_path, err, sizeof err) != 0) { printf("%s\n", err); return 1; }
    signal(SIGTERM, on_term); signal(SIGINT, on_term); signal(SIGHUP, SIG_IGN);
    s_fd = rx_i2c_open(bus);
    if (s_fd < 0 || rx_gpio_open() != 0 || rx_mpu_init(s_fd, A_MPU1, NULL) != 0) { perror("rxhw jog"); return 1; }
    rx_gpio_input(PIN_BUTTON, 1); rx_gpio_output(PIN_LED, 0);
    if (stop_us > 2400) stop_us = 2400;
    printf("waiting for the first button press to start (servo limp, LED blinking fast)...\n"); fflush(stdout);
    { int lowc = 0; double tw = now_s();
      while (!jog_quit && lowc < 2) {
          lowc = rx_gpio_read(PIN_BUTTON) == 0 ? lowc + 1 : 0;
          rx_gpio_write(PIN_LED, (int)(now_s() * 6) & 1);
          if (now_s() - tw > 900) { printf("no press in 15 min: nothing moved\n"); rx_gpio_write(PIN_LED, 0); return 1; }
          usleep(20000);
      }
      if (jog_quit) { rx_gpio_write(PIN_LED, 0); return 1; }
      while (rx_gpio_read(PIN_BUTTON) == 0) usleep(20000);     /* wait for release */
      usleep(200000); }
    float g[3]; if (gravity(g) != 0) return 1;
    float t_rest = beam_tilt(g);
    /* tilt map of the last sweep (step2-servo-cal-3): tilt = 17.0 deg at 1504 us, -6.51 deg per 1000 us */
    float us = 1504.0f + (17.0f - t_rest) / 0.00651f;
    if (us < cf.servo_min_us) us = (float)cf.servo_min_us;
    if (us > cf.servo_max_us) us = (float)cf.servo_max_us;
    printf("rest tilt %+.2f deg -> start at %.0f us (no jump); ramp %.0f us/s to %.0f us; press the button when horizontal\n",
           t_rest, us, rate, stop_us); fflush(stdout);
    if (rx_gpio_pwm_setup(PIN_SERVO, 50, JOG_RANGE) != 0 || rx_gpio_pwm_duty(PIN_SERVO, (unsigned)(us / JOG_US_PER_COUNT + 0.5f)) != 0) { jlimp(); perror("PWM"); return 1; }
    if (wait_watch(1200, NULL) != 0) return 1;
    if (gravity(g) != 0) { jlimp(); return 1; }
    float t_on = beam_tilt(g);
    printf("PWM on at %.0f us (5 us resolution): tilt %+.2f deg (rest %+.2f)\n", us, t_on, t_rest); fflush(stdout);
    if (fabsf(t_on - t_rest) > 2.0f) { jlimp(); printf("STOP: the beam moved %.1f deg when the PWM came on; the 1 us PWM setting is not what it seems\n", t_on - t_rest); return 1; }
    int marked = 0, low = 0, over = 0; double last_print = 0, t0 = now_s(); float mark_us = 0, mark_t = 0;
    float hist_us[64], hist_t[64]; int nh = 0;
    while (!jog_quit) {
        float a, v; if (rx_ina_read(s_fd, A_INA, &v, NULL, &a, NULL) != 0) { jlimp(); printf("STOP: INA219 read failed\n"); return 1; }
        a = fabsf(a); if (a > s_peak_a) s_peak_a = a; if (v < s_min_v) s_min_v = v;
        if (a > SERVO_TRIP_A) { jlimp(); printf("TRIP: %.2f A at %.0f us, servo limp\n", a, us); return 1; }
        over = a > 0.8f ? over + 1 : 0;
        if (over >= 25) { jlimp(); printf("STOP: holding current above 0.8 A for 0.5 s at %.0f us (binding?), servo limp\n", us); return 1; }
        int b = rx_gpio_read(PIN_BUTTON);
        low = b == 0 ? low + 1 : 0;
        if (low == 2) {                                         /* a press (debounced, 40 ms) */
            if (!marked) {
                marked = 1; mark_us = us;
                if (gravity(g) == 0) mark_t = beam_tilt(g);
                printf("MARK: horizontal at %.0f us, MPU tilt there %+.2f deg (holding; press again to release)\n", mark_us, mark_t); fflush(stdout);
            } else { printf("second press: release\n"); break; }
        }
        if (!marked && us < stop_us) {
            us += rate * 0.02f; if (us > stop_us) us = stop_us;
            rx_gpio_pwm_duty(PIN_SERVO, (unsigned)(us / JOG_US_PER_COUNT + 0.5f));
        }
        rx_gpio_write(PIN_LED, marked ? ((int)(now_s() * 2) & 1) : 1);
        if (now_s() - last_print >= 0.5) {
            last_print = now_s();
            rx_mpu_t m; float t = NAN; if (rx_mpu_read(s_fd, A_MPU1, &m) == 0) { float gg[3] = { m.ax, m.ay, m.az }; t = beam_tilt(gg); }
            printf("t=%5.1f  %6.1f us  tilt %+6.2f deg  %4.0f mA  %.2f V%s\n", now_s() - t0, us, t, a * 1000, v, marked ? "  HOLDING" : ""); fflush(stdout);
            /* not rising any more past the saved limit: the tilt barely changed over the last 60 us of travel */
            if (!marked && !isnan(t)) {
                if (nh < 64) { hist_us[nh] = us; hist_t[nh] = t; nh++; } else { memmove(hist_us, hist_us + 1, 63 * sizeof *hist_us); memmove(hist_t, hist_t + 1, 63 * sizeof *hist_t); hist_us[63] = us; hist_t[63] = t; }
                for (int j = 0; j < nh; j++) if (us - hist_us[j] >= 60 && us > cf.servo_max_us) {
                    if (hist_t[j] - t < 0.2f) { printf("STOP: the beam stopped rising (%.2f deg over %.0f us) at %.0f us; holding here\n", hist_t[j] - t, us - hist_us[j], us); fflush(stdout); stop_us = us; }
                    break;
                }
            }
        }
        usleep(20000);
    }
    jlimp(); rx_gpio_write(PIN_LED, 0);
    printf("servo limp. %s; peak %.0f mA, min supply %.2f V\n", marked ? "marked" : "no mark", s_peak_a * 1000, s_min_v);
    if (marked) printf("RESULT level %.0f us, MPU tilt at level %+.2f deg\n", mark_us, mark_t);
    return 0;
}

/* Read-only: the beam is level and the ball centred (a person says so). Mean MPU over 3 s (beam tilt, roll)
 * and the HC-SR04 distance to the ball. Nothing moves. */
static int cmd_at_level(int bus)
{
    s_fd = rx_i2c_open(bus);
    if (s_fd < 0 || rx_gpio_open() != 0 || rx_mpu_init(s_fd, A_MPU1, NULL) != 0) { perror("rxhw at-level"); return 1; }
    double st = 0, st2 = 0, sr = 0; int n = 0;
    for (int i = 0; i < 150; i++) {
        rx_mpu_t m;
        if (rx_mpu_read(s_fd, A_MPU1, &m) == 0) {
            float g[3] = { m.ax, m.ay, m.az }, t = beam_tilt(g);
            st += t; st2 += t * t; sr += rad2deg(atan2f(m.ay, m.az)); n++;
        }
        usleep(20000);
    }
    if (!n) { printf("no MPU data\n"); return 1; }
    double mt = st / n, sd = sqrt(fmax(0, st2 / n - mt * mt));
    printf("MPU over %d samples: beam tilt reading %+.2f deg (sd %.2f), roll %+.2f deg\n", n, mt, sd, sr / n);
    rx_sonar_t so; float d[40]; int k = 0;
    if (rx_sonar_open(&so, PIN_TRIG, PIN_ECHO) == 0)
        for (int i = 0; i < 40; i++) { float mm = rx_sonar_measure(&so, 40, NULL); if (mm > 0) d[k++] = mm; usleep(50000); }
    if (k) { qsort(d, (size_t)k, sizeof d[0], cmp_f);
             printf("HC-SR04 to the ball: median %.1f mm (p10 %.1f, p90 %.1f), %d/40 echoes\n", d[k / 2], d[k / 10], d[k - 1 - k / 10], k); }
    else printf("HC-SR04: no echoes\n");
    printf("RESULT mpu_level_deg %+.2f  ball_centre_mm %.1f\n", mt, k ? d[k / 2] : NAN);
    return 0;
}

/* Smooth range finder for a new linkage: from start_us (where the servo already is, so no jump) ramp down at
 * rate us/s until the beam stops moving (< 0.25 deg over the last 100 us), the current trips (1.5 A, or > 0.8 A
 * for 0.5 s), |tilt from level| > 12 deg or 500 us; back through start up to the same stops or 2500 us; back to
 * start, hold 3 s, limp. Tilt from level = MPU beam tilt - level_off (rxhw at-level). Prints pulse,tilt rows. */
static int ramp_to(float *us, float target, float rate, float level_off, float stop_lo, float stop_hi, int stall_check,
                   float *edge_us, const char **why, FILE *csv)
{
    float hist_us[200], hist_t[200]; int nh = 0; double lastp = 0;
    *why = "target";
    while (fabsf(*us - target) > 0.01f) {
        if (jog_quit) { *why = "stopped"; return -1; }
        float step = rate * 0.02f, d = target - *us;
        *us += fabsf(d) < step ? d : (d > 0 ? step : -step);
        rx_gpio_pwm_duty(PIN_SERVO, (unsigned)(*us / JOG_US_PER_COUNT + 0.5f));
        float a, v;
        if (rx_ina_read(s_fd, A_INA, &v, NULL, &a, NULL) != 0) { jlimp(); *why = "INA219 read failed"; return -1; }
        a = fabsf(a); if (a > s_peak_a) s_peak_a = a; if (v < s_min_v) s_min_v = v;
        static int over; over = a > 0.8f ? over + 1 : 0;
        if (a > SERVO_TRIP_A) { jlimp(); printf("TRIP %.2f A at %.0f us\n", a, *us); *why = "current trip"; return -1; }
        if (over >= 25) { *edge_us = *us; *why = "holding current > 0.8 A for 0.5 s (binding)"; return 1; }
        rx_mpu_t m;
        if (rx_mpu_read(s_fd, A_MPU1, &m) == 0) {
            float g[3] = { m.ax, m.ay, m.az }, t = beam_tilt(g) - level_off;
            if (csv) fprintf(csv, "%.1f,%.3f,%.0f,%.3f\n", *us, t, a * 1000, v);
            if (now_s() - lastp > 0.5) { lastp = now_s(); printf("  %6.1f us  tilt from level %+6.2f deg  %4.0f mA  %.2f V\n", *us, t, a * 1000, v); fflush(stdout); }
            if (fabsf(t) > 12.0f) { *edge_us = *us; *why = "tilt past 12 deg"; return 1; }
            if (nh < 200) { hist_us[nh] = *us; hist_t[nh] = t; nh++; }
            else { memmove(hist_us, hist_us + 1, 199 * sizeof *hist_us); memmove(hist_t, hist_t + 1, 199 * sizeof *hist_t); hist_us[199] = *us; hist_t[199] = t; }
            static float t_first; if (nh == 1) t_first = t;
            int moved = fabsf(t - t_first) > 0.5f;               /* only after the beam has really started moving */
            if (stall_check && moved && *us > stop_lo && *us < stop_hi)
                for (int j = 0; j < nh; j++) if (fabsf(*us - hist_us[j]) <= 100.0f) {
                    if (fabsf(*us - hist_us[j]) >= 99.0f) {
                        /* MPU noise is ~0.1 deg: average 5 samples each end */
                        float a0 = 0, a1 = 0; int k0 = 0, k1 = 0;
                        for (int q = j; q < j + 5 && q < nh; q++) { a0 += hist_t[q]; k0++; }
                        for (int q = nh - 5; q < nh; q++) if (q >= 0) { a1 += hist_t[q]; k1++; }
                        if (fabsf(a1 / k1 - a0 / k0) < 0.25f) { *edge_us = *us; *why = "beam stopped moving"; return 1; }
                    }
                    break;
                }
        }
        usleep(20000);
    }
    return 0;
}

static int cmd_range(int bus, float start, float level_off, float rate)
{
    signal(SIGTERM, on_term); signal(SIGINT, on_term); signal(SIGHUP, SIG_IGN);
    s_fd = rx_i2c_open(bus);
    if (s_fd < 0 || rx_gpio_open() != 0 || rx_mpu_init(s_fd, A_MPU1, NULL) != 0) { perror("rxhw range"); return 1; }
    float g[3]; if (gravity(g) != 0) return 1;
    float t_rest = beam_tilt(g) - level_off;
    if (rx_gpio_pwm_setup(PIN_SERVO, 50, JOG_RANGE) != 0 || rx_gpio_pwm_duty(PIN_SERVO, (unsigned)(start / JOG_US_PER_COUNT + 0.5f)) != 0) { jlimp(); perror("PWM"); return 1; }
    if (wait_watch(1000, NULL) != 0) return 1;
    if (gravity(g) != 0) { jlimp(); return 1; }
    float t_on = beam_tilt(g) - level_off;
    printf("PWM on at %.0f us: tilt from level %+.2f deg (limp %+.2f)\n", start, t_on, t_rest); fflush(stdout);
    if (fabsf(t_on - t_rest) > 2.0f) { jlimp(); printf("STOP: the beam jumped %.1f deg when the PWM came on: the servo was not at %.0f us\n", t_on - t_rest, start); return 1; }
    FILE *csv = fopen("range.csv", "w"); if (csv) fprintf(csv, "pulse_us,tilt_from_level_deg,current_ma,supply_v\n");
    float us = start, lo = 500, hi = 2500; const char *wl = "", *wh = "";
    printf("down\n");
    int r = ramp_to(&us, 500, rate, level_off, 500, 2500, 1, &lo, &wl, csv);
    if (r < 0) { jlimp(); if (csv) fclose(csv); return 1; }
    if (r == 0) lo = 500;
    printf("  -> low end %.0f us (%s)\nback up\n", lo, wl);
    if (ramp_to(&us, start, rate, level_off, 0, 0, 0, &hi, &wh, csv) < 0) { jlimp(); if (csv) fclose(csv); return 1; }
    r = ramp_to(&us, 2500, rate, level_off, 500, 2500, 1, &hi, &wh, csv);
    if (r < 0) { jlimp(); if (csv) fclose(csv); return 1; }
    if (r == 0) hi = 2500;
    printf("  -> high end %.0f us (%s)\nback to %.0f us\n", hi, wh, start);
    const char *w2; float dummy;
    if (ramp_to(&us, start, rate, level_off, 0, 0, 0, &dummy, &w2, csv) < 0) { jlimp(); if (csv) fclose(csv); return 1; }
    if (csv) fclose(csv);
    wait_watch(3000, NULL);
    if (gravity(g) == 0) printf("at %.0f us again: tilt from level %+.2f deg\n", start, beam_tilt(g) - level_off);
    jlimp();
    printf("RESULT low end %.0f us (%s), high end %.0f us (%s); peak %.0f mA, min supply %.2f V; rows in range.csv\n",
           lo, wl, hi, wh, s_peak_a * 1000, s_min_v);
    return 0;
}

/* Ramp smoothly from from_us (where the servo is) to to_us and hold there, watching the current, until the
 * button is pressed, SIGTERM, or max_s. For re-attaching a linkage with the servo at a known position. */
static int cmd_hold(int bus, float from, float to, float rate, double max_s)
{
    signal(SIGTERM, on_term); signal(SIGINT, on_term); signal(SIGHUP, SIG_IGN);
    s_fd = rx_i2c_open(bus);
    if (s_fd < 0 || rx_gpio_open() != 0 || rx_mpu_init(s_fd, A_MPU1, NULL) != 0) { perror("rxhw hold"); return 1; }
    if (from < 500 || from > 2500 || to < 500 || to > 2500) { printf("pulses must be 500..2500 us\n"); return 1; }
    rx_gpio_input(PIN_BUTTON, 1); rx_gpio_output(PIN_LED, 0);
    if (rx_gpio_pwm_setup(PIN_SERVO, 50, JOG_RANGE) != 0 || rx_gpio_pwm_duty(PIN_SERVO, (unsigned)(from / JOG_US_PER_COUNT + 0.5f)) != 0) { jlimp(); perror("PWM"); return 1; }
    if (wait_watch(800, NULL) != 0) return 1;
    float us = from, e; const char *why;
    printf("ramp %.0f -> %.0f us at %.0f us/s\n", from, to, rate); fflush(stdout);
    if (ramp_to(&us, to, rate, 0, 0, 0, 0, &e, &why, NULL) < 0) { jlimp(); printf("stopped: %s\n", why); return 1; }
    printf("HOLDING at %.0f us: press the button to release (or %.0f min)\n", us, max_s / 60); fflush(stdout);
    double t0 = now_s(); int low = 0;
    while (!jog_quit && now_s() - t0 < max_s) {
        if (wait_watch(100, NULL) != 0) return 1;
        low = rx_gpio_read(PIN_BUTTON) == 0 ? low + 1 : 0;
        if (low >= 1) { printf("button: release\n"); break; }
        rx_gpio_write(PIN_LED, (int)(now_s() * 2) & 1);
    }
    jlimp(); rx_gpio_write(PIN_LED, 0);
    printf("servo limp (peak %.0f mA, min supply %.2f V)\n", s_peak_a * 1000, s_min_v);
    return 0;
}

/* Ball sensing, driven by the button so the person sets the pace. For each position: LED blinks fast = place the
 * ball there and press; LED steady = sampling (3 s, one ping per period_ms). Then the noise run: ball left still
 * at the centre, press, 10 s. Servo untouched. Raw readings go to ballmap.csv (phase,t_s,mm; -1 = no echo). */
static void wait_press(void)
{
    int low = 0;
    while (rx_gpio_read(PIN_BUTTON) == 0) usleep(20000);           /* released first */
    while (low < 2) { low = rx_gpio_read(PIN_BUTTON) == 0 ? low + 1 : 0; rx_gpio_write(PIN_LED, (int)(now_s() * 6) & 1); usleep(20000); }
    rx_gpio_write(PIN_LED, 1);
    while (rx_gpio_read(PIN_BUTTON) == 0) usleep(20000);
    usleep(300000);                                               /* hand away from the sensor */
}
static void sonar_stats(const char *name, const float *v, int n, double secs, float near_mm)
{
    float d[2000]; int k = 0, none = 0, far = 0;
    for (int i = 0; i < n; i++) { if (v[i] < 0) none++; else if (v[i] > near_mm) far++; else if (k < 2000) d[k++] = v[i]; }
    printf("%-10s %d pings in %.1f s (%.1f Hz): ball-range (<%.0f mm) %d (%.0f %%), farther %d, no echo %d\n",
           name, n, secs, n / secs, near_mm, k, 100.0 * k / n, far, none);
    if (!k) return;
    qsort(d, (size_t)k, sizeof d[0], cmp_f);
    double m = 0, v2 = 0; for (int i = 0; i < k; i++) m += d[i];
    m /= k;
    for (int i = 0; i < k; i++) v2 += (d[i] - m) * (d[i] - m);
    printf("           median %.1f mm, mean %.1f, sd %.1f, p10 %.1f, p90 %.1f, min %.1f, max %.1f\n",
           d[k / 2], m, sqrt(v2 / k), d[k / 10], d[k - 1 - k / 10], d[0], d[k - 1]);
    int h[40] = { 0 }; for (int i = 0; i < k; i++) { int b = (int)(d[i] / 10); if (b >= 0 && b < 40) h[b]++; }
    printf("           histogram (10 mm bins):"); for (int b = 0; b < 40; b++) if (h[b]) printf(" %d:%d", b * 10, h[b]);
    printf("\n"); fflush(stdout);
}
static int cmd_ballmap(int period_ms, float near_mm)
{
    if (rx_gpio_open() != 0) { perror("rxhw ballmap"); return 1; }
    rx_gpio_input(PIN_BUTTON, 1); rx_gpio_output(PIN_LED, 0);
    rx_sonar_t so; if (rx_sonar_open(&so, PIN_TRIG, PIN_ECHO) != 0) { perror("sonar"); return 1; }
    FILE *csv = fopen("ballmap.csv", "w"); if (csv) fprintf(csv, "phase,t_s,mm\n");
    const char *ph[] = { "hinge-stop", "centre", "far-stop", "still" };
    const double dur[] = { 3, 3, 3, 10 };
    static float v[2000];
    for (int p = 0; p < 4; p++) {
        printf("\n[%s] %s, then press the button\n", ph[p], p < 3 ? "hold the ball there" : "leave the ball still at the centre, hands away");
        fflush(stdout);
        wait_press();
        int n = 0; double t0 = now_s(), next = t0;
        while (now_s() - t0 < dur[p] && n < 2000) {
            float mm = rx_sonar_measure(&so, 40, NULL);
            v[n++] = mm;
            if (csv) fprintf(csv, "%s,%.3f,%.1f\n", ph[p], now_s() - t0, mm);
            next += period_ms / 1000.0;
            double w = next - now_s(); if (w > 0) usleep((useconds_t)(w * 1e6));
        }
        rx_gpio_write(PIN_LED, 0);
        sonar_stats(ph[p], v, n, now_s() - t0, near_mm);
    }
    if (csv) fclose(csv);
    printf("\ndone (raw readings in ballmap.csv)\n");
    return 0;
}

/* Live aiming aid: ping continuously for secs; the LED is on while echoes come from closer than near_mm (the ball),
 * off when they come from farther (the room). Prints a summary every second. Read-only. */
static int cmd_aim(double secs, float near_mm)
{
    if (rx_gpio_open() != 0) { perror("rxhw aim"); return 1; }
    rx_gpio_output(PIN_LED, 0);
    rx_sonar_t so; if (rx_sonar_open(&so, PIN_TRIG, PIN_ECHO) != 0) { perror("sonar"); return 1; }
    double t0 = now_s(), tp = t0; int n = 0, hit = 0, tot = 0, toth = 0; float last = -1, sum = 0;
    while (now_s() - t0 < secs) {
        float mm = rx_sonar_measure(&so, 40, NULL);
        int h = mm > 0 && mm < near_mm;
        rx_gpio_write(PIN_LED, h);
        n++; tot++; if (h) { hit++; toth++; sum += mm; last = mm; }
        if (now_s() - tp >= 1.0) {
            printf("t=%4.0f s  ball seen in %3d %% of pings  %s\n", now_s() - t0, 100 * hit / (n ? n : 1),
                   hit ? "" : "(only the room)");
            if (hit) printf("           mean ball distance %.0f mm (last %.0f)\n", sum / hit, last);
            fflush(stdout); tp = now_s(); n = hit = 0; sum = 0;
        }
        usleep(30000);
    }
    rx_gpio_write(PIN_LED, 0);
    printf("overall: ball seen in %d of %d pings (%.0f %%)\n", toth, tot, 100.0 * toth / (tot ? tot : 1));
    return 0;
}

/* The guardian's sensing chain on the real sensor: one ping per 20 ms tick, rx_sonar filter, rx_est g-h
 * estimator (est_tau as given). Prints every 0.5 s; summary: rate, accepted/gated/spikes, position and
 * velocity spread. Read-only. Rows in track.csv (t, raw, filtered or -1, x_est, v_est). */
static int cmd_track(double secs, float est_tau)
{
    if (rx_gpio_open() != 0) { perror("rxhw track"); return 1; }
    rx_sonar_t so; if (rx_sonar_open(&so, PIN_TRIG, PIN_ECHO) != 0) { perror("sonar"); return 1; }
    rx_sonar_cfg_t sc; rx_sonar_cfg_default(&sc);
    rx_sonar_filt_t f; rx_sonar_init(&f, &sc);
    rx_cfg_t c; rx_cfg_default(&c, 50.0f); c.est_tau = est_tau;
    rx_est_t e = { 0, 0, 0 };
    FILE *csv = fopen("track.csv", "w"); if (csv) fprintf(csv, "t_s,raw_mm,filt_mm,x_mm,v_mm_s\n");
    double t0 = now_s(), next = t0, tp = t0, sx = 0, sx2 = 0, sv = 0, sv2 = 0, late = 0; int n = 0, ns = 0;
    while (now_s() - t0 < secs) {
        float raw = rx_sonar_measure(&so, 15, NULL), z;          /* 15 ms: anything later is not the ball */
        int ok = rx_sonar_step(&f, raw, 0.02f, &z);
        if (ok) rx_est_update(&e, &c, z * 1e-3f); else rx_est_predict(&e, &c);
        n++;
        if (e.init) { sx += e.x * 1e3; sx2 += e.x * 1e3 * e.x * 1e3; sv += e.v * 1e3; sv2 += e.v * 1e3 * e.v * 1e3; ns++; }
        if (csv) fprintf(csv, "%.3f,%.1f,%.1f,%.2f,%.1f\n", now_s() - t0, raw, ok ? z : -1.0f, e.x * 1e3, e.v * 1e3);
        if (now_s() - tp >= 0.5) { tp = now_s(); printf("t=%5.1f  raw %6.1f  ball %6.1f mm  v %+6.1f mm/s\n", now_s() - t0, raw, e.x * 1e3, e.v * 1e3); fflush(stdout); }
        next += 0.02; double w = next - now_s();
        if (w > 0) usleep((useconds_t)(w * 1e6)); else late++;
    }
    if (csv) fclose(csv);
    double dt = now_s() - t0, mx = sx / ns, mv = sv / ns;
    printf("\n%d ticks in %.1f s (%.1f Hz, %.0f late): accepted %u, gated %u, spikes refused %u, re-acquired %u\n",
           n, dt, n / dt, late, f.n_ok, f.n_gated, f.n_spikes, f.n_reacq);
    printf("estimate: x mean %.1f mm sd %.2f mm; v mean %+.1f mm/s sd %.1f mm/s (est_tau %.3f s)\n",
           mx, sqrt(fmax(0, sx2 / ns - mx * mx)), mv, sqrt(fmax(0, sv2 / ns - mv * mv)), est_tau);
    return 0;
}

static int cmd_led(int n)
{
    if (rx_gpio_output(PIN_LED, 0) != 0) { perror("rxhw led"); return 1; }
    for (int i = 0; i < n; i++) { rx_gpio_write(PIN_LED, 1); usleep(300000); rx_gpio_write(PIN_LED, 0); usleep(300000); }
    printf("LED blinked %d times\n", n);
    return 0;
}

static int cmd_sonar(int n, int ms)
{
    rx_sonar_t s;
    if (rx_sonar_open(&s, PIN_TRIG, PIN_ECHO) != 0) { perror("rxhw sonar"); return 1; }
    float *d = calloc((size_t)n, sizeof *d); int k = 0; double t0 = now_s();
    for (int i = 0; i < n; i++) { float mm = rx_sonar_measure(&s, 40, NULL); if (mm > 0) d[k++] = mm; usleep((useconds_t)ms * 1000); }
    double dt = now_s() - t0;
    if (!k) { printf("no echoes in %d tries\n", n); return 1; }
    qsort(d, (size_t)k, sizeof *d, cmp_f);
    double mean = 0, var = 0;
    for (int i = 0; i < k; i++) mean += d[i];
    mean /= k;
    for (int i = 0; i < k; i++) var += (d[i] - mean) * (d[i] - mean);
    var /= k;
    printf("%d/%d echoes in %.1f s (%.1f Hz): median %.1f mm, mean %.1f, sd %.1f, min %.1f, p5 %.1f, p95 %.1f, max %.1f\n",
           k, n, dt, n / dt, d[k / 2], mean, sqrt(var), d[0], d[k / 20], d[k - 1 - k / 20], d[k - 1]);
    return 0;
}

int main(int argc, char **argv)
{
    int bus = -1;
    for (int i = 1; i < argc; i++) if (!strcmp(argv[i], "--mpu2")) { want_mpu2 = 1; memmove(&argv[i], &argv[i + 1], (size_t)(argc - i) * sizeof *argv); argc--; break; }
    for (int i = 1; i + 1 < argc; i++) if (!strcmp(argv[i], "--bus")) { bus = atoi(argv[i + 1]); memmove(&argv[i], &argv[i + 2], (size_t)(argc - i - 1) * sizeof *argv); argc -= 2; break; }
    if (argc < 2) { fprintf(stderr, "usage: rxhw scan | watch <secs> | led <n> | sonar <n> [ms]   [--bus N]\n"); return 2; }
    bus = pick_bus(bus);
    if (!strcmp(argv[1], "scan")) return cmd_scan(bus);
    if (!strcmp(argv[1], "watch") && argc >= 3) return cmd_watch(bus, atof(argv[2]));
    if (!strcmp(argv[1], "mpu") && argc >= 3) return cmd_mpu(bus, (int)strtol(argv[2], NULL, 0));
    if (!strcmp(argv[1], "servo-cal")) {                       /* servo-cal [lo_us hi_us], never wider than 500..2500 */
        if (argc >= 4) { s_lo_us = fmaxf(500, (float)atof(argv[2])); s_hi_us = fminf(2500, (float)atof(argv[3])); }
        return cmd_servo_cal(bus);
    }
    if (!strcmp(argv[1], "servo-level")) return cmd_servo_level(bus, argc >= 3 ? argv[2] : "reflex.conf", argc >= 4 ? atof(argv[3]) : 30);
    if (!strcmp(argv[1], "pwm-probe")) {                       /* which PWM ranges the manager takes; duty stays 0 (no pulses) */
        static const unsigned r[] = { 20000, 4096, 4095, 4000, 2048, 1024 };   /* 2026-11-25: up to 4095 accepted */
        for (unsigned i = 0; i < sizeof r / sizeof r[0]; i++) {
            int e = rx_gpio_pwm_setup(PIN_SERVO, 50, r[i]); int en = errno;
            rx_gpio_pwm_duty(PIN_SERVO, 0);
            printf("range %5u: %s (%.2f us per count)\n", r[i], e ? strerror(en) : "accepted", 20000.0 / r[i]);
        }
        return 0;
    }
    if (!strcmp(argv[1], "range") && argc >= 4) return cmd_range(bus, (float)atof(argv[2]), (float)atof(argv[3]), argc >= 5 ? (float)atof(argv[4]) : 100);
    if (!strcmp(argv[1], "hold") && argc >= 4) return cmd_hold(bus, (float)atof(argv[2]), (float)atof(argv[3]), argc >= 5 ? (float)atof(argv[4]) : 100, argc >= 6 ? atof(argv[5]) : 1200);
    if (!strcmp(argv[1], "ballmap")) return cmd_ballmap(argc >= 3 ? atoi(argv[2]) : 30, argc >= 4 ? (float)atof(argv[3]) : 350);
    if (!strcmp(argv[1], "aim")) return cmd_aim(argc >= 3 ? atof(argv[2]) : 120, argc >= 4 ? (float)atof(argv[3]) : 300);
    if (!strcmp(argv[1], "track")) return cmd_track(argc >= 3 ? atof(argv[2]) : 20, argc >= 4 ? (float)atof(argv[3]) : 0.04f);
    if (!strcmp(argv[1], "at-level")) return cmd_at_level(bus);
    if (!strcmp(argv[1], "jog")) return cmd_jog(bus, argc >= 3 ? argv[2] : "reflex.conf", argc >= 4 ? (float)atof(argv[3]) : 2300, argc >= 5 ? (float)atof(argv[4]) : 20);
    if (!strcmp(argv[1], "servo-off")) { if (rx_gpio_pwm_duty(PIN_SERVO, 0) != 0) { perror("servo-off"); return 1; } printf("servo limp\n"); return 0; }
    if (!strcmp(argv[1], "led")) return cmd_led(argc >= 3 ? atoi(argv[2]) : 5);
    if (!strcmp(argv[1], "sonar") && argc >= 3) return cmd_sonar(atoi(argv[2]), argc >= 4 ? atoi(argv[3]) : 50);
    fprintf(stderr, "unknown command\n");
    return 2;
}
