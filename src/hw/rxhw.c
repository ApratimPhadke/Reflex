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
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "rx_rpi.h"
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
    if (!strcmp(argv[1], "servo-off")) { if (rx_gpio_pwm_duty(PIN_SERVO, 0) != 0) { perror("servo-off"); return 1; } printf("servo limp\n"); return 0; }
    if (!strcmp(argv[1], "led")) return cmd_led(argc >= 3 ? atoi(argv[2]) : 5);
    if (!strcmp(argv[1], "sonar") && argc >= 3) return cmd_sonar(atoi(argv[2]), argc >= 4 ? atoi(argv[3]) : 50);
    fprintf(stderr, "unknown command\n");
    return 2;
}
