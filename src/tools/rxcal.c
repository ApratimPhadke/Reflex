/* rxcal - calibration wizard for the Reflex rig. Writes reflex.conf (src/core/rx_conf.h).
 *
 *   ./rxcal [--dev /dev/serusb1] [--conf reflex.conf] [--auto]
 *
 * One screen per step: 1 link, 2 level, 3 direction, 4 centre, 5 range, 6 angle scale,
 * 7 validation (20 s closed loop on the verified fallback controller), 8 write the file.
 * Before every step that moves the servo it says "keep hands clear" and waits for Enter.
 * Stopping it (Ctrl-C, q, a crash) stops the servo commands; the I/O node then levels the beam
 * by itself within 150 ms.
 *
 * --auto runs every step without a person, on an EMULATED node only (the node must identify as
 * emulated in its "I" line): the true beam angle ("T" lines) stands in for the inclinometer and the
 * emulator's "E" commands place the ball. This is how rxcal is tested (make test-cal); it refuses
 * to run --auto on real hardware.
 * SPDX-License-Identifier: MIT */
#include <errno.h>
#include <fcntl.h>
#include <libgen.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include "rx_conf.h"
#include "rx_core.h"
#include "rx_link.h"
#include "rx_shm.h"

static int auto_mode;
static volatile sig_atomic_t g_stop;

static struct {
    int      fd;
    rx_link_t link;
    int      hold, pulse;               /* hold = keep sending "S pulse" so the watchdog stays quiet */
    uint64_t last_s_ms;
    /* sample accumulator */
    int      acc;
    long     n, bang;
    double   sum;
    int      mn, mx;
    long     total;                     /* samples seen since start */
    int      last_valid, last_raw;
} io = { .fd = -1 };

static uint64_t now_ms(void)
{
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u;
}

static void send_line(const char *s)
{
    if (io.fd < 0) return;
    size_t n = strlen(s);
    if (write(io.fd, s, n) != (ssize_t)n && errno != EAGAIN) { /* the next pump notices a dead link */ }
}

static void send_pulse(int p)
{
    char b[24];
    rx_link_cmd_servo(b, sizeof b, p, RX_PULSE_MIN, RX_PULSE_MAX);
    send_line(b);
}

static void hold_at(int p) { io.pulse = p; io.hold = 1; send_pulse(p); io.last_s_ms = now_ms(); }
static void release(void) { io.hold = 0; }      /* stop commanding: the node levels by itself */

static void acc_start(void) { io.acc = 1; io.n = io.bang = 0; io.sum = 0; io.mn = 1 << 30; io.mx = -(1 << 30); }
static void acc_stop(void) { io.acc = 0; }

/* Read the link for ms milliseconds (or until a new sample if until_sample), keep the servo
 * command alive, collect samples. Returns the number of samples seen; -1 if the link died. */
static int pump(int ms, int until_sample)
{
    uint64_t end = now_ms() + (uint64_t)ms;
    int got = 0;
    while (!g_stop) {
        uint64_t t = now_ms();
        if (io.hold && t - io.last_s_ms >= 40) { send_pulse(io.pulse); io.last_s_ms = t; }
        if (t >= end) break;
        struct pollfd p = { io.fd, POLLIN, 0 };
        int r = poll(&p, 1, 10);
        if (r < 0 && errno != EINTR) return -1;
        if (r > 0) {
            if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) return -1;
            char buf[256];
            ssize_t n = read(io.fd, buf, sizeof buf);
            if (n == 0 || (n < 0 && errno != EAGAIN && errno != EINTR)) return -1;
            for (ssize_t i = 0; i < n; i++) {
                if (rx_link_feed(&io.link, buf + i, 1) == 0) continue;
                got++; io.total++;
                io.last_valid = io.link.valid; io.last_raw = io.link.raw_mm;
                if (io.acc) {
                    if (!io.link.valid) io.bang++;
                    else {
                        io.n++; io.sum += io.link.raw_mm;
                        if (io.link.raw_mm < io.mn) io.mn = io.link.raw_mm;
                        if (io.link.raw_mm > io.mx) io.mx = io.link.raw_mm;
                    }
                }
            }
            if (until_sample && got) break;
        }
    }
    return got;
}

static void die(const char *fmt, ...) __attribute__((format(printf, 1, 2), noreturn));
static void die(const char *fmt, ...)
{
    va_list ap; va_start(ap, fmt);
    release();
    fprintf(stderr, "\nrxcal: "); vfprintf(stderr, fmt, ap); fprintf(stderr, "\n");
    va_end(ap);
    exit(1);
}

static void must_pump(int ms) { if (pump(ms, 0) < 0) die("lost the I/O node link"); }

/* ------------------------------------------------------------------ talking to the person */
static struct termios tty_saved; static int tty_raw;
static void tty_restore(void) { if (tty_raw) { tcsetattr(0, TCSANOW, &tty_saved); tty_raw = 0; } }
static void tty_set_raw(void)
{
    if (!isatty(0) || tty_raw) return;
    tcgetattr(0, &tty_saved);
    struct termios t = tty_saved;
    t.c_lflag &= ~(ICANON | ECHO); t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &t); tty_raw = 1;
}

static void header(int n, const char *title)
{
    printf("\n==================================================================\n");
    printf(" Step %d/8: %s\n", n, title);
    printf("==================================================================\n");
}

static void wait_enter(const char *what)
{
    if (auto_mode) { printf("%s  [auto]\n", what); return; }
    printf("%s\nPress Enter to continue (Ctrl-C to abort) ", what); fflush(stdout);
    char line[64];
    /* keep the link and the servo command alive while the person reads */
    for (;;) {
        struct pollfd p = { 0, POLLIN, 0 };
        if (pump(50, 0) < 0) die("lost the I/O node link");
        if (g_stop) die("aborted");
        if (poll(&p, 1, 0) > 0) { if (!fgets(line, sizeof line, stdin)) die("end of input"); break; }
    }
}

static void hands_clear(void) { wait_enter(">>> KEEP HANDS CLEAR of the beam and the servo linkage. The servo will move."); }

static int ask_yes(const char *q)
{
    if (auto_mode) { printf("%s  [auto: yes]\n", q); return 1; }
    for (;;) {
        printf("%s [y/n] ", q); fflush(stdout);
        char line[64];
        for (;;) {
            struct pollfd p = { 0, POLLIN, 0 };
            if (pump(50, 0) < 0) die("lost the I/O node link");
            if (g_stop) die("aborted");
            if (poll(&p, 1, 0) > 0) { if (!fgets(line, sizeof line, stdin)) die("end of input"); break; }
        }
        if (line[0] == 'y' || line[0] == 'Y') return 1;
        if (line[0] == 'n' || line[0] == 'N') return 0;
    }
}

static double ask_number(const char *q)
{
    for (;;) {
        printf("%s ", q); fflush(stdout);
        char line[64];
        for (;;) {
            struct pollfd p = { 0, POLLIN, 0 };
            if (pump(50, 0) < 0) die("lost the I/O node link");
            if (g_stop) die("aborted");
            if (poll(&p, 1, 0) > 0) { if (!fgets(line, sizeof line, stdin)) die("end of input"); break; }
        }
        char *e; double v = strtod(line, &e);
        while (*e == ' ' || *e == '\n' || *e == '\r') e++;
        if (e != line && !*e && isfinite(v)) return v;
        printf("  please type a number, for example -11.3\n");
    }
}

/* the true beam angle from an emulated node, waiting for a settled value */
static double truth_mrad(void)
{
    if (!io.link.have_truth) must_pump(200);
    if (!io.link.have_truth) die("emulated node sends no \"T\" lines");
    return io.link.theta_mrad;
}

/* average raw reading over ms; returns NAN if fewer than half the samples were valid */
static double average(int ms, long *n_out, long *bang_out)
{
    acc_start(); must_pump(ms); acc_stop();
    if (n_out) *n_out = io.n;
    if (bang_out) *bang_out = io.bang;
    return io.n > 0 && io.n >= io.bang ? io.sum / (double)io.n : NAN;
}

static void emu(const char *cmd) { char b[48]; snprintf(b, sizeof b, "E %s\n", cmd); send_line(b); must_pump(150); }

/* ------------------------------------------------------------------ main */
static void on_sig(int s) { (void)s; g_stop = 1; }

int main(int argc, char **argv)
{
    const char *dev = "/dev/serusb1", *conf_arg = NULL;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--dev") && i + 1 < argc) dev = argv[++i];
        else if (!strcmp(argv[i], "--conf") && i + 1 < argc) conf_arg = argv[++i];
        else if (!strcmp(argv[i], "--auto")) auto_mode = 1;
        else {
            fprintf(stderr, "usage: rxcal [--dev PATH] [--conf FILE] [--auto]\n"
                            "  --dev   serial device of the I/O node (Pi: /dev/serusb1, laptop: /dev/ttyACM0)\n"
                            "  --conf  file to write (default: reflex.conf next to this binary)\n"
                            "  --auto  no person needed; refused unless the node is an emulator (testing)\n");
            return 2;
        }
    }
    char conf_path[512];
    if (conf_arg) snprintf(conf_path, sizeof conf_path, "%s", conf_arg);
    else { char tmp[512]; snprintf(tmp, sizeof tmp, "%s", argv[0]); snprintf(conf_path, sizeof conf_path, "%s/reflex.conf", dirname(tmp)); }

    /* a running guardian owns the servo: never fight it */
    rx_shm_t *shm = rx_shm_open(0);
    if (shm) {
        pid_t g = atomic_load(&shm->guardian_pid);
        if (g > 0 && (kill(g, 0) == 0 || errno == EPERM)) {   /* EPERM: alive, but root (HAM) */
            fprintf(stderr, "rxcal: a reflex-guardian is running (pid %d). Stop it first (./reflex-demo.sh stop).\n", (int)g);
            return 1;
        }
    }

    signal(SIGINT, on_sig); signal(SIGTERM, on_sig); signal(SIGPIPE, SIG_IGN);
    atexit(tty_restore);
    setvbuf(stdout, NULL, _IOLBF, 0);

    /* start from the existing calibration if there is one, else the firmware's old defaults */
    rx_conf_t c; char err[256];
    int had_conf = rx_conf_load(&c, conf_path, err, sizeof err) == 0;
    if (!had_conf && rx_conf_load_any(&c, conf_path, err, sizeof err) != 0) rx_conf_default(&c);   /* keep pilot transport keys */
    printf("rxcal: device %s, writing %s%s\n", dev, conf_path, had_conf ? " (existing file used as the starting guess)" : "");

    /* ---------------------------------------------------------------- 1. link */
    header(1, "link check");
    io.fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (io.fd < 0) die("%s: %s (is the Black Pill plugged in? on the Pi: ls /dev/serusb*)", dev, strerror(errno));
    struct termios t;
    if (tcgetattr(io.fd, &t) == 0) {
        cfmakeraw(&t); cfsetispeed(&t, B115200); cfsetospeed(&t, B115200); t.c_cflag |= CLOCAL | CREAD;
        tcsetattr(io.fd, TCSANOW, &t);
        tcflush(io.fd, TCIFLUSH);
    }
    rx_link_init(&io.link);
    send_line("?\n");
    uint64_t t0 = now_ms();
    while (!io.link.have_info && now_ms() - t0 < 3000) { if (pump(100, 0) < 0) die("lost the link"); if (now_ms() - t0 > 1500) send_line("?\n"); }
    if (io.link.v1_lines && !io.link.have_info)
        die("the I/O node runs protocol-v1 firmware. Reflash it: make fw && make flash (docs/HARDWARE.md Part 5)");
    if (!io.link.have_info) die("no answer to \"?\" in 3 s: wrong device, or not Reflex firmware");
    if (io.link.proto != RX_LINK_PROTO) die("node speaks protocol %d, rxcal needs %d: reflash the firmware", io.link.proto, RX_LINK_PROTO);
    printf("  node: reflex_io protocol %d, board %s, %s\n", io.link.proto, io.link.board, io.link.emulated ? "EMULATED plant" : "real sensor and servo");
    if (auto_mode && !io.link.emulated) die("--auto is only allowed on an emulated node; this one drives a real servo");
    long n, bang;
    average(1000, &n, &bang);
    printf("  sensor: %ld readings in 1 s (%ld valid, %ld 'no reading')\n", n + bang, n, bang);
    if (n + bang < 30) die("expected ~50 readings per second, got %ld", n + bang);
    if (n == 0) printf("  WARNING: no valid reading at all. With no ball on the beam this can be normal;\n"
                       "           if it stays like this with the ball on the beam, check the sensor wiring.\n");
    snprintf(c.board, sizeof c.board, "%s", io.link.board);

    /* ---------------------------------------------------------------- 2. level */
    header(2, "level");
    printf("Take the ball off the beam. Put the phone inclinometer on the beam and adjust\nuntil it reads 0.0 deg (+/-0.3).\n");
    hands_clear();
    int level = c.level_us;
    if (auto_mode) {
        if (io.link.emulated) emu("remove");
        hold_at(level); must_pump(500);
        double a0 = truth_mrad();
        hold_at(level + 40); must_pump(500);
        double a1 = truth_mrad();
        double slope = (a1 - a0) / 40.0;                 /* mrad per us */
        if (fabs(slope) < 0.05) die("the beam angle does not follow the servo pulse (%.1f -> %.1f mrad)", a0, a1);
        for (int it = 0; it < 4; it++) {
            hold_at(level); must_pump(500);
            double a = truth_mrad();
            printf("  pulse %d us -> beam %+.0f mrad\n", level, a);
            if (fabs(a) <= 1.0) break;
            level = (int)lrint(level - a / slope);
            if (level < RX_PULSE_MIN || level > RX_PULSE_MAX) die("level search left the pulse range");
        }
    } else {
        printf("Keys: Up/Down = +/-5 us, Right/Left = +/-1 us, Enter = accept, q = abort\n");
        tty_set_raw();
        hold_at(level);
        for (;;) {
            if (pump(60, 0) < 0) { tty_restore(); die("lost the link"); }
            if (g_stop) { tty_restore(); die("aborted"); }
            printf("\r  pulse %4d us", level);
            if (io.link.emulated && io.link.have_truth) printf("   (emulator: beam %+5.1f deg)", io.link.theta_mrad * 0.0572958);
            printf("      "); fflush(stdout);
            char k[8]; ssize_t r = read(0, k, sizeof k);
            if (r <= 0) continue;
            if (k[0] == '\n' || k[0] == '\r') break;
            if (k[0] == 'q') { tty_restore(); die("aborted"); }
            if (r >= 3 && k[0] == 27 && k[1] == '[') {
                int d = k[2] == 'A' ? 5 : k[2] == 'B' ? -5 : k[2] == 'C' ? 1 : k[2] == 'D' ? -1 : 0;
                level += d;
                if (level < RX_PULSE_MIN) level = RX_PULSE_MIN;
                if (level > RX_PULSE_MAX) level = RX_PULSE_MAX;
                hold_at(level);
            }
        }
        tty_restore();
        printf("\n");
    }
    c.level_us = level;
    hold_at(level);
    printf("  level_us = %d\n", level);

    /* ---------------------------------------------------------------- 3. direction */
    header(3, "direction");
    printf("Put the ball on the beam near the centre and let go. The beam will tilt a little each way.\n");
    hands_clear();
    if (auto_mode) { emu("pin 0"); emu("free"); }
    int d = (int)lrintf(25.0f * c.us_per_mrad);          /* ~25 mrad with the current guess */
    if (d < 20) d = 20;
    hold_at(level); must_pump(400);
    double r0 = average(200, NULL, NULL);
    hold_at(level + d); must_pump(500);
    double r1 = average(200, NULL, NULL);
    hold_at(level - d); must_pump(1000);                  /* roll it back */
    hold_at(level); must_pump(300);
    if (!isfinite(r0) || !isfinite(r1)) die("lost the ball during the direction test; put it back near the centre and retry");
    printf("  pulse +%d us: reading moved %+.0f mm\n", d, r1 - r0);
    if (fabs(r1 - r0) < 10) die("the ball hardly moved (%.0f mm). Check that it is free to roll, or the servo linkage", r1 - r0);
    if (!ask_yes("Did you see the ball roll?")) die("then the sensor is not looking at the ball: check its aim");
    int sigma = (r1 > r0) ? 1 : -1;                        /* sign of reading change per pulse increase */

    /* ---------------------------------------------------------------- 4. centre */
    header(4, "centre");
    wait_enter("Hold the ball still exactly over the pivot mark (gently, on the beam).");
    if (auto_mode) emu("pin 0");
    double ctr = average(2000, &n, &bang);
    if (!isfinite(ctr)) die("no valid readings with the ball at the pivot");
    printf("  centre: %.1f mm (%ld readings, spread %d..%d)\n", ctr, n, io.mn, io.mx);
    c.center_mm = (int)lrint(ctr);

    /* ---------------------------------------------------------------- 5. range */
    header(5, "range");
    wait_enter("Hold the ball against the end stop NEAREST the sensor.");
    if (auto_mode) emu(sigma * c.servo_dir > 0 ? "pin -200" : "pin 200");
    double ra = average(1000, NULL, NULL);
    wait_enter("Now hold the ball against the FAR end stop.");
    if (auto_mode) emu(sigma * c.servo_dir > 0 ? "pin 200" : "pin -200");
    double rb = average(1000, NULL, NULL);
    wait_enter("Now take the ball off the beam.");
    if (auto_mode) emu("remove");
    double rn = average(1000, &n, &bang);
    if (!isfinite(ra) || !isfinite(rb)) die("no valid readings at an end stop");
    double near = fmin(ra, rb), far = fmax(ra, rb);
    printf("  near stop %.0f mm, far stop %.0f mm, no ball: %s", near, far, isfinite(rn) ? "" : "no valid reading\n");
    if (isfinite(rn)) printf("%.0f mm\n", rn);
    if (!(near < c.center_mm && c.center_mm < far)) die("the centre (%d) is not between the two stops", c.center_mm);
    c.range_min_mm = (int)fmax(0, floor(near - 15));
    c.range_max_mm = (int)ceil(far + 15);
    if (isfinite(rn) && rn > far) {
        if (rn - far < 30) printf("  WARNING: 'no ball' reads only %.0f mm beyond the far stop: a missing ball may be\n"
                                  "           mistaken for one at the far end. Move the sensor's backstop further away.\n", rn - far);
        if (c.range_max_mm > rn - 10) c.range_max_mm = (int)floor((far + rn) / 2);
    } else if (isfinite(rn) && rn >= near && rn <= far) {
        die("with no ball the sensor reads %.0f mm, inside the beam: it is seeing something else", rn);
    }
    if (c.range_max_mm > RX_LINK_MAX_MM) c.range_max_mm = RX_LINK_MAX_MM;
    double half = fmin(c.center_mm - near, far - c.center_mm) / 1000.0;
    c.half_len_m = (float)fmin(0.20, floor(half * 1000.0) / 1000.0);
    printf("  range window %d..%d mm, usable half length %.3f m\n", c.range_min_mm, c.range_max_mm, (double)c.half_len_m);
    if (c.half_len_m < 0.10f) die("usable half length %.3f m is below 0.10 m: check the stops and the centre", (double)c.half_len_m);

    /* ---------------------------------------------------------------- 6. angle scale */
    header(6, "angle scale");
    printf("Keep the ball OFF the beam. Put the inclinometer on the beam.\n");
    hands_clear();
    int amp = (int)lrintf(200.0f * c.us_per_mrad);       /* ~200 mrad with the current guess */
    if (level + amp > RX_PULSE_MAX) amp = RX_PULSE_MAX - level;
    if (level - amp < RX_PULSE_MIN) amp = level - RX_PULSE_MIN;
    double ap_deg, am_deg;
    hold_at(level + amp); must_pump(1000);
    if (auto_mode) { ap_deg = truth_mrad() * 0.0572958; printf("  pulse %d us: %+.2f deg  [auto]\n", level + amp, ap_deg); }
    else ap_deg = ask_number("Inclinometer reading in degrees (with its sign):");
    hold_at(level - amp); must_pump(1000);
    if (auto_mode) { am_deg = truth_mrad() * 0.0572958; printf("  pulse %d us: %+.2f deg  [auto]\n", level - amp, am_deg); }
    else am_deg = ask_number("And now, in degrees (with its sign):");
    hold_at(level); must_pump(500);
    double span_mrad = fabs(ap_deg - am_deg) / 0.0572958;
    if (span_mrad < 60) die("the beam moved only %.1f deg between the two pulses: check the servo linkage", span_mrad * 0.0572958);
    c.servo_dir = (ap_deg > am_deg) ? 1 : -1;
    c.us_per_mrad = (float)(2.0 * amp / span_mrad);
    c.sensor_dir = c.servo_dir * sigma;
    printf("  servo_dir %+d, us_per_mrad %.3f, sensor_dir %+d\n", c.servo_dir, (double)c.us_per_mrad, c.sensor_dir);
    if (fabs(fabs(ap_deg) - fabs(am_deg)) > 1.0)
        printf("  WARNING: the two tilts differ by %.1f deg in size (> 1 deg): the level is off, or the linkage is not symmetric.\n",
               fabs(fabs(ap_deg) - fabs(am_deg)));
    if (c.us_per_mrad < 0.3f || c.us_per_mrad > 10.0f) die("us_per_mrad %.3f is implausible", (double)c.us_per_mrad);

    /* ---------------------------------------------------------------- 7. validation */
    header(7, "validation: 20 s closed loop on the fallback controller");
    printf("Put the ball on the beam near the centre and let go.\n");
    hands_clear();
    if (auto_mode) { emu("pin 0"); emu("free"); }
    {
        rx_cfg_t cfg; rx_cfg_default(&cfg, 50.0f); rx_cfg_hardware(&cfg);
        if (c.half_len_m < cfg.half_len) cfg.half_len = c.half_len_m;
        rx_est_t e = { 0 };
        double worst = 0, se = 0; long k = 0, misses = 0, run = 0;
        uint64_t tv = now_ms();
        io.hold = 0;                                      /* from here the loop commands every sample */
        while (now_ms() - tv < 20000) {
            int g = pump(100, 1);
            if (g < 0) die("lost the link during validation");
            if (g_stop) die("aborted");
            if (g == 0) { send_pulse(c.level_us); continue; }
            int valid;
            float x = io.last_valid ? rx_conf_position(&c, io.last_raw, &valid) : (valid = 0, NAN);
            if (!valid) {
                misses++; run++;
                send_pulse(c.level_us);
                if (run >= 10) die("the ball left the sensor's window during validation (lost for 10 readings)");
                continue;
            }
            run = 0;
            rx_est_update(&e, &cfg, x);
            float u = rx_safe_u(&cfg, e.x, e.v, 0.0f);
            send_pulse(rx_conf_pulse(&c, u));
            if (fabs(e.x) > worst) worst = fabs(e.x);
            se += (double)e.x * e.x; k++;
            if (k % 50 == 0) { printf("%s  t=%2llus  x=%+6.1f mm  worst %5.1f mm   ", isatty(1) ? "\r" : "\n", (unsigned long long)((now_ms() - tv) / 1000), e.x * 1000.0, worst * 1000.0); fflush(stdout); }
        }
        hold_at(c.level_us);
        printf("\n  %ld control steps, worst |x| %.1f mm, RMS %.1f mm, %ld missing readings\n",
               k, worst * 1000.0, k ? sqrt(se / (double)k) * 1000.0 : 0.0, misses);
        if (k < 500) die("only %ld readings in 20 s", k);
        if (worst > 0.150) die("FAIL: the ball went %.0f mm from the centre (limit 150 mm). Calibration NOT written.", worst * 1000.0);
        printf("  PASS: the ball stayed inside +/-150 mm\n");
    }

    /* ---------------------------------------------------------------- 8. write */
    header(8, "write reflex.conf");
    time_t now = time(NULL); struct tm tm; localtime_r(&now, &tm);
    strftime(c.calibrated_at, sizeof c.calibrated_at, "%Y-%m-%dT%H:%M:%S", &tm);
    if (rx_conf_check(&c, err, sizeof err) != 0) die("calibration failed its own check: %s", err);
    if (access(conf_path, F_OK) == 0) {
        char bak[600], ts[32];
        strftime(ts, sizeof ts, "%Y%m%d-%H%M%S", &tm);
        snprintf(bak, sizeof bak, "%s.bak-%s", conf_path, ts);
        if (rename(conf_path, bak) == 0) printf("  previous file kept as %s\n", bak);
    }
    if (rx_conf_save(&c, conf_path, err, sizeof err) != 0) die("%s", err);
    char b[24]; rx_link_cmd_level(b, sizeof b, c.level_us, RX_PULSE_MIN, RX_PULSE_MAX);
    send_line(b);                                         /* the node keeps the level pulse for its watchdog */
    must_pump(300);
    release();
    if (auto_mode) emu("free");
    char text[1024]; rx_conf_format(&c, text, sizeof text);
    printf("  wrote %s:\n\n%s\n", conf_path, text);
    printf("rxcal: done. Start the demo with ./reflex-demo.sh start\n");
    close(io.fd);
    return 0;
}
