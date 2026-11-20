/* Offline tests and fuzzing of the I/O node line protocol (src/core/rx_link.c) and of the rig
 * calibration file (src/core/rx_conf.c). Exit code 0 only if every check passes.
 * SPDX-License-Identifier: MIT */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rx_conf.h"
#include "rx_link.h"

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static uint64_t rng = 0x243F6A8885A308D3ull;
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; return (uint32_t)(rng >> 11); }

static void line_cases(void)
{
    struct { const char *s; int kind, valid, mm; } t[] = {
        { "M 123", RX_LK_SAMPLE, 1, 123 }, { "M 0", RX_LK_SAMPLE, 1, 0 }, { "M +7", RX_LK_SAMPLE, 1, 7 },
        { "  M   45  ", RX_LK_SAMPLE, 1, 45 }, { "M 2000", RX_LK_SAMPLE, 1, 2000 },
        { "M !", RX_LK_SAMPLE, 0, 0 }, { "M  ! ", RX_LK_SAMPLE, 0, 0 },
        { "# reflex_io ready", RX_LK_NONE, 0, 0 }, { "", RX_LK_NONE, 0, 0 }, { "   ", RX_LK_NONE, 0, 0 },
        { "M 2001", RX_LK_GARBAGE, 0, 0 }, { "M -1", RX_LK_GARBAGE, 0, 0 }, { "M 12abc", RX_LK_GARBAGE, 0, 0 },
        { "M", RX_LK_GARBAGE, 0, 0 }, { "M ", RX_LK_GARBAGE, 0, 0 }, { "M1", RX_LK_GARBAGE, 0, 0 },
        { "M 1 2", RX_LK_GARBAGE, 0, 0 }, { "M !x", RX_LK_GARBAGE, 0, 0 }, { "M --1", RX_LK_GARBAGE, 0, 0 },
        { "M 1234567", RX_LK_GARBAGE, 0, 0 }, { "m 5", RX_LK_GARBAGE, 0, 0 }, { "M 1.5", RX_LK_GARBAGE, 0, 0 },
        { "M 0x10", RX_LK_GARBAGE, 0, 0 }, { "S 1500", RX_LK_GARBAGE, 0, 0 }, { "Q 5", RX_LK_GARBAGE, 0, 0 },
        { "T -37", RX_LK_TRUTH, 0, 0 }, { "T 2001", RX_LK_GARBAGE, 0, 0 }, { "T", RX_LK_GARBAGE, 0, 0 },
        { "P 12", RX_LK_V1, 0, 0 }, { "P !", RX_LK_V1, 0, 0 }, { "P 12abc", RX_LK_GARBAGE, 0, 0 },
        { "I reflex_io 2 BLACKPILL_F401CC 1", RX_LK_INFO, 0, 0 }, { "I reflex_io 2 UNO 0 ", RX_LK_INFO, 0, 0 },
        { "I reflex_io 2 BLACKPILL_F401CC 2", RX_LK_GARBAGE, 0, 0 }, { "I other 2 X 0", RX_LK_GARBAGE, 0, 0 },
        { "I reflex_io x B 0", RX_LK_GARBAGE, 0, 0 }, { "I reflex_io 2 B", RX_LK_GARBAGE, 0, 0 },
        { "I reflex_io 2 B 0 extra", RX_LK_GARBAGE, 0, 0 }, { "I reflex_io 2 B$d 0", RX_LK_GARBAGE, 0, 0 },
        { "I reflex_io 2 AVERYVERYVERYLONGBOARDNAME_123 0", RX_LK_GARBAGE, 0, 0 },
    };
    for (size_t i = 0; i < sizeof t / sizeof t[0]; i++) {
        rx_link_msg_t m;
        int k = rx_link_parse_line(t[i].s, &m);
        CHECK(k == t[i].kind && m.kind == k, "'%s': kind %d, want %d", t[i].s, k, t[i].kind);
        if (k == RX_LK_SAMPLE) CHECK(m.valid == t[i].valid && (!m.valid || m.raw_mm == t[i].mm), "'%s': valid=%d mm=%d", t[i].s, m.valid, (int)m.raw_mm);
    }
    rx_link_msg_t m;
    rx_link_parse_line("I reflex_io 2 BLACKPILL_F401CC 1", &m);
    CHECK(m.proto == 2 && m.emulated == 1 && !strcmp(m.board, "BLACKPILL_F401CC"), "I line fields: %d %d '%s'", m.proto, m.emulated, m.board);
    rx_link_parse_line("T -37", &m);
    CHECK(m.theta_mrad == -37, "T line: %d", (int)m.theta_mrad);
}

/* the same stream split at every possible point must give the same result */
static void split_invariance(void)
{
    const char *stream = "I reflex_io 2 BP 1\r\nM 10\nM !\r\ngarbage\nT 12\nM 250\n\nP 3\nM 7\r";
    rx_link_t ref; rx_link_init(&ref);
    int n_ref = rx_link_feed(&ref, stream, strlen(stream));
    CHECK(n_ref == 4 && ref.valid && ref.raw_mm == 7 && ref.garbage == 1 && ref.have_info && ref.emulated == 1 &&
          ref.have_truth && ref.theta_mrad == 12 && ref.v1_lines == 1,
          "reference parse: n=%d valid=%d mm=%d garbage=%u", n_ref, ref.valid, (int)ref.raw_mm, ref.garbage);
    size_t L = strlen(stream);
    for (size_t a = 0; a <= L; a++) for (size_t b = a; b <= L; b++) {
        rx_link_t l; rx_link_init(&l);
        int n = rx_link_feed(&l, stream, a) + rx_link_feed(&l, stream + a, b - a) + rx_link_feed(&l, stream + b, L - b);
        if (n != n_ref || l.raw_mm != ref.raw_mm || l.valid != ref.valid || l.garbage != ref.garbage ||
            l.infos != ref.infos || l.theta_mrad != ref.theta_mrad || l.v1_lines != ref.v1_lines) {
            CHECK(0, "split at %zu/%zu differs", a, b); return;
        }
    }
}

/* an over-long line is dropped whole: a valid-looking tail must not be parsed */
static void overflow(void)
{
    rx_link_t l; rx_link_init(&l);
    char buf[512]; memset(buf, 'x', sizeof buf);
    memcpy(buf + 400, "M 99", 4); buf[510] = '\n'; buf[511] = 0;
    int n = rx_link_feed(&l, buf, 511);
    CHECK(n == 0 && l.garbage == 1, "overlong line: n=%d garbage=%u", n, l.garbage);
    char long_m[160] = "M 5"; memset(long_m + 3, ' ', 100); strcpy(long_m + 103, "1\n");
    n = rx_link_feed(&l, long_m, strlen(long_m));
    CHECK(n == 0, "'M 5' followed by 100 spaces and '1' was accepted");
    n = rx_link_feed(&l, "M 42\n", 5);
    CHECK(n == 1 && l.raw_mm == 42, "line after overflow not parsed");
}

/* a burst after a stall: many lines in one read, the newest wins */
static void burst(void)
{
    rx_link_t l; rx_link_init(&l);
    char buf[4096]; int k = 0;
    for (int i = 0; i < 200; i++) k += snprintf(buf + k, sizeof buf - (size_t)k, "M %d\n", i + 100);
    int n = rx_link_feed(&l, buf, (size_t)k);
    CHECK(n == 200 && l.raw_mm == 299 && l.valid, "burst: n=%d mm=%d", n, (int)l.raw_mm);
}

/* random bytes, with valid lines interleaved after a line break: never crash, never accept an
 * out-of-range value, and always recover the planted line */
static void fuzz(void)
{
    rx_link_t l; rx_link_init(&l);
    char buf[256];
    long planted = 0, recovered = 0, accepted = 0;
    for (int it = 0; it < 200000; it++) {
        size_t n = rnd() % sizeof buf;
        for (size_t i = 0; i < n; i++) {
            uint32_t r = rnd() % 18;
            buf[i] = r < 2 ? '\n' : r < 3 ? 'M' : r < 4 ? ' ' : r < 6 ? (char)('0' + rnd() % 10) : r < 7 ? '-' :
                     r < 8 ? 'I' : r < 9 ? 'T' : (char)rnd();
        }
        int got = rx_link_feed(&l, buf, n);
        accepted += got;
        if (got) CHECK(!l.valid || (l.raw_mm >= 0 && l.raw_mm <= RX_LINK_MAX_MM), "fuzz accepted mm=%d", (int)l.raw_mm);
        if (l.have_info) CHECK(l.board[RX_LINK_BOARD - 1] == 0 && strlen(l.board) > 0, "fuzz: board name not terminated");
        if (it % 7 == 0) {
            int want = (int)(rnd() % 2001);
            char ok[32]; int k = snprintf(ok, sizeof ok, "\nM %d\n", want);
            planted++;
            if (rx_link_feed(&l, ok, (size_t)k) >= 1 && l.valid && l.raw_mm == want) recovered++;
        }
        if (fails > 20) break;
    }
    CHECK(recovered == planted, "fuzz: recovered %ld of %ld planted lines", recovered, planted);
    printf("  fuzz: 200000 random chunks, %ld lines accepted, %u rejected, planted lines recovered %ld/%ld\n",
           accepted, l.garbage, recovered, planted);
}

static void commands(void)
{
    char b[32];
    rx_link_cmd_servo(b, sizeof b, 1534, 1000, 2000); CHECK(!strcmp(b, "S 1534\n"), "servo 1534 -> '%s'", b);
    rx_link_cmd_servo(b, sizeof b, 400, 1000, 2000);  CHECK(!strcmp(b, "S 1000\n"), "servo 400 -> '%s'", b);
    rx_link_cmd_servo(b, sizeof b, 9999, 1000, 2000); CHECK(!strcmp(b, "S 2000\n"), "servo 9999 -> '%s'", b);
    rx_link_cmd_level(b, sizeof b, 1490, 1000, 2000); CHECK(!strcmp(b, "L 1490\n"), "level -> '%s'", b);
    rx_link_cmd_query(b, sizeof b);                   CHECK(!strcmp(b, "?\n"), "query -> '%s'", b);
}

static const char *GOOD =
    "# test\n"
    "level_us = 1530\nus_per_mrad = 2.1   # trailing comment\nservo_dir = -1\ncenter_mm = 247\n"
    "sensor_dir = -1\nrange_min_mm = 30\nrange_max_mm = 480\nhalf_len_m = 0.19\n"
    "calibrated_at = 2026-11-20T13:00:00+05:30\nboard = BLACKPILL_F401CC\n";

static void conf_cases(void)
{
    rx_conf_t c; char err[160];
    CHECK(rx_conf_parse(&c, GOOD, err, sizeof err) == 0, "good file rejected: %s", err);
    CHECK(c.level_us == 1530 && fabsf(c.us_per_mrad - 2.1f) < 1e-6f && c.servo_dir == -1 && c.center_mm == 247 &&
          c.sensor_dir == -1 && c.range_min_mm == 30 && c.range_max_mm == 480 && fabsf(c.half_len_m - 0.19f) < 1e-6f &&
          !strcmp(c.board, "BLACKPILL_F401CC"), "good file: wrong values");
    /* round trip */
    char buf[1024]; rx_conf_t d;
    CHECK(rx_conf_format(&c, buf, sizeof buf) > 0, "format failed");
    CHECK(rx_conf_parse(&d, buf, err, sizeof err) == 0 && d.level_us == c.level_us && d.center_mm == c.center_mm &&
          fabsf(d.us_per_mrad - c.us_per_mrad) < 1e-4f && !strcmp(d.calibrated_at, c.calibrated_at), "round trip: %s", err);
    CHECK(rx_conf_format(&c, buf, 20) == -1, "format into a tiny buffer did not fail");
    /* every single-key removal must be rejected with "missing key" */
    const char *keys[] = { "level_us", "us_per_mrad", "servo_dir", "center_mm", "sensor_dir", "range_min_mm",
                           "range_max_mm", "half_len_m", "calibrated_at", "board" };
    for (unsigned i = 0; i < 10; i++) {
        char t[1024] = ""; const char *p = GOOD;
        while (*p) {
            const char *nl = strchr(p, '\n'); size_t n = (size_t)(nl - p) + 1;
            if (strncmp(p, keys[i], strlen(keys[i])) || p[strlen(keys[i])] != ' ') strncat(t, p, n);
            p += n;
        }
        CHECK(rx_conf_parse(&d, t, err, sizeof err) != 0 && strstr(err, "missing key") && strstr(err, keys[i]),
              "without %s: '%s'", keys[i], err);
    }
    /* bad values and bad syntax */
    struct { const char *replace, *with, *why; } bad[] = {
        { "level_us = 1530", "level_us = 2500", "level_us" }, { "level_us = 1530", "level_us = 15x0", "not an integer" },
        { "us_per_mrad = 2.1", "us_per_mrad = 0", "us_per_mrad" }, { "us_per_mrad = 2.1", "us_per_mrad = nan", "number" },
        { "servo_dir = -1", "servo_dir = 2", "servo_dir" }, { "sensor_dir = -1", "sensor_dir = 0", "sensor_dir" },
        { "center_mm = 247", "center_mm = 600", "center_mm" }, { "range_min_mm = 30", "range_min_mm = 490", "range" },
        { "half_len_m = 0.19", "half_len_m = 0.25", "half_len_m" }, { "half_len_m = 0.19", "half_len_m = 0.05", "half_len_m" },
        { "board = BLACKPILL_F401CC", "board =", "board" }, { "servo_dir = -1", "servo_dir -1", "key = value" },
        { "servo_dir = -1", "servo_dir = -1\nservo_dir = 1", "twice" }, { "servo_dir = -1", "servo_dir = -1\ncolour = red", "unknown key" },
    };
    for (unsigned i = 0; i < sizeof bad / sizeof bad[0]; i++) {
        char t[1200]; const char *at = strstr(GOOD, bad[i].replace);
        snprintf(t, sizeof t, "%.*s%s%s", (int)(at - GOOD), GOOD, bad[i].with, at + strlen(bad[i].replace));
        int r = rx_conf_parse(&d, t, err, sizeof err);
        CHECK(r != 0 && strstr(err, bad[i].why), "'%s' -> r=%d '%s'", bad[i].with, r, err);
    }
    /* conversions */
    int v;
    CHECK(fabsf(rx_conf_position(&c, 247, &v)) < 1e-6f && v, "centre -> 0");
    CHECK(fabsf(rx_conf_position(&c, 347, &v) + 0.100f) < 1e-6f && v, "sensor_dir -1: +100 mm raw -> -0.1 m");
    rx_conf_position(&c, 481, &v); CHECK(!v, "outside range window is valid");
    rx_conf_position(&c, 29, &v);  CHECK(!v, "below range window is valid");
    CHECK(rx_conf_pulse(&c, 0.0f) == 1530, "0 rad -> level");
    CHECK(rx_conf_pulse(&c, 0.1f) == 1530 - 210, "0.1 rad with servo_dir -1, 2.1 us/mrad -> %d", rx_conf_pulse(&c, 0.1f));
    CHECK(rx_conf_pulse(&c, NAN) == 1530, "NaN -> level");
    CHECK(rx_conf_pulse(&c, 40.0f) == RX_PULSE_MIN && rx_conf_pulse(&c, -40.0f) == RX_PULSE_MAX, "huge -> clamped");
    rx_conf_default(&d);
    CHECK(rx_conf_check(&d, err, sizeof err) == 0, "defaults do not pass the check: %s", err);
}

int main(void)
{
    printf("== I/O node link parser (protocol v%d) ==\n", RX_LINK_PROTO);
    line_cases(); split_invariance(); overflow(); burst(); fuzz(); commands();
    printf("== reflex.conf ==\n");
    conf_cases();
    printf("%s\n", fails ? "LINKTEST FAILED" : "LINKTEST PASSED");
    return fails != 0;
}
