/* See rx_conf.h. SPDX-License-Identifier: MIT */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include "rx_conf.h"

void rx_conf_default(rx_conf_t *c)
{
    memset(c, 0, sizeof *c);
    c->level_us = 1500; c->us_per_mrad = 1.9f; c->servo_dir = 1;
    c->center_mm = 260; c->sensor_dir = 1;
    c->range_min_mm = 40; c->range_max_mm = 495;
    c->half_len_m = 0.20f;
    snprintf(c->calibrated_at, sizeof c->calibrated_at, "uncalibrated");
    snprintf(c->board, sizeof c->board, "unknown");
}

#define ERR(...) do { if (err && errcap) snprintf(err, errcap, __VA_ARGS__); return -1; } while (0)

int rx_conf_check(const rx_conf_t *c, char *err, size_t errcap)
{
    if (c->level_us < RX_PULSE_MIN || c->level_us > RX_PULSE_MAX) ERR("level_us %d outside %d..%d", c->level_us, RX_PULSE_MIN, RX_PULSE_MAX);
    if (!(c->us_per_mrad >= 0.3f && c->us_per_mrad <= 10.0f)) ERR("us_per_mrad %g outside 0.3..10", (double)c->us_per_mrad);
    if (c->servo_dir != 1 && c->servo_dir != -1) ERR("servo_dir must be 1 or -1");
    if (c->sensor_dir != 1 && c->sensor_dir != -1) ERR("sensor_dir must be 1 or -1");
    if (c->center_mm < 1 || c->center_mm > 1999) ERR("center_mm %d outside 1..1999", c->center_mm);
    if (c->range_min_mm < 0 || c->range_max_mm > 2000 || c->range_min_mm >= c->range_max_mm)
        ERR("range_min_mm..range_max_mm (%d..%d) must be an increasing window inside 0..2000", c->range_min_mm, c->range_max_mm);
    if (c->center_mm <= c->range_min_mm || c->center_mm >= c->range_max_mm)
        ERR("center_mm %d is not inside the range window %d..%d", c->center_mm, c->range_min_mm, c->range_max_mm);
    if (!(c->half_len_m >= 0.10f && c->half_len_m <= 0.20f)) ERR("half_len_m %g outside 0.10..0.20", (double)c->half_len_m);
    return 0;
}

static int set_key(rx_conf_t *c, const char *k, const char *v, unsigned *seen, char *err, size_t errcap)
{
    static const char *const keys[] = { "level_us", "us_per_mrad", "servo_dir", "center_mm", "sensor_dir",
                                        "range_min_mm", "range_max_mm", "half_len_m", "calibrated_at", "board" };
    unsigned i;
    for (i = 0; i < sizeof keys / sizeof keys[0]; i++) if (!strcmp(k, keys[i])) break;
    if (i == sizeof keys / sizeof keys[0]) ERR("unknown key '%s'", k);
    if (*seen & (1u << i)) ERR("key '%s' given twice", k);
    *seen |= 1u << i;
    if (i == 8 || i == 9) {                                  /* strings */
        char *dst = i == 8 ? c->calibrated_at : c->board;
        size_t cap = i == 8 ? sizeof c->calibrated_at : sizeof c->board;
        if (strlen(v) >= cap || !*v) ERR("%s: empty or longer than %zu chars", k, cap - 1);
        memcpy(dst, v, strlen(v) + 1);
        return 0;
    }
    char *end;
    if (i == 1 || i == 7) {                                  /* floats */
        float f = strtof(v, &end);
        if (end == v || *end || !isfinite(f)) ERR("%s: '%s' is not a number", k, v);
        if (i == 1) c->us_per_mrad = f; else c->half_len_m = f;
        return 0;
    }
    long n = strtol(v, &end, 10);                            /* integers */
    if (end == v || *end || n < -100000 || n > 100000) ERR("%s: '%s' is not an integer", k, v);
    switch (i) {
    case 0: c->level_us = (int)n; break;
    case 2: c->servo_dir = (int)n; break;
    case 3: c->center_mm = (int)n; break;
    case 4: c->sensor_dir = (int)n; break;
    case 5: c->range_min_mm = (int)n; break;
    case 6: c->range_max_mm = (int)n; break;
    }
    return 0;
}

static char *trim(char *s)
{
    while (*s == ' ' || *s == '\t') s++;
    char *e = s + strlen(s);
    while (e > s && (e[-1] == ' ' || e[-1] == '\t' || e[-1] == '\r')) *--e = 0;
    return s;
}

int rx_conf_parse(rx_conf_t *c, const char *text, char *err, size_t errcap)
{
    memset(c, 0, sizeof *c);
    unsigned seen = 0;
    int lineno = 0;
    const char *p = text;
    while (*p) {
        const char *nl = strchr(p, '\n');
        size_t n = nl ? (size_t)(nl - p) : strlen(p);
        char line[256];
        lineno++;
        if (n >= sizeof line) ERR("line %d is too long", lineno);
        memcpy(line, p, n); line[n] = 0;
        p += n + (nl ? 1 : 0);
        char *hash = strchr(line, '#');
        if (hash) *hash = 0;
        char *s = trim(line);
        if (!*s) continue;
        char *eq = strchr(s, '=');
        if (!eq) ERR("line %d: expected key = value", lineno);
        *eq = 0;
        char *k = trim(s), *v = trim(eq + 1);
        char e2[128];
        if (set_key(c, k, v, &seen, e2, sizeof e2) != 0) ERR("line %d: %s", lineno, e2);
    }
    if (seen != (1u << 10) - 1) {
        static const char *const keys[] = { "level_us", "us_per_mrad", "servo_dir", "center_mm", "sensor_dir",
                                            "range_min_mm", "range_max_mm", "half_len_m", "calibrated_at", "board" };
        for (unsigned i = 0; i < 10; i++) if (!(seen & (1u << i))) ERR("missing key '%s'", keys[i]);
    }
    return rx_conf_check(c, err, errcap);
}

int rx_conf_format(const rx_conf_t *c, char *buf, size_t cap)
{
    int k = snprintf(buf, cap,
        "# Reflex rig calibration, written by rxcal. Format: docs/ARCHITECTURE.md (reflex.conf).\n"
        "level_us      = %d\n"
        "us_per_mrad   = %.4f\n"
        "servo_dir     = %d\n"
        "center_mm     = %d\n"
        "sensor_dir    = %d\n"
        "range_min_mm  = %d\n"
        "range_max_mm  = %d\n"
        "half_len_m    = %.3f\n"
        "calibrated_at = %s\n"
        "board         = %s\n",
        c->level_us, (double)c->us_per_mrad, c->servo_dir, c->center_mm, c->sensor_dir,
        c->range_min_mm, c->range_max_mm, (double)c->half_len_m, c->calibrated_at, c->board);
    return (k < 0 || (size_t)k >= cap) ? -1 : k;
}

float rx_conf_position(const rx_conf_t *c, int raw_mm, int *valid)
{
    *valid = raw_mm >= c->range_min_mm && raw_mm <= c->range_max_mm;
    return *valid ? (float)(c->sensor_dir * (raw_mm - c->center_mm)) * 1e-3f : NAN;
}

int rx_conf_pulse(const rx_conf_t *c, float u_rad)
{
    if (!isfinite(u_rad)) return c->level_us;
    if (u_rad > 1.0f) u_rad = 1.0f;
    if (u_rad < -1.0f) u_rad = -1.0f;
    long p = lrintf((float)c->level_us + (float)c->servo_dir * u_rad * 1000.0f * c->us_per_mrad);
    if (p < RX_PULSE_MIN) p = RX_PULSE_MIN;
    if (p > RX_PULSE_MAX) p = RX_PULSE_MAX;
    return (int)p;
}

int rx_conf_load(rx_conf_t *c, const char *path, char *err, size_t errcap)
{
    FILE *f = fopen(path, "r");
    if (!f) ERR("%s: %s", path, strerror(errno));
    char text[4096];
    size_t n = fread(text, 1, sizeof text - 1, f);
    int big = !feof(f);
    fclose(f);
    if (big) ERR("%s: larger than %zu bytes", path, sizeof text - 1);
    text[n] = 0;
    char e2[160];
    if (rx_conf_parse(c, text, e2, sizeof e2) != 0) ERR("%s: %s", path, e2);
    return 0;
}

int rx_conf_save(const rx_conf_t *c, const char *path, char *err, size_t errcap)
{
    char text[2048], tmp[512];
    if (rx_conf_check(c, err, errcap) != 0) return -1;
    int n = rx_conf_format(c, text, sizeof text);
    if (n < 0) ERR("format failed");
    if (snprintf(tmp, sizeof tmp, "%s.tmp", path) >= (int)sizeof tmp) ERR("path too long");
    FILE *f = fopen(tmp, "w");
    if (!f) ERR("%s: %s", tmp, strerror(errno));
    int ok = fwrite(text, 1, (size_t)n, f) == (size_t)n;
    ok = (fclose(f) == 0) && ok;
    if (!ok || rename(tmp, path) != 0) { remove(tmp); ERR("%s: %s", path, strerror(errno)); }
    return 0;
}
