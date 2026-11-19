/* See rx_link.h. SPDX-License-Identifier: MIT */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "rx_link.h"

void rx_link_init(rx_link_t *l) { memset(l, 0, sizeof *l); }

static int is_space(char c) { return c == ' ' || c == '\t'; }

/* strict decimal integer: optional sign, 1..6 digits, nothing else */
static int parse_int(const char *s, int32_t *out)
{
    int neg = 0, nd = 0;
    int32_t v = 0;
    if (*s == '+' || *s == '-') { neg = *s == '-'; s++; }
    while (*s >= '0' && *s <= '9') {
        if (++nd > 6) return -1;
        v = v * 10 + (*s - '0');
        s++;
    }
    while (is_space(*s)) s++;
    if (nd == 0 || *s) return -1;
    *out = neg ? -v : v;
    return 0;
}

int rx_link_parse_line(const char *s, int *valid, int32_t *mm)
{
    while (is_space(*s)) s++;
    if (*s == 0 || *s == '#') return 0;
    if (s[0] != 'P' || !is_space(s[1])) return -1;
    s += 2;
    while (is_space(*s)) s++;
    if (s[0] == '!') {
        s++;
        while (is_space(*s)) s++;
        if (*s) return -1;
        *valid = 0; *mm = 0;
        return 1;
    }
    int32_t v;
    if (parse_int(s, &v) != 0 || v > RX_LINK_MAX_MM || v < -RX_LINK_MAX_MM) return -1;
    *valid = 1; *mm = v;
    return 1;
}

int rx_link_feed(rx_link_t *l, const char *buf, size_t n)
{
    int got = 0;
    for (size_t i = 0; i < n; i++) {
        char ch = buf[i];
        if (ch == '\n' || ch == '\r') {
            if (l->overflow) l->garbage++;
            else if (l->len > 0) {
                l->line[l->len] = 0;
                int valid; int32_t mm;
                int r = rx_link_parse_line(l->line, &valid, &mm);
                if (r > 0) { l->valid = valid; l->mm = mm; l->samples++; got++; }
                else if (r < 0) l->garbage++;
                else l->comments++;
            }
            l->len = 0; l->overflow = 0;
        } else if (l->overflow) {
            /* drop the rest of an over-long line */
        } else if (l->len < RX_LINK_LINE - 1) {
            l->line[l->len++] = ch;
        } else {
            l->overflow = 1;
        }
    }
    return got;
}

int rx_link_cmd(char *buf, size_t cap, float u_rad, int max_mrad)
{
    long m = isfinite(u_rad) ? lrintf(fminf(fmaxf(u_rad, -10.0f), 10.0f) * 1000.0f) : 0;
    if (m > max_mrad) m = max_mrad;
    if (m < -max_mrad) m = -max_mrad;
    int k = snprintf(buf, cap, "U %ld\n", m);
    return k < 0 ? 0 : (k >= (int)cap ? (int)cap - 1 : k);
}
