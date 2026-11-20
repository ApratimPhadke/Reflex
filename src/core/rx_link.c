/* See rx_link.h. SPDX-License-Identifier: MIT */
#include <stdio.h>
#include <string.h>
#include "rx_link.h"

void rx_link_init(rx_link_t *l) { memset(l, 0, sizeof *l); }

static int is_space(char c) { return c == ' ' || c == '\t'; }
static const char *skip(const char *s) { while (is_space(*s)) s++; return s; }

/* strict decimal integer: optional sign, 1..6 digits; *end = first char after the trailing spaces */
static int parse_int(const char *s, int32_t *out, const char **end)
{
    int neg = 0, nd = 0;
    int32_t v = 0;
    if (*s == '+' || *s == '-') { neg = *s == '-'; s++; }
    while (*s >= '0' && *s <= '9') {
        if (++nd > 6) return -1;
        v = v * 10 + (*s - '0');
        s++;
    }
    if (nd == 0 || (*s && !is_space(*s))) return -1;
    *out = neg ? -v : v;
    *end = skip(s);
    return 0;
}

/* a token of [A-Za-z0-9_-], 1..cap-1 chars */
static int parse_word(const char *s, char *out, size_t cap, const char **end)
{
    size_t n = 0;
    while ((*s >= 'A' && *s <= 'Z') || (*s >= 'a' && *s <= 'z') || (*s >= '0' && *s <= '9') || *s == '_' || *s == '-') {
        if (n + 1 >= cap) return -1;
        out[n++] = *s++;
    }
    if (n == 0 || (*s && !is_space(*s))) return -1;
    out[n] = 0;
    *end = skip(s);
    return 0;
}

int rx_link_parse_line(const char *s, rx_link_msg_t *m)
{
    memset(m, 0, sizeof *m);
    s = skip(s);
    if (*s == 0 || *s == '#') return m->kind = RX_LK_NONE;
    char k = s[0];
    if (!is_space(s[1])) return m->kind = RX_LK_GARBAGE;
    s = skip(s + 2);
    int32_t v; const char *e;
    switch (k) {
    case 'M':
        if (s[0] == '!') {
            if (*skip(s + 1)) break;
            m->valid = 0; return m->kind = RX_LK_SAMPLE;
        }
        if (parse_int(s, &v, &e) != 0 || *e || v < 0 || v > RX_LINK_MAX_MM) break;
        m->valid = 1; m->raw_mm = v;
        return m->kind = RX_LK_SAMPLE;
    case 'T':
        if (parse_int(s, &v, &e) != 0 || *e || v < -2000 || v > 2000) break;
        m->theta_mrad = v;
        return m->kind = RX_LK_TRUTH;
    case 'I': {
        char name[16]; int32_t proto, emu;
        if (parse_word(s, name, sizeof name, &e) != 0 || strcmp(name, "reflex_io")) break;
        if (parse_int(e, &proto, &e) != 0 || proto < 1 || proto > 99) break;
        if (parse_word(e, m->board, sizeof m->board, &e) != 0) break;
        if (parse_int(e, &emu, &e) != 0 || *e || (emu != 0 && emu != 1)) break;
        m->proto = (int)proto; m->emulated = (int)emu;
        return m->kind = RX_LK_INFO;
    }
    case 'P':                              /* protocol v1 position line: only recognised to report it */
        if (s[0] == '!' && !*skip(s + 1)) return m->kind = RX_LK_V1;
        if (parse_int(s, &v, &e) == 0 && !*e) return m->kind = RX_LK_V1;
        break;
    default: break;
    }
    memset(m, 0, sizeof *m);
    return m->kind = RX_LK_GARBAGE;
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
                rx_link_msg_t m;
                switch (rx_link_parse_line(l->line, &m)) {
                case RX_LK_SAMPLE: l->valid = m.valid; l->raw_mm = m.raw_mm; l->samples++; got++; break;
                case RX_LK_INFO:
                    l->have_info = 1; l->proto = m.proto; l->emulated = m.emulated;
                    memcpy(l->board, m.board, sizeof l->board); l->infos++; break;
                case RX_LK_TRUTH: l->have_truth = 1; l->theta_mrad = m.theta_mrad; l->truths++; break;
                case RX_LK_V1: l->v1_lines++; break;
                case RX_LK_NONE: l->comments++; break;
                default: l->garbage++; break;
                }
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

static int fmt(char *buf, size_t cap, char c, int pulse_us, int lo, int hi)
{
    if (pulse_us < lo) pulse_us = lo;
    if (pulse_us > hi) pulse_us = hi;
    int k = snprintf(buf, cap, "%c %d\n", c, pulse_us);
    return k < 0 ? 0 : (k >= (int)cap ? (int)cap - 1 : k);
}

int rx_link_cmd_servo(char *buf, size_t cap, int pulse_us, int lo, int hi) { return fmt(buf, cap, 'S', pulse_us, lo, hi); }
int rx_link_cmd_level(char *buf, size_t cap, int pulse_us, int lo, int hi) { return fmt(buf, cap, 'L', pulse_us, lo, hi); }
int rx_link_cmd_query(char *buf, size_t cap)
{
    int k = snprintf(buf, cap, "?\n");
    return k < 0 ? 0 : (k >= (int)cap ? (int)cap - 1 : k);
}
