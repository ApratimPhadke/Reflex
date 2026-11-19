/* Reflex I/O node link: line protocol parser and command formatter.
 * Pure functions, no OS calls, so they can be fuzzed offline (rx_linktest).
 *
 * Protocol v1 (ASCII, one message per line, '\n' or '\r' terminated):
 *   node -> guardian   "P <position_mm>"   integer, |mm| <= RX_LINK_MAX_MM
 *                      "P !"               sensor sees no ball
 *                      "# ..."             comment, ignored
 *   guardian -> node   "U <beam_angle_millirad>"
 * Anything else is counted as garbage and ignored. A line longer than the buffer is dropped
 * whole, never parsed from its prefix.
 * SPDX-License-Identifier: MIT */
#ifndef RX_LINK_H
#define RX_LINK_H
#include <stddef.h>
#include <stdint.h>

#define RX_LINK_LINE    48
#define RX_LINK_MAX_MM  2000

typedef struct {
    char     line[RX_LINK_LINE];
    int      len, overflow;
    /* latest sample */
    int      valid;          /* 1 = mm holds a position, 0 = "no ball" */
    int32_t  mm;
    /* counters */
    uint32_t samples;        /* "P" lines accepted (positions and "no ball") */
    uint32_t garbage;        /* lines rejected */
    uint32_t comments;
} rx_link_t;

void rx_link_init(rx_link_t *l);
/* Feed raw bytes. Returns the number of "P" lines accepted in this chunk (0 = no new sample);
 * l->valid / l->mm then hold the newest one. */
int  rx_link_feed(rx_link_t *l, const char *buf, size_t n);
/* Parse one complete line without terminator. 1 = sample, 0 = comment/empty, -1 = garbage. */
int  rx_link_parse_line(const char *s, int *valid, int32_t *mm);
/* Format "U <mrad>\n" for a beam angle in rad; NaN/Inf become 0, the value is clamped to
 * +/- max_mrad. Returns the length written (always < cap for cap >= 16). */
int  rx_link_cmd(char *buf, size_t cap, float u_rad, int max_mrad);

#endif
