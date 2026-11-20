/* Reflex I/O node link: line protocol parser and command formatter.
 * Pure functions, no OS calls, so they can be fuzzed offline (rx_linktest).
 *
 * Protocol v2 (ASCII, one message per line, '\n' or '\r' terminated). The node knows nothing about
 * metres or radians: it reports the raw sensor distance and takes a raw servo pulse. Calibration
 * lives on the host in reflex.conf (rx_conf.h), written by the rxcal wizard.
 *   node -> host   "M <raw_mm>"           raw sensor distance, integer 0..RX_LINK_MAX_MM, 50 Hz
 *                  "M !"                  no valid reading (timeout, out of range, sensor missing)
 *                  "I reflex_io <proto> <board> <emulated 0|1>"   identity, on reset and on "?"
 *                  "T <theta_mrad>"       emulated boards only: the true beam angle (stands in for
 *                                         the inclinometer when rxcal is tested); never used by the guardian
 *                  "# ..."                comment, ignored
 *   host -> node   "S <pulse_us>"         servo pulse; the node clamps it to its compile-time hard
 *                                         range and levels the beam if none arrives for 150 ms
 *                  "L <pulse_us>"         level pulse used by that watchdog, kept in the node's flash
 *                  "?"                    request the "I" line
 *                  "E pin <mm>" | "E free" | "E remove"   emulator only: place the simulated ball
 * A protocol-v1 position line ("P ...") is recognised only to report that the firmware is too old.
 * Anything else is counted as garbage and ignored. A line longer than the buffer is dropped whole,
 * never parsed from its prefix.
 * SPDX-License-Identifier: MIT */
#ifndef RX_LINK_H
#define RX_LINK_H
#include <stddef.h>
#include <stdint.h>

#define RX_LINK_PROTO   2
#define RX_LINK_LINE    64
#define RX_LINK_MAX_MM  2000
#define RX_LINK_BOARD   24

enum { RX_LK_GARBAGE = -1, RX_LK_NONE = 0, RX_LK_SAMPLE = 1, RX_LK_INFO = 2, RX_LK_TRUTH = 3, RX_LK_V1 = 4 };

typedef struct {
    int      kind;                       /* RX_LK_* */
    int      valid;  int32_t raw_mm;     /* SAMPLE: valid = 0 for "M !" */
    int      proto, emulated; char board[RX_LINK_BOARD];   /* INFO */
    int32_t  theta_mrad;                 /* TRUTH */
} rx_link_msg_t;

typedef struct {
    char     line[RX_LINK_LINE];
    int      len, overflow;
    /* latest sample */
    int      valid;          /* 1 = raw_mm holds a reading, 0 = "no valid reading" */
    int32_t  raw_mm;
    /* latest identity (have_info = 0 until an "I" line arrived) */
    int      have_info, proto, emulated;
    char     board[RX_LINK_BOARD];
    /* latest emulator truth */
    int      have_truth;
    int32_t  theta_mrad;
    /* counters */
    uint32_t samples;        /* "M" lines accepted (readings and "no reading") */
    uint32_t garbage;        /* lines rejected */
    uint32_t comments;
    uint32_t infos, truths;
    uint32_t v1_lines;       /* protocol-v1 "P" lines seen: the firmware needs reflashing */
} rx_link_t;

void rx_link_init(rx_link_t *l);
/* Feed raw bytes. Returns the number of "M" lines accepted in this chunk (0 = no new sample);
 * l->valid / l->raw_mm then hold the newest one. */
int  rx_link_feed(rx_link_t *l, const char *buf, size_t n);
/* Parse one complete line without terminator into *m. Returns m->kind. */
int  rx_link_parse_line(const char *s, rx_link_msg_t *m);
/* Commands. Each returns the length written (always < cap for cap >= 16); the pulse is clamped
 * to [lo, hi] before formatting. */
int  rx_link_cmd_servo(char *buf, size_t cap, int pulse_us, int lo, int hi);
int  rx_link_cmd_level(char *buf, size_t cap, int pulse_us, int lo, int hi);
int  rx_link_cmd_query(char *buf, size_t cap);

#endif
