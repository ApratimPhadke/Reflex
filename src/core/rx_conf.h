/* Reflex rig calibration (reflex.conf): parse, check, format, and convert between the I/O node's
 * raw units (sensor mm, servo pulse us) and the guardian's units (m, rad).
 * Pure functions on text buffers, no OS calls; file I/O is the caller's job.
 *
 * reflex.conf is plain "key = value" text, '#' starts a comment, every key below is required:
 *   level_us        servo pulse at which the beam is level                       1000..2000
 *   us_per_mrad     servo microseconds per milliradian of beam angle             0.3..10
 *   servo_dir       +1 / -1: sign of beam angle per pulse increase
 *   center_mm       raw sensor reading with the ball at the pivot                 1..1999
 *   sensor_dir      +1 / -1: so that a positive beam angle makes position increase
 *   range_min_mm    raw readings outside [range_min_mm, range_max_mm] mean "no ball"
 *   range_max_mm
 *   half_len_m      usable half length of the beam, m                            0.10..0.20
 *                   (0.20 = the safety kernel's default; a calibration can only make it shorter)
 *   calibrated_at   ISO date-time the file was written (informational)
 *   board           board name from the node's "I" line (informational)
 * SPDX-License-Identifier: MIT */
#ifndef RX_CONF_H
#define RX_CONF_H
#include <stddef.h>

#define RX_PULSE_MIN 1000     /* the host never sends a pulse outside this; the node clamps on its own too */
#define RX_PULSE_MAX 2000

typedef struct {
    int   level_us;
    float us_per_mrad;
    int   servo_dir;
    int   center_mm;
    int   sensor_dir;
    int   range_min_mm, range_max_mm;
    float half_len_m;
    char  calibrated_at[40];
    char  board[32];
} rx_conf_t;

/* Values matching the v1 firmware's built-in constants. Used only with --uncalibrated. */
void  rx_conf_default(rx_conf_t *c);
/* Parse a whole file's text. 0 = ok; otherwise -1 and a one-line reason in err. */
int   rx_conf_parse(rx_conf_t *c, const char *text, char *err, size_t errcap);
/* Check a filled-in struct (rx_conf_parse calls this). 0 = ok. */
int   rx_conf_check(const rx_conf_t *c, char *err, size_t errcap);
/* Write the file's text. Returns the length, or -1 if cap is too small. */
int   rx_conf_format(const rx_conf_t *c, char *buf, size_t cap);
/* raw sensor mm -> ball position in m along the beam; *valid = 0 outside the range window */
float rx_conf_position(const rx_conf_t *c, int raw_mm, int *valid);
/* Read and parse a file (the only function here that touches the file system). 0 = ok. */
int   rx_conf_load(rx_conf_t *c, const char *path, char *err, size_t errcap);
/* Write a file atomically (temp file + rename). 0 = ok. */
int   rx_conf_save(const rx_conf_t *c, const char *path, char *err, size_t errcap);
/* beam angle in rad -> servo pulse in us, clamped to [RX_PULSE_MIN, RX_PULSE_MAX]; NaN -> level */
int   rx_conf_pulse(const rx_conf_t *c, float u_rad);

#endif
