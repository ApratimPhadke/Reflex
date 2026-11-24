/* Ultrasonic ball-position filter for the beam-direct rig (HC-SR04 read by the host). Pure: no OS calls, no
 * globals, tested offline by rx_simtest.
 *
 * One raw reading per control tick (mm to the ball's near surface, < 0 = no echo) becomes either a clean
 * position or "no measurement this tick":
 *   1. range gate: outside [lo_mm, hi_mm] it is not the ball (the room behind it, the end stop, a hand);
 *   2. spike gate: further from the last accepted reading than the ball can travel in the time since
 *      (max_speed_mm_s, plus slack_mm) is refused, unless three consecutive refused readings agree within
 *      agree_mm (the ball really moved while unseen: re-acquire; two were not enough on the rig, where a
 *      reflector at 167 mm can answer twice in a row);
 *   3. median of the last 3 accepted readings.
 * Velocity is not estimated here: the guardian's critically damped g-h estimator (rx_est_update) does that
 * from these positions, and predicts through the ticks without one (rx_est_predict).
 * SPDX-License-Identifier: MIT */
#ifndef RX_SONAR_H
#define RX_SONAR_H
#include <stdint.h>

typedef struct {
    float lo_mm, hi_mm, max_speed_mm_s, slack_mm, agree_mm;
} rx_sonar_cfg_t;

typedef struct {
    rx_sonar_cfg_t c;
    float win[3]; int nwin;          /* last accepted readings (median window) */
    float last;   int have;          /* last accepted reading */
    float cand;   int ncand;         /* refused reading waiting for a second, agreeing one */
    uint32_t ticks_since;            /* ticks since the last accepted reading */
    uint32_t n_in, n_gated, n_spikes, n_reacq, n_ok;
} rx_sonar_filt_t;

void rx_sonar_cfg_default(rx_sonar_cfg_t *c);          /* 40..200 mm, 400 mm/s, 8 mm slack, 4 mm agree */
#define RX_SONAR_LOST_TICKS 5                          /* no accepted reading for this many ticks = ball lost */
void rx_sonar_init(rx_sonar_filt_t *f, const rx_sonar_cfg_t *c);
/* dt_s: control period. Returns 1 and *out_mm when this tick has a clean measurement, else 0. */
int  rx_sonar_step(rx_sonar_filt_t *f, float raw_mm, float dt_s, float *out_mm);

#endif
