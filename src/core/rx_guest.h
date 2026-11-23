/* Reflex guest supervision (pilot_transport = udp): is the pilot's machine alive, when may it be restarted.
 * Pure functions on a small state machine, no OS calls, so the rate limits are unit-tested offline
 * (rx_simtest). The guardian calls rx_guest_step() once per control tick; running the restart command
 * is the guardian's job (a helper thread, never the control loop).
 *
 *   WAITING      guardian started, nothing heard yet; after guest_boot_s it counts as DOWN
 *   UP           valid proposals arriving
 *   DOWN         none for guest_timeout_s: restart, then wait at least RX_GUEST_MIN_GAP_S (or guest_boot_s
 *                if longer) before the next restart
 *   QUARANTINE   a 6th restart would be needed within RX_GUEST_WINDOW_S: no more restarts; the guardian also
 *                quarantines the pilot, so the fallback flies until an operator runs `rxctl release`
 *
 * A guest that is UP but has used up its strikes (three unsafe commands in ten seconds, outside training
 * mode) is restarted through the same limiter. Nothing here makes a safety decision: whether a proposal
 * reaches the actuator is decided by rx_guard_step() alone. This only decides whether to reboot a guest.
 * SPDX-License-Identifier: MIT */
#ifndef RX_GUEST_H
#define RX_GUEST_H
#include <stdint.h>

#define RX_GUEST_MIN_GAP_S     15.0    /* at most one restart per 15 s */
#define RX_GUEST_MAX_RESTARTS  5       /* ... and at most 5 per window, then quarantine */
#define RX_GUEST_WINDOW_S      600.0

typedef enum { RX_GUEST_NA = 0, RX_GUEST_WAITING, RX_GUEST_UP, RX_GUEST_DOWN, RX_GUEST_QUARANTINE } rx_guest_state_e;
static const char *const rx_guest_name[] = { "-", "waiting", "up", "DOWN", "QUARANTINED" };

enum { RX_GA_DOWN = 1, RX_GA_UP = 2, RX_GA_RESTART = 4, RX_GA_QUARANTINE = 8 };   /* rx_guest_step() result bits */
enum { RX_GR_TIMEOUT = 1, RX_GR_NEVER_UP = 2, RX_GR_STRIKES = 3 };               /* why a restart was asked for */

typedef struct {
    float    timeout_s, boot_s;
    uint8_t  state;
    double   last_ok_s;              /* last valid proposal (or guardian start) */
    double   down_since_s;
    double   next_restart_s;         /* no restart before this */
    double   hist[RX_GUEST_MAX_RESTARTS];   /* times of the most recent restarts (ring) */
    uint32_t nhist, restarts;
    int      ever_up;                /* a valid proposal has been seen since the guardian started */
} rx_guest_t;

void     rx_guest_init(rx_guest_t *g, float timeout_s, float boot_s, double now_s);
/* got_valid: a valid proposal was taken this tick. strikes: the guard asked for a pilot restart.
 * Returns RX_GA_* bits; *reason is set (RX_GR_*) when RX_GA_RESTART is returned. */
unsigned rx_guest_step(rx_guest_t *g, double now_s, int got_valid, int strikes, int *reason);
/* operator released the quarantine: restart history cleared, a down guest may be restarted at once */
void     rx_guest_release(rx_guest_t *g, double now_s);

#endif
