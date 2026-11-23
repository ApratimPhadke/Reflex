/* See rx_guest.h. SPDX-License-Identifier: MIT */
#include <string.h>
#include "rx_guest.h"

void rx_guest_init(rx_guest_t *g, float timeout_s, float boot_s, double now_s)
{
    memset(g, 0, sizeof *g);
    g->timeout_s = timeout_s; g->boot_s = boot_s;
    g->state = RX_GUEST_WAITING;
    g->last_ok_s = g->down_since_s = now_s;
}

static double gap(const rx_guest_t *g) { return g->boot_s > RX_GUEST_MIN_GAP_S ? g->boot_s : RX_GUEST_MIN_GAP_S; }

/* restarts within the window ending now */
static uint32_t recent(const rx_guest_t *g, double now_s)
{
    uint32_t n = 0, k = g->nhist < RX_GUEST_MAX_RESTARTS ? g->nhist : RX_GUEST_MAX_RESTARTS;
    for (uint32_t i = 0; i < k; i++) if (now_s - g->hist[i] < RX_GUEST_WINDOW_S) n++;
    return n;
}

unsigned rx_guest_step(rx_guest_t *g, double now_s, int got_valid, int strikes, int *reason)
{
    unsigned acts = 0;
    if (got_valid) g->last_ok_s = now_s;
    if (g->state == RX_GUEST_QUARANTINE) return 0;               /* only an operator gets us out */

    if (got_valid) {
        g->ever_up = 1;
        if (g->state != RX_GUEST_UP) { g->state = RX_GUEST_UP; acts |= RX_GA_UP; }
    } else if (g->state == RX_GUEST_UP && now_s - g->last_ok_s > g->timeout_s) {
        g->state = RX_GUEST_DOWN; g->down_since_s = now_s; acts |= RX_GA_DOWN;
    } else if (g->state == RX_GUEST_WAITING && now_s - g->last_ok_s > g->boot_s) {
        g->state = RX_GUEST_DOWN; g->down_since_s = now_s; acts |= RX_GA_DOWN;
    }

    int want = g->state == RX_GUEST_DOWN || (g->state == RX_GUEST_UP && strikes);
    if (!want || now_s < g->next_restart_s) return acts;
    if (recent(g, now_s) >= RX_GUEST_MAX_RESTARTS) {
        g->state = RX_GUEST_QUARANTINE;
        return acts | RX_GA_QUARANTINE;
    }
    g->hist[g->nhist++ % RX_GUEST_MAX_RESTARTS] = now_s;
    g->restarts++;
    g->next_restart_s = now_s + gap(g);
    if (reason) *reason = g->state == RX_GUEST_UP ? RX_GR_STRIKES : g->ever_up ? RX_GR_TIMEOUT : RX_GR_NEVER_UP;
    return acts | RX_GA_RESTART;
}

void rx_guest_release(rx_guest_t *g, double now_s)
{
    g->nhist = 0;
    g->next_restart_s = now_s;
    g->state = now_s - g->last_ok_s <= g->timeout_s ? RX_GUEST_UP : RX_GUEST_DOWN;
    if (g->state == RX_GUEST_DOWN) g->down_since_s = now_s;
}
