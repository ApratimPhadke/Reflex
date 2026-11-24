/* See rx_sonar.h. SPDX-License-Identifier: MIT */
#include <math.h>
#include <string.h>
#include "rx_sonar.h"

void rx_sonar_cfg_default(rx_sonar_cfg_t *c)
{
    c->lo_mm = 40; c->hi_mm = 200;          /* step3-roll: ball from 46 mm (hinge stop) to 199 mm (far stop) */
    c->max_speed_mm_s = 400;                /* a ball on +-4 deg over 10 cm reaches ~0.3 m/s */
    c->slack_mm = 8; c->agree_mm = 4;
}

void rx_sonar_init(rx_sonar_filt_t *f, const rx_sonar_cfg_t *c)
{
    memset(f, 0, sizeof *f);
    f->c = *c;
}

static float med3(const float *w, int n)
{
    if (n < 3) return w[n - 1];
    float a = w[0], b = w[1], c = w[2];
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}

static void accept(rx_sonar_filt_t *f, float z)
{
    if (f->nwin < 3) f->win[f->nwin++] = z;
    else { f->win[0] = f->win[1]; f->win[1] = f->win[2]; f->win[2] = z; }
    f->last = z; f->have = 1; f->ticks_since = 0; f->ncand = 0;
    f->n_ok++;
}

int rx_sonar_step(rx_sonar_filt_t *f, float raw_mm, float dt_s, float *out_mm)
{
    f->n_in++;
    f->ticks_since++;
    if (!(raw_mm >= f->c.lo_mm && raw_mm <= f->c.hi_mm)) { f->n_gated++; return 0; }   /* also NaN */
    if (f->have) {
        float reach = f->c.max_speed_mm_s * dt_s * (float)f->ticks_since + f->c.slack_mm;
        if (fabsf(raw_mm - f->last) > reach) {
            if (f->ncand && fabsf(raw_mm - f->cand) <= f->c.agree_mm) {
                if (++f->ncand >= 3) {                                    /* three agreeing in a row: the ball moved */
                    f->n_reacq++;
                    f->nwin = 0;                                          /* old window is history */
                    accept(f, raw_mm);
                    *out_mm = raw_mm;
                    return 1;
                }
            } else { f->cand = raw_mm; f->ncand = 1; }
            f->n_spikes++;
            return 0;
        }
    }
    if (f->ticks_since > 3) f->nwin = 0;                  /* after a gap the old window is stale: do not let it outvote */
    accept(f, raw_mm);
    *out_mm = med3(f->win, f->nwin);
    return 1;
}
