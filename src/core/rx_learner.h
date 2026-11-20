/* Online learner for the untrusted pilot: a 3-12-1 tanh network whose weights are
 * improved on the live plant with an antithetic evolution strategy.
 * Pure C, no OS calls, so it can be tested offline against the simulator.
 * SPDX-License-Identifier: MIT */
#ifndef RX_LEARNER_H
#define RX_LEARNER_H
#include <stdint.h>
#include "rx_core.h"

#define RX_L_IN 3
#define RX_L_H  12
#define RX_L_NP (RX_L_H * RX_L_IN + RX_L_H + RX_L_H + 1)

typedef struct {
    float    mean[RX_L_NP];     /* current best guess of the weights */
    float    eps[RX_L_NP];      /* perturbation direction of the running pair */
    float    cur[RX_L_NP];      /* weights being flown right now */
    float    mom[RX_L_NP];      /* gradient momentum */
    float    best[RX_L_NP];     /* best weights seen so far */
    float    best_cost; int worse;
    float    sigma, lr;
    int      phase;             /* 0: flying mean+sigma*eps, 1: flying mean-sigma*eps */
    float    cost_plus;
    double   acc; uint32_t n;   /* cost accumulator of the running episode */
    float    last_sp; int have_sp;
    uint32_t episode;           /* finished episodes */
    float    cost_last;         /* cost of the last finished episode */
    float    cost_avg;          /* smoothed cost (the learning curve) */
    int      eq_corrected;      /* 1: equilibrium-corrected network like rx_policy_with() (started from frozen
                                   weights, which are trained in that form); 0: free form (learning from scratch) */
    uint32_t safe_ticks_last;   /* ticks the guardian had to take over in the last episode */
    uint32_t safe_ticks;
    uint64_t rng;
} rx_learner_t;

/* pretrained: start from these frozen weights (rx_policy_net()), or NULL to start untrained */
void  rx_learner_init(rx_learner_t *L, uint64_t seed, const rx_net_t *pretrained);
/* called once per control tick with the state the guardian handed out; returns the command */
float rx_learner_act(rx_learner_t *L, const rx_cfg_t *c, const rx_state_t *st);
int   rx_learner_save(const rx_learner_t *L, const char *path);
int   rx_learner_load(rx_learner_t *L, const char *path);
#endif
