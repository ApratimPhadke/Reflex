/* reflex-pilot - the UNTRUSTED AI controller.
 *
 *   reflex-pilot <guardian-pid> <channel-id> <hz> [--learn | --learn-from-sim] [--weights FILE]
 *
 * Default: a frozen neural network trained in simulation.
 * --learn: the network starts untrained and improves itself on the live plant with an
 *          evolution strategy. It WILL issue dangerous commands while it explores; that is
 *          the point. The guardian keeps the plant safe, and tells the pilot each tick
 *          whether it had to step in, which is the learner's main feedback signal.
 *
 * Nothing in this process is safety-relevant. It is started by the guardian, is told only
 * where to send proposals, and never touches the guardian's shared memory.
 *
 * The fault modes are test hooks: `rxctl inject <fault>` asks the pilot to misbehave on
 * purpose so the guardian's reaction can be measured.
 *
 * SPDX-License-Identifier: MIT */
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "rx_core.h"
#include "rx_ipc.h"
#include "rx_learner.h"

static rx_learner_t L, L_snap;
static pthread_mutex_t snap_mx = PTHREAD_MUTEX_INITIALIZER;
static volatile int snap_dirty;
static const char *weights_path = "./reflex-policy.bin";

/* File I/O can take milliseconds. Doing it in the control loop would make the pilot miss its
 * deadline, so the loop only takes a snapshot (trylock: it never waits) and this thread writes it. */
static void *saver(void *arg)
{
    (void)arg;
    static rx_learner_t copy;
    for (;;) {
        usleep(200000);
        if (!snap_dirty) continue;
        pthread_mutex_lock(&snap_mx);
        copy = L_snap; snap_dirty = 0;
        pthread_mutex_unlock(&snap_mx);
        rx_learner_save(&copy, weights_path);
    }
    return NULL;
}
static volatile sig_atomic_t g_term;
static void on_term(int s) { (void)s; g_term = 1; }

int main(int argc, char **argv)
{
    if (argc < 4) {
        fprintf(stderr, "reflex-pilot is started by reflex-guardian.\n"
                        "manual start (e.g. for the spoofing test): reflex-pilot $(rxctl endpoint)\n");
        return 2;
    }
    pid_t gpid = (pid_t)atoi(argv[1]);
    int chid = atoi(argv[2]);
    float hz = (float)atof(argv[3]);
    int learn = 0;
    const char *weights = weights_path;
    for (int i = 4; i < argc; i++) {
        if (!strcmp(argv[i], "--learn")) learn = 1;
        else if (!strcmp(argv[i], "--learn-from-sim")) learn = 2;
        else if (!strcmp(argv[i], "--weights") && i + 1 < argc) weights = argv[++i];
    }
    if (hz < 1) return 2;
    rx_cfg_t cfg; rx_cfg_default(&cfg, hz);

    if (learn) {
        rx_learner_init(&L, (uint64_t)time(NULL) * 2654435761u + (uint64_t)getpid(), learn == 2);
        if (rx_learner_load(&L, weights) == 0)
            fprintf(stderr, "[pilot] resumed learning from %s at episode %u\n", weights, L.episode);
    }
    weights_path = weights;
    if (learn) { pthread_t t; pthread_create(&t, NULL, saver, NULL); }
    signal(SIGTERM, on_term);

    rx_client_t *c = NULL;
    for (int i = 0; i < 200 && !c; i++) { c = rx_client_connect(gpid, chid); if (!c) usleep(10000); }
    if (!c) { fprintf(stderr, "[pilot] cannot reach guardian\n"); return 1; }

    rx_proposal_t p;
    memset(&p, 0, sizeof p);
    p.type = RX_MSG_PROPOSAL; p.flags = RX_PF_HELLO;
    rx_state_t st;
    volatile double sink = 0;
    uint32_t saved_ep = L.episode;

    while (!g_term) {
        /* Send the proposal; we stay blocked here until the guardian's next tick hands us fresh state. */
        if (rx_client_exchange(c, &p, &st) != 0) break;         /* refused, or guardian gone */

        float u = learn ? rx_learner_act(&L, &cfg, &st) : rx_policy(&cfg, st.x, st.v, st.sp);

        switch (st.fault) {
        case RX_F_NAN:   u = NAN; break;
        case RX_F_FLIP:  u = -u; break;
        case RX_F_STUCK: u = cfg.theta_max; break;
        case RX_F_HUGE:  u = 40.0f; break;
        case RX_F_SLOW:  usleep(st.fault_arg * 1000u); break;
        case RX_F_SPIN:  for (;;) sink += 1.0;                  /* never returns: burns a whole core */
        default: break;
        }
        p.seq = st.seq; p.u = u; p.flags = learn ? RX_PF_LEARNER : 0;
        if (learn) {
            p.episode = L.episode; p.ep_cost = L.cost_last; p.ep_avg = L.cost_avg;
            if (L.episode >= saved_ep + 10 && pthread_mutex_trylock(&snap_mx) == 0) {     /* survive a crash */
                L_snap = L; snap_dirty = 1; saved_ep = L.episode;
                pthread_mutex_unlock(&snap_mx);
            }
        }
    }
    if (learn) rx_learner_save(&L, weights);
    return 0;
}
