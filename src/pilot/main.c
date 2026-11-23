/* reflex-pilot - the UNTRUSTED AI controller.
 *
 *   reflex-pilot <guardian-pid> <channel-id> <hz> [--hardware] [--plant joint] [--learn | --learn-from-sim] [--weights FILE]
 *   reflex-pilot --udp <guardian a.b.c.d:port> [--bind a.b.c.d:port] [--hz N] [same options]
 *
 * The first form is how the guardian spawns it on the same QNX system (message passing). The second is
 * for a pilot somewhere else, e.g. in a Linux guest under QNX Hypervisor (pilot_transport = udp in the
 * guardian's reflex.conf, docs/HYPERVISOR.md); it builds natively for Linux with `make linux-pilot`.
 * Over UDP the pilot never gives up: if the guardian goes quiet it keeps sending a hello every 100 ms.
 *
 * Default: a frozen neural network trained in simulation.
 * --hardware: the plant is the real rig (serial backend). Fly the weights trained for
 *          rx_cfg_hardware() at 50 Hz instead of the simulator ones, and start --learn-from-sim
 *          from them. Only a hint for performance: the guardian judges every command the same way.
 * --learn: the network starts untrained and improves itself on the live plant with an
 *          evolution strategy. It WILL issue dangerous commands while it explores; that is
 *          the point. The guardian keeps the plant safe, and tells the pilot each tick
 *          whether it had to step in, which is the learner's main feedback signal.
 *
 * Nothing in this process is safety-relevant. It is started by the guardian, is told only
 * where to send proposals, and never touches the guardian's shared memory.
 *
 * The fault modes are test hooks: `rxctl inject <fault>` asks the pilot to misbehave on
 * purpose so the guardian's reaction can be measured. A fault can also start inside the pilot's own
 * machine: SIGUSR1 makes it sign-invert its output ("bad model", `rxctl guest badmodel`), SIGUSR2
 * heals it. The guardian is not told; it has to notice from the commands alone.
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
#include "rx_ipc_udp.h"
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
static volatile sig_atomic_t g_term, g_badmodel;
static void on_term(int s) { (void)s; g_term = 1; }
static void on_usr(int s) { g_badmodel = s == SIGUSR1; }

static void usage(void)
{
    fprintf(stderr, "reflex-pilot is started by reflex-guardian.\n"
                    "manual start (e.g. for the spoofing test): reflex-pilot $(rxctl endpoint)\n"
                    "pilot on another machine or VM:  reflex-pilot --udp <guardian a.b.c.d:port> [--bind a.b.c.d:port] [--hz N]\n"
                    "                                 [--hardware] [--plant joint] [--learn | --learn-from-sim] [--weights FILE]\n");
}

int main(int argc, char **argv)
{
    signal(SIGPIPE, SIG_IGN);            /* inherited stderr may have no reader (see the guardian) */
    const char *udp_guardian = NULL, *udp_bind = "0.0.0.0:47801";
    int first = 4;
    pid_t gpid = 0; int chid = 0; float hz = 0;
    if (argc >= 3 && !strcmp(argv[1], "--udp")) { udp_guardian = argv[2]; first = 3; }
    else if (argc < 4) { usage(); return 2; }
    else { gpid = (pid_t)atoi(argv[1]); chid = atoi(argv[2]); hz = (float)atof(argv[3]); }
    int learn = 0, hardware = 0, joint = 0;
    const char *weights = weights_path;
    for (int i = first; i < argc; i++) {
        if (udp_guardian && !strcmp(argv[i], "--bind") && i + 1 < argc) udp_bind = argv[++i];
        else if (udp_guardian && !strcmp(argv[i], "--hz") && i + 1 < argc) hz = (float)atof(argv[++i]);
        else if (!strcmp(argv[i], "--learn")) learn = 1;
        else if (!strcmp(argv[i], "--hardware")) hardware = 1;
        else if (!strcmp(argv[i], "--plant") && i + 1 < argc) joint = !strcmp(argv[++i], "joint");
        else if (!strcmp(argv[i], "--learn-from-sim")) learn = 2;
        else if (!strcmp(argv[i], "--weights") && i + 1 < argc) weights = argv[++i];
        else if (udp_guardian) { usage(); return 2; }
    }
    if (udp_guardian && hz <= 0) hz = hardware ? 50.0f : 1000.0f;   /* must match the guardian's --hz */
    if (hz < 1) return 2;
    rx_cfg_t cfg; rx_cfg_default(&cfg, hz);
    if (hardware) rx_cfg_hardware(&cfg);
    if (joint) { rx_cfg_joint(&cfg, hz); learn = 0; }        /* the learner is beam-only */
    const rx_net_t *net = rx_policy_net(joint ? RX_NET_JOINT : hardware ? RX_NET_HW : RX_NET_SIM);

    if (learn) {
        rx_learner_init(&L, (uint64_t)time(NULL) * 2654435761u + (uint64_t)getpid(), learn == 2 ? net : NULL);
        if (rx_learner_load(&L, weights) == 0)
            fprintf(stderr, "[pilot] resumed learning from %s at episode %u\n", weights, L.episode);
    }
    weights_path = weights;
    if (learn) { pthread_t t; pthread_create(&t, NULL, saver, NULL); }
    {   /* no SA_RESTART: a signal must interrupt the wait for the next state */
        struct sigaction sa; memset(&sa, 0, sizeof sa);
        sa.sa_handler = on_term; sigaction(SIGTERM, &sa, NULL); sigaction(SIGINT, &sa, NULL);
        /* UDP only: a signal unblocks a QNX MsgSend with EINTR, which would end a local pilot; poll() just retries */
        if (udp_guardian) { sa.sa_handler = on_usr; sigaction(SIGUSR1, &sa, NULL); sigaction(SIGUSR2, &sa, NULL); }
    }

    rx_client_t *c = NULL;
    rx_udp_client_t *uc = NULL;
    if (udp_guardian) {
        char err[200];
        if (!(uc = rx_udp_client_open(udp_guardian, udp_bind, err, sizeof err))) { fprintf(stderr, "[pilot] %s\n", err); return 1; }
        fprintf(stderr, "[pilot] udp %s -> guardian %s, %.0f Hz, instance %08x%s\n", udp_bind, udp_guardian, hz,
                rx_udp_client_boot_id(uc), learn ? ", learning" : "");
    } else {
        for (int i = 0; i < 200 && !c; i++) { c = rx_client_connect(gpid, chid); if (!c) usleep(10000); }
        if (!c) { fprintf(stderr, "[pilot] cannot reach guardian\n"); return 1; }
    }

    rx_proposal_t p;
    memset(&p, 0, sizeof p);
    p.type = RX_MSG_PROPOSAL; p.flags = RX_PF_HELLO;
    rx_state_t st;
    volatile double sink = 0;
    uint32_t saved_ep = L.episode;

    while (!g_term) {
        /* Send the proposal; we stay blocked here until the guardian's next tick hands us fresh state. */
        if (uc ? rx_udp_client_exchange(uc, &p, &st, &g_term) != 0
               : rx_client_exchange(c, &p, &st) != 0) break;    /* refused, guardian gone, or told to stop */

        float u = learn ? rx_learner_act(&L, &cfg, &st) : rx_policy_with(&cfg, net, st.x, st.v, st.sp);
        if (g_badmodel && st.fault == RX_F_NONE) st.fault = RX_F_FLIP;   /* a fault from inside the pilot's machine */

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
