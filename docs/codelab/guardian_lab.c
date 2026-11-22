/* guardian_lab.c - the QNX message-passing pattern behind Reflex, in one small program.
 * Codelab: docs/codelab/README.md. QNX 8.0 only.
 *
 *   ./guardian_lab            the guardian (server); it starts a pilot and a spoofer by itself
 *   ./guardian_lab --no-fixed the same, without _NTO_CHF_FIXED_PRIORITY (watch the server's priority)
 *
 * What it shows, step by step:
 *   1. a channel with _NTO_CHF_FIXED_PRIORITY | _NTO_CHF_DISCONNECT
 *   2. a periodic timer that delivers pulses to that channel
 *   3. one MsgReceive() that gets both the timer pulse and the client's message
 *   4. the reply is held until the next tick, so the client is paced by the server and never the reverse
 *   5. the kernel says who sent a message (_msg_info.pid): anyone else is refused with MsgError
 *   6. when the client process dies, the kernel delivers a _PULSE_CODE_DISCONNECT pulse
 * SPDX-License-Identifier: MIT */
#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <spawn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include <sys/wait.h>

#define PULSE_TICK   (_PULSE_CODE_MINAVAIL + 1)
#define SERVER_PRIO  30          /* the guardian: high, fixed */
#define CLIENT_PRIO  10          /* the untrusted pilot: low */
#define TICK_MS      10          /* 100 Hz */

extern char **environ;

typedef struct { int value; } proposal_t;          /* client -> server */
typedef struct { unsigned tick; } state_t;         /* server -> client (the reply) */
typedef union { struct _pulse pulse; proposal_t prop; } lab_msg_t;                 /* not msg_t: QNX already defines that in <sys/types.h> */

static int my_prio(void) { struct sched_param sp; int pol; pthread_getschedparam(pthread_self(), &pol, &sp); return sp.sched_priority; }
static void set_prio(int policy, int prio) { struct sched_param sp = { .sched_priority = prio }; pthread_setschedparam(pthread_self(), policy, &sp); }

static pid_t start(const char *self, const char *role, int chid)
{
    char a_pid[16], a_chid[16];
    snprintf(a_pid, sizeof a_pid, "%d", (int)getpid());
    snprintf(a_chid, sizeof a_chid, "%d", chid);
    char *argv[] = { (char *)self, (char *)role, a_pid, a_chid, NULL };
    pid_t pid;
    return posix_spawn(&pid, self, NULL, NULL, argv, environ) == 0 ? pid : -1;
}

/* ------------------------------------------------------------------ client (pilot or spoofer) */
static int client(const char *role, pid_t server, int chid)
{
    set_prio(SCHED_RR, CLIENT_PRIO);
    int coid = ConnectAttach(ND_LOCAL_NODE, server, chid, _NTO_SIDE_CHANNEL, 0);
    if (coid == -1) { perror("ConnectAttach"); return 1; }
    proposal_t p = { 42 };
    state_t st;
    if (!strcmp(role, "spoof")) {
        if (MsgSend(coid, &p, sizeof p, &st, sizeof st) == -1)
            printf("[spoofer pid %d] MsgSend refused: %s\n", (int)getpid(), strerror(errno));
        return 0;
    }
    for (int i = 0; i < 150; i++) {                 /* 150 exchanges, then the process exits */
        p.value = i;
        if (MsgSend(coid, &p, sizeof p, &st, sizeof st) == -1) { perror("[pilot] MsgSend"); return 1; }
    }
    printf("[pilot   pid %d] sent 150 proposals, last reply was tick %u; exiting\n", (int)getpid(), st.tick);
    return 0;                                         /* no goodbye: the kernel tells the server */
}

/* ------------------------------------------------------------------ server (the guardian) */
int main(int argc, char **argv)
{
    if (argc == 4) return client(argv[1], (pid_t)atoi(argv[2]), atoi(argv[3]));
    int fixed = !(argc == 2 && !strcmp(argv[1], "--no-fixed"));
    setvbuf(stdout, NULL, _IOLBF, 0);
    set_prio(SCHED_FIFO, SERVER_PRIO);

    /* Step 1: the channel */
    int chid = ChannelCreate((fixed ? _NTO_CHF_FIXED_PRIORITY : 0) | _NTO_CHF_DISCONNECT);
    if (chid == -1) { perror("ChannelCreate"); return 1; }

    /* Step 2: a timer that sends a pulse to our own channel every TICK_MS */
    int self = ConnectAttach(ND_LOCAL_NODE, 0, chid, _NTO_SIDE_CHANNEL, 0);
    struct sigevent ev;
    SIGEV_PULSE_INIT(&ev, self, SERVER_PRIO, PULSE_TICK, 0);
    timer_t tid;
    if (timer_create(CLOCK_MONOTONIC, &ev, &tid) == -1) { perror("timer_create"); return 1; }
    struct itimerspec its = { { 0, TICK_MS * 1000000L }, { 0, TICK_MS * 1000000L } };
    timer_settime(tid, 0, &its, NULL);
    printf("[guardian pid %d] channel %d (%s), timer pulse every %d ms, running at priority %d\n",
           (int)getpid(), chid, fixed ? "_NTO_CHF_FIXED_PRIORITY" : "priority inheritance ON", TICK_MS, my_prio());

    pid_t pilot = start(argv[0], "pilot", chid), spoofer = -1;
    unsigned tick = 0, msgs = 0, refused = 0;
    int pending = 0, shown_prio = 0, pilot_gone = 0, pilot_scoid = -1;

    while (tick < 300 && !(pilot_gone && tick > 200)) {
        /* Step 3: one blocking call receives everything */
        lab_msg_t m;
        struct _msg_info info;
        int rcvid = MsgReceive(chid, &m, sizeof m, &info);
        if (rcvid == -1) { perror("MsgReceive"); break; }

        if (rcvid == 0) {                             /* a pulse */
            if (m.pulse.code == PULSE_TICK) {
                tick++;
                /* Step 4: the pilot's reply waits for the tick: it runs once per period, at our pace */
                if (pending) { state_t st = { tick }; MsgReply(pending, 0, &st, sizeof st); pending = 0; }
                if (tick == 50) spoofer = start(argv[0], "spoof", chid);
                if (tick % 100 == 0) printf("[guardian] tick %3u: %u proposals accepted, %u refused\n", tick, msgs, refused);
            } else if (m.pulse.code == _PULSE_CODE_DISCONNECT) {
                /* Step 6: the kernel tells us a client process went away (crash, kill -9, exit). Every client
                 * that connected sends one, so check WHICH: compare its server connection id with the pilot's. */
                if (m.pulse.scoid == pilot_scoid) {
                    printf("[guardian] tick %3u: disconnect pulse from the pilot - fall back to the safe controller\n", tick);
                    if (pending) { MsgError(pending, ECANCELED); pending = 0; }
                    pilot_gone = 1;
                } else {
                    printf("[guardian] tick %3u: disconnect pulse from another client (scoid %d) - ignored\n", tick, m.pulse.scoid);
                }
                ConnectDetach(m.pulse.scoid);
            }
            continue;
        }

        /* Step 5: a message. The kernel, not the sender, says who sent it. */
        if (info.pid != pilot) {
            printf("[guardian] tick %3u: message from pid %d refused (only pid %d may propose)\n", tick, info.pid, (int)pilot);
            MsgError(rcvid, EPERM);
            refused++;
            continue;
        }
        pilot_scoid = info.scoid;                     /* remember the pilot's connection for step 6 */
        if (!shown_prio) {
            printf("[guardian] handling a message from a priority-%d client, the guardian runs at priority %d\n", CLIENT_PRIO, my_prio());
            shown_prio = 1;
        }
        msgs++;
        if (pending) MsgError(pending, EBUSY);
        pending = rcvid;                              /* reply on the next tick */
    }
    timer_delete(tid);
    waitpid(pilot, NULL, 0);
    if (spoofer > 0) waitpid(spoofer, NULL, 0);
    printf("[guardian] done after %u ticks: %u proposals accepted, %u refused\n", tick, msgs, refused);
    return 0;
}
