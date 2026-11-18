/* Reflex IPC - native QNX backend.
 *
 * Why this is not a socket:
 *  1. One MsgReceive() gets BOTH the periodic timer pulse and the pilot's message, so
 *     the guardian is a single-threaded, lock-free state machine.
 *  2. Send/Receive/Reply gives synchronisation for free: the pilot is REPLY-blocked
 *     (off the run queue, zero CPU) until the guardian publishes the next state.
 *  3. _NTO_CHF_FIXED_PRIORITY: by default a QNX server inherits the priority of the
 *     client it is serving. That is exactly wrong here - an untrusted, low-priority
 *     pilot must not be able to drag the guardian down (or boost itself through it).
 *  4. The kernel tells us who sent the message (_msg_info.pid). A proposal from any
 *     process other than the pilot the guardian spawned is refused with EPERM.
 *  5. _NTO_CHF_DISCONNECT: when the pilot dies the kernel delivers a pulse, so a
 *     crash is noticed immediately rather than by polling.
 *
 * SPDX-License-Identifier: MIT */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include "rx_ipc.h"
#include "rx_shm.h"

#define RX_PULSE_TICK (_PULSE_CODE_MINAVAIL + 1)

struct rx_server {
    int     chid, coid;
    timer_t timer;
    long    pending;        /* rcvid of the REPLY-blocked pilot, 0 if none */
    int     pending_scoid;
    pid_t   allowed;
    uint64_t t_start;
    rx_shm_t *shm;
};
struct rx_client { int coid; };

typedef union { struct _pulse pulse; rx_proposal_t prop; } rx_msg_u;

const char *rx_ipc_backend(void) { return "qnx-msgpass"; }

rx_server_t *rx_server_create(uint64_t period_ns, int prio, rx_shm_t *shm)
{
    rx_server_t *s = calloc(1, sizeof *s);
    if (!s) return NULL;
    s->shm = shm;
    s->chid = ChannelCreate(_NTO_CHF_FIXED_PRIORITY | _NTO_CHF_DISCONNECT);
    if (s->chid == -1) { perror("ChannelCreate"); free(s); return NULL; }
    s->coid = ConnectAttach(ND_LOCAL_NODE, 0, s->chid, _NTO_SIDE_CHANNEL, 0);
    if (s->coid == -1) { perror("ConnectAttach(self)"); free(s); return NULL; }

    struct sigevent ev;
    SIGEV_PULSE_INIT(&ev, s->coid, prio, RX_PULSE_TICK, 0);
    if (timer_create(CLOCK_MONOTONIC, &ev, &s->timer) == -1) { perror("timer_create"); free(s); return NULL; }
    struct itimerspec its;
    its.it_value.tv_sec = (time_t)(period_ns / 1000000000ull);
    its.it_value.tv_nsec = (long)(period_ns % 1000000000ull);
    its.it_interval = its.it_value;
    int flags = 0;
#ifdef TIMER_PRECISE
    flags |= TIMER_PRECISE;         /* do not let timer tolerance coalesce our tick */
#endif
    s->t_start = rx_now_ns();
    if (timer_settime(s->timer, flags, &its, NULL) == -1) { perror("timer_settime"); free(s); return NULL; }

    atomic_store(&shm->chid, s->chid);
    return s;
}

void rx_server_allow(rx_server_t *s, pid_t pid) { s->allowed = pid; }
uint64_t rx_server_start_ns(const rx_server_t *s) { return s->t_start; }

void rx_server_drop(rx_server_t *s)
{
    if (s->pending) { MsgError(s->pending, ECANCELED); s->pending = 0; }
}

rx_event_e rx_server_wait(rx_server_t *s, rx_proposal_t *prop)
{
    rx_msg_u m;
    struct _msg_info info;
    long rcvid = MsgReceive(s->chid, &m, sizeof m, &info);
    if (rcvid == -1) return errno == EINTR ? RX_EV_NONE : RX_EV_ERROR;

    if (rcvid == 0) {                                   /* pulse */
        switch (m.pulse.code) {
        case RX_PULSE_TICK:
            return RX_EV_TICK;
        case _PULSE_CODE_DISCONNECT:                    /* a client process went away */
            ConnectDetach(m.pulse.scoid);
            if (m.pulse.scoid == s->pending_scoid) { s->pending = 0; s->pending_scoid = 0; return RX_EV_GONE; }
            return RX_EV_NONE;
        default:
            return RX_EV_NONE;
        }
    }

    /* a message: decide whether we are willing to listen to this sender at all */
    if (s->allowed && info.pid != s->allowed) {
        MsgError(rcvid, EPERM);
        atomic_fetch_add(&s->shm->rejected_senders, 1);
        rx_event_push(s->shm, 0, RX_E_SPOOF, info.pid);
        return RX_EV_REJECTED;
    }
    if (info.msglen < (int)sizeof(rx_proposal_t) || m.prop.type != RX_MSG_PROPOSAL) {
        MsgError(rcvid, EINVAL);
        return RX_EV_REJECTED;
    }
    if (s->pending) MsgError(s->pending, EBUSY);        /* one outstanding request per pilot */
    s->pending = rcvid;
    s->pending_scoid = info.scoid;
    *prop = m.prop;
    return RX_EV_PROPOSAL;
}

int rx_server_publish(rx_server_t *s, const rx_state_t *st)
{
    if (!s->pending) return 0;
    long rcvid = s->pending;
    s->pending = 0;
    /* MsgReply never blocks. If the pilot died in the meantime it simply fails. */
    return MsgReply(rcvid, 0, st, sizeof *st) == -1 ? 0 : 1;
}

int rx_server_chid(const rx_server_t *s) { return s->chid; }

rx_client_t *rx_client_connect(pid_t guardian_pid, int chid)
{
    rx_client_t *c = calloc(1, sizeof *c);
    if (!c) return NULL;
    c->coid = ConnectAttach(ND_LOCAL_NODE, guardian_pid, chid, _NTO_SIDE_CHANNEL, 0);
    if (c->coid == -1) { free(c); return NULL; }
    return c;
}

int rx_client_exchange(rx_client_t *c, const rx_proposal_t *prop, rx_state_t *st)
{
    return MsgSend(c->coid, prop, sizeof *prop, st, sizeof *st) == -1 ? -1 : 0;
}
