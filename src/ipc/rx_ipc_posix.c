/* Reflex IPC - Linux reference backend (timerfd + SOCK_SEQPACKET).
 * Exists so the identical guardian/pilot can be run on Linux as an A/B baseline
 * and so the logic can be tested on a development host.
 * SPDX-License-Identifier: MIT */
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/timerfd.h>
#include <sys/un.h>
#include "rx_ipc.h"
#include "rx_shm.h"

#define RX_SOCK_PATH "/tmp/reflex.sock"

struct rx_server { int tfd, lfd, cfd; int pending; uint64_t backlog; _Atomic pid_t allowed; uint64_t t_start; rx_shm_t *shm; };
struct rx_client { int fd; };

const char *rx_ipc_backend(void) { return "posix-seqpacket"; }

rx_server_t *rx_server_create(uint64_t period_ns, int prio, rx_shm_t *shm)
{
    (void)prio;
    rx_server_t *s = calloc(1, sizeof *s);
    if (!s) return NULL;
    s->shm = shm; s->cfd = -1;
    s->tfd = timerfd_create(CLOCK_MONOTONIC, 0);
    struct itimerspec its;
    its.it_value.tv_sec = period_ns / 1000000000ull;
    its.it_value.tv_nsec = period_ns % 1000000000ull;
    its.it_interval = its.it_value;
    s->t_start = rx_now_ns();
    if (s->tfd < 0 || timerfd_settime(s->tfd, 0, &its, NULL) < 0) { perror("timerfd"); free(s); return NULL; }

    struct sockaddr_un a = { .sun_family = AF_UNIX };
    strncpy(a.sun_path, RX_SOCK_PATH, sizeof a.sun_path - 1);
    unlink(RX_SOCK_PATH);
    s->lfd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK, 0);
    if (s->lfd < 0 || bind(s->lfd, (struct sockaddr *)&a, sizeof a) < 0 || listen(s->lfd, 4) < 0) {
        perror("reflex socket"); free(s); return NULL;
    }
    atomic_store(&shm->chid, 0);
    return s;
}

void rx_server_allow(rx_server_t *s, pid_t pid) { s->allowed = pid; }
uint64_t rx_server_start_ns(const rx_server_t *s) { return s->t_start; }
void rx_server_drop(rx_server_t *s) { if (s->cfd >= 0) { close(s->cfd); s->cfd = -1; } s->pending = 0; }

rx_event_e rx_server_wait(rx_server_t *s, rx_proposal_t *prop)
{
    if (s->backlog) { s->backlog--; return RX_EV_TICK; }
    struct pollfd p[3] = { { s->tfd, POLLIN, 0 }, { s->lfd, POLLIN, 0 }, { s->cfd, POLLIN, 0 } };
    if (poll(p, 3, -1) < 0) return errno == EINTR ? RX_EV_NONE : RX_EV_ERROR;

    if (p[2].revents) {                                 /* pilot traffic first: it arrived before the tick */
        ssize_t n = recv(s->cfd, prop, sizeof *prop, MSG_DONTWAIT);
        if (n == 0 || (n < 0 && errno != EAGAIN)) { close(s->cfd); s->cfd = -1; s->pending = 0; return RX_EV_GONE; }
        if (n == (ssize_t)sizeof *prop && prop->type == RX_MSG_PROPOSAL) { s->pending = 1; return RX_EV_PROPOSAL; }
        return RX_EV_REJECTED;
    }
    if (p[1].revents) {
        int fd = accept(s->lfd, NULL, NULL);
        if (fd >= 0) {
            struct ucred uc; socklen_t l = sizeof uc;
            int ok = getsockopt(fd, SOL_SOCKET, SO_PEERCRED, &uc, &l) == 0 && s->allowed != 0 && uc.pid == s->allowed;   /* nobody before the first spawn */
            if (!ok) { close(fd); atomic_fetch_add(&s->shm->rejected_senders, 1); rx_event_push(s->shm, 0, RX_E_SPOOF, uc.pid); return RX_EV_REJECTED; }
            if (s->cfd >= 0) close(s->cfd);
            s->cfd = fd; s->pending = 0;
        }
        return RX_EV_NONE;
    }
    if (p[0].revents) {
        uint64_t exp = 0;
        if (read(s->tfd, &exp, sizeof exp) == (ssize_t)sizeof exp && exp > 1) s->backlog = exp - 1;
        return RX_EV_TICK;
    }
    return RX_EV_NONE;
}

int rx_server_publish(rx_server_t *s, const rx_state_t *st)
{
    if (s->cfd < 0 || !s->pending) return 0;
    s->pending = 0;
    return send(s->cfd, st, sizeof *st, MSG_DONTWAIT | MSG_NOSIGNAL) == (ssize_t)sizeof *st;
}

int rx_server_chid(const rx_server_t *s) { (void)s; return 0; }

rx_client_t *rx_client_connect(pid_t guardian_pid, int chid)
{
    (void)guardian_pid; (void)chid;
    rx_client_t *c = calloc(1, sizeof *c);
    struct sockaddr_un a = { .sun_family = AF_UNIX };
    strncpy(a.sun_path, RX_SOCK_PATH, sizeof a.sun_path - 1);
    c->fd = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    if (c->fd < 0 || connect(c->fd, (struct sockaddr *)&a, sizeof a) < 0) { free(c); return NULL; }
    return c;
}

int rx_client_exchange(rx_client_t *c, const rx_proposal_t *prop, rx_state_t *st)
{
    if (send(c->fd, prop, sizeof *prop, MSG_NOSIGNAL) != (ssize_t)sizeof *prop) return -1;
    return recv(c->fd, st, sizeof *st, 0) == (ssize_t)sizeof *st ? 0 : -1;
}
