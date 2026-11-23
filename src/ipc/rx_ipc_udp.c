/* Reflex pilot link over UDP. See rx_ipc_udp.h (threads and why) and src/core/rx_wire.h (format).
 * Builds on QNX 8.0 (io-sock, -lsocket) and Linux.
 * SPDX-License-Identifier: MIT */
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include "rx_ipc_udp.h"
#include "rx_wire.h"

/* ---- single-writer seqlock mailbox. The reader never waits: if it catches the writer mid-copy it
 * reports "nothing new" and the value is picked up on the next tick. All accesses are atomic words,
 * so there is no data race in the C11 sense. */
#define BOX_WORDS 12
typedef struct { _Atomic uint32_t ver; _Atomic uint32_t w[BOX_WORDS]; } box_t;
_Static_assert(sizeof(rx_proposal_t) + 4 <= BOX_WORDS * 4 && sizeof(rx_state_t) <= BOX_WORDS * 4, "mailbox too small");

static void box_put(box_t *b, const void *src, size_t n)
{
    uint32_t tmp[BOX_WORDS] = { 0 };
    memcpy(tmp, src, n);
    uint32_t v = atomic_load_explicit(&b->ver, memory_order_relaxed);
    atomic_store_explicit(&b->ver, v + 1, memory_order_relaxed);         /* odd: being written */
    atomic_thread_fence(memory_order_release);
    for (int i = 0; i < BOX_WORDS; i++) atomic_store_explicit(&b->w[i], tmp[i], memory_order_relaxed);
    atomic_store_explicit(&b->ver, v + 2, memory_order_release);
}

/* version of the copied value (even, nonzero), or 0 if empty / caught mid-write */
static uint32_t box_get(box_t *b, void *dst, size_t n)
{
    for (int attempt = 0; attempt < 2; attempt++) {
        uint32_t tmp[BOX_WORDS];
        uint32_t v1 = atomic_load_explicit(&b->ver, memory_order_acquire);
        if (v1 == 0) return 0;
        if (v1 & 1) continue;
        for (int i = 0; i < BOX_WORDS; i++) tmp[i] = atomic_load_explicit(&b->w[i], memory_order_relaxed);
        atomic_thread_fence(memory_order_acquire);
        if (atomic_load_explicit(&b->ver, memory_order_relaxed) == v1) { memcpy(dst, tmp, n); return v1; }
    }
    return 0;
}

uint32_t rx_udp_new_boot_id(void)
{
    uint32_t id = 0;
    int fd = open("/dev/urandom", O_RDONLY);
    if (fd >= 0) { if (read(fd, &id, sizeof id) != (ssize_t)sizeof id) id = 0; close(fd); }
    if (!id) {
        struct timespec ts; clock_gettime(CLOCK_REALTIME, &ts);
        id = (uint32_t)ts.tv_nsec * 2654435761u ^ (uint32_t)ts.tv_sec ^ ((uint32_t)getpid() << 16);
    }
    return id ? id : 1;
}

static int open_bound(const char *bind_s, char *err, size_t errcap)
{
    uint32_t ip; uint16_t port;
    if (rx_wire_addr_parse(bind_s, &ip, &port) != 0) { snprintf(err, errcap, "bad address '%s' (want a.b.c.d:port)", bind_s); return -1; }
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) { snprintf(err, errcap, "socket: %s", strerror(errno)); return -1; }
    struct sockaddr_in a; memset(&a, 0, sizeof a);
    a.sin_family = AF_INET; a.sin_addr.s_addr = htonl(ip); a.sin_port = htons(port);
    if (bind(fd, (struct sockaddr *)&a, sizeof a) != 0) { snprintf(err, errcap, "bind %s: %s", bind_s, strerror(errno)); close(fd); return -1; }
    return fd;
}

/* ================================================================ guardian side */
struct rx_udp {
    int fd;
    struct sockaddr_in peer;
    rx_wire_filter_t filt;          /* rx thread only */
    box_t inbox, outbox;
    uint32_t taken_ver;             /* control thread only */
    sem_t tx_sem;
    uint32_t boot_id, tx_seq;       /* tx thread only (boot_id fixed) */
    _Atomic int stop;
    int prio;
    pthread_t rx_t, tx_t;
    rx_shm_t *shm;
};

static void helper_prio(int prio)
{
    struct sched_param sp = { .sched_priority = prio };
    if (prio > 0) pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);   /* best effort (needs privileges) */
}

static void *rx_main(void *arg)
{
    rx_udp_t *u = arg;
    helper_prio(u->prio);
    unsigned char buf[512];
    while (!atomic_load(&u->stop)) {
        struct sockaddr_in src; socklen_t sl = sizeof src;
        ssize_t n = recvfrom(u->fd, buf, sizeof buf, 0, (struct sockaddr *)&src, &sl);
        if (n < 0) continue;                                    /* timeout (SO_RCVTIMEO) or EINTR: check stop */
        int new_boot = 0;
        int v = sl >= (socklen_t)sizeof src && src.sin_family == AF_INET
              ? rx_wire_accept(&u->filt, ntohl(src.sin_addr.s_addr), ntohs(src.sin_port), buf, (size_t)n, RX_WIRE_PROPOSAL, &new_boot)
              : RX_WIRE_BAD_PEER;
        switch (v) {
        case RX_WIRE_OK: {
            struct { rx_proposal_t p; uint32_t boot; } m;
            memcpy(&m.p, buf + sizeof(rx_wire_hdr_t), sizeof m.p);
            m.boot = u->filt.boot_id;
            box_put(&u->inbox, &m, sizeof m);
            atomic_fetch_add(&u->shm->udp_rx_ok, 1);
            if (new_boot) { atomic_store(&u->shm->udp_boot_id, m.boot); atomic_fetch_add(&u->shm->udp_boots, 1); }
            break;
        }
        case RX_WIRE_BAD_PEER:  atomic_fetch_add(&u->shm->udp_rx_peer, 1); break;
        case RX_WIRE_MALFORMED: atomic_fetch_add(&u->shm->udp_rx_malformed, 1); break;
        default:                atomic_fetch_add(&u->shm->udp_rx_replay, 1); break;
        }
    }
    return NULL;
}

static void *tx_main(void *arg)
{
    rx_udp_t *u = arg;
    helper_prio(u->prio);
    uint32_t sent_ver = 0;
    for (;;) {
        if (sem_wait(&u->tx_sem) != 0) continue;
        if (atomic_load(&u->stop)) break;
        rx_wire_state_t d;
        uint32_t v = box_get(&u->outbox, &d.st, sizeof d.st);
        if (!v || v == sent_ver) continue;                      /* nothing new (extra wake-ups are harmless) */
        sent_ver = v;
        rx_wire_hdr(&d.h, RX_WIRE_STATE, u->boot_id, ++u->tx_seq);
        ssize_t n = sendto(u->fd, &d, sizeof d, MSG_DONTWAIT | MSG_NOSIGNAL, (struct sockaddr *)&u->peer, sizeof u->peer);
        atomic_fetch_add(n == (ssize_t)sizeof d ? &u->shm->udp_tx_ok : &u->shm->udp_tx_err, 1);
    }
    return NULL;
}

rx_udp_t *rx_udp_open(const char *bind_s, const char *peer_s, int prio, rx_shm_t *shm, char *err, size_t errcap)
{
    uint32_t pip; uint16_t pport;
    if (rx_wire_addr_parse(peer_s, &pip, &pport) != 0 || pip == 0 || pport == 0) {
        snprintf(err, errcap, "bad pilot_peer '%s' (want a.b.c.d:port, not 0.0.0.0)", peer_s ? peer_s : "");
        return NULL;
    }
    rx_udp_t *u = calloc(1, sizeof *u);
    if (!u) { snprintf(err, errcap, "out of memory"); return NULL; }
    u->shm = shm; u->prio = prio; u->boot_id = rx_udp_new_boot_id();
    u->fd = open_bound(bind_s, err, errcap);
    if (u->fd < 0) { free(u); return NULL; }
    struct timeval tv = { 0, 200000 };                          /* rx thread wakes 5x/s to notice stop */
    setsockopt(u->fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    u->peer.sin_family = AF_INET; u->peer.sin_addr.s_addr = htonl(pip); u->peer.sin_port = htons(pport);
    rx_wire_filter_init(&u->filt, pip, pport);
    if (sem_init(&u->tx_sem, 0, 0) != 0) { snprintf(err, errcap, "sem_init: %s", strerror(errno)); close(u->fd); free(u); return NULL; }
    int rc = pthread_create(&u->rx_t, NULL, rx_main, u);
    if (rc == 0 && (rc = pthread_create(&u->tx_t, NULL, tx_main, u)) != 0) {
        atomic_store(&u->stop, 1); pthread_join(u->rx_t, NULL);
    }
    if (rc != 0) { snprintf(err, errcap, "pthread_create: %s", strerror(rc)); sem_destroy(&u->tx_sem); close(u->fd); free(u); return NULL; }
    return u;
}

int rx_udp_take(rx_udp_t *u, rx_proposal_t *p, uint32_t *boot_id)
{
    struct { rx_proposal_t p; uint32_t boot; } m;
    uint32_t v = box_get(&u->inbox, &m, sizeof m);
    if (!v || v == u->taken_ver) return 0;
    u->taken_ver = v;
    *p = m.p;
    if (boot_id) *boot_id = m.boot;
    return 1;
}

void rx_udp_publish(rx_udp_t *u, const rx_state_t *st)
{
    box_put(&u->outbox, st, sizeof *st);
    sem_post(&u->tx_sem);                                       /* never blocks; EOVERFLOW if the tx thread is starved: harmless */
}

void rx_udp_close(rx_udp_t *u)
{
    if (!u) return;
    atomic_store(&u->stop, 1);
    sem_post(&u->tx_sem);
    pthread_join(u->tx_t, NULL);
    pthread_join(u->rx_t, NULL);
    sem_destroy(&u->tx_sem);
    close(u->fd);
    free(u);
}

/* ================================================================ pilot side */
struct rx_udp_client {
    int fd;
    struct sockaddr_in guardian;
    rx_wire_filter_t filt;
    uint32_t boot_id, tx_seq;
};

rx_udp_client_t *rx_udp_client_open(const char *guardian_s, const char *bind_s, char *err, size_t errcap)
{
    uint32_t gip; uint16_t gport;
    if (rx_wire_addr_parse(guardian_s, &gip, &gport) != 0 || gip == 0 || gport == 0) {
        snprintf(err, errcap, "bad guardian address '%s' (want a.b.c.d:port)", guardian_s ? guardian_s : "");
        return NULL;
    }
    rx_udp_client_t *c = calloc(1, sizeof *c);
    if (!c) { snprintf(err, errcap, "out of memory"); return NULL; }
    c->fd = open_bound(bind_s, err, errcap);
    if (c->fd < 0) { free(c); return NULL; }
    c->guardian.sin_family = AF_INET; c->guardian.sin_addr.s_addr = htonl(gip); c->guardian.sin_port = htons(gport);
    rx_wire_filter_init(&c->filt, gip, gport);
    c->boot_id = rx_udp_new_boot_id();
    return c;
}

uint32_t rx_udp_client_boot_id(const rx_udp_client_t *c) { return c->boot_id; }

static int client_send(rx_udp_client_t *c, const rx_proposal_t *p)
{
    rx_wire_prop_t d;
    rx_wire_hdr(&d.h, RX_WIRE_PROPOSAL, c->boot_id, ++c->tx_seq);
    d.p = *p;
    ssize_t n = sendto(c->fd, &d, sizeof d, MSG_NOSIGNAL, (struct sockaddr *)&c->guardian, sizeof c->guardian);
    return n == (ssize_t)sizeof d || errno == ENETUNREACH || errno == EHOSTUNREACH || errno == ECONNREFUSED || errno == EHOSTDOWN ? 0 : -1;
}

int rx_udp_client_exchange(rx_udp_client_t *c, const rx_proposal_t *p, rx_state_t *st, volatile sig_atomic_t *stop)
{
    if (client_send(c, p) != 0) return -1;
    rx_proposal_t hello; memset(&hello, 0, sizeof hello);
    hello.type = RX_MSG_PROPOSAL; hello.flags = RX_PF_HELLO | (p->flags & RX_PF_LEARNER);
    for (;;) {
        if (stop && *stop) return -1;
        struct pollfd pf = { c->fd, POLLIN, 0 };
        int r = poll(&pf, 1, 100);
        if (r < 0) { if (errno == EINTR) continue; return -1; }
        if (r == 0) { if (client_send(c, &hello) != 0) return -1; continue; }   /* guardian silent: keep saying hello */
        int got = 0;
        for (;;) {                                              /* drain: only the newest state matters */
            unsigned char buf[512];
            struct sockaddr_in src; socklen_t sl = sizeof src;
            ssize_t n = recvfrom(c->fd, buf, sizeof buf, MSG_DONTWAIT, (struct sockaddr *)&src, &sl);
            if (n < 0) break;
            int nb;
            if (sl >= (socklen_t)sizeof src && src.sin_family == AF_INET &&
                rx_wire_accept(&c->filt, ntohl(src.sin_addr.s_addr), ntohs(src.sin_port), buf, (size_t)n, RX_WIRE_STATE, &nb) == RX_WIRE_OK) {
                memcpy(st, buf + sizeof(rx_wire_hdr_t), sizeof *st);
                got = 1;
            }
        }
        if (got) return 0;
    }
}
