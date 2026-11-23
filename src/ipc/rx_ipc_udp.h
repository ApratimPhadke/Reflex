/* Reflex pilot link over UDP (pilot_transport = udp): for a pilot outside this QNX system, e.g. in a
 * Linux guest under QNX Hypervisor. Wire format and filter: src/core/rx_wire.h.
 *
 * Guardian side. The control thread never touches the network stack: a socket call is a message to
 * io-sock (QNX) or a syscall into a shared network stack (Linux), and an untrusted guest can flood it.
 * Two helper threads below the control loop's priority do the socket work:
 *   rx thread  blocking recvfrom -> filter -> newest proposal into a seqlock mailbox
 *   tx thread  waits on a semaphore -> newest state from a seqlock mailbox -> sendto the peer
 * The control thread only copies a mailbox (rx_udp_take) and posts a semaphore (rx_udp_publish);
 * neither can block. A starved or flooded rx thread simply makes proposals stale, which the guardian
 * already handles by flying the fallback.
 * SPDX-License-Identifier: MIT */
#ifndef RX_IPC_UDP_H
#define RX_IPC_UDP_H
#include <stddef.h>
#include <signal.h>
#include <stdint.h>
#include "rx_proto.h"

typedef struct rx_udp rx_udp_t;

/* bind / peer: "a.b.c.d:port". Only datagrams from exactly `peer` are accepted, and state goes only to it.
 * helper_prio: SCHED_FIFO priority of the two helper threads (best effort). Counters go to shm. */
rx_udp_t *rx_udp_open(const char *bind, const char *peer, int helper_prio, rx_shm_t *shm, char *err, size_t errcap);
/* control thread, never blocks: 1 = a proposal arrived since the last call (copied to *p, sender instance
 * to *boot_id), 0 = nothing new (or the rx thread was mid-write: it will be there next tick). */
int       rx_udp_take(rx_udp_t *u, rx_proposal_t *p, uint32_t *boot_id);
/* control thread, never blocks: hand the next state to the tx thread. */
void      rx_udp_publish(rx_udp_t *u, const rx_state_t *st);
void      rx_udp_close(rx_udp_t *u);

/* Pilot side. guardian: where the guardian listens; bind: our own address (must match the guardian's pilot_peer). */
typedef struct rx_udp_client rx_udp_client_t;
rx_udp_client_t *rx_udp_client_open(const char *guardian, const char *bind, char *err, size_t errcap);
/* Send a proposal, then wait for the next state from the guardian. While none comes, a HELLO keepalive
 * goes out every 100 ms and we keep waiting: a guardian restart is not the pilot's problem. If several
 * states are queued (the pilot was descheduled), only the newest is returned. 0 = ok, -1 = socket error
 * or *stop became nonzero. */
int       rx_udp_client_exchange(rx_udp_client_t *c, const rx_proposal_t *p, rx_state_t *st, volatile sig_atomic_t *stop);
uint32_t  rx_udp_client_boot_id(const rx_udp_client_t *c);

/* a random nonzero 32-bit sender instance id */
uint32_t  rx_udp_new_boot_id(void);

#endif
