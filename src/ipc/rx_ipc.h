/* Reflex IPC: the one hop between the untrusted pilot and the guardian.
 *
 *   rx_ipc_qnx.c   - native QNX: channel + pulses + synchronous Send/Receive/Reply
 *   rx_ipc_posix.c - Linux reference backend (timerfd + SOCK_SEQPACKET) used as the
 *                    A/B baseline and for host-side tests
 *
 * Contract (both backends):
 *   - the pilot SENDS a proposal and stays blocked until the guardian REPLIES with
 *     the next state. The guardian never blocks on the pilot.
 *   - the guardian's control tick arrives on the same wait call as pilot messages.
 * SPDX-License-Identifier: MIT */
#ifndef RX_IPC_H
#define RX_IPC_H

#include <stdint.h>
#include <sys/types.h>
#include "rx_proto.h"

typedef enum { RX_EV_NONE = 0, RX_EV_TICK, RX_EV_PROPOSAL, RX_EV_GONE, RX_EV_REJECTED, RX_EV_ERROR } rx_event_e;

typedef struct rx_server rx_server_t;
typedef struct rx_client rx_client_t;

/* guardian side */
rx_server_t *rx_server_create(uint64_t period_ns, int prio, rx_shm_t *shm);
rx_event_e   rx_server_wait(rx_server_t *s, rx_proposal_t *prop);
void         rx_server_allow(rx_server_t *s, pid_t pilot_pid);  /* only this pid may send proposals */
int          rx_server_publish(rx_server_t *s, const rx_state_t *st); /* 1 = a pilot was unblocked */
void         rx_server_drop(rx_server_t *s);                    /* forget a pending pilot */
uint64_t     rx_server_start_ns(const rx_server_t *s);              /* instant the periodic timer was armed */
const char  *rx_ipc_backend(void);

/* pilot side */
rx_client_t *rx_client_connect(pid_t guardian_pid, int chid);   /* the pilot never maps the guardian's memory */
int          rx_server_chid(const rx_server_t *s);
int          rx_client_exchange(rx_client_t *c, const rx_proposal_t *prop, rx_state_t *st); /* 0 ok, -1 guardian gone */

#endif
