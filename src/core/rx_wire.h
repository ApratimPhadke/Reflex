/* Reflex pilot link over UDP: datagram format and the receiver's filter.
 * Pure functions, no OS calls, so the filter can be fuzzed offline (rx_linktest).
 *
 * Used when the untrusted pilot runs somewhere the guardian cannot reach with QNX message passing,
 * e.g. in a Linux guest under QNX Hypervisor (pilot_transport = udp in reflex.conf). One datagram is
 * a 16-byte header followed by exactly the same rx_state_t / rx_proposal_t the local backends carry:
 *
 *   offset  size  field
 *   0       4     magic     RX_WIRE_MAGIC
 *   4       2     version   RX_WIRE_VERSION
 *   6       2     kind      RX_WIRE_STATE (guardian -> pilot) or RX_WIRE_PROPOSAL (pilot -> guardian)
 *   8       4     boot_id   random, nonzero, chosen once per sender process start
 *   12      4     pkt_seq   +1 per datagram from that sender instance
 *   16      ...   payload   rx_state_t (36 bytes) or rx_proposal_t (32 bytes)
 *
 * Every field is a little-endian 32/16-bit integer or IEEE-754 float (both ends are aarch64 or x86_64).
 *
 * The filter only decides whether a datagram is a well-formed, new message from the one expected
 * peer. It is not a safety check: an accepted proposal still goes through the guardian's stale,
 * non-finite, range and envelope checks unchanged, exactly as one that arrived by MsgSend.
 * SPDX-License-Identifier: MIT */
#ifndef RX_WIRE_H
#define RX_WIRE_H
#include <stddef.h>
#include <stdint.h>
#include "rx_proto.h"

#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error "the Reflex UDP wire format is little-endian; add byte swapping for this target"
#endif

#define RX_WIRE_MAGIC     0x56485852u   /* bytes 'R' 'X' 'H' 'V' */
#define RX_WIRE_VERSION   1
#define RX_WIRE_PORT      47800         /* default guardian port (pilot_bind) */
#define RX_WIRE_PILOT_PORT 47801        /* default pilot port (pilot_peer) */
enum { RX_WIRE_STATE = 1, RX_WIRE_PROPOSAL = 2 };

typedef struct { uint32_t magic; uint16_t version, kind; uint32_t boot_id, pkt_seq; } rx_wire_hdr_t;
typedef struct { rx_wire_hdr_t h; rx_state_t st; } rx_wire_state_t;
typedef struct { rx_wire_hdr_t h; rx_proposal_t p; } rx_wire_prop_t;
_Static_assert(sizeof(rx_wire_hdr_t) == 16, "wire header layout");
_Static_assert(sizeof(rx_wire_state_t) == 16 + 36, "state datagram layout");
_Static_assert(sizeof(rx_wire_prop_t) == 16 + 32, "proposal datagram layout");

/* Verdicts of rx_wire_accept */
enum {
    RX_WIRE_OK = 0,
    RX_WIRE_BAD_PEER,     /* not from the configured peer address and port */
    RX_WIRE_MALFORMED,    /* wrong length, magic, version, kind, message type or boot id 0 */
    RX_WIRE_REPLAY,       /* duplicate or older pkt_seq of the current sender instance */
    RX_WIRE_RETIRED,      /* from the sender instance before the current one (a late packet across a restart) */
};

typedef struct {
    uint32_t peer_ip;     /* expected source, host byte order */
    uint16_t peer_port;
    int      have;        /* a sender instance has been seen */
    uint32_t boot_id, last_seq, retired_boot;
} rx_wire_filter_t;

void rx_wire_filter_init(rx_wire_filter_t *f, uint32_t peer_ip, uint16_t peer_port);
/* Check one datagram of the given kind from src_ip:src_port (host byte order). RX_WIRE_OK: *new_boot is 1
 * if it is the first message of a new sender instance (a restarted pilot or guardian). */
int  rx_wire_accept(rx_wire_filter_t *f, uint32_t src_ip, uint16_t src_port, const void *buf, size_t len,
                    uint16_t kind, int *new_boot);
void rx_wire_hdr(rx_wire_hdr_t *h, uint16_t kind, uint32_t boot_id, uint32_t pkt_seq);

/* Strict "a.b.c.d:port" (decimal, no spaces, port 0..65535). 0 = ok; ip and port in host byte order. */
int  rx_wire_addr_parse(const char *s, uint32_t *ip, uint16_t *port);

#endif
