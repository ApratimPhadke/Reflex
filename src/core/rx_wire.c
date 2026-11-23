/* See rx_wire.h. SPDX-License-Identifier: MIT */
#include <string.h>
#include "rx_wire.h"

void rx_wire_filter_init(rx_wire_filter_t *f, uint32_t peer_ip, uint16_t peer_port)
{
    memset(f, 0, sizeof *f);
    f->peer_ip = peer_ip; f->peer_port = peer_port;
}

void rx_wire_hdr(rx_wire_hdr_t *h, uint16_t kind, uint32_t boot_id, uint32_t pkt_seq)
{
    h->magic = RX_WIRE_MAGIC; h->version = RX_WIRE_VERSION; h->kind = kind;
    h->boot_id = boot_id; h->pkt_seq = pkt_seq;
}

int rx_wire_accept(rx_wire_filter_t *f, uint32_t src_ip, uint16_t src_port, const void *buf, size_t len,
                   uint16_t kind, int *new_boot)
{
    *new_boot = 0;
    if (src_ip != f->peer_ip || src_port != f->peer_port) return RX_WIRE_BAD_PEER;
    size_t want = kind == RX_WIRE_STATE ? sizeof(rx_wire_state_t) : kind == RX_WIRE_PROPOSAL ? sizeof(rx_wire_prop_t) : 0;
    if (!want || len != want) return RX_WIRE_MALFORMED;
    rx_wire_hdr_t h;
    memcpy(&h, buf, sizeof h);
    if (h.magic != RX_WIRE_MAGIC || h.version != RX_WIRE_VERSION || h.kind != kind || h.boot_id == 0) return RX_WIRE_MALFORMED;
    if (kind == RX_WIRE_PROPOSAL) {
        uint32_t type;
        memcpy(&type, (const char *)buf + sizeof h + offsetof(rx_proposal_t, type), sizeof type);
        if (type != RX_MSG_PROPOSAL) return RX_WIRE_MALFORMED;
    }
    if (f->have && h.boot_id == f->boot_id) {
        if ((int32_t)(h.pkt_seq - f->last_seq) <= 0) return RX_WIRE_REPLAY;
        f->last_seq = h.pkt_seq;
        return RX_WIRE_OK;
    }
    if (f->have && h.boot_id == f->retired_boot) return RX_WIRE_RETIRED;
    if (f->have) f->retired_boot = f->boot_id;
    f->have = 1; f->boot_id = h.boot_id; f->last_seq = h.pkt_seq;
    *new_boot = 1;
    return RX_WIRE_OK;
}

static const char *num(const char *p, unsigned long max, unsigned long *out)
{
    unsigned long v = 0; int n = 0;
    while (*p >= '0' && *p <= '9') { v = v * 10 + (unsigned long)(*p - '0'); if (v > max || ++n > 5) return NULL; p++; }
    if (!n) return NULL;
    *out = v;
    return p;
}

int rx_wire_addr_parse(const char *s, uint32_t *ip, uint16_t *port)
{
    if (!s) return -1;
    uint32_t a = 0; unsigned long v;
    const char *p = s;
    for (int i = 0; i < 4; i++) {
        if (!(p = num(p, 255, &v))) return -1;
        a = (a << 8) | (uint32_t)v;
        if (*p != (i < 3 ? '.' : ':')) return -1;
        p++;
    }
    if (!(p = num(p, 65535, &v)) || *p) return -1;
    *ip = a; *port = (uint16_t)v;
    return 0;
}
