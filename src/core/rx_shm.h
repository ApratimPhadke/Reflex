/* SPDX-License-Identifier: MIT */
#ifndef RX_SHM_H
#define RX_SHM_H
#include "rx_proto.h"

uint64_t  rx_now_ns(void);
rx_shm_t *rx_shm_open(int create);
/* delete the shared-memory object (the next guardian starts a fresh world and fresh counters) */
int       rx_shm_remove(void);
void      rx_shm_push(rx_shm_t *s, const rx_rec_t *r);                 /* guardian only, wait-free */
int       rx_shm_get(const rx_shm_t *s, uint64_t idx, rx_rec_t *out);  /* 1 if a consistent record was copied */
int       rx_lat_bin(uint32_t lat_ns);
void      rx_event_push(rx_shm_t *s, uint32_t seq, uint16_t code, int32_t arg);   /* guardian only */
int       rx_event_get(const rx_shm_t *s, uint64_t idx, rx_event_t *out);
void      rx_shm_defaults(rx_shm_t *s);

int       rx_fault_apply(rx_shm_t *s, const char *name, uint32_t arg);
int       rx_command(rx_shm_t *s, const char *line);
int       rx_status_text(const rx_shm_t *s, char *buf, int cap);
#endif
