/* Plant I/O backends. SPDX-License-Identifier: MIT */
#ifndef RX_IO_H
#define RX_IO_H
#include "rx_core.h"

typedef struct rx_io {
    /* returns measured position [m]; NaN = no valid sample. *x_true is ground truth if known, else NaN.
     * *fresh = 1 if a new measurement arrived since the last call (the simulator: always). */
    float (*read)(struct rx_io *io, float *x_true, int *lost, int *fresh);
    void  (*write)(struct rx_io *io, float u);
    void  (*close)(struct rx_io *io);
    const char *name;
    void *priv;
} rx_io_t;

rx_io_t *rx_io_sim_open(const rx_cfg_t *cfg, rx_shm_t *shm, int fresh);
rx_io_t *rx_io_serial_open(const rx_cfg_t *cfg, const char *dev, rx_shm_t *shm);
#endif
