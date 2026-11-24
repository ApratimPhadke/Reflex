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
    /* optional (NULL on sim/serial): e-stop or actuator trip active; operator release (rxctl release) */
    int   (*estop)(struct rx_io *io);
    void  (*release)(struct rx_io *io);
    const char *name;
    void *priv;
} rx_io_t;

rx_io_t *rx_io_sim_open(const rx_cfg_t *cfg, rx_shm_t *shm, int fresh);
#include "rx_conf.h"
/* conf: rig calibration (reflex.conf) used to convert raw sensor mm / servo pulses; copied */
rx_io_t *rx_io_serial_open(const rx_cfg_t *cfg, const char *dev, rx_shm_t *shm, const rx_conf_t *conf);
/* beam-direct: the Pi drives the servo and reads the sensors itself (src/guardian/rx_io_beam.c, QNX only);
 * log_path: per-tick CSV of the I/O helper (NULL = none) */
#define RX_BEAM_SLEW_US 60    /* beam-direct: servo pulse change per control tick, at most (the t_delay depends on it) */
rx_io_t *rx_io_beam_open(const rx_cfg_t *cfg, rx_shm_t *shm, const rx_conf_t *conf, const char *log_path);
#endif
