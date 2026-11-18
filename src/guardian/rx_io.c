/* Plant I/O backends: built-in simulator and serial link to the Reflex I/O node (MCU).
 * SPDX-License-Identifier: MIT */
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include "rx_io.h"
#include "rx_shm.h"

/* ------------------------------------------------------------------ simulator */
typedef struct { const rx_cfg_t *cfg; rx_shm_t *shm; } sim_t;

static double sim_noise(rx_shm_t *shm)
{
    uint64_t r = shm->sim.rng ? shm->sim.rng : 0x9E3779B97F4A7C15ull;
    double s = 0;
    for (int i = 0; i < 4; i++) { r ^= r << 13; r ^= r >> 7; r ^= r << 17; s += (double)(r >> 11) / 9007199254740992.0; }
    shm->sim.rng = r;
    return (s - 2.0) * 1.732;               /* ~N(0,1) */
}

static float sim_read(rx_io_t *io, float *x_true, int *lost)
{
    sim_t *s = io->priv;
    *lost = s->shm->sim.lost_ticks > 0;
    *x_true = (float)s->shm->sim.x;
    return (float)(s->shm->sim.x + 0.0015 * sim_noise(s->shm));
}

static void sim_write(rx_io_t *io, float u)
{
    sim_t *s = io->priv;
    rx_shm_t *shm = s->shm;
    const rx_cfg_t *c = s->cfg;
    if (shm->sim.lost_ticks > 0) {                      /* ball is on the floor; put a new one on after 1 s */
        if (--shm->sim.lost_ticks == 0) { shm->sim.x = 0; shm->sim.v = 0; }
    } else {
        rx_plant_t p = { shm->sim.x, shm->sim.v, shm->sim.theta };
        rx_plant_step(&p, c, u, c->dt);
        shm->sim.x = p.x; shm->sim.v = p.v; shm->sim.theta = p.theta;
        if (fabs(p.x) > c->half_len) {
            shm->sim.x = p.x > 0 ? c->half_len : -c->half_len;
            shm->sim.v = 0;
            shm->sim.lost_ticks = (uint32_t)c->hz;
            atomic_fetch_add(&shm->balls_lost, 1);
        }
    }
    shm->sim.t_last_ns = (double)rx_now_ns();
}

static void sim_close(rx_io_t *io) { free(io->priv); free(io); }

rx_io_t *rx_io_sim_open(const rx_cfg_t *cfg, rx_shm_t *shm, int fresh)
{
    rx_io_t *io = calloc(1, sizeof *io);
    sim_t *s = calloc(1, sizeof *s);
    s->cfg = cfg; s->shm = shm;
    double now = (double)rx_now_ns(), gap = (now - shm->sim.t_last_ns) * 1e-9;
    if (fresh || !shm->sim.valid || gap < 0 || gap > 5.0) {
        memset(&shm->sim, 0, sizeof shm->sim);
        shm->sim.x = 0.05; shm->sim.valid = 1;
    } else if (shm->sim.lost_ticks == 0) {
        /* The guardian was down for `gap` seconds. Physics did not stop: the actuator held its
         * last position (that is what the I/O node's watchdog-hold does), so advance the world. */
        rx_plant_t p = { shm->sim.x, shm->sim.v, shm->sim.theta };
        rx_plant_step(&p, cfg, (float)p.theta, gap);
        shm->sim.x = p.x; shm->sim.v = p.v;
        fprintf(stderr, "[guardian] resumed simulated plant after %.1f ms outage: x=%.1f mm v=%.3f m/s\n",
                gap * 1e3, p.x * 1e3, p.v);
    }
    io->read = sim_read; io->write = sim_write; io->close = sim_close; io->name = "sim"; io->priv = s;
    return io;
}

/* ------------------------------------------------------------------ serial I/O node
 * ASCII protocol, one line per message (easy to debug with a terminal):
 *   node -> guardian   "P <position_mm>\n"   or "P !\n" when the sensor sees no ball
 *   guardian -> node   "U <beam_angle_millirad>\n"
 * The node centres the beam by itself if it hears no "U" for 150 ms. */
typedef struct { int fd; char line[64]; int len; float last; uint64_t last_ns; int nosample; } ser_t;

static float ser_read(rx_io_t *io, float *x_true, int *lost)
{
    ser_t *s = io->priv;
    char buf[256];
    ssize_t n;
    while ((n = read(s->fd, buf, sizeof buf)) > 0) {
        for (ssize_t i = 0; i < n; i++) {
            char ch = buf[i];
            if (ch == '\n' || ch == '\r') {
                s->line[s->len] = 0;
                if (s->line[0] == 'P') {
                    if (strchr(s->line, '!')) s->nosample = 1;
                    else { s->last = (float)atof(s->line + 1) * 1e-3f; s->nosample = 0; }
                    s->last_ns = rx_now_ns();
                }
                s->len = 0;
            } else if (s->len < (int)sizeof s->line - 1) s->line[s->len++] = ch;
        }
    }
    *x_true = NAN;
    int stale = rx_now_ns() - s->last_ns > 200000000ull;
    *lost = s->nosample || stale;
    return (*lost) ? NAN : s->last;
}

static void ser_write(rx_io_t *io, float u)
{
    ser_t *s = io->priv;
    char b[32];
    int n = snprintf(b, sizeof b, "U %d\n", (int)lrintf((isfinite(u) ? u : 0.0f) * 1000.0f));
    if (write(s->fd, b, n) < 0) { /* link down: the node's own watchdog takes over */ }
}

static void ser_close(rx_io_t *io) { ser_t *s = io->priv; ser_write(io, 0); close(s->fd); free(s); free(io); }

rx_io_t *rx_io_serial_open(const rx_cfg_t *cfg, const char *dev)
{
    (void)cfg;
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { perror(dev); return NULL; }
    struct termios t;
    if (tcgetattr(fd, &t) == 0) {
        cfmakeraw(&t);
        cfsetispeed(&t, B115200); cfsetospeed(&t, B115200);
        t.c_cflag |= CLOCAL | CREAD;
        tcsetattr(fd, TCSANOW, &t);
    }
    rx_io_t *io = calloc(1, sizeof *io);
    ser_t *s = calloc(1, sizeof *s);
    s->fd = fd; s->nosample = 1;
    io->read = ser_read; io->write = ser_write; io->close = ser_close; io->name = "serial"; io->priv = s;
    return io;
}
