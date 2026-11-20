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

static float sim_read(rx_io_t *io, float *x_true, int *lost, int *fresh)
{
    sim_t *s = io->priv;
    *fresh = 1;
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
 * Protocol and parser: src/core/rx_link.h. The node levels the beam by itself if it hears no
 * "U" for 150 ms, so every failure here ends in "say nothing" and the node takes over.
 *
 * Link loss (cable pulled, node reset, pty closed) shows up as a read/write error or EOF. The
 * control loop then stops using the descriptor and hands it to a helper thread, which closes it,
 * reopens the device until it is back, and publishes the new descriptor. open()/close()/tcsetattr()
 * are blocking requests to the driver, so they never run in the control loop. The loop and the
 * helper share only atomics: `fd` (owned by the loop while >= 0) and `dead_fd` (handed over). */
#include <pthread.h>
#include "rx_link.h"

typedef struct {
    const rx_cfg_t *cfg;
    rx_shm_t   *shm;
    char        dev[256];
    _Atomic int fd;             /* -1 = link down, helper is reopening */
    _Atomic int dead_fd;        /* descriptor the loop gave up on; the helper closes it */
    _Atomic int stop;
    pthread_t   helper;
    int         helper_ok;
    int         up;             /* loop's view of the link, for up/down events */
    rx_link_t   link;
    rx_conf_t   conf;           /* rig calibration: raw mm / pulse <-> m / rad */
    int         told_info, told_v1;
    uint64_t    last_ns;        /* time of the newest "M" line */
    int         last_valid;     /* newest sample had a position */
    float       last_x;
} ser_t;

static int ser_configure(int fd)
{
    struct termios t;
    if (tcgetattr(fd, &t) != 0) return isatty(fd) ? -1 : 0;   /* not a tty (e.g. a FIFO in tests): fine */
    cfmakeraw(&t);
    cfsetispeed(&t, B115200); cfsetospeed(&t, B115200);
    t.c_cflag |= CLOCAL | CREAD;
    return tcsetattr(fd, TCSANOW, &t);
}

static int ser_open_dev(const char *dev)
{
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;
    if (ser_configure(fd) != 0) { close(fd); return -1; }
    /* Drop whatever the driver queued while nobody was reading. QNX devc-serusb keeps reading the
     * node while the port is closed and holds about 2 KB, keeping the OLDEST data when full
     * (measured 2026-11-20 on the Pi 4: 370 lines, about 7 s old, after 30 s unread). Without this the
     * first tick acts on a stale position. After the flush only ~5 lines remain (results/2026-11-20-pi4-emulated-io/).
     * Fails harmlessly on a FIFO in tests. */
    (void)tcflush(fd, TCIFLUSH);
    return fd;
}

static void *ser_helper(void *arg)
{
    ser_t *s = arg;
    struct timespec nap = { 0, 100 * 1000000 };
    while (!atomic_load(&s->stop)) {
        int dead = atomic_exchange(&s->dead_fd, -1);
        if (dead >= 0) close(dead);
        if (atomic_load(&s->fd) < 0) {
            int fd = ser_open_dev(s->dev);
            if (fd >= 0) atomic_store(&s->fd, fd);
        }
        nanosleep(&nap, NULL);
    }
    return NULL;
}

static void ser_link_down(ser_t *s, int fd)
{
    atomic_store(&s->fd, -1);
    atomic_store(&s->dead_fd, fd);       /* the helper closes it and starts reopening */
}

static float ser_read(rx_io_t *io, float *x_true, int *lost, int *fresh)
{
    ser_t *s = io->priv;
    *x_true = NAN; *fresh = 0;
    int fd = atomic_load(&s->fd);
    uint64_t now = rx_now_ns();
    if (fd >= 0 && !s->up) {
        s->up = 1;
        rx_link_init(&s->link);
        s->last_ns = now;                /* give the node one stale window to start talking */
        s->last_valid = 0;
        s->told_info = s->told_v1 = 0;
        atomic_store(&s->shm->node_proto, 0);
        atomic_store(&s->shm->node_link, 1);
        rx_event_push(s->shm, 0, RX_E_LINK_UP, fd);
        /* tell the node our level pulse (it keeps it in flash for its watchdog) and ask who it is;
         * non-blocking writes, dropped if the driver is full (the node re-levels on its own anyway) */
        char b[48];
        int n = rx_link_cmd_level(b, sizeof b, s->conf.level_us, RX_PULSE_MIN, RX_PULSE_MAX);
        n += rx_link_cmd_query(b + n, sizeof b - (size_t)n);
        if (write(fd, b, (size_t)n) < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            ser_link_down(s, fd); fd = -1;
        }
    }
    if (fd >= 0) {
        char buf[256];
        for (int i = 0; i < 16; i++) {   /* bounded: at 115200 baud 16*256 bytes is ~350 ms of traffic */
            ssize_t n = read(fd, buf, sizeof buf);
            if (n > 0) {
                if (rx_link_feed(&s->link, buf, (size_t)n) > 0) {
                    *fresh = 1;
                    s->last_ns = now;
                    int was_valid = s->last_valid, in_window = 0;
                    float x = s->link.valid ? rx_conf_position(&s->conf, s->link.raw_mm, &in_window) : NAN;
                    s->last_valid = s->link.valid && in_window;
                    if (s->last_valid) s->last_x = x;
                    /* The node cannot tell "fell off the end" from "sensor dropout". If the ball
                     * was last seen within 4 cm of an end and then vanished, count it as lost.
                     * Evaluation only: the guardian's decisions never use this. */
                    if (was_valid && !s->last_valid && fabsf(s->last_x) > s->cfg->half_len - 0.04f) {
                        atomic_fetch_add(&s->shm->balls_lost, 1);
                        rx_event_push(s->shm, 0, RX_E_BALL_LOST, (int32_t)lrintf(s->last_x * 1000.0f));
                    }
                }
                if (s->link.have_info && !s->told_info) {          /* publish the node's identity once */
                    s->told_info = 1;
                    memcpy(s->shm->node_board, s->link.board, sizeof s->shm->node_board);
                    s->shm->node_board[sizeof s->shm->node_board - 1] = 0;
                    atomic_store(&s->shm->node_emulated, (uint32_t)s->link.emulated);
                    atomic_store(&s->shm->node_proto, (uint32_t)s->link.proto);
                    rx_event_push(s->shm, 0, RX_E_NODE_INFO, s->link.proto * 10 + s->link.emulated);
                }
                if (s->link.v1_lines && !s->told_v1) { s->told_v1 = 1; rx_event_push(s->shm, 0, RX_E_NODE_OLD, 1); }
                if ((size_t)n < sizeof buf) break;
            } else if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
                ser_link_down(s, fd);    /* EOF or I/O error: cable gone, node reset, pty closed */
                fd = -1;
                break;
            } else break;                /* nothing more to read */
        }
    }
    if (fd < 0 && s->up) {
        s->up = 0;
        atomic_store(&s->shm->node_link, 0);
        rx_event_push(s->shm, 0, RX_E_LINK_DOWN, 0);
    }
    int stale = fd < 0 || now - s->last_ns > 200000000ull;
    *lost = stale || !s->last_valid;
    return *lost ? NAN : s->last_x;
}

static void ser_write(rx_io_t *io, float u)
{
    ser_t *s = io->priv;
    int fd = atomic_load(&s->fd);
    if (fd < 0) return;                  /* link down: the node's own watchdog has the beam */
    char b[32];
    float lim = s->cfg->theta_max;                          /* never ask for more than the kernel allows */
    if (isfinite(u)) u = u > lim ? lim : (u < -lim ? -lim : u);
    int n = rx_link_cmd_servo(b, sizeof b, rx_conf_pulse(&s->conf, u), RX_PULSE_MIN, RX_PULSE_MAX);
    ssize_t w = write(fd, b, (size_t)n);
    if (w < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) ser_link_down(s, fd);
}

static void ser_close(rx_io_t *io)
{
    ser_t *s = io->priv;
    ser_write(io, 0);
    atomic_store(&s->stop, 1);
    if (s->helper_ok) pthread_join(s->helper, NULL);
    int fd = atomic_exchange(&s->fd, -1), dead = atomic_exchange(&s->dead_fd, -1);
    if (fd >= 0) close(fd);
    if (dead >= 0) close(dead);
    free(s); free(io);
}

rx_io_t *rx_io_serial_open(const rx_cfg_t *cfg, const char *dev, rx_shm_t *shm, const rx_conf_t *conf)
{
    rx_io_t *io = calloc(1, sizeof *io);
    ser_t *s = calloc(1, sizeof *s);
    if (!io || !s) { free(io); free(s); return NULL; }
    s->cfg = cfg; s->shm = shm; s->conf = *conf;
    snprintf(s->dev, sizeof s->dev, "%s", dev);
    atomic_store(&s->dead_fd, -1);
    int fd = ser_open_dev(dev);
    if (fd < 0) fprintf(stderr, "[guardian] %s: %s - waiting for the I/O node to appear\n", dev, strerror(errno));
    atomic_store(&s->fd, fd);
    int rc = pthread_create(&s->helper, NULL, ser_helper, s);   /* inherits our (lower, not yet RT) priority */
    if (rc) { fprintf(stderr, "[guardian] serial helper: %s\n", strerror(rc)); if (fd >= 0) close(fd); free(s); free(io); return NULL; }
    s->helper_ok = 1;
    io->read = ser_read; io->write = ser_write; io->close = ser_close; io->name = "serial"; io->priv = s;
    return io;
}
