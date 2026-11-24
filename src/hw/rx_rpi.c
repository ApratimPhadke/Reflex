/* See rx_rpi.h. SPDX-License-Identifier: MIT */
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <devctl.h>
#include <hw/i2c.h>
#include <sys/iomsg.h>
#include <sys/iomgr.h>
#include <sys/neutrino.h>
#include <sys/syspage.h>
#include "rx_rpi.h"

/* ---- GPIO manager protocol (as in hardware-component-samples common/system/gpio/sys/rpi_gpio.h) */
#define RPI_GPIO_IOMGR (_IOMGR_PRIVATE_BASE + 35)
enum { G_SET_SELECT, G_GET_SELECT, G_WRITE, G_READ, G_ADD_EVENT, G_PWM_SETUP, G_PWM_DUTY, G_PUD };
typedef struct { struct _io_msg hdr; unsigned gpio, value; } g_msg_t;
typedef struct { struct _io_msg hdr; unsigned gpio, detect; struct sigevent event; unsigned match, reserved; } g_event_t;
typedef struct { struct _io_msg hdr; unsigned gpio, frequency, range, mode; } g_pwm_t;

static int gfd = -1;

int rx_gpio_open(void)
{
    if (gfd >= 0) return 0;
    gfd = open("/dev/gpio/msg", O_RDWR);
    return gfd >= 0 ? 0 : -1;
}

static void hdr(struct _io_msg *h, unsigned sub)
{
    memset(h, 0, sizeof *h);
    h->type = _IO_MSG; h->subtype = (uint16_t)sub; h->mgrid = RPI_GPIO_IOMGR;
}

static int g_send(unsigned sub, int pin, unsigned value, int reply, unsigned *out)
{
    if (rx_gpio_open() != 0) return -1;
    if (pin < 0 || pin > 27) { errno = EINVAL; return -1; }
    g_msg_t m; hdr(&m.hdr, sub); m.gpio = (unsigned)pin; m.value = value;
    if (MsgSend(gfd, &m, sizeof m, reply ? &m : NULL, reply ? sizeof m : 0) == -1) return -1;
    if (out) *out = m.value;
    return 0;
}

int rx_gpio_input(int pin, int pull)
{
    static const unsigned pud[3] = { 0, 2, 1 };          /* off, up, down in the manager's codes */
    if (g_send(G_SET_SELECT, pin, 0, 0, NULL) != 0) return -1;
    return g_send(G_PUD, pin, pud[pull < 0 || pull > 2 ? 0 : pull], 0, NULL);
}
int rx_gpio_output(int pin, int level)
{
    if (g_send(G_SET_SELECT, pin, 1, 0, NULL) != 0) return -1;
    return rx_gpio_write(pin, level);
}
int rx_gpio_write(int pin, int level) { return g_send(G_WRITE, pin, level ? 1 : 0, 0, NULL); }
int rx_gpio_read(int pin) { unsigned v = 0; return g_send(G_READ, pin, 1, 1, &v) == 0 ? (int)(v & 1) : -1; }
int rx_gpio_func(int pin) { unsigned v = 0; return g_send(G_GET_SELECT, pin, 0, 1, &v) == 0 ? (int)v : -1; }

int rx_gpio_edge_event(int pin, int coid, int edges, int code, int value)
{
    if (rx_gpio_open() != 0) return -1;
    g_event_t e; memset(&e, 0, sizeof e); hdr(&e.hdr, G_ADD_EVENT);
    e.gpio = (unsigned)pin; e.detect = (unsigned)(edges & 3);
    SIGEV_PULSE_INIT(&e.event, coid, SIGEV_PULSE_PRIO_INHERIT, code, value);
    if (MsgRegisterEvent(&e.event, gfd) == -1) return -1;
    return MsgSend(gfd, &e, sizeof e, NULL, 0) == -1 ? -1 : 0;
}

int rx_gpio_pwm_setup(int pin, unsigned freq_hz, unsigned range)
{
    if (rx_gpio_open() != 0) return -1;
    g_pwm_t p; memset(&p, 0, sizeof p); hdr(&p.hdr, G_PWM_SETUP);
    p.gpio = (unsigned)pin; p.frequency = freq_hz; p.range = range; p.mode = 1;   /* mark-space */
    return MsgSend(gfd, &p, sizeof p, NULL, 0) == -1 ? -1 : 0;
}
int rx_gpio_pwm_duty(int pin, unsigned counts) { return g_send(G_PWM_DUTY, pin, counts, 0, NULL); }

/* ---- I2C */
int rx_i2c_open(int bus)
{
    char p[24]; snprintf(p, sizeof p, "/dev/i2c%d", bus);
    return open(p, O_RDWR);
}

int rx_i2c_read(int fd, uint8_t addr, uint8_t reg, uint8_t *buf, unsigned n)
{
    struct { i2c_sendrecv_t h; uint8_t b[64]; } m;
    if (n < 1 || n > sizeof m.b) { errno = EINVAL; return -1; }
    memset(&m.h, 0, sizeof m.h);
    m.h.slave.addr = addr; m.h.slave.fmt = I2C_ADDRFMT_7BIT; m.h.send_len = 1; m.h.recv_len = n; m.h.stop = 1;
    m.b[0] = reg;
    int r = devctl(fd, DCMD_I2C_SENDRECV, &m, sizeof m.h + n, NULL);
    if (r != EOK) { errno = r; return -1; }
    memcpy(buf, m.b, n);
    return 0;
}

int rx_i2c_write(int fd, uint8_t addr, uint8_t reg, const uint8_t *buf, unsigned n)
{
    struct { i2c_send_t h; uint8_t b[65]; } m;
    if (n > sizeof m.b - 1) { errno = EINVAL; return -1; }
    memset(&m.h, 0, sizeof m.h);
    m.h.slave.addr = addr; m.h.slave.fmt = I2C_ADDRFMT_7BIT; m.h.len = n + 1; m.h.stop = 1;
    m.b[0] = reg; if (n) memcpy(m.b + 1, buf, n);
    int r = devctl(fd, DCMD_I2C_SEND, &m, sizeof m.h + n + 1, NULL);
    if (r != EOK) { errno = r; return -1; }
    return 0;
}

int rx_i2c_probe(int fd, uint8_t addr)
{
    struct { i2c_recv_t h; uint8_t b[4]; } m;
    memset(&m, 0, sizeof m);
    m.h.slave.addr = addr; m.h.slave.fmt = I2C_ADDRFMT_7BIT; m.h.len = 1; m.h.stop = 1;
    return devctl(fd, DCMD_I2C_RECV, &m, sizeof m.h + 1, NULL) == EOK;
}

/* ---- MPU6050 */
int rx_mpu_init(int fd, uint8_t addr, uint8_t *who)
{
    uint8_t w = 0, v;
    if (rx_i2c_read(fd, addr, 0x75, &w, 1) != 0) return -1;      /* WHO_AM_I */
    if (who) *who = w;
    v = 0x01; if (rx_i2c_write(fd, addr, 0x6B, &v, 1) != 0) return -1;   /* wake, PLL on gyro X */
    usleep(50000);
    v = 0x03; if (rx_i2c_write(fd, addr, 0x1A, &v, 1) != 0) return -1;   /* DLPF 44 Hz */
    v = 0x00; if (rx_i2c_write(fd, addr, 0x19, &v, 1) != 0) return -1;   /* 1 kHz sample rate */
    v = 0x00; if (rx_i2c_write(fd, addr, 0x1B, &v, 1) != 0) return -1;   /* +-250 deg/s */
    v = 0x00; if (rx_i2c_write(fd, addr, 0x1C, &v, 1) != 0) return -1;   /* +-2 g */
    usleep(150000);                                               /* gyro start-up + filter settle */
    return 0;
}

int rx_mpu_read(int fd, uint8_t addr, rx_mpu_t *m)
{
    uint8_t b[14];
    if (rx_i2c_read(fd, addr, 0x3B, b, 14) != 0) return -1;
#define S16(i) ((int16_t)((b[i] << 8) | b[(i) + 1]))
    m->ax = S16(0) / 16384.0f; m->ay = S16(2) / 16384.0f; m->az = S16(4) / 16384.0f;
    m->temp_c = S16(6) / 340.0f + 36.53f;
    m->gx = S16(8) / 131.0f; m->gy = S16(10) / 131.0f; m->gz = S16(12) / 131.0f;
#undef S16
    return 0;
}

/* ---- INA219 */
int rx_ina_read(int fd, uint8_t addr, float *bus_v, float *shunt_mv, float *current_a, uint16_t *config)
{
    uint8_t b[2];
    if (rx_i2c_read(fd, addr, 0x00, b, 2) != 0) return -1;
    if (config) *config = (uint16_t)((b[0] << 8) | b[1]);
    if (rx_i2c_read(fd, addr, 0x01, b, 2) != 0) return -1;
    float sh = (int16_t)((b[0] << 8) | b[1]) * 0.01f;            /* 10 uV per LSB -> mV */
    if (rx_i2c_read(fd, addr, 0x02, b, 2) != 0) return -1;
    float bv = (((b[0] << 8) | b[1]) >> 3) * 0.004f;             /* 4 mV per LSB */
    if (bus_v) *bus_v = bv;
    if (shunt_mv) *shunt_mv = sh;
    if (current_a) *current_a = sh / 100.0f;                    /* mV / 0.1 ohm / 1000 */
    return 0;
}

/* ---- HC-SR04 */
#define SONAR_CODE (_PULSE_CODE_MINAVAIL + 3)

int rx_sonar_open(rx_sonar_t *s, int trig, int echo)
{
    s->trig = trig; s->echo = echo;
    if ((s->chid = ChannelCreate(_NTO_CHF_PRIVATE)) == -1) return -1;
    if ((s->coid = ConnectAttach(0, 0, s->chid, _NTO_SIDE_CHANNEL, 0)) == -1) return -1;
    if (rx_gpio_output(trig, 0) != 0) return -1;
    if (rx_gpio_input(echo, 0) != 0) return -1;                   /* the 1k/2k divider sets the level */
    return rx_gpio_edge_event(echo, s->coid, 3, SONAR_CODE, 1);
}

static int wait_pulse(int chid, uint64_t timeout_ns, uint64_t *t)
{
    struct _pulse p;
    for (;;) {
        TimerTimeout(CLOCK_MONOTONIC, _NTO_TIMEOUT_RECEIVE, NULL, &timeout_ns, NULL);
        if (MsgReceivePulse(chid, &p, sizeof p, NULL) == -1) return -1;    /* ETIMEDOUT */
        if (t) *t = ClockCycles();
        if (p.code == SONAR_CODE) return 0;
    }
}

float rx_sonar_measure(rx_sonar_t *s, int timeout_ms, float *echo_us)
{
    const uint64_t cps = SYSPAGE_ENTRY(qtime)->cycles_per_sec;
    while (wait_pulse(s->chid, 0, NULL) == 0) {}                  /* drop edges left from before */
    if (echo_us) *echo_us = -1;
    rx_gpio_write(s->trig, 1);
    uint64_t t0 = ClockCycles();
    while (ClockCycles() - t0 < cps / 80000) {}                   /* >= 12 us high */
    rx_gpio_write(s->trig, 0);
    uint64_t tr, tf, to = (uint64_t)timeout_ms * 1000000ull;
    if (wait_pulse(s->chid, to, &tr) != 0) return -1;
    if (wait_pulse(s->chid, to, &tf) != 0) return -1;
    float us = (float)((double)(tf - tr) * 1e6 / (double)cps);
    if (echo_us) *echo_us = us;
    return us * 0.1715f;                                          /* 343 m/s, there and back: mm */
}
