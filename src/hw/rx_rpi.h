/* Raspberry Pi 4 under QNX 8.0 (quick-start image): GPIO, hardware PWM and I2C, plus the beam-direct rig's
 * sensors (MPU6050, INA219, HC-SR04). QNX only.
 *
 * GPIO and PWM go to the image's GPIO resource manager (/dev/gpio/msg, group gpio), with the message protocol
 * of the QNX hardware-component-samples (common/system/gpio/sys/rpi_gpio.h); this is an independent
 * implementation of that protocol. I2C goes to /dev/i2cN with DCMD_I2C_SENDRECV / DCMD_I2C_SEND (<hw/i2c.h>).
 * SPDX-License-Identifier: MIT */
#ifndef RX_RPI_H
#define RX_RPI_H
#include <stdint.h>

/* ---- GPIO (BCM numbering, 0..27) */
int  rx_gpio_open(void);                               /* 0 = ok; -1 and errno */
int  rx_gpio_input(int pin, int pull);                 /* pull: 0 off, 1 up, 2 down */
int  rx_gpio_output(int pin, int level);
int  rx_gpio_write(int pin, int level);
int  rx_gpio_read(int pin);                            /* 0/1, -1 on error */
int  rx_gpio_func(int pin);                            /* function select code (0 in, 1 out, 2.. alt), -1 on error */
/* Deliver a pulse (code, value) on chid's connection on edges: rising 1, falling 2 */
int  rx_gpio_edge_event(int pin, int coid, int edges, int code, int value);
/* Hardware PWM, mark-space mode. range = counts per period; duty in counts. */
int  rx_gpio_pwm_setup(int pin, unsigned freq_hz, unsigned range);
int  rx_gpio_pwm_duty(int pin, unsigned counts);

/* ---- I2C */
int  rx_i2c_open(int bus);                             /* fd, -1 on error */
int  rx_i2c_read(int fd, uint8_t addr, uint8_t reg, uint8_t *buf, unsigned n);   /* 0 = ok */
int  rx_i2c_write(int fd, uint8_t addr, uint8_t reg, const uint8_t *buf, unsigned n);
int  rx_i2c_probe(int fd, uint8_t addr);               /* 1 = a device acknowledged */

/* ---- MPU6050 */
typedef struct { float ax, ay, az, gx, gy, gz, temp_c; } rx_mpu_t;   /* g, deg/s, deg C */
int  rx_mpu_init(int fd, uint8_t addr, uint8_t *who);  /* wake, +-2 g, +-250 deg/s, 44 Hz DLPF */
int  rx_mpu_read(int fd, uint8_t addr, rx_mpu_t *m);

/* ---- INA219 (shunt assumed 0.1 ohm, the usual breakout) */
int  rx_ina_read(int fd, uint8_t addr, float *bus_v, float *shunt_mv, float *current_a, uint16_t *config);

/* ---- HC-SR04: trigger + echo through edge events from the GPIO manager */
typedef struct { int trig, echo, chid, coid; } rx_sonar_t;
int  rx_sonar_open(rx_sonar_t *s, int trig, int echo);
/* One measurement: distance in mm (-1 = no echo within timeout_ms), echo width in us */
float rx_sonar_measure(rx_sonar_t *s, int timeout_ms, float *echo_us);

#endif
