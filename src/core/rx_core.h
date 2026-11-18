/* Reflex safety kernel - pure functions, no OS calls, no allocation.
 * Everything that decides whether an AI command reaches the actuator is in
 * rx_core.c so it can be reviewed and unit-tested on its own.
 * SPDX-License-Identifier: MIT */
#ifndef RX_CORE_H
#define RX_CORE_H

#include <stdint.h>
#include "rx_proto.h"

typedef struct {
    float hz, dt;
    float half_len;      /* beam half length [m] */
    float edge_margin;   /* keep-out strip at each end [m] */
    float k_acc;         /* ball acceleration per sin(beam angle) [m/s^2] */
    float theta_max;     /* actuator limit [rad] */
    float slew;          /* actuator slew rate [rad/s] (simulator + envelope sizing) */
    float t_delay;       /* worst-case time before braking takes effect [s] */
    float brake_eta;     /* fraction of ideal braking we are willing to rely on */
    float wn, zeta;      /* safe controller: natural frequency [rad/s], damping */
    float est_tau;       /* estimator time constant [s] */
    float max_age_s;     /* pilot deadline: older proposals are stale */
    float hold_s;        /* minimum time in SAFE after an intervention */
    float inner_margin;  /* margin required before AI gets control back [m] */
    float sp_limit;      /* setpoints are clamped to +/- this [m] */
    uint32_t strike_limit;   /* misbehaviour strikes before a restart is requested */
    float strike_window_s;
} rx_cfg_t;

void  rx_cfg_default(rx_cfg_t *c, float hz);
void  rx_cfg_hardware(rx_cfg_t *c);   /* adjustments for the real beam: slow, noisy sensor and a real servo */

/* alpha-beta position/velocity estimator */
typedef struct { float x, v; int init; } rx_est_t;
void  rx_est_update(rx_est_t *e, const rx_cfg_t *c, float z);

/* Distance [m] between the worst-case stopping point and the keep-out strip.
 * > 0  : the safe controller can still stop the ball in time (recoverable)
 * <= 0 : not guaranteed any more */
float rx_margin(const rx_cfg_t *c, float x, float v);

/* The verified fallback controller (brake when close to the limit, PD otherwise) */
float rx_safe_u(const rx_cfg_t *c, float x, float v, float sp);

typedef struct {
    uint8_t  mode, reason;
    uint8_t  quarantined;
    uint32_t hold_left;
    uint32_t strikes, strike_age;
    float    u_last;
} rx_guard_t;

typedef struct {
    int      guard_enabled;
    int      pilot_present;
    int      have_prop;
    uint32_t age_ticks;      /* ticks since the state the proposal was computed from */
    float    u_ai;
} rx_guard_in_t;

typedef struct {
    float   u_out;
    float   margin;
    int     intervened;      /* 1 on the tick control was taken away from the pilot */
    int     want_restart;    /* 1 when the pilot has used up its strikes */
} rx_guard_out_t;

void rx_guard_init(rx_guard_t *g);
void rx_guard_step(rx_guard_t *g, const rx_cfg_t *c, float x, float v, float sp,
                   const rx_guard_in_t *in, rx_guard_out_t *out);

/* ball-and-beam simulator (used by the sim backend and the tests) */
typedef struct { double x, v, theta; } rx_plant_t;
void rx_plant_step(rx_plant_t *p, const rx_cfg_t *c, float u_cmd, double dt);

/* the neural pilot (weights in policy_weights.h) */
float rx_policy(const rx_cfg_t *c, float x, float v, float sp);

#endif
