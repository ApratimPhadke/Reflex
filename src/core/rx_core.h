/* Reflex safety kernel - pure functions, no OS calls, no allocation.
 * Everything that decides whether an AI command reaches the actuator is in
 * rx_core.c so it can be reviewed and unit-tested on its own.
 * SPDX-License-Identifier: MIT */
#ifndef RX_CORE_H
#define RX_CORE_H

#include <stdint.h>
#include "rx_proto.h"

/* Plants. The guardian, its envelope check, the fallback controller and the simulator reach the plant only
 * through rx_plant_ops_t below; position x, velocity v and actuator command u are in the plant's own units.
 *   RX_PLANT_BEAM   ball on a tilting beam: x [m], u = beam angle [rad]           (the real rig)
 *   RX_PLANT_JOINT  one robot joint with a gravity load: x [rad], u = torque [N m] (simulation only; a stand-in
 *                   for an exoskeleton or surgical-robot joint) */
enum { RX_PLANT_BEAM = 0, RX_PLANT_JOINT = 1 };

typedef struct {
    float hz, dt;
    float half_len;      /* position limit: beam half length [m] / joint angle limit [rad] */
    float edge_margin;   /* keep-out strip at each end, same units */
    float k_acc;         /* beam: ball acceleration per sin(beam angle) [m/s^2] */
    float theta_max;     /* actuator limit: beam angle [rad] / joint torque [N m] */
    float slew;          /* actuator slew rate [rad/s] / [N m/s] (simulator + envelope sizing) */
    float t_delay;       /* worst-case time before braking takes effect [s] */
    float brake_eta;     /* fraction of ideal braking we are willing to rely on */
    float wn, zeta;      /* safe controller: natural frequency [rad/s], damping */
    float est_tau;       /* estimator time constant [s] */
    float max_age_s;     /* pilot deadline: older proposals are stale */
    float hold_s;        /* minimum time in SAFE after an intervention */
    float inner_margin;  /* margin required before AI gets control back [plant units] */
    float sp_limit;      /* setpoints are clamped to +/- this [plant units] */
    uint32_t strike_limit;   /* misbehaviour strikes before a restart is requested */
    float strike_window_s;
    int   plant;         /* RX_PLANT_BEAM (rx_cfg_default) or RX_PLANT_JOINT (rx_cfg_joint) */
    float inertia;       /* joint: link inertia about the axis [kg m^2] */
    float mgl;           /* joint: gravity torque with the link horizontal [N m] */
    float damping;       /* joint: viscous friction [N m s/rad] */
} rx_cfg_t;

void  rx_cfg_default(rx_cfg_t *c, float hz);
void  rx_cfg_hardware(rx_cfg_t *c);   /* adjustments for the real beam: slow, noisy sensor and a real servo */
void  rx_cfg_joint(rx_cfg_t *c, float hz);
/* the beam-direct rig (after rx_cfg_hardware): physical numbers measured in results/2026-11-24-beam-direct.
 * theta_max: the smaller side of the servo's authority around level [rad]; reversal_s: time to swing the
 * beam from +theta_max to -theta_max at the backend's slew limit. */
void  rx_cfg_beam_direct(rx_cfg_t *c, float theta_max, float reversal_s);   /* the robot-joint plant (replaces everything rx_cfg_default set) */

/* alpha-beta position/velocity estimator */
typedef struct { float x, v; int init; } rx_est_t;
void  rx_est_update(rx_est_t *e, const rx_cfg_t *c, float z);
void  rx_est_predict(rx_est_t *e, const rx_cfg_t *c);   /* one tick without a new measurement */

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

/* The plant interface. A new plant is one more table: its dynamics for the one-step lookahead, the
 * one-sided recoverable-set bound, the fallback controller, the simulator, and the command that holds it at rest. */
typedef struct {
    const char *name;            /* "beam", "joint" */
    const char *unit;            /* position unit for people: "mm" or "deg" */
    float       unit_scale;      /* position in plant units * unit_scale = position in `unit` */
    float (*accel)(const rx_cfg_t *c, float x, float v, float u);           /* x'' under command u */
    float (*margin_side)(const rx_cfg_t *c, float x, float v, float s);     /* rx_margin toward one end (s = +-1) */
    float (*safe_u)(const rx_cfg_t *c, float x, float v, float sp);         /* the fallback controller */
    void  (*step)(rx_plant_t *p, const rx_cfg_t *c, float u_cmd, double dt);/* simulator; p->theta = applied actuator */
    float (*u_eq)(const rx_cfg_t *c, float x);                              /* command that holds the plant at rest at x */
} rx_plant_ops_t;
const rx_plant_ops_t *rx_plant_ops(int plant);   /* RX_PLANT_BEAM or RX_PLANT_JOINT; anything else -> beam */

/* the neural pilot: a 3-12-1 tanh network. Frozen weight sets compiled in, all made by tools/train_policy.py:
 * RX_NET_SIM for the simulated beam (policy_weights.h), RX_NET_HW for the serial backend at 50 Hz with
 * rx_cfg_hardware() (policy_weights_hw.h, --hardware), RX_NET_JOINT for the joint (policy_weights_joint.h, --plant joint).
 * The command is u_eq(x) + theta_max * tanh(net(e, v, x) - net(0, 0, x)), clamped to +-theta_max: exactly the
 * holding command with the plant at rest on the setpoint. */
#define RX_NET_IN 3
#define RX_NET_H  12
enum { RX_NET_SIM = 0, RX_NET_HW = 1, RX_NET_JOINT = 2 };
typedef struct {
    const float (*w1)[RX_NET_IN];   /* [RX_NET_H][RX_NET_IN] */
    const float *b1, *w2;           /* [RX_NET_H] */
    float b2;
    float s_e, s_v, s_x;            /* input scales: (sp - x) / s_e, v / s_v, x / s_x */
} rx_net_t;
const rx_net_t *rx_policy_net(int which);   /* RX_NET_SIM, RX_NET_HW or RX_NET_JOINT; anything else -> RX_NET_SIM */
float rx_policy_with(const rx_cfg_t *c, const rx_net_t *n, float x, float v, float sp);
float rx_policy(const rx_cfg_t *c, float x, float v, float sp);   /* = rx_policy_with(c, RX_NET_SIM, ...) */

#endif
