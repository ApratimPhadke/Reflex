/* Reflex - shared wire formats and the shared-memory telemetry block.
 * SPDX-License-Identifier: MIT */
#ifndef RX_PROTO_H
#define RX_PROTO_H

#include <stdint.h>
#include <stdatomic.h>

#define RX_VERSION      4
#define RX_MAGIC        0x52464c58u /* "RFLX" */
#define RX_SHM_NAME     "/reflex"
#define RX_RING         8192u       /* telemetry records kept (power of two) */
#define RX_EVENTS       256u        /* event log entries kept (power of two) */
#define RX_LEARN_HIST   512u        /* learning-curve points kept (power of two) */
#define RX_HIST_BINS    24          /* latency histogram: bin i = [2^i, 2^(i+1)) microseconds, bin 0 = <2us */

/* ---- pilot <-> guardian messages ------------------------------------ */
#define RX_MSG_PROPOSAL 0x5258u     /* above _IO_MAX so it can never look like a QNX system message */

typedef struct {            /* guardian -> pilot (the reply) */
    uint32_t seq;           /* control tick this state belongs to */
    uint32_t fault;         /* test hook: fault the pilot must emulate (rx_fault_e) */
    uint32_t fault_arg;
    float    x;             /* estimated ball position [m], 0 = beam centre */
    float    v;             /* estimated ball velocity [m/s] */
    float    sp;            /* setpoint [m] */
    uint32_t mode;          /* rx_mode_e: who was in control this tick (feedback for a learning pilot) */
    uint32_t reason;        /* rx_reason_e */
    uint32_t train;         /* 1 = a learning pilot may explore, 0 = fly your best weights */
} rx_state_t;

typedef struct {            /* pilot -> guardian (the send) */
    uint32_t type;          /* RX_MSG_PROPOSAL */
    uint32_t seq;           /* seq of the state this command was computed from */
    float    u;             /* proposed beam angle [rad] */
    uint32_t flags;         /* RX_PF_* */
    uint32_t episode;       /* learning pilots: finished episodes (display only, never trusted) */
    float    ep_cost;       /* learning pilots: cost of the last finished episode */
    float    ep_avg;        /* learning pilots: smoothed cost */
    uint32_t reserved;
} rx_proposal_t;
#define RX_PF_HELLO   1u    /* first message of a fresh pilot, carries no command */
#define RX_PF_LEARNER 2u

/* ---- enums ----------------------------------------------------------- */
typedef enum { RX_MODE_AI = 0, RX_MODE_SAFE = 1, RX_MODE_BYPASS = 2 } rx_mode_e;

typedef enum {
    RX_R_NONE = 0,
    RX_R_NO_PILOT,      /* pilot process not connected (crashed / restarting) */
    RX_R_STALE,         /* pilot missed its deadline */
    RX_R_NONFINITE,     /* NaN / Inf command */
    RX_R_RANGE,         /* command outside actuator limits */
    RX_R_ENVELOPE,      /* command would leave the recoverable set */
    RX_R_HOLD,          /* pilot looks fine again, guardian is still holding */
    RX_R_QUARANTINE,    /* pilot disabled after repeated offences */
    RX_R_COUNT
} rx_reason_e;

typedef enum {
    RX_F_NONE = 0,
    RX_F_NAN,           /* pilot outputs NaN */
    RX_F_FLIP,          /* pilot output sign-inverted (adversarial / positive feedback) */
    RX_F_STUCK,         /* pilot output stuck at +max */
    RX_F_HUGE,          /* pilot output far outside actuator range */
    RX_F_SLOW,          /* pilot sleeps fault_arg ms before answering */
    RX_F_SPIN,          /* pilot enters an infinite busy loop */
    RX_F_COUNT
} rx_fault_e;

static const char *const rx_mode_name[]   = { "AI", "SAFE", "BYPASS" };
static const char *const rx_reason_name[] = { "none", "no-pilot", "stale", "non-finite", "range",
                                              "envelope", "hold", "quarantine" };
static const char *const rx_fault_name[]  = { "none", "nan", "flip", "stuck", "huge", "slow", "spin" };

/* ---- event log (everything the guardian would otherwise print) -------- */
typedef enum {
    RX_E_GUARDIAN_START = 1, RX_E_INTERVENE, RX_E_AI_RESUMED, RX_E_PILOT_SPAWN, RX_E_PILOT_DIED,
    RX_E_KILL_HUNG, RX_E_KILL_STRIKES, RX_E_QUARANTINE, RX_E_RELEASE, RX_E_BALL_LOST, RX_E_SPOOF,
    RX_E_SENSOR_LOST, RX_E_LINK_DOWN, RX_E_LINK_UP, RX_E_NODE_INFO, RX_E_NODE_OLD,
    /* pilot over UDP (guest supervision, src/core/rx_guest.h) */
    RX_E_GUEST_DOWN, RX_E_GUEST_UP, RX_E_GUEST_RESTART, RX_E_GUEST_QUARANTINE, RX_E_GUEST_CMD_FAILED, RX_E_GUEST_INSTANCE,
    RX_E_COUNT
} rx_evcode_e;
static const char *const rx_event_name[] = { "?", "guardian started", "guardian took over", "AI back in control",
    "pilot spawned", "pilot died", "pilot killed: unresponsive", "pilot killed: 3 unsafe commands",
    "pilot quarantined", "quarantine released", "BALL LOST", "spoofed sender refused", "sensor lost",
    "I/O node link down", "I/O node link up", "I/O node identified (arg = protocol*10 + emulated)",
    "I/O node firmware speaks protocol v1: reflash it (make fw && make flash)",
    "GUEST DOWN: no valid proposal (arg = ms since the last)", "guest up (arg = ms it was down, 0 = first contact)",
    "guest restart command issued (arg 1 = timed out, 2 = never came up, 3 = unsafe commands)",
    "GUEST QUARANTINED: 5 restarts in 10 min (rxctl release)",
    "guest restart command failed (arg = exit status, -1 = ran over 60 s, -2 = could not start)",
    "new pilot instance (arg = its id)" };
typedef struct { uint64_t t_ns; uint32_t seq; uint16_t code; int32_t arg; } rx_event_t;

/* ---- telemetry record (one per control tick) ------------------------- */
typedef struct {
    uint64_t t_ns;
    uint32_t seq;
    uint32_t lat_ns;        /* wake-up latency of this tick */
    uint32_t exec_ns;       /* time spent inside the guardian this tick */
    float    x, v, sp;
    float    x_true;        /* simulator only: ground truth */
    float    u_ai;          /* what the pilot asked for (NaN if nothing usable) */
    float    u_out;         /* what the guardian sent to the actuator */
    float    margin;        /* distance to the edge of the recoverable set [m] */
    uint8_t  mode, reason, pilot_present, lost;
} rx_rec_t;

/* ---- shared memory block --------------------------------------------- */
typedef struct {
    uint32_t magic, version;

    /* endpoint: how a pilot finds the guardian */
    _Atomic int32_t guardian_pid;
    _Atomic int32_t chid;
    _Atomic int32_t pilot_pid;

    /* plant description (written once by the guardian, read by tools) */
    float hz, half_len, theta_max;
    char  backend[16];

    /* operator / fault-injection controls (written by tools, read by guardian) */
    _Atomic uint32_t guard_enabled;
    _Atomic uint32_t fault, fault_arg;
    _Atomic uint32_t sp_auto;
    _Atomic int32_t  sp_manual_mm;
    _Atomic uint32_t cmd_reset_stats;
    _Atomic uint32_t cmd_release_quarantine;
    _Atomic uint32_t training;              /* learning mode: protect as usual, but do not punish exploration */
    _Atomic uint32_t sp_period_ms;          /* period of the automatic setpoint square wave (one half) */

    /* live counters (written by the guardian) */
    _Atomic uint64_t ticks;
    _Atomic uint64_t interventions[RX_R_COUNT];
    _Atomic uint64_t deadline_miss;         /* ticks that woke later than half a period */
    _Atomic uint64_t lat_hist[RX_HIST_BINS];
    _Atomic uint32_t lat_max_ns, exec_max_ns;
    _Atomic uint32_t pilot_restarts, pilot_kills, balls_lost, guardian_starts;
    _Atomic uint32_t mode, reason, quarantined, pilot_present;
    _Atomic uint32_t rejected_senders;

    /* simulator state lives here so a restarted guardian resumes the same world */
    struct { double x, v, theta, t_last_ns; uint32_t valid, lost_ticks; uint64_t rng; } sim;

    /* learning curve reported by a learning pilot; interventions are counted by the guardian itself */
    _Atomic uint32_t learn_episodes;
    float    learn_avg;
    float    learn_cost[RX_LEARN_HIST];
    uint16_t learn_iv[RX_LEARN_HIST];

    _Atomic uint64_t ev_head;
    rx_event_t events[RX_EVENTS];

    _Atomic uint64_t head;                  /* next record index to be written */
    rx_rec_t ring[RX_RING];

    /* serial backend (appended so older segments keep their layout; zero = unknown) */
    char  calibrated_at[40];                /* reflex.conf calibrated_at, or "uncalibrated" (written once at start) */
    _Atomic uint32_t node_link;             /* 1 = I/O node link up */
    _Atomic uint32_t node_proto;            /* from the node's "I" line; 0 until seen. Written last, after the two below */
    _Atomic uint32_t node_emulated;
    char  node_board[24];
    /* demo: a caption the dashboard shows as a banner (rxctl caption / rxctl script) */
    _Atomic uint32_t caption_seq;           /* odd while being written */
    char  caption[96];
    /* operator control: fly on the fallback controller alone, ignoring the pilot (rxctl bench, dashboard) */
    _Atomic uint32_t force_fallback;
    /* last AI-vs-fallback benchmark (rxctl bench), shown on the dashboard */
    struct {
        _Atomic uint32_t state;             /* 0 none, 1 running with the AI, 2 running fallback-only, 3 done */
        _Atomic uint32_t step, nsteps;
        float settle_s[2], overshoot_mm[2], rms_mm[2];   /* [0] = AI, [1] = fallback only */
        uint32_t interventions[2], unsettled[2];
    } bench;
    char  plant[16];                        /* "beam" or "joint" (written once at start) */
    float edge_margin;                      /* keep-out strip before each end [plant units] (written once at start) */
    /* pilot over UDP (pilot_transport = udp, src/ipc/rx_ipc_udp.c); all zero on the local transport.
     * The udp_* counters are written by the guardian's UDP helper threads (atomics only, never the event log). */
    char  pilot_transport[8];               /* "local" or "udp" (written once at start) */
    char  pilot_peer[24];                   /* the only address proposals are accepted from (written once at start) */
    _Atomic uint64_t udp_rx_ok, udp_rx_malformed, udp_rx_peer, udp_rx_replay, udp_tx_ok, udp_tx_err;
    _Atomic uint32_t udp_boot_id;           /* sender instance of the last accepted proposal (display only) */
    _Atomic uint32_t udp_boots;             /* sender instances seen (a restarted pilot or guest is a new one) */
    _Atomic uint32_t guest_state;           /* rx_guest_state_e (src/core/rx_guest.h); 0 on the local transport */
    _Atomic uint32_t guest_restarts;        /* restart commands issued */
    _Atomic uint32_t guest_age_ms;          /* time since the last valid proposal (updated ~10x/s) */
    _Atomic uint32_t guest_cmd_running;     /* the restart command is running */
} rx_shm_t;

#endif
