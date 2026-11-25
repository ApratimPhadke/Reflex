/* reflex-guardian - the hard-real-time safety reflex.
 *
 * One thread, one blocking call. Every control period it:
 *   1. reads the sensor            4. sends the command to the actuator
 *   2. judges the pilot's proposal 5. replies to the pilot with the new state
 *   3. picks pilot or safe command 6. supervises the pilot process
 *
 * Rules this file keeps inside the control loop:
 *   - never wait for the pilot        - never allocate
 *   - never take a lock               - never print (events go to a wait-free log in
 *                                       shared memory; a console can block for milliseconds)
 *
 * SPDX-License-Identifier: MIT */
#include <errno.h>
#include <libgen.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <spawn.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include "rx_core.h"
#include "rx_shm.h"
#include "rx_ipc.h"
#include "rx_ipc_udp.h"
#include "rx_guest.h"
#include "rx_io.h"

#if defined(__QNXNTO__) && !defined(RX_NO_TRACE)
#include <sys/trace.h>
#define RX_TRACE(...) trace_logf(_NTO_TRACE_USERFIRST + 42, __VA_ARGS__)   /* visible in the System Profiler */
#else
#define RX_TRACE(...) ((void)0)
#endif

extern char **environ;

static volatile sig_atomic_t g_stop;
static void on_stop(int s) { (void)s; g_stop = 1; }

typedef struct {
    float hz; int prio, pilot_prio; int sporadic, no_pilot, fresh, verbose, learn;
    const char *backend, *dev, *weights; char pilot_path[512];
    int hardware;                  /* serial backend: tell the pilot it flies the real rig (set from --backend) */
    const char *plant;             /* "beam" (default) or "joint" */
    const char *conf; int uncalibrated, check_conf;
    int pilot_cred; uid_t pilot_uid; gid_t pilot_gid;   /* spawn the pilot with these credentials (QNX) */
    const char *pilot_user;
} opts_t;

static void usage(void)
{
    puts("reflex-guardian [options]\n"
         "  --hz N             control rate (default 1000 for sim, 50 for serial)\n"
         "  --backend sim|serial|beam   beam = the Pi drives servo and sensors itself (beam-direct rig, QNX)\n"
         "  --plant beam|joint simulated plant (default beam); joint = a robot joint with gravity load, sim only\n"
         "  --dev PATH         serial device of the I/O node (e.g. /dev/serusb1)\n"
         "  --conf PATH        rig calibration written by rxcal (default: reflex.conf next to this binary);\n"
         "                     also says where the pilot runs: pilot_transport = local | udp (docs/HYPERVISOR.md)\n"
         "  --uncalibrated     serial backend without reflex.conf, on built-in defaults (debugging only)\n"
         "  --check-conf       only check reflex.conf: exit 0 if valid, 2 if not\n"
         "  --pilot-user U:G   run the pilot with this uid:gid (QNX). Default when we run as root:\n"
         "                     the owner of the pilot executable; the pilot never runs as root\n"
         "  --prio N           guardian priority (default 50)\n"
         "  --pilot-prio N     pilot priority (default 10)\n"
         "  --sporadic         QNX: run the pilot under SCHED_SPORADIC with a CPU budget\n"
         "  --learn            pilot learns online, starting from an untrained network\n"
         "  --learn-from-sim   pilot learns online, starting from the simulator-trained weights\n"
         "  --weights PATH     where the learning pilot keeps its weights (default ./reflex-policy.bin)\n"
         "  --pilot PATH       pilot executable (default: reflex-pilot next to this binary)\n"
         "  --no-pilot         do not spawn a pilot (fallback controller only)\n"
         "  --fresh            reset the simulated world and all counters\n"
         "  --verbose          print a status line once per second (debugging only: printing\n"
         "                     from a real-time loop adds latency)");
}

/* Starting a process is a blocking request to the process manager that allocates memory and
 * loads an executable: on the Pi 4 it took up to ~0.8 ms inside a 1 ms control tick. So the control
 * loop never spawns. It raises `req` (an atomic store) and a helper thread at a lower priority does
 * the work; the loop picks the result up from `result` on a later tick. The two threads share
 * nothing else, and the loop never waits for the helper. */
typedef struct {
    const opts_t *o;
    rx_server_t  *srv;
    int           chid, prio;
    float         hz;
    _Atomic int   req, stop;
    _Atomic pid_t result;          /* >0 new pilot pid, -1 spawn failed, 0 nothing new */
} spawner_t;

static pid_t spawn_pilot(const opts_t *o, rx_server_t *srv, int chid, float hz)
{
    posix_spawnattr_t at;
    posix_spawnattr_init(&at);
    struct sched_param sp; memset(&sp, 0, sizeof sp);
#ifdef __QNXNTO__
    sp.sched_priority = o->pilot_prio;                       /* never let the child inherit our priority */
    int policy = SCHED_RR;
    if (o->sporadic) {
        /* The pilot gets prompt service at its normal priority, but only for 3 ms in every 10 ms.
         * Once the budget is gone it drops to priority 1 until the budget is replenished, so even
         * a spinning pilot cannot starve the rest of the system. Set at creation, so it applies
         * before the pilot runs its first instruction. */
        policy = SCHED_SPORADIC;
        sp.sched_ss_low_priority = 1;
        /* Every block/run cycle uses one pending replenishment, and a thread that reaches
         * max_repl drops to low priority regardless of budget. The pilot wakes once per control
         * tick, i.e. hz * 10 ms times per replenishment period (10 at 1 kHz). With the old fixed
         * value of 8 a perfectly healthy pilot at 1 kHz ran permanently at priority 1 (seen with
         * pidin on the Pi 4). Size it with 2x headroom; SS_REPL_MAX is USHRT_MAX on QNX 8.0. */
        int repl = (int)(hz * 0.010f + 0.999f) * 2 + 4;
        sp.sched_ss_max_repl = repl;
        sp.sched_ss_repl_period.tv_nsec = 10 * 1000000;
        sp.sched_ss_init_budget.tv_nsec = 3 * 1000000;
    }
    posix_spawnattr_setschedpolicy(&at, policy);
    posix_spawnattr_setschedparam(&at, &sp);
    /* HOLD: the child is created stopped. We register its pid with the server before it can run,
     * so its very first message is never mistaken for a spoofing attempt. */
    unsigned xf = POSIX_SPAWN_SETSCHEDULER | POSIX_SPAWN_SETSCHEDPARAM | POSIX_SPAWN_HOLD;
    /* The pilot is untrusted: a guardian running as root (e.g. started by HAM) never hands it root.
     * It runs as o->pilot_uid/gid, decided once at start-up (see main). */
    if (o->pilot_cred) { posix_spawnattr_setcred(&at, o->pilot_uid, o->pilot_gid); xf |= POSIX_SPAWN_SETCRED; }
    posix_spawnattr_setxflags(&at, xf);
#else
    sp.sched_priority = 0;
    posix_spawnattr_setschedpolicy(&at, SCHED_OTHER);
    posix_spawnattr_setschedparam(&at, &sp);
    posix_spawnattr_setflags(&at, POSIX_SPAWN_SETSCHEDULER | POSIX_SPAWN_SETSCHEDPARAM);
#endif

    /* The pilot is told where to send its proposals and nothing else. It does not get to map
     * the guardian's shared memory. */
    char a_pid[16], a_chid[16], a_hz[16];
    snprintf(a_pid, sizeof a_pid, "%d", (int)getpid());
    snprintf(a_chid, sizeof a_chid, "%d", chid);
    snprintf(a_hz, sizeof a_hz, "%.3f", hz);
    char *argv[12]; int n = 0;
    argv[n++] = (char *)o->pilot_path; argv[n++] = a_pid; argv[n++] = a_chid; argv[n++] = a_hz;
    if (o->hardware) argv[n++] = "--hardware";              /* real rig: fly the hardware-trained weights */
    if (!strcmp(o->plant, "joint")) { argv[n++] = "--plant"; argv[n++] = "joint"; }
    if (o->learn == 1) argv[n++] = "--learn";
    if (o->learn == 2) argv[n++] = "--learn-from-sim";
    if (o->learn) { argv[n++] = "--weights"; argv[n++] = (char *)o->weights; }
    argv[n] = NULL;

    pid_t pid = 0;
    int rc = posix_spawn(&pid, o->pilot_path, NULL, &at, argv, environ);
    posix_spawnattr_destroy(&at);
    if (rc != 0) return 0;
    rx_server_allow(srv, pid);
#ifdef __QNXNTO__
    kill(pid, SIGCONT);                                      /* release the held child */
#endif
    return pid;
}

static void *spawner_main(void *arg)
{
    spawner_t *s = arg;
    struct sched_param sp = { .sched_priority = s->prio };
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);  /* best effort; below the loop, above the pilot */
    struct timespec nap = { 0, 2 * 1000000 };
    while (!atomic_load(&s->stop)) {
        if (atomic_exchange(&s->req, 0)) {
            pid_t p = spawn_pilot(s->o, s->srv, s->chid, s->hz);
            atomic_store(&s->result, p > 0 ? p : -1);
        } else nanosleep(&nap, NULL);
    }
    return NULL;
}

/* Guest restarts (pilot_transport = udp). guest_restart_cmd may take seconds (it might talk to qvm or ssh),
 * so, like spawning, it is run by a helper thread; the control loop only raises `req` and later reads
 * `status`. The command must return once the guest is on its way (start qvm in the background). */
typedef struct {
    const char   *cmd;
    int           prio, child_prio;
    _Atomic int   req, stop, done;
    _Atomic int   status;          /* exit status, -1 = ran over 60 s (killed), -2 = could not start */
    rx_shm_t     *shm;
} runner_t;

static int run_restart_cmd(const runner_t *r)
{
    posix_spawnattr_t at;
    posix_spawnattr_init(&at);
    struct sched_param sp; memset(&sp, 0, sizeof sp);
#ifdef __QNXNTO__
    sp.sched_priority = r->child_prio;                       /* not our helper's priority */
    posix_spawnattr_setschedpolicy(&at, SCHED_RR);
#else
    posix_spawnattr_setschedpolicy(&at, SCHED_OTHER);
#endif
    posix_spawnattr_setschedparam(&at, &sp);
    posix_spawnattr_setflags(&at, POSIX_SPAWN_SETSCHEDULER | POSIX_SPAWN_SETSCHEDPARAM);
    char *argv[] = { "/bin/sh", "-c", (char *)r->cmd, NULL };
    pid_t pid;
    int rc = posix_spawn(&pid, "/bin/sh", NULL, &at, argv, environ);
    posix_spawnattr_destroy(&at);
    if (rc != 0) return -2;
    struct timespec nap = { 0, 50 * 1000000 };
    for (int i = 0; i < 1200; i++) {                         /* 60 s */
        int ws;
        if (waitpid(pid, &ws, WNOHANG) == pid) return WIFEXITED(ws) ? WEXITSTATUS(ws) : 128 + (WIFSIGNALED(ws) ? WTERMSIG(ws) : 0);
        nanosleep(&nap, NULL);
    }
    kill(pid, SIGKILL); waitpid(pid, NULL, 0);
    return -1;
}

static void *runner_main(void *arg)
{
    runner_t *r = arg;
    struct sched_param sp = { .sched_priority = r->prio };
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);  /* best effort */
    struct timespec nap = { 0, 20 * 1000000 };
    while (!atomic_load(&r->stop)) {
        if (atomic_exchange(&r->req, 0)) {
            atomic_store(&r->shm->guest_cmd_running, 1);
            atomic_store(&r->status, run_restart_cmd(r));
            atomic_store(&r->shm->guest_cmd_running, 0);
            atomic_store(&r->done, 1);
        } else nanosleep(&nap, NULL);
    }
    return NULL;
}

int main(int argc, char **argv)
{
    /* First thing: a guardian restarted by HAM may inherit a stderr whose reader is gone (a closed
     * ssh session). Without this, its first diagnostic line raises SIGPIPE and kills it, HAM
     * restarts it, and it dies again: seen on the Pi 4 as ~1700 restarts in a few seconds. */
    signal(SIGPIPE, SIG_IGN);
    opts_t o = { 0, 50, 10, 0, 0, 0, 0, 0, "sim", "/dev/serusb1", "./reflex-policy.bin", "", 0, NULL, NULL, 0, 0, 0, 0, 0, NULL };
    {   /* default pilot path: next to our own binary */
        char tmp[512]; snprintf(tmp, sizeof tmp, "%s", argv[0]);
        snprintf(o.pilot_path, sizeof o.pilot_path, "%s/reflex-pilot", dirname(tmp));
    }
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i], *n = i + 1 < argc ? argv[i + 1] : NULL;
        if (!strcmp(a, "--hz") && n) { o.hz = (float)atof(n); i++; }
        else if (!strcmp(a, "--backend") && n) { o.backend = n; i++; }
        else if (!strcmp(a, "--dev") && n) { o.dev = n; i++; }
        else if (!strcmp(a, "--conf") && n) { o.conf = n; i++; }
        else if (!strcmp(a, "--uncalibrated")) o.uncalibrated = 1;
        else if (!strcmp(a, "--check-conf")) o.check_conf = 1;
        else if (!strcmp(a, "--pilot-user") && n) { o.pilot_user = n; i++; }
        else if (!strcmp(a, "--prio") && n) { o.prio = atoi(n); i++; }
        else if (!strcmp(a, "--pilot-prio") && n) { o.pilot_prio = atoi(n); i++; }
        else if (!strcmp(a, "--pilot") && n) { snprintf(o.pilot_path, sizeof o.pilot_path, "%s", n); i++; }
        else if (!strcmp(a, "--weights") && n) { o.weights = n; i++; }
        else if (!strcmp(a, "--sporadic")) o.sporadic = 1;
        else if (!strcmp(a, "--learn")) o.learn = 1;
        else if (!strcmp(a, "--learn-from-sim")) o.learn = 2;
        else if (!strcmp(a, "--plant") && n) { o.plant = n; i++; }
        else if (!strcmp(a, "--no-pilot")) o.no_pilot = 1;
        else if (!strcmp(a, "--fresh")) o.fresh = 1;
        else if (!strcmp(a, "--verbose")) o.verbose = 1;
        else if (!strcmp(a, "--quiet")) { /* default now, accepted for compatibility */ }
        else { usage(); return strcmp(a, "--help") ? 2 : 0; }
    }
    int serial = !strcmp(o.backend, "serial"), beam = !strcmp(o.backend, "beam");
    int hw = serial || beam;                                /* a real rig: calibration required */
    if (!hw && strcmp(o.backend, "sim")) { fprintf(stderr, "--backend must be sim, serial or beam\n"); return 2; }
    o.hardware = hw;
    if (!o.plant) o.plant = "beam";
    int joint = !strcmp(o.plant, "joint");
    if (!joint && strcmp(o.plant, "beam")) { fprintf(stderr, "--plant must be beam or joint\n"); return 2; }
    if (joint && hw) { fprintf(stderr, "--plant joint is simulated only (the real rig is a ball and beam)\n"); return 2; }
    if (joint && o.learn) { fprintf(stderr, "--plant joint: the online learner is for the beam only; the joint flies its frozen pilot\n"); return 2; }
    if (o.hz <= 0) o.hz = hw ? 50.0f : 1000.0f;
    if (o.hz < 10 || o.hz > 4000) { fprintf(stderr, "--hz must be 10..4000\n"); return 2; }

    rx_cfg_t cfg; rx_cfg_default(&cfg, o.hz);
    if (hw) rx_cfg_hardware(&cfg);
    if (joint) rx_cfg_joint(&cfg, o.hz);

    /* rig calibration: the serial backend refuses to run on guessed values */
    rx_conf_t conf; rx_conf_default(&conf);
    char conf_path[512];
    if (o.conf) snprintf(conf_path, sizeof conf_path, "%s", o.conf);
    else { char tmp[512]; snprintf(tmp, sizeof tmp, "%s", argv[0]); snprintf(conf_path, sizeof conf_path, "%s/reflex.conf", dirname(tmp)); }
    if (o.check_conf) {                                     /* used by reflex-demo.sh to pick the backend */
        char err[256];
        if (rx_conf_load(&conf, conf_path, err, sizeof err) != 0) { fprintf(stderr, "%s\n", err); return 2; }
        printf("calibration %s (%s)\n", conf.calibrated_at, conf_path);
        return 0;
    }
    if (hw && !o.uncalibrated) {
        char err[256];
        if (rx_conf_load(&conf, conf_path, err, sizeof err) != 0) {
            fprintf(stderr, "[guardian] no valid calibration: %s\n"
                            "[guardian] run the calibration wizard first:  ./rxcal --dev %s\n"
                            "[guardian] (--uncalibrated runs on built-in defaults, for debugging only)\n", err, o.dev);
            return 2;
        }
        /* a measured beam can only make the kernel's beam shorter, never longer */
        if (conf.half_len_m < cfg.half_len) cfg.half_len = conf.half_len_m;
        if (beam) {
            /* the servo's authority around level, the smaller side; never more than the kernel's own limit */
            if (!conf.servo_min_us) { fprintf(stderr, "[guardian] --backend beam needs servo_min_us/servo_max_us in %s\n", conf_path); return 2; }
            float up = (float)(conf.servo_max_us - conf.level_us), dn = (float)(conf.level_us - conf.servo_min_us);
            /* +u (ball toward +x) moves the pulse by servo_dir: which side of level has the room for it */
            float pos = (conf.servo_dir < 0 ? dn : up) / (conf.us_per_mrad * 1000.0f), neg = (conf.servo_dir < 0 ? up : dn) / (conf.us_per_mrad * 1000.0f);
            float th = pos < neg ? pos : neg;                     /* the AI's range: the smaller side */
            if (th > cfg.theta_max) th = cfg.theta_max;
            float fbp = pos > th ? pos : th;                      /* the fallback alone, toward +x */
            if (fbp > 0.21f) fbp = 0.21f;
            float reversal = (fbp + th) * 1000.0f * conf.us_per_mrad / (float)RX_BEAM_SLEW_US * cfg.dt;
            rx_cfg_beam_direct(&cfg, th, fbp, reversal);
            fprintf(stderr, "[guardian] beam-direct: theta_max %.1f mrad (%.2f deg, AI and fallback), fallback toward +x up to %.1f mrad (%.2f deg); "
                            "from %d..%d us around %d, t_delay %.0f ms, half_len %.0f mm\n",
                    th * 1e3, th * 57.29578f, fbp * 1e3, fbp * 57.29578f, conf.servo_min_us, conf.servo_max_us, conf.level_us, cfg.t_delay * 1e3, cfg.half_len * 1e3);
        }
    } else if (o.conf || access(conf_path, R_OK) == 0) {   /* sim (or --uncalibrated): only the pilot transport keys matter */
        char err[256];
        if (rx_conf_load_any(&conf, conf_path, err, sizeof err) != 0) { fprintf(stderr, "[guardian] %s\n", err); return 2; }
    }
    /* where the pilot runs: spawned here and reached by message passing, or somewhere else over UDP
     * (e.g. a Linux guest under QNX Hypervisor). Either way its proposals are judged identically. */
    const int udp = conf.pilot_udp;
    /* a pilot across a network answers later than one on the same CPU: reflex.conf may set how old its command
     * may be (pilot_max_age_ms, chosen from a measured round trip). This is the stale check only; the command is
     * still judged by the envelope, range and NaN checks from the current state, every tick. */
    if (udp && conf.pilot_max_age_ms > 0) cfg.max_age_s = conf.pilot_max_age_ms * 1e-3f;
    const int spawn_local = !o.no_pilot && !udp;
    {   /* who the untrusted pilot runs as */
        if (o.pilot_user) {
            unsigned u, g;
            if (sscanf(o.pilot_user, "%u:%u", &u, &g) != 2) { fprintf(stderr, "--pilot-user wants UID:GID\n"); return 2; }
            o.pilot_cred = 1; o.pilot_uid = (uid_t)u; o.pilot_gid = (gid_t)g;
        } else if (geteuid() == 0 && !o.no_pilot && !udp) {   /* a UDP pilot is not ours to spawn */
            struct stat st;
            if (stat(o.pilot_path, &st) != 0 || st.st_uid == 0) {
                fprintf(stderr, "[guardian] running as root, and %s is owned by root (or missing): refusing to run the\n"
                                "[guardian] untrusted pilot as root. Give it an ordinary owner, or pass --pilot-user UID:GID.\n", o.pilot_path);
                return 2;
            }
            o.pilot_cred = 1; o.pilot_uid = st.st_uid; o.pilot_gid = st.st_gid;
        }
#ifndef __QNXNTO__
        if (o.pilot_cred && geteuid() == 0) {
            fprintf(stderr, "[guardian] --pilot-user / root is only supported on QNX (posix_spawnattr_setcred)\n");
            return 2;
        }
        o.pilot_cred = 0;
#endif
    }

    rx_shm_t *shm = rx_shm_open(1);
    if (!shm) { perror("shm_open " RX_SHM_NAME); return 1; }
    if (o.fresh) rx_shm_defaults(shm);
    {   /* a previous guardian may have died and left its pilot behind */
        pid_t old = atomic_load(&shm->pilot_pid);
        if (old > 0) kill(old, SIGKILL);
        atomic_store(&shm->pilot_pid, 0);
    }
    shm->hz = cfg.hz; shm->half_len = cfg.half_len; shm->theta_max = cfg.theta_max; shm->edge_margin = cfg.edge_margin;
    snprintf(shm->backend, sizeof shm->backend, "%s", o.backend);
    snprintf(shm->plant, sizeof shm->plant, "%s", rx_plant_ops(cfg.plant)->name);
    snprintf(shm->calibrated_at, sizeof shm->calibrated_at, "%s", hw ? (o.uncalibrated ? "uncalibrated" : conf.calibrated_at) : "-");
    snprintf(shm->pilot_transport, sizeof shm->pilot_transport, "%s", udp ? "udp" : "local");
    snprintf(shm->pilot_peer, sizeof shm->pilot_peer, "%s", udp ? conf.pilot_peer : "");
    atomic_store(&shm->node_link, 0); atomic_store(&shm->node_proto, 0);
    atomic_store(&shm->guardian_pid, getpid());
    atomic_fetch_add(&shm->guardian_starts, 1);
    atomic_store(&shm->pilot_present, 0);
    if (o.learn) atomic_store(&shm->training, 1);

    if (beam && o.uncalibrated) { fprintf(stderr, "[guardian] --backend beam never runs uncalibrated\n"); return 2; }
    rx_io_t *io = serial ? rx_io_serial_open(&cfg, o.dev, shm, &conf)
                : beam   ? rx_io_beam_open(&cfg, shm, &conf, "beam.csv")
                :          rx_io_sim_open(&cfg, shm, o.fresh);
    if (!io) return 1;

    /* real-time hygiene: no page faults, fixed priority */
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) perror("[guardian] mlockall (continuing)");
    struct sched_param sp = { .sched_priority = o.prio };
    int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (rc) fprintf(stderr, "[guardian] WARNING: could not get SCHED_FIFO %d (%s) - timing numbers will not be meaningful\n",
                    o.prio, strerror(rc));

    signal(SIGINT, on_stop); signal(SIGTERM, on_stop);

    uint64_t period = (uint64_t)(1e9 / cfg.hz);
    rx_server_t *srv = rx_server_create(period, o.prio, shm);
    if (!srv) return 1;

    static spawner_t spw;                                    /* static: lives as long as the helper thread */
    spw.o = &o; spw.srv = srv; spw.chid = rx_server_chid(srv); spw.hz = cfg.hz;
    spw.prio = o.prio - 10 > o.pilot_prio ? o.prio - 10 : o.pilot_prio + 1;
    pthread_t spw_thread;
    if (spawn_local && (rc = pthread_create(&spw_thread, NULL, spawner_main, &spw)) != 0) {
        fprintf(stderr, "[guardian] cannot start the pilot spawner: %s\n", strerror(rc));
        return 1;
    }
    rx_udp_t *udpl = NULL;
    static runner_t run;                                     /* static: lives as long as its thread */
    pthread_t run_thread;
    rx_guest_t gst;
    if (udp) {
        char err[256];
        udpl = rx_udp_open(conf.pilot_bind, conf.pilot_peer, spw.prio, shm, err, sizeof err);
        if (!udpl) { fprintf(stderr, "[guardian] pilot over UDP: %s\n", err); return 1; }
        run.cmd = conf.guest_restart_cmd; run.prio = spw.prio; run.child_prio = o.pilot_prio; run.shm = shm;
        if (conf.guest_restart_cmd[0] && (rc = pthread_create(&run_thread, NULL, runner_main, &run)) != 0) {
            fprintf(stderr, "[guardian] cannot start the guest restart helper: %s\n", strerror(rc));
            return 1;
        }
        rx_guest_init(&gst, conf.guest_timeout_s, conf.guest_boot_s, 0.0);
        atomic_store(&shm->guest_state, gst.state); atomic_store(&shm->guest_cmd_running, 0);
        if (!conf.guest_restart_cmd[0])
            fprintf(stderr, "[guardian] no guest_restart_cmd in %s: a guest that goes down is logged, not restarted\n", conf_path);
    }

    char pilot_desc[600];
    if (udp) {
        unsigned ma = (unsigned)(cfg.max_age_s * cfg.hz + 0.5f); if (ma < 2) ma = 2;     /* as rx_guard_step rounds it */
        snprintf(pilot_desc, sizeof pilot_desc, "udp %s <- %s, stale after %u ticks (%.0f ms)%s", conf.pilot_bind, conf.pilot_peer,
                 ma, ma * 1e3 / cfg.hz, conf.pilot_max_age_ms > 0 ? " from pilot_max_age_ms" : "");
    }
    else snprintf(pilot_desc, sizeof pilot_desc, "%s", o.no_pilot ? "(none)" : o.pilot_path);
    fprintf(stderr, "[guardian] pid %d  %s  %.0f Hz  plant=%s  prio=%d  pilot=%s%s\n", getpid(), rx_ipc_backend(),
            cfg.hz, io->name, o.prio, pilot_desc,
            o.learn == 1 ? "  LEARNING from scratch" : o.learn == 2 ? "  LEARNING from sim weights" : "");
    fprintf(stderr, "[guardian] running. Use rxctl / rxdash / cat /dev/reflex to look inside.\n");

    rx_est_t est = { 0 };
    rx_guard_t g; rx_guard_init(&g);
    rx_proposal_t prop, last_prop;
    memset(&prop, 0, sizeof prop); memset(&last_prop, 0, sizeof last_prop);
    int have_prop = 0, sensor_ok = 1, estop_on = 0;
    pid_t pilot = 0;
    int spawn_pending = 0;
    uint32_t seq = 0, last_prop_tick = 0, pilot_started_tick = 0, respawn_at = 0;
    uint32_t offences = 0, offence_age = 0, spawned = 0, ep_seen = 0, ep_iv = 0;
    uint32_t kill_ticks = (uint32_t)(0.4f * cfg.hz), backoff = (uint32_t)(0.2f * cfg.hz);
    uint32_t sup_div = (uint32_t)(cfg.hz / 100.0f); if (sup_div < 1) sup_div = 1;   /* supervise at ~100 Hz */
    uint64_t t_ideal = rx_server_start_ns(srv) + period, last_print = 0;            /* expiry k is due at start + k*period */
    uint64_t sp_phase_ns = 0; int sp_sign = 1;
    const rx_plant_ops_t *P = rx_plant_ops(cfg.plant);
    float sp_amp = cfg.plant == RX_PLANT_JOINT ? 0.6f : 0.08f;         /* automatic square wave: +-80 mm / +-0.6 rad */
    if (sp_amp > cfg.sp_limit) sp_amp = cfg.sp_limit;                    /* a short beam (beam-direct: +-15 mm) */
    uint8_t prev_mode = 255;
    uint64_t udp_refused_seen = atomic_load(&shm->udp_rx_peer);
    uint32_t udp_boots_seen = atomic_load(&shm->udp_boots);
    int guest_ok = 0;                                        /* a command-carrying proposal was taken this tick */
    double guest_down_at = -1;
    rx_event_push(shm, 0, RX_E_GUARDIAN_START, (int32_t)atomic_load(&shm->guardian_starts));

    while (!g_stop) {
        rx_event_e ev = rx_server_wait(srv, &prop);
        if (ev == RX_EV_ERROR) { perror("[guardian] wait"); break; }
        if (ev == RX_EV_TICK && udpl && rx_udp_take(udpl, &prop, NULL)) {
            /* a UDP proposal is picked up at the tick, from a mailbox the rx helper thread fills; it is then
             * handled exactly like one that arrived by message passing between ticks */
            atomic_store(&shm->pilot_present, 1);
            if (prop.flags & RX_PF_HELLO) have_prop = 0;
            else { last_prop = prop; have_prop = 1; last_prop_tick = seq; guest_ok = 1; }
        }
        if (ev == RX_EV_PROPOSAL) {
            atomic_store(&shm->pilot_present, 1);
            if (prop.flags & RX_PF_HELLO) have_prop = 0;
            else { last_prop = prop; have_prop = 1; last_prop_tick = seq; }
            continue;
        }
        if (ev == RX_EV_GONE) { atomic_store(&shm->pilot_present, 0); have_prop = 0; continue; }
        if (ev != RX_EV_TICK) continue;

        /* ------------------------------------------------ control tick */
        uint64_t t0 = rx_now_ns();
        uint32_t lat = t0 > t_ideal ? (uint32_t)(t0 - t_ideal > 4000000000ull ? 4000000000ull : t0 - t_ideal) : 0;
        t_ideal += period;
        if (lat > 8 * period) t_ideal = t0 + period;                 /* we were stopped/suspended: resync */
        seq++;

        float x_true; int lost, fresh;
        float z = io->read(io, &x_true, &lost, &fresh);
        if (fresh && isfinite(z)) rx_est_update(&est, &cfg, z);
        else rx_est_predict(&est, &cfg);                             /* no new sample this tick */
        int ok_now = isfinite(z) && !lost;
        if (sensor_ok && !ok_now) rx_event_push(shm, seq, hw ? RX_E_SENSOR_LOST : RX_E_BALL_LOST, 0);
        sensor_ok = ok_now;

        /* setpoint: automatic square wave, or whatever the operator asked for */
        uint64_t half = (uint64_t)atomic_load(&shm->sp_period_ms) * 1000000ull;
        if (t0 - sp_phase_ns >= half) { sp_phase_ns = t0; sp_sign = -sp_sign; }
        /* manual setpoints are in thousandths of the plant unit (mm on the beam, mrad on the joint) */
        float sp_m = atomic_load(&shm->sp_auto) ? sp_amp * sp_sign : atomic_load(&shm->sp_manual_mm) * 1e-3f;

        /* force_fallback (operator, rxctl bench): treat the pilot as absent. Only ever more conservative. */
        int present = atomic_load(&shm->pilot_present) && !atomic_load(&shm->force_fallback);
        int training = atomic_load(&shm->training);
        rx_guard_in_t in = { (int)atomic_load(&shm->guard_enabled), present, have_prop,
                             seq - last_prop.seq, have_prop ? last_prop.u : NAN };
        g.quarantined = (uint8_t)atomic_load(&shm->quarantined);
        rx_guard_out_t out;
        rx_guard_step(&g, &cfg, est.x, est.v, sp_m, &in, &out);
        if (!ok_now) out.u_out = P->u_eq(&cfg, est.x);               /* nothing measured: hold (beam: level) */
        if (io->estop && io->estop(io)) {                            /* e-stop / actuator trip: beam level, AI stopped */
            out.u_out = P->u_eq(&cfg, est.x);
            if (!estop_on) { estop_on = 1; atomic_store(&shm->force_fallback, 1); }
        }

        io->write(io, out.u_out);

        rx_state_t st = { seq, atomic_load(&shm->fault), atomic_load(&shm->fault_arg), est.x, est.v, sp_m,
                          g.mode, g.reason, (uint32_t)training };
        if (udpl) rx_udp_publish(udpl, &st);                         /* mailbox + semaphore; the tx thread sends it */
        else rx_server_publish(srv, &st);                            /* unblocks the pilot; never blocks us */

        /* ------------------------------------------------ pilot supervision */
        int want_restart = out.want_restart && !training;            /* a learner is allowed to be wrong */
        if (pilot > 0 && (seq % sup_div == 0 || want_restart)) {
            int ws;
            if (waitpid(pilot, &ws, WNOHANG) == pilot) {             /* it died (crash, or we killed it) */
                rx_event_push(shm, seq, RX_E_PILOT_DIED, pilot);
                pilot = 0; have_prop = 0;
                atomic_store(&shm->pilot_pid, 0); atomic_store(&shm->pilot_present, 0);
                rx_server_drop(srv);
                respawn_at = seq + backoff;
            } else {
                int hung = seq - pilot_started_tick > (uint32_t)cfg.hz && seq - last_prop_tick > kill_ticks;
                if (hung || want_restart) {
                    kill(pilot, SIGKILL);
                    rx_event_push(shm, seq, hung ? RX_E_KILL_HUNG : RX_E_KILL_STRIKES, pilot);
                    atomic_fetch_add(&shm->pilot_kills, 1);
                    offences++; offence_age = (uint32_t)(30 * cfg.hz);
                    last_prop_tick = seq;                            /* do not re-kill while it is being reaped */
                    if (offences >= 3) {
                        atomic_store(&shm->quarantined, 1);
                        rx_event_push(shm, seq, RX_E_QUARANTINE, 0);
                        offences = 0;
                    }
                }
            }
        }
        if (udpl) {                                                  /* guest supervision: is the pilot's machine alive */
            double now_s = seq / (double)cfg.hz;
            int why = 0;
            unsigned ga = rx_guest_step(&gst, now_s, guest_ok, out.want_restart && !training, &why);
            guest_ok = 0;
            if (ga & RX_GA_DOWN) {
                rx_event_push(shm, seq, RX_E_GUEST_DOWN, (int32_t)((now_s - gst.last_ok_s) * 1000));
                atomic_store(&shm->pilot_present, 0); have_prop = 0;    /* already stale for a second; now also absent */
                guest_down_at = now_s;
            }
            if (ga & RX_GA_UP) {
                rx_event_push(shm, seq, RX_E_GUEST_UP, guest_down_at >= 0 ? (int32_t)((now_s - guest_down_at) * 1000) : 0);
                guest_down_at = -1;
            }
            if (ga & RX_GA_RESTART) {
                rx_event_push(shm, seq, RX_E_GUEST_RESTART, why);
                atomic_fetch_add(&shm->guest_restarts, 1);
                if (run.cmd && run.cmd[0]) atomic_store(&run.req, 1); /* the helper runs it; we never wait */
            }
            if (ga & RX_GA_QUARANTINE) {
                atomic_store(&shm->quarantined, 1);
                rx_event_push(shm, seq, RX_E_GUEST_QUARANTINE, (int32_t)gst.restarts);
            }
            atomic_store(&shm->guest_state, gst.state);
            if (atomic_exchange(&run.done, 0)) {
                int st = atomic_load(&run.status);
                if (st != 0) rx_event_push(shm, seq, RX_E_GUEST_CMD_FAILED, st);
            }
            uint32_t boots = atomic_load(&shm->udp_boots);
            if (boots != udp_boots_seen) {
                udp_boots_seen = boots;
                rx_event_push(shm, seq, RX_E_GUEST_INSTANCE, (int32_t)(atomic_load(&shm->udp_boot_id) & 0x7fffffff));
            }
            if (seq % sup_div == 0) atomic_store(&shm->guest_age_ms, (uint32_t)((now_s - gst.last_ok_s) * 1000));
        }
        if (udpl && seq % (uint32_t)cfg.hz == 0) {                  /* once a second: datagrams from anyone but the peer */
            uint64_t refused = atomic_load(&shm->udp_rx_peer);
            if (refused != udp_refused_seen) {
                rx_event_push(shm, seq, RX_E_SPOOF, (int32_t)(refused - udp_refused_seen));   /* arg = how many this second */
                udp_refused_seen = refused;
            }
        }
        if (offence_age > 0 && --offence_age == 0) offences = 0;
        if (atomic_exchange(&shm->cmd_release_quarantine, 0)) {
            if (io->release) io->release(io);                       /* e-stop latch (beam-direct) */
            if (estop_on) { estop_on = 0; atomic_store(&shm->force_fallback, 0); }
            if (atomic_exchange(&shm->quarantined, 0)) rx_event_push(shm, seq, RX_E_RELEASE, 0);
            offences = 0;
            if (udpl) rx_guest_release(&gst, seq / (double)cfg.hz);
        }
        if (pilot == 0 && !spawn_pending && spawn_local && !atomic_load(&shm->quarantined) && seq >= respawn_at) {
            atomic_store(&spw.req, 1);                               /* the helper thread spawns; we never wait */
            spawn_pending = 1;
        }
        if (spawn_pending) {
            pid_t np = atomic_exchange(&spw.result, 0);
            if (np > 0) {
                spawn_pending = 0;
                pilot = np;
                atomic_store(&shm->pilot_pid, pilot);
                pilot_started_tick = last_prop_tick = seq;
                if (spawned++) atomic_fetch_add(&shm->pilot_restarts, 1);
                rx_event_push(shm, seq, RX_E_PILOT_SPAWN, pilot);
            } else if (np < 0) { spawn_pending = 0; respawn_at = seq + (uint32_t)cfg.hz; }
        }

        /* ------------------------------------------------ bookkeeping */
        if (out.intervened) {
            atomic_fetch_add(&shm->interventions[g.reason], 1);
            rx_event_push(shm, seq, RX_E_INTERVENE, g.reason);
            ep_iv++;
        }
        if (g.mode != prev_mode) {
            if (g.mode == RX_MODE_AI && prev_mode == RX_MODE_SAFE) rx_event_push(shm, seq, RX_E_AI_RESUMED, 0);
            RX_TRACE("reflex %s/%s x=%dmm", rx_mode_name[g.mode], rx_reason_name[g.reason], (int)(est.x * 1000));
            prev_mode = g.mode;
        }
        atomic_store(&shm->mode, g.mode); atomic_store(&shm->reason, g.reason);

        /* learning curve: the pilot's self-reported cost is display-only; interventions are ours */
        if (have_prop && (last_prop.flags & RX_PF_LEARNER) && last_prop.episode != ep_seen) {
            ep_seen = last_prop.episode;
            uint32_t n = atomic_load(&shm->learn_episodes);
            shm->learn_cost[n & (RX_LEARN_HIST - 1)] = isfinite(last_prop.ep_cost) ? last_prop.ep_cost : 0;
            shm->learn_iv[n & (RX_LEARN_HIST - 1)] = (uint16_t)(ep_iv > 65535 ? 65535 : ep_iv);
            shm->learn_avg = isfinite(last_prop.ep_avg) ? last_prop.ep_avg : 0;
            atomic_store(&shm->learn_episodes, n + 1);
            ep_iv = 0;
        }

        if (atomic_exchange(&shm->cmd_reset_stats, 0)) {
            for (int i = 0; i < RX_R_COUNT; i++) atomic_store(&shm->interventions[i], 0);
            for (int i = 0; i < RX_HIST_BINS; i++) atomic_store(&shm->lat_hist[i], 0);
            atomic_store(&shm->lat_max_ns, 0); atomic_store(&shm->exec_max_ns, 0);
            atomic_store(&shm->deadline_miss, 0); atomic_store(&shm->balls_lost, 0);
            atomic_store(&shm->pilot_restarts, 0); atomic_store(&shm->pilot_kills, 0);
            atomic_store(&shm->ticks, 0);
        }
        uint64_t t1 = rx_now_ns();
        uint32_t exec = (uint32_t)(t1 - t0);
        if (seq > 50) {                                              /* ignore start-up transients */
            atomic_fetch_add(&shm->lat_hist[rx_lat_bin(lat)], 1);
            if (lat > atomic_load(&shm->lat_max_ns)) atomic_store(&shm->lat_max_ns, lat);
            if (exec > atomic_load(&shm->exec_max_ns)) atomic_store(&shm->exec_max_ns, exec);
            if (lat > period / 2) atomic_fetch_add(&shm->deadline_miss, 1);
        }
        atomic_fetch_add(&shm->ticks, 1);
        rx_rec_t r = { t0, seq, lat, exec, est.x, est.v, sp_m, x_true, in.u_ai, out.u_out, out.margin,
                       g.mode, g.reason, (uint8_t)present, (uint8_t)!ok_now };
        rx_shm_push(shm, &r);

        if (o.verbose && t1 - last_print > 1000000000ull) {
            last_print = t1;
            fprintf(stderr, "[guardian] %-6s %-10s x=%+6.1fmm sp=%+4.0fmm u=%+.3f  lat max %.0fus  miss %llu  lost %u\n",
                    rx_mode_name[g.mode], rx_reason_name[g.reason], est.x * 1e3, sp_m * 1e3, out.u_out,
                    atomic_load(&shm->lat_max_ns) / 1e3, (unsigned long long)atomic_load(&shm->deadline_miss),
                    atomic_load(&shm->balls_lost));
        }
    }

    if (spawn_local) {
        atomic_store(&spw.stop, 1);
        pthread_join(spw_thread, NULL);
        pid_t np = atomic_exchange(&spw.result, 0);                  /* spawned while we were stopping */
        if (np > 0 && pilot <= 0) pilot = np;
    }
    if (pilot > 0) { kill(pilot, SIGTERM); usleep(50000); kill(pilot, SIGKILL); waitpid(pilot, NULL, 0); }
    atomic_store(&shm->pilot_pid, 0); atomic_store(&shm->pilot_present, 0);
    if (udpl && run.cmd && run.cmd[0]) {                     /* a restart command still running is left to finish on its own */
        atomic_store(&run.stop, 1);
        if (!atomic_load(&shm->guest_cmd_running)) pthread_join(run_thread, NULL);
    }
    rx_udp_close(udpl);
    io->write(io, 0.0f);
    io->close(io);
    fprintf(stderr, "[guardian] stopped\n");
    return 0;
}
