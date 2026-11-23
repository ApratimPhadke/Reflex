/* rxctl - operator console, fault injector and evidence generator for Reflex.
 * SPDX-License-Identifier: MIT */
#include <errno.h>
#include <libgen.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/wait.h>
#include "rx_shm.h"
#include "rx_conf.h"
#include "rx_guest.h"

#ifndef RX_GIT
#define RX_GIT "unknown"
#endif

static rx_shm_t *shm;
static const char *argv0 = "rxctl";

static void msleep(unsigned ms) { struct timespec t = { ms / 1000, (long)(ms % 1000) * 1000000L }; nanosleep(&t, NULL); }

static void usage(void)
{
    puts("rxctl <command>\n"
         "  status                 one-shot status (same text as `cat /dev/reflex` on QNX)\n"
         "  watch                  live status, refreshed 5x per second\n"
         "  inject <fault> [ms]    crash | freeze | nan | huge | flip | stuck | slow [ms] | spin\n"
         "  clear                  remove the injected fault\n"
         "  guard on|off           off = let pilot commands straight through (to show what Reflex prevents)\n"
         "  sp <mm>|auto           manual setpoint in mm, or the automatic square wave\n"
         "  train on|off           learning pilot: explore / fly the best weights found\n"
         "  period <ms>            half-period of the automatic setpoint square wave\n"
         "  release                let a quarantined pilot back in\n"
         "  events                 the guardian's event log\n"
         "  incidents <dir>        flight recorder: write a CSV around every intervention\n"
         "  endpoint               print '<guardian-pid> <channel> <hz>' (for the spoofing test)\n"
         "  reset                  zero all counters and histograms\n"
         "  latency                wake-up latency histogram of the guardian\n"
         "  hog <prio> <secs>      burn every CPU at the given priority (load test)\n"
         "  log <file.csv> <secs>  record telemetry\n"
         "  campaign [file.md]     run the full fault campaign, with and without the guardian\n"
         "  caption [text]         show a banner on the dashboard (no text = clear it)\n"
         "  wipe                   (no guardian running) delete the shared memory: fresh world and counters next start\n"
         "  fallback on|off        fly on the fallback controller alone (the pilot keeps running, ignored)\n"
         "  stuck [N] [f.md]       inject stuck-at-full-tilt N times (default 5), report closest approach vs the strip, PASS/FAIL\n"
         "  bench [f.md] [--csv f] [--hold s]  same setpoint sequence with the AI, then fallback-only; compare\n"
         "  script <file> [--check] play a timed demo script (demo/video.rxs); --check only validates it\n"
         "  guest <what> [secs]    pilot in a guest VM (pilot_transport = udp), over ssh (guest_ssh in reflex.conf):\n"
         "                           panic     kernel panic in the guest (echo c > /proc/sysrq-trigger)\n"
         "                           hog [s]   every guest CPU spinning at real-time priority (default 20 s)\n"
         "                           badmodel  the guest's pilot sign-inverts its output (SIGUSR1); the guardian is not told\n"
         "                           heal      undo badmodel / hog / freeze\n"
         "                           kill | freeze   SIGKILL / SIGSTOP the guest's pilot\n"
         "                         reflex.conf: $RX_CONF, else next to this binary\n"
         "  campaign [file.md] [--guest]   --guest adds the guest stage (panic, hog, bad model, pilot killed)");
}

/* ------------------------------------------------------------------ latency */
static void cmd_latency(FILE *f)
{
    uint64_t h[RX_HIST_BINS], tot = 0, mx = 0;
    for (int i = 0; i < RX_HIST_BINS; i++) { h[i] = atomic_load(&shm->lat_hist[i]); tot += h[i]; if (h[i] > mx) mx = h[i]; }
    fprintf(f, "guardian wake-up latency, %llu ticks at %.0f Hz (period %.0f us)\n",
            (unsigned long long)tot, shm->hz, 1e6 / shm->hz);
    uint64_t acc = 0; double p50 = -1, p99 = -1, p9999 = -1;
    for (int i = 0; i < RX_HIST_BINS; i++) {
        acc += h[i];
        double up = (double)(1u << (i + 1));
        if (p50 < 0 && acc >= tot * 0.50) p50 = up;
        if (p99 < 0 && acc >= tot * 0.99) p99 = up;
        if (p9999 < 0 && acc >= tot * 0.9999) p9999 = up;
        if (!h[i]) continue;
        int bar = mx ? (int)(40.0 * log1p((double)h[i]) / log1p((double)mx)) : 0;
        fprintf(f, "  %8u - %-8u us |%-40.*s| %llu\n", i ? 1u << i : 0, 1u << (i + 1), bar,
                "########################################", (unsigned long long)h[i]);
    }
    fprintf(f, "  p50 < %.0f us   p99 < %.0f us   p99.99 < %.0f us   max = %.1f us   late ticks (> period/2) = %llu\n",
            p50, p99, p9999, atomic_load(&shm->lat_max_ns) / 1e3, (unsigned long long)atomic_load(&shm->deadline_miss));
}

/* ------------------------------------------------------------------ hog */
static volatile int hog_stop;
static void *hog_thread(void *arg)
{
    int prio = *(int *)arg;
    struct sched_param sp = { .sched_priority = prio };
    if (prio > 0 && pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
        fprintf(stderr, "hog: could not set SCHED_FIFO %d (running at default priority)\n", prio);
    volatile double x = 0;
    while (!hog_stop) x += 1.0;
    return NULL;
}
static void cmd_hog(int prio, int secs)
{
    long n = sysconf(_SC_NPROCESSORS_ONLN);
    if (n < 1) n = 1;
    if (n > 64) n = 64;
    pthread_t t[64];
    printf("hog: %ld threads spinning at priority %d for %d s\n", n, prio, secs);
    for (long i = 0; i < n; i++) pthread_create(&t[i], NULL, hog_thread, &prio);
    sleep((unsigned)secs);
    hog_stop = 1;
    for (long i = 0; i < n; i++) pthread_join(t[i], NULL);
}

/* ------------------------------------------------------------------ log */
static void cmd_log(const char *path, int secs)
{
    FILE *f = fopen(path, "w");
    if (!f) { perror(path); exit(1); }
    fprintf(f, "t_s,seq,x_mm,v_mps,sp_mm,x_true_mm,u_ai_rad,u_out_rad,margin_mm,mode,reason,pilot,lost,lat_us,exec_us\n");
    uint64_t idx = atomic_load(&shm->head), t0 = 0, end = rx_now_ns() + (uint64_t)secs * 1000000000ull, n = 0;
    while (rx_now_ns() < end) {
        rx_rec_t r;
        uint64_t h = atomic_load(&shm->head);
        if (h - idx > RX_RING / 2) idx = h - RX_RING / 2;
        for (; idx < h; idx++) {
            if (!rx_shm_get(shm, idx, &r)) continue;
            if (!t0) t0 = r.t_ns;
            fprintf(f, "%.6f,%u,%.2f,%.4f,%.1f,%.2f,%.5f,%.5f,%.2f,%s,%s,%u,%u,%.1f,%.1f\n", (r.t_ns - t0) * 1e-9, r.seq,
                    r.x * 1e3, r.v, r.sp * 1e3, r.x_true * 1e3, r.u_ai, r.u_out, r.margin * 1e3,
                    rx_mode_name[r.mode % 3], rx_reason_name[r.reason % RX_R_COUNT], r.pilot_present, r.lost,
                    r.lat_ns / 1e3, r.exec_ns / 1e3);
            n++;
        }
        msleep(20);
    }
    fclose(f);
    printf("wrote %llu records to %s\n", (unsigned long long)n, path);
}

/* ------------------------------------------------------------------ events + flight recorder */
static void print_event(FILE *f, const rx_event_t *e, uint64_t now)
{
    const char *n = e->code < RX_E_COUNT ? rx_event_name[e->code] : "?";
    fprintf(f, "%8.2fs ago  tick %-9u %s", (now - e->t_ns) * 1e-9, e->seq, n);
    if (e->code == RX_E_INTERVENE) fprintf(f, " (%s)", rx_reason_name[e->arg % RX_R_COUNT]);
    else if (e->arg) fprintf(f, " [%d]", e->arg);
    fputc('\n', f);
}
static void cmd_events(void)
{
    uint64_t h = atomic_load(&shm->ev_head), now = rx_now_ns();
    rx_event_t e;
    for (uint64_t i = h > 40 ? h - 40 : 0; i < h; i++) if (rx_event_get(shm, i, &e)) print_event(stdout, &e, now);
}
static void dump_window(const char *path, uint64_t from, uint64_t to)
{
    FILE *f = fopen(path, "w");
    if (!f) { perror(path); return; }
    fprintf(f, "seq,t_ms,x_mm,v_mps,sp_mm,u_ai_rad,u_out_rad,margin_mm,mode,reason,pilot,lat_us\n");
    rx_rec_t r; uint64_t t0 = 0;
    for (uint64_t i = from; i < to; i++) {
        if (!rx_shm_get(shm, i, &r)) continue;
        if (!t0) t0 = r.t_ns;
        fprintf(f, "%u,%.3f,%.2f,%.4f,%.1f,%.5f,%.5f,%.2f,%s,%s,%u,%.1f\n", r.seq, (r.t_ns - t0) * 1e-6, r.x * 1e3, r.v, r.sp * 1e3,
                r.u_ai, r.u_out, r.margin * 1e3, rx_mode_name[r.mode % 3], rx_reason_name[r.reason % RX_R_COUNT], r.pilot_present, r.lat_ns / 1e3);
    }
    fclose(f);
}
static void cmd_incidents(const char *dir)
{
    /* Low-priority observer. For every intervention it saves 1 s before and 1 s after, so there
     * is evidence of exactly what the pilot asked for and what the guardian did about it. */
    uint64_t ei = atomic_load(&shm->ev_head); unsigned n = 0;
    printf("recording incidents to %s/ (Ctrl-C to stop)\n", dir);
    for (;;) {
        rx_event_t e;
        uint64_t h = atomic_load(&shm->ev_head);
        for (; ei < h; ei++) {
            if (!rx_event_get(shm, ei, &e) || e.code != RX_E_INTERVENE) continue;
            uint64_t at = atomic_load(&shm->head), span = (uint64_t)shm->hz;
            msleep(1000);
            char path[512];
            snprintf(path, sizeof path, "%s/incident-%04u-%s-tick%u.csv", dir, ++n, rx_reason_name[e.arg % RX_R_COUNT], e.seq);
            dump_window(path, at > span ? at - span : 0, atomic_load(&shm->head));
            printf("  %s\n", path); fflush(stdout);
        }
        msleep(50);
    }
}

/* ------------------------------------------------------------------ guest (pilot over UDP in a VM)
 * The commands are fixed strings built here; only the ssh target comes from reflex.conf (checked by
 * rx_conf_check_transport: user@host characters only), and it is passed to ssh as one argv element,
 * never through a local shell. Nothing is taken from the shared memory, which any local user can write. */
static int guest_target(char *ssh, size_t cap)
{
    char path[512], tmp[512], err[256];
    const char *env = getenv("RX_CONF");
    if (env && *env) snprintf(path, sizeof path, "%s", env);
    else { snprintf(tmp, sizeof tmp, "%s", argv0); snprintf(path, sizeof path, "%s/reflex.conf", dirname(tmp)); }
    rx_conf_t c;
    if (rx_conf_load_any(&c, path, err, sizeof err) != 0) { fprintf(stderr, "rxctl guest: %s\n", err); return -1; }
    if (!c.guest_ssh[0]) { fprintf(stderr, "rxctl guest: no guest_ssh in %s (user@host of the guest, or 'local')\n", path); return -1; }
    snprintf(ssh, cap, "%s", c.guest_ssh);
    return 0;
}

static int guest_cmd(const char *what, int secs, int quiet)
{
    char ssh[64];
    if (guest_target(ssh, sizeof ssh) != 0) return -1;
    const int local = !strcmp(ssh, "local");
    char rc[600];
    if (!strcmp(what, "panic")) {
        if (local) { fprintf(stderr, "rxctl guest panic: refused with guest_ssh = local (it would crash THIS machine)\n"); return -1; }
        snprintf(rc, sizeof rc, "nohup sh -c 'sleep 0.3; echo c > /proc/sysrq-trigger' >/dev/null 2>&1 </dev/null &");
    } else if (!strcmp(what, "hog")) {
        if (secs <= 0 || secs > 600) secs = 20;
        /* in a guest: real-time priority, so the guest's own scheduler starves its pilot; on a local test machine: niced */
        snprintf(rc, sizeof rc, "%s n=$(grep -c ^processor /proc/cpuinfo); [ \"$n\" -ge 1 ] || n=1; i=0; "
                 "while [ $i -lt $n ]; do nohup timeout %d $P sh -c ': rxhog; while :; do :; done' >/dev/null 2>&1 </dev/null & i=$((i+1)); done",
                 local ? "P='nice -n 19';" : "P=; command -v chrt >/dev/null 2>&1 && P='chrt -f 50';", secs);
    } else if (!strcmp(what, "badmodel")) snprintf(rc, sizeof rc, "pkill -USR1 -x reflex-pilot");
    else if (!strcmp(what, "heal"))  snprintf(rc, sizeof rc, "pkill -USR2 -x reflex-pilot; pkill -f 'rxho[g]'; pkill -CONT -x reflex-pilot; true");
    else if (!strcmp(what, "kill"))  snprintf(rc, sizeof rc, "pkill -KILL -x reflex-pilot");
    else if (!strcmp(what, "freeze")) snprintf(rc, sizeof rc, "pkill -STOP -x reflex-pilot");
    else { fprintf(stderr, "rxctl guest: unknown '%s' (panic | hog [s] | badmodel | heal | kill | freeze)\n", what); return -1; }

    pid_t pid = fork();
    if (pid < 0) { perror("rxctl guest: fork"); return -1; }
    if (pid == 0) {
        if (quiet) { FILE *n = freopen("/dev/null", "w", stdout); (void)n; }
        if (local) execl("/bin/sh", "sh", "-c", rc, (char *)NULL);
        else execlp("ssh", "ssh", "-n", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3", "-o", "ServerAliveInterval=1",
                    "-o", "ServerAliveCountMax=2", "-o", "StrictHostKeyChecking=accept-new", "--", ssh, rc, (char *)NULL);
        _exit(127);
    }
    int ws = 0;
    for (int i = 0; i < 300; i++) {                           /* 15 s */
        if (waitpid(pid, &ws, WNOHANG) == pid) {
            int code = WIFEXITED(ws) ? WEXITSTATUS(ws) : -1;
            /* pkill exits 1 when nothing matched; a panic can cut the ssh session (255): neither is a failure here */
            if (code == 0 || (code == 1 && strcmp(what, "panic")) || (code == 255 && !strcmp(what, "panic"))) return 0;
            fprintf(stderr, "rxctl guest %s: %s exited with %d\n", what, local ? "sh" : "ssh", code);
            return -1;
        }
        msleep(50);
    }
    kill(pid, SIGKILL); waitpid(pid, NULL, 0);
    if (!strcmp(what, "panic")) return 0;                     /* the guest died under the session: that was the point */
    fprintf(stderr, "rxctl guest %s: no answer from %s in 15 s\n", what, ssh);
    return -1;
}

static int is_udp(void) { return !strcmp(shm->pilot_transport, "udp"); }

/* inject a campaign fault: over UDP the pilot is not our child, so crash/freeze happen inside the guest */
static int inject(const char *name, unsigned arg)
{
    if (is_udp() && !strcmp(name, "crash"))  return guest_cmd("kill", 0, 1);
    if (is_udp() && !strcmp(name, "freeze")) return guest_cmd("freeze", 0, 1);
    return rx_fault_apply(shm, name, arg);
}
static void clear_faults(void)
{
    rx_command(shm, "fault clear");
    if (is_udp()) guest_cmd("heal", 0, 1);
}

/* ------------------------------------------------------------------ campaign */
static int wait_ai(int ms)
{
    int stable = 0;
    for (int t = 0; t < ms; t += 50) {
        msleep(50);
        stable = (atomic_load(&shm->mode) == RX_MODE_AI) ? stable + 50 : 0;
        if (stable >= 1500) return 1;
    }
    return 0;
}

typedef struct { const char *name, *what; unsigned arg; } fault_t;
static const fault_t FAULTS[] = {
    { "crash",  "pilot process killed (SIGKILL)", 0 },
    { "freeze", "pilot frozen (SIGSTOP), no CPU use", 0 },
    { "spin",   "pilot stuck in an infinite loop, 100% CPU", 0 },
    { "slow",   "pilot answers 50 ms late", 50 },
    { "nan",    "pilot outputs NaN", 0 },
    { "huge",   "pilot outputs 40 rad (limit is 0.21)", 0 },
    { "stuck",  "pilot output stuck at +max", 0 },
    { "flip",   "pilot output sign-inverted (drives the ball off)", 0 },
};
#define NFAULTS (sizeof FAULTS / sizeof FAULTS[0])

static void campaign(FILE *out)
{
    const int joint = !strcmp(shm->plant, "joint");
    const int run_ms = 4000;
    fprintf(out, "# Reflex fault campaign\n\n");
    fprintf(out, "- build `%s`; pilot transport `%s`%s%s\n", RX_GIT, is_udp() ? "udp" : "local", is_udp() ? " from " : "", is_udp() ? shm->pilot_peer : "");
    fprintf(out, "- backend: `%s`, control rate %.0f Hz\n- each fault is injected into a healthy system and left active for %d s\n"
                 "- *without Reflex* = the same pilot, same faults, guardian in bypass (pilot commands go straight to the actuator)\n\n",
            shm->backend, shm->hz, run_ms / 1000);
    if (strcmp(shm->backend, "sim"))
        fprintf(out, "- *closest to the end* is the guardian's **estimate** (no ground truth on this backend; it includes sensor noise)\n\n");
    if (joint) fprintf(out, "- plant: **robot joint** (simulated); \"limit hit\" = the joint reached its hard stop\n\n");
    fprintf(out, "| # | Injected pilot fault | Guardian took over after | %s | %s (with Reflex) | %s | %s (without Reflex) |\n",
            joint ? "Closest to the stop" : "Closest to beam end", joint ? "Joint" : "Ball", is_udp() ? "Pilot/guest restarts" : "Pilot restarts",
            joint ? "Joint" : "Ball");
    fprintf(out, "|---|---|---|---|---|---|---|\n");

    int kept = 0, lost_without = 0, skipped = 0;
    atomic_store(&shm->sp_auto, 1);
    rx_command(shm, "train off"); rx_command(shm, "period 4000");   /* a campaign is run against a frozen pilot */
    for (unsigned fi = 0; fi < NFAULTS; fi++) {
        const fault_t *F = &FAULTS[fi];
        double detect_ms = -1, min_edge = 1e9; unsigned lost_g = 0, lost_b = 0, restarts = 0;
        int not_injected = 0;
        for (int pass = 0; pass < 2; pass++) {                  /* 0 = guarded, 1 = bypass */
            clear_faults(); rx_command(shm, "guard on"); rx_command(shm, "release");
            if (!wait_ai(is_udp() ? 60000 : 10000)) fprintf(stderr, "campaign: warning, system did not return to AI mode before '%s'\n", F->name);
            /* always start a run at the same phase of the setpoint square wave: 1 s after it goes positive */
            { rx_rec_t r; float prev = 1; for (int t = 0; t < 12000; t += 5) { uint64_t h = atomic_load(&shm->head);
                if (h && rx_shm_get(shm, h - 1, &r)) { if (prev < 0 && r.sp > 0) break; prev = r.sp; } msleep(5); } msleep(1000); }
            if (pass == 1) rx_command(shm, "guard off");
            unsigned b0 = atomic_load(&shm->balls_lost);
            unsigned r0 = atomic_load(&shm->pilot_restarts) + atomic_load(&shm->pilot_kills) + atomic_load(&shm->guest_restarts);
            uint64_t idx = atomic_load(&shm->head);
            uint64_t t_inj = rx_now_ns();
            if (inject(F->name, F->arg) != 0) {                     /* e.g. no permission to signal the pilot */
                fprintf(stderr, "campaign: could not inject '%s' (%s): row marked NOT INJECTED\n", F->name, strerror(errno));
                not_injected = 1;
            }
            uint64_t end = t_inj + (uint64_t)run_ms * 1000000ull;
            while (rx_now_ns() < end) {
                uint64_t h = atomic_load(&shm->head);
                rx_rec_t r;
                for (; idx < h; idx++) {
                    if (!rx_shm_get(shm, idx, &r) || r.t_ns < t_inj) continue;
                    if (pass == 0) {
                        if (detect_ms < 0 && r.mode == RX_MODE_SAFE) detect_ms = (r.t_ns - t_inj) * 1e-6;
                        float xx = isfinite(r.x_true) ? r.x_true : r.x;
                        double edge = shm->half_len - fabs(xx);
                        if (edge < min_edge) min_edge = edge;
                    }
                }
                msleep(10);
            }
            unsigned lost = atomic_load(&shm->balls_lost) - b0;
            if (pass == 0) { lost_g = lost; restarts = atomic_load(&shm->pilot_restarts) + atomic_load(&shm->pilot_kills) + atomic_load(&shm->guest_restarts) - r0; }
            else lost_b = lost;
            clear_faults(); rx_command(shm, "guard on");
            msleep(1500);                                       /* let a dropped ball be replaced */
        }
        char det[32] = "not needed";
        if (detect_ms >= 0) snprintf(det, sizeof det, "%.1f ms", detect_ms);
        if (not_injected) {
            fprintf(out, "| %u | %s | **NOT INJECTED** (could not signal the pilot) | - | - | - | - |\n", fi + 1, F->what);
            fflush(out);
            fprintf(stderr, "campaign: %-7s NOT INJECTED\n", F->name);
            skipped++;
            continue;
        }
        const char *what = !joint ? F->what : !strcmp(F->name, "huge") ? "pilot outputs 40 N m (limit is 4)"
                         : !strcmp(F->name, "stuck") ? "pilot output stuck at full torque"
                         : !strcmp(F->name, "flip") ? "pilot output sign-inverted (drives the joint into a stop)" : F->what;
        if (joint) fprintf(out, "| %u | %s | %s | %.1f deg | %s | %u | %s |\n", fi + 1, what, det, min_edge * 57.29578,
                           lost_g ? "**LIMIT HIT**" : "kept", restarts, lost_b ? "**LIMIT HIT**" : "kept");
        else fprintf(out, "| %u | %s | %s | %.1f cm | %s | %u | %s |\n", fi + 1, F->what, det, min_edge * 100,
                lost_g ? "**LOST**" : "kept", restarts, lost_b ? "**LOST**" : "kept");
        fflush(out);
        if (!lost_g) kept++;
        if (lost_b) lost_without++;
        fprintf(stderr, "campaign: %-7s guarded=%s bypass=%s\n", F->name, lost_g ? "LOST" : "kept", lost_b ? "LOST" : "kept");
    }
    rx_command(shm, "release");
    unsigned ran = (unsigned)NFAULTS - (unsigned)skipped;
    fprintf(out, "\n**With Reflex: %s in %d of %u fault scenarios. Without Reflex: %s in %d of %u.**\n\n",
            joint ? "joint kept off its stops" : "ball kept", kept, ran, joint ? "limit hit" : "ball lost", lost_without, ran);
    if (skipped) fprintf(out, "**%d fault(s) could not be injected and are not counted above.**\n\n", skipped);
    fprintf(out, "```\n"); cmd_latency(out); fprintf(out, "```\n");
    fprintf(out, "\nGuardian worst-case execution time per tick: %.1f us. Spoofed proposals refused: %llu.\n",
            atomic_load(&shm->exec_max_ns) / 1e3,
            (unsigned long long)atomic_load(&shm->rejected_senders) + (unsigned long long)atomic_load(&shm->udp_rx_peer));
}

/* ------------------------------------------------------------------ campaign, guest stage
 * Faults that start inside the pilot's own machine, the guest VM (pilot_transport = udp). Same method as the
 * main table: inject into a healthy system, once with the guardian and once in bypass. After each run the guest
 * is healed and the next run waits until the AI flies again; a panicked guest has to reboot first (by itself
 * with panic=N on its kernel command line, or by the guardian's guest_restart_cmd). */
typedef struct { const char *what, *desc; int secs, watch_ms; } gfault_t;
static const gfault_t GFAULTS[] = {
    { "panic",    "Linux kernel panic in the guest (`echo c > /proc/sysrq-trigger`)", 0, 8000 },
    { "hog",      "every guest CPU spinning at real-time priority for 10 s", 10, 12000 },
    { "badmodel", "the guest's pilot sign-inverts its output (signal inside the guest; the guardian is not told)", 0, 6000 },
    { "kill",     "the guest's pilot killed (SIGKILL inside the guest)", 0, 6000 },
};

static void campaign_guest(FILE *out)
{
    const int joint = !strcmp(shm->plant, "joint");
    char ssh[64] = "";
    if (guest_target(ssh, sizeof ssh) != 0) {
        fprintf(out, "\n## Guest stage\n\n**Not run:** no `guest_ssh` in reflex.conf, so faults cannot be started inside the guest.\n");
        return;
    }
    fprintf(out, "\n## Guest stage: faults inside the AI's own operating system\n\n");
    fprintf(out, "- pilot in a guest reached at `%s`, datagrams from `%s`; guest_timeout and the restart limiter as configured in reflex.conf\n",
            ssh, shm->pilot_peer);
    fprintf(out, "- *AI back after* = from the injection until the AI had flown for 1.5 s again (includes a guest reboot after a panic)\n\n");
    fprintf(out, "| # | Fault inside the guest | Guardian took over after | Closest to the end | %s (with Reflex) | Guest restarts by the guardian | New pilot instances | AI back after | %s (without Reflex) |\n",
            joint ? "Joint" : "Ball", joint ? "Joint" : "Ball");
    fprintf(out, "|---|---|---|---|---|---|---|---|---|\n");
    int kept = 0, lost_without = 0, ran = 0;
    for (unsigned fi = 0; fi < sizeof GFAULTS / sizeof GFAULTS[0]; fi++) {
        const gfault_t *F = &GFAULTS[fi];
        if (!strcmp(F->what, "panic") && !strcmp(ssh, "local")) {
            fprintf(out, "| G%u | %s | **NOT INJECTED** (guest_ssh = local) | - | - | - | - | - | - |\n", fi + 1, F->desc);
            continue;
        }
        double detect_ms = -1, min_edge = 1e9, back_s = -1; unsigned lost_g = 0, lost_b = 0, rest = 0, inst = 0; int failed = 0;
        for (int pass = 0; pass < 2 && !failed; pass++) {
            clear_faults(); rx_command(shm, "guard on"); rx_command(shm, "release");
            if (!wait_ai(240000)) fprintf(stderr, "campaign: warning, the AI did not fly before guest fault '%s'\n", F->what);
            { rx_rec_t r; float prev = 1; for (int t = 0; t < 12000; t += 5) { uint64_t h = atomic_load(&shm->head);
                if (h && rx_shm_get(shm, h - 1, &r)) { if (prev < 0 && r.sp > 0) break; prev = r.sp; } msleep(5); } msleep(1000); }
            if (pass == 1) rx_command(shm, "guard off");
            unsigned b0 = atomic_load(&shm->balls_lost), r0 = atomic_load(&shm->guest_restarts), i0 = atomic_load(&shm->udp_boots);
            uint64_t idx = atomic_load(&shm->head), t_inj = rx_now_ns(), end = t_inj + (uint64_t)F->watch_ms * 1000000ull;
            if (guest_cmd(F->what, F->secs, 1) != 0) { failed = 1; break; }
            while (rx_now_ns() < end) {
                uint64_t h = atomic_load(&shm->head); rx_rec_t r;
                for (; idx < h; idx++) {
                    if (!rx_shm_get(shm, idx, &r) || r.t_ns < t_inj || pass) continue;
                    if (detect_ms < 0 && r.mode == RX_MODE_SAFE) detect_ms = (r.t_ns - t_inj) * 1e-6;
                    float xx = isfinite(r.x_true) ? r.x_true : r.x;
                    if (isfinite(xx) && shm->half_len - fabs(xx) < min_edge) min_edge = shm->half_len - fabs(xx);
                }
                msleep(10);
            }
            unsigned lost = atomic_load(&shm->balls_lost) - b0;
            rx_command(shm, "guard on");
            clear_faults();
            if (pass == 0) {
                lost_g = lost;
                if (wait_ai(240000)) back_s = (rx_now_ns() - t_inj) * 1e-9 - 1.5;
                rest = atomic_load(&shm->guest_restarts) - r0; inst = atomic_load(&shm->udp_boots) - i0;
            } else lost_b = lost;
            msleep(1500);
        }
        const char *desc = !strcmp(F->what, "hog") && !strcmp(ssh, "local") ? "every CPU spinning (local test: niced, not real-time) for 10 s" : F->desc;
        if (failed) {
            fprintf(out, "| G%u | %s | **NOT INJECTED** (could not reach the guest) | - | - | - | - | - | - |\n", fi + 1, desc);
            fprintf(stderr, "campaign: guest %-8s NOT INJECTED\n", F->what);
            continue;
        }
        char det[32] = "not needed", back[32] = "**not within 240 s**";
        if (detect_ms >= 0) snprintf(det, sizeof det, "%.1f ms", detect_ms);
        if (back_s >= 0) snprintf(back, sizeof back, "%.1f s", back_s);
        if (joint) fprintf(out, "| G%u | %s | %s | %.1f deg | %s | %u | %u | %s | %s |\n", fi + 1, desc, det, min_edge * 57.29578,
                           lost_g ? "**LIMIT HIT**" : "kept", rest, inst, back, lost_b ? "**LIMIT HIT**" : "kept");
        else fprintf(out, "| G%u | %s | %s | %.1f cm | %s | %u | %u | %s | %s |\n", fi + 1, desc, det, min_edge * 100,
                     lost_g ? "**LOST**" : "kept", rest, inst, back, lost_b ? "**LOST**" : "kept");
        fflush(out);
        ran++; if (!lost_g) kept++; if (lost_b) lost_without++;
        fprintf(stderr, "campaign: guest %-8s guarded=%s bypass=%s\n", F->what, lost_g ? "LOST" : "kept", lost_b ? "LOST" : "kept");
    }
    rx_command(shm, "release");
    fprintf(out, "\n**Guest stage, with Reflex: %s in %d of %d. Without Reflex: %s in %d of %d.**\n",
            joint ? "joint kept off its stops" : "ball kept", kept, ran, joint ? "limit hit" : "ball lost", lost_without, ran);
}

/* ------------------------------------------------------------------ script
 * One step per line; '#' starts a comment line. Steps:
 *   wait <seconds>            pause (decimals allowed)
 *   wait-ai [seconds]         wait until the AI has been in control for 1.5 s (default timeout 15 s)
 *   caption [text]            dashboard banner (empty = clear)
 *   say <text>                cue for the operator, printed in the terminal only
 *   inject <fault> [ms]       as rxctl inject
 *   clear | release | reset | guard on|off | fallback on|off | sp <mm>|auto | train on|off | period <ms>
 * The whole file is checked before the first step runs. */
static int script_step(const char *ln, int run, int lineno, double t0)
{
    char w[32] = "", a1[64] = "", a2[32] = "";
    int k = sscanf(ln, "%31s %63s %31s", w, a1, a2);
    if (k < 1) return 0;
    const char *rest = ln;
    while (*rest == ' ' || *rest == '\t') rest++;
    rest += strlen(w);
    while (*rest == ' ' || *rest == '\t') rest++;
    double now = 0;
    if (run) { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); now = ts.tv_sec + ts.tv_nsec * 1e-9 - t0; }
    if (!strcmp(w, "wait") || !strcmp(w, "wait-ai")) {
        char *e; double sec = k >= 2 ? strtod(a1, &e) : (!strcmp(w, "wait-ai") ? 15.0 : -1);
        if (k >= 2 && (*e || e == a1)) sec = -1;
        if (!(sec >= 0 && sec <= 600)) { fprintf(stderr, "line %d: '%s' needs seconds (0..600)\n", lineno, w); return -1; }
        if (!run) return 0;
        printf("[%6.1f s] %s %.1f\n", now, w, sec); fflush(stdout);
        if (!strcmp(w, "wait")) msleep((unsigned)(sec * 1000));
        else if (!wait_ai((int)(sec * 1000))) printf("           (AI not back in control after %.0f s, continuing)\n", sec);
        return 0;
    }
    if (!strcmp(w, "say")) { if (run) { printf("[%6.1f s] >>> %s\n", now, rest); fflush(stdout); } return 0; }
    if (!strcmp(w, "caption")) { if (run) { char c[160]; snprintf(c, sizeof c, "caption %s", rest); rx_command(shm, c); printf("[%6.1f s] caption: %s\n", now, rest); fflush(stdout); } return 0; }
    if (!strcmp(w, "inject")) {
        static const char *const known[] = { "crash", "freeze", "nan", "huge", "flip", "stuck", "slow", "spin" };
        int ok = 0; for (unsigned i = 0; i < sizeof known / sizeof known[0]; i++) ok |= k >= 2 && !strcmp(a1, known[i]);
        if (!ok) { fprintf(stderr, "line %d: unknown fault '%s'\n", lineno, a1); return -1; }
        if (!run) return 0;
        printf("[%6.1f s] inject %s\n", now, a1); fflush(stdout);
        if (inject(a1, k >= 3 ? (uint32_t)atoi(a2) : 0) != 0) printf("           (no pilot to inject into)\n");
        return 0;
    }
    int known = (!strcmp(w, "clear") || !strcmp(w, "release") || !strcmp(w, "reset")) && k == 1;
    known |= !strcmp(w, "guard") && k == 2 && (!strcmp(a1, "on") || !strcmp(a1, "off"));
    known |= !strcmp(w, "train") && k == 2 && (!strcmp(a1, "on") || !strcmp(a1, "off"));
    known |= !strcmp(w, "fallback") && k == 2 && (!strcmp(a1, "on") || !strcmp(a1, "off"));
    known |= (!strcmp(w, "sp") || !strcmp(w, "period")) && k == 2;
    if (!known) { fprintf(stderr, "line %d: unknown step '%s'\n", lineno, ln); return -1; }
    if (!run) return 0;
    char c[128];
    if (!strcmp(w, "clear")) snprintf(c, sizeof c, "fault clear"); else snprintf(c, sizeof c, "%s", ln);
    printf("[%6.1f s] %s\n", now, ln); fflush(stdout);
    if (rx_command(shm, c) != 0) printf("           (rejected by the guardian)\n");
    return 0;
}

static int cmd_script(const char *path, int check_only)
{
    FILE *f = fopen(path, "r");
    if (!f) { perror(path); return 1; }
    char ln[256]; int lineno = 0, bad = 0, steps = 0;
    while (fgets(ln, sizeof ln, f)) {                         /* pass 1: check everything */
        lineno++; ln[strcspn(ln, "\r\n")] = 0;
        char *p = ln; while (*p == ' ' || *p == '\t') p++;
        if (!*p || *p == '#') continue;
        if (script_step(p, 0, lineno, 0) != 0) bad++; else steps++;
    }
    if (bad) { fclose(f); fprintf(stderr, "rxctl: %s has %d bad line(s); nothing was run\n", path, bad); return 2; }
    printf("%s: %d steps OK\n", path, steps);
    if (check_only) { fclose(f); return 0; }
    rewind(f); lineno = 0;
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    double t0 = ts.tv_sec + ts.tv_nsec * 1e-9;
    while (fgets(ln, sizeof ln, f)) {                         /* pass 2: play it */
        lineno++; ln[strcspn(ln, "\r\n")] = 0;
        char *p = ln; while (*p == ' ' || *p == '\t') p++;
        if (!*p || *p == '#') continue;
        script_step(p, 1, lineno, t0);
    }
    fclose(f);
    rx_command(shm, "caption");
    return 0;
}


/* ------------------------------------------------------------------ bench
 * The same setpoint sequence flown twice: once with the AI pilot in charge (guardian on as always),
 * once on the guardian's fallback controller alone (rxctl fallback on). Measured on the guardian's
 * own telemetry: ground truth on the simulator, the estimate on the serial backend.
 *   settling time  last time |x - sp| was outside the band, after the step (band: 10 % of the step, at least 5 mm)
 *   overshoot      furthest the position went past the new setpoint, in the direction of the step
 *   RMS error      over every tick of the sequence
 *   interventions  guardian take-overs during the sequence */
/* setpoints in thousandths of the plant unit: mm on the beam, mrad on the joint */
static const int BENCH_SP_BEAM[] = { 80, -80, 40, -40, 0 }, BENCH_SP_JOINT[] = { 600, -600, 300, -300, 0 };
static const int *BENCH_SP_MM = BENCH_SP_BEAM;
#define BENCH_N 5
typedef struct { double settle_s[BENCH_N], over_mm[BENCH_N]; int settled[BENCH_N]; double rms_mm; unsigned interventions; long n; } bench_res_t;

static uint64_t iv_total(void) { uint64_t t = 0; for (int i = 1; i < RX_R_COUNT; i++) t += atomic_load(&shm->interventions[i]); return t; }

static void bench_run(int fallback, double hold_s, FILE *csv, bench_res_t *R)
{
    memset(R, 0, sizeof *R);
    rx_command(shm, "fault clear"); rx_command(shm, "guard on"); rx_command(shm, "release"); rx_command(shm, "train off");
    rx_command(shm, "sp 0");
    rx_command(shm, fallback ? "fallback on" : "fallback off");
    atomic_store(&shm->bench.state, fallback ? 2u : 1u);
    if (!fallback && !wait_ai(15000)) fprintf(stderr, "bench: warning, the AI did not take control within 15 s\n");
    msleep(3000);                                              /* settle at the centre */
    uint64_t iv0 = iv_total(), idx = atomic_load(&shm->head);
    double se = 0; int prev = 0;
    for (int i = 0; i < BENCH_N; i++) {
        atomic_store(&shm->bench.step, (uint32_t)i + 1);
        char c[32]; snprintf(c, sizeof c, "sp %d", BENCH_SP_MM[i]); rx_command(shm, c);
        uint64_t t0 = rx_now_ns(), end = t0 + (uint64_t)(hold_s * 1e9);
        double d = (BENCH_SP_MM[i] - prev) * 1e-3, band = fmax(0.005, 0.1 * fabs(d)), last_out = 0, over = 0;
        double dir = d > 0 ? 1 : (d < 0 ? -1 : 0), sp = BENCH_SP_MM[i] * 1e-3;
        while (rx_now_ns() < end) {
            uint64_t h = atomic_load(&shm->head);
            rx_rec_t r;
            for (; idx < h; idx++) {
                if (!rx_shm_get(shm, idx, &r) || r.t_ns < t0) continue;
                double x = isfinite(r.x_true) ? r.x_true : r.x, e = x - sp, t = (r.t_ns - t0) * 1e-9;
                if (!isfinite(x)) continue;
                if (fabs(e) > band) last_out = t;
                if (dir * e > over) over = dir * e;
                se += e * e; R->n++;
                if (csv) fprintf(csv, "%s,%d,%.4f,%.1f,%.2f,%u\n", fallback ? "fallback" : "ai", i + 1, t, sp * 1e3, x * 1e3, r.mode);
            }
            msleep(10);
        }
        R->settle_s[i] = last_out; R->over_mm[i] = over * 1e3;
        R->settled[i] = last_out < hold_s - 0.5;                 /* still outside the band in the last 0.5 s: not settled */
        prev = BENCH_SP_MM[i];
    }
    R->rms_mm = R->n ? sqrt(se / (double)R->n) * 1e3 : NAN;
    R->interventions = (unsigned)(iv_total() - iv0);
    rx_command(shm, "fallback off");
}

static void bench_summary(const bench_res_t *R, double *settle, double *over, unsigned *unsettled)
{
    double s = 0, o = 0; int k = 0; *unsettled = 0;
    for (int i = 0; i < BENCH_N; i++) {
        if (BENCH_SP_MM[i] == (i ? BENCH_SP_MM[i - 1] : 0)) continue;
        if (R->settled[i]) { s += R->settle_s[i]; k++; } else (*unsettled)++;
        o = fmax(o, R->over_mm[i]);
    }
    *settle = k ? s / k : NAN; *over = o;
}

static int cmd_bench(FILE *out, const char *csv_path, double hold_s)
{
    const int joint = !strcmp(shm->plant, "joint");
    const char *U = joint ? "mrad" : "mm";
    if (joint) BENCH_SP_MM = BENCH_SP_JOINT;
    FILE *csv = csv_path ? fopen(csv_path, "w") : NULL;
    if (csv_path && !csv) { perror(csv_path); return 1; }
    if (csv) fprintf(csv, "pilot,step,t_s,sp_mm,x_mm,mode\n");
    atomic_store(&shm->bench.nsteps, (uint32_t)BENCH_N);
    bench_res_t R[2];
    bench_run(0, hold_s, csv, &R[0]);
    bench_run(1, hold_s, csv, &R[1]);
    if (csv) fclose(csv);
    rx_command(shm, "sp auto");
    double st[2], ov[2]; unsigned un[2];
    for (int m = 0; m < 2; m++) {
        bench_summary(&R[m], &st[m], &ov[m], &un[m]);
        shm->bench.settle_s[m] = (float)st[m]; shm->bench.overshoot_mm[m] = (float)ov[m]; shm->bench.rms_mm[m] = (float)R[m].rms_mm;
        shm->bench.interventions[m] = R[m].interventions; shm->bench.unsettled[m] = un[m];
    }
    atomic_store(&shm->bench.state, 3u);

    fprintf(out, "# Reflex benchmark: AI pilot vs fallback controller alone\n\n");
    fprintf(out, "- backend: `%s`, control rate %.0f Hz; position: %s\n", shm->backend, shm->hz,
            strcmp(shm->backend, "sim") ? "the guardian's estimate (no ground truth on this backend)" : "simulator ground truth");
    fprintf(out, "- plant: %s; setpoint sequence (%s), each held %.0f s:", joint ? "robot joint (simulated)" : "ball and beam", U, hold_s);
    for (int i = 0; i < BENCH_N; i++) fprintf(out, " %d", BENCH_SP_MM[i]);
    fprintf(out, "\n- settling: last time outside a band of 10 %% of the step (at least 5 %s); a step still outside it in the last 0.5 s counts as not settled\n\n", U);
    fprintf(out, "| | AI pilot (guardian on) | fallback controller alone |\n|---|---|---|\n");
    fprintf(out, "| mean settling time | %.2f s | %.2f s |\n", st[0], st[1]);
    fprintf(out, "| worst overshoot | %.1f %s | %.1f %s |\n", ov[0], U, ov[1], U);
    fprintf(out, "| RMS tracking error | %.1f %s | %.1f %s |\n", R[0].rms_mm, U, R[1].rms_mm, U);
    fprintf(out, "| guardian interventions | %u | %u |\n", R[0].interventions, R[1].interventions);
    fprintf(out, "| steps not settled | %u of %d | %u of %d |\n\n", un[0], BENCH_N, un[1], BENCH_N);
    fprintf(out, "| step | setpoint | AI settling | AI overshoot | fallback settling | fallback overshoot |\n|---|---|---|---|---|---|\n");
    for (int i = 0; i < BENCH_N; i++)
        fprintf(out, "| %d | %+d %s | %s%.2f s | %.1f %s | %s%.2f s | %.1f %s |\n", i + 1, BENCH_SP_MM[i], U,
                R[0].settled[i] ? "" : "not settled, ", R[0].settle_s[i], R[0].over_mm[i], U,
                R[1].settled[i] ? "" : "not settled, ", R[1].settle_s[i], R[1].over_mm[i], U);
    double fs = (st[1] - st[0]) / st[1] * 100, fr = (R[1].rms_mm - R[0].rms_mm) / R[1].rms_mm * 100;
    fprintf(out, "\nAI vs fallback: settling %s by %.0f %%, RMS error %s by %.0f %%, %u interventions while the AI flew.\n",
            fs >= 0 ? "faster" : "SLOWER", fabs(fs), fr >= 0 ? "lower" : "HIGHER", fabs(fr), R[0].interventions);
    return 0;
}


/* ------------------------------------------------------------------ stuck
 * Hardware-day check of the thinnest margin: the stuck-at-full-tilt fault is the adversary the recoverable-set
 * bound is sized for. Inject it N times with the guardian ON, each time 1 s after the setpoint square wave turns
 * positive (the phase where the fake node showed the closest approach), and report how close the plant came to
 * the end against the keep-out strip. Simulator: ground truth. Serial backend: the guardian's estimate, which
 * includes sensor noise, so also film the beam side-on with a ruler (docs/HARDWARE_DAY.md step 6). */
static int cmd_stuck(FILE *out, int n_runs)
{
    const int joint = !strcmp(shm->plant, "joint");
    const double us = joint ? 57.29578 : 1000.0, strip = shm->edge_margin > 0 ? shm->edge_margin : (joint ? 0.05 : 0.010);
    const char *U = joint ? "deg" : "mm";
    const int truth = !strcmp(shm->backend, "sim");
    double worst = 1e9; int ok = 0, lost_total = 0;
    atomic_store(&shm->sp_auto, 1);
    rx_command(shm, "train off"); rx_command(shm, "period 4000");
    fprintf(out, "# Stuck-at-full-tilt check: closest approach to the end, guardian ON\n\n");
    fprintf(out, "- backend `%s`, plant `%s`, %.0f Hz; position: %s\n", shm->backend, joint ? "joint" : "beam", shm->hz,
            truth ? "simulator ground truth" : "the guardian's estimate (includes sensor noise: confirm on video)");
    fprintf(out, "- keep-out strip: %.1f %s before each end; each run: fault for 4 s, 1 s after the setpoint turns positive\n\n", strip * us, U);
    fprintf(out, "| run | closest to the end | margin over the strip | ball/joint |\n|---|---|---|---|\n");
    for (int k = 0; k < n_runs; k++) {
        rx_command(shm, "fault clear"); rx_command(shm, "guard on"); rx_command(shm, "release");
        if (!wait_ai(15000)) fprintf(stderr, "stuck: warning, the AI did not take control before run %d\n", k + 1);
        { rx_rec_t r; float prev = 1; for (int t = 0; t < 12000; t += 5) { uint64_t h = atomic_load(&shm->head);
            if (h && rx_shm_get(shm, h - 1, &r)) { if (prev < 0 && r.sp > 0) break; prev = r.sp; } msleep(5); } msleep(1000); }
        unsigned b0 = atomic_load(&shm->balls_lost);
        uint64_t idx = atomic_load(&shm->head), t_inj = rx_now_ns(), end = t_inj + 4000000000ull;
        double edge = 1e9;
        if (rx_fault_apply(shm, "stuck", 0) != 0) { fprintf(stderr, "stuck: could not inject (%s)\n", strerror(errno)); return 1; }
        while (rx_now_ns() < end) {
            uint64_t h = atomic_load(&shm->head); rx_rec_t r;
            for (; idx < h; idx++) {
                if (!rx_shm_get(shm, idx, &r) || r.t_ns < t_inj) continue;
                double x = isfinite(r.x_true) ? r.x_true : r.x;
                if (isfinite(x) && shm->half_len - fabs(x) < edge) edge = shm->half_len - fabs(x);
            }
            msleep(10);
        }
        rx_command(shm, "fault clear");
        int lost = (int)(atomic_load(&shm->balls_lost) - b0);
        lost_total += lost;
        if (edge < worst) worst = edge;
        if (edge > strip && !lost) ok++;
        fprintf(out, "| %d | %.1f %s | %+.1f %s | %s |\n", k + 1, edge * us, U, (edge - strip) * us, U, lost ? "**LOST**" : "kept");
        fflush(out);
        msleep(1500);
    }
    int pass = ok == n_runs && lost_total == 0;
    fprintf(out, "\n**%s: closest approach %.1f %s against a %.1f %s strip; %d of %d runs outside the strip%s.**\n",
            pass ? "PASS" : "FAIL", worst * us, U, strip * us, U, ok, n_runs, lost_total ? ", ball/joint lost" : "");
    if (!pass) fprintf(out, "\nDo not tune anything: bring this file and the video (docs/HARDWARE_DAY.md step 6). The bound's t_delay must come from the measured delay.\n");
    return pass ? 0 : 3;
}

int main(int argc, char **argv)
{
    argv0 = argv[0];
    if (argc < 2 || !strcmp(argv[1], "--help") || !strcmp(argv[1], "help")) { usage(); return argc < 2 ? 2 : 0; }
    if (!strcmp(argv[1], "script") && argc >= 4 && !strcmp(argv[3], "--check")) return cmd_script(argv[2], 1);   /* no guardian needed */
    if (!strcmp(argv[1], "wipe")) {                            /* fresh world and counters for the next guardian */
        rx_shm_t *w = rx_shm_open(0);
        if (w) {
            pid_t g = atomic_load(&w->guardian_pid);
            if (g > 0 && (kill(g, 0) == 0 || errno == EPERM)) {   /* EPERM: alive, but root (HAM) */ fprintf(stderr, "rxctl: a guardian is running (pid %d); stop it first\n", (int)g); return 1; }
        }
        if (rx_shm_remove() != 0 && errno != ENOENT) { perror("rxctl: wipe"); return 1; }
        printf("shared memory removed: the next guardian starts with a fresh world and counters\n");
        return 0;
    }
    shm = rx_shm_open(0);
    if (!shm) { fprintf(stderr, "rxctl: no running guardian found (%s)\n", strerror(errno)); return 1; }
    const char *c = argv[1];
    char buf[4096], line[128];

    if (!strcmp(c, "status")) { rx_status_text(shm, buf, sizeof buf); fputs(buf, stdout); }
    else if (!strcmp(c, "watch")) {
        for (;;) { rx_status_text(shm, buf, sizeof buf); printf("\033[H\033[2J%s", buf); fflush(stdout); msleep(200); }
    }
    else if (!strcmp(c, "inject") && argc >= 3) {
        if (inject(argv[2], argc >= 4 ? (uint32_t)atoi(argv[3]) : 0) != 0) { fprintf(stderr, "rxctl: could not inject '%s': %s\n", argv[2], strerror(errno)); return 1; }
        printf("injected: %s\n", argv[2]);
    }
    else if (!strcmp(c, "clear")) rx_command(shm, "fault clear");
    else if ((!strcmp(c, "guard") || !strcmp(c, "sp") || !strcmp(c, "train") || !strcmp(c, "period")) && argc >= 3) {
        snprintf(line, sizeof line, "%s %s", c, argv[2]);
        if (rx_command(shm, line) != 0) { usage(); return 2; }
    }
    else if (!strcmp(c, "release") || !strcmp(c, "reset")) rx_command(shm, c);
    else if (!strcmp(c, "latency")) cmd_latency(stdout);
    else if (!strcmp(c, "events")) cmd_events();
    else if (!strcmp(c, "incidents") && argc >= 3) cmd_incidents(argv[2]);
    else if (!strcmp(c, "endpoint")) printf("%d %d %.3f\n", atomic_load(&shm->guardian_pid), atomic_load(&shm->chid), shm->hz);
    else if (!strcmp(c, "hog") && argc >= 4) cmd_hog(atoi(argv[2]), atoi(argv[3]));
    else if (!strcmp(c, "log") && argc >= 4) cmd_log(argv[2], atoi(argv[3]));
    else if (!strcmp(c, "caption")) {
        char t[160] = "caption";
        for (int i = 2; i < argc; i++) { strncat(t, " ", sizeof t - strlen(t) - 1); strncat(t, argv[i], sizeof t - strlen(t) - 1); }
        rx_command(shm, t);
    }
    else if (!strcmp(c, "script") && argc >= 3) return cmd_script(argv[2], argc >= 4 && !strcmp(argv[3], "--check"));
    else if (!strcmp(c, "bench")) {                            /* bench [file.md] [--csv file] [--hold s] */
        const char *md = NULL, *csvp = NULL; double hold = 5.0;
        for (int i = 2; i < argc; i++) {
            if (!strcmp(argv[i], "--csv") && i + 1 < argc) csvp = argv[++i];
            else if (!strcmp(argv[i], "--hold") && i + 1 < argc) hold = atof(argv[++i]);
            else md = argv[i];
        }
        if (!(hold >= 2 && hold <= 30)) { fprintf(stderr, "rxctl bench: --hold 2..30 s\n"); return 2; }
        FILE *f = md ? fopen(md, "w") : stdout;
        if (!f) { perror(md); return 1; }
        int rc = cmd_bench(f, csvp, hold);
        if (f != stdout) { fclose(f); printf("report written to %s\n", md); }
        return rc;
    }
    else if (!strcmp(c, "stuck")) {                            /* stuck [N] [file.md] */
        int n = 5; const char *md = NULL;
        for (int i = 2; i < argc; i++) { if (atoi(argv[i]) > 0) n = atoi(argv[i]); else md = argv[i]; }
        if (n > 50) n = 50;
        FILE *f = md ? fopen(md, "w") : stdout;
        if (!f) { perror(md); return 1; }
        int rc = cmd_stuck(f, n);
        if (f != stdout) { fclose(f); printf("report written to %s\n", md); }
        return rc;
    }
    else if (!strcmp(c, "fallback") && argc >= 3) {
        snprintf(line, sizeof line, "fallback %s", argv[2]);
        if (rx_command(shm, line) != 0) { usage(); return 2; }
    }
    else if (!strcmp(c, "campaign")) {                         /* campaign [file.md] [--guest] */
        const char *md = NULL; int guest = 0;
        for (int i = 2; i < argc; i++) { if (!strcmp(argv[i], "--guest")) guest = 1; else md = argv[i]; }
        if (guest && !is_udp()) { fprintf(stderr, "rxctl campaign --guest: the guardian's pilot transport is not udp\n"); return 2; }
        FILE *f = md ? fopen(md, "w") : stdout;
        if (!f) { perror(md); return 1; }
        campaign(f);
        if (guest) campaign_guest(f);
        if (f != stdout) { fclose(f); printf("report written to %s\n", md); }
    }
    else if (!strcmp(c, "guest") && argc >= 3) {
        if (!is_udp()) fprintf(stderr, "rxctl guest: note, this guardian's pilot transport is local, not udp\n");
        return guest_cmd(argv[2], argc >= 4 ? atoi(argv[3]) : 0, 0) == 0 ? 0 : 1;
    }
    else { usage(); return 2; }
    return 0;
}
