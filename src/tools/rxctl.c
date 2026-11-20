/* rxctl - operator console, fault injector and evidence generator for Reflex.
 * SPDX-License-Identifier: MIT */
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "rx_shm.h"

static rx_shm_t *shm;

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
         "  script <file> [--check] play a timed demo script (demo/video.rxs); --check only validates it");
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
    const int run_ms = 4000;
    fprintf(out, "# Reflex fault campaign\n\n");
    fprintf(out, "- backend: `%s`, control rate %.0f Hz\n- each fault is injected into a healthy system and left active for %d s\n"
                 "- *without Reflex* = the same pilot, same faults, guardian in bypass (pilot commands go straight to the actuator)\n\n",
            shm->backend, shm->hz, run_ms / 1000);
    if (strcmp(shm->backend, "sim"))
        fprintf(out, "- *closest to beam end* is the guardian's **estimate** (no ground truth on this backend; it includes sensor noise)\n\n");
    fprintf(out, "| # | Injected pilot fault | Guardian took over after | Closest to beam end | Ball (with Reflex) | Pilot restarts | Ball (without Reflex) |\n");
    fprintf(out, "|---|---|---|---|---|---|---|\n");

    int kept = 0, lost_without = 0, skipped = 0;
    atomic_store(&shm->sp_auto, 1);
    rx_command(shm, "train off"); rx_command(shm, "period 4000");   /* a campaign is run against a frozen pilot */
    for (unsigned fi = 0; fi < NFAULTS; fi++) {
        const fault_t *F = &FAULTS[fi];
        double detect_ms = -1, min_edge = 1e9; unsigned lost_g = 0, lost_b = 0, restarts = 0;
        int not_injected = 0;
        for (int pass = 0; pass < 2; pass++) {                  /* 0 = guarded, 1 = bypass */
            rx_command(shm, "fault clear"); rx_command(shm, "guard on"); rx_command(shm, "release");
            if (!wait_ai(10000)) fprintf(stderr, "campaign: warning, system did not return to AI mode before '%s'\n", F->name);
            /* always start a run at the same phase of the setpoint square wave: 1 s after it goes positive */
            { rx_rec_t r; float prev = 1; for (int t = 0; t < 12000; t += 5) { uint64_t h = atomic_load(&shm->head);
                if (h && rx_shm_get(shm, h - 1, &r)) { if (prev < 0 && r.sp > 0) break; prev = r.sp; } msleep(5); } msleep(1000); }
            if (pass == 1) rx_command(shm, "guard off");
            unsigned b0 = atomic_load(&shm->balls_lost);
            unsigned r0 = atomic_load(&shm->pilot_restarts) + atomic_load(&shm->pilot_kills);
            uint64_t idx = atomic_load(&shm->head);
            uint64_t t_inj = rx_now_ns();
            if (rx_fault_apply(shm, F->name, F->arg) != 0) {        /* e.g. no permission to signal the pilot */
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
            if (pass == 0) { lost_g = lost; restarts = atomic_load(&shm->pilot_restarts) + atomic_load(&shm->pilot_kills) - r0; }
            else lost_b = lost;
            rx_command(shm, "fault clear"); rx_command(shm, "guard on");
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
        fprintf(out, "| %u | %s | %s | %.1f cm | %s | %u | %s |\n", fi + 1, F->what, det, min_edge * 100,
                lost_g ? "**LOST**" : "kept", restarts, lost_b ? "**LOST**" : "kept");
        fflush(out);
        if (!lost_g) kept++;
        if (lost_b) lost_without++;
        fprintf(stderr, "campaign: %-7s guarded=%s bypass=%s\n", F->name, lost_g ? "LOST" : "kept", lost_b ? "LOST" : "kept");
    }
    rx_command(shm, "release");
    unsigned ran = (unsigned)NFAULTS - (unsigned)skipped;
    fprintf(out, "\n**With Reflex: ball kept in %d of %u fault scenarios. Without Reflex: ball lost in %d of %u.**\n\n",
            kept, ran, lost_without, ran);
    if (skipped) fprintf(out, "**%d fault(s) could not be injected and are not counted above.**\n\n", skipped);
    fprintf(out, "```\n"); cmd_latency(out); fprintf(out, "```\n");
    fprintf(out, "\nGuardian worst-case execution time per tick: %.1f us. Spoofed proposals refused: %u.\n",
            atomic_load(&shm->exec_max_ns) / 1e3, atomic_load(&shm->rejected_senders));
}

/* ------------------------------------------------------------------ script
 * One step per line; '#' starts a comment line. Steps:
 *   wait <seconds>            pause (decimals allowed)
 *   wait-ai [seconds]         wait until the AI has been in control for 1.5 s (default timeout 15 s)
 *   caption [text]            dashboard banner (empty = clear)
 *   say <text>                cue for the operator, printed in the terminal only
 *   inject <fault> [ms]       as rxctl inject
 *   clear | release | reset | guard on|off | sp <mm>|auto | train on|off | period <ms>
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
        if (rx_fault_apply(shm, a1, k >= 3 ? (uint32_t)atoi(a2) : 0) != 0) printf("           (no pilot to inject into)\n");
        return 0;
    }
    int known = (!strcmp(w, "clear") || !strcmp(w, "release") || !strcmp(w, "reset")) && k == 1;
    known |= !strcmp(w, "guard") && k == 2 && (!strcmp(a1, "on") || !strcmp(a1, "off"));
    known |= !strcmp(w, "train") && k == 2 && (!strcmp(a1, "on") || !strcmp(a1, "off"));
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

int main(int argc, char **argv)
{
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
        if (rx_fault_apply(shm, argv[2], argc >= 4 ? (uint32_t)atoi(argv[3]) : 0) != 0) { fprintf(stderr, "rxctl: could not inject '%s': %s\n", argv[2], strerror(errno)); return 1; }
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
    else if (!strcmp(c, "campaign")) {
        FILE *f = argc >= 3 ? fopen(argv[2], "w") : stdout;
        if (!f) { perror(argv[2]); return 1; }
        campaign(f);
        if (f != stdout) { fclose(f); printf("report written to %s\n", argv[2]); }
    }
    else { usage(); return 2; }
    return 0;
}
