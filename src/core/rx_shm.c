/* Shared-memory telemetry block + helpers used by every Reflex process.
 * SPDX-License-Identifier: MIT */
#include <errno.h>
#include <math.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include "rx_shm.h"
#include "rx_guest.h"

uint64_t rx_now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

rx_shm_t *rx_shm_open(int create)
{
    int fd = shm_open(RX_SHM_NAME, create ? (O_RDWR | O_CREAT) : O_RDWR, 0666);
    if (fd < 0) return NULL;
    if (create && ftruncate(fd, sizeof(rx_shm_t)) < 0) { close(fd); return NULL; }
    if (create) (void)fchmod(fd, 0666);   /* a guardian started as root (HAM) must stay reachable for rxctl as a user */
    struct stat st;
    if (fstat(fd, &st) < 0 || (size_t)st.st_size < sizeof(rx_shm_t)) { close(fd); errno = EINVAL; return NULL; }
    rx_shm_t *s = mmap(NULL, sizeof(rx_shm_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (s == MAP_FAILED) return NULL;
    if (create && (s->magic != RX_MAGIC || s->version != RX_VERSION)) {
        rx_shm_defaults(s);
    }
    if (!create && (s->magic != RX_MAGIC || s->version != RX_VERSION)) { munmap(s, sizeof *s); errno = EPROTO; return NULL; }
    return s;
}

int rx_shm_remove(void) { return shm_unlink(RX_SHM_NAME); }

void rx_shm_defaults(rx_shm_t *s)
{
    memset(s, 0, sizeof *s);
    s->magic = RX_MAGIC; s->version = RX_VERSION;
    atomic_store(&s->guard_enabled, 1);
    atomic_store(&s->sp_auto, 1);
    atomic_store(&s->sp_period_ms, 4000);
}

void rx_event_push(rx_shm_t *s, uint32_t seq, uint16_t code, int32_t arg)
{
    uint64_t h = atomic_load_explicit(&s->ev_head, memory_order_relaxed);
    rx_event_t e = { rx_now_ns(), seq, code, arg };
    s->events[h & (RX_EVENTS - 1)] = e;
    atomic_store_explicit(&s->ev_head, h + 1, memory_order_release);
}

int rx_event_get(const rx_shm_t *s, uint64_t idx, rx_event_t *out)
{
    uint64_t h = atomic_load_explicit(&s->ev_head, memory_order_acquire);
    if (idx >= h || h - idx >= RX_EVENTS - 2) return 0;
    *out = s->events[idx & (RX_EVENTS - 1)];
    atomic_thread_fence(memory_order_acquire);                  /* see rx_shm_get() */
    h = atomic_load_explicit(&s->ev_head, memory_order_relaxed);
    return h - idx < RX_EVENTS - 2;
}

void rx_shm_push(rx_shm_t *s, const rx_rec_t *r)
{
    uint64_t h = atomic_load_explicit(&s->head, memory_order_relaxed);
    s->ring[h & (RX_RING - 1)] = *r;
    atomic_store_explicit(&s->head, h + 1, memory_order_release);
}

int rx_shm_get(const rx_shm_t *s, uint64_t idx, rx_rec_t *out)
{
    uint64_t h = atomic_load_explicit(&s->head, memory_order_acquire);
    if (idx >= h || h - idx >= RX_RING - 8) return 0;
    *out = s->ring[idx & (RX_RING - 1)];
    /* Overwritten while copying? An acquire load alone does not stop the copy above from being
     * reordered after it (it only orders LATER accesses), so on a weakly ordered CPU such as the
     * Pi's Cortex-A72 a torn record could pass the check. The fence orders the copy first. */
    atomic_thread_fence(memory_order_acquire);
    h = atomic_load_explicit(&s->head, memory_order_relaxed);
    return h - idx < RX_RING - 8;
}

int rx_lat_bin(uint32_t lat_ns)
{
    uint32_t us = lat_ns / 1000u;
    int b = 0;
    while (us > 1 && b < RX_HIST_BINS - 1) { us >>= 1; b++; }
    return b;
}

/* ---- fault injection, shared by rxctl, rxdash and the QNX resource manager ---- */
int rx_fault_apply(rx_shm_t *s, const char *name, uint32_t arg)
{
    pid_t pilot = atomic_load(&s->pilot_pid);
    if (!strcasecmp(name, "clear") || !strcasecmp(name, "none")) {
        atomic_store(&s->fault, RX_F_NONE);
        if (pilot > 0) kill(pilot, SIGCONT);
        return 0;
    }
    if ((!strcasecmp(name, "crash") || !strcasecmp(name, "freeze")) && pilot <= 0) { errno = ESRCH; return -1; }
    if (!strcasecmp(name, "crash"))  return kill(pilot, SIGKILL);
    if (!strcasecmp(name, "freeze")) return kill(pilot, SIGSTOP);
    for (unsigned i = 1; i < RX_F_COUNT; i++) {
        if (!strcasecmp(name, rx_fault_name[i])) {
            atomic_store(&s->fault_arg, arg ? arg : 50);
            atomic_store(&s->fault, i);
            return 0;
        }
    }
    errno = EINVAL;
    return -1;
}

/* One-line textual commands: "fault flip", "guard off", "sp 50", "sp auto", "train on", "period 3000", "release", "reset" */
int rx_command(rx_shm_t *s, const char *line)
{
    char a[32] = "", b[32] = ""; unsigned n = 0;
    {   /* "caption <free text>" sets the dashboard banner, "caption" alone clears it */
        const char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (!strncasecmp(p, "caption", 7) && (p[7] == 0 || p[7] == ' ' || p[7] == '\t' || p[7] == '\n')) {
            p += 7;
            while (*p == ' ' || *p == '\t') p++;
            char t[sizeof s->caption];
            size_t k = 0;
            for (; *p && *p != '\n' && *p != '\r' && k < sizeof t - 1; p++) t[k++] = (*p >= 32 && *p < 127) ? *p : ' ';
            t[k] = 0;
            atomic_fetch_add(&s->caption_seq, 1);
            memcpy(s->caption, t, sizeof t);
            atomic_fetch_add(&s->caption_seq, 1);
            return 0;
        }
    }
    {   /* "gains <wn> <zeta>": the fallback PD's natural frequency [rad/s] and damping, bounded */
        float wn, ze; char w[8] = "";
        if (sscanf(line, " %7s %f %f", w, &wn, &ze) == 3 && !strcasecmp(w, "gains")) {
            if (!(wn >= 0.5f && wn <= 8.0f && ze >= 0.3f && ze <= 2.0f)) { errno = EINVAL; return -1; }
            s->cmd_wn = wn; s->cmd_zeta = ze; atomic_store(&s->cmd_gains, 1); return 0;
        }
    }
    int k = sscanf(line, "%31s %31s %u", a, b, &n);
    if (k < 1) return -1;
    if (!strcasecmp(a, "fault") && k >= 2) return rx_fault_apply(s, b, n);
    if (!strcasecmp(a, "guard") && k >= 2) { atomic_store(&s->guard_enabled, !strcasecmp(b, "on")); return 0; }
    if (!strcasecmp(a, "sp") && k >= 2) {
        if (!strcasecmp(b, "auto")) atomic_store(&s->sp_auto, 1);
        else { atomic_store(&s->sp_manual_mm, atoi(b)); atomic_store(&s->sp_auto, 0); }
        return 0;
    }
    if (!strcasecmp(a, "train") && k >= 2) { atomic_store(&s->training, !strcasecmp(b, "on")); return 0; }
    if (!strcasecmp(a, "period") && k >= 2) { int ms = atoi(b); if (ms < 500 || ms > 60000) { errno = EINVAL; return -1; } atomic_store(&s->sp_period_ms, (uint32_t)ms); return 0; }
    if (!strcasecmp(a, "release")) { atomic_store(&s->cmd_release_quarantine, 1); return 0; }
    if (!strcasecmp(a, "tilt") && k >= 2) {                      /* beam-direct: operator tilt hold */
        if (!strcasecmp(b, "off")) { atomic_store(&s->beam_hold, 0); return 0; }
        char *e; double mr = strtod(b, &e);
        if (e == b || *e || !isfinite(mr) || mr < -60 || mr > 60) { errno = EINVAL; return -1; }
        atomic_store(&s->beam_hold_mrad10, (int32_t)lrint(mr * 10)); atomic_store(&s->beam_hold, 1); return 0;
    }
    if (!strcasecmp(a, "fallback") && k >= 2) {                  /* fly on the fallback controller alone */
        if (strcasecmp(b, "on") && strcasecmp(b, "off")) { errno = EINVAL; return -1; }
        atomic_store(&s->force_fallback, !strcasecmp(b, "on")); return 0;
    }
    if (!strcasecmp(a, "reset"))   { atomic_store(&s->cmd_reset_stats, 1); return 0; }
    errno = EINVAL;
    return -1;
}

int rx_status_text(const rx_shm_t *s, char *buf, int cap)
{
    uint64_t ticks = atomic_load(&s->ticks), iv = 0;
    for (int i = 1; i < RX_R_COUNT; i++) iv += atomic_load(&s->interventions[i]);
    rx_rec_t r; memset(&r, 0, sizeof r);
    uint64_t h = atomic_load(&s->head);
    if (h) rx_shm_get(s, h - 1, &r);
    char plant_line[128], cmd_line[96];
    if (!strcmp(s->plant, "joint")) {
        snprintf(plant_line, sizeof plant_line, "joint             angle=%+.1fdeg w=%+.1fdeg/s setpoint=%+.0fdeg  margin=%.1fdeg",
                 r.x * 57.29578, r.v * 57.29578, r.sp * 57.29578, r.margin * 57.29578);
        snprintf(cmd_line, sizeof cmd_line, "command           pilot=%+.3fNm  actuator=%+.3fNm", r.u_ai, r.u_out);
    } else {
        snprintf(plant_line, sizeof plant_line, "ball              x=%+.1fmm v=%+.3fm/s setpoint=%+.0fmm  margin=%.1fmm",
                 r.x * 1e3, r.v, r.sp * 1e3, r.margin * 1e3);
        snprintf(cmd_line, sizeof cmd_line, "command           pilot=%+.4frad  actuator=%+.4frad", r.u_ai, r.u_out);
    }
    int n = snprintf(buf, cap,
        "reflex            v%d  backend=%s  plant=%s  rate=%.0fHz\n"
        "mode              %s (%s)%s\n"
        "%s\n"
        "%s\n"
        "ticks             %llu\n"
        "wake latency      max=%.1fus  deadline-misses=%llu\n"
        "guardian exec     max=%.1fus\n"
        "interventions     %llu total\n",
        RX_VERSION, s->backend, s->plant[0] ? s->plant : "beam", s->hz,
        rx_mode_name[atomic_load(&s->mode) % 3], rx_reason_name[atomic_load(&s->reason) % RX_R_COUNT],
        !atomic_load(&s->guard_enabled) ? "  ** GUARDIAN BYPASSED **" : atomic_load(&s->force_fallback) ? "  (fallback only: pilot ignored)" : "",
        plant_line, cmd_line,
        (unsigned long long)ticks,
        atomic_load(&s->lat_max_ns) / 1e3, (unsigned long long)atomic_load(&s->deadline_miss),
        atomic_load(&s->exec_max_ns) / 1e3, (unsigned long long)iv);
    for (int i = 1; i < RX_R_COUNT && n < cap; i++) {
        uint64_t c = atomic_load(&s->interventions[i]);
        if (c) n += snprintf(buf + n, cap - n, "  %-15s %llu\n", rx_reason_name[i], (unsigned long long)c);
    }
    if (n < cap)
        n += snprintf(buf + n, cap - n,
        "pilot             pid=%d present=%u restarts=%u kills=%u quarantined=%u fault=%s\n"
        "balls lost        %u\n"
        "guardian starts   %u\n"
        "spoofed senders   %u refused\n"
        "learning          %s  episodes=%u  smoothed cost=%.3f\n",
        atomic_load(&s->pilot_pid), atomic_load(&s->pilot_present), atomic_load(&s->pilot_restarts),
        atomic_load(&s->pilot_kills), atomic_load(&s->quarantined),
        rx_fault_name[atomic_load(&s->fault) % RX_F_COUNT],
        atomic_load(&s->balls_lost), atomic_load(&s->guardian_starts), atomic_load(&s->rejected_senders),
        atomic_load(&s->training) ? "ON" : "off", atomic_load(&s->learn_episodes), s->learn_avg);
    if (n < cap && !strcmp(s->pilot_transport, "udp"))
        n += snprintf(buf + n, cap - n,
        "guest             %s  last valid proposal %u ms ago  restarts=%u%s\n"
        "pilot link        udp from %s  instance=%08x (%u seen)\n"
        "  datagrams       in ok=%llu refused: wrong-peer=%llu malformed=%llu replayed=%llu   out ok=%llu failed=%llu\n",
        rx_guest_name[atomic_load(&s->guest_state) % 5], atomic_load(&s->guest_age_ms), atomic_load(&s->guest_restarts),
        atomic_load(&s->guest_cmd_running) ? "  (restart command running)" : "",
        s->pilot_peer, atomic_load(&s->udp_boot_id), atomic_load(&s->udp_boots),
        (unsigned long long)atomic_load(&s->udp_rx_ok), (unsigned long long)atomic_load(&s->udp_rx_peer),
        (unsigned long long)atomic_load(&s->udp_rx_malformed), (unsigned long long)atomic_load(&s->udp_rx_replay),
        (unsigned long long)atomic_load(&s->udp_tx_ok), (unsigned long long)atomic_load(&s->udp_tx_err));
    return n < cap ? n : cap;
}
