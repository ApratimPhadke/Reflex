/* rxdash - tiny dependency-free HTTP server for the Reflex live dashboard.
 * Reads the shared-memory telemetry block; runs at normal priority and can be
 * killed at any time without the guardian noticing.
 *   GET /            dashboard (embedded)
 *   GET /api/state   JSON snapshot: counters + last 10 s of telemetry, decimated
 *   GET /api/cmd?... same commands as `rxctl` / `echo ... > /dev/reflex`
 * SPDX-License-Identifier: MIT */
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include "rx_shm.h"
#include "dash_html.h"

static rx_shm_t *shm;
static char out[262144];

static void send_all(int fd, const char *b, size_t n)
{
    while (n) { ssize_t w = send(fd, b, n, 0); if (w <= 0) return; b += w; n -= (size_t)w; }
}

static void respond(int fd, const char *type, const char *body, size_t n)
{
    char h[256];
    int hl = snprintf(h, sizeof h, "HTTP/1.1 200 OK\r\nContent-Type: %s\r\nContent-Length: %zu\r\n"
                      "Cache-Control: no-store\r\nConnection: close\r\n\r\n", type, n);
    send_all(fd, h, (size_t)hl);
    send_all(fd, body, n);
}

static int jnum(char *p, int cap, double v, int prec)
{
    return isfinite(v) ? snprintf(p, cap, "%.*f", prec, v) : snprintf(p, cap, "null");
}

/* copy a string from shared memory into JSON-safe printable ASCII (no quotes, backslashes or controls) */
static void jstr(char *dst, size_t cap, const char *src)
{
    size_t k = 0;
    for (size_t i = 0; i < cap - 1 && src[i]; i++) {
        char c = src[i];
        dst[k++] = (c < 32 || c > 126 || c == '"' || c == '\\') ? ' ' : c;
    }
    dst[k] = 0;
}

static size_t build_state(void)
{
    char *p = out; int cap = sizeof out, n = 0;
#define ADD(...) do { n += snprintf(p + n, n < cap ? cap - n : 0, __VA_ARGS__); } while (0)
    ADD("{\"hz\":%.0f,\"half_len\":%.3f,\"theta_max\":%.3f,\"backend\":\"%s\",", shm->hz, shm->half_len, shm->theta_max, shm->backend);
    {   /* serial backend: calibration, link and node identity; demo caption (all strings sanitised for JSON) */
        char cal[sizeof shm->calibrated_at], brd[sizeof shm->node_board], capt[sizeof shm->caption];
        jstr(cal, sizeof cal, shm->calibrated_at); jstr(brd, sizeof brd, shm->node_board);
        uint32_t c0 = atomic_load(&shm->caption_seq);
        jstr(capt, sizeof capt, shm->caption);
        if ((c0 & 1) || atomic_load(&shm->caption_seq) != c0) capt[0] = 0;      /* being rewritten: show nothing this frame */
        ADD("\"calibrated_at\":\"%s\",\"node_link\":%u,\"node_proto\":%u,\"node_emulated\":%u,\"node_board\":\"%s\",\"caption\":\"%s\",",
            cal, atomic_load(&shm->node_link), atomic_load(&shm->node_proto), atomic_load(&shm->node_emulated), brd, capt);
    }
    ADD("\"mode\":\"%s\",\"reason\":\"%s\",\"guard\":%u,\"quarantined\":%u,\"fault\":\"%s\",",
        rx_mode_name[atomic_load(&shm->mode) % 3], rx_reason_name[atomic_load(&shm->reason) % RX_R_COUNT],
        atomic_load(&shm->guard_enabled), atomic_load(&shm->quarantined), rx_fault_name[atomic_load(&shm->fault) % RX_F_COUNT]);
    ADD("\"ticks\":%llu,\"deadline_miss\":%llu,\"lat_max_us\":%.1f,\"exec_max_us\":%.1f,",
        (unsigned long long)atomic_load(&shm->ticks), (unsigned long long)atomic_load(&shm->deadline_miss),
        atomic_load(&shm->lat_max_ns) / 1e3, atomic_load(&shm->exec_max_ns) / 1e3);
    ADD("\"pilot_pid\":%d,\"pilot_present\":%u,\"pilot_restarts\":%u,\"pilot_kills\":%u,\"balls_lost\":%u,\"guardian_starts\":%u,",
        atomic_load(&shm->pilot_pid), atomic_load(&shm->pilot_present), atomic_load(&shm->pilot_restarts),
        atomic_load(&shm->pilot_kills), atomic_load(&shm->balls_lost), atomic_load(&shm->guardian_starts));
    ADD("\"interventions\":{");
    for (int i = 1; i < RX_R_COUNT; i++) ADD("%s\"%s\":%llu", i > 1 ? "," : "", rx_reason_name[i], (unsigned long long)atomic_load(&shm->interventions[i]));
    ADD("},\"hist\":[");
    for (int i = 0; i < RX_HIST_BINS; i++) ADD("%s%llu", i ? "," : "", (unsigned long long)atomic_load(&shm->lat_hist[i]));
    ADD("],\"training\":%u,\"learn\":{\"episodes\":%u,\"avg\":%.4f,\"cost\":[", atomic_load(&shm->training),
        atomic_load(&shm->learn_episodes), isfinite(shm->learn_avg) ? shm->learn_avg : 0.0);
    {
        uint32_t ne = atomic_load(&shm->learn_episodes), span = ne < RX_LEARN_HIST - 2 ? ne : RX_LEARN_HIST - 2;
        for (uint32_t i = ne - span; i < ne; i++) ADD("%s%.3f", i > ne - span ? "," : "", shm->learn_cost[i & (RX_LEARN_HIST - 1)]);
        ADD("],\"iv\":[");
        for (uint32_t i = ne - span; i < ne; i++) ADD("%s%u", i > ne - span ? "," : "", shm->learn_iv[i & (RX_LEARN_HIST - 1)]);
        ADD("]},\"events\":[");
        uint64_t eh = atomic_load(&shm->ev_head), now = rx_now_ns(); int first = 1;
        for (uint64_t i = eh > 9 ? eh - 9 : 0; i < eh; i++) {
            rx_event_t e;
            if (!rx_event_get(shm, i, &e)) continue;
            const char *nm = e.code < RX_E_COUNT ? rx_event_name[e.code] : "?";
            ADD("%s[%.1f,\"%s%s%s\"]", first ? "" : ",", (now - e.t_ns) * 1e-9, nm, e.code == RX_E_INTERVENE ? ": " : "",
                e.code == RX_E_INTERVENE ? rx_reason_name[e.arg % RX_R_COUNT] : "");
            first = 0;
        }
    }
    ADD("],\"recs\":[");
    /* last 10 s, at most 300 points. columns: seq,x,sp,u_ai,u_out,mode,reason,lat_us,lost,x_true */
    uint64_t head = atomic_load(&shm->head), span = (uint64_t)(10 * shm->hz);
    if (span > RX_RING - 64) span = RX_RING - 64;
    if (span > head) span = head;
    uint64_t step = span / 300 + 1, first = 1;
    for (uint64_t i = head - span; i < head && n < cap - 256; i += step) {
        rx_rec_t r;
        if (!rx_shm_get(shm, i, &r)) continue;
        if (i + step >= head) rx_shm_get(shm, head - 1, &r);         /* always end on the newest record */
        ADD("%s[%u,", first ? "" : ",", r.seq); first = 0;
        n += jnum(p + n, cap - n, r.x, 4);      ADD(",");
        n += jnum(p + n, cap - n, r.sp, 3);     ADD(",");
        n += jnum(p + n, cap - n, r.u_ai, 4);   ADD(",");
        n += jnum(p + n, cap - n, r.u_out, 4);  ADD(",%u,%u,%.1f,%u,", r.mode, r.reason, r.lat_ns / 1e3, r.lost);
        n += jnum(p + n, cap - n, r.x_true, 4); ADD("]");
    }
    ADD("]}");
#undef ADD
    return (size_t)(n < cap ? n : cap - 1);
}

static void urldecode(char *s)
{
    char *o = s;
    for (; *s; s++) {
        if (*s == '+') *o++ = ' ';
        else if (*s == '%' && isxdigit((unsigned char)s[1]) && isxdigit((unsigned char)s[2])) {
            char h[3] = { s[1], s[2], 0 }; *o++ = (char)strtol(h, NULL, 16); s += 2;
        } else *o++ = *s;
    }
    *o = 0;
}

int main(int argc, char **argv)
{
    int port = argc > 1 ? atoi(argv[1]) : 8080;
    signal(SIGPIPE, SIG_IGN);
    while (!(shm = rx_shm_open(0))) { fprintf(stderr, "rxdash: waiting for guardian...\n"); sleep(1); }

    int ls = socket(AF_INET, SOCK_STREAM, 0), one = 1;
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in a; memset(&a, 0, sizeof a);
    a.sin_family = AF_INET; a.sin_addr.s_addr = htonl(INADDR_ANY); a.sin_port = htons((uint16_t)port);
    if (bind(ls, (struct sockaddr *)&a, sizeof a) < 0 || listen(ls, 16) < 0) { perror("rxdash: bind"); return 1; }
    fprintf(stderr, "rxdash: open http://<target-ip>:%d/ in a browser\n", port);

    for (;;) {
        int fd = accept(ls, NULL, NULL);
        if (fd < 0) continue;
        struct timeval tv = { 2, 0 };
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
        char req[1024];
        ssize_t n = recv(fd, req, sizeof req - 1, 0);
        if (n > 0) {
            req[n] = 0;
            char path[512] = "";
            sscanf(req, "%*s %511s", path);
            if (!strncmp(path, "/api/state", 10)) respond(fd, "application/json", out, build_state());
            else if (!strncmp(path, "/api/cmd?", 9)) {
                urldecode(path + 9);
                int rc = rx_command(shm, path + 9);
                respond(fd, "text/plain", rc ? "error\n" : "ok\n", rc ? 6 : 3);
            } else respond(fd, "text/html; charset=utf-8", (const char *)dash_html, dash_html_len);
        }
        close(fd);
    }
}
