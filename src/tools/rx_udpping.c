/* rx_udpping - UDP round-trip time of the pilot link, measured the way the guardian uses it.
 * SPDX-License-Identifier: MIT
 *
 *   rx_udpping echo [port]                          (pilot side: answer every probe; default port 47801)
 *   rx_udpping ping <ip:port> [--hz 50] [--secs 150] [--bind 47800] [--csv file]   (guardian side)
 *
 * The pinger sends one probe per period at an absolute schedule (like the guardian's state datagram: 52 bytes),
 * the echo answers with 48 bytes (like a proposal). The pinger timestamps send and receive with
 * CLOCK_MONOTONIC on its own clock, so no clock sync is needed. A reply that never arrives within --drain
 * seconds after the last probe counts as lost; duplicates are counted, not used. Measurement tool only:
 * it shares nothing with the guardian. */
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#ifndef RX_GIT
#define RX_GIT "unknown"
#endif

#define PROBE_MAGIC 0x47505852u   /* 'R' 'X' 'P' 'G' */
#define PROBE_LEN   52            /* sizeof(rx_wire_state_t) */
#define REPLY_LEN   48            /* sizeof(rx_wire_prop_t) */

static uint64_t now_ns(void)
{
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * 1000000000ull + (uint64_t)t.tv_nsec;
}

static int udp_socket(int port)
{
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) { perror("socket"); return -1; }
    struct sockaddr_in a; memset(&a, 0, sizeof a);
    a.sin_family = AF_INET; a.sin_port = htons((uint16_t)port); a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(s, (struct sockaddr *)&a, sizeof a) != 0) { perror("bind"); close(s); return -1; }
    return s;
}

static int do_echo(int port)
{
    int s = udp_socket(port);
    if (s < 0) return 1;
    fprintf(stderr, "rx_udpping echo: listening on UDP %d (build %s)\n", port, RX_GIT);
    unsigned char b[256];
    for (;;) {
        struct sockaddr_in from; socklen_t fl = sizeof from;
        ssize_t n = recvfrom(s, b, sizeof b, 0, (struct sockaddr *)&from, &fl);
        if (n < 0) { if (errno == EINTR) continue; perror("recvfrom"); return 1; }
        uint32_t m; if (n != PROBE_LEN) continue;
        memcpy(&m, b, 4); if (m != PROBE_MAGIC) continue;
        sendto(s, b, REPLY_LEN, 0, (struct sockaddr *)&from, fl);
    }
}

static int cmp_u64(const void *a, const void *b)
{
    uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;
    return x < y ? -1 : x > y;
}

static double pct(const uint64_t *v, size_t n, double p)   /* nearest rank, in us */
{
    if (!n) return -1;
    size_t k = (size_t)(p * (double)n + 0.999999); if (k < 1) k = 1; if (k > n) k = n;
    return v[k - 1] / 1e3;
}

static int do_ping(const char *dst, double hz, double secs, int bind_port, const char *csv, double drain)
{
    char host[64]; snprintf(host, sizeof host, "%s", dst);
    char *colon = strrchr(host, ':');
    if (!colon) { fprintf(stderr, "ping: want ip:port\n"); return 2; }
    *colon = 0;
    struct sockaddr_in to; memset(&to, 0, sizeof to);
    to.sin_family = AF_INET; to.sin_port = htons((uint16_t)atoi(colon + 1));
    if (inet_pton(AF_INET, host, &to.sin_addr) != 1) { fprintf(stderr, "ping: bad address %s\n", host); return 2; }
    if (hz < 1 || hz > 2000 || secs <= 0) { fprintf(stderr, "ping: --hz 1..2000, --secs > 0\n"); return 2; }

    size_t n = (size_t)(hz * secs + 0.5);
    uint64_t *t_send = calloc(n, sizeof *t_send), *rtt = calloc(n, sizeof *rtt), *sorted = calloc(n, sizeof *sorted);
    unsigned *seen = calloc(n, sizeof *seen);
    if (!t_send || !rtt || !seen || !sorted) { fprintf(stderr, "ping: out of memory\n"); return 1; }
    int s = udp_socket(bind_port);
    if (s < 0) return 1;

    const uint64_t period = (uint64_t)(1e9 / hz);
    uint64_t start = now_ns() + 100000000ull, next = start, send_late_max = 0;
    size_t sent = 0; unsigned long dups = 0, junk = 0, send_err = 0;
    uint64_t end = 0;
    unsigned char b[256];
    for (;;) {
        uint64_t t = now_ns();
        if (sent < n && t >= next) {
            memset(b, 0, PROBE_LEN);
            uint32_t m = PROBE_MAGIC, q = (uint32_t)sent;
            memcpy(b, &m, 4); memcpy(b + 4, &q, 4);
            t_send[sent] = now_ns();
            if (t_send[sent] - next > send_late_max) send_late_max = t_send[sent] - next;
            if (sendto(s, b, PROBE_LEN, 0, (struct sockaddr *)&to, sizeof to) != PROBE_LEN) send_err++;
            sent++; next += period;
            if (sent == n) end = now_ns() + (uint64_t)(drain * 1e9);
            continue;
        }
        if (sent == n && t >= end) break;
        uint64_t until = sent < n ? next : end;
        int ms = (int)((until - t + 999999) / 1000000);
        struct pollfd p = { .fd = s, .events = POLLIN };
        if (poll(&p, 1, ms) <= 0) continue;
        ssize_t r = recv(s, b, sizeof b, 0);
        uint64_t tr = now_ns();
        uint32_t m, q;
        if (r != REPLY_LEN) { junk++; continue; }
        memcpy(&m, b, 4); memcpy(&q, b + 4, 4);
        if (m != PROBE_MAGIC || q >= sent) { junk++; continue; }
        if (seen[q]++) { dups++; continue; }
        rtt[q] = tr - t_send[q];
    }

    size_t got = 0; unsigned long over_period = 0, over_stale = 0, lost_runs = 0, run = 0, max_run = 0;
    const uint64_t stale = (uint64_t)(2.5e9 / hz);     /* the guardian's max_age_s = 2.5 periods (rx_cfg_hardware) */
    for (size_t i = 0; i < n; i++) {
        if (seen[i]) {
            sorted[got++] = rtt[i];
            if (rtt[i] > period) over_period++;
            if (rtt[i] > stale) over_stale++;
            if (run) { lost_runs++; if (run > max_run) max_run = run; run = 0; }
        } else run++;
    }
    if (run) { lost_runs++; if (run > max_run) max_run = run; }
    qsort(sorted, got, sizeof *sorted, cmp_u64);

    if (csv) {
        FILE *f = fopen(csv, "w");
        if (!f) perror(csv);
        else {
            fprintf(f, "seq,t_send_ms,rtt_us\n");
            for (size_t i = 0; i < n; i++) {
                if (seen[i]) fprintf(f, "%zu,%.3f,%.1f\n", i, (t_send[i] - start) / 1e6, rtt[i] / 1e3);
                else fprintf(f, "%zu,%.3f,\n", i, (t_send[i] - start) / 1e6);
            }
            fclose(f);
        }
    }
    printf("rx_udpping build %s: %zu probes of %d B to %s at %.1f Hz for %.0f s, replies of %d B, drain %.1f s\n",
           RX_GIT, n, PROBE_LEN, dst, hz, secs, REPLY_LEN, drain);
    printf("received %zu, lost %zu (%.3f %%), lost bursts %lu, longest burst %lu probes (%.0f ms), duplicates %lu, junk %lu, send errors %lu\n",
           got, n - got, 100.0 * (double)(n - got) / (double)n, lost_runs, max_run, max_run * 1e3 / hz, dups, junk, send_err);
    if (got) {
        printf("rtt us: min %.0f  p50 %.0f  p90 %.0f  p99 %.0f  p99.9 %.0f  max %.0f\n",
               sorted[0] / 1e3, pct(sorted, got, .50), pct(sorted, got, .90), pct(sorted, got, .99), pct(sorted, got, .999),
               sorted[got - 1] / 1e3);
        printf("replies later than one period (%.1f ms): %lu (%.3f %%); later than 2.5 periods (%.1f ms): %lu (%.3f %%)\n",
               period / 1e6, over_period, 100.0 * over_period / got, stale / 1e6, over_stale, 100.0 * over_stale / got);
    }
    printf("send schedule: worst send %.2f ms after its slot\n", send_late_max / 1e6);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc >= 2 && !strcmp(argv[1], "echo")) return do_echo(argc >= 3 ? atoi(argv[2]) : 47801);
    if (argc >= 3 && !strcmp(argv[1], "ping")) {
        double hz = 50, secs = 150, drain = 2; int bind_port = 47800; const char *csv = NULL;
        for (int i = 3; i < argc; i++) {
            const char *v = i + 1 < argc ? argv[i + 1] : NULL;
            if (!strcmp(argv[i], "--hz") && v) { hz = atof(v); i++; }
            else if (!strcmp(argv[i], "--secs") && v) { secs = atof(v); i++; }
            else if (!strcmp(argv[i], "--bind") && v) { bind_port = atoi(v); i++; }
            else if (!strcmp(argv[i], "--csv") && v) { csv = v; i++; }
            else if (!strcmp(argv[i], "--drain") && v) { drain = atof(v); i++; }
            else { fprintf(stderr, "unknown option %s\n", argv[i]); return 2; }
        }
        return do_ping(argv[2], hz, secs, bind_port, csv, drain);
    }
    fprintf(stderr, "usage: rx_udpping echo [port]\n"
                    "       rx_udpping ping <ip:port> [--hz 50] [--secs 150] [--bind 47800] [--csv file] [--drain 2]\n");
    return 2;
}
