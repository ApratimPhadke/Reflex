/* Offline tests and fuzzing of the I/O node line parser (src/core/rx_link.c).
 * Exit code 0 only if every check passes.
 * SPDX-License-Identifier: MIT */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rx_link.h"

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static uint64_t rng = 0x243F6A8885A308D3ull;
static uint32_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; return (uint32_t)(rng >> 11); }

static void line_cases(void)
{
    struct { const char *s; int r, valid, mm; } t[] = {
        { "P 123", 1, 1, 123 }, { "P -45", 1, 1, -45 }, { "P +7", 1, 1, 7 }, { "  P   0  ", 1, 1, 0 },
        { "P !", 1, 0, 0 }, { "P  ! ", 1, 0, 0 }, { "P 2000", 1, 1, 2000 }, { "P -2000", 1, 1, -2000 },
        { "# reflex_io ready", 0, 0, 0 }, { "", 0, 0, 0 }, { "   ", 0, 0, 0 },
        { "P 2001", -1, 0, 0 }, { "P 12abc", -1, 0, 0 }, { "P", -1, 0, 0 }, { "P ", -1, 0, 0 },
        { "P1", -1, 0, 0 }, { "P 1 2", -1, 0, 0 }, { "P !x", -1, 0, 0 }, { "P --1", -1, 0, 0 },
        { "P 1234567", -1, 0, 0 }, { "Q 5", -1, 0, 0 }, { "p 5", -1, 0, 0 }, { "P 1.5", -1, 0, 0 },
        { "P -", -1, 0, 0 }, { "P 0x10", -1, 0, 0 }, { "U 100", -1, 0, 0 },
    };
    for (size_t i = 0; i < sizeof t / sizeof t[0]; i++) {
        int v = -9; int32_t mm = -9;
        int r = rx_link_parse_line(t[i].s, &v, &mm);
        CHECK(r == t[i].r, "'%s': result %d, want %d", t[i].s, r, t[i].r);
        if (r == 1) CHECK(v == t[i].valid && (!v || mm == t[i].mm), "'%s': valid=%d mm=%d", t[i].s, v, (int)mm);
    }
}

/* the same stream split at every possible point must give the same samples */
static void split_invariance(void)
{
    const char *stream = "# reflex_io ready\r\nP 10\nP !\r\ngarbage\nP -250\n\nP 7\r";
    rx_link_t ref; rx_link_init(&ref);
    int n_ref = rx_link_feed(&ref, stream, strlen(stream));
    CHECK(n_ref == 4 && ref.valid && ref.mm == 7 && ref.garbage == 1 && ref.comments == 1,
          "reference parse: n=%d valid=%d mm=%d garbage=%u", n_ref, ref.valid, (int)ref.mm, ref.garbage);
    size_t L = strlen(stream);
    for (size_t a = 0; a <= L; a++) for (size_t b = a; b <= L; b++) {
        rx_link_t l; rx_link_init(&l);
        int n = rx_link_feed(&l, stream, a) + rx_link_feed(&l, stream + a, b - a) + rx_link_feed(&l, stream + b, L - b);
        if (n != n_ref || l.mm != ref.mm || l.valid != ref.valid || l.garbage != ref.garbage) {
            CHECK(0, "split at %zu/%zu differs", a, b); return;
        }
    }
}

/* an over-long line is dropped whole: a valid-looking tail must not be parsed */
static void overflow(void)
{
    rx_link_t l; rx_link_init(&l);
    char buf[512]; memset(buf, 'x', sizeof buf);
    memcpy(buf + 400, "P 99", 4); buf[510] = '\n'; buf[511] = 0;
    int n = rx_link_feed(&l, buf, 511);
    CHECK(n == 0 && l.garbage == 1, "overlong line: n=%d garbage=%u", n, l.garbage);
    char long_p[128] = "P 5"; memset(long_p + 3, ' ', 100); strcpy(long_p + 103, "1\n");
    n = rx_link_feed(&l, long_p, strlen(long_p));
    CHECK(n == 0, "'P 5' followed by 100 spaces and '1' was accepted");
    n = rx_link_feed(&l, "P 42\n", 5);
    CHECK(n == 1 && l.mm == 42, "line after overflow not parsed");
}

/* a burst after a stall: many lines in one read, the newest wins */
static void burst(void)
{
    rx_link_t l; rx_link_init(&l);
    char buf[4096]; int k = 0;
    for (int i = 0; i < 200; i++) k += snprintf(buf + k, sizeof buf - (size_t)k, "P %d\n", i - 100);
    int n = rx_link_feed(&l, buf, (size_t)k);
    CHECK(n == 200 && l.mm == 99 && l.valid, "burst: n=%d mm=%d", n, (int)l.mm);
}

/* random bytes, with valid lines interleaved after a line break: never crash, never accept an
 * out-of-range value, and always recover the planted line */
static void fuzz(void)
{
    rx_link_t l; rx_link_init(&l);
    char buf[256];
    long planted = 0, recovered = 0, accepted = 0;
    for (int it = 0; it < 200000; it++) {
        size_t n = rnd() % sizeof buf;
        for (size_t i = 0; i < n; i++) {
            uint32_t r = rnd() % 16;
            buf[i] = r < 2 ? '\n' : r < 3 ? 'P' : r < 4 ? ' ' : r < 6 ? (char)('0' + rnd() % 10) : r < 7 ? '-' : (char)rnd();
        }
        int got = rx_link_feed(&l, buf, n);
        accepted += got;
        if (got) CHECK(!l.valid || (l.mm >= -RX_LINK_MAX_MM && l.mm <= RX_LINK_MAX_MM), "fuzz accepted mm=%d", (int)l.mm);
        if (it % 7 == 0) {
            int want = (int)(rnd() % 4001) - 2000;
            char ok[32]; int k = snprintf(ok, sizeof ok, "\nP %d\n", want);
            planted++;
            if (rx_link_feed(&l, ok, (size_t)k) >= 1 && l.valid && l.mm == want) recovered++;
        }
        if (fails > 20) break;
    }
    CHECK(recovered == planted, "fuzz: recovered %ld of %ld planted lines", recovered, planted);
    printf("  fuzz: 200000 random chunks, %ld lines accepted, %u rejected, planted lines recovered %ld/%ld\n",
           accepted, l.garbage, recovered, planted);
}

static void commands(void)
{
    char b[32];
    rx_link_cmd(b, sizeof b, 0.1234f, 230);  CHECK(!strcmp(b, "U 123\n"), "cmd 0.1234 -> '%s'", b);
    rx_link_cmd(b, sizeof b, -0.21f, 230);   CHECK(!strcmp(b, "U -210\n"), "cmd -0.21 -> '%s'", b);
    rx_link_cmd(b, sizeof b, NAN, 230);      CHECK(!strcmp(b, "U 0\n"), "cmd NaN -> '%s'", b);
    rx_link_cmd(b, sizeof b, INFINITY, 230); CHECK(!strcmp(b, "U 0\n"), "cmd Inf -> '%s'", b);
    rx_link_cmd(b, sizeof b, 40.0f, 230);    CHECK(!strcmp(b, "U 230\n"), "cmd 40 -> '%s'", b);
    rx_link_cmd(b, sizeof b, -1e30f, 230);   CHECK(!strcmp(b, "U -230\n"), "cmd -1e30 -> '%s'", b);
}

int main(void)
{
    printf("== I/O node link parser ==\n");
    line_cases(); split_invariance(); overflow(); burst(); fuzz(); commands();
    printf("%s\n", fails ? "LINKTEST FAILED" : "LINKTEST PASSED");
    return fails != 0;
}
