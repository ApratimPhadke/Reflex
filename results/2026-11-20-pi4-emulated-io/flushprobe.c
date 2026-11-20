// SPDX-License-Identifier: MIT
/* throwaway probe: does tcflush(TCIFLUSH) discard devc-serusb's buffered input on QNX?
 * usage: flushprobe DEV 0|1   (1 = flush after open). Opens like rx_io.c, waits 30 ms, reads once. */
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

int main(int argc, char **argv)
{
    if (argc < 3) return 2;
    int fd = open(argv[1], O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { perror("open"); return 1; }
    struct termios t;
    tcgetattr(fd, &t); cfmakeraw(&t); cfsetispeed(&t, B115200); cfsetospeed(&t, B115200);
    t.c_cflag |= CLOCAL | CREAD; tcsetattr(fd, TCSANOW, &t);
    int rc = 0;
    if (atoi(argv[2])) rc = tcflush(fd, TCIFLUSH);
    struct timespec nap = { 0, 30 * 1000000 };
    nanosleep(&nap, NULL);
    char buf[8192];
    ssize_t n = read(fd, buf, sizeof buf);
    int lines = 0, bang = 0;
    for (ssize_t i = 0; i < n; i++) if (buf[i] == '\n') { lines++; if (i >= 3 && buf[i-2] == '!') bang++; }
    printf("flush=%s rc=%d bytes=%zd lines=%d bang=%d first=%.12s\n", argv[2], rc, n, lines, bang, n > 0 ? buf : "");
    close(fd);
    return 0;
}
