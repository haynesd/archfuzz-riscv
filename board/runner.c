/*
===============================================================================
File        : runner.c
Language    : C (GNU/Linux userspace)
Target      : Linux on RISC-V boards
Build       : GCC on-board
Output      : runner

PURPOSE
-------------------------------------------------------------------------------
This program runs on each target RISC-V board in the differential lab.

It waits for commands from the FPGA over UART.

Supported commands:
  1) PING
       Input : PING\n
       Input : PING\r\n
       Output: PONG\n

  2) RUN
       Input : RUN <seed_hex8> <steps_dec>\n
       Output: DONE <seed_hex8> <score_dec> <flags_hex8> <worst_window_dec> <worst_cycles_dec>\n
*/

#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

/* ============================================================================
   Workload Configuration
   ========================================================================== */

#define MEM_WORDS   (64 * 1024)
#define WINDOW_SIZE 32
#define MAX_WINDOWS 1024

/* ============================================================================
   UART Helpers
   ========================================================================== */

static void die(const char *msg) {
    perror(msg);
    exit(1);
}

static void print_visible_line(const char *tag, const char *s) {
    fputs(tag, stdout);

    for (size_t i = 0; s[i] != '\0'; i++) {
        unsigned char c = (unsigned char)s[i];

        if (c == '\n') {
            fputs("\\n", stdout);
        } else if (c == '\r') {
            fputs("\\r", stdout);
        } else if (c >= 32 && c <= 126) {
            fputc((int)c, stdout);
        } else {
            fprintf(stdout, "\\x%02X", c);
        }
    }

    fputc('\n', stdout);
    fflush(stdout);
}

static int open_uart(const char *dev) {
    int fd = open(dev, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        die("open uart");
    }

    struct termios tio;
    if (tcgetattr(fd, &tio) != 0) {
        die("tcgetattr");
    }

    cfmakeraw(&tio);

    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);

    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;

    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        die("tcsetattr");
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

static int readline_uart(int fd, char *out, size_t max) {
    size_t n = 0;

    while (n + 1 < max) {
        char c;
        int r = (int)read(fd, &c, 1);

        if (r < 0) {
            if (errno == EINTR) {
                continue;
            }
            die("read");
        }

        if (r == 0) {
            if (n == 0) {
                return 0;
            }
            continue;
        }

        out[n++] = c;
        if (c == '\n') {
            break;
        }
    }

    out[n] = '\0';
    return (int)n;
}

static void write_all(int fd, const char *s) {
    size_t len = strlen(s);

    print_visible_line("[TX] ", s);

    while (len) {
        ssize_t n = write(fd, s, len);

        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            die("write");
        }

        s   += (size_t)n;
        len -= (size_t)n;
    }
}

/* ============================================================================
   Timing Source
   ========================================================================== */

static inline uint64_t rdcycle_u64(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        die("clock_gettime");
    }
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

/* ============================================================================
   Deterministic Workload
   ========================================================================== */

static uint32_t mem[MEM_WORDS];

static inline uint32_t xorshift32(uint32_t *s) {
    uint32_t x = *s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return x;
}

typedef struct {
    uint32_t checksum;
    uint32_t flags;
    uint64_t total_cycles;
    uint32_t worst_window;
    uint64_t worst_cycles;
} run_result_t;

static run_result_t workload_windowed(uint32_t seed, int steps) {
    run_result_t rr;
    memset(&rr, 0, sizeof(rr));

    uint32_t st  = seed ^ 0xA5A5A5A5u;
    uint32_t acc = 0x12345678u;

    int n_windows = (steps + WINDOW_SIZE - 1) / WINDOW_SIZE;
    if (n_windows > MAX_WINDOWS) {
        n_windows = MAX_WINDOWS;
    }

    for (int i = 0; i < 1024; i++) {
        uint32_t r = xorshift32(&st);
        mem[r % MEM_WORDS] ^= (r + (uint32_t)i);
    }

    uint64_t total_start = rdcycle_u64();

    for (int w = 0; w < n_windows; w++) {
        int step_begin = w * WINDOW_SIZE;
        int step_end   = step_begin + WINDOW_SIZE;
        if (step_end > steps) {
            step_end = steps;
        }

        uint64_t w0 = rdcycle_u64();

        for (int i = step_begin; i < step_end; i++) {
            uint32_t r = xorshift32(&st);

            acc ^= (r * 2654435761u);
            acc += (acc << 7) ^ (r >> 3);
            acc = (acc << 3) | (acc >> 29);

            uint32_t idx = (r ^ acc) % MEM_WORDS;
            uint32_t v   = mem[idx];
            v ^= (acc + r);
            v += (v << 11) ^ (v >> 9);
            mem[idx] = v;

            acc ^= mem[(idx + (acc & 1023)) % MEM_WORDS];

            if ((r & 0x3FF) == 0x155) {
                acc ^= 0xDEADBEEFu;
            }
        }

        uint64_t w1 = rdcycle_u64();
        uint64_t wcycles = w1 - w0;

        if (wcycles > rr.worst_cycles) {
            rr.worst_cycles = wcycles;
            rr.worst_window = (uint32_t)w;
        }
    }

    uint64_t total_end = rdcycle_u64();

    rr.total_cycles = total_end - total_start;
    rr.checksum     = acc;
    rr.flags        = 0;

    return rr;
}

/* ============================================================================
   Main
   ========================================================================== */

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <uart_device>\n", argv[0]);
        fprintf(stderr, "Example: %s /dev/ttyS1\n", argv[0]);
        return 2;
    }

    int fd = open_uart(argv[1]);
    char line[256];

    printf("[INFO] runner started on %s\n", argv[1]);
    fflush(stdout);

    for (;;) {
        int n = readline_uart(fd, line, sizeof(line));
        if (n == 0) {
            continue;
        }

        print_visible_line("[RX] ", line);

        if (strcmp(line, "PING\n") == 0 || strcmp(line, "PING\r\n") == 0) {
            write_all(fd, "PONG\n");
            continue;
        }

        unsigned seed = 0;
        int steps = 0;

        if (sscanf(line, "RUN %x %d", &seed, &steps) != 2) {
            printf("[INFO] ignored unrecognized command\n");
            fflush(stdout);
            continue;
        }

        if (steps < 1) {
            steps = 1;
        }

        if (steps > WINDOW_SIZE * MAX_WINDOWS) {
            steps = WINDOW_SIZE * MAX_WINDOWS;
        }

        printf("[INFO] running workload seed=%08X steps=%d\n",
               (uint32_t)seed, steps);
        fflush(stdout);

        run_result_t rr = workload_windowed((uint32_t)seed, steps);

        uint64_t score = rr.total_cycles ^ (uint64_t)rr.checksum;

        char out[256];
        snprintf(
            out,
            sizeof(out),
            "DONE %08X %" PRIu64 " %08X %u %" PRIu64 "\n",
            (uint32_t)seed,
            (uint64_t)score,
            (uint32_t)rr.flags,
            rr.worst_window,
            rr.worst_cycles
        );

        write_all(fd, out);
    }

    return 0;
}