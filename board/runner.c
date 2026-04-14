/*
===============================================================================
File        : runner.c
Language    : C (GNU/Linux userspace)
Target      : Linux on RISC-V boards
Build       : GCC on-board
Output      : runner

TITLE
-------------------------------------------------------------------------------
Deterministic Board Runner for FPGA Differential Fuzzing Lab

PURPOSE
-------------------------------------------------------------------------------
This program runs on each target RISC-V board in the differential fuzzing lab.

It listens for newline-terminated ASCII commands from the FPGA over UART and
returns newline-terminated ASCII responses.

Supported commands:
  1) PING
       Input : PING\n
       Input : PING\r\n
       Output: PONG\n

  2) RUN
       Input : RUN <seed_dec> <steps_dec>\n
       Output: DONE <seed_dec> <score_dec> <flags_dec> <worst_window_dec> <worst_cycles_dec>\n

The RUN command executes a deterministic synthetic workload driven by the seed.
The workload is divided into fixed-size windows so timing hotspots can be
localized.

SEED 
-------------------------------------------------------------------------------
A seed random number that controls how the test behaves.

Same seed:
  - same generated operation sequence
  - same memory access pattern
  - same workload structure

Different seed:
  - different deterministic test pattern

In practice:
  seed = one reproducible test case

ATOMIC OPERATIONS 
-------------------------------------------------------------------------------
Atomic Operations are used to better expose differences across architectures and 
implementations, this runner includes explicit RISC-V atomic memory operations in 
the workload.

These operations can amplify differences in:
  - memory subsystem timing
  - atomic instruction implementation
  - cache / coherence behavior
  - ordering overhead
  - compiler / ISA support differences

This makes the workload more useful for differential analysis than a purely
non-atomic arithmetic loop.

LOGGING
-------------------------------------------------------------------------------
This runner logs to the local board console:
  - [RX] UART line
  - [TX] UART line
  - workload start information
  - human-readable DONE summary

Example console output:
  [INFO] runner started on /dev/ttyS1
  [RX] PING\n
  [TX] PONG\n
  [RX] RUN 12345 256\n
  [RUN] seed=12345 steps=256
  [DONE] seed=12345 score=987654321 flags=0 worst_window=5 worst_cycles=1842
  [TX] DONE 12345 987654321 0 5 1842\n

BUILD (on-board)
-------------------------------------------------------------------------------
On each Linux board:
  gcc -O2 -Wall -Wextra -std=gnu11 -o runner runner.c

Optional explicit ISA build if needed:
  gcc -O2 -Wall -Wextra -std=gnu11 -march=rv64gc -mabi=lp64d -o runner runner.c

RUN
-------------------------------------------------------------------------------
  ./runner /dev/ttyS0

Replace /dev/ttyS0 with the UART device connected to the FPGA.

===============================================================================
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

//============================================================================
//   Workload Configuration
//============================================================================

#define MEM_WORDS   (64 * 1024)
#define WINDOW_SIZE 32
#define MAX_WINDOWS 1024

//============================================================================
//   UART Helpers
//============================================================================

static void die(const char *msg) {
    perror(msg);
    exit(1);
}

//--------------------------------------------------------------------------
//   print_visible_line
//--------------------------------------------------------------------------
//   Print a tagged string to the local console and render CR/LF visibly.
//   Example:
//     [RX] PING\n
//     [TX] PONG\n
//-------------------------------------------------------------------------- 
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

//--------------------------------------------------------------------------
//   open_uart
//--------------------------------------------------------------------------
//   Open and configure a UART device in raw 115200 8N1 mode.
//-------------------------------------------------------------------------- 
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

//============================================================================
//   readline_uart
//============================================================================
//   Read one newline-terminated line from UART.

//   Returns:
//     0  -> timeout slice / no full line yet
//     0  -> bytes read
//----------------------------------------------------------------------------
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

//============================================================================
//   write_all
//============================================================================
//   Write the full string to UART and log it to the local console.
//============================================================================ */
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

//============================================================================
//   Timing Source
//============================================================================

static inline uint64_t monotonic_ticks_u64(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        die("clock_gettime");
    }

    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

//============================================================================
//   RISC-V AMO Helpers
//============================================================================ 

#if defined(__riscv)

/* Arithmetic atomic update */
static inline uint32_t amoadd_w(volatile uint32_t *p, uint32_t val) {
    uint32_t old;
    asm volatile (
        "amoadd.w %0, %2, (%1)"
        : "=r"(old)
        : "r"(p), "r"(val)
        : "memory"
    );
    return old;
}

/* Bitwise atomic updates */
static inline uint32_t amoxor_w(volatile uint32_t *p, uint32_t val) {
    uint32_t old;
    asm volatile (
        "amoxor.w %0, %2, (%1)"
        : "=r"(old)
        : "r"(p), "r"(val)
        : "memory"
    );
    return old;
}

static inline uint32_t amoand_w(volatile uint32_t *p, uint32_t val) {
    uint32_t old;
    asm volatile (
        "amoand.w %0, %2, (%1)"
        : "=r"(old)
        : "r"(p), "r"(val)
        : "memory"
    );
    return old;
}

static inline uint32_t amoor_w(volatile uint32_t *p, uint32_t val) {
    uint32_t old;
    asm volatile (
        "amoor.w %0, %2, (%1)"
        : "=r"(old)
        : "r"(p), "r"(val)
        : "memory"
    );
    return old;
}

/* Exchange */
static inline uint32_t amoswap_w(volatile uint32_t *p, uint32_t val) {
    uint32_t old;
    asm volatile (
        "amoswap.w %0, %2, (%1)"
        : "=r"(old)
        : "r"(p), "r"(val)
        : "memory"
    );
    return old;
}

/* Signed comparison AMOs */
static inline uint32_t amomin_w(volatile uint32_t *p, uint32_t val) {
    uint32_t old;
    asm volatile (
        "amomin.w %0, %2, (%1)"
        : "=r"(old)
        : "r"(p), "r"(val)
        : "memory"
    );
    return old;
}

static inline uint32_t amomax_w(volatile uint32_t *p, uint32_t val) {
    uint32_t old;
    asm volatile (
        "amomax.w %0, %2, (%1)"
        : "=r"(old)
        : "r"(p), "r"(val)
        : "memory"
    );
    return old;
}

/* Unsigned comparison AMOs */
static inline uint32_t amomin_w(volatile uint32_t *p, uint32_t val) {
    uint32_t old;
    asm volatile (
        "amominu.w %0, %2, (%1)"
        : "=r"(old)
        : "r"(p), "r"(val)
        : "memory"
    );
    return old;
}

static inline uint32_t amomax_w(volatile uint32_t *p, uint32_t val) {
    uint32_t old;
    asm volatile (
        "amomaxu.w %0, %2, (%1)"
        : "=r"(old)
        : "r"(p), "r"(val)
        : "memory"
    );
    return old;
}

#else

/* Non-RISC-V fallback stubs for development only */
static inline uint32_t amoadd_w(volatile uint32_t *p, uint32_t val) {
    uint32_t old = *p; *p = old + val; return old;
}
static inline uint32_t amoxor_w(volatile uint32_t *p, uint32_t val) {
    uint32_t old = *p; *p = old ^ val; return old;
}
static inline uint32_t amoand_w(volatile uint32_t *p, uint32_t val) {
    uint32_t old = *p; *p = old & val; return old;
}
static inline uint32_t amoor_w(volatile uint32_t *p, uint32_t val) {
    uint32_t old = *p; *p = old | val; return old;
}
static inline uint32_t amoswap_w(volatile uint32_t *p, uint32_t val) {
    uint32_t old = *p; *p = val; return old;
}
static inline uint32_t amomin_w(volatile uint32_t *p, uint32_t val) {
    uint32_t old = *p; *p = ((int32_t)val < (int32_t)old) ? val : old; return old;
}
static inline uint32_t amomax_w(volatile uint32_t *p, uint32_t val) {
    uint32_t old = *p; *p = ((int32_t)val > (int32_t)old) ? val : old; return old;
}
static inline uint32_t amominu_w(volatile uint32_t *p, uint32_t val) {
    uint32_t old = *p; *p = (val < old) ? val : old; return old;
}
static inline uint32_t amomaxu_w(volatile uint32_t *p, uint32_t val) {
    uint32_t old = *p; *p = (val > old) ? val : old; return old;
}

#endif

//============================================================================
//   Deterministic Workload
//============================================================================

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

//--------------------------------------------------------------------------
//   workload_windowed
//--------------------------------------------------------------------------
//   Execute a deterministic synthetic workload for 'steps' iterations.
//-------------------------------------------------------------------------- 
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

    uint64_t total_start = monotonic_ticks_u64();

    for (int w = 0; w < n_windows; w++) {
        int step_begin = w * WINDOW_SIZE;
        int step_end   = step_begin + WINDOW_SIZE;
        if (step_end > steps) {
            step_end = steps;
        }

        uint64_t w0 = monotonic_ticks_u64();

        for (int i = step_begin; i < step_end; i++) {
            uint32_t r = xorshift32(&st);

            /* ALU-heavy section */
            acc ^= (r * 2654435761u);
            acc += (acc << 7) ^ (r >> 3);
            acc = (acc << 3) | (acc >> 29);

            /* Normal memory section */
            uint32_t idx = (r ^ acc) % MEM_WORDS;
            uint32_t v   = mem[idx];

            v ^= (acc + r);
            v += (v << 11) ^ (v >> 9);
            mem[idx] = v;

            acc ^= mem[(idx + (acc & 1023u)) % MEM_WORDS];

            /* Atomic stress section using explicit RISC-V AMO operations */
            if ((r & 0x1Fu) == 0u) {
                volatile uint32_t *aptr = &mem[(idx + 17u) % MEM_WORDS];

                uint32_t old_add  = amoadd_w(aptr, (r | 1u));
                uint32_t old_xor  = amoxor_w(aptr, acc);
                uint32_t old_and  = amoand_w(aptr, ~r);
                uint32_t old_or   = amoor_w(aptr, (acc | 1u));
                uint32_t old_swap = amoswap_w(aptr, r ^ acc);

                uint32_t old_min  = amomin_w(aptr, r);
                uint32_t old_max  = amomax_w(aptr, acc);
                uint32_t old_umin = amominu_w(aptr, r ^ 0xAAAAAAAAu);
                uint32_t old_umax = amomaxu_w(aptr, acc ^ 0x55555555u);

                acc ^= old_add;
                acc += old_xor;
                acc ^= old_and;
                acc += old_or;
                acc ^= old_swap;
                acc += old_min;
                acc ^= old_max;
                acc += old_umin;
                acc ^= old_umax;
            }

            /* Occasional branch-like perturbation */
            if ((r & 0x3FFu) == 0x155u) {
                acc ^= 0xDEADBEEFu;
            }
        }

        uint64_t w1 = monotonic_ticks_u64();
        uint64_t wcycles = w1 - w0;

        if (wcycles > rr.worst_cycles) {
            rr.worst_cycles = wcycles;
            rr.worst_window = (uint32_t)w;
        }
    }

    uint64_t total_end = monotonic_ticks_u64();

    rr.total_cycles = total_end - total_start;
    rr.checksum     = acc;
    rr.flags        = 0;

    return rr;
}

//============================================================================
//  Main
//============================================================================

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

        /* PING / PONG path */
        if (strcmp(line, "PING\n") == 0 || strcmp(line, "PING\r\n") == 0) {
            write_all(fd, "PONG\n");
            continue;
        }

        /* RUN command: RUN <seed_dec> <steps_dec>\n */
        unsigned seed = 0;
        int steps = 0;

        if (sscanf(line, "RUN %u %d", &seed, &steps) != 2) {
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

        printf("[RUN] seed=%u steps=%d\n", (uint32_t)seed, steps);
        fflush(stdout);

        run_result_t rr = workload_windowed((uint32_t)seed, steps);

        uint64_t score = rr.total_cycles ^ (uint64_t)rr.checksum;

        /* Human-readable local console summary */
        printf(
            "[DONE] seed=%u score=%" PRIu64
            " flags=%u worst_window=%u worst_cycles=%" PRIu64 "\n",
            (uint32_t)seed,
            (uint64_t)score,
            (uint32_t)rr.flags,
            rr.worst_window,
            rr.worst_cycles
        );
        fflush(stdout);

        /* Machine-readable protocol line back to FPGA/host */
        char out[256];
        snprintf(
            out,
            sizeof(out),
            "DONE %u %" PRIu64 " %u %u %" PRIu64 "\n",
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