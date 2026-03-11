/*
===============================================================================
File        : runner_windowed.c
Language    : C (GNU/Linux userspace)
Target      : Linux on RISC-V boards
Build       : GCC on-board
Output      : runner

PURPOSE
-------------------------------------------------------------------------------
This program runs on each target RISC-V board in the differential lab.

It waits for a command from the FPGA:

  RUN <seed_hex8> <steps_dec>

Then it executes a deterministic synthetic workload driven by the seed and
measures timing using rdcycle. The execution is partitioned into fixed-size
windows so the runner can localize the timing hotspot that produced the
largest cycle cost.

The runner returns:

  DONE <seed_hex8> <score_dec> <flags_hex8> <worst_window_dec> <worst_cycles_dec>

where:
  score         = total_cycles XOR checksum
  flags         = anomaly flags (reserved for future expansion)
  worst_window  = window index with highest cycle cost
  worst_cycles  = cycle count of the worst window

This provides both:
  - whole-run divergence information
  - sequence-localized anomaly information

ASCII PROTOCOL
-------------------------------------------------------------------------------
Input:
  RUN 00112233 256\n

Output:
  DONE 00112233 123456789 00000000 5 1842\n

BUILD
-------------------------------------------------------------------------------
On each Linux board:
  gcc -O2 -Wall -Wextra -o runner runner_windowed.c

RUN
-------------------------------------------------------------------------------
  ./runner /dev/ttyS1

Replace /dev/ttyS1 with the UART device connected to the FPGA.

NOTES
-------------------------------------------------------------------------------
- This code is intended for Linux-based boards such as:
    - VisionFive2
    - Orange Pi RV2
    - Lichee RV variants with Linux userspace
- The timing source is rdcycle on RISC-V.
- On RV64, rdcycle is read directly.
- On RV32, rdcycleh/rdcycle is used to build a consistent 64-bit value.
- The workload is synthetic and deterministic; it is not intended to emulate
  a particular ISA test suite, only to create a reproducible instruction mix
  with memory interaction.
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

/* ============================================================================
   Workload Configuration
   ========================================================================== */

#define MEM_WORDS   (64 * 1024)   /* 256 KB scratch region */
#define WINDOW_SIZE 32            /* steps per timing window */
#define MAX_WINDOWS 1024          /* hard cap on windows */

/* ============================================================================
   UART Helpers
   ========================================================================== */

/* --------------------------------------------------------------------------
   die
   --------------------------------------------------------------------------
   Print perror and terminate.
   -------------------------------------------------------------------------- */
static void die(const char *msg) {
    perror(msg);
    exit(1);
}

/* --------------------------------------------------------------------------
   open_uart
   --------------------------------------------------------------------------
   Open and configure a UART device in raw 115200 8N1 mode.
   -------------------------------------------------------------------------- */
static int open_uart(const char *dev) {
    int fd = open(dev, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        die("open uart");
    }

    struct termios tio;
    memset(&tio, 0, sizeof(tio));

    if (tcgetattr(fd, &tio) != 0) {
        die("tcgetattr");
    }

    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);

    tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS8;
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~(PARENB | PARODD);
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CRTSCTS;

    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_lflag = 0;
    tio.c_oflag = 0;

    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 1;   /* 100 ms */

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        die("tcsetattr");
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

/* --------------------------------------------------------------------------
   readline_uart
   --------------------------------------------------------------------------
   Read one line from UART.
   Returns:
     0  -> timeout slice / no full line yet
     >0 -> bytes read
   -------------------------------------------------------------------------- */
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

    out[n] = 0;
    return (int)n;
}

/* --------------------------------------------------------------------------
   write_all
   --------------------------------------------------------------------------
   Write the full string to UART.
   -------------------------------------------------------------------------- */
static void write_all(int fd, const char *s) {
    size_t len = strlen(s);

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

/* --------------------------------------------------------------------------
   rdcycle_u64
   --------------------------------------------------------------------------
   Read the RISC-V cycle counter as a 64-bit value.

   RV64:
     rdcycle directly

   RV32:
     combine rdcycleh / rdcycle / rdcycleh until stable
   -------------------------------------------------------------------------- */
static inline uint64_t rdcycle_u64(void) {
#if defined(__riscv) && (__riscv_xlen == 64)
    uint64_t x;
    asm volatile ("rdcycle %0" : "=r"(x));
    return x;
#elif defined(__riscv)
    uint32_t hi0, lo, hi1;
    do {
        asm volatile ("rdcycleh %0" : "=r"(hi0));
        asm volatile ("rdcycle %0"  : "=r"(lo));
        asm volatile ("rdcycleh %0" : "=r"(hi1));
    } while (hi0 != hi1);
    return ((uint64_t)hi0 << 32) | lo;
#else
    /* Non-RISC-V fallback for development only */
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
#endif
}

/* ============================================================================
   Deterministic Workload
   ========================================================================== */

static uint32_t mem[MEM_WORDS];

/* --------------------------------------------------------------------------
   xorshift32
   --------------------------------------------------------------------------
   Deterministic pseudorandom generator.
   -------------------------------------------------------------------------- */
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

/* --------------------------------------------------------------------------
   workload_windowed
   --------------------------------------------------------------------------
   Execute a deterministic workload for 'steps' iterations.

   Timing is measured per fixed-size window:
     window 0 = steps 0..31
     window 1 = steps 32..63
     ...

   Returns:
     - final checksum
     - total cycles
     - worst window index
     - worst window cycles

   Notes:
     - This workload deliberately mixes:
         * arithmetic
         * rotates/shifts
         * memory reads/writes
         * data-dependent addressing
         * occasional branch-like path variation
     - The goal is not functional correctness of an application, but a stable,
       reproducible instruction/memory mix for differential analysis.
   -------------------------------------------------------------------------- */
static run_result_t workload_windowed(uint32_t seed, int steps) {
    run_result_t rr;
    memset(&rr, 0, sizeof(rr));

    uint32_t st  = seed ^ 0xA5A5A5A5u;
    uint32_t acc = 0x12345678u;

    int n_windows = (steps + WINDOW_SIZE - 1) / WINDOW_SIZE;
    if (n_windows > MAX_WINDOWS) {
        n_windows = MAX_WINDOWS;
    }

    /* Small deterministic warm-up initialization */
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

            /* ALU-heavy section */
            acc ^= (r * 2654435761u);
            acc += (acc << 7) ^ (r >> 3);
            acc = (acc << 3) | (acc >> 29);

            /* Memory section */
            uint32_t idx = (r ^ acc) % MEM_WORDS;
            uint32_t v   = mem[idx];
            v ^= (acc + r);
            v += (v << 11) ^ (v >> 9);
            mem[idx] = v;

            /* Dependent access to amplify microarchitectural sensitivity */
            acc ^= mem[(idx + (acc & 1023)) % MEM_WORDS];

            /* Occasional branch-ish perturbation */
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
    rr.flags        = 0; /* Reserved for future trap/fault encoding */

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

    for (;;) {
        int n = readline_uart(fd, line, sizeof(line));
        if (n == 0) {
            continue;
        }

        /*
        Expected command:
          RUN <seed_hex8> <steps_dec>
        */
        unsigned seed = 0;
        int steps = 0;

        if (sscanf(line, "RUN %x %d", &seed, &steps) != 2) {
            continue;
        }

        if (steps < 1) {
            steps = 1;
        }

        if (steps > WINDOW_SIZE * MAX_WINDOWS) {
            steps = WINDOW_SIZE * MAX_WINDOWS;
        }

        run_result_t rr = workload_windowed((uint32_t)seed, steps);

        /*
        Score definition:
          score = total_cycles XOR checksum

        This mixes:
          - architectural output proxy (checksum)
          - timing behavior (total cycles)
        */
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