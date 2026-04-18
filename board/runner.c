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

STEPS (steps_dec)
-------------------------------------------------------------------------------
The 'steps' parameter defines how many iterations of the workload will be 
executed for a given seed. 

The workload is divided into fixed-size windows so timing issues can be
localized.

Each step represents one full pass through the workload loop, which includes:
  - ALU operations (integer math, shifts, mixing)
  - Memory accesses (loads, stores, data-dependent indexing)
  - Atomic operations (RISC-V AMO instructions)
  - Control-flow variation (branch-like behavior)

Steps control the *duration and depth* of a test case.

In the differential fuzzing system:
  - The same (seed, steps) pair is executed on multiple boards.
  - Results are compared to detect architectural differences.
  - Increasing steps increases the opportunity for divergence.

STEPS - SO WHAT?
-------------------------------------------------------------------------------
Small step counts:
  - Faster execution
  - Good for quick scanning of many seeds
  - Lower chance of exposing subtle timing or state differences

Large step counts:
  - Longer execution time
  - More stress on memory, pipelines, and atomic units
  - Higher chance of exposing:
      - timing differences
      - cache/memory effects
      - ordering/atomic behavior differences
      - rare control-flow interactions

STEPS - WINDOWING RELATIONSHIP
-------------------------------------------------------------------------------
The workload is divided into fixed-size windows (WINDOW_SIZE).

steps determines:
  total_windows = ceil(steps / WINDOW_SIZE)

Each window is timed independently to identify the "worst" region of execution.
This allows the system to detect not just total slowdown, but *where* it occurs.

HOW TO USE STEPS
-------------------------------------------------------------------------------
Typical usage patterns:

  Exploration phase:
    - Use smaller step counts (e.g., 64–512)
    - Scan large ranges of seeds quickly

  Deep analysis phase:
    - Use larger step counts (e.g., 1024–10000+)
    - Focus on seeds that already show divergence



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

/* ----------------------------------------------------------------------------
   workload_windowed
   ----------------------------------------------------------------------------
   This function is the core synthetic workload used by the fuzzing runner compiled
   and ran on each target board.

   The goal is to generate a deterministic pattern that can expose architectural
   differences across boards.

   This function turns one seed into one reproducible stress pattern, and that
   stress pattern is used to find architectural differences through repeated,
   comparable execution across multiple boards.

   Architectual Differential Fuzzing Process (ADFP):
     1. The same seed and step count are sent to multiple boards.
     2. Each board executes the same intended workload.
     3. If the boards differ in timing, atomic behavior, memory behavior,
        instruction implementation, or platform effects, their measured results
        may diverge.
     4. The host compares those results and scores seeds that produce the most
        interesting differences.

   Workload structure:
     - ALU-heavy math stresses arithmetic datapaths and compiler codegen.
     - Memory loads/stores to stress caches, buses, and memory subsystems.
     - Data-dependent indexing makes access patterns harder to predict.
     - Atomic AMO operations stress ordering and architectural support for the
       RISC-V A-extension.
     - Occasional branch-like alterations to create control-flow variation.
     - Windowed timing identifies *where* in the run the highest timing cost
       occurred, not just the total time.

   Function results:
     - checksum      : a deterministic summary of the computed data path
     - total_cycles  : total run time (using monotonic clock) for the entire workload
     - worst_window  : which timing window was slowest
     - worst_cycles  : how slow that worst window was
     - flags         : reserved for future anomaly markers
   ---------------------------------------------------------------------------- */
static run_result_t workload_windowed(uint32_t seed, int steps) {
    run_result_t rr;                                  /* Create the result struct that will hold checksum, timing, and window info. */
    memset(&rr, 0, sizeof(rr));                      /* Start with all result fields cleared so defaults are known and safe. */

    uint32_t st  = seed ^ 0xA5A5A5A5u;               /* Initialize the pseudo-random state from the input seed, but mix it first so small seed patterns do not map too directly into the generator state. */
    uint32_t acc = 0x12345678u;                      /* Initialize the running accumulator/checksum state with a non-zero constant so the workload starts from a fixed, known baseline. */

    int n_windows = (steps + WINDOW_SIZE - 1) / WINDOW_SIZE; /* Compute how many fixed-size timing windows are needed to cover the requested step count, rounding up for partial final windows. */
    if (n_windows > MAX_WINDOWS) {                   /* Guard against excessive window counts so the workload stays inside the designed analysis limits. */
        n_windows = MAX_WINDOWS;                     /* Limit the number of windows to the maximum supported value. */
    }

    for (int i = 0; i < 1024; i++) {                 /* Run a deterministic warm-up loop to initialize scratch memory into a seed-dependent state before timing the main workload. */
        uint32_t r = xorshift32(&st);                /* Advance the deterministic pseudo-random generator to get the next workload-driving value. */
        mem[r % MEM_WORDS] ^= (r + (uint32_t)i);     /* Touch memory at a pseudo-random location and perturb it so later operations depend on a nontrivial, seed-specific memory state. */
    }

    uint64_t total_start = monotonic_ticks_u64();    /* Record the start time of the full workload so total execution cost can be measured. */

    for (int w = 0; w < n_windows; w++) {            /* Iterate over timing windows so the run can be analyzed in smaller regions, not only as one total block. */
        int step_begin = w * WINDOW_SIZE;            /* Compute the first step index belonging to this window. */
        int step_end   = step_begin + WINDOW_SIZE;   /* Compute the nominal end step index for this window. */
        if (step_end > steps) {                      /* Check whether the last window would run past the requested number of steps. */
            step_end = steps;                        /* Trim the final window so the total number of executed steps exactly matches the request. */
        }

        uint64_t w0 = monotonic_ticks_u64();         /* Record the start time of this window so its local timing cost can be measured. */

        for (int i = step_begin; i < step_end; i++) { /* Execute each step in this timing window. */
            uint32_t r = xorshift32(&st);            /* Generate the next deterministic pseudo-random value that drives this step’s behavior. */

            /* ALU-heavy section */
            acc ^= (r * 2654435761u);                /* Mix the random value into the accumulator with multiplication and XOR to exercise integer datapaths and create nontrivial data evolution. */
            acc += (acc << 7) ^ (r >> 3);            /* Combine shifts, XOR, and addition to create more varied arithmetic pressure and data dependencies. */
            acc = (acc << 3) | (acc >> 29);          /* Perform a rotate-like operation so bits move across positions and the accumulator remains highly mixed. */

            /* Normal memory section */
            uint32_t idx = (r ^ acc) % MEM_WORDS;    /* Choose a memory index based on both current pseudo-random state and accumulator state, making accesses data-dependent and less predictable. */
            uint32_t v   = mem[idx];                 /* Load the current value from the selected scratch-memory location. */

            v ^= (acc + r);                          /* Perturb the loaded value with current arithmetic state so memory contents evolve with execution history. */
            v += (v << 11) ^ (v >> 9);               /* Apply more local arithmetic mixing to amplify differences in data patterns and instruction mix. */
            mem[idx] = v;                            /* Store the updated value back into memory, creating a read-modify-write memory access pattern. */

            acc ^= mem[(idx + (acc & 1023u)) % MEM_WORDS]; /* Perform a second dependent memory read using both the current index and low bits of the accumulator, making later behavior depend on earlier state and memory contents. */

            /* Atomic stress section using explicit RISC-V AMO operations */
            if ((r & 0x1Fu) == 0u) {                 /* Only execute the atomic stress block occasionally so it influences the workload without completely dominating every step. */
                volatile uint32_t *aptr = &mem[(idx + 17u) % MEM_WORDS]; /* Select a nearby scratch-memory word for atomic operations; mark the pointer volatile so the explicit AMO memory side effects are preserved as intended. */

                uint32_t old_add  = amoadd_w(aptr, (r | 1u));           /* Atomically add a pseudo-random odd value and capture the old memory value; this stresses arithmetic AMO behavior. */
                uint32_t old_xor  = amoxor_w(aptr, acc);                /* Atomically XOR the accumulator into memory and capture the previous value; this stresses bitwise AMO behavior. */
                uint32_t old_and  = amoand_w(aptr, ~r);                 /* Atomically AND memory with the inverse of the random value and capture the previous value. */
                uint32_t old_or   = amoor_w(aptr, (acc | 1u));          /* Atomically OR memory with the accumulator and capture the previous value. */
                uint32_t old_swap = amoswap_w(aptr, r ^ acc);           /* Atomically replace memory entirely and capture the previous value, stressing exchange semantics. */

                uint32_t old_min  = amomin_w(aptr, r);                  /* Atomically perform a signed minimum update and capture the previous value. */
                uint32_t old_max  = amomax_w(aptr, acc);                /* Atomically perform a signed maximum update and capture the previous value. */
                uint32_t old_umin = amominu_w(aptr, r ^ 0xAAAAAAAAu);   /* Atomically perform an unsigned minimum update and capture the previous value. */
                uint32_t old_umax = amomaxu_w(aptr, acc ^ 0x55555555u); /* Atomically perform an unsigned maximum update and capture the previous value. */

                acc ^= old_add;                         /* Fold the old result of the atomic add into the accumulator so the final checksum depends on AMO behavior. */
                acc += old_xor;                         /* Fold the old result of the atomic XOR into the accumulator. */
                acc ^= old_and;                         /* Fold the old result of the atomic AND into the accumulator. */
                acc += old_or;                          /* Fold the old result of the atomic OR into the accumulator. */
                acc ^= old_swap;                        /* Fold the old result of the atomic swap into the accumulator. */
                acc += old_min;                         /* Fold the old result of the signed min AMO into the accumulator. */
                acc ^= old_max;                         /* Fold the old result of the signed max AMO into the accumulator. */
                acc += old_umin;                        /* Fold the old result of the unsigned min AMO into the accumulator. */
                acc ^= old_umax;                        /* Fold the old result of the unsigned max AMO into the accumulator. */
            }

            /* Occasional branch-like perturbation */
            if ((r & 0x3FFu) == 0x155u) {              /* Occasionally take an alternate path based on the pseudo-random pattern so the control-flow profile is not completely uniform. */
                acc ^= 0xDEADBEEFu;                    /* Perturb the accumulator strongly when that rare condition is met, making branch timing and path differences visible in results. */
            }
        }

        uint64_t w1 = monotonic_ticks_u64();          /* Record the end time of the current window. */
        uint64_t wcycles = w1 - w0;                   /* Compute the elapsed time for just this window. */

        if (wcycles > rr.worst_cycles) {              /* Check whether this window is the slowest one seen so far. */
            rr.worst_cycles = wcycles;                /* Save the timing of the slowest window so far. */
            rr.worst_window = (uint32_t)w;            /* Save which window index produced that worst timing. */
        }
    }

    uint64_t total_end = monotonic_ticks_u64();       /* Record the end time of the full workload. */

    rr.total_cycles = total_end - total_start;        /* Save the total run time so the host can compare overall timing behavior across boards. */
    rr.checksum     = acc;                            /* Save the final accumulator as the deterministic data-path summary for this seed/run. */
    rr.flags        = 0;                              /* Leave flags clear for now; TBD */

    return rr;                                        /* Return the completed result structure to the caller so it can be formatted into the DONE line. */
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