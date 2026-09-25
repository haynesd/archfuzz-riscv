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
       Input : RUN <seed_dec> <steps_dec> [diag_mode_dec]\n
       Output: DONE <seed_dec> <checksum_dec> <total_ns_dec> <flags_dec> <worst_window_dec> <worst_ns_dec>\n

checksum and total_ns are reported as separate fields (not combined) so the
host can tell architectural/correctness divergence (checksum) apart from
pure clock-speed/timing divergence (total_ns) instead of conflating both
into one number.

diag_mode is an optional third field, defaulting to 0 (the normal, full
workload - identical to always omitting it) when absent, for manual
bisection of a cross-board checksum divergence:
  0 = full workload (default): ALU + memory + AMO(1/32 steps) + branch(1/1024 steps)
  1 = ALU-only: only the unconditional multiply/shift/rotate mixing lines run;
      memory section, AMO block, and branch perturbation are all skipped
  2 = ALU + memory, no AMO/branch: adds the unconditional load/modify/store
      memory section back on top of mode 1
  3 = ALU + memory + AMO, no branch: adds the AMO block (1/32 steps) back on
      top of mode 2, still without the branch perturbation, to tell AMO and
      branch-perturbation apart as the last two candidates once modes 1/2
      have both matched across boards
Only rl_host.exe's run1 mode can send diag_mode != 0; the rl/rl_scope
bandit loop always sends 0 (or omits it), so a live campaign's results are
unaffected by this field's existence.

Investigation note (2026-09-25): this tooling was built to bisect a
cross-board checksum divergence where board 0 disagreed with boards 1/2 on
essentially every run in a multi-day campaign (overnight_divergence.csv,
944782 rows, 100% divergence). Modes 1/2/3 each matched cleanly across all
three boards for known-divergent (seed, steps) pairs, which was the first
sign something was off - a real per-instruction architectural difference
should have shown up in at least one isolated mode. A fresh, carefully
verified redeploy to all three boards (confirmed via systemd's
"active (running) since Xms ago" on every board, not just a successful scp)
then reproduced 0 divergence across 347 fresh iterations, mode 0 included,
on the exact (seed, steps) pairs that had previously diverged 100% of the
time. Conclusion: the original divergence was board 0 silently running
stale/different code during the campaign (the same class of bug as the
ExecStart path mismatch caught earlier), not a genuine SpacemiT K1 defect.
The diag_mode infrastructure is kept because it's useful diagnostic tooling
in general, not because there's a currently open divergence to explain.

flags is 0 for a clean run, or a bitwise-OR of FLAG_SIGILL (0x1) /
FLAG_SIGSEGV (0x2) / FLAG_SIGBUS (0x4) / FLAG_SIGFPE (0x8) if the board
raised a genuine hardware fault partway through the workload - in that case
checksum/total_ns/worst_window/worst_ns are all 0 since the run did not
complete. See the "Fault Detection" section below for why/how.

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
  [DONE] seed=12345 checksum=987654321 total_ns=48213 flags=0 worst_window=5 worst_ns=1842
  [TX] DONE 12345 987654321 48213 0 5 1842\n

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
#include <setjmp.h>
#include <signal.h>
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
//   Fault Detection
//============================================================================
//   The workload only ever performs bounds-checked memory access and valid
//   instruction sequences, so under correct CPU/memory-subsystem behavior it
//   should never raise a hardware fault. If a board's silicon, cache
//   coherency, or atomic-unit implementation has a genuine defect, the most
//   likely externally-visible symptom is the kernel delivering SIGILL
//   (illegal/unsupported instruction - e.g. a botched AMO decode), SIGBUS
//   (misaligned or otherwise invalid access the bus can't service), SIGSEGV
//   (an access the MMU/cache path resolves to the wrong or a protected
//   page), or SIGFPE (an arithmetic trap).
//
//   Without handling these, the runner process would simply be killed by
//   the kernel: the host sees a UART timeout with zero diagnostic
//   information about what happened, and systemd silently respawns the
//   process, losing the finding entirely. Instead, a handler here records
//   which fault occurred and uses sigsetjmp/siglongjmp to unwind straight
//   back to the top of the current RUN's handling, so the runner reports a
//   DONE line with the fault flag set and keeps serving the UART.
//
//   Caveat: process/memory state after a genuine SIGSEGV/SIGBUS is not
//   strictly well-defined by the C standard. Continuing in-process (rather
//   than exiting and letting systemd restart) is a deliberate choice for
//   this research tool - it keeps a long unattended campaign moving and
//   preserves UART framing - but any fault-flagged result should be treated
//   as a strong finding to manually reproduce in isolation, not blindly
//   trusted alongside ordinary runs.
//============================================================================

#define FLAG_SIGILL  (1u << 0)
#define FLAG_SIGSEGV (1u << 1)
#define FLAG_SIGBUS  (1u << 2)
#define FLAG_SIGFPE  (1u << 3)

static sigjmp_buf g_fault_jmp;
static volatile sig_atomic_t g_fault_flags;

static void fault_signal_handler(int sig) {
    switch (sig) {
        case SIGILL:  g_fault_flags |= FLAG_SIGILL;  break;
        case SIGSEGV: g_fault_flags |= FLAG_SIGSEGV; break;
        case SIGBUS:  g_fault_flags |= FLAG_SIGBUS;  break;
        case SIGFPE:  g_fault_flags |= FLAG_SIGFPE;  break;
        default: break;
    }
    siglongjmp(g_fault_jmp, 1);
}

static void install_fault_handlers(void) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = fault_signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0; /* no SA_RESTART: readline_uart/write_all already retry on EINTR */

    int sigs[] = { SIGILL, SIGSEGV, SIGBUS, SIGFPE };
    for (size_t i = 0; i < sizeof(sigs) / sizeof(sigs[0]); ++i) {
        if (sigaction(sigs[i], &sa, NULL) != 0) {
            die("sigaction");
        }
    }
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
static inline uint32_t amominu_w(volatile uint32_t *p, uint32_t val) {
    uint32_t old;
    asm volatile (
        "amominu.w %0, %2, (%1)"
        : "=r"(old)
        : "r"(p), "r"(val)
        : "memory"
    );
    return old;
}

static inline uint32_t amomaxu_w(volatile uint32_t *p, uint32_t val) {
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
    uint64_t total_ns;      /* wall-clock nanoseconds, from clock_gettime(CLOCK_MONOTONIC); not a CPU cycle count */
    uint32_t worst_window;
    uint64_t worst_ns;      /* wall-clock nanoseconds for the worst window; not a CPU cycle count */
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
     - total_ns      : total run time in nanoseconds (clock_gettime(CLOCK_MONOTONIC)) for the entire workload
     - worst_window  : which timing window was slowest
     - worst_ns      : how slow that worst window was, in nanoseconds
     - flags         : 0 if the run completed cleanly; otherwise a bitwise-OR
                       of FLAG_SIGILL/FLAG_SIGSEGV/FLAG_SIGBUS/FLAG_SIGFPE
                       recorded by fault_signal_handler() when the caller's
                       sigsetjmp recovery point is hit mid-run (see "Fault
                       Detection" above) - this function itself never sets a
                       fault bit, since by the time it can return normally
                       the run genuinely completed without one.

   diag_mode selects which sections of the per-step body actually execute,
   for manually bisecting a cross-board checksum divergence down to a
   specific construct (see the diag_mode doc at the top of this file for the
   0/1/2 meanings). It defaults to 0, which is byte-for-byte the original,
   always-on behavior this function had before diag_mode existed.
   ---------------------------------------------------------------------------- */
static run_result_t workload_windowed(uint32_t seed, int steps, int diag_mode) {
    run_result_t rr;                                  /* Create the result struct that will hold checksum, timing, and window info. */
    memset(&rr, 0, sizeof(rr));                      /* Start with all result fields cleared so defaults are known and safe. */

    uint32_t st  = seed ^ 0xA5A5A5A5u;               /* Initialize the pseudo-random state from the input seed, but mix it first so small seed patterns do not map too directly into the generator state. */
    uint32_t acc = 0x12345678u;                      /* Initialize the running accumulator/checksum state with a non-zero constant so the workload starts from a fixed, known baseline. */

    int n_windows = (steps + WINDOW_SIZE - 1) / WINDOW_SIZE; /* Compute how many fixed-size timing windows are needed to cover the requested step count, rounding up for partial final windows. */
    if (n_windows > MAX_WINDOWS) {                   /* Guard against excessive window counts so the workload stays inside the designed analysis limits. */
        n_windows = MAX_WINDOWS;                     /* Limit the number of windows to the maximum supported value. */
    }

    /*
     * mem[] is a static array reused across every RUN command this process
     * ever handles, and the warm-up/main loops below only ever XOR into it -
     * without resetting it here first, this run's result would depend on
     * the accumulated history of every prior seed this runner has executed
     * since it started, not just on (seed, steps) as intended. Confirmed
     * experimentally: five back-to-back runs of the same seed on the same
     * board produced three different checksums before this fix. Resetting
     * to a fixed baseline here (before total_start is captured below) makes
     * checksum a pure, reproducible function of (seed, steps), and costs
     * nothing in the measured timing since it happens before the clock
     * starts.
     */
    memset(mem, 0, sizeof(mem));

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

            /* Normal memory section - skipped when diag_mode == 1 (ALU-only bisection) */
            if (diag_mode != 1) {
            uint32_t idx = (r ^ acc) % MEM_WORDS;    /* Choose a memory index based on both current pseudo-random state and accumulator state, making accesses data-dependent and less predictable. */
            uint32_t v   = mem[idx];                 /* Load the current value from the selected scratch-memory location. */

            v ^= (acc + r);                          /* Perturb the loaded value with current arithmetic state so memory contents evolve with execution history. */
            v += (v << 11) ^ (v >> 9);               /* Apply more local arithmetic mixing to amplify differences in data patterns and instruction mix. */
            mem[idx] = v;                            /* Store the updated value back into memory, creating a read-modify-write memory access pattern. */

            acc ^= mem[(idx + (acc & 1023u)) % MEM_WORDS]; /* Perform a second dependent memory read using both the current index and low bits of the accumulator, making later behavior depend on earlier state and memory contents. */

            /* Atomic stress section using explicit RISC-V AMO operations - skipped
               entirely for diag_mode 1 and 2 (both bisect AMO/branch out); included
               for diag_mode 0 and 3 */
            if ((diag_mode == 0 || diag_mode == 3) && (r & 0x1Fu) == 0u) { /* Only execute the atomic stress block occasionally so it influences the workload without completely dominating every step. */
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

            /* Occasional branch-like perturbation - skipped for diag_mode 1/2 along with AMO above */
            if (diag_mode == 0 && (r & 0x3FFu) == 0x155u) { /* Occasionally take an alternate path based on the pseudo-random pattern so the control-flow profile is not completely uniform. */
                acc ^= 0xDEADBEEFu;                    /* Perturb the accumulator strongly when that rare condition is met, making branch timing and path differences visible in results. */
            }
            } /* end: if (diag_mode != 1) - normal memory section */
        }

        uint64_t w1 = monotonic_ticks_u64();          /* Record the end time of the current window. */
        uint64_t w_ns = w1 - w0;                      /* Compute the elapsed nanoseconds for just this window. */

        if (w_ns > rr.worst_ns) {                     /* Check whether this window is the slowest one seen so far. */
            rr.worst_ns = w_ns;                       /* Save the timing of the slowest window so far. */
            rr.worst_window = (uint32_t)w;            /* Save which window index produced that worst timing. */
        }
    }

    uint64_t total_end = monotonic_ticks_u64();       /* Record the end time of the full workload. */

    rr.total_ns = total_end - total_start;            /* Save the total run time (ns) so the host can compare overall timing behavior across boards. */
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

    install_fault_handlers();

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

        /* RUN command: RUN <seed_dec> <steps_dec> [diag_mode_dec]\n
           diag_mode defaults to 0 (full workload) when the third field is
           absent, which is the normal case for every existing caller. */
        unsigned seed = 0;
        int steps = 0;
        int diag_mode = 0;

        {
            int parsed = sscanf(line, "RUN %u %d %d", &seed, &steps, &diag_mode);
            if (parsed == 2) {
                diag_mode = 0;
            } else if (parsed != 3) {
                printf("[INFO] ignored unrecognized command\n");
                fflush(stdout);
                continue;
            }
        }

        if (steps < 1) {
            steps = 1;
        }

        if (steps > WINDOW_SIZE * MAX_WINDOWS) {
            steps = WINDOW_SIZE * MAX_WINDOWS;
        }

        if (diag_mode < 0 || diag_mode > 3) {
            diag_mode = 0;
        }

        printf("[RUN] seed=%u steps=%d diag_mode=%d\n", (uint32_t)seed, steps, diag_mode);
        fflush(stdout);

        run_result_t rr;
        g_fault_flags = 0;
        if (sigsetjmp(g_fault_jmp, 1) != 0) {
            /* Landed here via siglongjmp from fault_signal_handler(): the
               workload below did not complete, so checksum/total_ns/
               worst_window/worst_ns stay at their zeroed defaults and only
               the fault flag(s) are meaningful for this result. */
            memset(&rr, 0, sizeof(rr));
            rr.flags = (uint32_t)g_fault_flags;
            printf("[FAULT] seed=%u flags=0x%x\n", (uint32_t)seed, rr.flags);
            fflush(stdout);
        } else {
            rr = workload_windowed((uint32_t)seed, steps, diag_mode);
        }

        /* Human-readable local console summary */
        printf(
            "[DONE] seed=%u checksum=%u total_ns=%" PRIu64
            " flags=%u worst_window=%u worst_ns=%" PRIu64 "\n",
            (uint32_t)seed,
            rr.checksum,
            rr.total_ns,
            (uint32_t)rr.flags,
            rr.worst_window,
            rr.worst_ns
        );
        fflush(stdout);

        /* Machine-readable protocol line back to FPGA/host */
        char out[256];
        snprintf(
            out,
            sizeof(out),
            "DONE %u %u %" PRIu64 " %u %u %" PRIu64 "\n",
            (uint32_t)seed,
            rr.checksum,
            rr.total_ns,
            (uint32_t)rr.flags,
            rr.worst_window,
            rr.worst_ns
        );

        write_all(fd, out);
    }

    return 0;
}