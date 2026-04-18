/*
===============================================================================
File        : rl_host.c
Language    : C (C11-compatible style)
Target      : Windows host PC
Build       : MinGW-w64 / GCC
Output      : rl_host.exe

TITLE
-------------------------------------------------------------------------------
Reinforcement Learning Host Controller for FPGA Differential Fuzzing Lab

DESCRIPTION
-------------------------------------------------------------------------------
This program is the Windows host-side controller for the FPGA differential
fuzzing lab.

It can be used in three modes:

  1) ping
     Send a PING command to one board and wait for PONG.
     Useful for quick end-to-end connectivity testing.

  2) run1
     Send one RUN command to one board and print the DONE response.
     Useful for validating one board before running RL.

  3) rl
     Run the reinforcement-learning loop across all 3 boards.
     The host sends the same seed/step count to each board, collects the
     results, computes a reward, and updates the UCB policy.

ARCHITECTURE
-------------------------------------------------------------------------------
        ┌──────────┐
        │  RL_HOST │
        └────┬─────┘
             │ UART (USB-Serial)
             ▼
        ┌──────────┐
        │   FPGA   │
        │ (Router) │
        └─┬──┬──┬──┘
          │  │  │
          ▼  ▼  ▼
        Board0 Board1 Board2
        (runner processes)

PROTOCOL
-------------------------------------------------------------------------------
Host -> FPGA:
  [0x01][BOARD][ASCII COMMAND]

Examples:
  [0x01][0] "PING\n"
  [0x01][0] "RUN 12345 256\n"

FPGA -> Host:
  PONG\n
  DONE <seed> <score> <flags> <window> <cycles>\n
  T\n

SEED EXPLANATION
-------------------------------------------------------------------------------
A seed is just a number that controls how the board's deterministic workload
behaves.

Same seed:
  - same generated test pattern
  - same sequence of operations

Different seed:
  - different test pattern
  - different memory / arithmetic / timing behavior

In other words:
  Seed = one specific reproducible test case

COMPILATION
-------------------------------------------------------------------------------
Release build:
  gcc -O2 -Wall -Wextra -std=c11 -o rl_host.exe rl_host.c

Debug build:
  gcc -O0 -g -Wall -Wextra -std=c11 -o rl_host_debug.exe rl_host.c

No external libraries are required.

USAGE
-------------------------------------------------------------------------------
1) Ping one board:
     rl_host.exe ping <COM_PORT> <BOARD>

   Example:
     rl_host.exe ping COM5 0

2) Run one test on one board:
     rl_host.exe run1 <COM_PORT> <BOARD> <SEED> <STEPS>

   Example:
     rl_host.exe run1 COM5 0 12345 256

3) Run RL loop on all 3 boards:
     rl_host.exe rl <COM_PORT> <SEED_START> <SEED_END>

   Example:
     rl_host.exe rl COM5 16 65536

Board numbers:
  0 = Board 0
  1 = Board 1
  2 = Board 2

Notes:
  - Use normal decimal numbers
  - In RL mode, the host randomly picks seeds between SEED_START and SEED_END
  - Larger seed ranges provide more test variety

===============================================================================
*/

#define _CRT_SECURE_NO_WARNINGS

#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <inttypes.h>

//=============================================================================
// SERIAL
//=============================================================================

typedef struct {
    HANDLE h;
} com_t;

static void die(const char *msg) {
    fprintf(stderr, "%s\n", msg);
    exit(1);
}

static void print_usage_and_exit(void) {
    die(
        "Usage:\n"
        "  rl_host.exe ping <COM_PORT> <BOARD>\n"
        "  rl_host.exe run1 <COM_PORT> <BOARD> <SEED> <STEPS>\n"
        "  rl_host.exe rl   <COM_PORT> <SEED_START> <SEED_END>\n\n"
        "Examples:\n"
        "  rl_host.exe ping COM5 0\n"
        "  rl_host.exe run1 COM5 0 12345 256\n"
        "  rl_host.exe rl COM5 16 65536\n"
    );
}

static com_t com_open(const char *port) {
    char path[64];
    snprintf(path, sizeof(path), "\\\\.\\%s", port);

    HANDLE h = CreateFileA(
        path,
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL
    );

    if (h == INVALID_HANDLE_VALUE) die("COM open failed");

    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);

    if (!GetCommState(h, &dcb)) die("GetCommState failed");

    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity   = NOPARITY;
    dcb.fBinary  = TRUE;
    dcb.fParity  = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fTXContinueOnXoff = TRUE;
    dcb.fOutX = FALSE;
    dcb.fInX  = FALSE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;

    if (!SetCommState(h, &dcb)) die("SetCommState failed");

    COMMTIMEOUTS t = {0};
    t.ReadIntervalTimeout         = 50;
    t.ReadTotalTimeoutConstant    = 200;
    t.ReadTotalTimeoutMultiplier  = 0;
    t.WriteTotalTimeoutConstant   = 200;
    t.WriteTotalTimeoutMultiplier = 0;

    if (!SetCommTimeouts(h, &t)) die("SetCommTimeouts failed");

    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);

    return (com_t){h};
}

static void dump_bytes_hex(const uint8_t *buf, int len) {
    for (int i = 0; i < len; i++) {
        printf("%02X ", buf[i]);
    }
    printf("\n");
}

static void write_frame(com_t *c, int board, const char *payload) {
    uint8_t frame[256];
    int len = (int)strlen(payload);

    if (board < 0 || board > 2) die("Board must be 0, 1, or 2");
    if (len + 2 > (int)sizeof(frame)) die("Payload too large");

    frame[0] = 0x01;
    frame[1] = (uint8_t)board;
    memcpy(&frame[2], payload, len);

    DWORD w = 0;
    if (!WriteFile(c->h, frame, (DWORD)(len + 2), &w, NULL) || w != (DWORD)(len + 2)) {
        die("WriteFile failed");
    }

    printf("[HOST TX][board %d] ", board);
    dump_bytes_hex(frame, len + 2);
    printf("[HOST TX ASCII][board %d] %s", board, payload);
}

static void send_ping(com_t *c, int board) {
    write_frame(c, board, "PING\n");
}

static void send_run(com_t *c, int board, uint32_t seed, int steps) {
    char payload[128];
    snprintf(payload, sizeof(payload), "RUN %u %d\n", seed, steps);
    write_frame(c, board, payload);
}

static int readline(com_t *c, char *buf, int max) {
    int i = 0;

    while (i < max - 1) {
        char ch = 0;
        DWORD n = 0;

        if (!ReadFile(c->h, &ch, 1, &n, NULL)) {
            return -1;
        }

        if (n == 0) {
            continue;
        }

        buf[i++] = ch;
        if (ch == '\n') break;
    }

    buf[i] = 0;
    return i;
}

//=============================================================================
// DATA STRUCTURES
//=============================================================================

typedef struct {
    uint32_t seed;
    int      steps;

    uint64_t score[3];
    uint32_t flags[3];
    uint32_t worst_window[3];
    uint64_t worst_cycles[3];

    bool done[3];
    bool ok;
} triple_t;

static bool parse_done(const char *line, int idx, triple_t *triple) {
    unsigned seed = 0, flags = 0, worst_window = 0;
    unsigned long long score = 0, worst_cycles = 0;

    int rc = sscanf(
        line,
        "DONE %u %llu %u %u %llu",
        &seed, &score, &flags, &worst_window, &worst_cycles
    );

    if (rc != 5) {
        return false;
    }

    triple->seed = seed;
    triple->score[idx] = (uint64_t)score;
    triple->flags[idx] = flags;
    triple->worst_window[idx] = worst_window;
    triple->worst_cycles[idx] = (uint64_t)worst_cycles;
    triple->done[idx] = true;
    triple->ok = triple->done[0] && triple->done[1] && triple->done[2];
    return true;
}

//=============================================================================
// REWARD FUNCTION
//=============================================================================

static double digital_divergence(const triple_t *t) {
    double d01 = fabs((double)t->score[0] - (double)t->score[1]);
    double d02 = fabs((double)t->score[0] - (double)t->score[2]);
    double d12 = fabs((double)t->score[1] - (double)t->score[2]);

    double score_term = (d01 + d02 + d12) / 1000.0;

    double fault_bonus = 0.0;
    for (int i = 0; i < 3; i++) {
        if (t->flags[i] != 0) fault_bonus += 1e6;
    }

    double window_bonus = 0.0;
    if (!(t->worst_window[0] == t->worst_window[1] &&
          t->worst_window[1] == t->worst_window[2])) {
        window_bonus += 2e5;
    }

    double wc_term =
        fabs((double)t->worst_cycles[0] - (double)t->worst_cycles[1]) +
        fabs((double)t->worst_cycles[0] - (double)t->worst_cycles[2]) +
        fabs((double)t->worst_cycles[1] - (double)t->worst_cycles[2]);

    return score_term + fault_bonus + window_bonus + wc_term;
}

//=============================================================================
// UCB (Upper Confidence Bound) - Multi-Armed Bandit Strategy
//=============================================================================
/*
Probability Theory:
  - Multi-armed bandit problem: a framework for decision-making under uncertainty 
  - UCB1 algorithm: a strategy for balancing exploration and exploitation 

The multi-armed bandit problem is named from imagining a gambler at a row of 
slot machines ("one-armed bandits"), who has to decide which machines to play, 
how many times to play each machine and in which order to play them,
and whether to continue with the current machine or try a different machine. [1]

UCB1 is a specific algorithm for solving the multi-armed bandit problem. It works by
assigning a score to each arm (configuration) that combines:
  - the average reward observed from that arm (exploitation)
  - a bonus term that encourages trying less-tested arms (exploration) [2]

Ref: 
[1] Weber, Richard (1992), "On the Gittins index for multiarmed bandits"
[2] Auer, Cesa-Bianchi and Fischer (2002), "Finite-time Analysis of the Multiarmed Bandit Problem"

This section implements a simple reinforcement learning strategy (UCB1) used
by the host to decide *which test configuration to run next*.

  - Each "arm" represents a different workload configuration (typically a
    different step count).
  - Each run produces a "reward" (how much divergence or interesting behavior
    was observed across boards).
  - The goal is to automatically focus testing effort on configurations that
    produce the most interesting architectural differences.

Instead of blindly testing all configurations equally, UCB1 balances:
  - Exploration  - try new or under-tested configurations
  - Exploitation - focus on configurations that already showed strong signals

This dramatically improves efficiency when searching for:
  - timing divergence
  - atomic behavior differences
  - microarchitectural inconsistencies
  - power analysis differences
------------------------------------------------------------------------------
*/

/*
------------------------------------------------------------------------------
arm_t
------------------------------------------------------------------------------
Represents one "arm" in the bandit problem.

In this system, an arm corresponds to:
  → a specific workload configuration (e.g., a specific step count)

Each arm tracks how well that configuration performs in terms of producing
useful fuzzing signals.
------------------------------------------------------------------------------
*/
typedef struct {
    int steps;           /* The number of steps used when running this arm.
                           This directly controls workload duration and stress. */

    uint64_t pulls;     /* How many times this configuration has been tested.
                           More pulls = more confidence in its observed behavior. */

    double mean;        /* Running average reward for this configuration.
                           Reward reflects how much architectural divergence
                           this configuration has produced historically. */
} arm_t;


/*
------------------------------------------------------------------------------
choose_ucb
------------------------------------------------------------------------------
Selects the next arm (configuration) to run using the UCB1 algorithm.

This is the "decision engine" of the Architecture Fuzzing Board Process (AFBP).
Answering which workload configuration should be tested next.

UCB formula:
  value = mean_reward + exploration_bonus

Where:
  - mean_reward = how good this arm has been so far
  - bonus       = encourages trying less-tested arms

This ensures:
  - High-performing configs are reused
  - Under-tested configs are still explored
------------------------------------------------------------------------------
*/
static int choose_ucb(arm_t *a, int n) {

    uint64_t total = 0;                     /* Total number of pulls across all arms.
                                               This represents total experiment count. */

    for (int i = 0; i < n; i++)             /* Sum pulls across all arms. */
        total += a[i].pulls;

    /*
    --------------------------------------------------------------------------
    Exploration guarantee:
    --------------------------------------------------------------------------
    If any arm has never been tested, choose it immediately.

    Why:
      - UCB1 requires at least one sample per arm
      - Ensures no configuration is ignored
      - Guarantees baseline coverage across all step sizes
    */
    for (int i = 0; i < n; i++) {
        if (a[i].pulls == 0)
            return i;                       /* Force exploration of new arm */
    }

    double best = -1e300;                  /* Initialize best score to a very small number so any real value wins. */

    int best_i = 0;                        /* Track index of best arm found so far */

    /*
    --------------------------------------------------------------------------
    Core UCB selection loop
    --------------------------------------------------------------------------
    Evaluate each arm using:

      UCB = mean + sqrt(2 * log(total) / pulls)

    Interpretation:
      - mean - exploitation (how good it is)
      - sqrt(...) - exploration bonus
    */
    for (int i = 0; i < n; i++) {

        /*
        Exploration bonus:
          - Larger when pulls is small - favors under-tested arms
          - Shrinks as pulls increases - trust established arms more
        */
        double bonus = sqrt(
            2.0 * log((double)total) / (double)a[i].pulls
        );

        /* Total value for this arm:
          - mean reward (exploitation)
          + exploration bonus
        */
        double val = a[i].mean + bonus;

        /* Select arm with highest UCB value */
        if (val > best) {
            best = val;
            best_i = i;
        }
    }

    return best_i; /* Return index of chosen configuration */
}


/*
------------------------------------------------------------------------------
update_arm
------------------------------------------------------------------------------
Updates statistics for an arm after a run completes.

Inputs:
  - a : pointer to the arm being updated
  - r : reward from the most recent run

Reward in your system typically reflects:
  - divergence between boards
  - timing differences
  - anomalous behavior

This function updates:
  - how many times the arm was used
  - its average reward

This feeds back into UCB for future decisions.
------------------------------------------------------------------------------
*/
static void update_arm(arm_t *a, double r) {

    a->pulls++; /* Increment usage count. This reduces exploration bonus for this arm in future selections. */

    /*Learning rate: alpha = 1 / pulls

    This implements an incremental average:
      - Early runs have large influence
      - Later runs refine the estimate more slowly
    */
    double alpha = 1.0 / (double)a->pulls;

    /*
    Running mean update:

      new_mean = (1 - alpha)*old_mean + alpha*reward

    Equivalent to:
      mean = average of all observed rewards

    Why this matters:
      - Tracks how "interesting" this configuration is
      - Drives exploitation in UCB selection
    */
    a->mean = (1.0 - alpha) * a->mean + alpha * r;
}

//=============================================================================
// HELPERS
//=============================================================================

static int parse_board(const char *s) {
    int b = atoi(s);
    if (b < 0 || b > 2) die("Board must be 0, 1, or 2");
    return b;
}

static uint32_t parse_u32_decimal(const char *s, const char *what) {
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 10);
    if (end == s || *end != '\0') {
        fprintf(stderr, "Invalid %s: %s\n", what, s);
        exit(1);
    }
    return (uint32_t)v;
}

static int parse_int_decimal(const char *s, const char *what) {
    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (end == s || *end != '\0') {
        fprintf(stderr, "Invalid %s: %s\n", what, s);
        exit(1);
    }
    return (int)v;
}

static bool read_one_response(com_t *c, char *line, int line_sz) {
    int n = readline(c, line, line_sz);
    if (n < 0) die("ReadFile failed");
    if (n == 0) return false;
    return true;
}

//=============================================================================
// MODES
//=============================================================================

static int mode_ping(const char *com_port, int board) {
    com_t c = com_open(com_port);
    char line[512];

    send_ping(&c, board);

    while (true) {
        if (!read_one_response(&c, line, (int)sizeof(line))) continue;

        printf("[HOST RX][board %d] %s", board, line);

        if (strncmp(line, "PONG", 4) == 0) {
            printf("[OK] Received PONG from board %d\n", board);
            return 0;
        }

        if (strcmp(line, "T\n") == 0 || strcmp(line, "T\r\n") == 0) {
            printf("[FAIL] Timeout waiting for board %d\n", board);
            return 1;
        }
    }
}

static int mode_run1(const char *com_port, int board, uint32_t seed, int steps) {
    com_t c = com_open(com_port);
    char line[512];

    send_run(&c, board, seed, steps);

    while (true) {
        if (!read_one_response(&c, line, (int)sizeof(line))) continue;

        printf("[HOST RX][board %d] %s", board, line);

        if (strncmp(line, "DONE", 4) == 0) {
            printf("[OK] Single run completed on board %d\n", board);
            return 0;
        }

        if (strcmp(line, "T\n") == 0 || strcmp(line, "T\r\n") == 0) {
            printf("[FAIL] Timeout waiting for board %d\n", board);
            return 1;
        }
    }
}

static int mode_rl(const char *com_port, uint32_t lo, uint32_t hi) {
    if (hi < lo) die("SEED_END must be >= SEED_START");

    com_t c = com_open(com_port);

    arm_t arms[] = {
        {64,0,0},{128,0,0},{256,0,0},{512,0,0},{1024,0,0}
    };

    srand((unsigned)time(NULL));

    char line[512];
    uint64_t iter = 0;

    while (true) {
        int ai = choose_ucb(arms, 5);
        int steps = arms[ai].steps;

        uint32_t span = hi - lo + 1;
        uint32_t seed = lo + (uint32_t)(rand() % span);

        triple_t tr = {0};
        tr.steps = steps;

        bool any_timeout = false;
        bool any_parse_error = false;

        for (int b = 0; b < 3; b++) {
            send_run(&c, b, seed, steps);

            while (true) {
                if (!read_one_response(&c, line, (int)sizeof(line))) continue;

                printf("[HOST RX][board %d] %s", b, line);

                if (strcmp(line, "T\n") == 0 || strcmp(line, "T\r\n") == 0) {
                    printf("[TIMEOUT] board %d\n", b);
                    any_timeout = true;
                    break;
                }

                if (strncmp(line, "DONE", 4) == 0) {
                    if (!parse_done(line, b, &tr)) {
                        printf("[PARSE ERROR] board %d line=%s", b, line);
                        any_parse_error = true;
                    }
                    break;
                }
            }
        }

        if (any_timeout || any_parse_error || !tr.ok) {
            printf("iter=%" PRIu64 " seed=%u steps=%d skipped\n",
                   iter++, seed, steps);
            continue;
        }

        double r = digital_divergence(&tr);
        update_arm(&arms[ai], r);

        printf("iter=%" PRIu64 " seed=%u steps=%d reward=%.2f\n",
               iter++, seed, steps, r);
    }
}

//=============================================================================
// MAIN
//=============================================================================

int main(int argc, char **argv) {
    if (argc < 2) {
        print_usage_and_exit();
    }

    if (strcmp(argv[1], "ping") == 0) {
        if (argc != 4) print_usage_and_exit();

        const char *com_port = argv[2];
        int board = parse_board(argv[3]);
        return mode_ping(com_port, board);
    }

    if (strcmp(argv[1], "run1") == 0) {
        if (argc != 6) print_usage_and_exit();

        const char *com_port = argv[2];
        int board = parse_board(argv[3]);
        uint32_t seed = parse_u32_decimal(argv[4], "SEED");
        int steps = parse_int_decimal(argv[5], "STEPS");

        return mode_run1(com_port, board, seed, steps);
    }

    if (strcmp(argv[1], "rl") == 0) {
        if (argc != 5) print_usage_and_exit();

        const char *com_port = argv[2];
        uint32_t lo = parse_u32_decimal(argv[3], "SEED_START");
        uint32_t hi = parse_u32_decimal(argv[4], "SEED_END");

        return mode_rl(com_port, lo, hi);
    }

    print_usage_and_exit();
    return 0;
}