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
This program implements a host-side reinforcement learning (RL) controller
for a multi-board differential fuzzing system.

The system architecture consists of:
  - Host PC (this program)
  - FPGA (UART router + scheduler)
  - Multiple RISC-V boards running runner.c

The host:
  - Selects test parameters (seed + step count)
  - Sends jobs to each board via the FPGA
  - Collects results
  - Computes a reward based on divergence
  - Updates an RL policy (UCB multi-armed bandit)

This enables automated discovery of interesting execution divergences
across heterogeneous or identical RISC-V implementations.

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

Example:
  [0x01][0] "RUN 00112233 256\n"

Commands:
  RUN <seed_hex8> <steps>\n

FPGA behavior:
  - Routes command to selected board
  - Buffers UART traffic (TX + RX)
  - Returns board response line to host
  - Returns "T\n" on timeout

Board (runner.c):
  - Receives ASCII command
  - Executes deterministic workload
  - Returns:
      DONE <seed> <score> <flags> <window> <cycles>\n
    or:
      T\n

DATA FORMAT
-------------------------------------------------------------------------------
DONE line fields:
  seed          : test seed (hex)
  score         : total_cycles XOR checksum
  flags         : anomaly flags (future use)
  worst_window  : index of slowest window
  worst_cycles  : cycle/time cost of that window

Note:
  Timing uses clock_gettime() (not rdcycle) for Linux compatibility.

REINFORCEMENT LEARNING
-------------------------------------------------------------------------------
Multi-armed bandit (UCB):

Arms:
  {64, 128, 256, 512, 1024}

Loop:
  1. Select arm via UCB
  2. Choose random seed
  3. Execute test on all boards
  4. Collect results
  5. Compute reward
  6. Update arm statistics

REWARD FUNCTION
-------------------------------------------------------------------------------
Reward is based on divergence across boards:

  - Score divergence (primary signal)
  - Fault flags (large bonus)
  - Worst-window disagreement
  - Worst-cycle differences

Goal:
  Maximize behavioral divergence → discover corner cases,
  timing anomalies, or hardware inconsistencies.

===============================================================================
COMPILATION
-------------------------------------------------------------------------------

Windows (MinGW-w64 / MSYS2 / Git Bash):

Release build:
  gcc -O2 -Wall -Wextra -std=c11 -o rl_host.exe rl_host.c

Debug build:
  gcc -O0 -g -Wall -Wextra -std=c11 -o rl_host_debug.exe rl_host.c

Notes:
  - Requires Windows headers (windows.h)
  - No external libraries needed
  - Works with MinGW-w64 

===============================================================================
USAGE
-------------------------------------------------------------------------------
  rl_host COM5 0x00000010 0x00010000

Arguments:
  COMx     : Windows serial port
  seed_lo  : lower bound of seed range
  seed_hi  : upper bound of seed range

Example:
  rl_host COM5 0x00000010 0x00010000

===============================================================================
OUTPUT
-------------------------------------------------------------------------------
Example:

  iter=42 seed=00112233 steps=256 reward=12345.67

===============================================================================
NOTES
-------------------------------------------------------------------------------
- Requires FPGA firmware with UART buffering (TX + RX)
- Requires runner.c running on each board
- Serial settings: 115200 8N1
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

static com_t com_open(const char *port) {
    char path[64];
    snprintf(path, sizeof(path), "\\\\.\\%s", port);

    HANDLE h = CreateFileA(path, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
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
    dcb.fDtrControl  = DTR_CONTROL_DISABLE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fTXContinueOnXoff = TRUE;
    dcb.fOutX = FALSE;
    dcb.fInX  = FALSE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;

    if (!SetCommState(h, &dcb)) die("SetCommState failed");

    // Important: avoid blocking forever
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

static void send_to_board(com_t *c, int board, uint32_t seed, int steps) {
    char payload[128];
    snprintf(payload, sizeof(payload), "RUN %08X %d\n", seed, steps);

    uint8_t frame[256];
    int len = (int)strlen(payload);

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

static int readline(com_t *c, char *buf, int max) {
    int i = 0;

    while (i < max - 1) {
        char ch = 0;
        DWORD n = 0;

        if (!ReadFile(c->h, &ch, 1, &n, NULL)) {
            return -1;
        }

        if (n == 0) {
            // timeout slice
            continue;
        }

        buf[i++] = ch;
        if (ch == '\n') {
            break;
        }
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

    int rc = sscanf(line,
        "DONE %x %llu %x %u %llu",
        &seed, &score, &flags, &worst_window, &worst_cycles);

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
// UCB
//=============================================================================

typedef struct {
    int steps;
    uint64_t pulls;
    double mean;
} arm_t;

static int choose_ucb(arm_t *a, int n) {
    uint64_t total = 0;
    for (int i = 0; i < n; i++) total += a[i].pulls;

    for (int i = 0; i < n; i++) {
        if (a[i].pulls == 0) return i;
    }

    double best = -1e300;
    int best_i = 0;

    for (int i = 0; i < n; i++) {
        double bonus = sqrt(2.0 * log((double)total) / (double)a[i].pulls);
        double val = a[i].mean + bonus;
        if (val > best) {
            best = val;
            best_i = i;
        }
    }

    return best_i;
}

static void update_arm(arm_t *a, double r) {
    a->pulls++;
    double alpha = 1.0 / (double)a->pulls;
    a->mean = (1.0 - alpha) * a->mean + alpha * r;
}

//=============================================================================
// MAIN
//=============================================================================

int main(int argc, char **argv) {
    if (argc < 4) {
        die("Usage: rl_host COMx seed_lo seed_hi\n"
            "Example: rl_host COM5 0x00000010 0x00010000");
    }

    uint32_t lo = (uint32_t)strtoul(argv[2], 0, 0);
    uint32_t hi = (uint32_t)strtoul(argv[3], 0, 0);

    if (hi < lo) {
        die("seed_hi must be >= seed_lo");
    }

    com_t c = com_open(argv[1]);

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
            send_to_board(&c, b, seed, steps);

            while (true) {
                int n = readline(&c, line, (int)sizeof(line));
                if (n < 0) {
                    die("ReadFile failed");
                }

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
            printf("iter=%" PRIu64 " seed=%08X steps=%d skipped\n",
                   iter++, seed, steps);
            continue;
        }

        double r = digital_divergence(&tr);
        update_arm(&arms[ai], r);

        printf("iter=%" PRIu64 " seed=%08X steps=%d reward=%.2f\n",
               iter++, seed, steps, r);
    }
}