/*
===============================================================================
File        : rl_host.c
Language    : C (C11-compatible style)
Target      : Windows host PC
Build       : MinGW-w64 / GCC on Git Bash or MSYS2
Output      : rl_host.exe

PURPOSE
-------------------------------------------------------------------------------
This program is the HOST-SIDE controller for a differential multi-board fuzzing
and divergence analysis lab.

It communicates with an FPGA over a UART link exposed through a USB-to-TTL
adapter (e.g., FT232). The FPGA orchestrates execution across three target
boards and returns a single aggregated ASCII result line ("TRIPLE ...").

The host performs four primary functions:

  1) Select the next experimental stimulus:
       - seed
       - steps
     using a lightweight UCB multi-armed bandit policy.

  2) Send the stimulus to the FPGA as:
       RUN SEED=<hex> STEPS=<dec>

  3) Receive and parse the FPGA aggregate result:
       TRIPLE SEED=... STEPS=...
              S0=... F0=... W0=... WC0=...
              S1=... F1=... W1=... WC1=...
              S2=... F2=... W2=... WC2=...

  4) Compute a reward signal based on:
       - score divergence
       - fault divergence
       - worst-window disagreement
       - worst-window cycle disagreement

This host program does NOT directly execute any target workload. It is purely
a controller, parser, logger, and policy engine.

ASCII PROTOCOL
-------------------------------------------------------------------------------
Host -> FPGA:
  RUN SEED=00112233 STEPS=256\n

FPGA -> Host:
  TRIPLE SEED=00112233 STEPS=256
         S0=123456789 F0=00000000 W0=5 WC0=1842
         S1=123450001 F1=00000000 W1=5 WC1=1770
         S2=123460220 F2=00000000 W2=6 WC2=2055\n

BUILD
-------------------------------------------------------------------------------
Example (Git Bash / MinGW-w64):
  gcc -O2 -Wall -Wextra -o rl_host.exe rl_host.c -lm

RUN
-------------------------------------------------------------------------------
  ./rl_host.exe COM3 0x10 0x10000

Arguments:
  argv[1] : COM port (e.g., COM3)
  argv[2] : seed lower bound (hex or decimal)
  argv[3] : seed upper bound (hex or decimal)

NOTES
-------------------------------------------------------------------------------
- This version is Windows-native and uses WinAPI serial functions.
- It does not depend on termios.h.
- It expects the FT232 cable to be wired:
      FT232 TXD -> FPGA RX
      FT232 RXD <- FPGA TX
      FT232 GND <-> FPGA GND
- VCC from the FT232 cable should not be connected to the FPGA I/O rail.
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
#include <stdarg.h>

/* ============================================================================
   Serial Port Layer
   ========================================================================== */

typedef struct {
    HANDLE h;
    char   name[64];
} com_t;

/* --------------------------------------------------------------------------
   die_last
   --------------------------------------------------------------------------
   Print a WinAPI error and terminate.
   -------------------------------------------------------------------------- */
static void die_last(const char *msg) {
    DWORD e = GetLastError();
    fprintf(stderr, "%s (GetLastError=%lu)\n", msg, (unsigned long)e);
    exit(1);
}

/* --------------------------------------------------------------------------
   die_msg
   --------------------------------------------------------------------------
   Print a plain error and terminate.
   -------------------------------------------------------------------------- */
static void die_msg(const char *msg) {
    fprintf(stderr, "%s\n", msg);
    exit(1);
}

/* --------------------------------------------------------------------------
   com_make_path
   --------------------------------------------------------------------------
   Convert a friendly COM name like "COM3" into a WinAPI path "\\\\.\\COM3".
   -------------------------------------------------------------------------- */
static void com_make_path(const char *in, char *out, size_t outsz) {
    if (strncmp(in, "\\\\.\\", 4) == 0) {
        strncpy(out, in, outsz - 1);
        out[outsz - 1] = 0;
        return;
    }
    snprintf(out, outsz, "\\\\.\\%s", in);
}

/* --------------------------------------------------------------------------
   com_open_115200_8n1
   --------------------------------------------------------------------------
   Open a COM port in raw 115200 8N1 mode, no flow control.
   -------------------------------------------------------------------------- */
static com_t com_open_115200_8n1(const char *port) {
    com_t c = {0};
    strncpy(c.name, port, sizeof(c.name) - 1);

    char path[64];
    com_make_path(port, path, sizeof(path));

    HANDLE h = CreateFileA(
        path,
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL
    );

    if (h == INVALID_HANDLE_VALUE) {
        die_last("CreateFile(COM) failed");
    }

    SetupComm(h, 1 << 15, 1 << 15);

    COMMTIMEOUTS to = {0};
    to.ReadIntervalTimeout         = 50;
    to.ReadTotalTimeoutConstant    = 50;
    to.ReadTotalTimeoutMultiplier  = 10;
    to.WriteTotalTimeoutConstant   = 50;
    to.WriteTotalTimeoutMultiplier = 10;

    if (!SetCommTimeouts(h, &to)) {
        die_last("SetCommTimeouts failed");
    }

    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);

    if (!GetCommState(h, &dcb)) {
        die_last("GetCommState failed");
    }

    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary  = TRUE;

    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl  = DTR_CONTROL_DISABLE;
    dcb.fRtsControl  = RTS_CONTROL_DISABLE;
    dcb.fOutX        = FALSE;
    dcb.fInX         = FALSE;

    if (!SetCommState(h, &dcb)) {
        die_last("SetCommState failed");
    }

    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);

    c.h = h;
    return c;
}

/* --------------------------------------------------------------------------
   com_close
   --------------------------------------------------------------------------
   Close a COM port handle.
   -------------------------------------------------------------------------- */
static void com_close(com_t *c) {
    if (c && c->h && c->h != INVALID_HANDLE_VALUE) {
        CloseHandle(c->h);
        c->h = NULL;
    }
}

/* --------------------------------------------------------------------------
   com_write_all
   --------------------------------------------------------------------------
   Write the full string to the serial port.
   -------------------------------------------------------------------------- */
static void com_write_all(com_t *c, const char *s) {
    DWORD n = 0;
    size_t len = strlen(s);

    if (!WriteFile(c->h, s, (DWORD)len, &n, NULL)) {
        die_last("WriteFile failed");
    }

    FlushFileBuffers(c->h);
}

/* --------------------------------------------------------------------------
   com_writef
   --------------------------------------------------------------------------
   printf-style serial transmission.
   -------------------------------------------------------------------------- */
static void com_writef(com_t *c, const char *fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    com_write_all(c, buf);
}

/* --------------------------------------------------------------------------
   com_readline
   --------------------------------------------------------------------------
   Read one newline-terminated ASCII line from the serial port.
   Returns:
     0  -> timeout slice / no line yet
     >0 -> number of chars read
   -------------------------------------------------------------------------- */
static int com_readline(com_t *c, char *out, size_t max) {
    size_t n = 0;

    while (n + 1 < max) {
        char ch;
        DWORD got = 0;

        if (!ReadFile(c->h, &ch, 1, &got, NULL)) {
            die_last("ReadFile failed");
        }

        if (got == 0) {
            if (n == 0) {
                return 0;
            }
            continue;
        }

        out[n++] = ch;
        if (ch == '\n') {
            break;
        }
    }

    out[n] = 0;
    return (int)n;
}

/* ============================================================================
   TRIPLE Parsing
   ========================================================================== */

/*
Expected FPGA aggregate line:

TRIPLE SEED=00112233 STEPS=256
       S0=123456789 F0=00000000 W0=5 WC0=1842
       S1=123450001 F1=00000000 W1=5 WC1=1770
       S2=123460220 F2=00000000 W2=6 WC2=2055
*/

typedef struct {
    uint32_t seed;
    int      steps;

    uint64_t s[3];   /* score */
    uint32_t f[3];   /* flags */
    uint32_t w[3];   /* worst window index */
    uint64_t wc[3];  /* worst window cycle count */

    bool ok;
} triple_t;

/* --------------------------------------------------------------------------
   parse_triple
   --------------------------------------------------------------------------
   Parse the FPGA aggregate line into a structured form.
   -------------------------------------------------------------------------- */
static triple_t parse_triple(const char *line) {
    triple_t t = {0};

    unsigned seed = 0;
    int steps = 0;

    unsigned long long s0 = 0, s1 = 0, s2 = 0;
    unsigned f0 = 0, f1 = 0, f2 = 0;
    unsigned w0 = 0, w1 = 0, w2 = 0;
    unsigned long long wc0 = 0, wc1 = 0, wc2 = 0;

    int n = sscanf(
        line,
        "TRIPLE SEED=%x STEPS=%d "
        "S0=%llu F0=%x W0=%u WC0=%llu "
        "S1=%llu F1=%x W1=%u WC1=%llu "
        "S2=%llu F2=%x W2=%u WC2=%llu",
        &seed, &steps,
        &s0, &f0, &w0, &wc0,
        &s1, &f1, &w1, &wc1,
        &s2, &f2, &w2, &wc2
    );

    if (n == 14) {
        t.seed  = (uint32_t)seed;
        t.steps = steps;

        t.s[0]  = (uint64_t)s0;
        t.s[1]  = (uint64_t)s1;
        t.s[2]  = (uint64_t)s2;

        t.f[0]  = (uint32_t)f0;
        t.f[1]  = (uint32_t)f1;
        t.f[2]  = (uint32_t)f2;

        t.w[0]  = (uint32_t)w0;
        t.w[1]  = (uint32_t)w1;
        t.w[2]  = (uint32_t)w2;

        t.wc[0] = (uint64_t)wc0;
        t.wc[1] = (uint64_t)wc1;
        t.wc[2] = (uint64_t)wc2;

        t.ok = true;
    }

    return t;
}

/* ============================================================================
   Reward Model
   ========================================================================== */

/* --------------------------------------------------------------------------
   digital_divergence
   --------------------------------------------------------------------------
   Compute a host-side divergence reward from score/fault/window data.

   Components:
     - score divergence across the three boards
     - anomaly flag bonus
     - worst-window disagreement bonus
     - worst-window cycle disagreement

   This is intentionally simple and transparent for research prototyping.
   -------------------------------------------------------------------------- */
static double digital_divergence(const triple_t *t) {
    double d01 = (double)llabs((long long)(t->s[0] - t->s[1]));
    double d02 = (double)llabs((long long)(t->s[0] - t->s[2]));
    double d12 = (double)llabs((long long)(t->s[1] - t->s[2]));

    double score_term = (d01 + d02 + d12) / 1000.0;

    double fault_bonus = 0.0;
    for (int i = 0; i < 3; i++) {
        if (t->f[i] != 0) {
            fault_bonus += 1e6;
        }
    }

    double window_bonus = 0.0;
    if (!(t->w[0] == t->w[1] && t->w[1] == t->w[2])) {
        window_bonus += 2e5;
    }

    double wc01 = (double)llabs((long long)(t->wc[0] - t->wc[1]));
    double wc02 = (double)llabs((long long)(t->wc[0] - t->wc[2]));
    double wc12 = (double)llabs((long long)(t->wc[1] - t->wc[2]));
    double wc_term = (wc01 + wc02 + wc12) / 10.0;

    return score_term + fault_bonus + window_bonus + wc_term;
}

/* ============================================================================
   UCB Bandit Policy
   ========================================================================== */

typedef struct {
    int      steps;
    uint64_t pulls;
    double   mean_reward;
} arm_t;

/* --------------------------------------------------------------------------
   choose_ucb
   --------------------------------------------------------------------------
   Select an arm using Upper Confidence Bound.
   -------------------------------------------------------------------------- */
static int choose_ucb(arm_t *arms, int n) {
    uint64_t total = 0;
    for (int i = 0; i < n; i++) {
        total += arms[i].pulls;
    }

    for (int i = 0; i < n; i++) {
        if (arms[i].pulls == 0) {
            return i;
        }
    }

    double best = -1e300;
    int best_i = 0;

    for (int i = 0; i < n; i++) {
        double bonus = sqrt(2.0 * log((double)total) / (double)arms[i].pulls);
        double score = arms[i].mean_reward + bonus;
        if (score > best) {
            best = score;
            best_i = i;
        }
    }

    return best_i;
}

/* --------------------------------------------------------------------------
   update_arm
   --------------------------------------------------------------------------
   Update an arm's running mean reward.
   -------------------------------------------------------------------------- */
static void update_arm(arm_t *a, double reward) {
    a->pulls++;
    double alpha = 1.0 / (double)a->pulls;
    a->mean_reward = (1.0 - alpha) * a->mean_reward + alpha * reward;
}

typedef struct {
    uint32_t seed;
    double   reward;
} best_t;

/* --------------------------------------------------------------------------
   rand_u32
   --------------------------------------------------------------------------
   Lightweight 32-bit random helper.
   -------------------------------------------------------------------------- */
static uint32_t rand_u32(void) {
    return ((uint32_t)rand() << 16) ^ (uint32_t)rand();
}

/* --------------------------------------------------------------------------
   pick_seed
   --------------------------------------------------------------------------
   Pick a seed either:
     - uniformly from [lo, hi], or
     - near the best-known seed for local exploitation
   -------------------------------------------------------------------------- */
static uint32_t pick_seed(uint32_t lo, uint32_t hi, const best_t *best, double p_local) {
    if (best && ((double)rand() / (double)RAND_MAX) < p_local) {
        int32_t delta = (int32_t)(rand_u32() % 2048) - 1024;
        uint32_t s = best->seed + (uint32_t)delta;
        if (s < lo) s = lo;
        if (s > hi) s = hi;
        return s;
    }

    uint32_t span = (hi - lo) + 1;
    return lo + (rand_u32() % span);
}

/* ============================================================================
   Main
   ========================================================================== */

int main(int argc, char **argv) {
    if (argc < 4) {
        fprintf(stderr, "Usage: %s <COMx> <seed_lo_hex> <seed_hi_hex>\n", argv[0]);
        fprintf(stderr, "Example: %s COM3 0x10 0x10000\n", argv[0]);
        return 2;
    }

    const char *port = argv[1];
    uint32_t seed_lo = (uint32_t)strtoul(argv[2], NULL, 0);
    uint32_t seed_hi = (uint32_t)strtoul(argv[3], NULL, 0);

    if (seed_hi < seed_lo) {
        die_msg("seed_hi must be >= seed_lo");
    }

    srand((unsigned)time(NULL));

    com_t fpga = com_open_115200_8n1(port);

    arm_t arms[] = {
        {  64, 0, 0.0 },
        { 128, 0, 0.0 },
        { 256, 0, 0.0 },
        { 512, 0, 0.0 },
        {1024, 0, 0.0 }
    };
    const int N_ARMS = (int)(sizeof(arms) / sizeof(arms[0]));

    best_t best = {0, -1e300};
    double p_local = 0.20;

    char line[1024];
    uint64_t iter = 0;

    for (;;) {
        int ai = choose_ucb(arms, N_ARMS);
        int steps = arms[ai].steps;
        uint32_t seed = pick_seed(seed_lo, seed_hi, (best.reward > 0 ? &best : NULL), p_local);

        /* Send host command to FPGA */
        com_writef(&fpga, "RUN SEED=%08X STEPS=%d\n", seed, steps);

        /* Wait for TRIPLE */
        triple_t tr = {0};

        for (int tries = 0; tries < 500; tries++) {
            int n = com_readline(&fpga, line, sizeof(line));
            if (n == 0) {
                continue;
            }

            tr = parse_triple(line);
            if (tr.ok) {
                break;
            }

            /* Uncomment for debugging raw UART traffic
            printf("RAW: %s", line);
            */
        }

        if (!tr.ok) {
            fprintf(stderr, "[WARN] No valid TRIPLE for seed=%08X steps=%d\n", seed, steps);
            update_arm(&arms[ai], -1.0);
            continue;
        }

        double reward = digital_divergence(&tr);
        update_arm(&arms[ai], reward);

        if (reward > best.reward) {
            best.seed = tr.seed;
            best.reward = reward;
            p_local = fmin(0.80, p_local + 0.05);
        } else {
            p_local = fmax(0.10, p_local * 0.999);
        }

        printf(
            "iter=%" PRIu64
            " seed=%08X"
            " steps=%d"
            "  S=[%" PRIu64 ",%" PRIu64 ",%" PRIu64 "]"
            " F=[%08X,%08X,%08X]"
            " W=[%u,%u,%u]"
            " WC=[%" PRIu64 ",%" PRIu64 ",%" PRIu64 "]"
            " reward=%.2f"
            " best=%08X(%.2f)"
            " p_local=%.2f"
            " arms:",
            iter++,
            tr.seed,
            tr.steps,
            tr.s[0], tr.s[1], tr.s[2],
            tr.f[0], tr.f[1], tr.f[2],
            tr.w[0], tr.w[1], tr.w[2],
            tr.wc[0], tr.wc[1], tr.wc[2],
            reward,
            best.seed, best.reward,
            p_local
        );

        for (int i = 0; i < N_ARMS; i++) {
            printf(" %d:%.1f(%" PRIu64 ")", arms[i].steps, arms[i].mean_reward, arms[i].pulls);
        }
        printf("\n");
        fflush(stdout);
    }

    com_close(&fpga);
    return 0;
}