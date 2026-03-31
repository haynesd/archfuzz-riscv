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
This program implements a full RL-driven differential fuzzing controller.

UPDATED ARCHITECTURE
-------------------------------------------------------------------------------
Old:
  FPGA -> returns TRIPLE

New:
  FPGA -> returns per-board:
      DONE ...
      OR T (timeout)

Host:
  - Sends job to each board individually
  - Collects 3 DONE results
  - Builds TRIPLE locally
  - Computes reward
  - Updates RL policy

This drastically reduces FPGA complexity and synthesis time.

PROTOCOL
-------------------------------------------------------------------------------
Host -> FPGA:
  [0x01][BOARD][ASCII COMMAND]

Example:
  [0x01][0] "RUN 00112233 256\n"

FPGA -> Host:
  DONE <seed> <score> <flags> <window> <cycles>\n
  OR
  T\n

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

/* ============================================================================
   SERIAL
   ========================================================================== */

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

    HANDLE h = CreateFileA(path, GENERIC_READ|GENERIC_WRITE,0,NULL,OPEN_EXISTING,0,NULL);
    if (h == INVALID_HANDLE_VALUE) die("COM open failed");

    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(h, &dcb);

    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity   = NOPARITY;

    SetCommState(h, &dcb);

    return (com_t){h};
}

static void send_to_board(com_t *c, int board, uint32_t seed, int steps) {
    char payload[128];
    snprintf(payload, sizeof(payload), "RUN %08X %d\n", seed, steps);

    char frame[256];
    int len = (int)strlen(payload);

    frame[0] = 0x01;
    frame[1] = (char)board;
    memcpy(&frame[2], payload, len);

    DWORD w;
    WriteFile(c->h, frame, len + 2, &w, NULL);
}

static int readline(com_t *c, char *buf) {
    int i = 0;
    while (1) {
        char ch;
        DWORD n;
        ReadFile(c->h, &ch, 1, &n, NULL);
        if (n == 0) continue;

        buf[i++] = ch;
        if (ch == '\n') break;
    }
    buf[i] = 0;
    return i;
}

/* ============================================================================
   DATA STRUCTURES
   ========================================================================== */

typedef struct {
    uint32_t seed;
    int      steps;

    uint64_t s[3];
    uint32_t f[3];
    uint32_t w[3];
    uint64_t wc[3];

    bool ok;
} triple_t;

/* ============================================================================
   PARSE DONE
   ========================================================================== */

static void parse_done(const char *line, int idx, triple_t *t) {
    unsigned seed, flags, w;
    unsigned long long score, wc;

    sscanf(line,
        "DONE %x %llu %x %u %llu",
        &seed, &score, &flags, &w, &wc);

    t->seed = seed;
    t->s[idx] = score;
    t->f[idx] = flags;
    t->w[idx] = w;
    t->wc[idx] = wc;
    t->ok = true;
}

/* ============================================================================
   REWARD FUNCTION
   ========================================================================== */

static double digital_divergence(const triple_t *t) {
    double d01 = fabs((double)t->s[0] - (double)t->s[1]);
    double d02 = fabs((double)t->s[0] - (double)t->s[2]);
    double d12 = fabs((double)t->s[1] - (double)t->s[2]);

    double score_term = (d01 + d02 + d12) / 1000.0;

    double fault_bonus = 0;
    for (int i = 0; i < 3; i++)
        if (t->f[i] != 0) fault_bonus += 1e6;

    double window_bonus = 0;
    if (!(t->w[0]==t->w[1] && t->w[1]==t->w[2]))
        window_bonus += 2e5;

    double wc_term =
        fabs((double)t->wc[0] - t->wc[1]) +
        fabs((double)t->wc[0] - t->wc[2]) +
        fabs((double)t->wc[1] - t->wc[2]);

    return score_term + fault_bonus + window_bonus + wc_term;
}

/* ============================================================================
   RL (UCB)
   ========================================================================== */

typedef struct {
    int steps;
    uint64_t pulls;
    double mean;
} arm_t;

static int choose_ucb(arm_t *a, int n) {
    uint64_t total = 0;
    for (int i=0;i<n;i++) total += a[i].pulls;

    for (int i=0;i<n;i++)
        if (a[i].pulls == 0) return i;

    double best = -1e9;
    int best_i = 0;

    for (int i=0;i<n;i++) {
        double bonus = sqrt(2*log((double)total)/a[i].pulls);
        double val = a[i].mean + bonus;
        if (val > best) { best = val; best_i = i; }
    }

    return best_i;
}

static void update_arm(arm_t *a, double r) {
    a->pulls++;
    double alpha = 1.0 / a->pulls;
    a->mean = (1-alpha)*a->mean + alpha*r;
}

/* ============================================================================
   MAIN
   ========================================================================== */

int main(int argc, char **argv) {

    if (argc < 4)
        die("Usage: rl_host COM3 seed_lo seed_hi");

    uint32_t lo = strtoul(argv[2],0,0);
    uint32_t hi = strtoul(argv[3],0,0);

    com_t c = com_open(argv[1]);

    arm_t arms[] = {
        {64,0,0},{128,0,0},{256,0,0},{512,0,0},{1024,0,0}
    };

    srand(time(NULL));

    char line[512];
    uint64_t iter = 0;

    while (1) {

        int ai = choose_ucb(arms,5);
        int steps = arms[ai].steps;
        uint32_t seed = lo + rand()%(hi-lo+1);

        triple_t tr = {0};
        tr.steps = steps;

        for (int b=0;b<3;b++) {
            send_to_board(&c,b,seed,steps);

            while (1) {
                readline(&c,line);

                if (line[0]=='T') {
                    printf("[TIMEOUT] board %d\n", b);
                    break;
                }

                if (strncmp(line,"DONE",4)==0) {
                    parse_done(line,b,&tr);
                    break;
                }
            }
        }

        double r = digital_divergence(&tr);
        update_arm(&arms[ai], r);

        printf("iter=%llu seed=%08X steps=%d reward=%.2f\n",
            iter++, seed, steps, r);
    }
}