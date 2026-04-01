/*
===============================================================================
File        : rl_host.c
Language    : C (C11-compatible style)
Target      : Windows host PC
Build       : MinGW-w64 / GCC
Output      : rl_host.exe

TITLE
-------------------------------------------------------------------------------
Reinforcement Learning Host Controller for FPGA Differential Fuzzing Lab.

DESCRIPTION
-------------------------------------------------------------------------------
This program implements a full RL-driven differential fuzzing controller.

ARCHITECTURE
-------------------------------------------------------------------------------
FPGA -> returns per-board:
      DONE ...
      OR T (timeout)

Host:
  - Sends job to each board individually
  - Collects 3 DONE results
  - Builds TRIPLE locally
  - Computes reward
  - Updates RL policy

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

RUNNER binary on each BOARD:
  - Waits for RUN command
  - Executes workload
  - Measures timing
  - Returns DONE line
    - If workload takes too long, returns T (timeout)
where:
  score         = total_cycles XOR checksum
  flags         = anomaly flags (reserved for future expansion)
  worst_window  = window index with highest cycle cost
  worst_cycles  = cycle count of the worst window

REWARD FUNCTION
-------------------------------------------------------------------------------
The reward function is a combination of:
  - Score divergence between boards (primary signal)
  - Presence of flags (e.g. detected faults)
  - Divergence in worst window index
  - Divergence in worst window cycles
This reward function is designed to guide the RL agent towards test cases that cause 
divergent behavior across the boards, which may indicate interesting corner cases or 
potential faults.

===============================================================================
*/

// Disable MSVC / MinGW warnings about "unsafe" functions like sprintf
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

// Simple wrapper around Windows COM port APIs
typedef struct {
    HANDLE h;
} com_t;

// Error handling helper
static void die(const char *msg) {
    fprintf(stderr, "%s\n", msg);
    exit(1);
}

// Open a COM port with 115200 baud, 8N1 settings
static com_t com_open(const char *port) {
    char path[64];
    // Windows COM ports above COM9 require a special path format
    snprintf(path, sizeof(path), "\\\\.\\%s", port);

    // Open the COM port with read|write access
    HANDLE h = CreateFileA(path, GENERIC_READ|GENERIC_WRITE,0,NULL,OPEN_EXISTING,0,NULL);
    if (h == INVALID_HANDLE_VALUE) die("COM open failed");

    // Configure COM port settings
    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(h, &dcb);

    // Set baud rate, 8 data bits, no parity, 1 stop bit
    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity   = NOPARITY;

    // Apply the settings
    SetCommState(h, &dcb);

    return (com_t){h};
}

// Send a command to a specific board
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

// Read a line of text from the COM port (blocking)
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

//=============================================================================
// DATA STRUCTURES
//=============================================================================

// Triple structure to hold results from 3 boards for a single test case
typedef struct {
    uint32_t seed;
    int      steps;

    uint64_t score[3];
    uint32_t flags[3];
    uint32_t worst_window[3];
    uint64_t worst_cycles[3];

    bool ok;
} triple_t;

// Parse a DONE line from the FPGA and fill in the triple structure
static void parse_done(const char *line, int idx, triple_t *triple) {
    // Example DONE line:
    // DONE 00112233 12345678 0x5 64 987    
    unsigned seed, flags, worst_window;
    unsigned long long score, worst_cycles;

    sscanf(line,
        "DONE %x %llu %x %u %llu",
        &seed, &score, &flags, &worst_window, &worst_cycles);

    triple->seed = seed;
    triple->score[idx] = score;
    triple->flags[idx] = flags;
    triple->worst_window[idx] = worst_window;
    triple->worst_cycles[idx] = worst_cycles;
    triple->ok = true;
}

//=============================================================================
//   REWARD FUNCTION
//=============================================================================

// Compute a reward based on the divergence of the 3 board results in the triple
static double digital_divergence(const triple_t *t) {
    double d01 = fabs((double)t->score[0] - (double)t->score[1]);
    double d02 = fabs((double)t->score[0] - (double)t->score[2]);
    double d12 = fabs((double)t->score[1] - (double)t->score[2]);

    double score_term = (d01 + d02 + d12) / 1000.0;

    double fault_bonus = 0;
    for (int i = 0; i < 3; i++)
        if (t->flags[i] != 0) fault_bonus += 1e6;

    double window_bonus = 0;
    if (!(t->worst_window[0]==t->worst_window[1] && t->worst_window[1]==t->worst_window[2]))
        window_bonus += 2e5;

    double wc_term =
        fabs((double)t->worst_cycles[0] - t->worst_cycles[1]) +
        fabs((double)t->worst_cycles[0] - t->worst_cycles[2]) +
        fabs((double)t->worst_cycles[1] - t->worst_cycles[2]);

    return score_term + fault_bonus + window_bonus + wc_term;
}

//=============================================================================
//   Reinforcement Learning Upper Confidence Bound (UCB)
//=============================================================================

// Simple multi-armed bandit implementation using Upper Confidence Bound (UCB) algorithm
typedef struct {
    int steps;
    uint64_t pulls;
    double mean;
} arm_t;

// Choose an arm using UCB strategy
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

// Update an arm's statistics with a new reward
static void update_arm(arm_t *a, double r) {
    a->pulls++;
    double alpha = 1.0 / a->pulls;
    a->mean = (1-alpha)*a->mean + alpha*r;
}

//=============================================================================
//   MAIN
//=============================================================================
int main(int argc, char **argv) {

    if (argc < 4)
        die("Usage: rl_host COM Port seed_lo seed_hi\n"
            "Example: rl_host COM5 0x00000010 0x00010000");

    // Parse command line arguments
    uint32_t lo = strtoul(argv[2],0,0);
    uint32_t hi = strtoul(argv[3],0,0);

    // Open COM port
    com_t c = com_open(argv[1]);

    // Initialize arms for different step counts
    arm_t arms[] = {
        {64,0,0},{128,0,0},{256,0,0},{512,0,0},{1024,0,0}
    };

    // Seed the random number generator
    srand(time(NULL));

    char line[512];
    uint64_t iter = 0;

    // Main loop: choose an arm, send jobs to boards, collect results, compute reward, update arm
    while (true) {

        int ai = choose_ucb(arms,5);
        int steps = arms[ai].steps;
        uint32_t seed = lo + rand()%(hi-lo+1);

        triple_t tr = {0};
        tr.steps = steps;

        // Send the same seed and step count to all 3 boards and wait for their responses
        for (int b=0;b<3;b++) {
            send_to_board(&c,b,seed,steps);

            while (true) {
                readline(&c,line);

                // Check for timeout or DONE response
                if (line[0]=='T') {
                    printf("[TIMEOUT] board %d\n", b);
                    break;
                }

                // If we get a DONE line, parse it and move on to the next board
                if (strncmp(line,"DONE",4)==0) {
                    parse_done(line,b,&tr);
                    break;
                }
            }
        }

        // Compute the reward based on the triple and update the chosen arm
        double r = digital_divergence(&tr);

        // If we got valid results from all 3 boards, update the arm with the computed reward
        update_arm(&arms[ai], r);

        printf("iter=%llu seed=%08X steps=%d reward=%.2f\n",
            iter++, seed, steps, r);
    }
}