/*
===============================================================================
File        : rl.h
Language    : C (C11-compatible style)
Target      : Windows host PC
Purpose     : Reinforcement-learning control loop and reward computation

Description
-------------------------------------------------------------------------------
This module contains the orchestration logic for the architectural differential
fuzzing host. It parses DONE lines, computes architectural divergence, combines
that with waveform divergence when available, and drives the UCB-based arm
selection policy.

In scope-aware mode, the RL loop:
  1. arms the Rigol for one single-shot acquisition,
  2. runs the three boards sequentially,
  3. reads CH1-CH4,
  4. aligns CH1-CH3 using the CH4 trigger edge, and
  5. uses the aligned window in the reward.
===============================================================================
*/
#ifndef RL_H
#define RL_H

#include "common.h"
#include "rigol.h"
#include "waveform.h"

typedef struct {
    uint32_t seed;
    int steps;
    uint64_t score[RL_BOARD_COUNT];
    uint32_t flags[RL_BOARD_COUNT];
    uint32_t worst_window[RL_BOARD_COUNT];
    uint64_t worst_cycles[RL_BOARD_COUNT];
    wave_metrics_t wave[RL_BOARD_COUNT];
    bool done[RL_BOARD_COUNT];
    bool ok;
} triple_result_t;

typedef struct {
    int steps;
    uint64_t pulls;
    double mean_reward;
} rl_arm_t;

bool rl_parse_done_line(const char *line, int board_index, triple_result_t *result);
double rl_compute_digital_divergence(const triple_result_t *result);
double rl_compute_combined_reward(const triple_result_t *result, const wave_diff_summary_t *wave_summary);
int rl_choose_ucb_arm(rl_arm_t *arms, int arm_count);
void rl_update_arm(rl_arm_t *arm, double reward);
int rl_mode_ping(const char *com_port, int board_index);
int rl_mode_run1(const char *com_port, int board_index, uint32_t seed, int steps);
int rl_mode_loop(const char *com_port, uint32_t seed_lo, uint32_t seed_hi, const rigol_config_t *rigol);

#endif
