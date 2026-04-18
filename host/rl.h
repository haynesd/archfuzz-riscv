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

Design Notes
-------------------------------------------------------------------------------
- The RL layer depends on serial transport for board execution.
- Optional Rigol integration is injected through rigol_config_t.
- The public API below corresponds directly to the host executable modes.
===============================================================================
*/

#ifndef RL_H
#define RL_H

#include "common.h"
#include "rigol.h"
#include "waveform.h"

/*
-------------------------------------------------------------------------------
triple_result_t
-------------------------------------------------------------------------------
Aggregates the architectural and waveform results for one seed/steps test case
executed across all boards.
-------------------------------------------------------------------------------
*/
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

/*
-------------------------------------------------------------------------------
rl_arm_t
-------------------------------------------------------------------------------
Represents one bandit arm corresponding to a specific step-count configuration.
-------------------------------------------------------------------------------
*/
typedef struct {
    int steps;
    uint64_t pulls;
    double mean_reward;
} rl_arm_t;

/*
-------------------------------------------------------------------------------
rl_parse_done_line
-------------------------------------------------------------------------------
Parses one board DONE response into the aggregate result structure.

Parameters:
  line        - raw DONE line received from the board path.
  board_index - board index associated with this line.
  result      - aggregate result structure to update.

Returns:
  true  - line parsed successfully.
  false - line did not match the expected DONE format.
-------------------------------------------------------------------------------
*/
bool rl_parse_done_line(const char *line, int board_index, triple_result_t *result);

/*
-------------------------------------------------------------------------------
rl_compute_digital_divergence
-------------------------------------------------------------------------------
Computes the architectural-only divergence score for a completed triplet.

Parameters:
  result - aggregate triplet result.

Returns:
  Architectural divergence scalar.
-------------------------------------------------------------------------------
*/
double rl_compute_digital_divergence(const triple_result_t *result);

/*
-------------------------------------------------------------------------------
rl_compute_combined_reward
-------------------------------------------------------------------------------
Combines architectural divergence with optional waveform divergence.

Parameters:
  result       - aggregate triplet result.
  wave_summary - optional waveform differential summary, or NULL.

Returns:
  Combined reward value used by the bandit policy.
-------------------------------------------------------------------------------
*/
double rl_compute_combined_reward(const triple_result_t *result, const wave_diff_summary_t *wave_summary);

/*
-------------------------------------------------------------------------------
rl_choose_ucb_arm
-------------------------------------------------------------------------------
Selects the next arm using the UCB1 policy.

Parameters:
  arms      - array of arm descriptors.
  arm_count - number of valid entries in arms.

Returns:
  Index of the selected arm.
-------------------------------------------------------------------------------
*/
int rl_choose_ucb_arm(rl_arm_t *arms, int arm_count);

/*
-------------------------------------------------------------------------------
rl_update_arm
-------------------------------------------------------------------------------
Updates one arm with an observed reward using an incremental running mean.

Parameters:
  arm    - arm to update.
  reward - observed reward from the most recent run.
-------------------------------------------------------------------------------
*/
void rl_update_arm(rl_arm_t *arm, double reward);

/*
-------------------------------------------------------------------------------
rl_mode_ping
-------------------------------------------------------------------------------
Implements the executable's ping mode.

Parameters:
  com_port    - COM port name.
  board_index - board to ping.

Returns:
  Process-style status code.
-------------------------------------------------------------------------------
*/
int rl_mode_ping(const char *com_port, int board_index);

/*
-------------------------------------------------------------------------------
rl_mode_run1
-------------------------------------------------------------------------------
Implements the executable's single-run validation mode.

Parameters:
  com_port    - COM port name.
  board_index - board to exercise.
  seed        - deterministic workload seed.
  steps       - workload length or stress parameter.

Returns:
  Process-style status code.
-------------------------------------------------------------------------------
*/
int rl_mode_run1(const char *com_port, int board_index, uint32_t seed, int steps);

/*
-------------------------------------------------------------------------------
rl_mode_loop
-------------------------------------------------------------------------------
Implements the continuous RL loop. When a Rigol configuration is provided and
enabled, each board execution is followed by a waveform capture.

Parameters:
  com_port - COM port name.
  seed_lo  - inclusive lower seed bound.
  seed_hi  - inclusive upper seed bound.
  rigol    - optional Rigol capture configuration, or NULL.

Returns:
  Process-style status code. The loop is normally continuous until terminated.
-------------------------------------------------------------------------------
*/
int rl_mode_loop(const char *com_port, uint32_t seed_lo, uint32_t seed_hi, const rigol_config_t *rigol);

#endif
