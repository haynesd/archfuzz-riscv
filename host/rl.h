/*
===============================================================================
File        : rl.h
Language    : C (C11-compatible style)
Target      : Windows host PC
Purpose     : Reinforcement-learning control loop and reward computation

Description
-------------------------------------------------------------------------------
This module contains the logic for the architectural differential fuzzing host. 
It parses DONE lines, computes architectural divergence, combines
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
    uint64_t worst_ns[RL_BOARD_COUNT]; /* wall-clock nanoseconds for the worst window; not a CPU cycle count */
    wave_metrics_t wave[RL_BOARD_COUNT];
    bool done[RL_BOARD_COUNT];
    bool ok;
} triple_result_t;

/*
-------------------------------------------------------------------------------
rl_arm_t
-------------------------------------------------------------------------------
Represents one "arm" in the bandit problem.

In this system, an arm corresponds to:
  a specific workload configuration (e.g., a specific step count)

Each arm tracks how well that configuration performs in terms of producing
useful fuzzing signals.

Parameters:
  steps       - the step count or other workload parameter associated with this arm.
  pulls       - how many times this arm has been selected and executed.
  mean_reward - the average reward observed from this arm so far.
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
  result     - aggregate triplet result.
  skip_board - board index to exclude from every pairwise comparison, or -1
               to compare all RL_BOARD_COUNT boards as usual.

Returns:
  Architectural divergence scalar.
-------------------------------------------------------------------------------
*/
double rl_compute_digital_divergence(const triple_result_t *result, int skip_board);

/*
-------------------------------------------------------------------------------
rl_compute_combined_reward
-------------------------------------------------------------------------------
Combines architectural divergence with optional waveform divergence.

Parameters:
  result       - aggregate triplet result.
  wave_summary - optional waveform differential summary, or NULL.
  skip_board   - board index to exclude from every pairwise comparison, or -1
                 to compare all RL_BOARD_COUNT boards as usual.

Returns:
  Combined reward value used by the bandit policy.
-------------------------------------------------------------------------------
*/
double rl_compute_combined_reward(const triple_result_t *result, const wave_diff_summary_t *wave_summary, int skip_board);

/*
-------------------------------------------------------------------------------
rl_choose_ucb_arm
-------------------------------------------------------------------------------
Selects the next arm (configuration) to run using the UCB1 algorithm.

This is the "decision engine" of the Architecture Fuzzing Board Process (AFBP).
Answering which workload configuration should be tested next.

UCB formula:
  value = mean_reward + reward_scale * exploration_bonus

Where:
  - mean_reward   = how good this arm has been so far
  - bonus         = classic UCB1 term, sqrt(2*ln(total_pulls)/pulls);
                    encourages trying less-tested arms
  - reward_scale  = average magnitude of the arms' own observed mean
                    rewards. Classic UCB1 assumes rewards roughly in
                    [0,1]; this system's rewards are raw divergence
                    magnitudes (often in the hundreds of thousands), so
                    the bonus is rescaled to that magnitude - otherwise
                    it is negligible next to mean_reward and the bandit
                    stops meaningfully exploring after each arm's first
                    pull.

This ensures:
  - High-performing configs are reused
  - Under-tested configs are still explored

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
rl_run_options_t
-------------------------------------------------------------------------------
Optional experiment-control settings for rl_mode_loop, layered on top of the
required com_port/seed range/rigol arguments so campaigns are reproducible and
can survive being killed and restarted.

Fields:
  rng_seed           - master seed for the deterministic seed-selection PRNG.
                        0 means "derive a seed from the current time" (the
                        previous, non-reproducible default behavior).
  results_path       - optional path to a CSV file that receives one row per
                        completed iteration, or NULL to disable.
  checkpoint_path    - optional path used to persist/restore bandit arm
                        statistics so a long campaign can resume after being
                        interrupted, or NULL to disable.
  checkpoint_interval - number of iterations between checkpoint writes.
                        Ignored when checkpoint_path is NULL.
  skip_board          - board index to exclude entirely (not run, not
                         compared) for this session, or -1 to run and
                         compare all RL_BOARD_COUNT boards as usual. Useful
                         for continuing to collect data from known-good
                         boards while a specific board's link is being
                         debugged separately.
  board_delay_ms      - milliseconds to pause before addressing each board
                         within an iteration, or 0 for no delay (back to
                         back, as fast as possible). Diagnostic knob for
                         testing whether a board's link is being disturbed
                         by insufficient settling time after the previous
                         board's activity (e.g. a shared power rail not
                         having fully settled) rather than a wiring fault.
-------------------------------------------------------------------------------
*/
typedef struct {
    uint32_t rng_seed;
    const char *results_path;
    const char *checkpoint_path;
    int checkpoint_interval;
    int skip_board;
    int board_delay_ms;
} rl_run_options_t;

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
  options  - optional experiment-control settings, or NULL to use defaults
             (time-derived RNG seed, no results CSV, no checkpointing).

Returns:
  Process-style status code. The loop is normally continuous until terminated.
-------------------------------------------------------------------------------
*/
int rl_mode_loop(const char *com_port,
                 uint32_t seed_lo,
                 uint32_t seed_hi,
                 const rigol_config_t *rigol,
                 const rl_run_options_t *options);

#endif
