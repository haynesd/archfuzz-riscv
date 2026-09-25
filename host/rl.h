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

checksum and total_ns are reported by board/runner.c as separate fields
(rather than one XORed "score") specifically so architectural/correctness
divergence (checksum) and pure clock-speed/timing divergence (total_ns) can
be told apart instead of conflated into a single number - three boards
running at different clock speeds will show total_ns differences on
essentially every run regardless of whether the computed result was
actually correct, so that term alone was never a reliable defect signal.

flags is 0 for boards whose runner_c workload/runner.c completed cleanly,
or a bitwise-OR of the board-side FLAG_SIGILL/FLAG_SIGSEGV/FLAG_SIGBUS/
FLAG_SIGFPE bits (see board/runner.c) if the board's kernel delivered a
genuine hardware fault signal partway through the workload - in that case
that board's checksum/total_ns/worst_window/worst_ns are all 0, since the
run did not complete.

A cross-board checksum mismatch is a real, meaningful correctness-divergence
signal - but treat it as a lead, not a conclusion, until the boards' running
binaries are independently confirmed identical (see the diag_mode
investigation note in board/runner.c for a case where an apparent 100%
divergence across 944782 campaign rows turned out to be board 0 silently
running stale code, not an architectural difference).
-------------------------------------------------------------------------------
*/
typedef struct {
    uint32_t seed;
    int steps;
    uint32_t checksum[RL_BOARD_COUNT];
    uint64_t total_ns[RL_BOARD_COUNT]; /* wall-clock nanoseconds for the whole run; not a CPU cycle count */
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
  diag_mode   - 0 for the normal full workload, or 1/2 to bisect a cross-board
                checksum divergence down to a specific construct (see the
                diag_mode doc at the top of board/runner.c). Only this manual
                single-run mode can send a nonzero diag_mode; the rl/rl_scope
                bandit loop always runs diag_mode 0.

Returns:
  Process-style status code.
-------------------------------------------------------------------------------
*/
int rl_mode_run1(const char *com_port, int board_index, uint32_t seed, int steps, int diag_mode);

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
