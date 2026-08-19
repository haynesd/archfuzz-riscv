#define _CRT_SECURE_NO_WARNINGS
#include "rl.h"

#include "serial.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/*
-------------------------------------------------------------------------------
Seed-selection PRNG
-------------------------------------------------------------------------------
Windows rand()/RAND_MAX is capped at 32767 (on both MSVC and MinGW). Any seed
range wider than that (e.g. "rl COM5 16 65536", the value used in the README
example) silently drops the seeds above RAND_MAX from ever being selected by
`rand() % span`, halving the effective seed-space coverage of a campaign with
no error or warning. This xorshift32 generator has full 32-bit range and is
explicitly seeded so a campaign can also be reproduced via --rng-seed.
-------------------------------------------------------------------------------
*/
static uint32_t rl_rng_next(uint32_t *state) {
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

/*
-------------------------------------------------------------------------------
Reward function coefficients
-------------------------------------------------------------------------------
Named to match the formal reward function documented in README.md:
  R = alpha*D_semantic + beta*D_timing + gamma*D_power + delta*D_fault
Centralizing them here makes the mapping from the paper's formula to this
implementation explicit and gives a single place to change for future
ablation experiments. Values are chosen to reproduce the magnitudes used
before this was split out (score/1000, +1e6 per fault, +2e5 window-location
mismatch, wave divergence * 1000).

Note: D_fault (delta term) is currently always 0 because board/runner.c never
sets a nonzero flags value (its `flags` field is explicitly reserved/TBD).
The wiring here is ready for fault detection to be added on-board; until
then this term does not contribute to the reward.
-------------------------------------------------------------------------------
*/
#define RL_ALPHA 1.0    /* D_semantic weight */
#define RL_BETA  1.0    /* D_timing weight   */
#define RL_GAMMA 1000.0 /* D_power weight    */
#define RL_DELTA 1.0    /* D_fault weight    */

bool rl_parse_done_line(const char *line, int board_index, triple_result_t *result) {
    unsigned seed = 0;
    unsigned flags = 0;
    unsigned worst_window = 0;
    unsigned long long score = 0;
    unsigned long long worst_cycles = 0;

    if (!line || !result || board_index < 0 || board_index >= RL_BOARD_COUNT) {
        return false;
    }

    int matched = sscanf(line,
                         "DONE %u %llu %u %u %llu",
                         &seed,
                         &score,
                         &flags,
                         &worst_window,
                         &worst_cycles);
    if (matched != 5) {
        return false;
    }

    result->seed = seed;
    result->score[board_index] = (uint64_t)score;
    result->flags[board_index] = flags;
    result->worst_window[board_index] = worst_window;
    result->worst_cycles[board_index] = (uint64_t)worst_cycles;
    result->done[board_index] = true;
    result->ok = result->done[0] && result->done[1] && result->done[2];
    return true;
}

double rl_compute_digital_divergence(const triple_result_t *result) {
    double d01 = fabs((double)result->score[0] - (double)result->score[1]);
    double d02 = fabs((double)result->score[0] - (double)result->score[2]);
    double d12 = fabs((double)result->score[1] - (double)result->score[2]);

    double d_semantic = (d01 + d02 + d12) / 1000.0;

    double d_fault = 0.0;
    for (int i = 0; i < RL_BOARD_COUNT; ++i) {
        if (result->flags[i] != 0) {
            d_fault += 1e6;
        }
    }

    double window_bonus = 0.0;
    if (!(result->worst_window[0] == result->worst_window[1] &&
          result->worst_window[1] == result->worst_window[2])) {
        window_bonus = 2e5;
    }

    double cycle_term =
        fabs((double)result->worst_cycles[0] - (double)result->worst_cycles[1]) +
        fabs((double)result->worst_cycles[0] - (double)result->worst_cycles[2]) +
        fabs((double)result->worst_cycles[1] - (double)result->worst_cycles[2]);

    double d_timing = cycle_term + window_bonus;

    return (RL_ALPHA * d_semantic) + (RL_BETA * d_timing) + (RL_DELTA * d_fault);
}

double rl_compute_combined_reward(const triple_result_t *result, const wave_diff_summary_t *wave_summary) {
    double reward = rl_compute_digital_divergence(result);
    if (wave_summary && wave_summary->valid) {
        reward += RL_GAMMA * wave_summary->grand_total;
    }
    return reward;
}

int rl_choose_ucb_arm(rl_arm_t *arms, int arm_count) {
    uint64_t total_pulls = 0;
    for (int i = 0; i < arm_count; ++i) {
        total_pulls += arms[i].pulls;
    }

    for (int i = 0; i < arm_count; ++i) {
        if (arms[i].pulls == 0) {
            return i;
        }
    }

    double best_value = -1e300;
    int best_index = 0;

    for (int i = 0; i < arm_count; ++i) {
        double bonus = sqrt(2.0 * log((double)total_pulls) / (double)arms[i].pulls);
        double value = arms[i].mean_reward + bonus;
        if (value > best_value) {
            best_value = value;
            best_index = i;
        }
    }

    return best_index;
}

void rl_update_arm(rl_arm_t *arm, double reward) {
    arm->pulls++;
    double alpha = 1.0 / (double)arm->pulls;
    arm->mean_reward = (1.0 - alpha) * arm->mean_reward + alpha * reward;
}

static bool rl_read_one_response(serial_t *serial, char *line, int line_size) {
    return serial_read_line(serial, line, line_size);
}

int rl_mode_ping(const char *com_port, int board_index) {
    serial_t serial = serial_open(com_port);
    char line[512];

    serial_send_ping(&serial, board_index);

    while (true) {
        if (!rl_read_one_response(&serial, line, (int)sizeof(line))) {
            continue;
        }

        if (strncmp(line, "PONG", 4) == 0) {
            rl_log_message(RL_LOG_INFO, "[OK] Received PONG from board %d", board_index);
            serial_close(&serial);
            return 0;
        }

        if (strcmp(line, "T\n") == 0 || strcmp(line, "T\r\n") == 0) {
            rl_log_message(RL_LOG_ERROR, "[FAIL] Timeout waiting for board %d", board_index);
            serial_close(&serial);
            return 1;
        }
    }
}

int rl_mode_run1(const char *com_port, int board_index, uint32_t seed, int steps) {
    serial_t serial = serial_open(com_port);
    char line[512];

    serial_send_run(&serial, board_index, seed, steps);

    while (true) {
        if (!rl_read_one_response(&serial, line, (int)sizeof(line))) {
            continue;
        }

        if (strncmp(line, "DONE", 4) == 0) {
            rl_log_message(RL_LOG_INFO, "[OK] Single run completed on board %d", board_index);
            serial_close(&serial);
            return 0;
        }

        if (strcmp(line, "T\n") == 0 || strcmp(line, "T\r\n") == 0) {
            rl_log_message(RL_LOG_ERROR, "[FAIL] Timeout waiting for board %d", board_index);
            serial_close(&serial);
            return 1;
        }
    }
}

/*
-------------------------------------------------------------------------------
Results CSV
-------------------------------------------------------------------------------
Every prior version of this loop only ever wrote human-readable text to the
log stream, which is awkward to turn into the coverage/divergence/reward
plots a research write-up needs. When enabled via rl_run_options_t, one row
per completed iteration is appended to a CSV file instead.
-------------------------------------------------------------------------------
*/
static FILE *g_results_file = NULL;

static void rl_results_open(const char *path) {
    if (!path || !*path) {
        return;
    }

    g_results_file = fopen(path, "a");
    if (!g_results_file) {
        rl_log_message(RL_LOG_WARN, "Failed to open results CSV: %s", path);
        return;
    }

    if (ftell(g_results_file) == 0) {
        fprintf(g_results_file,
                "iteration,seed,steps,arm_steps,pulls,mean_reward,digital,reward,"
                "score0,score1,score2,flags0,flags1,flags2,"
                "worst_window0,worst_window1,worst_window2,"
                "worst_cycles0,worst_cycles1,worst_cycles2,"
                "wave_valid,wave_grand_total\n");
        fflush(g_results_file);
    }

    rl_log_message(RL_LOG_INFO, "Results CSV: %s", path);
}

static void rl_results_write_row(uint64_t iteration,
                                 const triple_result_t *result,
                                 int arm_steps,
                                 uint64_t pulls,
                                 double mean_reward,
                                 double digital,
                                 double reward,
                                 const wave_diff_summary_t *wave_summary) {
    if (!g_results_file) {
        return;
    }

    fprintf(g_results_file,
            "%" PRIu64 ",%u,%d,%d,%" PRIu64 ",%.6f,%.6f,%.6f,"
            "%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%u,%u,%u,"
            "%u,%u,%u,"
            "%" PRIu64 ",%" PRIu64 ",%" PRIu64 ","
            "%d,%.6f\n",
            iteration, result->seed, result->steps, arm_steps, pulls, mean_reward, digital, reward,
            result->score[0], result->score[1], result->score[2],
            result->flags[0], result->flags[1], result->flags[2],
            result->worst_window[0], result->worst_window[1], result->worst_window[2],
            result->worst_cycles[0], result->worst_cycles[1], result->worst_cycles[2],
            wave_summary ? (int)wave_summary->valid : 0,
            wave_summary ? wave_summary->grand_total : 0.0);
    fflush(g_results_file);
}

static void rl_results_close(void) {
    if (g_results_file) {
        fclose(g_results_file);
        g_results_file = NULL;
    }
}

/*
-------------------------------------------------------------------------------
Bandit-state checkpointing
-------------------------------------------------------------------------------
The loop below runs forever until the process is killed, which previously
meant every UCB arm statistic was lost on restart. When enabled, arm state
(and the iteration counter) is periodically written to a small text file and
restored from it at startup, so a multi-day campaign can resume instead of
starting the bandit from scratch. This only checkpoints periodically (not on
every iteration or on a clean-shutdown hook), so up to checkpoint_interval
iterations of state can be lost if the process is killed between saves.
-------------------------------------------------------------------------------
*/
static void rl_checkpoint_save(const char *path, const rl_arm_t *arms, int arm_count, uint64_t iteration) {
    if (!path || !*path) {
        return;
    }

    FILE *f = fopen(path, "w");
    if (!f) {
        rl_log_message(RL_LOG_WARN, "Failed to write checkpoint: %s", path);
        return;
    }

    fprintf(f, "%" PRIu64 "\n", iteration);
    for (int i = 0; i < arm_count; ++i) {
        fprintf(f, "%d %" PRIu64 " %.17g\n", arms[i].steps, arms[i].pulls, arms[i].mean_reward);
    }
    fclose(f);
}

static uint64_t rl_checkpoint_load(const char *path, rl_arm_t *arms, int arm_count) {
    if (!path || !*path) {
        return 0;
    }

    FILE *f = fopen(path, "r");
    if (!f) {
        return 0;
    }

    uint64_t iteration = 0;
    if (fscanf(f, "%" SCNu64, &iteration) != 1) {
        fclose(f);
        return 0;
    }

    for (int i = 0; i < arm_count; ++i) {
        int steps = 0;
        uint64_t pulls = 0;
        double mean_reward = 0.0;
        if (fscanf(f, "%d %" SCNu64 " %lf", &steps, &pulls, &mean_reward) != 3) {
            rl_log_message(RL_LOG_WARN, "Checkpoint %s is truncated; ignoring", path);
            fclose(f);
            return 0;
        }
        if (steps != arms[i].steps) {
            rl_log_message(RL_LOG_WARN,
                           "Checkpoint arm %d steps mismatch (file=%d expected=%d); ignoring checkpoint",
                           i, steps, arms[i].steps);
            fclose(f);
            return 0;
        }
        arms[i].pulls = pulls;
        arms[i].mean_reward = mean_reward;
    }

    fclose(f);
    rl_log_message(RL_LOG_INFO, "Resumed checkpoint %s at iteration=%" PRIu64, path, iteration);
    return iteration;
}

int rl_mode_loop(const char *com_port,
                 uint32_t seed_lo,
                 uint32_t seed_hi,
                 const rigol_config_t *rigol,
                 const rl_run_options_t *options) {
    if (seed_hi < seed_lo) {
        die("SEED_END must be >= SEED_START");
    }

    static const rl_run_options_t default_options = {0, NULL, NULL, 0};
    if (!options) {
        options = &default_options;
    }
    int checkpoint_interval = options->checkpoint_interval > 0 ? options->checkpoint_interval : 20;

    serial_t serial = serial_open(com_port);

    rl_arm_t arms[RL_ARM_COUNT] = {
        {64, 0, 0.0}, {128, 0, 0.0}, {256, 0, 0.0}, {512, 0, 0.0}, {1024, 0, 0.0}
    };

    uint64_t iteration = rl_checkpoint_load(options->checkpoint_path, arms, RL_ARM_COUNT);

    uint32_t rng_state = options->rng_seed ? options->rng_seed : (uint32_t)time(NULL);
    if (rng_state == 0) {
        rng_state = 1; /* xorshift32 requires a nonzero state */
    }
    rl_log_message(RL_LOG_INFO, "RNG seed=%u (%s)", rng_state,
                   options->rng_seed ? "explicit" : "time-derived");

    rl_results_open(options->results_path);

    char line[512];

    while (true) {
        int arm_index = rl_choose_ucb_arm(arms, RL_ARM_COUNT);
        int steps = arms[arm_index].steps;
        uint32_t span = seed_hi - seed_lo + 1;
        uint32_t seed = seed_lo + (rl_rng_next(&rng_state) % span);

        triple_result_t result;
        memset(&result, 0, sizeof(result));
        result.steps = steps;

        bool any_timeout = false;
        bool any_parse_error = false;

        if (rigol && rigol->enabled) {
            rigol_arm_single_capture(rigol);
        }

        for (int b = 0; b < RL_BOARD_COUNT; ++b) {
            serial_send_run(&serial, b, seed, steps);

            while (true) {
                if (!rl_read_one_response(&serial, line, (int)sizeof(line))) {
                    continue;
                }

                if (strcmp(line, "T\n") == 0 || strcmp(line, "T\r\n") == 0) {
                    rl_log_message(RL_LOG_WARN, "[TIMEOUT] board %d", b);
                    any_timeout = true;
                    break;
                }

                if (strncmp(line, "DONE", 4) == 0) {
                    if (!rl_parse_done_line(line, b, &result)) {
                        rl_log_message(RL_LOG_WARN, "[PARSE ERROR] board %d line=%s", b, line);
                        any_parse_error = true;
                    }
                    break;
                }
            }

            if (any_timeout || any_parse_error) {
                break;
            }
        }

        if (any_timeout || any_parse_error || !result.ok) {
            rl_log_message(RL_LOG_WARN,
                           "iter=%" PRIu64 " seed=%u steps=%d skipped",
                           iteration,
                           seed,
                           steps);
            iteration++;
            continue;
        }

        wave_diff_summary_t wave_summary;
        memset(&wave_summary, 0, sizeof(wave_summary));
        wave_diff_summary_t *wave_summary_ptr = NULL;

        if (rigol && rigol->enabled) {
            waveform_capture_set_t capture = rigol_capture_scope_set(rigol);
            waveform_t aligned[RL_BOARD_COUNT];
            memset(aligned, 0, sizeof(aligned));

            wave_summary = waveform_align_and_analyze_capture_set(
                &capture,
                rigol->trigger_threshold_v,
                rigol->pre_trigger_samples,
                rigol->window_samples,
                aligned);

            if (wave_summary.valid) {
                wave_summary_ptr = &wave_summary;

                for (int i = 0; i < RL_BOARD_COUNT; ++i) {
                    result.wave[i] = aligned[i].metrics;
                }

                waveform_log_pair("01", &wave_summary.pair01);
                waveform_log_pair("02", &wave_summary.pair02);
                waveform_log_pair("12", &wave_summary.pair12);

                rl_log_message(RL_LOG_INFO,
                               "Aligned scope window: trigger_idx=%zu start=%zu samples=%zu total=%.6f",
                               wave_summary.trigger_index,
                               wave_summary.window_start,
                               wave_summary.window_samples,
                               wave_summary.grand_total);
            } else {
                rl_log_message(RL_LOG_WARN, "Aligned scope analysis unavailable for this iteration");
            }

            for (int i = 0; i < RL_BOARD_COUNT; ++i) {
                waveform_free(&aligned[i]);
            }
            waveform_capture_set_free(&capture);
        }

        double digital = rl_compute_digital_divergence(&result);
        double reward = rl_compute_combined_reward(&result, wave_summary_ptr);
        rl_update_arm(&arms[arm_index], reward);

        rl_log_message(RL_LOG_INFO,
                       "iter=%" PRIu64 " seed=%u steps=%d reward=%.2f digital=%.2f arm_steps=%d pulls=%" PRIu64 " mean=%.2f",
                       iteration,
                       seed,
                       steps,
                       reward,
                       digital,
                       arms[arm_index].steps,
                       arms[arm_index].pulls,
                       arms[arm_index].mean_reward);

        rl_results_write_row(iteration, &result, arms[arm_index].steps, arms[arm_index].pulls,
                             arms[arm_index].mean_reward, digital, reward, wave_summary_ptr);

        iteration++;

        if (options->checkpoint_path && (iteration % (uint64_t)checkpoint_interval) == 0) {
            rl_checkpoint_save(options->checkpoint_path, arms, RL_ARM_COUNT, iteration);
        }
    }

    rl_results_close();
    serial_close(&serial);
    return 0;
}
