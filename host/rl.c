#include "rl.h"
#include "serial.h"

#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/*
-------------------------------------------------------------------------------
rl_read_one_response
-------------------------------------------------------------------------------
Reads one line from the serial transport and handles fatal read failures.
-------------------------------------------------------------------------------
*/
static bool rl_read_one_response(serial_t *serial, char *line, int line_size) {
    return serial_read_line(serial, line, line_size);
}

bool rl_parse_done_line(const char *line, int board_index, triple_result_t *result) {
    unsigned seed = 0;
    unsigned flags = 0;
    unsigned worst_window = 0;
    unsigned long long score = 0;
    unsigned long long worst_cycles = 0;

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

    double score_term = (d01 + d02 + d12) / 1000.0;

    double fault_bonus = 0.0;
    for (int i = 0; i < RL_BOARD_COUNT; ++i) {
        if (result->flags[i] != 0) {
            fault_bonus += 1e6;
        }
    }

    double window_bonus = 0.0;
    if (!(result->worst_window[0] == result->worst_window[1] &&
          result->worst_window[1] == result->worst_window[2])) {
        window_bonus += 2e5;
    }

    double wc_term =
        fabs((double)result->worst_cycles[0] - (double)result->worst_cycles[1]) +
        fabs((double)result->worst_cycles[0] - (double)result->worst_cycles[2]) +
        fabs((double)result->worst_cycles[1] - (double)result->worst_cycles[2]);

    return score_term + fault_bonus + window_bonus + wc_term;
}

double rl_compute_combined_reward(const triple_result_t *result, const wave_diff_summary_t *wave_summary) {
    double reward = rl_compute_digital_divergence(result);
    if (wave_summary && wave_summary->valid) {
        reward += wave_summary->grand_total;
    }
    return reward;
}

int rl_choose_ucb_arm(rl_arm_t *arms, int arm_count) {
    uint64_t total = 0;
    for (int i = 0; i < arm_count; ++i) {
        total += arms[i].pulls;
    }

    for (int i = 0; i < arm_count; ++i) {
        if (arms[i].pulls == 0) {
            return i;
        }
    }

    double best = -1e300;
    int best_i = 0;

    for (int i = 0; i < arm_count; ++i) {
        double bonus = sqrt(2.0 * log((double)total) / (double)arms[i].pulls);
        double value = arms[i].mean_reward + bonus;
        if (value > best) {
            best = value;
            best_i = i;
        }
    }

    return best_i;
}

void rl_update_arm(rl_arm_t *arm, double reward) {
    arm->pulls++;
    double alpha = 1.0 / (double)arm->pulls;
    arm->mean_reward = (1.0 - alpha) * arm->mean_reward + alpha * reward;
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

int rl_mode_loop(const char *com_port, uint32_t seed_lo, uint32_t seed_hi, const rigol_config_t *rigol) {
    if (seed_hi < seed_lo) {
        die("SEED_END must be >= SEED_START");
    }

    serial_t serial = serial_open(com_port);

    rl_arm_t arms[RL_ARM_COUNT] = {
        {64, 0, 0.0}, {128, 0, 0.0}, {256, 0, 0.0}, {512, 0, 0.0}, {1024, 0, 0.0}
    };

    srand((unsigned)time(NULL));

    char line[512];
    uint64_t iteration = 0;

    while (true) {
        int arm_index = rl_choose_ucb_arm(arms, RL_ARM_COUNT);
        int steps = arms[arm_index].steps;
        uint32_t span = seed_hi - seed_lo + 1;
        uint32_t seed = seed_lo + (uint32_t)(rand() % span);

        triple_result_t result;
        memset(&result, 0, sizeof(result));
        result.steps = steps;

        waveform_t captured[RL_BOARD_COUNT];
        memset(captured, 0, sizeof(captured));

        bool any_timeout = false;
        bool any_parse_error = false;

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

            if (rigol && rigol->enabled && !any_timeout && !any_parse_error) {
                captured[b] = rigol_capture_waveform(rigol);
                result.wave[b] = captured[b].metrics;
            }
        }

        if (any_timeout || any_parse_error || !result.ok) {
            rl_log_message(RL_LOG_WARN,
                           "iter=%" PRIu64 " seed=%u steps=%d skipped",
                           iteration++,
                           seed,
                           steps);
            for (int i = 0; i < RL_BOARD_COUNT; ++i) {
                waveform_free(&captured[i]);
            }
            continue;
        }

        wave_diff_summary_t wave_summary;
        memset(&wave_summary, 0, sizeof(wave_summary));
        wave_diff_summary_t *wave_summary_ptr = NULL;

        if (rigol && rigol->enabled) {
            wave_summary = waveform_analyze_triplet(captured);
            if (wave_summary.valid) {
                waveform_log_pair("01", &wave_summary.pair01);
                waveform_log_pair("02", &wave_summary.pair02);
                waveform_log_pair("12", &wave_summary.pair12);
                rl_log_message(RL_LOG_INFO,
                               "Wave summary: scalar=%.6f trace=%.6f total=%.6f",
                               wave_summary.scalar_total,
                               wave_summary.trace_total,
                               wave_summary.grand_total);
                wave_summary_ptr = &wave_summary;
            } else {
                rl_log_message(RL_LOG_WARN, "Wave summary unavailable for this iteration");
            }
        }

        double digital = rl_compute_digital_divergence(&result);
        double reward = rl_compute_combined_reward(&result, wave_summary_ptr);
        rl_update_arm(&arms[arm_index], reward);

        rl_log_message(RL_LOG_INFO,
                       "iter=%" PRIu64 " seed=%u steps=%d reward=%.2f digital=%.2f arm_steps=%d pulls=%" PRIu64 " mean=%.2f",
                       iteration++,
                       seed,
                       steps,
                       reward,
                       digital,
                       arms[arm_index].steps,
                       arms[arm_index].pulls,
                       arms[arm_index].mean_reward);

        for (int i = 0; i < RL_BOARD_COUNT; ++i) {
            waveform_free(&captured[i]);
        }
    }

    serial_close(&serial);
    return 0;
}
