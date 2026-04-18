#include "waveform.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static double waveform_safe_rel_delta(double a, double b) {
    double denom = fmax(fabs(a), fabs(b));
    if (denom < 1e-18) {
        return 0.0;
    }
    return fabs(a - b) / denom;
}

double waveform_compute_energy_proxy(const double *volts, size_t sample_count, double dt_s) {
    double energy = 0.0;
    for (size_t i = 0; i < sample_count; ++i) {
        energy += volts[i] * volts[i] * dt_s;
    }
    return energy;
}

double waveform_compute_abs_area(const double *volts, size_t sample_count, double dt_s) {
    double area = 0.0;
    for (size_t i = 0; i < sample_count; ++i) {
        area += fabs(volts[i]) * dt_s;
    }
    return area;
}

wave_metrics_t waveform_compute_metrics(const double *volts, size_t sample_count, double dt_s) {
    wave_metrics_t metrics;
    memset(&metrics, 0, sizeof(metrics));

    if (!volts || sample_count == 0 || dt_s <= 0.0) {
        return metrics;
    }

    double sum = 0.0;
    double peak = 0.0;

    for (size_t i = 0; i < sample_count; ++i) {
        const double v = volts[i];
        const double av = fabs(v);
        sum += v;
        if (av > peak) {
            peak = av;
        }
    }

    metrics.mean = sum / (double)sample_count;
    metrics.energy_proxy = waveform_compute_energy_proxy(volts, sample_count, dt_s);
    metrics.abs_area = waveform_compute_abs_area(volts, sample_count, dt_s);
    metrics.rms = sqrt(metrics.energy_proxy / ((double)sample_count * dt_s));
    metrics.peak_abs = peak;
    metrics.duration_s = (double)sample_count * dt_s;
    metrics.sample_count = sample_count;
    metrics.valid = true;
    return metrics;
}

size_t waveform_find_rising_edge_index(const waveform_t *trigger_wave, double threshold_v, bool *found_out) {
    if (found_out) {
        *found_out = false;
    }

    if (!trigger_wave || !trigger_wave->volts || trigger_wave->sample_count < 2) {
        return 0;
    }

    for (size_t i = 1; i < trigger_wave->sample_count; ++i) {
        const double prev = trigger_wave->volts[i - 1];
        const double curr = trigger_wave->volts[i];
        if (prev < threshold_v && curr >= threshold_v) {
            if (found_out) {
                *found_out = true;
            }
            return i;
        }
    }

    return 0;
}

bool waveform_extract_window(const waveform_t *source,
                             size_t start_index,
                             size_t window_samples,
                             waveform_t *out_window) {
    if (!out_window) {
        return false;
    }

    memset(out_window, 0, sizeof(*out_window));

    if (!source || !source->time_s || !source->volts || source->sample_count == 0) {
        return false;
    }
    if (window_samples == 0 || start_index >= source->sample_count) {
        return false;
    }

    size_t max_available = source->sample_count - start_index;
    size_t count = window_samples < max_available ? window_samples : max_available;
    if (count == 0) {
        return false;
    }

    out_window->time_s = (double *)malloc(count * sizeof(double));
    out_window->volts = (double *)malloc(count * sizeof(double));
    if (!out_window->time_s || !out_window->volts) {
        waveform_free(out_window);
        return false;
    }

    for (size_t i = 0; i < count; ++i) {
        out_window->time_s[i] = source->time_s[start_index + i];
        out_window->volts[i] = source->volts[start_index + i];
    }

    out_window->sample_count = count;
    out_window->dt_s = source->dt_s;
    out_window->metrics = waveform_compute_metrics(out_window->volts, count, out_window->dt_s);
    return out_window->metrics.valid;
}

wave_pair_diff_t waveform_compare_pair(const waveform_t *a, const waveform_t *b) {
    wave_pair_diff_t diff;
    memset(&diff, 0, sizeof(diff));

    if (!a || !b || !a->metrics.valid || !b->metrics.valid) {
        return diff;
    }

    diff.d_energy = waveform_safe_rel_delta(a->metrics.energy_proxy, b->metrics.energy_proxy);
    diff.d_area = waveform_safe_rel_delta(a->metrics.abs_area, b->metrics.abs_area);
    diff.d_rms = waveform_safe_rel_delta(a->metrics.rms, b->metrics.rms);
    diff.d_peak = waveform_safe_rel_delta(a->metrics.peak_abs, b->metrics.peak_abs);

    size_t overlap = (a->sample_count < b->sample_count) ? a->sample_count : b->sample_count;
    if (overlap > 0) {
        double l1 = 0.0;
        for (size_t i = 0; i < overlap; ++i) {
            double xa = a->volts[i] - a->metrics.mean;
            double xb = b->volts[i] - b->metrics.mean;
            l1 += fabs(xa - xb);
        }
        diff.trace_l1 = l1 / (double)overlap;
    }

    diff.total = diff.d_energy + diff.d_area + diff.d_rms + diff.d_peak + diff.trace_l1;
    diff.valid = true;
    return diff;
}

wave_diff_summary_t waveform_analyze_triplet(const waveform_t waveforms[RL_BOARD_COUNT]) {
    wave_diff_summary_t summary;
    memset(&summary, 0, sizeof(summary));

    summary.pair01 = waveform_compare_pair(&waveforms[0], &waveforms[1]);
    summary.pair02 = waveform_compare_pair(&waveforms[0], &waveforms[2]);
    summary.pair12 = waveform_compare_pair(&waveforms[1], &waveforms[2]);

    if (!(summary.pair01.valid && summary.pair02.valid && summary.pair12.valid)) {
        return summary;
    }

    summary.scalar_total =
        (summary.pair01.d_energy + summary.pair01.d_area + summary.pair01.d_rms + summary.pair01.d_peak) +
        (summary.pair02.d_energy + summary.pair02.d_area + summary.pair02.d_rms + summary.pair02.d_peak) +
        (summary.pair12.d_energy + summary.pair12.d_area + summary.pair12.d_rms + summary.pair12.d_peak);

    summary.trace_total = summary.pair01.trace_l1 + summary.pair02.trace_l1 + summary.pair12.trace_l1;
    summary.grand_total = summary.scalar_total + summary.trace_total;
    summary.valid = true;
    return summary;
}

wave_diff_summary_t waveform_align_and_analyze_capture_set(
    const waveform_capture_set_t *capture,
    double trigger_threshold_v,
    size_t pre_trigger_samples,
    size_t window_samples,
    waveform_t aligned_out[RL_BOARD_COUNT]) {

    wave_diff_summary_t summary;
    memset(&summary, 0, sizeof(summary));

    if (!capture || !capture->valid || !capture->trigger.metrics.valid) {
        return summary;
    }

    bool found = false;
    size_t trig = waveform_find_rising_edge_index(&capture->trigger, trigger_threshold_v, &found);
    if (!found) {
        rl_log_message(RL_LOG_WARN, "No trigger edge found on CH4 at threshold %.6f V", trigger_threshold_v);
        return summary;
    }

    size_t start = (trig > pre_trigger_samples) ? (trig - pre_trigger_samples) : 0;

    waveform_t local_aligned[RL_BOARD_COUNT];
    memset(local_aligned, 0, sizeof(local_aligned));

    for (int i = 0; i < RL_BOARD_COUNT; ++i) {
        if (!waveform_extract_window(&capture->power[i], start, window_samples, &local_aligned[i])) {
            rl_log_message(RL_LOG_WARN, "Failed to extract aligned window for power channel %d", i + 1);
            for (int j = 0; j <= i; ++j) {
                waveform_free(&local_aligned[j]);
            }
            return summary;
        }
    }

    summary = waveform_analyze_triplet(local_aligned);
    summary.trigger_index = trig;
    summary.window_start = start;
    summary.window_samples = local_aligned[0].sample_count;

    if (aligned_out) {
        for (int i = 0; i < RL_BOARD_COUNT; ++i) {
            aligned_out[i] = local_aligned[i];
        }
    } else {
        for (int i = 0; i < RL_BOARD_COUNT; ++i) {
            waveform_free(&local_aligned[i]);
        }
    }

    return summary;
}

void waveform_free(waveform_t *waveform) {
    if (!waveform) {
        return;
    }

    free(waveform->time_s);
    free(waveform->volts);
    waveform->time_s = NULL;
    waveform->volts = NULL;
    waveform->sample_count = 0;
    waveform->dt_s = 0.0;
    memset(&waveform->metrics, 0, sizeof(waveform->metrics));
}

void waveform_capture_set_free(waveform_capture_set_t *capture) {
    if (!capture) {
        return;
    }

    for (int i = 0; i < RL_BOARD_COUNT; ++i) {
        waveform_free(&capture->power[i]);
    }
    waveform_free(&capture->trigger);
    capture->valid = false;
}

void waveform_log_pair(const char *name, const wave_pair_diff_t *pair) {
    if (!pair || !pair->valid) {
        rl_log_message(RL_LOG_WARN, "[WAVE %s] unavailable", name ? name : "?");
        return;
    }

    rl_log_message(RL_LOG_INFO,
                   "[WAVE %s] dE=%.6f dA=%.6f dRMS=%.6f dPeak=%.6f traceL1=%.6f total=%.6f",
                   name ? name : "?",
                   pair->d_energy,
                   pair->d_area,
                   pair->d_rms,
                   pair->d_peak,
                   pair->trace_l1,
                   pair->total);
}
