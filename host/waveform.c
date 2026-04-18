#include "waveform.h"

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

/*
-------------------------------------------------------------------------------
waveform_safe_rel_delta
-------------------------------------------------------------------------------
Computes a numerically safe normalized difference between two scalar values.
-------------------------------------------------------------------------------
*/
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
        sum += volts[i];
        double abs_value = fabs(volts[i]);
        if (abs_value > peak) {
            peak = abs_value;
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

void waveform_log_pair(const char *name, const wave_pair_diff_t *pair) {
    if (!pair || !pair->valid) {
        rl_log_message(RL_LOG_WARN, "[WAVE %s] unavailable", name ? name : "?");
        return;
    }

    rl_log_message(RL_LOG_INFO,
                   "[WAVE %s] dE=%.6f dA=%.6f dRMS=%.6f dPeak=%.6f traceL1=%.6f total=%.6f",
                   name,
                   pair->d_energy,
                   pair->d_area,
                   pair->d_rms,
                   pair->d_peak,
                   pair->trace_l1,
                   pair->total);
}
