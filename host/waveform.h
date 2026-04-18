/*
===============================================================================
File        : waveform.h
Language    : C (C11-compatible style)
Target      : Portable analysis logic
Purpose     : Waveform metrics, trigger alignment, window extraction, and
              differential comparison for the RISC-V fuzzing lab

Description
-------------------------------------------------------------------------------
This module owns the signal-analysis layer for scope captures produced during
one differential fuzzing experiment. A full scope capture contains:

  - CH1 : Board A power
  - CH2 : Board B power
  - CH3 : Board C power
  - CH4 : FPGA trigger

The trigger channel is used to define a consistent comparison window. That same
window is then extracted from the three power channels so pairwise waveform
comparisons are based on the same time region.

Design Notes
-------------------------------------------------------------------------------
- This module is transport-independent and does not depend on Rigol SCPI code.
- It can operate on live captures, replayed CSV traces, or synthetic data.
- waveform_t owns heap buffers and must be released with waveform_free().
===============================================================================
*/
#ifndef WAVEFORM_H
#define WAVEFORM_H

#include "common.h"

/*
-------------------------------------------------------------------------------
wave_metrics_t
-------------------------------------------------------------------------------
Summary statistics derived from a waveform.
-------------------------------------------------------------------------------
*/
typedef struct {
    double energy_proxy;
    double abs_area;
    double rms;
    double peak_abs;
    double mean;
    double duration_s;
    size_t sample_count;
    bool valid;
} wave_metrics_t;

/*
-------------------------------------------------------------------------------
waveform_t
-------------------------------------------------------------------------------
Represents one uniformly sampled waveform.

Ownership:
  - time_s and volts are heap-allocated when present.
  - waveform_free() must be called when the waveform is no longer needed.
-------------------------------------------------------------------------------
*/
typedef struct {
    double *time_s;
    double *volts;
    size_t sample_count;
    double dt_s;
    wave_metrics_t metrics;
} waveform_t;

/*
-------------------------------------------------------------------------------
waveform_capture_set_t
-------------------------------------------------------------------------------
Represents one four-channel scope capture for a complete experiment.
-------------------------------------------------------------------------------
*/
typedef struct {
    waveform_t power[RL_BOARD_COUNT];
    waveform_t trigger;
    bool valid;
} waveform_capture_set_t;

/*
-------------------------------------------------------------------------------
wave_pair_diff_t
-------------------------------------------------------------------------------
Contains pairwise difference metrics between two aligned power waveforms.
-------------------------------------------------------------------------------
*/
typedef struct {
    double d_energy;
    double d_area;
    double d_rms;
    double d_peak;
    double trace_l1;
    double total;
    bool valid;
} wave_pair_diff_t;

/*
-------------------------------------------------------------------------------
wave_diff_summary_t
-------------------------------------------------------------------------------
Contains the full triplet comparison summary for three aligned board waveforms.
-------------------------------------------------------------------------------
*/
typedef struct {
    wave_pair_diff_t pair01;
    wave_pair_diff_t pair02;
    wave_pair_diff_t pair12;
    double scalar_total;
    double trace_total;
    double grand_total;
    size_t trigger_index;
    size_t window_start;
    size_t window_samples;
    bool valid;
} wave_diff_summary_t;

/* Scalar metric helpers. */
double waveform_compute_energy_proxy(const double *volts, size_t sample_count, double dt_s);
double waveform_compute_abs_area(const double *volts, size_t sample_count, double dt_s);
wave_metrics_t waveform_compute_metrics(const double *volts, size_t sample_count, double dt_s);

/*
-------------------------------------------------------------------------------
waveform_find_rising_edge_index
-------------------------------------------------------------------------------
Finds the first rising-edge sample index at which the waveform crosses the
specified threshold from below to at-or-above the threshold.

Parameters:
  trigger_wave - trigger waveform to inspect.
  threshold_v  - trigger threshold in volts.
  found_out    - optional output indicating whether a crossing was found.

Returns:
  Sample index of the first threshold crossing. Returns 0 if no edge was found.
-------------------------------------------------------------------------------
*/
size_t waveform_find_rising_edge_index(const waveform_t *trigger_wave, double threshold_v, bool *found_out);

/*
-------------------------------------------------------------------------------
waveform_extract_window
-------------------------------------------------------------------------------
Extracts a contiguous sample window from a source waveform.

Parameters:
  source         - source waveform.
  start_index    - first sample index to copy from the source.
  window_samples - number of samples to extract.
  out_window     - destination waveform that receives a deep copy.

Returns:
  true on success, false if the request is invalid or cannot be satisfied.
-------------------------------------------------------------------------------
*/
bool waveform_extract_window(const waveform_t *source,
                             size_t start_index,
                             size_t window_samples,
                             waveform_t *out_window);

/*
-------------------------------------------------------------------------------
waveform_compare_pair
-------------------------------------------------------------------------------
Compares two aligned waveforms using normalized scalar deltas and an L1 trace
comparison over the overlapping sample region.
-------------------------------------------------------------------------------
*/
wave_pair_diff_t waveform_compare_pair(const waveform_t *a, const waveform_t *b);

/*
-------------------------------------------------------------------------------
waveform_analyze_triplet
-------------------------------------------------------------------------------
Analyzes an already-aligned triplet of power waveforms and produces a full
pairwise summary.
-------------------------------------------------------------------------------
*/
wave_diff_summary_t waveform_analyze_triplet(const waveform_t waveforms[RL_BOARD_COUNT]);

/*
-------------------------------------------------------------------------------
waveform_align_and_analyze_capture_set
-------------------------------------------------------------------------------
Uses the trigger channel to define a common window, extracts that same window
from CH1-CH3, and returns a pairwise differential summary.

Parameters:
  capture             - full four-channel scope capture.
  trigger_threshold_v - rising-edge threshold used on CH4.
  pre_trigger_samples - samples to retain before the trigger crossing.
  window_samples      - number of samples to keep in the aligned window.
  aligned_out         - optional array that receives aligned CH1-CH3 waveforms.

Returns:
  Triplet summary for the aligned power windows. The valid field indicates
  whether alignment and comparison succeeded.
-------------------------------------------------------------------------------
*/
wave_diff_summary_t waveform_align_and_analyze_capture_set(
    const waveform_capture_set_t *capture,
    double trigger_threshold_v,
    size_t pre_trigger_samples,
    size_t window_samples,
    waveform_t aligned_out[RL_BOARD_COUNT]);

/* Memory-management helpers. */
void waveform_free(waveform_t *waveform);
void waveform_capture_set_free(waveform_capture_set_t *capture);

/* Logging helpers. */
void waveform_log_pair(const char *name, const wave_pair_diff_t *pair);

#endif
