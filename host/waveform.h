/*
===============================================================================
File        : waveform.h
Language    : C (C11-compatible style)
Target      : Portable analysis logic
Purpose     : Waveform metrics, pairwise comparison, and triplet analysis

Description
-------------------------------------------------------------------------------
This module contains the signal-analysis layer for the differential fuzzing
lab. It computes summary metrics for one captured waveform and compares the
waveforms associated with N sequential board runs for the same test case.

Design Notes
-------------------------------------------------------------------------------
- This module is intentionally independent from serial and Rigol transport.
- It can be reused with live captures, saved traces, or simulated waveforms.
- All memory ownership rules for waveform_t buffers are documented below.
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
Represents one acquired or synthesized waveform.

Ownership:
  - time_s and volts are heap-allocated when present.
  - waveform_free must be called when the waveform is no longer needed.
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
wave_pair_diff_t
-------------------------------------------------------------------------------
Contains pairwise difference metrics between two waveforms.
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
Contains the full triplet comparison summary for three board waveforms.
-------------------------------------------------------------------------------
*/
typedef struct {
    wave_pair_diff_t pair01;
    wave_pair_diff_t pair02;
    wave_pair_diff_t pair12;
    double scalar_total;
    double trace_total;
    double grand_total;
    bool valid;
} wave_diff_summary_t;

/*
-------------------------------------------------------------------------------
waveform_compute_energy_proxy
-------------------------------------------------------------------------------
Computes a simple energy-like integral proportional to the squared voltage.

Parameters:
  volts        - waveform voltage samples.
  sample_count - number of samples in volts.
  dt_s         - time step between adjacent samples.

Returns:
  Energy-like scalar suitable for relative comparison.
-------------------------------------------------------------------------------
*/
double waveform_compute_energy_proxy(const double *volts, size_t sample_count, double dt_s);

/*
-------------------------------------------------------------------------------
waveform_compute_abs_area
-------------------------------------------------------------------------------
Computes the absolute area under the waveform magnitude.

Parameters:
  volts        - waveform voltage samples.
  sample_count - number of samples in volts.
  dt_s         - time step between adjacent samples.

Returns:
  Absolute-area scalar suitable for relative comparison.
-------------------------------------------------------------------------------
*/
double waveform_compute_abs_area(const double *volts, size_t sample_count, double dt_s);

/*
-------------------------------------------------------------------------------
waveform_compute_metrics
-------------------------------------------------------------------------------
Computes the summary metrics for one waveform.

Parameters:
  volts        - waveform voltage samples.
  sample_count - number of samples in volts.
  dt_s         - time step between adjacent samples.

Returns:
  Filled wave_metrics_t structure. The valid field indicates success.
-------------------------------------------------------------------------------
*/
wave_metrics_t waveform_compute_metrics(const double *volts, size_t sample_count, double dt_s);

/*
-------------------------------------------------------------------------------
waveform_compare_pair
-------------------------------------------------------------------------------
Compares two waveforms using normalized scalar deltas and an L1 trace-shape
comparison over the overlapping sample region.

Parameters:
  a - first waveform.
  b - second waveform.

Returns:
  Filled wave_pair_diff_t structure. The valid field indicates success.
-------------------------------------------------------------------------------
*/
wave_pair_diff_t waveform_compare_pair(const waveform_t *a, const waveform_t *b);

/*
-------------------------------------------------------------------------------
waveform_analyze_triplet
-------------------------------------------------------------------------------
Analyzes a set of three board waveforms and produces a full pairwise summary.

Parameters:
  waveforms - array of RL_BOARD_COUNT waveforms.

Returns:
  Triplet-level summary structure.
-------------------------------------------------------------------------------
*/
wave_diff_summary_t waveform_analyze_triplet(const waveform_t waveforms[RL_BOARD_COUNT]);

/*
-------------------------------------------------------------------------------
waveform_free
-------------------------------------------------------------------------------
Releases memory owned by a waveform_t and resets its fields.

Parameters:
  waveform - waveform object whose internal buffers should be freed.
-------------------------------------------------------------------------------
*/
void waveform_free(waveform_t *waveform);

/*
-------------------------------------------------------------------------------
waveform_log_pair
-------------------------------------------------------------------------------
Logs the contents of a wave_pair_diff_t using the shared logging subsystem.

Parameters:
  name - descriptive label such as "01" or "0-1".
  pair - pairwise difference structure to log.
-------------------------------------------------------------------------------
*/
void waveform_log_pair(const char *name, const wave_pair_diff_t *pair);

#endif
