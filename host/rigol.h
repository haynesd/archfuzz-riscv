/*
===============================================================================
File        : rigol.h
Language    : C (C11-compatible style)
Target      : Windows host PC / Winsock TCP client
Purpose     : Rigol DS-class scope transport and multi-channel capture API

Description
-------------------------------------------------------------------------------
This module owns the SCPI/TCP transport used to communicate with the Rigol
scope. It captures one full experiment as a synchronized four-channel data set:

  - CH1 : Board A power
  - CH2 : Board B power
  - CH3 : Board C power
  - CH4 : FPGA trigger

The trigger channel is intentionally captured alongside the power channels so
window alignment can be performed after acquisition.
===============================================================================
*/
#ifndef RIGOL_H
#define RIGOL_H

#include "common.h"
#include "waveform.h"

/*
-------------------------------------------------------------------------------
rigol_config_t
-------------------------------------------------------------------------------
Describes the scope endpoint and channel assignments used by the host.
-------------------------------------------------------------------------------
*/
typedef struct {
    const char *scope_ip;
    const char *scope_port;
    const char *power_channels[RL_BOARD_COUNT];
    const char *trigger_channel;
    double trigger_threshold_v;
    size_t pre_trigger_samples;
    size_t window_samples;
    /*
     * Optional explicit timebase configuration, sent via SCPI before arming
     * the single-shot capture. A value of 0 leaves the scope's timebase
     * exactly as manually configured on the front panel (prior behavior).
     * Set both when the acquisition window must be scripted/reproducible,
     * e.g. to guarantee CH1-CH3 activity from all sequential board runs
     * lands inside the single captured record.
     */
    double timebase_scale_s;
    double timebase_offset_s;
    /*
     * Optional explicit acquisition memory depth (points per channel), sent
     * via ":ACQ:MDEP <N>" before arming. 0 leaves memory depth exactly as
     * manually configured on the front panel (prior behavior). Needed
     * because a large window_samples request silently gets truncated by the
     * scope once it exceeds whatever memory depth happens to already be
     * configured - :WAV:STAR/:WAV:STOP can only select a sub-range of what
     * was actually acquired, they cannot make the scope acquire more than
     * its current memory depth allows. Discovered by requesting a window
     * of ~500k samples and getting back only ~152k regardless. Only
     * specific values are valid per Rigol's own memory-depth menu (varies
     * by model and by how many channels are active) - if the value here
     * doesn't match one of those, the scope will typically round it to the
     * nearest supported value rather than reject it outright, so check the
     * "Captured ...: samples=" log line against what you asked for.
     */
    size_t memory_depth_points;
    bool enabled;
} rigol_config_t;

void rigol_net_init(void);
void rigol_net_cleanup(void);

/*
-------------------------------------------------------------------------------
rigol_session_t
-------------------------------------------------------------------------------
Opaque handle for one held-open scope connection, spanning arm -> board
activity -> read. The scope's SCPI/LAN server appears to cancel a pending
single-shot trigger arm if the TCP connection that armed it is closed before
the result is read back, so the same connection used to send :SING must stay
open through the FPGA/board sequence and into the eventual capture read.
-------------------------------------------------------------------------------
*/
typedef struct rigol_session rigol_session_t;

/*
-------------------------------------------------------------------------------
rigol_arm_single_capture
-------------------------------------------------------------------------------
Arms the scope for one single-shot acquisition before the FPGA sequence begins.
This is useful when the FPGA trigger pulse should define the captured record.

Returns an open session to pass to rigol_capture_scope_set() (or
rigol_session_abandon() if the capture ends up not being needed), or NULL if
config is disabled.
-------------------------------------------------------------------------------
*/
rigol_session_t *rigol_arm_single_capture(const rigol_config_t *config);

/*
-------------------------------------------------------------------------------
rigol_capture_scope_set
-------------------------------------------------------------------------------
Reads CH1-CH4 from the scope after an experiment completes and returns the full
synchronized capture set. Consumes (closes and frees) the session regardless
of outcome; the pointer must not be reused afterward.
-------------------------------------------------------------------------------
*/
waveform_capture_set_t rigol_capture_scope_set(rigol_session_t *session);

/*
-------------------------------------------------------------------------------
rigol_session_abandon
-------------------------------------------------------------------------------
Closes and frees a session returned by rigol_arm_single_capture() without
performing a capture, e.g. when the board round that would have defined the
capture window was skipped or failed. Safe to call with NULL.
-------------------------------------------------------------------------------
*/
void rigol_session_abandon(rigol_session_t *session);

#endif
