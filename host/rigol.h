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
    bool enabled;
} rigol_config_t;

void rigol_net_init(void);
void rigol_net_cleanup(void);

/*
-------------------------------------------------------------------------------
rigol_arm_single_capture
-------------------------------------------------------------------------------
Arms the scope for one single-shot acquisition before the FPGA sequence begins.
This is useful when the FPGA trigger pulse should define the captured record.
-------------------------------------------------------------------------------
*/
void rigol_arm_single_capture(const rigol_config_t *config);

/*
-------------------------------------------------------------------------------
rigol_capture_scope_set
-------------------------------------------------------------------------------
Reads CH1-CH4 from the scope after an experiment completes and returns the full
synchronized capture set.
-------------------------------------------------------------------------------
*/
waveform_capture_set_t rigol_capture_scope_set(const rigol_config_t *config);

#endif
