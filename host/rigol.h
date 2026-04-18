/*
===============================================================================
File        : rigol.h
Language    : C (C11-compatible style)
Target      : Windows host PC / Winsock TCP client
Purpose     : Rigol DS-class scope capture interface

Description
-------------------------------------------------------------------------------
This module provides the scope-side measurement interface used by the RL host.
It connects to the Rigol scope over TCP, sends SCPI commands, retrieves a raw
waveform, converts it into engineering units, and returns a waveform_t object
for downstream comparison.

Design Notes
-------------------------------------------------------------------------------
- This header exposes only the public capture configuration and API.
- SCPI transport details remain hidden in rigol.c.
- The waveform processing itself is delegated to waveform.c.
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
Describes the scope endpoint and capture settings used by the host.
-------------------------------------------------------------------------------
*/
typedef struct {
    const char *scope_ip;
    const char *scope_port;
    const char *channel;
    bool enabled;
} rigol_config_t;

/*
-------------------------------------------------------------------------------
rigol_net_init
-------------------------------------------------------------------------------
Initializes any operating-system networking state required by the Rigol client.
On Windows, this initializes Winsock.
-------------------------------------------------------------------------------
*/
void rigol_net_init(void);

/*
-------------------------------------------------------------------------------
rigol_net_cleanup
-------------------------------------------------------------------------------
Releases operating-system networking resources associated with rigol_net_init.
-------------------------------------------------------------------------------
*/
void rigol_net_cleanup(void);

/*
-------------------------------------------------------------------------------
rigol_capture_waveform
-------------------------------------------------------------------------------
Captures one waveform from the configured scope source and returns it as a
waveform_t structure containing time, voltage, and derived metrics.

Parameters:
  config - capture configuration for the target scope and channel.

Returns:
  A waveform_t object. If capture is disabled or fails to produce a valid
  waveform, the returned structure may be zero-initialized or invalid.
-------------------------------------------------------------------------------
*/
waveform_t rigol_capture_waveform(const rigol_config_t *config);

#endif
