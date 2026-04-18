/*
===============================================================================
File        : serial.h
Language    : C (C11-compatible style)
Target      : Windows host PC / Win32 serial APIs
Purpose     : Serial transport API for host-to-FPGA communication

Description
-------------------------------------------------------------------------------
This module encapsulates all COM-port interactions used by the host controller.
It is responsible for:
  - opening and configuring the serial port,
  - framing messages for the FPGA router,
  - transmitting PING and RUN commands,
  - reading line-oriented responses from the FPGA/board path, and
  - parsing common decimal command-line values.

Design Notes
-------------------------------------------------------------------------------
- The FPGA framing convention remains local to this module.
- The RL controller consumes only the public transport API below.
- Logging of TX/RX activity is performed inside the implementation.
===============================================================================
*/

#ifndef SERIAL_H
#define SERIAL_H

#include "common.h"
#include <windows.h>

/*
-------------------------------------------------------------------------------
serial_t
-------------------------------------------------------------------------------
Represents one open host-side serial connection to the FPGA controller.
-------------------------------------------------------------------------------
*/
typedef struct {
    HANDLE handle;
} serial_t;

/*
-------------------------------------------------------------------------------
serial_open
-------------------------------------------------------------------------------
Opens and configures the specified COM port for 115200-8N1 communication.

Parameters:
  port_name - COM port name such as "COM5".

Returns:
  A configured serial_t handle.
-------------------------------------------------------------------------------
*/
serial_t serial_open(const char *port_name);

/*
-------------------------------------------------------------------------------
serial_close
-------------------------------------------------------------------------------
Closes an open serial connection and invalidates the handle.

Parameters:
  serial - serial transport instance to close.
-------------------------------------------------------------------------------
*/
void serial_close(serial_t *serial);

/*
-------------------------------------------------------------------------------
serial_send_ping
-------------------------------------------------------------------------------
Transmits a framed PING command to a selected board through the FPGA router.

Parameters:
  serial      - open serial transport.
  board_index - board number in the range [0, RL_BOARD_COUNT).
-------------------------------------------------------------------------------
*/
void serial_send_ping(serial_t *serial, int board_index);

/*
-------------------------------------------------------------------------------
serial_send_run
-------------------------------------------------------------------------------
Transmits a framed RUN command to a selected board.

Parameters:
  serial      - open serial transport.
  board_index - board number in the range [0, RL_BOARD_COUNT).
  seed        - deterministic workload seed.
  steps       - workload length or stress parameter.
-------------------------------------------------------------------------------
*/
void serial_send_run(serial_t *serial, int board_index, uint32_t seed, int steps);

/*
-------------------------------------------------------------------------------
serial_read_line
-------------------------------------------------------------------------------
Reads one newline-terminated response from the serial channel.

Parameters:
  serial     - open serial transport.
  line       - destination buffer for the received line.
  line_size  - capacity of the destination buffer.

Returns:
  true  - a line was received.
  false - no bytes were received before the read cycle completed.
-------------------------------------------------------------------------------
*/
bool serial_read_line(serial_t *serial, char *line, int line_size);

/*
-------------------------------------------------------------------------------
parse_board_index
-------------------------------------------------------------------------------
Parses and validates a board index from decimal text.

Parameters:
  text - decimal string containing the board number.

Returns:
  Parsed board index.
-------------------------------------------------------------------------------
*/
int parse_board_index(const char *text);

/*
-------------------------------------------------------------------------------
parse_u32_decimal
-------------------------------------------------------------------------------
Parses a 32-bit unsigned integer from decimal text.

Parameters:
  text - decimal input string.
  what - descriptive field name used in error reporting.

Returns:
  Parsed unsigned 32-bit value.
-------------------------------------------------------------------------------
*/
uint32_t parse_u32_decimal(const char *text, const char *what);

/*
-------------------------------------------------------------------------------
parse_int_decimal
-------------------------------------------------------------------------------
Parses a signed integer from decimal text.

Parameters:
  text - decimal input string.
  what - descriptive field name used in error reporting.

Returns:
  Parsed signed integer value.
-------------------------------------------------------------------------------
*/
int parse_int_decimal(const char *text, const char *what);

#endif
