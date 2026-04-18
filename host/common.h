/*
===============================================================================
File        : common.h
Language    : C (C11-compatible style)
Target      : Windows host PC / MinGW-w64
Purpose     : Shared constants, utility functions, and lightweight logging API

Description
-------------------------------------------------------------------------------
This header contains definitions that are intentionally shared across the
serial transport, Rigol scope client, waveform analysis, reinforcement
learning controller, and command-line entry point.

Design Notes
-------------------------------------------------------------------------------
- Keep only truly shared types and functions here.
- Avoid placing module-specific structs in this file.
- The logging helpers provide a single place to control message formatting
  and output destinations for the host application.
===============================================================================
*/

#ifndef COMMON_H
#define COMMON_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>

#define RL_BOARD_COUNT 3
#define RL_ARM_COUNT   5

/*
-------------------------------------------------------------------------------
rl_log_level_t
-------------------------------------------------------------------------------
Defines the severity of a log message emitted by the host application.
-------------------------------------------------------------------------------
*/
typedef enum {
    RL_LOG_DEBUG = 0,
    RL_LOG_INFO,
    RL_LOG_WARN,
    RL_LOG_ERROR
} rl_log_level_t;

/*
-------------------------------------------------------------------------------
rl_log_set_level
-------------------------------------------------------------------------------
Sets the minimum severity that will be emitted by the logging subsystem.

Parameters:
  level - minimum log severity to print.
-------------------------------------------------------------------------------
*/
void rl_log_set_level(rl_log_level_t level);

/*
-------------------------------------------------------------------------------
rl_log_set_file
-------------------------------------------------------------------------------
Redirects log output to the provided file path. If the file cannot be opened,
logging continues on stderr.

Parameters:
  path - path to a log file, or NULL to keep the current sink.
-------------------------------------------------------------------------------
*/
void rl_log_set_file(const char *path);

/*
-------------------------------------------------------------------------------
rl_log_close
-------------------------------------------------------------------------------
Closes any owned log file handle and restores logging to stderr.
-------------------------------------------------------------------------------
*/
void rl_log_close(void);

/*
-------------------------------------------------------------------------------
rl_log_message
-------------------------------------------------------------------------------
Emits a formatted log record when the message severity is at or above the
active logging threshold.

Parameters:
  level  - severity of the message.
  fmt    - printf-style format string.
  ...    - format arguments.
-------------------------------------------------------------------------------
*/
void rl_log_message(rl_log_level_t level, const char *fmt, ...);

/*
-------------------------------------------------------------------------------
rl_log_vmessage
-------------------------------------------------------------------------------
Internal/helper variant of rl_log_message that accepts a va_list.

Parameters:
  level  - severity of the message.
  fmt    - printf-style format string.
  args   - argument list corresponding to fmt.
-------------------------------------------------------------------------------
*/
void rl_log_vmessage(rl_log_level_t level, const char *fmt, va_list args);

/*
-------------------------------------------------------------------------------
die
-------------------------------------------------------------------------------
Logs a fatal error and terminates the process.

Parameters:
  msg - fatal message describing the unrecoverable error.
-------------------------------------------------------------------------------
*/
void die(const char *msg);

#endif
