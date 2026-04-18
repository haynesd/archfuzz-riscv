/*
===============================================================================
File        : main.c
Language    : C (C11-compatible style)
Target      : Windows host PC
Purpose     : Command-line entry point for the RL host controller

Description
-------------------------------------------------------------------------------
This file owns command-line parsing, usage text, top-level logging
configuration, and mode dispatch into the RL controller.

Modes
-------------------------------------------------------------------------------
  ping      - send PING to one board and wait for PONG
  run1      - execute one RUN command on one board
  rl        - run the architectural-only RL loop
  rl_scope  - run the RL loop with Rigol waveform capture and comparison
===============================================================================
*/

#include "serial.h"
#include "rigol.h"
#include "rl.h"

#include <string.h>

/*
-------------------------------------------------------------------------------
print_usage_and_exit
-------------------------------------------------------------------------------
Prints the supported command-line syntax and terminates the process.
-------------------------------------------------------------------------------
*/
static void print_usage_and_exit(void) {
    die(
        "Usage:\n"
        "  rl_host.exe ping <COM_PORT> <BOARD>\n"
        "  rl_host.exe run1 <COM_PORT> <BOARD> <SEED> <STEPS>\n"
        "  rl_host.exe rl <COM_PORT> <SEED_START> <SEED_END>\n"
        "  rl_host.exe rl_scope <COM_PORT> <SEED_START> <SEED_END> <SCOPE_IP> <SCOPE_PORT> <SCOPE_CHANNEL>\n\n"
        "Optional logging:\n"
        "  rl_host.exe --debug ...\n"
        "  rl_host.exe --log <FILE> ...\n\n"
        "Examples:\n"
        "  rl_host.exe ping COM5 0\n"
        "  rl_host.exe run1 COM5 0 12345 256\n"
        "  rl_host.exe rl COM5 16 65536\n"
        "  rl_host.exe rl_scope COM5 16 65536 192.168.1.55 5555 CHAN1\n"
    );
}

int main(int argc, char **argv) {
    int argi = 1;

    while (argi < argc) {
        if (strcmp(argv[argi], "--debug") == 0) {
            rl_log_set_level(RL_LOG_DEBUG);
            ++argi;
            continue;
        }
        if (strcmp(argv[argi], "--log") == 0) {
            if (argi + 1 >= argc) {
                print_usage_and_exit();
            }
            rl_log_set_file(argv[argi + 1]);
            argi += 2;
            continue;
        }
        break;
    }

    if (argc - argi < 1) {
        print_usage_and_exit();
    }

    int rc = 0;
    if (strcmp(argv[argi], "ping") == 0) {
        if (argc - argi != 3) {
            print_usage_and_exit();
        }
        rc = rl_mode_ping(argv[argi + 1], parse_board_index(argv[argi + 2]));
    } else if (strcmp(argv[argi], "run1") == 0) {
        if (argc - argi != 5) {
            print_usage_and_exit();
        }
        rc = rl_mode_run1(argv[argi + 1],
                          parse_board_index(argv[argi + 2]),
                          parse_u32_decimal(argv[argi + 3], "SEED"),
                          parse_int_decimal(argv[argi + 4], "STEPS"));
    } else if (strcmp(argv[argi], "rl") == 0) {
        if (argc - argi != 4) {
            print_usage_and_exit();
        }
        rc = rl_mode_loop(argv[argi + 1],
                          parse_u32_decimal(argv[argi + 2], "SEED_START"),
                          parse_u32_decimal(argv[argi + 3], "SEED_END"),
                          NULL);
    } else if (strcmp(argv[argi], "rl_scope") == 0) {
        if (argc - argi != 7) {
            print_usage_and_exit();
        }
        rigol_config_t rigol = { argv[argi + 4], argv[argi + 5], argv[argi + 6], true };
        rigol_net_init();
        rc = rl_mode_loop(argv[argi + 1],
                          parse_u32_decimal(argv[argi + 2], "SEED_START"),
                          parse_u32_decimal(argv[argi + 3], "SEED_END"),
                          &rigol);
        rigol_net_cleanup();
    } else {
        print_usage_and_exit();
    }

    rl_log_close();
    return rc;
}
