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
  rl_scope  - run the RL loop with 4-channel Rigol capture, CH4 trigger
              alignment, and CH1-CH3 differential waveform analysis
===============================================================================
*/

#include "serial.h"
#include "rigol.h"
#include "rl.h"

#include <stdlib.h>
#include <string.h>

static void print_usage_and_exit(void) {
    die(
        "Usage:\n"
        "  rl_host.exe ping <COM_PORT> <BOARD>\n"
        "  rl_host.exe run1 <COM_PORT> <BOARD> <SEED> <STEPS> [DIAG_MODE]\n"
        "  rl_host.exe rl <COM_PORT> <SEED_START> <SEED_END>\n"
        "  rl_host.exe rl_scope <COM_PORT> <SEED_START> <SEED_END> <SCOPE_IP> <SCOPE_PORT> "
        "[PRE_TRIGGER_SAMPLES] [WINDOW_SAMPLES] [TRIGGER_THRESHOLD_V] [TIMEBASE_SCALE_S] [TIMEBASE_OFFSET_S] [MEMORY_DEPTH]\n\n"
        "MEMORY_DEPTH (rl_scope only, optional, requires both timebase args present) -\n"
        "points per channel to request via :ACQ:MDEP before arming. Needed because\n"
        "WINDOW_SAMPLES can only select a sub-range of what the scope actually\n"
        "acquired - requesting a window larger than the current memory depth just\n"
        "gets silently truncated. Check the \"Captured ...: samples=\" log line\n"
        "against what you requested; the scope may round to its nearest supported\n"
        "value rather than reject an unsupported one.\n\n"
        "rl_scope channel model:\n"
        "  CH1 = Board A power\n"
        "  CH2 = Board B power\n"
        "  CH3 = Board C power\n"
        "  CH4 = FPGA trigger (window alignment reference)\n\n"
        "Optional flags (apply to rl / rl_scope, must precede the mode name):\n"
        "  --debug                     enable debug-level logging\n"
        "  --log <FILE>                also log to FILE\n"
        "  --rng-seed <N>               seed the deterministic seed-selection PRNG\n"
        "                               (default: derived from the current time, i.e. not\n"
        "                               reproducible across runs)\n"
        "  --results <FILE>            append one CSV row per iteration to FILE\n"
        "  --checkpoint <FILE>         persist/restore UCB bandit state to FILE so a long\n"
        "                               campaign can resume after being interrupted\n"
        "  --checkpoint-interval <N>   iterations between checkpoint writes (default 20)\n"
        "  --skip-board <N>            exclude board N (0/1/2) from this session - not run,\n"
        "                              not compared - so the remaining boards can still be\n"
        "                              fuzzed/compared while board N is being debugged\n"
        "  --board-delay-ms <N>        pause N ms before addressing each board within an\n"
        "                              iteration - diagnostic for testing whether a link is\n"
        "                              being disturbed by insufficient settling time after\n"
        "                              the previous board's activity (default 0, no delay)\n"
        "  --dump-waveform <DIR>       write raw per-channel waveform samples (time_s,volts\n"
        "                              CSV) to DIR for the first few valid rl_scope\n"
        "                              iterations, so a capture can be plotted and visually\n"
        "                              confirmed as a real workload-driven current burst\n"
        "                              instead of trusting only the reduced scalar metrics\n"
        "  --dump-waveform-count <N>   number of iterations to dump before stopping\n"
        "                              (default 5). Ignored without --dump-waveform\n\n"
        "DIAG_MODE (run1 only, optional, default 0) - for manually bisecting a\n"
        "cross-board checksum divergence down to a specific workload construct:\n"
        "  0 = full workload (default): ALU + memory + AMO(1/32 steps) + branch(1/1024 steps)\n"
        "  1 = ALU-only: multiply/shift/rotate mixing lines only, memory/AMO/branch skipped\n"
        "  2 = ALU + memory, no AMO/branch\n"
        "  3 = ALU + memory + AMO, no branch\n"
        "  4 = FP-only: double-precision denormal/NaN/rounding-tie stress, isolated\n"
        "      from ALU/memory/AMO/branch entirely - see board/runner.c\n"
        "  5 = misaligned-access + CSR-read probe: deliberately misaligned 16/32-bit\n"
        "      mem[] access, plus an rdcycle/rdtime/rdinstret read that raises SIGILL\n"
        "      if the kernel's mcounteren/scounteren CSR doesn't permit it - see\n"
        "      board/runner.c\n"
        "  6 = misaligned-access only (bisects mode 5's two constructs apart)\n"
        "  7 = CSR-read only (bisects mode 5's two constructs apart)\n\n"
        "Examples:\n"
        "  rl_host.exe ping COM5 0\n"
        "  rl_host.exe run1 COM5 0 12345 256\n"
        "  rl_host.exe run1 COM5 0 12345 4096 1\n"
        "  rl_host.exe rl COM5 16 65536\n"
        "  rl_host.exe --rng-seed 42 --results run1.csv rl COM5 16 65536\n"
        "  rl_host.exe rl_scope COM5 16 65536 192.168.1.178 5555\n"
        "  rl_host.exe rl_scope COM5 16 65536 192.168.1.178 5555 32 512 1.0\n"
        "  rl_host.exe --checkpoint run1.ckpt --results run1.csv rl_scope COM5 16 65536 192.168.1.178 5555 32 512 1.0 0.001 0.0\n"
        "  rl_host.exe --dump-waveform wavedump rl_scope COM5 16 65536 192.168.1.178 5555 200 50000 0.15\n"
        "  rl_host.exe rl_scope COM5 16 65536 192.168.1.178 5555 2000 500000 0.15 0.0 0.0 3000000\n"
    );
}

int main(int argc, char **argv) {
    int argi = 1;

    rl_run_options_t options;
    memset(&options, 0, sizeof(options));
    options.skip_board = -1;

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
        if (strcmp(argv[argi], "--rng-seed") == 0) {
            if (argi + 1 >= argc) {
                print_usage_and_exit();
            }
            options.rng_seed = parse_u32_decimal(argv[argi + 1], "RNG_SEED");
            argi += 2;
            continue;
        }
        if (strcmp(argv[argi], "--results") == 0) {
            if (argi + 1 >= argc) {
                print_usage_and_exit();
            }
            options.results_path = argv[argi + 1];
            argi += 2;
            continue;
        }
        if (strcmp(argv[argi], "--checkpoint") == 0) {
            if (argi + 1 >= argc) {
                print_usage_and_exit();
            }
            options.checkpoint_path = argv[argi + 1];
            argi += 2;
            continue;
        }
        if (strcmp(argv[argi], "--checkpoint-interval") == 0) {
            if (argi + 1 >= argc) {
                print_usage_and_exit();
            }
            options.checkpoint_interval = parse_int_decimal(argv[argi + 1], "CHECKPOINT_INTERVAL");
            argi += 2;
            continue;
        }
        if (strcmp(argv[argi], "--skip-board") == 0) {
            if (argi + 1 >= argc) {
                print_usage_and_exit();
            }
            options.skip_board = parse_board_index(argv[argi + 1]);
            argi += 2;
            continue;
        }
        if (strcmp(argv[argi], "--board-delay-ms") == 0) {
            if (argi + 1 >= argc) {
                print_usage_and_exit();
            }
            options.board_delay_ms = parse_int_decimal(argv[argi + 1], "BOARD_DELAY_MS");
            argi += 2;
            continue;
        }
        if (strcmp(argv[argi], "--dump-waveform") == 0) {
            if (argi + 1 >= argc) {
                print_usage_and_exit();
            }
            options.dump_waveform_dir = argv[argi + 1];
            argi += 2;
            continue;
        }
        if (strcmp(argv[argi], "--dump-waveform-count") == 0) {
            if (argi + 1 >= argc) {
                print_usage_and_exit();
            }
            options.dump_waveform_count = parse_int_decimal(argv[argi + 1], "DUMP_WAVEFORM_COUNT");
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
        if (argc - argi != 5 && argc - argi != 6) {
            print_usage_and_exit();
        }
        int diag_mode = (argc - argi == 6) ? parse_int_decimal(argv[argi + 5], "DIAG_MODE") : 0;
        rc = rl_mode_run1(argv[argi + 1],
                          parse_board_index(argv[argi + 2]),
                          parse_u32_decimal(argv[argi + 3], "SEED"),
                          parse_int_decimal(argv[argi + 4], "STEPS"),
                          diag_mode);
    } else if (strcmp(argv[argi], "rl") == 0) {
        if (argc - argi != 4) {
            print_usage_and_exit();
        }
        rc = rl_mode_loop(argv[argi + 1],
                          parse_u32_decimal(argv[argi + 2], "SEED_START"),
                          parse_u32_decimal(argv[argi + 3], "SEED_END"),
                          NULL,
                          &options);
    } else if (strcmp(argv[argi], "rl_scope") == 0) {
        if ((argc - argi) != 6 && (argc - argi) != 9 && (argc - argi) != 11 && (argc - argi) != 12) {
            print_usage_and_exit();
        }

        rigol_config_t rigol;
        memset(&rigol, 0, sizeof(rigol));
        rigol.scope_ip = argv[argi + 4];
        rigol.scope_port = argv[argi + 5];
        rigol.power_channels[0] = "CHAN1";
        rigol.power_channels[1] = "CHAN2";
        rigol.power_channels[2] = "CHAN3";
        rigol.trigger_channel = "CHAN4";
        rigol.pre_trigger_samples = 32;
        rigol.window_samples = 512;
        rigol.trigger_threshold_v = 1.0;
        rigol.timebase_scale_s = 0.0;
        rigol.timebase_offset_s = 0.0;
        rigol.enabled = true;

        if ((argc - argi) >= 9) {
            rigol.pre_trigger_samples = (size_t)parse_u32_decimal(argv[argi + 6], "PRE_TRIGGER_SAMPLES");
            rigol.window_samples = (size_t)parse_u32_decimal(argv[argi + 7], "WINDOW_SAMPLES");
            rigol.trigger_threshold_v = strtod(argv[argi + 8], NULL);
        }

        if ((argc - argi) >= 11) {
            rigol.timebase_scale_s = strtod(argv[argi + 9], NULL);
            rigol.timebase_offset_s = strtod(argv[argi + 10], NULL);
        }

        if ((argc - argi) == 12) {
            rigol.memory_depth_points = (size_t)parse_u32_decimal(argv[argi + 11], "MEMORY_DEPTH");
        }

        rigol_net_init();
        rc = rl_mode_loop(argv[argi + 1],
                          parse_u32_decimal(argv[argi + 2], "SEED_START"),
                          parse_u32_decimal(argv[argi + 3], "SEED_END"),
                          &rigol,
                          &options);
        rigol_net_cleanup();
    } else {
        print_usage_and_exit();
    }

    rl_log_close();
    return rc;
}
