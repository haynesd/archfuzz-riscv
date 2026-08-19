# RISC-V Differential Fuzzing Lab  

This repository implements a **multi-architecture differential fuzzing laboratory** for discovering vulnerabilities in RISC-V systems.

The lab combines:
- FPGA orchestration
- Multi-board execution
- Power side-channel measurement
- Reinforcement learning (RL)

to automatically identify **architectural, microarchitectural, and side-channel divergences**.

---

# System Architecture

```text
HOST PC (rl_host)
   │
   │ UART (FT232)
   │
   ▼
FPGA Controller (Nexys A7)
   │
   ├── UART0 → Board A
   ├── UART1 → Board B
   └── UART2 → Board C
   │
   └── Trigger → Oscilloscope CH4
```

Boards → Power Shunts → Oscilloscope CH1–CH3

Execution is **sequential per board** to ensure clean waveform capture.

---

# Waveform Alignment

```text
Time →
        ┌──────────── Execution Window ────────────┐
Board A     ____/''''''''''''''''''''''''''''\____
Board B     ___/'''''''''''''''''''''''''''''\___
Board C     ____/''''''''''''''''''''''''''''\____
                 ↑ Trigger (FPGA)
```

Process:
1. FPGA emits trigger pulse (CH4)
2. Scope captures full waveform
3. Host extracts window relative to trigger
4. Signals are normalized and compared

---

# RL Loop

```text
Select Action (UCB)
        ↓
Send RUN Command
        ↓
FPGA + Boards Execute
        ↓
Collect Results + Waveforms
        ↓
Compute Reward
        ↓
Update Policy
```

---

# Formal Reward Function

R = α D_semantic + β D_timing + γ D_power + δ D_fault

Semantic:
D_semantic = Σ |S_i - S_j|

Timing:
D_timing = Σ |C_i - C_j|

Power:
D_power = Σ ( |E_i - E_j| / max(E_i, E_j) + λ * Σ |V_i - V_j| / N )

Fault:
D_fault = K if mismatch else 0

---

# Repository Structure

```text
archfuzz-riscv/
├── README.md
├── LICENSE
├── fpga/
│   ├── fpga_fuzz_ctrl.v
│   ├── fpga_fuzz.ctrl.xdc
│   ├── uart_rx.v
│   └── uart_tx.v
├── board/
│   └── runner.c
└── host/
    ├── BUILD_VISUAL_STUDIO_2022.md
    ├── CMakeLists.txt
    ├── common.h
    ├── logging.c
    ├── main.c
    ├── rigol.c
    ├── rigol.h
    ├── rigol_client.c
    ├── rl.c
    ├── rl.h
    ├── serial.c
    ├── serial.h
    ├── waveform.c
    └── waveform.h
```

---

# Building

## Visual Studio 2022

    cmake -S . -B build -G "Visual Studio 17 2022"
    cmake --build build --config Debug
    cmake --build build --config Release

## GCC (MinGW-w64)

No CMake/make/ninja needed for this path — one direct compiler invocation
compiles all six sources and links `ws2_32` (Winsock, for the Rigol TCP/SCPI
client). Run from inside `host/`.

Git Bash / MSYS2 shell (backslash line continuation):

    gcc -O2 -Wall -Wextra -std=c11 \
      -o rl_host.exe \
      main.c serial.c rigol.c waveform.c rl.c logging.c \
      -lws2_32 -lm

PowerShell / cmd.exe (single line — backslash continuation doesn't work there):

    gcc -O2 -Wall -Wextra -std=c11 -o rl_host.exe main.c serial.c rigol.c waveform.c rl.c logging.c -lws2_32 -lm

If `gcc` isn't found, make sure a MinGW-w64 `gcc.exe` (e.g. from MSYS2's
`mingw64` environment) is on `PATH` — running `gcc --version` should report
an `x86_64-w64-mingw32` target, not a Linux one.

---

# Running

    rl_host.exe rl_scope COM5 16 65536 192.168.1.178 5555

## Reproducible / unattended campaigns

By default the seed-selection PRNG is seeded from the current time, so
consecutive runs explore different seeds. For a reproducible campaign, or one
that runs unattended for hours/days, use:

    rl_host.exe --rng-seed 42 --results run1.csv --checkpoint run1.ckpt rl COM5 16 65536

- `--rng-seed <N>`  makes the explored seed sequence reproducible across runs.
- `--results <FILE>` appends one CSV row per iteration (seed, steps, per-board
  scores/flags/timing, wave divergence, reward) for offline analysis.
- `--checkpoint <FILE>` persists UCB bandit statistics periodically so the
  campaign can resume after being interrupted, instead of restarting the
  bandit from scratch.

These flags apply to both `rl` and `rl_scope` and must precede the mode name.
Run `rl_host.exe` with no arguments for the full flag/usage list.

---

# License

Research / educational use only.
