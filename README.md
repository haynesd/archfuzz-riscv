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

Boards → Power Shunts → Oscilloscope CH1–CH3

Execution is **sequential per board** to ensure clean waveform capture.

---

# Waveform Alignment

Time →
        ┌──────────── Execution Window ────────────┐
Board A     ____/''''''''''''''''''''''''''''\____
Board B     ___/'''''''''''''''''''''''''''''\___
Board C     ____/''''''''''''''''''''''''''''\____
                 ↑ Trigger (FPGA)

Process:
1. FPGA emits trigger pulse (CH4)
2. Scope captures full waveform
3. Host extracts window relative to trigger
4. Signals are normalized and compared

---

# RL Loop

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
archdiff-riscv
│
├── README.md
│
├── fpga/
|   ├── fpga_fuzz_ctrl.v
|   ├── fpga_fuzz_ctrl.xdc
|   ├── uart_rx.v
|   ├── uart_tx.v
├── board/
|   ├── runner.c
├── host/
│   ├── BUILD_MING_W64.tx
│   ├── BUILD_VISUAL_STUDIO_2022.md
│   ├── CMakeLists.txt
│   ├── Common.h
│   ├── logging.c
│   ├── main.c
│   ├── rigol_client
│   ├── rigol.c
│   ├── rigol.h
│   ├── rl.c
│   ├── rl.h
│   ├── serial.c
│   ├── serial.h
│   ├── waveform.c
│   ├── waveform.h

# Building

## Visual Studio 2022
cmake -S . -B build -G "Visual Studio 17 2022"
cmake --build build --config Debug
cmake --build build --config Release

## GCC
gcc -O2 -Wall -Wextra -std=c11 -o rl_host.exe main.c serial.c rigol.c waveform.c rl.c logging.c -lws2_32 -lm

---

# Running

rl_host.exe rl_scope COM5 16 65536 192.168.1.178 5555 CHAN1

---

# License

Research / educational use only.
