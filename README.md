
# RISC-V Differential Fuzzing Lab
### FPGA-Orchestrated Vulnerability Discovery

This repository contains the hardware and software used to build a **multi-architecture differential fuzzing laboratory** for discovering vulnerabilities in RISC-V implementations.

The system uses an **FPGA experiment controller** to coordinate execution across multiple RISC-V boards while measuring:

- Architectural behavior
- Microarchitectural timing
- Power side-channel leakage
- Fault behavior

A host computer analyzes results and uses **reinforcement learning (RL)** to guide fuzz exploration.

---

# System Architecture
```
HOST PC
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
Boards → Power Shunts → Oscilloscope CH1-3

The FPGA runs experiments sequentially across boards to allow clean oscilloscope captures.

---

# What the Lab Detects

## Semantic Divergence

Same program produces different outputs.

checksum_A ≠ checksum_B

Indicates potential **ISA compliance issues or undefined behavior.**

---

## Timing Divergence

Same output but different cycle counts.

checksum_A == checksum_B  
cycles_A ≠ cycles_B

Indicates **microarchitectural differences** or **timing side channels.**

---

## Power Divergence

Same output and timing but different power traces.

checksum_A == checksum_B  
cycles_A ≈ cycles_B  
power_A ≠ power_B

Indicates **side-channel leakage**.

---

## Fault Divergence

One board traps or hangs.

board_A faults  
board_B completes  
board_C completes

Indicates **privilege or exception boundary vulnerabilities.**

---

# Hardware

## FPGA Controller

Xilinx Nexys A7  
Artix-7 FPGA

Responsibilities:

- dispatch RUN commands
- generate oscilloscope trigger pulses
- collect board results
- detect timeouts
- aggregate results

---

## RISC-V Boards

Example supported boards:

VisionFive2  
Orange Pi RV2  
Lichee RV D1

Each board runs the **runner program** that:

- executes fuzzed instruction sequences
- reads cycle counts (`rdcycle`)
- computes output checksums
- detects anomalies
- returns results to FPGA

---

## Oscilloscope

Rigol DS1054Z

Channel assignments:

CH1 → Board A power  
CH2 → Board B power  
CH3 → Board C power  
CH4 → FPGA trigger

The host retrieves waveforms via **SCPI over LAN**.

---

## Power Measurement

Each board power rail includes a shunt resistor.

Example:

50A 50mV shunt resistor

Current is derived as:

I = V / R

Power waveforms are used to detect side-channel differences.

---

# Repository Structure
```
archdiff-riscv
│
├── README.md
│
├── fpga
│   ├── fpga_fuzz_ctrl.v
│   ├── uart_rx.v
│   ├── uart_tx.v
│   ├── nexys_a7.xdc
|
├── runner
│   ├── runner_window.c
│
├── host
│   ├── rl_host.c
```
---

# Communication Protocol

Host → FPGA

PING

CFG SEED_START=00100000 SEED_END=0010FFFF STEPS_MIN=64 STEPS_MAX=1024 POLICY=1

RUN SEED=00112233 STEPS=256

REPLAY SEED=00112233 STEPS=256 START=160 COUNT=32

---

FPGA → Host

PONG  
ACK CFG  
ACK RUN  
ACK REPLAY

Example result:

TRIPLE SEED=00112233 STEPS=256  
S0=123456789 F0=00000000 W0=5 WC0=1842 T0=0 D0=1043211  
S1=123450001 F1=00000000 W1=5 WC1=1770 T1=0 D1=1035520  
S2=0 F2=00000008 W2=0 WC2=0 T2=1 D2=300000000  
RS=160 RC=32

---

# Building

## Runner

gcc -O2 -Wall -o runner runner_window.c

Run:

./runner /dev/ttyS1

---

## FPGA

Using Vivado:

synth_design  
impl_design  
write_bitstream

Program FPGA:

program_device

---

# Running an Experiment

1. Start runner on each board.

./runner /dev/ttyS1

2. Start host controller.

./rl_host

3. Launch experiment.

RUN SEED=00112233 STEPS=256

The FPGA will:

1. trigger scope
2. run board A
3. run board B
4. run board C
5. return aggregated results

---

# Reinforcement Learning

Example reward signals:

semantic divergence → high reward  
timing divergence → medium reward  
power divergence → medium reward  
fault divergence → high reward

---

# Replay Minimization

REPLAY SEED=00112233 STEPS=256 START=160 COUNT=32

Repeated narrowing isolates the minimal instruction sequence.

---

# Research Applications

- CPU fuzzing
- ISA validation
- microarchitectural vulnerability discovery
- side-channel research
- reinforcement learning guided fuzzing

---

# License

Research / educational use.
