//`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////////
//
// Title:        FPGA Differential Fuzzing Controller - Top Level
// Target:       Xilinx Nexys A7 (Artix-7)
// Clock:        100 MHz
//
// Description:
// -------------------------------------------------------------------------------
// This is the top-level wrapper for the `fpga_fuzz_ctrl` module used in the
// multi-architecture RISC-V differential fuzzing laboratory.
//
// System Architecture:
//
//        HOST PC
//           ?
//           ?  UART (FTDI USB-Serial)
//           ?
//           ?
//     FPGA Controller (Nexys A7)
//        ?        ?        ?
//        ?        ?        ?
//        ?        ?        ??? UART2 ? Target Board C
//        ?        ??????????? UART1 ? Target Board B
//        ???????????????????? UART0 ? Target Board A
//
//        Trigger Output ???????????? Oscilloscope
//
// In this simplified configuration:
//
//   • UART3 inside `fpga_fuzz_ctrl` is connected to the HOST PC.
//   • UART0/1/2 (board interfaces) are currently unused and tied to idle.
//   • `trig_out` is connected to the oscilloscope to synchronize
//     side-channel measurements during fuzz runs.
//
// The FPGA controller performs:
//
//   1. Command reception from the host over UART.
//   2. Dispatching fuzz instructions to target boards.
//   3. Capturing board results.
//   4. Generating a deterministic trigger signal for oscilloscopes.
//   5. Returning aggregated results back to the host.
//
// LEDs provide simple runtime visibility.
//
// LED Mapping:
//   LED[0] = Reset button state
//   LED[1] = Trigger signal (scope trigger)
//   LED[2] = Host UART RX activity (PC ? FPGA)
//   LED[3] = Host UART TX activity (FPGA ? PC)
//
// Reset Behavior:
//
//   The controller uses an **active-low reset (`rst_n`)**.
//   The Nexys A7 pushbutton reset (`rst_btn`) is active-high,
//   therefore it is inverted before connecting to the controller.
//
// Notes:
//
//   • Unused board UART RX pins are tied to logic high (idle state).
//   • Board TX signals are left unconnected but captured in internal wires
//     to prevent synthesis warnings.
//   • This top module is intentionally minimal to simplify lab wiring.
//
// Author: David Haynes Lab
//
////////////////////////////////////////////////////////////////////////////////////

module top (
  input  wire clk100,     // 100 MHz system clock from Nexys A7
  input  wire rst_btn,    // Active-high pushbutton reset

  // Host UART connection (USB-Serial from PC)
  input  wire uart_rx,    // PC ? FPGA
  output wire uart_tx,    // FPGA ? PC

  // Oscilloscope trigger output
  output wire trig_out,

  // On-board LEDs for quick visual debugging
  output wire [3:0] led
);

  // ---------------------------------------------------------------------------
  // Unused board UART outputs
  // ---------------------------------------------------------------------------
  // These wires capture TX signals from the controller's board interfaces.
  // They are currently unused in this configuration but declared to avoid
  // synthesis warnings about unconnected outputs.

  wire uart0_tx_unused;
  wire uart1_tx_unused;
  wire uart2_tx_unused;

  // ---------------------------------------------------------------------------
  // FPGA Fuzzing Controller
  // ---------------------------------------------------------------------------
  // The core module implementing the UART command processor,
  // fuzz execution controller, and result aggregation engine.

  fpga_fuzz_ctrl #(
    .CLK_HZ(100_000_000),
    .UART_BAUD(115200)
  ) dut (
    .clk(clk100),

    // Controller uses active-low reset
    .rst_n(~rst_btn),

    // Host interface (mapped to UART3 inside controller)
    .uart3_rx(uart_rx),
    .uart3_tx(uart_tx),

    // Board interfaces (currently unused)
    .uart0_rx(1'b1),                 // Idle high
    .uart0_tx(uart0_tx_unused),

    .uart1_rx(1'b1),
    .uart1_tx(uart1_tx_unused),

    .uart2_rx(1'b1),
    .uart2_tx(uart2_tx_unused),

    // Oscilloscope trigger output
    .trig_out(trig_out)
  );

  // ---------------------------------------------------------------------------
  // LED Debug Indicators
  // ---------------------------------------------------------------------------
  // These provide simple runtime status indicators without needing
  // a debugger or ILA.

  assign led[0] = rst_btn;   // Reset button state
  assign led[1] = trig_out;  // Trigger output
  assign led[2] = uart_rx;   // Host RX line
  assign led[3] = uart_tx;   // Host TX line

endmodule