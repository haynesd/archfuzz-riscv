// ============================================================================
// File        : fpga_fuzz_ctrl.v
// Target      : Xilinx Nexys A7 (Artix-7)
// Language    : Verilog-2001
//
// TITLE
// ----------------------------------------------------------------------------
// Architectual FPGA Fuzz Controller 
//
// DESCRIPTION
// ----------------------------------------------------------------------------
// This module implements a lightweight FPGA controller used in a
// multi-board differential architectual fuzzing lab.
//
// The FPGA acts as a deterministic transport + timing device:
//
//   - Routes host commands to one of three target boards
//   - Generates oscilloscope trigger during execution
//   - Captures a newline-terminated DONE response
//   - Enforces a timeout if board does not respond
//   - Streams response back to host PC
//
// The FPGA intentionally DOES NOT:
//   - Parse ASCII commands
//   - Aggregate results
//   - Perform scheduling or RL logic
//
// All intelligence resides on the Host PC.
//
// UART PROTOCOL (Host → FPGA)
// ----------------------------------------------------------------------------
//   [0x01][BOARD_ID][ASCII PAYLOAD...]
//
//   BOARD_ID:
//     0 = UART0 (Board A)
//     1 = UART1 (Board B)
//     2 = UART2 (Board C)
//
//   PAYLOAD example:
//     "RUN 00112233 256\n"
//
// UART PROTOCOL (FPGA → Host)
// ----------------------------------------------------------------------------
//   Streams raw board response:
//
//     "DONE <seed> <score> <flags> <window> <cycles>\n"
//
//   OR timeout indicator:
//
//     "T\n"
//
// TRIGGER BEHAVIOR
// ----------------------------------------------------------------------------
//   trig_out = HIGH during board execution
//   trig_out = LOW after DONE or timeout
//
// HARDWARE CONNECTIONS (NEXYS A7 PMOD / TTL-232)
// ----------------------------------------------------------------------------
// Use PMOD JA for Host UART (FT232 USB-TTL)
//
//   FT232 TXD  -> FPGA RX (JA2 / pin D18)
//   FT232 RXD  <- FPGA TX (JA1 / pin C17)
//   FT232 GND  <-> FPGA GND
//
//   IMPORTANT:
//     - DO NOT connect FT232 VCC to FPGA I/O rail
//     - Ensure 3.3V logic levels
//
// Boards connected to:
//   UART0 -> JB
//   UART1 -> JC
//   UART2 -> JD
//
// ============================================================================

`timescale 1ns/1ps

module fpga_fuzz_ctrl #(
    parameter CLK_HZ = 100_000_000,
    parameter BAUD   = 115200,
    parameter TIMEOUT_CYCLES = 300_000_000
)(
    input wire clk,
    input wire rst_n,

    input  wire uart_host_rx,
    output wire uart_host_tx,

    input  wire uart0_rx, output wire uart0_tx,
    input  wire uart1_rx, output wire uart1_tx,
    input  wire uart2_rx, output wire uart2_tx,

    output reg trig_out
);

    // ================= UART =================
    wire [7:0] h_rx_data;
    wire h_rx_valid;

    reg [7:0] h_tx_data;
    reg h_tx_start;
    wire h_tx_busy;

    uart_rx #(CLK_HZ, BAUD) RXH (clk, rst_n, uart_host_rx, h_rx_data, h_rx_valid);
    uart_tx #(CLK_HZ, BAUD) TXH (clk, rst_n, uart_host_tx, h_tx_data, h_tx_start, h_tx_busy);

    // ================= STATE =================
    reg [2:0] state;
    reg [1:0] active_board;

    localparam IDLE    = 0;
    localparam GET_BRD = 1;
    localparam SEND    = 2;
    localparam WAIT    = 3;
    localparam STREAM  = 4;
    localparam TIMEOUT = 5;

    // ================= BUFFER =================
    reg [7:0] linebuf [0:127];
    reg [6:0] line_len;

    reg [31:0] timeout_cnt;

    // ================= BOARD RX SELECT =================
    wire [7:0] b_rx_data =
        (active_board == 0) ? uart0_rx :
        (active_board == 1) ? uart1_rx : uart2_rx;

    // ================= TX =================
    reg [7:0] tx_data;
    reg tx_start;

    assign uart0_tx = (active_board == 0) ? tx_data[0] : 1'b1;
    assign uart1_tx = (active_board == 1) ? tx_data[0] : 1'b1;
    assign uart2_tx = (active_board == 2) ? tx_data[0] : 1'b1;

    // ================= FSM =================
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= IDLE;
            trig_out <= 0;
            timeout_cnt <= 0;
            line_len <= 0;
        end else begin
            h_tx_start <= 0;
            tx_start   <= 0;

            case (state)

            IDLE:
                if (h_rx_valid && h_rx_data == 8'h01)
                    state <= GET_BRD;

            GET_BRD:
                if (h_rx_valid) begin
                    active_board <= h_rx_data;
                    trig_out <= 1;
                    state <= SEND;
                end

            SEND:
                if (h_rx_valid) begin
                    tx_data  <= h_rx_data;
                    tx_start <= 1;

                    if (h_rx_data == 8'h0A) begin
                        timeout_cnt <= 0;
                        line_len <= 0;
                        state <= WAIT;
                    end
                end

            WAIT:
                begin
                    timeout_cnt <= timeout_cnt + 1;

                    if (timeout_cnt > TIMEOUT_CYCLES)
                        state <= TIMEOUT;
                end

            STREAM:
                if (!h_tx_busy) begin
                    h_tx_data  <= linebuf[0];
                    h_tx_start <= 1;
                    line_len <= line_len - 1;

                    if (line_len == 0) begin
                        trig_out <= 0;
                        state <= IDLE;
                    end
                end

            TIMEOUT:
                if (!h_tx_busy) begin
                    h_tx_data <= "T";
                    h_tx_start <= 1;
                    trig_out <= 0;
                    state <= IDLE;
                end

            endcase
        end
    end

endmodule