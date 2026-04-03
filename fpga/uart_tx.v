// ============================================================================
// File        : uart_tx.v
// Language    : Verilog-2001
//
// TITLE
// ----------------------------------------------------------------------------
// Simple 8N1 UART Transmitter
//
// DESCRIPTION
// ----------------------------------------------------------------------------
// Transmits one UART byte in 8N1 format:
//   1 start bit (0)
//   8 data bits, LSB first
//   1 stop bit (1)
//
// When 'start' pulses high for one clock, the transmitter latches 'data',
// begins transmission, and asserts 'busy' until the stop bit completes.
//
// NOTES
// ----------------------------------------------------------------------------
// This version is kept simple for fast synthesis:
//   - minimum-width counters
//   - no unnecessary reset fanout
//   - flat FSM
// ============================================================================

`timescale 1ns/1ps

module uart_tx #(
    parameter integer CLK_HZ = 100_000_000,           // Input FPGA clock frequency
    parameter integer BAUD   = 115200                 // UART baud rate
)(
    input  wire       clk,                            // System clock
    input  wire       rst_n,                          // Active-low synchronous reset
    output reg        tx,                             // UART TX output
    input  wire [7:0] data,                           // Byte to transmit
    input  wire       start,                          // One-clock pulse to start transmit
    output reg        busy                            // High while transmission is active
);

    // Number of FPGA clocks per UART bit.
    localparam integer CLKS_PER_BIT = CLK_HZ / BAUD;

    // Small FSM encoding.
    localparam [1:0]
        ST_IDLE  = 2'd0,                              // Waiting for a start request
        ST_START = 2'd1,                              // Driving start bit
        ST_DATA  = 2'd2,                              // Driving 8 data bits
        ST_STOP  = 2'd3;                              // Driving stop bit

    // Minimum-width baud counter.
    localparam integer CNT_W = (CLKS_PER_BIT <= 2) ? 1 : $clog2(CLKS_PER_BIT);

    // FSM state register.
    reg [1:0] state;                                  // Current transmitter state

    // Baud counter.
    reg [CNT_W-1:0] clk_cnt;                          // Counts clocks within one UART bit

    // Which data bit is currently being transmitted.
    reg [2:0] bit_idx;                                // Data bit index 0..7

    // Latched copy of transmit data.
    reg [7:0] shreg;                                  // Byte currently being sent

    // Main UART transmitter state machine.
    always @(posedge clk) begin
        if (!rst_n) begin
            state   <= ST_IDLE;                       // Reset to idle state
            tx      <= 1'b1;                          // UART idle level is high
            busy    <= 1'b0;                          // Not transmitting on reset
            clk_cnt <= {CNT_W{1'b0}};                // Clear baud counter
            bit_idx <= 3'd0;                          // Clear bit index
            shreg   <= 8'd0;                          // Clear shift register
        end else begin
            case (state)

                ST_IDLE: begin
                    tx   <= 1'b1;                     // Hold line high when idle
                    busy <= 1'b0;                     // Not busy in idle
                    clk_cnt <= {CNT_W{1'b0}};        // Keep counter cleared in idle
                    bit_idx <= 3'd0;                  // Keep bit index reset in idle

                    if (start) begin                  // Start request received
                        shreg   <= data;              // Latch input byte
                        busy    <= 1'b1;              // Mark transmitter busy
                        state   <= ST_START;          // Begin start bit
                    end
                end

                ST_START: begin
                    tx <= 1'b0;                       // Drive start bit low

                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt <= {CNT_W{1'b0}};    // Restart bit timer
                        bit_idx <= 3'd0;              // Start with bit 0
                        state   <= ST_DATA;           // Move to data phase
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;    // Continue start-bit timing
                    end
                end

                ST_DATA: begin
                    tx <= shreg[bit_idx];             // Drive current data bit

                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt <= {CNT_W{1'b0}};    // Restart timer for next bit

                        if (bit_idx == 3'd7) begin
                            state <= ST_STOP;         // Last bit sent, move to stop
                        end else begin
                            bit_idx <= bit_idx + 1'b1;// Advance to next data bit
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;    // Continue data-bit timing
                    end
                end

                ST_STOP: begin
                    tx <= 1'b1;                       // Drive stop bit high

                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt <= {CNT_W{1'b0}};    // Clear timer after stop bit
                        state   <= ST_IDLE;           // Return to idle
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;    // Continue stop-bit timing
                    end
                end

                default: begin
                    state <= ST_IDLE;                 // Safe recovery path
                end
            endcase
        end
    end

endmodule