// ============================================================================
// File        : uart_rx.v
// Language    : Verilog-2001
//
// TITLE
// ----------------------------------------------------------------------------
// Simple 8N1 UART Receiver
//
// DESCRIPTION
// ----------------------------------------------------------------------------
// Receives one UART byte in 8N1 format:
//   1 start bit (0)
//   8 data bits, LSB first
//   1 stop bit (1)
//
// When a full byte has been received, 'valid' pulses high for one clock and
// 'data' presents the received byte.
//
// NOTES
// ----------------------------------------------------------------------------
// This version is written to synthesize quickly:
//   - counter widths are minimized
//   - reset logic is minimized
//   - logic is kept simple and flat
// ============================================================================

`timescale 1ns/1ps

module uart_rx #(
    parameter integer CLK_HZ = 100_000_000,           // Input FPGA clock frequency
    parameter integer BAUD   = 115200                 // UART baud rate
)(
    input  wire       clk,                            // System clock
    input  wire       rst_n,                          // Active-low synchronous reset
    input  wire       rx,                             // Asynchronous UART RX input
    output reg  [7:0] data,                           // Received byte output
    output reg        valid                           // One-clock pulse when byte is ready
);

    // Number of FPGA clocks per UART bit period.
    localparam integer CLKS_PER_BIT = CLK_HZ / BAUD;

    // Half-bit delay used to sample in the middle of the start bit.
    localparam integer HALF_BIT = CLKS_PER_BIT / 2;

    // Small state machine encoding.
    localparam [1:0]
        ST_IDLE  = 2'd0,                              // Waiting for start bit
        ST_START = 2'd1,                              // Verifying start bit at mid-bit
        ST_DATA  = 2'd2,                              // Sampling 8 data bits
        ST_STOP  = 2'd3;                              // Sampling stop bit

    // Minimum counter width needed for the baud counter.
    localparam integer CNT_W = (CLKS_PER_BIT <= 2) ? 1 : $clog2(CLKS_PER_BIT);

    // Two-flop synchronizer for the asynchronous RX pin.
    reg rx_meta;                                      // First sync flop
    reg rx_sync;                                      // Second sync flop

    // FSM state register.
    reg [1:0] state;                                  // Current receiver state

    // Baud clock counter.
    reg [CNT_W-1:0] clk_cnt;                          // Counts clocks within one UART bit

    // Tracks which data bit is being received (0 through 7).
    reg [2:0] bit_idx;                                // Current data bit index

    // Temporary storage shift register for incoming byte.
    reg [7:0] shreg;                                  // Captured data bits

    // Synchronize the asynchronous RX input into the FPGA clock domain.
    always @(posedge clk) begin
        if (!rst_n) begin
            rx_meta <= 1'b1;                          // UART idle level is high
            rx_sync <= 1'b1;                          // UART idle level is high
        end else begin
            rx_meta <= rx;                            // First sync stage
            rx_sync <= rx_meta;                       // Second sync stage
        end
    end

    // Main UART receiver state machine.
    always @(posedge clk) begin
        if (!rst_n) begin
            state   <= ST_IDLE;                       // Go to idle on reset
            clk_cnt <= {CNT_W{1'b0}};                // Clear baud counter
            bit_idx <= 3'd0;                          // Clear bit index
            shreg   <= 8'd0;                          // Clear shift register
            data    <= 8'd0;                          // Clear output byte
            valid   <= 1'b0;                          // No valid byte on reset
        end else begin
            valid <= 1'b0;                            // Default: valid pulses only for one clock

            case (state)

                ST_IDLE: begin
                    clk_cnt <= {CNT_W{1'b0}};        // Reset baud counter in idle
                    bit_idx <= 3'd0;                  // Reset bit index in idle

                    if (rx_sync == 1'b0) begin        // Detect falling edge / start bit
                        state <= ST_START;            // Move to start verification
                    end
                end

                ST_START: begin
                    if (clk_cnt == HALF_BIT[CNT_W-1:0]) begin
                        if (rx_sync == 1'b0) begin    // Start bit still low at mid-bit
                            clk_cnt <= {CNT_W{1'b0}};// Restart counter for data bits
                            state   <= ST_DATA;       // Valid start bit, receive data
                        end else begin
                            state <= ST_IDLE;         // False start, return to idle
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;    // Keep waiting to middle of start bit
                    end
                end

                ST_DATA: begin
                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt        <= {CNT_W{1'b0}}; // Restart per-bit timing counter
                        shreg[bit_idx] <= rx_sync;       // Sample current data bit

                        if (bit_idx == 3'd7) begin
                            bit_idx <= 3'd0;          // Reset bit index after last bit
                            state   <= ST_STOP;       // Move to stop-bit check
                        end else begin
                            bit_idx <= bit_idx + 1'b1;// Advance to next bit
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;    // Wait until next bit sample point
                    end
                end

                ST_STOP: begin
                    if (clk_cnt == CLKS_PER_BIT-1) begin
                        clk_cnt <= {CNT_W{1'b0}};    // Clear counter for next frame

                        if (rx_sync == 1'b1) begin    // Stop bit must be high
                            data  <= shreg;           // Commit received byte
                            valid <= 1'b1;            // Pulse valid for one clock
                        end

                        state <= ST_IDLE;             // Done with frame
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;    // Wait for stop-bit sample point
                    end
                end

                default: begin
                    state <= ST_IDLE;                 // Safe recovery path
                end
            endcase
        end
    end

endmodule