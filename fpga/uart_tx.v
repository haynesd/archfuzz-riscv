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
// PARAMETERS
// ----------------------------------------------------------------------------
//   CLK_HZ : input clock frequency
//   BAUD   : UART baud rate
//
// ============================================================================

`timescale 1ns/1ps

module uart_rx #(
    parameter integer CLK_HZ = 100_000_000,
    parameter integer BAUD   = 115200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,
    output reg  [7:0] data,
    output reg        valid
);

    localparam integer CLKS_PER_BIT = CLK_HZ / BAUD;
    localparam integer HALF_BIT     = CLKS_PER_BIT / 2;

    localparam [1:0]
        ST_IDLE  = 2'd0,
        ST_START = 2'd1,
        ST_DATA  = 2'd2,
        ST_STOP  = 2'd3;

    reg [1:0]  state;
    reg [31:0] clk_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  shreg;

    always @(posedge clk) begin
        if (!rst_n) begin
            state   <= ST_IDLE;
            clk_cnt <= 32'd0;
            bit_idx <= 3'd0;
            shreg   <= 8'd0;
            data    <= 8'd0;
            valid   <= 1'b0;
        end else begin
            valid <= 1'b0;

            case (state)

                ST_IDLE: begin
                    clk_cnt <= 32'd0;
                    bit_idx <= 3'd0;

                    if (rx == 1'b0) begin
                        // Possible start bit detected
                        state <= ST_START;
                    end
                end

                ST_START: begin
                    if (clk_cnt == HALF_BIT) begin
                        if (rx == 1'b0) begin
                            // Confirmed start bit, align to data sampling
                            clk_cnt <= 32'd0;
                            state   <= ST_DATA;
                        end else begin
                            // False start
                            state <= ST_IDLE;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                ST_DATA: begin
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt        <= 32'd0;
                        shreg[bit_idx] <= rx;

                        if (bit_idx == 3'd7) begin
                            bit_idx <= 3'd0;
                            state   <= ST_STOP;
                        end else begin
                            bit_idx <= bit_idx + 1'b1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                ST_STOP: begin
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 32'd0;
                        data    <= shreg;
                        valid   <= 1'b1;
                        state   <= ST_IDLE;
                    end else begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule