// ============================================================================
// File        : fpga_fuzz_ctrl.v
// Target      : Xilinx Nexys A7 (Artix-7)
// Language    : Verilog-2001
//
// TITLE
// ----------------------------------------------------------------------------
// Architectural FPGA Fuzz Controller
//
// DESCRIPTION
// ----------------------------------------------------------------------------
// Lightweight UART router / trigger controller for a differential fuzzing lab.
//
// FAST-SYNTH VERSION
// ----------------------------------------------------------------------------
// This version is optimized to synthesize quickly by:
//   - removing the line response buffer
//   - removing the reset-time memory clear loop
//   - streaming board response bytes directly to host
//   - minimizing register widths where practical
//   - keeping the FSM flat and simple
//
// UART PROTOCOL (Host -> FPGA)
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
// UART PROTOCOL (FPGA -> Host)
// ----------------------------------------------------------------------------
//   Streams raw board response as bytes arrive:
//
//     "DONE <seed> <score> <flags> <window> <cycles>\n"
//
//   OR timeout indicator:
//
//     "T\n"
//
// TRIGGER BEHAVIOR
// ----------------------------------------------------------------------------
//   trig_out = HIGH after a valid board is selected
//   trig_out = LOW after newline from board or timeout
// ============================================================================

`timescale 1ns / 1ps

module fpga_fuzz_ctrl #(
    parameter integer CLK_HZ         = 100_000_000,   // FPGA system clock frequency
    parameter integer BAUD           = 115200,        // UART baud rate for all ports
    parameter integer TIMEOUT_CYCLES = 300_000_000    // Timeout while waiting for board response
)(
    input  wire clk,                                  // System clock
    input  wire rst_n,                                // Active-low synchronous reset

    input  wire uart_host_rx,                         // UART RX from host PC
    output wire uart_host_tx,                         // UART TX back to host PC

    input  wire uart0_rx,                             // UART RX from Board A
    output wire uart0_tx,                             // UART TX to Board A

    input  wire uart1_rx,                             // UART RX from Board B
    output wire uart1_tx,                             // UART TX to Board B

    input  wire uart2_rx,                             // UART RX from Board C
    output wire uart2_tx,                             // UART TX to Board C

    output reg  trig_out                              // Oscilloscope trigger output
);

    // =========================================================================
    // Width calculation for timeout counter
    // =========================================================================
    // Use the minimum number of bits needed for timeout counting.
    localparam integer TIMEOUT_W =
        (TIMEOUT_CYCLES <= 2) ? 1 : $clog2(TIMEOUT_CYCLES);

    // =========================================================================
    // Host UART wires and regs
    // =========================================================================
    wire [7:0] h_rx_data;                             // Byte received from host
    wire       h_rx_valid;                            // One-clock pulse when host byte is valid

    reg  [7:0] h_tx_data;                             // Byte to send back to host
    reg        h_tx_start;                            // One-clock pulse to launch host TX byte
    wire       h_tx_busy;                             // Host TX busy indicator

    // Instantiate host receiver.
    uart_rx #(
        .CLK_HZ(CLK_HZ),                              // Pass system clock frequency
        .BAUD(BAUD)                                   // Pass UART baud rate
    ) RXH (
        .clk(clk),                                    // System clock
        .rst_n(rst_n),                                // Reset
        .rx(uart_host_rx),                            // Serial input from host
        .data(h_rx_data),                             // Decoded host byte
        .valid(h_rx_valid)                            // Byte-valid pulse
    );

    // Instantiate host transmitter.
    uart_tx #(
        .CLK_HZ(CLK_HZ),                              // Pass system clock frequency
        .BAUD(BAUD)                                   // Pass UART baud rate
    ) TXH (
        .clk(clk),                                    // System clock
        .rst_n(rst_n),                                // Reset
        .tx(uart_host_tx),                            // Serial output to host
        .data(h_tx_data),                             // Byte to transmit
        .start(h_tx_start),                           // Start pulse
        .busy(h_tx_busy)                              // Busy flag
    );

    // =========================================================================
    // Board UART wires and regs
    // =========================================================================
    wire [7:0] b0_rx_data, b1_rx_data, b2_rx_data;    // Bytes received from boards
    wire       b0_rx_valid, b1_rx_valid, b2_rx_valid; // Valid pulses from boards

    reg  [7:0] b0_tx_data, b1_tx_data, b2_tx_data;    // Bytes to send to boards
    reg        b0_tx_start, b1_tx_start, b2_tx_start; // Start pulses to board TXs
    wire       b0_tx_busy,  b1_tx_busy,  b2_tx_busy;  // Busy flags for board TXs

    // Board A receiver.
    uart_rx #(
        .CLK_HZ(CLK_HZ),                              // Pass system clock frequency
        .BAUD(BAUD)                                   // Pass UART baud rate
    ) RX0 (
        .clk(clk),                                    // System clock
        .rst_n(rst_n),                                // Reset
        .rx(uart0_rx),                                // Serial input from Board A
        .data(b0_rx_data),                            // Byte from Board A
        .valid(b0_rx_valid)                           // Valid pulse from Board A
    );

    // Board A transmitter.
    uart_tx #(
        .CLK_HZ(CLK_HZ),                              // Pass system clock frequency
        .BAUD(BAUD)                                   // Pass UART baud rate
    ) TX0 (
        .clk(clk),                                    // System clock
        .rst_n(rst_n),                                // Reset
        .tx(uart0_tx),                                // Serial output to Board A
        .data(b0_tx_data),                            // Byte to Board A
        .start(b0_tx_start),                          // Start pulse to Board A TX
        .busy(b0_tx_busy)                             // Busy flag from Board A TX
    );

    // Board B receiver.
    uart_rx #(
        .CLK_HZ(CLK_HZ),                              // Pass system clock frequency
        .BAUD(BAUD)                                   // Pass UART baud rate
    ) RX1 (
        .clk(clk),                                    // System clock
        .rst_n(rst_n),                                // Reset
        .rx(uart1_rx),                                // Serial input from Board B
        .data(b1_rx_data),                            // Byte from Board B
        .valid(b1_rx_valid)                           // Valid pulse from Board B
    );

    // Board B transmitter.
    uart_tx #(
        .CLK_HZ(CLK_HZ),                              // Pass system clock frequency
        .BAUD(BAUD)                                   // Pass UART baud rate
    ) TX1 (
        .clk(clk),                                    // System clock
        .rst_n(rst_n),                                // Reset
        .tx(uart1_tx),                                // Serial output to Board B
        .data(b1_tx_data),                            // Byte to Board B
        .start(b1_tx_start),                          // Start pulse to Board B TX
        .busy(b1_tx_busy)                             // Busy flag from Board B TX
    );

    // Board C receiver.
    uart_rx #(
        .CLK_HZ(CLK_HZ),                              // Pass system clock frequency
        .BAUD(BAUD)                                   // Pass UART baud rate
    ) RX2 (
        .clk(clk),                                    // System clock
        .rst_n(rst_n),                                // Reset
        .rx(uart2_rx),                                // Serial input from Board C
        .data(b2_rx_data),                            // Byte from Board C
        .valid(b2_rx_valid)                           // Valid pulse from Board C
    );

    // Board C transmitter.
    uart_tx #(
        .CLK_HZ(CLK_HZ),                              // Pass system clock frequency
        .BAUD(BAUD)                                   // Pass UART baud rate
    ) TX2 (
        .clk(clk),                                    // System clock
        .rst_n(rst_n),                                // Reset
        .tx(uart2_tx),                                // Serial output to Board C
        .data(b2_tx_data),                            // Byte to Board C
        .start(b2_tx_start),                          // Start pulse to Board C TX
        .busy(b2_tx_busy)                             // Busy flag from Board C TX
    );

    // =========================================================================
    // Selected board muxing
    // =========================================================================
    reg [1:0] active_board;                           // Currently selected board ID

    // Mux selected board RX data based on active_board.
    wire [7:0] sel_rx_data =
        (active_board == 2'd0) ? b0_rx_data :         // Use Board A RX when selected
        (active_board == 2'd1) ? b1_rx_data :         // Use Board B RX when selected
                                 b2_rx_data;          // Otherwise use Board C RX

    // Mux selected board RX valid pulse based on active_board.
    wire sel_rx_valid =
        (active_board == 2'd0) ? b0_rx_valid :        // Board A valid pulse
        (active_board == 2'd1) ? b1_rx_valid :        // Board B valid pulse
                                 b2_rx_valid;         // Board C valid pulse

    // Mux selected board TX busy flag based on active_board.
    wire sel_tx_busy =
        (active_board == 2'd0) ? b0_tx_busy :         // Board A busy flag
        (active_board == 2'd1) ? b1_tx_busy :         // Board B busy flag
                                 b2_tx_busy;          // Board C busy flag

    // =========================================================================
    // FSM states
    // =========================================================================
    localparam [2:0]
        ST_IDLE       = 3'd0,                         // Waiting for host command prefix 0x01
        ST_GET_BRD    = 3'd1,                         // Waiting for board ID byte
        ST_SEND       = 3'd2,                         // Forwarding payload bytes to chosen board
        ST_WAIT_RESP  = 3'd3,                         // Waiting for board response bytes
        ST_TIMEOUT_T  = 3'd4,                         // Sending 'T' to host on timeout
        ST_TIMEOUT_NL = 3'd5;                         // Sending newline after timeout

    reg [2:0] state;                                  // Current FSM state

    // =========================================================================
    // Timeout counter
    // =========================================================================
    reg [TIMEOUT_W-1:0] timeout_cnt;                  // Counts idle cycles while waiting for board

    // =========================================================================
    // Main control FSM
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            state        <= ST_IDLE;                  // Reset FSM to idle
            active_board <= 2'd0;                     // Default selected board
            trig_out     <= 1'b0;                     // Trigger low on reset

            h_tx_data    <= 8'h00;                    // Clear host TX data register
            h_tx_start   <= 1'b0;                     // No host TX launch on reset

            b0_tx_data   <= 8'h00;                    // Clear Board A TX byte register
            b1_tx_data   <= 8'h00;                    // Clear Board B TX byte register
            b2_tx_data   <= 8'h00;                    // Clear Board C TX byte register

            b0_tx_start  <= 1'b0;                     // No Board A TX launch on reset
            b1_tx_start  <= 1'b0;                     // No Board B TX launch on reset
            b2_tx_start  <= 1'b0;                     // No Board C TX launch on reset

            timeout_cnt  <= {TIMEOUT_W{1'b0}};       // Clear timeout counter
        end else begin
            // Default all TX start strobes low every clock.
            h_tx_start  <= 1'b0;                      // Host TX start is a one-clock pulse
            b0_tx_start <= 1'b0;                      // Board A TX start is a one-clock pulse
            b1_tx_start <= 1'b0;                      // Board B TX start is a one-clock pulse
            b2_tx_start <= 1'b0;                      // Board C TX start is a one-clock pulse

            case (state)

                ST_IDLE: begin
                    trig_out    <= 1'b0;              // Trigger stays low when idle
                    timeout_cnt <= {TIMEOUT_W{1'b0}};// Clear timeout counter in idle

                    // Wait for command prefix 0x01 from host.
                    if (h_rx_valid && h_rx_data == 8'h01) begin
                        state <= ST_GET_BRD;          // Next byte must be board ID
                    end
                end

                ST_GET_BRD: begin
                    // Wait for board ID byte from host.
                    if (h_rx_valid) begin
                        if (h_rx_data < 8'd3) begin   // Accept only board IDs 0,1,2
                            active_board <= h_rx_data[1:0]; // Store selected board
                            trig_out     <= 1'b1;     // Raise trigger once board is selected
                            state        <= ST_SEND;  // Begin forwarding payload bytes
                        end else begin
                            state <= ST_IDLE;         // Invalid board ID, abandon command
                        end
                    end
                end

                ST_SEND: begin
                    // Forward host payload bytes to selected board.
                    if (h_rx_valid && !sel_tx_busy) begin
                        case (active_board)
                            2'd0: begin
                                b0_tx_data  <= h_rx_data; // Present byte to Board A TX
                                b0_tx_start <= 1'b1;      // Pulse Board A TX start
                            end

                            2'd1: begin
                                b1_tx_data  <= h_rx_data; // Present byte to Board B TX
                                b1_tx_start <= 1'b1;      // Pulse Board B TX start
                            end

                            default: begin
                                b2_tx_data  <= h_rx_data; // Present byte to Board C TX
                                b2_tx_start <= 1'b1;      // Pulse Board C TX start
                            end
                        endcase

                        // Newline marks the end of the outbound command.
                        if (h_rx_data == 8'h0A) begin
                            timeout_cnt <= {TIMEOUT_W{1'b0}}; // Reset response timeout
                            state       <= ST_WAIT_RESP;      // Wait for board reply
                        end
                    end
                end

                ST_WAIT_RESP: begin
                    // Increment timeout counter while waiting for a board response.
                    timeout_cnt <= timeout_cnt + 1'b1;

                    // If the board produced a byte and host TX is available, forward it immediately.
                    if (sel_rx_valid && !h_tx_busy) begin
                        h_tx_data  <= sel_rx_data;    // Put board byte onto host TX data bus
                        h_tx_start <= 1'b1;           // Launch host TX of that byte
                        timeout_cnt <= {TIMEOUT_W{1'b0}}; // Reset timeout after activity

                        // If newline arrives, response is complete.
                        if (sel_rx_data == 8'h0A) begin
                            trig_out <= 1'b0;         // Drop trigger at end of line
                            state    <= ST_IDLE;      // Return to idle for next command
                        end
                    end else if (timeout_cnt == TIMEOUT_CYCLES-1) begin
                        // If board stays silent too long, emit timeout indicator.
                        state <= ST_TIMEOUT_T;        // Send 'T'
                    end
                end

                ST_TIMEOUT_T: begin
                    // Send ASCII 'T' to host to indicate timeout.
                    if (!h_tx_busy) begin
                        h_tx_data  <= "T";            // Timeout marker
                        h_tx_start <= 1'b1;           // Launch host TX
                        state      <= ST_TIMEOUT_NL;  // Then send newline
                    end
                end

                ST_TIMEOUT_NL: begin
                    // Send newline after timeout marker.
                    if (!h_tx_busy) begin
                        h_tx_data  <= 8'h0A;          // ASCII newline
                        h_tx_start <= 1'b1;           // Launch host TX
                        trig_out   <= 1'b0;           // Drop trigger on timeout completion
                        state      <= ST_IDLE;        // Return to idle
                    end
                end

                default: begin
                    state <= ST_IDLE;                 // Safe recovery state
                end
            endcase
        end
    end

endmodule