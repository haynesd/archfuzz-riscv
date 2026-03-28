// ============================================================================
// fpga_fuzz_ctrl.v
// ----------------------------------------------------------------------------
// Title      : Multi-Board Differential Fuzzing UART Orchestrator
// Target     : Xilinx Nexys A7 (Artix-7) or similar 3.3V FPGA platform
// Language   : Verilog-2001
//
// PURPOSE
// ----------------------------------------------------------------------------
// This module implements a deterministic FPGA-side controller for a
// multi-architecture differential analysis lab. It coordinates:
//
//   1) A HOST PC connected through a control UART (UART3)
//   2) Three target BOARDS connected through UART0, UART1, and UART2
//   3) A TRIGGER output used to synchronize oscilloscope capture
//
// The controller supports four host-visible functions:
//
//   - Connectivity / liveness checking
//   - Configuration of experiment bounds and policy hints
//   - Standard differential runs
//   - Replay-window runs for hierarchical minimization
//
// Example workflow:
//
//   Host  -> FPGA : "PING\n"
//   FPGA  -> Host : "PONG\n"
//
//   Host  -> FPGA :
//      "CFG SEED_START=00100000 SEED_END=0010FFFF "
//      "STEPS_MIN=64 STEPS_MAX=1024 POLICY=1\n"
//   FPGA  -> Host : "ACK CFG\n"
//
//   Host  -> FPGA : "RUN SEED=00112233 STEPS=256\n"
//   FPGA  -> Host : "ACK RUN\n"
//
//   Host  -> FPGA :
//      "REPLAY SEED=00112233 STEPS=256 START=160 COUNT=32\n"
//   FPGA  -> Host : "ACK REPLAY\n"
//
//   FPGA  -> B0   : "RUN 00112233 256\n"
//   FPGA  -> B1   : ...
//   FPGA  -> B2   : ...
//
//   Or replay mode:
//
//   FPGA  -> B0   : "REPLAY 00112233 256 160 32\n"
//   FPGA  -> B1   : ...
//   FPGA  -> B2   : ...
//
//   B0    -> FPGA : "DONE 00112233 123456789 00000000 5 1842\n"
//   B1    -> FPGA : ...
//   B2    -> FPGA : ...
//
//   FPGA  -> Host :
//      "TRIPLE SEED=00112233 STEPS=256 "
//      "S0=<score> F0=<flags> W0=<worst_window> WC0=<worst_cycles> T0=<0|1> D0=<duration> "
//      "S1=<score> F1=<flags> W1=<worst_window> WC1=<worst_cycles> T1=<0|1> D1=<duration> "
//      "S2=<score> F2=<flags> W2=<worst_window> WC2=<worst_cycles> T2=<0|1> D2=<duration> "
//      "RS=<replay_start> RC=<replay_count>\n"
//
// DESIGN NOTES
// ----------------------------------------------------------------------------
// - UART3 is reserved for HOST communication.
// - UART0/1/2 are reserved for the three target boards.
// - Boards are exercised sequentially, not in parallel.
// - Trigger goes high before command send, remains high during board execution,
//   and goes low after a configurable post-run hold time.
// - If a board never returns DONE before timeout, a timeout result is synthesized.
// - Replay-window commands let the host request execution over a smaller
//   instruction subrange for hierarchical minimization.
//
// LIMITATIONS
// ----------------------------------------------------------------------------
// - Parsing assumes valid newline-terminated ASCII commands.
// - DONE parsing assumes exact field order.
// - No CRC / checksum / framing beyond UART byte stream and newline.
// - No autonomous scheduling logic is implemented in hardware.
// - Loops are intentionally written with constant bounds and without modifying
//   loop variables inside loop bodies to keep Vivado synthesis happy.
//
// ============================================================================

`timescale 1ns/1ps

module fpga_fuzz_ctrl #(
    parameter integer CLK_HZ               = 100_000_000,
    parameter integer UART_BAUD            = 115200,
    parameter integer PRETRIG_CYCLES       = 5_000_000,
    parameter integer POSTDONE_CYCLES      = 5_000_000,
    parameter integer GAP_CYCLES           = 2_000_000,
    parameter integer BOARD_TIMEOUT_CYCLES = 300_000_000,
    parameter [31:0] TIMEOUT_FAULT_CODE    = 32'h0000_0008
)(
    input  wire clk,
    input  wire rst_n,

    // Host UART3
    input  wire uart3_rx,
    output wire uart3_tx,

    // Board UART0
    input  wire uart0_rx,
    output wire uart0_tx,

    // Board UART1
    input  wire uart1_rx,
    output wire uart1_tx,

    // Board UART2
    input  wire uart2_rx,
    output wire uart2_tx,

    // Oscilloscope trigger output
    output reg  trig_out
);

    // =========================================================================
    // UART interfaces
    // =========================================================================
    wire [7:0] h_rx_data;
    wire       h_rx_valid;
    reg  [7:0] h_tx_data;
    reg        h_tx_start;
    wire       h_tx_busy;

    wire [7:0] b0_rx_data;
    wire       b0_rx_valid;
    reg  [7:0] b0_tx_data;
    reg        b0_tx_start;
    wire       b0_tx_busy;

    wire [7:0] b1_rx_data;
    wire       b1_rx_valid;
    reg  [7:0] b1_tx_data;
    reg        b1_tx_start;
    wire       b1_tx_busy;

    wire [7:0] b2_rx_data;
    wire       b2_rx_valid;
    reg  [7:0] b2_tx_data;
    reg        b2_tx_start;
    wire       b2_tx_busy;

    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(UART_BAUD)) U_H_RX (
        .clk(clk), .rst_n(rst_n), .rx(uart3_rx), .data(h_rx_data), .valid(h_rx_valid)
    );

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(UART_BAUD)) U_H_TX (
        .clk(clk), .rst_n(rst_n), .tx(uart3_tx), .data(h_tx_data), .start(h_tx_start), .busy(h_tx_busy)
    );

    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(UART_BAUD)) U_B0_RX (
        .clk(clk), .rst_n(rst_n), .rx(uart0_rx), .data(b0_rx_data), .valid(b0_rx_valid)
    );

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(UART_BAUD)) U_B0_TX (
        .clk(clk), .rst_n(rst_n), .tx(uart0_tx), .data(b0_tx_data), .start(b0_tx_start), .busy(b0_tx_busy)
    );

    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(UART_BAUD)) U_B1_RX (
        .clk(clk), .rst_n(rst_n), .rx(uart1_rx), .data(b1_rx_data), .valid(b1_rx_valid)
    );

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(UART_BAUD)) U_B1_TX (
        .clk(clk), .rst_n(rst_n), .tx(uart1_tx), .data(b1_tx_data), .start(b1_tx_start), .busy(b1_tx_busy)
    );

    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(UART_BAUD)) U_B2_RX (
        .clk(clk), .rst_n(rst_n), .rx(uart2_rx), .data(b2_rx_data), .valid(b2_rx_valid)
    );

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(UART_BAUD)) U_B2_TX (
        .clk(clk), .rst_n(rst_n), .tx(uart2_tx), .data(b2_tx_data), .start(b2_tx_start), .busy(b2_tx_busy)
    );

    // =========================================================================
    // Basic helpers
    // =========================================================================

    // Convert one ASCII hex character into a 4-bit nibble.
    function [3:0] hexval;
        input [7:0] c;
        begin
            if (c >= "0" && c <= "9")
                hexval = c - "0";
            else if (c >= "A" && c <= "F")
                hexval = c - "A" + 4'd10;
            else if (c >= "a" && c <= "f")
                hexval = c - "a" + 4'd10;
            else
                hexval = 4'h0;
        end
    endfunction

    // Convert a nibble to uppercase ASCII hex.
    function [7:0] nibble_to_ascii;
        input [3:0] n;
        begin
            if (n < 10)
                nibble_to_ascii = "0" + n;
            else
                nibble_to_ascii = "A" + (n - 10);
        end
    endfunction

    // Convert a small integer digit 0..9 to ASCII.
    function [7:0] digit_ascii;
        input integer d;
        begin
            digit_ascii = "0" + d[7:0];
        end
    endfunction

    // =========================================================================
    // Host parser
    // =========================================================================

    // Raw host command line buffer.
    reg [7:0] host_line [0:127];
    reg [6:0] host_len;

    // Host command types.
    localparam [1:0]
        HCMD_NONE = 2'd0,
        HCMD_PING = 2'd1,
        HCMD_CFG  = 2'd2,
        HCMD_RUN  = 2'd3;

    reg [1:0] host_cmd_type;
    reg       host_cmd_pending;
    reg       host_cmd_replay;
    reg       host_cmd_consume;

    // Requested run parameters from host.
    reg [31:0] run_seed;
    reg [15:0] run_steps;
    reg [31:0] replay_start;
    reg [31:0] replay_count;

    // Stored configuration from CFG command.
    reg [31:0] cfg_seed_start;
    reg [31:0] cfg_seed_end;
    reg [15:0] cfg_steps_min;
    reg [15:0] cfg_steps_max;
    reg [15:0] cfg_policy_hint;

    // Parse exactly 8 hex chars from the host line.
    function [31:0] parse_hex8_host;
        input integer idx;
        integer i;
        reg [31:0] v;
        begin
            v = 0;
            for (i = 0; i < 8; i = i + 1)
                v = (v << 4) | hexval(host_line[idx + i]);
            parse_hex8_host = v;
        end
    endfunction

    // Parse decimal from host line using only constant-bound loops.
    function [31:0] parse_dec_u32_host;
        input integer idx;
        input integer maxlen;
        integer i;
        reg [31:0] v;
        reg stop;
        begin
            v = 0;
            stop = 1'b0;
            for (i = 0; i < 128; i = i + 1) begin
                if (!stop && (i >= idx)) begin
                    if ((i < maxlen) && (host_line[i] >= "0") && (host_line[i] <= "9"))
                        v = (v * 10) + (host_line[i] - "0");
                    else
                        stop = 1'b1;
                end
            end
            parse_dec_u32_host = v;
        end
    endfunction

    // Find KEY= in host line and return index of first char after '='.
    // This version avoids:
    //   - variable loop limits
    //   - editing the loop variable inside the loop
    function integer find_key_eq_host;
        input integer maxlen;
        input [8*12-1:0] key;
        input integer keylen;
        integer i, j;
        reg match;
        reg found;
        begin
            find_key_eq_host = -1;
            found = 1'b0;

            for (i = 0; i < 128; i = i + 1) begin
                if (!found) begin
                    if ((i + keylen) <= maxlen) begin
                        match = 1'b1;
                        for (j = 0; j < 12; j = j + 1) begin
                            if (j < keylen) begin
                                if (host_line[i + j] != key[8*(keylen - 1 - j) +: 8])
                                    match = 1'b0;
                            end
                        end
                        if (match) begin
                            find_key_eq_host = i + keylen;
                            found = 1'b1;
                        end
                    end
                end
            end
        end
    endfunction

    // Temporary parser positions.
    integer pos_seed;
    integer pos_steps;
    integer pos_ss;
    integer pos_se;
    integer pos_sm;
    integer pos_sx;
    integer pos_pol;
    integer pos_start;
    integer pos_count;

    // Host parser process.
    always @(posedge clk) begin
        if (!rst_n) begin
            host_len         <= 0;
            host_cmd_type    <= HCMD_NONE;
            host_cmd_pending <= 1'b0;
            host_cmd_replay  <= 1'b0;

            run_seed         <= 32'd0;
            run_steps        <= 16'd0;
            replay_start     <= 32'd0;
            replay_count     <= 32'd0;

            cfg_seed_start   <= 32'h0000_0001;
            cfg_seed_end     <= 32'h0001_0000;
            cfg_steps_min    <= 16'd64;
            cfg_steps_max    <= 16'd1024;
            cfg_policy_hint  <= 16'd0;
        end else begin
            // Clear pending command after FSM consumes it.
            if (host_cmd_consume) begin
                host_cmd_pending <= 1'b0;
                host_cmd_type    <= HCMD_NONE;
                host_cmd_replay  <= 1'b0;
            end

            if (h_rx_valid) begin
                // Newline or CR ends a command line.
                if ((h_rx_data == 8'h0A) || (h_rx_data == 8'h0D)) begin
                    if (!host_cmd_pending) begin
                        // PING
                        if ((host_len >= 4) &&
                            (host_line[0] == "P") && (host_line[1] == "I") &&
                            (host_line[2] == "N") && (host_line[3] == "G")) begin
                            host_cmd_type    <= HCMD_PING;
                            host_cmd_pending <= 1'b1;
                            host_cmd_replay  <= 1'b0;
                        end

                        // CFG
                        else if ((host_len >= 3) &&
                                 (host_line[0] == "C") && (host_line[1] == "F") && (host_line[2] == "G")) begin
                            pos_ss  = find_key_eq_host(host_len, "SEED_START=", 11);
                            pos_se  = find_key_eq_host(host_len, "SEED_END=",    9);
                            pos_sm  = find_key_eq_host(host_len, "STEPS_MIN=",  10);
                            pos_sx  = find_key_eq_host(host_len, "STEPS_MAX=",  10);
                            pos_pol = find_key_eq_host(host_len, "POLICY=",      7);

                            if (pos_ss  >= 0) cfg_seed_start  <= parse_hex8_host(pos_ss);
                            if (pos_se  >= 0) cfg_seed_end    <= parse_hex8_host(pos_se);
                            if (pos_sm  >= 0) cfg_steps_min   <= parse_dec_u32_host(pos_sm, host_len);
                            if (pos_sx  >= 0) cfg_steps_max   <= parse_dec_u32_host(pos_sx, host_len);
                            if (pos_pol >= 0) cfg_policy_hint <= parse_dec_u32_host(pos_pol, host_len);

                            host_cmd_type    <= HCMD_CFG;
                            host_cmd_pending <= 1'b1;
                            host_cmd_replay  <= 1'b0;
                        end

                        // REPLAY
                        else if ((host_len >= 6) &&
                                 (host_line[0] == "R") && (host_line[1] == "E") &&
                                 (host_line[2] == "P") && (host_line[3] == "L") &&
                                 (host_line[4] == "A") && (host_line[5] == "Y")) begin
                            pos_seed  = find_key_eq_host(host_len, "SEED=",  5);
                            pos_steps = find_key_eq_host(host_len, "STEPS=", 6);
                            pos_start = find_key_eq_host(host_len, "START=", 6);
                            pos_count = find_key_eq_host(host_len, "COUNT=", 6);

                            if (pos_seed  >= 0) run_seed     <= parse_hex8_host(pos_seed);
                            if (pos_steps >= 0) run_steps    <= parse_dec_u32_host(pos_steps, host_len);
                            if (pos_start >= 0) replay_start <= parse_dec_u32_host(pos_start, host_len);
                            if (pos_count >= 0) replay_count <= parse_dec_u32_host(pos_count, host_len);

                            host_cmd_type    <= HCMD_RUN;
                            host_cmd_pending <= 1'b1;
                            host_cmd_replay  <= 1'b1;
                        end

                        // RUN
                        else if ((host_len >= 3) &&
                                 (host_line[0] == "R") && (host_line[1] == "U") && (host_line[2] == "N")) begin
                            pos_seed  = find_key_eq_host(host_len, "SEED=",  5);
                            pos_steps = find_key_eq_host(host_len, "STEPS=", 6);

                            if (pos_seed  >= 0) run_seed  <= parse_hex8_host(pos_seed);
                            if (pos_steps >= 0) run_steps <= parse_dec_u32_host(pos_steps, host_len);

                            host_cmd_type    <= HCMD_RUN;
                            host_cmd_pending <= 1'b1;
                            host_cmd_replay  <= 1'b0;
                        end
                    end

                    // Clear line buffer length after end-of-line.
                    host_len <= 0;
                end else begin
                    if (host_len < 127) begin
                        host_line[host_len] <= h_rx_data;
                        host_len            <= host_len + 1'b1;
                    end
                end
            end
        end
    end

    // =========================================================================
    // Board response parser
    // =========================================================================

    // Raw board response line buffer for currently selected board.
    reg [7:0] b_line [0:127];
    reg [6:0] b_len;
    reg [1:0] cur_board;

    // Final per-board results captured by the controller.
    reg        done_got          [0:2];
    reg [31:0] done_seed         [0:2];
    reg [31:0] done_flags        [0:2];
    reg [63:0] done_score        [0:2];
    reg [31:0] done_worst_window [0:2];
    reg [63:0] done_worst_cycles [0:2];
    reg        done_timeout      [0:2];
    reg [63:0] done_duration     [0:2];

    // Single-cycle parser pulse into FSM.
    reg        board_done_pulse;
    reg [1:0]  board_done_board;
    reg [31:0] board_done_seed;
    reg [31:0] board_done_flags;
    reg [63:0] board_done_score;
    reg [31:0] board_done_worst_window;
    reg [63:0] board_done_worst_cycles;

    // Select currently active board RX stream.
    wire [7:0] bx_data  = (cur_board == 2'd0) ? b0_rx_data  :
                          (cur_board == 2'd1) ? b1_rx_data  :
                                                 b2_rx_data;

    wire       bx_valid = (cur_board == 2'd0) ? b0_rx_valid :
                          (cur_board == 2'd1) ? b1_rx_valid :
                                                 b2_rx_valid;

    // Parse exactly 8 hex chars from board line.
    function [31:0] parse_hex8_board;
        input integer idx;
        integer i;
        reg [31:0] v;
        begin
            v = 0;
            for (i = 0; i < 8; i = i + 1)
                v = (v << 4) | hexval(b_line[idx + i]);
            parse_hex8_board = v;
        end
    endfunction

    // Parse decimal u32 from board line.
    function [31:0] parse_dec_u32_board;
        input integer idx;
        input integer maxlen;
        integer i;
        reg [31:0] v;
        reg stop;
        begin
            v = 0;
            stop = 1'b0;
            for (i = 0; i < 128; i = i + 1) begin
                if (!stop && (i >= idx)) begin
                    if ((i < maxlen) && (b_line[i] >= "0") && (b_line[i] <= "9"))
                        v = (v * 10) + (b_line[i] - "0");
                    else
                        stop = 1'b1;
                end
            end
            parse_dec_u32_board = v;
        end
    endfunction

    // Parse decimal u64 from board line.
    function [63:0] parse_dec_u64_board;
        input integer idx;
        input integer maxlen;
        integer i;
        reg [63:0] v;
        reg stop;
        begin
            v = 0;
            stop = 1'b0;
            for (i = 0; i < 128; i = i + 1) begin
                if (!stop && (i >= idx)) begin
                    if ((i < maxlen) && (b_line[i] >= "0") && (b_line[i] <= "9"))
                        v = (v * 10) + (b_line[i] - "0");
                    else
                        stop = 1'b1;
                end
            end
            parse_dec_u64_board = v;
        end
    endfunction

    // Find the next token start after a space.
    // This version does not modify loop variables inside the loop.
    function integer find_token_start_after_space_board;
        input integer idx;
        input integer maxlen;
        integer i;
        reg seen_space;
        reg found;
        begin
            seen_space = 1'b0;
            found = 1'b0;
            find_token_start_after_space_board = maxlen;

            for (i = 0; i < 128; i = i + 1) begin
                if (!found && (i >= idx) && (i < maxlen)) begin
                    if (!seen_space) begin
                        if (b_line[i] == " ")
                            seen_space = 1'b1;
                    end else begin
                        if (b_line[i] != " ") begin
                            find_token_start_after_space_board = i;
                            found = 1'b1;
                        end
                    end
                end
            end
        end
    endfunction

    integer score_idx;
    integer flags_idx;
    integer win_idx;
    integer wcyc_idx;

    // Board response parser process.
    always @(posedge clk) begin
        if (!rst_n) begin
            b_len                   <= 0;
            board_done_pulse        <= 1'b0;
            board_done_board        <= 2'd0;
            board_done_seed         <= 32'd0;
            board_done_flags        <= 32'd0;
            board_done_score        <= 64'd0;
            board_done_worst_window <= 32'd0;
            board_done_worst_cycles <= 64'd0;
        end else begin
            board_done_pulse <= 1'b0;

            if (bx_valid) begin
                if ((bx_data == 8'h0A) || (bx_data == 8'h0D)) begin
                    if ((b_len >= 20) &&
                        (b_line[0] == "D") && (b_line[1] == "O") &&
                        (b_line[2] == "N") && (b_line[3] == "E") && (b_line[4] == " ")) begin

                        // DONE <8hex> <score> <flags> <worst_window> <worst_cycles>
                        score_idx = 14;
                        flags_idx = find_token_start_after_space_board(score_idx, b_len);
                        win_idx   = find_token_start_after_space_board(flags_idx, b_len);
                        wcyc_idx  = find_token_start_after_space_board(win_idx,   b_len);

                        board_done_board        <= cur_board;
                        board_done_seed         <= parse_hex8_board(5);
                        board_done_score        <= parse_dec_u64_board(score_idx, b_len);
                        board_done_flags        <= ((flags_idx + 8) <= b_len) ? parse_hex8_board(flags_idx) : 32'hDEAD_BEEF;
                        board_done_worst_window <= parse_dec_u32_board(win_idx,  b_len);
                        board_done_worst_cycles <= parse_dec_u64_board(wcyc_idx, b_len);
                        board_done_pulse        <= 1'b1;
                    end

                    b_len <= 0;
                end else begin
                    if (b_len < 127) begin
                        b_line[b_len] <= bx_data;
                        b_len         <= b_len + 1'b1;
                    end
                end
            end
        end
    end

    // =========================================================================
    // TX buffer / ASCII formatter
    // =========================================================================

    // Shared transmit buffer used for both host and board commands.
    reg [7:0] txbuf [0:511];
    reg [8:0] txlen;
    reg [8:0] txidx;
    reg       tx_active;
    reg [1:0] tx_target; // 0=b0,1=b1,2=b2,3=host

    // Append one ASCII byte to tx buffer.
    task append_ascii_char;
        input [7:0] c;
        begin
            txbuf[txlen] = c;
            txlen        = txlen + 1'b1;
        end
    endtask

    // Append 8 uppercase hex digits.
    task append_hex8_to_txbuf;
        input [31:0] val;
        integer q;
        reg [3:0] nib;
        begin
            for (q = 0; q < 8; q = q + 1) begin
                nib = (val >> (28 - 4*q)) & 4'hF;
                txbuf[txlen] = nibble_to_ascii(nib);
                txlen        = txlen + 1'b1;
            end
        end
    endtask

    // Append decimal u32 using only constant-bound loops.
    task append_u32_dec_to_txbuf;
        input [31:0] val;
        integer u;
        integer dlen;
        reg [7:0] digits [0:15];
        reg [31:0] x;
        reg stop;
        begin
            dlen = 0;
            x    = val;
            stop = 1'b0;

            if (x == 0) begin
                digits[0] = "0";
                dlen = 1;
            end else begin
                for (u = 0; u < 10; u = u + 1) begin
                    if (!stop) begin
                        digits[dlen] = "0" + (x % 10);
                        x = x / 10;
                        dlen = dlen + 1;
                        if (x == 0)
                            stop = 1'b1;
                    end
                end
            end

            for (u = 0; u < 10; u = u + 1) begin
                if (u < dlen) begin
                    txbuf[txlen] = digits[dlen - 1 - u];
                    txlen = txlen + 1'b1;
                end
            end
        end
    endtask

    // Append decimal u64 using only constant-bound loops.
    task append_u64_dec_to_txbuf;
        input [63:0] val;
        integer u;
        integer dlen;
        reg [7:0] digits [0:31];
        reg [63:0] x;
        reg stop;
        begin
            dlen = 0;
            x    = val;
            stop = 1'b0;

            if (x == 0) begin
                digits[0] = "0";
                dlen = 1;
            end else begin
                for (u = 0; u < 20; u = u + 1) begin
                    if (!stop) begin
                        digits[dlen] = "0" + (x % 10);
                        x = x / 10;
                        dlen = dlen + 1;
                        if (x == 0)
                            stop = 1'b1;
                    end
                end
            end

            for (u = 0; u < 20; u = u + 1) begin
                if (u < dlen) begin
                    txbuf[txlen] = digits[dlen - 1 - u];
                    txlen = txlen + 1'b1;
                end
            end
        end
    endtask

    // Build "PONG\n".
    task tx_load_literal_pong;
        begin
            txlen = 0;
            append_ascii_char("P");
            append_ascii_char("O");
            append_ascii_char("N");
            append_ascii_char("G");
            append_ascii_char(8'h0A);
        end
    endtask

    // Build "ACK CFG\n".
    task tx_load_literal_ack_cfg;
        begin
            txlen = 0;
            append_ascii_char("A");
            append_ascii_char("C");
            append_ascii_char("K");
            append_ascii_char(" ");
            append_ascii_char("C");
            append_ascii_char("F");
            append_ascii_char("G");
            append_ascii_char(8'h0A);
        end
    endtask

    // Build "ACK RUN\n".
    task tx_load_literal_ack_run;
        begin
            txlen = 0;
            append_ascii_char("A");
            append_ascii_char("C");
            append_ascii_char("K");
            append_ascii_char(" ");
            append_ascii_char("R");
            append_ascii_char("U");
            append_ascii_char("N");
            append_ascii_char(8'h0A);
        end
    endtask

    // Build "ACK REPLAY\n".
    task tx_load_literal_ack_replay;
        begin
            txlen = 0;
            append_ascii_char("A");
            append_ascii_char("C");
            append_ascii_char("K");
            append_ascii_char(" ");
            append_ascii_char("R");
            append_ascii_char("E");
            append_ascii_char("P");
            append_ascii_char("L");
            append_ascii_char("A");
            append_ascii_char("Y");
            append_ascii_char(8'h0A);
        end
    endtask

    // Build "RUN <seed> <steps>\n" for a board.
    task tx_load_run;
        input [31:0] seed;
        input [15:0] steps;
        begin
            txlen = 0;
            append_ascii_char("R");
            append_ascii_char("U");
            append_ascii_char("N");
            append_ascii_char(" ");
            append_hex8_to_txbuf(seed);
            append_ascii_char(" ");
            append_u32_dec_to_txbuf({16'h0, steps});
            append_ascii_char(8'h0A);
        end
    endtask

    // Build "REPLAY <seed> <steps> <start> <count>\n" for a board.
    task tx_load_replay;
        input [31:0] seed;
        input [15:0] steps;
        input [31:0] start;
        input [31:0] count;
        begin
            txlen = 0;
            append_ascii_char("R");
            append_ascii_char("E");
            append_ascii_char("P");
            append_ascii_char("L");
            append_ascii_char("A");
            append_ascii_char("Y");
            append_ascii_char(" ");
            append_hex8_to_txbuf(seed);
            append_ascii_char(" ");
            append_u32_dec_to_txbuf({16'h0, steps});
            append_ascii_char(" ");
            append_u32_dec_to_txbuf(start);
            append_ascii_char(" ");
            append_u32_dec_to_txbuf(count);
            append_ascii_char(8'h0A);
        end
    endtask

    // Append one board's result fields to the TRIPLE line.
    task append_triplet_board;
        input integer idx;
        begin
            append_ascii_char(" ");
            append_ascii_char("S");
            append_ascii_char(digit_ascii(idx));
            append_ascii_char("=");
            append_u64_dec_to_txbuf(done_score[idx]);

            append_ascii_char(" ");
            append_ascii_char("F");
            append_ascii_char(digit_ascii(idx));
            append_ascii_char("=");
            append_hex8_to_txbuf(done_flags[idx]);

            append_ascii_char(" ");
            append_ascii_char("W");
            append_ascii_char(digit_ascii(idx));
            append_ascii_char("=");
            append_u32_dec_to_txbuf(done_worst_window[idx]);

            append_ascii_char(" ");
            append_ascii_char("W");
            append_ascii_char("C");
            append_ascii_char(digit_ascii(idx));
            append_ascii_char("=");
            append_u64_dec_to_txbuf(done_worst_cycles[idx]);

            append_ascii_char(" ");
            append_ascii_char("T");
            append_ascii_char(digit_ascii(idx));
            append_ascii_char("=");
            append_u32_dec_to_txbuf({31'd0, done_timeout[idx]});

            append_ascii_char(" ");
            append_ascii_char("D");
            append_ascii_char(digit_ascii(idx));
            append_ascii_char("=");
            append_u64_dec_to_txbuf(done_duration[idx]);
        end
    endtask

    // Build the full aggregated TRIPLE line for the host.
    task tx_load_triple;
        input [31:0] seed;
        input [15:0] steps;
        input [31:0] rs;
        input [31:0] rc;
        begin
            txlen = 0;

            append_ascii_char("T");
            append_ascii_char("R");
            append_ascii_char("I");
            append_ascii_char("P");
            append_ascii_char("L");
            append_ascii_char("E");

            append_ascii_char(" ");
            append_ascii_char("S");
            append_ascii_char("E");
            append_ascii_char("E");
            append_ascii_char("D");
            append_ascii_char("=");
            append_hex8_to_txbuf(seed);

            append_ascii_char(" ");
            append_ascii_char("S");
            append_ascii_char("T");
            append_ascii_char("E");
            append_ascii_char("P");
            append_ascii_char("S");
            append_ascii_char("=");
            append_u32_dec_to_txbuf({16'h0, steps});

            append_triplet_board(0);
            append_triplet_board(1);
            append_triplet_board(2);

            append_ascii_char(" ");
            append_ascii_char("R");
            append_ascii_char("S");
            append_ascii_char("=");
            append_u32_dec_to_txbuf(rs);

            append_ascii_char(" ");
            append_ascii_char("R");
            append_ascii_char("C");
            append_ascii_char("=");
            append_u32_dec_to_txbuf(rc);

            append_ascii_char(8'h0A);
        end
    endtask

    // Start transmission of the currently built message to one target.
    task tx_start_msg;
        input [1:0] target;
        begin
            tx_target <= target;
            txidx     <= 0;
            tx_active <= 1'b1;
        end
    endtask

    // Shared UART transmit sequencer.
    always @(posedge clk) begin
        if (!rst_n) begin
            h_tx_start  <= 1'b0;
            b0_tx_start <= 1'b0;
            b1_tx_start <= 1'b0;
            b2_tx_start <= 1'b0;
            tx_active   <= 1'b0;
            txidx       <= 9'd0;
            txlen       <= 9'd0;
            tx_target   <= 2'd3;
            h_tx_data   <= 8'd0;
            b0_tx_data  <= 8'd0;
            b1_tx_data  <= 8'd0;
            b2_tx_data  <= 8'd0;
        end else begin
            h_tx_start  <= 1'b0;
            b0_tx_start <= 1'b0;
            b1_tx_start <= 1'b0;
            b2_tx_start <= 1'b0;

            if (tx_active) begin
                if (txidx < txlen) begin
                    if ((tx_target == 2'd3) && !h_tx_busy) begin
                        h_tx_data  <= txbuf[txidx];
                        h_tx_start <= 1'b1;
                        txidx      <= txidx + 1'b1;
                    end else if ((tx_target == 2'd0) && !b0_tx_busy) begin
                        b0_tx_data  <= txbuf[txidx];
                        b0_tx_start <= 1'b1;
                        txidx       <= txidx + 1'b1;
                    end else if ((tx_target == 2'd1) && !b1_tx_busy) begin
                        b1_tx_data  <= txbuf[txidx];
                        b1_tx_start <= 1'b1;
                        txidx       <= txidx + 1'b1;
                    end else if ((tx_target == 2'd2) && !b2_tx_busy) begin
                        b2_tx_data  <= txbuf[txidx];
                        b2_tx_start <= 1'b1;
                        txidx       <= txidx + 1'b1;
                    end
                end else begin
                    tx_active <= 1'b0;
                end
            end
        end
    end

    // =========================================================================
    // Main control FSM
    // =========================================================================

    localparam [3:0]
        ST_IDLE      = 4'd0,
        ST_HOST_MSG  = 4'd1,
        ST_PRETRIG   = 4'd2,
        ST_SEND_CMD  = 4'd3,
        ST_WAIT_DONE = 4'd4,
        ST_TIMEOUT   = 4'd5,
        ST_POSTDONE  = 4'd6,
        ST_GAP       = 4'd7,
        ST_SEND_TRIP = 4'd8;

    reg [3:0]  st;
    reg [31:0] seed_latched;
    reg [15:0] steps_latched;
    reg [31:0] replay_start_latched;
    reg [31:0] replay_count_latched;
    reg        is_replay_latched;

    reg [31:0] ctr;
    reg [31:0] board_wait_ctr;
    reg [63:0] run_duration_ctr;

    integer ci;

    // Main controller FSM.
    always @(posedge clk) begin
        if (!rst_n) begin
            st                   <= ST_IDLE;
            trig_out             <= 1'b0;
            ctr                  <= 32'd0;
            board_wait_ctr       <= 32'd0;
            run_duration_ctr     <= 64'd0;
            cur_board            <= 2'd0;
            host_cmd_consume     <= 1'b0;

            seed_latched         <= 32'd0;
            steps_latched        <= 16'd0;
            replay_start_latched <= 32'd0;
            replay_count_latched <= 32'd0;
            is_replay_latched    <= 1'b0;

            for (ci = 0; ci < 3; ci = ci + 1) begin
                done_got[ci]          <= 1'b0;
                done_seed[ci]         <= 32'd0;
                done_flags[ci]        <= 32'd0;
                done_score[ci]        <= 64'd0;
                done_worst_window[ci] <= 32'd0;
                done_worst_cycles[ci] <= 64'd0;
                done_timeout[ci]      <= 1'b0;
                done_duration[ci]     <= 64'd0;
            end
        end else begin
            host_cmd_consume <= 1'b0;

            case (st)
                // -------------------------------------------------------------
                // Wait for a parsed host command.
                // -------------------------------------------------------------
                ST_IDLE: begin
                    trig_out <= 1'b0;
                    if (host_cmd_pending && !tx_active)
                        st <= ST_HOST_MSG;
                end

                // -------------------------------------------------------------
                // Send PONG / ACK or latch run request.
                // -------------------------------------------------------------
                ST_HOST_MSG: begin
                    if (!tx_active) begin
                        if (host_cmd_type == HCMD_PING) begin
                            tx_load_literal_pong();
                            tx_start_msg(2'd3);
                            host_cmd_consume <= 1'b1;
                            st <= ST_IDLE;
                        end
                        else if (host_cmd_type == HCMD_CFG) begin
                            tx_load_literal_ack_cfg();
                            tx_start_msg(2'd3);
                            host_cmd_consume <= 1'b1;
                            st <= ST_IDLE;
                        end
                        else if (host_cmd_type == HCMD_RUN) begin
                            seed_latched         <= run_seed;
                            steps_latched        <= run_steps;
                            replay_start_latched <= replay_start;
                            replay_count_latched <= replay_count;
                            is_replay_latched    <= host_cmd_replay;

                            // Clear per-board result-valid/timeouts for the new run.
                            done_got[0]      <= 1'b0;
                            done_got[1]      <= 1'b0;
                            done_got[2]      <= 1'b0;
                            done_timeout[0]  <= 1'b0;
                            done_timeout[1]  <= 1'b0;
                            done_timeout[2]  <= 1'b0;
                            done_duration[0] <= 64'd0;
                            done_duration[1] <= 64'd0;
                            done_duration[2] <= 64'd0;

                            cur_board <= 2'd0;
                            ctr       <= 32'd0;
                            trig_out  <= 1'b1;

                            if (host_cmd_replay)
                                tx_load_literal_ack_replay();
                            else
                                tx_load_literal_ack_run();

                            tx_start_msg(2'd3);
                            host_cmd_consume <= 1'b1;
                            st <= ST_PRETRIG;
                        end
                        else begin
                            st <= ST_IDLE;
                        end
                    end
                end

                // -------------------------------------------------------------
                // Hold trigger high before sending board command.
                // -------------------------------------------------------------
                ST_PRETRIG: begin
                    if (ctr < PRETRIG_CYCLES)
                        ctr <= ctr + 1'b1;
                    else begin
                        ctr <= 32'd0;
                        st  <= ST_SEND_CMD;
                    end
                end

                // -------------------------------------------------------------
                // Send RUN/REPLAY to current board.
                // -------------------------------------------------------------
                ST_SEND_CMD: begin
                    if (!tx_active) begin
                        if (is_replay_latched)
                            tx_load_replay(seed_latched, steps_latched, replay_start_latched, replay_count_latched);
                        else
                            tx_load_run(seed_latched, steps_latched);

                        tx_start_msg(cur_board);
                        board_wait_ctr   <= 32'd0;
                        run_duration_ctr <= 64'd0;
                        st <= ST_WAIT_DONE;
                    end
                end

                // -------------------------------------------------------------
                // Wait for DONE from current board or timeout.
                // -------------------------------------------------------------
                ST_WAIT_DONE: begin
                    board_wait_ctr   <= board_wait_ctr + 1'b1;
                    run_duration_ctr <= run_duration_ctr + 1'b1;

                    if (board_done_pulse && (board_done_board == cur_board)) begin
                        done_seed[cur_board]         <= board_done_seed;
                        done_score[cur_board]        <= board_done_score;
                        done_flags[cur_board]        <= board_done_flags;
                        done_worst_window[cur_board] <= board_done_worst_window;
                        done_worst_cycles[cur_board] <= board_done_worst_cycles;
                        done_timeout[cur_board]      <= 1'b0;
                        done_duration[cur_board]     <= run_duration_ctr;
                        done_got[cur_board]          <= 1'b1;
                        ctr                          <= 32'd0;
                        st                           <= ST_POSTDONE;
                    end
                    else if (board_wait_ctr >= BOARD_TIMEOUT_CYCLES) begin
                        st <= ST_TIMEOUT;
                    end
                end

                // -------------------------------------------------------------
                // Synthesize timeout result if board did not respond.
                // -------------------------------------------------------------
                ST_TIMEOUT: begin
                    done_seed[cur_board]         <= seed_latched;
                    done_score[cur_board]        <= 64'd0;
                    done_flags[cur_board]        <= TIMEOUT_FAULT_CODE;
                    done_worst_window[cur_board] <= 32'd0;
                    done_worst_cycles[cur_board] <= 64'd0;
                    done_timeout[cur_board]      <= 1'b1;
                    done_duration[cur_board]     <= run_duration_ctr;
                    done_got[cur_board]          <= 1'b1;

                    ctr <= 32'd0;
                    st  <= ST_POSTDONE;
                end

                // -------------------------------------------------------------
                // Hold trigger after board completion.
                // -------------------------------------------------------------
                ST_POSTDONE: begin
                    if (ctr < POSTDONE_CYCLES)
                        ctr <= ctr + 1'b1;
                    else begin
                        trig_out <= 1'b0;
                        ctr      <= 32'd0;
                        st       <= ST_GAP;
                    end
                end

                // -------------------------------------------------------------
                // Gap before next board or send final TRIPLE line.
                // -------------------------------------------------------------
                ST_GAP: begin
                    if (ctr < GAP_CYCLES)
                        ctr <= ctr + 1'b1;
                    else begin
                        ctr <= 32'd0;
                        if (cur_board < 2) begin
                            cur_board <= cur_board + 1'b1;
                            trig_out  <= 1'b1;
                            st        <= ST_PRETRIG;
                        end else begin
                            st <= ST_SEND_TRIP;
                        end
                    end
                end

                // -------------------------------------------------------------
                // Send aggregated host result.
                // -------------------------------------------------------------
                ST_SEND_TRIP: begin
                    if (!tx_active) begin
                        tx_load_triple(seed_latched, steps_latched, replay_start_latched, replay_count_latched);
                        tx_start_msg(2'd3);
                        st <= ST_IDLE;
                    end
                end

                default: begin
                    st <= ST_IDLE;
                end
            endcase
        end
    end

endmodule

// ============================================================================
// UART RX: 8N1 receiver
// ----------------------------------------------------------------------------
// Simple UART receiver:
//   - waits for falling edge start bit
//   - samples in middle of bit cells
//   - captures 8 data bits, LSB first
//   - asserts valid for one clk when byte is ready
// ============================================================================
module uart_rx #(
    parameter integer CLK_HZ = 100_000_000,
    parameter integer BAUD   = 115200
)(
    input  wire clk,
    input  wire rst_n,
    input  wire rx,
    output reg  [7:0] data,
    output reg  valid
);
    localparam integer DIV  = CLK_HZ / BAUD;
    localparam integer HALF = DIV / 2;

    reg [15:0] divcnt;
    reg [3:0]  bitcnt;
    reg [7:0]  sh;
    reg        busy;
    reg        rx_d;

    always @(posedge clk) begin
        if (!rst_n) begin
            divcnt <= 0;
            bitcnt <= 0;
            sh     <= 0;
            busy   <= 0;
            valid  <= 0;
            data   <= 0;
            rx_d   <= 1'b1;
        end else begin
            valid <= 1'b0;
            rx_d  <= rx;

            if (!busy) begin
                // Detect start bit edge.
                if ((rx_d == 1'b1) && (rx == 1'b0)) begin
                    busy   <= 1'b1;
                    divcnt <= HALF;
                    bitcnt <= 0;
                end
            end else begin
                if (divcnt != 0) begin
                    divcnt <= divcnt - 1'b1;
                end else begin
                    divcnt <= DIV - 1;
                    if (bitcnt < 8) begin
                        sh     <= {rx, sh[7:1]};
                        bitcnt <= bitcnt + 1'b1;
                    end else begin
                        data  <= sh;
                        valid <= 1'b1;
                        busy  <= 1'b0;
                    end
                end
            end
        end
    end
endmodule

// ============================================================================
// UART TX: 8N1 transmitter
// ----------------------------------------------------------------------------
// Simple UART transmitter:
//   - start bit, 8 data bits, stop bit
//   - start pulse launches one frame
//   - busy stays high while frame is transmitting
// ============================================================================
module uart_tx #(
    parameter integer CLK_HZ = 100_000_000,
    parameter integer BAUD   = 115200
)(
    input  wire clk,
    input  wire rst_n,
    output reg  tx,
    input  wire [7:0] data,
    input  wire start,
    output reg  busy
);
    localparam integer DIV = CLK_HZ / BAUD;

    reg [15:0] divcnt;
    reg [3:0]  bitcnt;
    reg [9:0]  frame;

    always @(posedge clk) begin
        if (!rst_n) begin
            tx     <= 1'b1;
            busy   <= 1'b0;
            divcnt <= 0;
            bitcnt <= 0;
            frame  <= 10'h3FF;
        end else begin
            if (!busy) begin
                tx <= 1'b1;
                if (start) begin
                    // Frame = {stop bit, data[7:0], start bit}
                    frame  <= {1'b1, data, 1'b0};
                    busy   <= 1'b1;
                    divcnt <= DIV - 1;
                    bitcnt <= 0;
                    tx     <= 1'b0;
                end
            end else begin
                if (divcnt != 0) begin
                    divcnt <= divcnt - 1'b1;
                end else begin
                    divcnt <= DIV - 1;
                    bitcnt <= bitcnt + 1'b1;
                    frame  <= {1'b1, frame[9:1]};
                    tx     <= frame[1];
                    if (bitcnt == 9) begin
                        busy <= 1'b0;
                        tx   <= 1'b1;
                    end
                end
            end
        end
    end
endmodule