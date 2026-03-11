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
// The intended workflow is:
//
//   Host  -> FPGA : "PING\n"
//   FPGA  -> Host : "PONG\n"
//
//   Host  -> FPGA :
//      "CFG SEED_START=<8hex> SEED_END=<8hex> "
//      "STEPS_MIN=<dec> STEPS_MAX=<dec> POLICY=<dec>\n"
//   FPGA  -> Host : "ACK CFG\n"
//
//   Host  -> FPGA : "RUN SEED=<8hex> STEPS=<dec>\n"
//   FPGA  -> Host : "ACK RUN\n"
//
//   Host  -> FPGA :
//      "REPLAY SEED=<8hex> STEPS=<dec> START=<dec> COUNT=<dec>\n"
//   FPGA  -> Host : "ACK REPLAY\n"
//
//   FPGA  -> B0   : "RUN <8hex> <dec>\n"
//   FPGA  -> B1   : ...
//   FPGA  -> B2   : ...
//
//   or, for replay-mode operation:
//
//   FPGA  -> B0   : "REPLAY <8hex> <dec> <start> <count>\n"
//   FPGA  -> B1   : ...
//   FPGA  -> B2   : ...
//
//   B0    -> FPGA :
//      "DONE <8hex> <score> <flags> <worst_window> <worst_cycles>\n"
//   B1    -> FPGA : ...
//   B2    -> FPGA : ...
//
//   FPGA  -> Host :
//      "TRIPLE SEED=<8hex> STEPS=<dec> "
//      "S0=<score> F0=<flags> W0=<worst_window> WC0=<worst_cycles> T0=<0|1> D0=<duration> "
//      "S1=<score> F1=<flags> W1=<worst_window> WC1=<worst_cycles> T1=<0|1> D1=<duration> "
//      "S2=<score> F2=<flags> W2=<worst_window> WC2=<worst_cycles> T2=<0|1> D2=<duration> "
//      "RS=<replay_start> RC=<replay_count>\n"
//
// The trigger output emits three distinct pulses, one per board execution,
// enabling CH4 oscilloscope windowing for serialized differential capture.
//
// DESIGN NOTES
// ----------------------------------------------------------------------------
// - UART3 is reserved for HOST communication (e.g., FT232 USB-TTL cable).
// - UART0/1/2 are reserved for the three target boards.
// - The boards are exercised SEQUENTIALLY, not in parallel.
// - The trigger output goes high before a board command is sent, remains high
//   during board execution, and returns low after a configurable post-run
//   hold period.
// - The FPGA stores per-board results and forwards an aggregated TRIPLE line
//   to the host.
// - Divergence metrics are expected to be computed on the HOST, not here.
// - If a board never returns DONE before BOARD_TIMEOUT_CYCLES expires, the
//   FPGA synthesizes a timeout result using TIMEOUT_FAULT_CODE.
// - Replay-window commands allow the host to request execution over a smaller
//   instruction subrange for hierarchical anomaly minimization.
// - Configuration commands are latched internally and may be used by the host
//   as experiment metadata or future autonomous scheduling logic.
// - Additional metadata fields are forwarded to the host, including timeout
//   status and FPGA-observed board duration.
//
// LIMITATIONS
// ----------------------------------------------------------------------------
// - Host command parsing is intentionally lightweight and assumes valid,
//   newline-terminated ASCII commands.
// - Board response parsing assumes the DONE field order is exact.
// - No checksum / CRC / framing beyond UART byte stream and newline token.
// - No autonomous seed scheduling is performed in hardware; host policy logic
//   remains external.
// - No per-board retry logic is implemented after timeout.
// - Timeout synthesis indicates failure to respond, but does not distinguish
//   between hang, transport corruption, or board reset.
// - Replay behavior assumes the target runner supports the REPLAY command.
//
// RECOMMENDED EXTENSIONS
// ----------------------------------------------------------------------------
// - Add explicit board retry / reset signaling after timeout
// - Add host-side status / error summary messages beyond ACK/PONG
// - Add board-side support for richer metadata (e.g., perf counters,
//   retired instructions, replay-window local checksums)
// - Add autonomous FPGA seed stepping using CFG bounds for hardware-driven
//   batch execution
// - Add optional CRC or packet framing for stronger UART robustness
// - Add persistent logging FIFOs for raw host/board UART traffic
//
// INTERFACE SUMMARY
// ----------------------------------------------------------------------------
// Inputs:
//   clk        : system clock
//   rst_n      : active-low synchronous reset
//   uart3_rx   : host control UART RX
//   uart0_rx   : board0 UART RX
//   uart1_rx   : board1 UART RX
//   uart2_rx   : board2 UART RX
//
// Outputs:
//   uart3_tx   : host control UART TX
//   uart0_tx   : board0 UART TX
//   uart1_tx   : board1 UART TX
//   uart2_tx   : board2 UART TX
//   trig_out   : oscilloscope trigger output
//
// ASCII PROTOCOL
// ----------------------------------------------------------------------------
// Host -> FPGA
//   PING
//
// FPGA -> Host
//   PONG
//
// Host -> FPGA
//   CFG SEED_START=00100000 SEED_END=0010FFFF STEPS_MIN=64 STEPS_MAX=1024 POLICY=1
//
// FPGA -> Host
//   ACK CFG
//
// Host -> FPGA
//   RUN SEED=00112233 STEPS=256
//
// FPGA -> Host
//   ACK RUN
//
// Host -> FPGA
//   REPLAY SEED=00112233 STEPS=256 START=160 COUNT=32
//
// FPGA -> Host
//   ACK REPLAY
//
// FPGA -> Board
//   RUN 00112233 256
//
// FPGA -> Board (replay mode)
//   REPLAY 00112233 256 160 32
//
// Board -> FPGA
//   DONE 00112233 123456789 00000000 5 1842
//
// FPGA -> Host
//   TRIPLE SEED=00112233 STEPS=256
//          S0=123456789 F0=00000000 W0=5 WC0=1842 T0=0 D0=1043211
//          S1=123450001 F1=00000000 W1=5 WC1=1770 T1=0 D1=1035520
//          S2=0         F2=00000008 W2=0 WC2=0    T2=1 D2=300000000
//          RS=160 RC=32
//
// ============================================================================ 

`timescale 1ns/1ps

module fpga_fuzz_ctrl #(
    parameter integer CLK_HZ              = 100_000_000,
    parameter integer BAUD                = 115200,
    parameter integer PRETRIG_CYCLES      = 5_000_000,
    parameter integer POSTDONE_CYCLES     = 5_000_000,
    parameter integer GAP_CYCLES          = 2_000_000,
    parameter integer BOARD_TIMEOUT_CYCLES= 300_000_000,   // 3.0 s @ 100 MHz
    parameter [31:0] TIMEOUT_FAULT_CODE   = 32'h0000_0008  // bit3 = timeout
)(
    input  wire clk,
    input  wire rst_n,

    // Host UART3
    input  wire uart3_rx,
    output wire uart3_tx,

    // Board UARTs
    input  wire uart0_rx,
    output wire uart0_tx,

    input  wire uart1_rx,
    output wire uart1_tx,

    input  wire uart2_rx,
    output wire uart2_tx,

    // Trigger output (scope CH4)
    output reg  trig_out
);

    // ========================================================================
    // UARTs
    // ========================================================================
    wire [7:0] h_rx_data; wire h_rx_valid;
    reg  [7:0] h_tx_data; reg  h_tx_start; wire h_tx_busy;

    wire [7:0] b0_rx_data; wire b0_rx_valid;
    reg  [7:0] b0_tx_data; reg  b0_tx_start; wire b0_tx_busy;

    wire [7:0] b1_rx_data; wire b1_rx_valid;
    reg  [7:0] b1_tx_data; reg  b1_tx_start; wire b1_tx_busy;

    wire [7:0] b2_rx_data; wire b2_rx_valid;
    reg  [7:0] b2_tx_data; reg  b2_tx_start; wire b2_tx_busy;

    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) U_H_RX (
        .clk(clk), .rst_n(rst_n), .rx(uart3_rx), .data(h_rx_data), .valid(h_rx_valid)
    );
    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) U_H_TX (
        .clk(clk), .rst_n(rst_n), .tx(uart3_tx), .data(h_tx_data), .start(h_tx_start), .busy(h_tx_busy)
    );

    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) U_B0_RX (
        .clk(clk), .rst_n(rst_n), .rx(uart0_rx), .data(b0_rx_data), .valid(b0_rx_valid)
    );
    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) U_B0_TX (
        .clk(clk), .rst_n(rst_n), .tx(uart0_tx), .data(b0_tx_data), .start(b0_tx_start), .busy(b0_tx_busy)
    );

    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) U_B1_RX (
        .clk(clk), .rst_n(rst_n), .rx(uart1_rx), .data(b1_rx_data), .valid(b1_rx_valid)
    );
    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) U_B1_TX (
        .clk(clk), .rst_n(rst_n), .tx(uart1_tx), .data(b1_tx_data), .start(b1_tx_start), .busy(b1_tx_busy)
    );

    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) U_B2_RX (
        .clk(clk), .rst_n(rst_n), .rx(uart2_rx), .data(b2_rx_data), .valid(b2_rx_valid)
    );
    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) U_B2_TX (
        .clk(clk), .rst_n(rst_n), .tx(uart2_tx), .data(b2_tx_data), .start(b2_tx_start), .busy(b2_tx_busy)
    );

    // ========================================================================
    // Helpers
    // ========================================================================
    function automatic [3:0] hexval(input [7:0] c);
        begin
            if (c >= "0" && c <= "9")      hexval = c - "0";
            else if (c >= "A" && c <= "F") hexval = c - "A" + 4'd10;
            else if (c >= "a" && c <= "f") hexval = c - "a" + 4'd10;
            else                           hexval = 4'h0;
        end
    endfunction

    function automatic [7:0] nibble_to_ascii(input [3:0] n);
        begin
            if (n < 10) nibble_to_ascii = "0" + n;
            else        nibble_to_ascii = "A" + (n - 10);
        end
    endfunction

    function automatic [31:0] parse_hex8_from_arr;
        input [7:0] arr [0:127];
        input integer idx;
        integer i;
        reg [31:0] v;
        begin
            v = 0;
            for (i = 0; i < 8; i = i + 1)
                v = (v << 4) | hexval(arr[idx + i]);
            parse_hex8_from_arr = v;
        end
    endfunction

    function automatic [31:0] parse_dec_u32_from_arr;
        input [7:0] arr [0:127];
        input integer idx;
        input integer maxlen;
        integer i;
        reg [31:0] v;
        begin
            v = 0;
            for (i = idx; i < maxlen; i = i + 1) begin
                if (arr[i] >= "0" && arr[i] <= "9")
                    v = v * 10 + (arr[i] - "0");
                else
                    i = maxlen;
            end
            parse_dec_u32_from_arr = v;
        end
    endfunction

    function automatic [63:0] parse_dec_u64_from_arr;
        input [7:0] arr [0:127];
        input integer idx;
        input integer maxlen;
        integer i;
        reg [63:0] v;
        begin
            v = 0;
            for (i = idx; i < maxlen; i = i + 1) begin
                if (arr[i] >= "0" && arr[i] <= "9")
                    v = v * 10 + (arr[i] - "0");
                else
                    i = maxlen;
            end
            parse_dec_u64_from_arr = v;
        end
    endfunction

    function automatic integer find_token_start_after_space_arr;
        input [7:0] arr [0:127];
        input integer idx;
        input integer maxlen;
        integer i;
        reg seen_space;
        begin
            seen_space = 0;
            find_token_start_after_space_arr = maxlen;
            for (i = idx; i < maxlen; i = i + 1) begin
                if (!seen_space) begin
                    if (arr[i] == " ")
                        seen_space = 1;
                end else begin
                    if (arr[i] != " ") begin
                        find_token_start_after_space_arr = i;
                        i = maxlen;
                    end
                end
            end
        end
    endfunction

    // Search for KEY=... patterns in host line
    function automatic integer find_key_eq;
        input [7:0] arr [0:127];
        input integer maxlen;
        input [8*12-1:0] key;
        input integer keylen;
        integer i, j;
        reg match;
        begin
            find_key_eq = -1;
            for (i = 0; i <= maxlen - keylen; i = i + 1) begin
                match = 1'b1;
                for (j = 0; j < keylen; j = j + 1) begin
                    if (arr[i + j] != key[8*(keylen-1-j) +: 8])
                        match = 1'b0;
                end
                if (match) begin
                    find_key_eq = i + keylen;
                    i = maxlen;
                end
            end
        end
    endfunction

    // ========================================================================
    // Host line parser
    // ========================================================================
    reg [7:0] host_line [0:127];
    reg [6:0] host_len;

    reg        host_ping_req;
    reg        host_cfg_req;
    reg        host_run_req;
    reg        host_replay_req;

    // Current execution request
    reg [31:0] run_seed;
    reg [15:0] run_steps;
    reg [31:0] replay_start;
    reg [31:0] replay_count;
    reg        request_is_replay;

    // Stored configuration
    reg [31:0] cfg_seed_start;
    reg [31:0] cfg_seed_end;
    reg [15:0] cfg_steps_min;
    reg [15:0] cfg_steps_max;
    reg [15:0] cfg_policy_hint;

    integer hk;
    integer pos_seed, pos_steps, pos_ss, pos_se, pos_sm, pos_sx, pos_pol, pos_start, pos_count;
    always @(posedge clk) begin
        if (!rst_n) begin
            host_len         <= 0;
            host_ping_req    <= 0;
            host_cfg_req     <= 0;
            host_run_req     <= 0;
            host_replay_req  <= 0;

            run_seed         <= 0;
            run_steps        <= 0;
            replay_start     <= 0;
            replay_count     <= 0;
            request_is_replay<= 0;

            cfg_seed_start   <= 32'h0000_0001;
            cfg_seed_end     <= 32'h0001_0000;
            cfg_steps_min    <= 16'd64;
            cfg_steps_max    <= 16'd1024;
            cfg_policy_hint  <= 16'd0;
        end else begin
            host_ping_req   <= 0;
            host_cfg_req    <= 0;
            host_run_req    <= 0;
            host_replay_req <= 0;

            if (h_rx_valid) begin
                if (h_rx_data == 8'h0A || h_rx_data == 8'h0D) begin
                    // Parse completed host line
                    if (host_len >= 4 &&
                        host_line[0]=="P" && host_line[1]=="I" &&
                        host_line[2]=="N" && host_line[3]=="G") begin
                        host_ping_req <= 1'b1;
                    end

                    else if (host_len >= 3 &&
                             host_line[0]=="C" && host_line[1]=="F" && host_line[2]=="G") begin
                        pos_ss  = find_key_eq(host_line, host_len, "SEED_START=", 11);
                        pos_se  = find_key_eq(host_line, host_len, "SEED_END=",   9);
                        pos_sm  = find_key_eq(host_line, host_len, "STEPS_MIN=",  10);
                        pos_sx  = find_key_eq(host_line, host_len, "STEPS_MAX=",  10);
                        pos_pol = find_key_eq(host_line, host_len, "POLICY=",      7);

                        if (pos_ss >= 0) cfg_seed_start  <= parse_hex8_from_arr(host_line, pos_ss);
                        if (pos_se >= 0) cfg_seed_end    <= parse_hex8_from_arr(host_line, pos_se);
                        if (pos_sm >= 0) cfg_steps_min   <= parse_dec_u32_from_arr(host_line, pos_sm, host_len)[15:0];
                        if (pos_sx >= 0) cfg_steps_max   <= parse_dec_u32_from_arr(host_line, pos_sx, host_len)[15:0];
                        if (pos_pol>= 0) cfg_policy_hint <= parse_dec_u32_from_arr(host_line, pos_pol, host_len)[15:0];

                        host_cfg_req <= 1'b1;
                    end

                    else if (host_len >= 6 &&
                             host_line[0]=="R" && host_line[1]=="E" &&
                             host_line[2]=="P" && host_line[3]=="L" &&
                             host_line[4]=="A" && host_line[5]=="Y") begin
                        pos_seed  = find_key_eq(host_line, host_len, "SEED=", 5);
                        pos_steps = find_key_eq(host_line, host_len, "STEPS=", 6);
                        pos_start = find_key_eq(host_line, host_len, "START=", 6);
                        pos_count = find_key_eq(host_line, host_len, "COUNT=", 6);

                        if (pos_seed  >= 0) run_seed     <= parse_hex8_from_arr(host_line, pos_seed);
                        if (pos_steps >= 0) run_steps    <= parse_dec_u32_from_arr(host_line, pos_steps, host_len)[15:0];
                        if (pos_start >= 0) replay_start <= parse_dec_u32_from_arr(host_line, pos_start, host_len);
                        if (pos_count >= 0) replay_count <= parse_dec_u32_from_arr(host_line, pos_count, host_len);

                        request_is_replay <= 1'b1;
                        host_replay_req   <= 1'b1;
                    end

                    else if (host_len >= 3 &&
                             host_line[0]=="R" && host_line[1]=="U" && host_line[2]=="N") begin
                        pos_seed  = find_key_eq(host_line, host_len, "SEED=", 5);
                        pos_steps = find_key_eq(host_line, host_len, "STEPS=", 6);

                        if (pos_seed  >= 0) run_seed  <= parse_hex8_from_arr(host_line, pos_seed);
                        if (pos_steps >= 0) run_steps <= parse_dec_u32_from_arr(host_line, pos_steps, host_len)[15:0];

                        request_is_replay <= 1'b0;
                        host_run_req      <= 1'b1;
                    end

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

    // ========================================================================
    // Board line parser
    // ========================================================================
    reg [7:0] b_line [0:127];
    reg [6:0] b_len;
    reg [1:0] cur_board;

    reg        done_got          [0:2];
    reg [31:0] done_seed         [0:2];
    reg [31:0] done_flags        [0:2];
    reg [63:0] done_score        [0:2];
    reg [31:0] done_worst_window [0:2];
    reg [63:0] done_worst_cycles [0:2];
    reg        done_timeout      [0:2];
    reg [63:0] done_duration     [0:2];  // board run duration observed by FPGA
    reg [31:0] done_meta0        [0:2];  // spare metadata
    reg [31:0] done_meta1        [0:2];  // spare metadata

    wire [7:0] bx_data  = (cur_board == 2'd0) ? b0_rx_data  :
                          (cur_board == 2'd1) ? b1_rx_data  :
                                                 b2_rx_data;
    wire       bx_valid = (cur_board == 2'd0) ? b0_rx_valid :
                          (cur_board == 2'd1) ? b1_rx_valid :
                                                 b2_rx_valid;

    integer bi;
    integer score_idx, flags_idx, win_idx, wcyc_idx;
    always @(posedge clk) begin
        if (!rst_n) begin
            b_len <= 0;
            for (bi = 0; bi < 3; bi = bi + 1) begin
                done_got[bi]          <= 0;
                done_seed[bi]         <= 0;
                done_flags[bi]        <= 0;
                done_score[bi]        <= 0;
                done_worst_window[bi] <= 0;
                done_worst_cycles[bi] <= 0;
                done_timeout[bi]      <= 0;
                done_duration[bi]     <= 0;
                done_meta0[bi]        <= 0;
                done_meta1[bi]        <= 0;
            end
        end else begin
            if (bx_valid) begin
                if (bx_data == 8'h0A || bx_data == 8'h0D) begin
                    if (b_len >= 20 &&
                        b_line[0]=="D" && b_line[1]=="O" &&
                        b_line[2]=="N" && b_line[3]=="E" && b_line[4]==" ") begin

                        score_idx = 14;
                        flags_idx = find_token_start_after_space_arr(b_line, score_idx, b_len);
                        win_idx   = find_token_start_after_space_arr(b_line, flags_idx, b_len);
                        wcyc_idx  = find_token_start_after_space_arr(b_line, win_idx,   b_len);

                        done_seed[cur_board]         <= parse_hex8_from_arr(b_line, 5);
                        done_score[cur_board]        <= parse_dec_u64_from_arr(b_line, score_idx, b_len);
                        done_flags[cur_board]        <= (flags_idx + 8 <= b_len) ? parse_hex8_from_arr(b_line, flags_idx) : 32'hDEAD_BEEF;
                        done_worst_window[cur_board] <= parse_dec_u32_from_arr(b_line, win_idx,  b_len);
                        done_worst_cycles[cur_board] <= parse_dec_u64_from_arr(b_line, wcyc_idx, b_len);
                        done_timeout[cur_board]      <= 1'b0;
                        done_got[cur_board]          <= 1'b1;
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

    // ========================================================================
    // Shared TX buffer / formatter
    // ========================================================================
    reg [7:0] txbuf [0:511];
    reg [8:0] txlen;
    reg [8:0] txidx;
    reg       tx_active;
    reg [1:0] tx_target; // 0=b0,1=b1,2=b2,3=host

    task automatic append_ascii_char(input [7:0] c);
        begin
            txbuf[txlen] = c;
            txlen = txlen + 1'b1;
        end
    endtask

    task automatic append_hex8_to_txbuf(input [31:0] val);
        integer q;
        reg [3:0] nib;
        begin
            for (q = 0; q < 8; q = q + 1) begin
                nib = (val >> (28 - 4*q)) & 4'hF;
                txbuf[txlen] = nibble_to_ascii(nib);
                txlen = txlen + 1'b1;
            end
        end
    endtask

    task automatic append_u32_dec_to_txbuf(input [31:0] val);
        integer u;
        reg [7:0] digits [0:15];
        integer dlen;
        reg [31:0] x;
        begin
            dlen = 0;
            x = val;
            if (x == 0) begin
                digits[dlen] = "0";
                dlen = dlen + 1;
            end else begin
                while (x > 0 && dlen < 10) begin
                    digits[dlen] = "0" + (x % 10);
                    x = x / 10;
                    dlen = dlen + 1;
                end
            end
            for (u = dlen - 1; u >= 0; u = u - 1) begin
                txbuf[txlen] = digits[u];
                txlen = txlen + 1'b1;
            end
        end
    endtask

    task automatic append_u64_dec_to_txbuf(input [63:0] val);
        integer u;
        reg [7:0] digits [0:31];
        integer dlen;
        reg [63:0] x;
        begin
            dlen = 0;
            x = val;
            if (x == 0) begin
                digits[dlen] = "0";
                dlen = dlen + 1;
            end else begin
                while (x > 0 && dlen < 20) begin
                    digits[dlen] = "0" + (x % 10);
                    x = x / 10;
                    dlen = dlen + 1;
                end
            end
            for (u = dlen - 1; u >= 0; u = u - 1) begin
                txbuf[txlen] = digits[u];
                txlen = txlen + 1'b1;
            end
        end
    endtask

    task automatic tx_load_literal_pong;
        begin
            txlen = 0;
            append_ascii_char("P");
            append_ascii_char("O");
            append_ascii_char("N");
            append_ascii_char("G");
            append_ascii_char(8'h0A);
        end
    endtask

    task automatic tx_load_literal_ack_cfg;
        begin
            txlen = 0;
            append_ascii_char("A"); append_ascii_char("C"); append_ascii_char("K");
            append_ascii_char(" ");
            append_ascii_char("C"); append_ascii_char("F"); append_ascii_char("G");
            append_ascii_char(8'h0A);
        end
    endtask

    task automatic tx_load_literal_ack_run;
        begin
            txlen = 0;
            append_ascii_char("A"); append_ascii_char("C"); append_ascii_char("K");
            append_ascii_char(" ");
            append_ascii_char("R"); append_ascii_char("U"); append_ascii_char("N");
            append_ascii_char(8'h0A);
        end
    endtask

    task automatic tx_load_literal_ack_replay;
        begin
            txlen = 0;
            append_ascii_char("A"); append_ascii_char("C"); append_ascii_char("K");
            append_ascii_char(" ");
            append_ascii_char("R"); append_ascii_char("E"); append_ascii_char("P");
            append_ascii_char("L"); append_ascii_char("A"); append_ascii_char("Y");
            append_ascii_char(8'h0A);
        end
    endtask

    task automatic tx_load_run(input [31:0] seed, input [15:0] steps);
        begin
            txlen = 0;
            append_ascii_char("R"); append_ascii_char("U"); append_ascii_char("N");
            append_ascii_char(" ");
            append_hex8_to_txbuf(seed);
            append_ascii_char(" ");
            append_u32_dec_to_txbuf({16'h0, steps});
            append_ascii_char(8'h0A);
        end
    endtask

    task automatic tx_load_replay(input [31:0] seed, input [15:0] steps, input [31:0] start, input [31:0] count);
        begin
            txlen = 0;
            append_ascii_char("R"); append_ascii_char("E"); append_ascii_char("P");
            append_ascii_char("L"); append_ascii_char("A"); append_ascii_char("Y");
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

    task automatic append_triplet_board(input integer idx);
        begin
            append_ascii_char(" ");
            append_ascii_char("S"); append_ascii_char("0"+idx[7:0]); append_ascii_char("=");
            append_u64_dec_to_txbuf(done_score[idx]);

            append_ascii_char(" ");
            append_ascii_char("F"); append_ascii_char("0"+idx[7:0]); append_ascii_char("=");
            append_hex8_to_txbuf(done_flags[idx]);

            append_ascii_char(" ");
            append_ascii_char("W"); append_ascii_char("0"+idx[7:0]); append_ascii_char("=");
            append_u32_dec_to_txbuf(done_worst_window[idx]);

            append_ascii_char(" ");
            append_ascii_char("W"); append_ascii_char("C"); append_ascii_char("0"+idx[7:0]); append_ascii_char("=");
            append_u64_dec_to_txbuf(done_worst_cycles[idx]);

            append_ascii_char(" ");
            append_ascii_char("T"); append_ascii_char("0"+idx[7:0]); append_ascii_char("=");
            append_u32_dec_to_txbuf({31'd0, done_timeout[idx]});

            append_ascii_char(" ");
            append_ascii_char("D"); append_ascii_char("0"+idx[7:0]); append_ascii_char("=");
            append_u64_dec_to_txbuf(done_duration[idx]);
        end
    endtask

    task automatic tx_load_triple(input [31:0] seed, input [15:0] steps, input [31:0] rs, input [31:0] rc);
        begin
            txlen = 0;

            append_ascii_char("T"); append_ascii_char("R"); append_ascii_char("I");
            append_ascii_char("P"); append_ascii_char("L"); append_ascii_char("E");
            append_ascii_char(" ");

            append_ascii_char("S"); append_ascii_char("E"); append_ascii_char("E");
            append_ascii_char("D"); append_ascii_char("=");
            append_hex8_to_txbuf(seed);

            append_ascii_char(" ");
            append_ascii_char("S"); append_ascii_char("T"); append_ascii_char("E");
            append_ascii_char("P"); append_ascii_char("S"); append_ascii_char("=");
            append_u32_dec_to_txbuf({16'h0, steps});

            append_triplet_board(0);
            append_triplet_board(1);
            append_triplet_board(2);

            append_ascii_char(" ");
            append_ascii_char("R"); append_ascii_char("S"); append_ascii_char("=");
            append_u32_dec_to_txbuf(rs);

            append_ascii_char(" ");
            append_ascii_char("R"); append_ascii_char("C"); append_ascii_char("=");
            append_u32_dec_to_txbuf(rc);

            append_ascii_char(8'h0A);
        end
    endtask

    task automatic tx_start_msg(input [1:0] target);
        begin
            tx_target <= target;
            txidx     <= 0;
            tx_active <= 1'b1;
        end
    endtask

    always @(posedge clk) begin
        if (!rst_n) begin
            h_tx_start  <= 0;
            b0_tx_start <= 0;
            b1_tx_start <= 0;
            b2_tx_start <= 0;
            tx_active   <= 0;
            txidx       <= 0;
        end else begin
            h_tx_start  <= 0;
            b0_tx_start <= 0;
            b1_tx_start <= 0;
            b2_tx_start <= 0;

            if (tx_active) begin
                if (txidx < txlen) begin
                    if (tx_target == 2'd3 && !h_tx_busy) begin
                        h_tx_data  <= txbuf[txidx];
                        h_tx_start <= 1'b1;
                        txidx      <= txidx + 1'b1;
                    end else if (tx_target == 2'd0 && !b0_tx_busy) begin
                        b0_tx_data  <= txbuf[txidx];
                        b0_tx_start <= 1'b1;
                        txidx       <= txidx + 1'b1;
                    end else if (tx_target == 2'd1 && !b1_tx_busy) begin
                        b1_tx_data  <= txbuf[txidx];
                        b1_tx_start <= 1'b1;
                        txidx       <= txidx + 1'b1;
                    end else if (tx_target == 2'd2 && !b2_tx_busy) begin
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

    // ========================================================================
    // Main FSM
    // ========================================================================
    localparam [3:0]
        ST_IDLE        = 4'd0,
        ST_HOST_MSG    = 4'd1,
        ST_PRETRIG     = 4'd2,
        ST_SEND_CMD    = 4'd3,
        ST_WAIT_DONE   = 4'd4,
        ST_TIMEOUT     = 4'd5,
        ST_POSTDONE    = 4'd6,
        ST_GAP         = 4'd7,
        ST_SEND_TRIP   = 4'd8;

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
    always @(posedge clk) begin
        if (!rst_n) begin
            st <= ST_IDLE;
            trig_out <= 0;
            ctr <= 0;
            board_wait_ctr <= 0;
            run_duration_ctr <= 0;
            cur_board <= 0;

            seed_latched <= 0;
            steps_latched <= 0;
            replay_start_latched <= 0;
            replay_count_latched <= 0;
            is_replay_latched <= 0;

            for (ci = 0; ci < 3; ci = ci + 1) begin
                done_got[ci]      <= 0;
                done_timeout[ci]  <= 0;
                done_duration[ci] <= 0;
            end
        end else begin
            case (st)
                ST_IDLE: begin
                    trig_out <= 1'b0;

                    if ((host_ping_req || host_cfg_req || host_run_req || host_replay_req) && !tx_active) begin
                        st <= ST_HOST_MSG;
                    end
                end

                ST_HOST_MSG: begin
                    if (!tx_active) begin
                        if (host_ping_req) begin
                            tx_load_literal_pong();
                            tx_start_msg(2'd3);
                            st <= ST_IDLE;
                        end
                        else if (host_cfg_req) begin
                            tx_load_literal_ack_cfg();
                            tx_start_msg(2'd3);
                            st <= ST_IDLE;
                        end
                        else if (host_run_req) begin
                            seed_latched         <= run_seed;
                            steps_latched        <= run_steps;
                            replay_start_latched <= 0;
                            replay_count_latched <= 0;
                            is_replay_latched    <= 0;

                            done_got[0] <= 0; done_got[1] <= 0; done_got[2] <= 0;
                            done_timeout[0] <= 0; done_timeout[1] <= 0; done_timeout[2] <= 0;

                            tx_load_literal_ack_run();
                            tx_start_msg(2'd3);

                            cur_board <= 0;
                            ctr <= 0;
                            trig_out <= 1'b1;
                            st <= ST_PRETRIG;
                        end
                        else if (host_replay_req) begin
                            seed_latched         <= run_seed;
                            steps_latched        <= run_steps;
                            replay_start_latched <= replay_start;
                            replay_count_latched <= replay_count;
                            is_replay_latched    <= 1'b1;

                            done_got[0] <= 0; done_got[1] <= 0; done_got[2] <= 0;
                            done_timeout[0] <= 0; done_timeout[1] <= 0; done_timeout[2] <= 0;

                            tx_load_literal_ack_replay();
                            tx_start_msg(2'd3);

                            cur_board <= 0;
                            ctr <= 0;
                            trig_out <= 1'b1;
                            st <= ST_PRETRIG;
                        end
                        else begin
                            st <= ST_IDLE;
                        end
                    end
                end

                ST_PRETRIG: begin
                    if (ctr < PRETRIG_CYCLES) begin
                        ctr <= ctr + 1'b1;
                    end else begin
                        ctr <= 0;
                        st <= ST_SEND_CMD;
                    end
                end

                ST_SEND_CMD: begin
                    if (!tx_active) begin
                        if (is_replay_latched)
                            tx_load_replay(seed_latched, steps_latched, replay_start_latched, replay_count_latched);
                        else
                            tx_load_run(seed_latched, steps_latched);

                        tx_start_msg(cur_board);
                        board_wait_ctr   <= 0;
                        run_duration_ctr <= 0;
                        st <= ST_WAIT_DONE;
                    end
                end

                ST_WAIT_DONE: begin
                    board_wait_ctr   <= board_wait_ctr + 1'b1;
                    run_duration_ctr <= run_duration_ctr + 1'b1;

                    if (done_got[cur_board]) begin
                        done_duration[cur_board] <= run_duration_ctr;
                        ctr <= 0;
                        st <= ST_POSTDONE;
                    end else if (board_wait_ctr >= BOARD_TIMEOUT_CYCLES) begin
                        st <= ST_TIMEOUT;
                    end
                end

                ST_TIMEOUT: begin
                    done_seed[cur_board]         <= seed_latched;
                    done_score[cur_board]        <= 64'd0;
                    done_flags[cur_board]        <= TIMEOUT_FAULT_CODE;
                    done_worst_window[cur_board] <= 32'd0;
                    done_worst_cycles[cur_board] <= 64'd0;
                    done_timeout[cur_board]      <= 1'b1;
                    done_duration[cur_board]     <= run_duration_ctr;
                    done_got[cur_board]          <= 1'b1;

                    ctr <= 0;
                    st <= ST_POSTDONE;
                end

                ST_POSTDONE: begin
                    if (ctr < POSTDONE_CYCLES) begin
                        ctr <= ctr + 1'b1;
                    end else begin
                        trig_out <= 1'b0;
                        ctr <= 0;
                        st <= ST_GAP;
                    end
                end

                ST_GAP: begin
                    if (ctr < GAP_CYCLES) begin
                        ctr <= ctr + 1'b1;
                    end else begin
                        ctr <= 0;
                        if (cur_board < 2) begin
                            cur_board <= cur_board + 1'b1;
                            trig_out <= 1'b1;
                            st <= ST_PRETRIG;
                        end else begin
                            st <= ST_SEND_TRIP;
                        end
                    end
                end

                ST_SEND_TRIP: begin
                    if (!tx_active) begin
                        tx_load_triple(seed_latched, steps_latched, replay_start_latched, replay_count_latched);
                        tx_start_msg(2'd3);
                        st <= ST_IDLE;
                    end
                end

                default: st <= ST_IDLE;
            endcase
        end
    end

endmodule

// ============================================================================
// UART RX: 8N1 receiver
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
            valid <= 0;
            rx_d  <= rx;

            if (!busy) begin
                if (rx_d == 1'b1 && rx == 1'b0) begin
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
                    frame  <= {1'b1, data, 1'b0}; // stop, data, start
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