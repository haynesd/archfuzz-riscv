`timescale 1ns / 1ps

module fpga_fuzz_ctrl #(
    parameter integer CLK_HZ           = 100_000_000,
    parameter integer BAUD             = 115200,
    parameter integer TIMEOUT_CYCLES   = 300_000_000,
    parameter integer LED_HOLD_CYCLES  = 5_000_000   // ~50 ms (visible blink)
)(
    input  wire clk,
    input  wire rst_n,

    input  wire uart_host_rx,
    output wire uart_host_tx,

    input  wire uart0_rx,
    output wire uart0_tx,

    input  wire uart1_rx,
    output wire uart1_tx,

    input  wire uart2_rx,
    output wire uart2_tx,

    output reg  trig_out,
    output wire [3:0] led
);

    // ============================================================
    // Derived widths
    // ============================================================
    localparam integer TIMEOUT_W =
        (TIMEOUT_CYCLES <= 2) ? 1 : $clog2(TIMEOUT_CYCLES);

    localparam integer LED_W =
        (LED_HOLD_CYCLES <= 2) ? 1 : $clog2(LED_HOLD_CYCLES);

    // ============================================================
    // UARTs
    // ============================================================
    wire [7:0] h_rx_data;
    wire       h_rx_valid;

    reg  [7:0] h_tx_data;
    reg        h_tx_start;
    wire       h_tx_busy;

    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) RXH (
        .clk(clk), .rst_n(rst_n),
        .rx(uart_host_rx),
        .data(h_rx_data),
        .valid(h_rx_valid)
    );

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) TXH (
        .clk(clk), .rst_n(rst_n),
        .tx(uart_host_tx),
        .data(h_tx_data),
        .start(h_tx_start),
        .busy(h_tx_busy)
    );

    wire [7:0] b0_rx_data, b1_rx_data, b2_rx_data;
    wire       b0_rx_valid, b1_rx_valid, b2_rx_valid;

    reg  [7:0] b0_tx_data, b1_tx_data, b2_tx_data;
    reg        b0_tx_start, b1_tx_start, b2_tx_start;
    wire       b0_tx_busy,  b1_tx_busy,  b2_tx_busy;

    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) RX0 (
        .clk(clk), .rst_n(rst_n),
        .rx(uart0_rx),
        .data(b0_rx_data),
        .valid(b0_rx_valid)
    );

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) TX0 (
        .clk(clk), .rst_n(rst_n),
        .tx(uart0_tx),
        .data(b0_tx_data),
        .start(b0_tx_start),
        .busy(b0_tx_busy)
    );

    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) RX1 (
        .clk(clk), .rst_n(rst_n),
        .rx(uart1_rx),
        .data(b1_rx_data),
        .valid(b1_rx_valid)
    );

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) TX1 (
        .clk(clk), .rst_n(rst_n),
        .tx(uart1_tx),
        .data(b1_tx_data),
        .start(b1_tx_start),
        .busy(b1_tx_busy)
    );

    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) RX2 (
        .clk(clk), .rst_n(rst_n),
        .rx(uart2_rx),
        .data(b2_rx_data),
        .valid(b2_rx_valid)
    );

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) TX2 (
        .clk(clk), .rst_n(rst_n),
        .tx(uart2_tx),
        .data(b2_tx_data),
        .start(b2_tx_start),
        .busy(b2_tx_busy)
    );

    // ============================================================
    // Board select
    // ============================================================
    reg [1:0] active_board;

    wire [7:0] sel_rx_data =
        (active_board == 0) ? b0_rx_data :
        (active_board == 1) ? b1_rx_data : b2_rx_data;

    wire sel_rx_valid =
        (active_board == 0) ? b0_rx_valid :
        (active_board == 1) ? b1_rx_valid : b2_rx_valid;

    wire sel_tx_busy =
        (active_board == 0) ? b0_tx_busy :
        (active_board == 1) ? b1_tx_busy : b2_tx_busy;

    // ============================================================
    // FSM
    // ============================================================
    localparam ST_IDLE       = 0;
    localparam ST_GET_BRD    = 1;
    localparam ST_SEND       = 2;
    localparam ST_WAIT_RESP  = 3;
    localparam ST_TIMEOUT_T  = 4;
    localparam ST_TIMEOUT_NL = 5;

    reg [2:0] state;
    reg [TIMEOUT_W-1:0] timeout_cnt;

    // ============================================================
    // LED logic
    // ============================================================
    reg [LED_W-1:0] led1_cnt, led2_cnt;

    assign led[0] = heartbeat;
    assign led[1] = (led1_cnt != 0);
    assign led[2] = (led2_cnt != 0);
    assign led[3] = trig_out;

    // ============================================================
    // Heartbeat (alive indicator)
    // ============================================================
    reg [25:0] hb_cnt;
    wire heartbeat = hb_cnt[25];

    always @(posedge clk) begin
        if (!rst_n)
            hb_cnt <= 0;
        else
            hb_cnt <= hb_cnt + 1;
    end

    // ============================================================
    // Main FSM
    // ============================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            trig_out <= 0;
            active_board <= 0;

            h_tx_start <= 0;
            timeout_cnt <= 0;

            b0_tx_start <= 0;
            b1_tx_start <= 0;
            b2_tx_start <= 0;
        end else begin
            h_tx_start <= 0;
            b0_tx_start <= 0;
            b1_tx_start <= 0;
            b2_tx_start <= 0;

            case (state)

                ST_IDLE: begin
                    trig_out <= 0;
                    timeout_cnt <= 0;

                    if (h_rx_valid && h_rx_data == 8'h01)
                        state <= ST_GET_BRD;
                end

                ST_GET_BRD: begin
                    if (h_rx_valid) begin
                        if (h_rx_data < 3) begin
                            active_board <= h_rx_data;
                            trig_out <= 1;
                            state <= ST_SEND;
                        end else
                            state <= ST_IDLE;
                    end
                end

                ST_SEND: begin
                    if (h_rx_valid && !sel_tx_busy) begin
                        case (active_board)
                            0: begin b0_tx_data <= h_rx_data; b0_tx_start <= 1; end
                            1: begin b1_tx_data <= h_rx_data; b1_tx_start <= 1; end
                            default: begin b2_tx_data <= h_rx_data; b2_tx_start <= 1; end
                        endcase

                        if (h_rx_data == 8'h0A)
                            state <= ST_WAIT_RESP;
                    end
                end

                ST_WAIT_RESP: begin
                    timeout_cnt <= timeout_cnt + 1;

                    if (sel_rx_valid && !h_tx_busy) begin
                        h_tx_data  <= sel_rx_data;
                        h_tx_start <= 1;

                        // LED: board RX blink
                        led2_cnt <= LED_HOLD_CYCLES;

                        if (sel_rx_data == 8'h0A) begin
                            trig_out <= 0;
                            state <= ST_IDLE;
                        end
                    end
                    else if (timeout_cnt == TIMEOUT_CYCLES - 1)
                        state <= ST_TIMEOUT_T;
                end

                ST_TIMEOUT_T: begin
                    if (!h_tx_busy) begin
                        h_tx_data <= "T";
                        h_tx_start <= 1;
                        state <= ST_TIMEOUT_NL;
                    end
                end

                ST_TIMEOUT_NL: begin
                    if (!h_tx_busy) begin
                        h_tx_data <= 8'h0A;
                        h_tx_start <= 1;
                        trig_out <= 0;
                        state <= ST_IDLE;
                    end
                end
            endcase

            // LED: host RX blink
            if (h_rx_valid)
                led1_cnt <= LED_HOLD_CYCLES;
            else if (led1_cnt != 0)
                led1_cnt <= led1_cnt - 1;

            if (led2_cnt != 0)
                led2_cnt <= led2_cnt - 1;
        end
    end

endmodule