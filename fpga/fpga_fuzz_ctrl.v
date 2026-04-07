`timescale 1ns / 1ps

module fpga_fuzz_ctrl #(
    parameter integer CLK_HZ           = 100_000_000,
    parameter integer BAUD             = 115200,
    parameter integer TIMEOUT_CYCLES   = 300_000_000,
    parameter integer LED_HOLD_CYCLES  = 5_000_000
)(
    input  wire       clk,
    input  wire       rst_n,

    input  wire       uart_host_rx,
    output wire       uart_host_tx,

    input  wire       uart0_rx,
    output wire       uart0_tx,

    input  wire       uart1_rx,
    output wire       uart1_tx,

    input  wire       uart2_rx,
    output wire       uart2_tx,

    output reg        trig_out,
    output wire [3:0] led
);

    // =========================================================================
    // Width calculations
    // =========================================================================
    localparam integer TIMEOUT_W =
        (TIMEOUT_CYCLES <= 2) ? 1 : $clog2(TIMEOUT_CYCLES);

    localparam integer LED_W =
        (LED_HOLD_CYCLES <= 2) ? 1 : $clog2(LED_HOLD_CYCLES);

    // =========================================================================
    // Host UART
    // =========================================================================
    wire [7:0] h_rx_data;
    wire       h_rx_valid;

    reg  [7:0] h_tx_data;
    reg        h_tx_start;
    wire       h_tx_busy;

    uart_rx #(
        .CLK_HZ(CLK_HZ),
        .BAUD(BAUD)
    ) RXH (
        .clk   (clk),
        .rst_n (rst_n),
        .rx    (uart_host_rx),
        .data  (h_rx_data),
        .valid (h_rx_valid)
    );

    uart_tx #(
        .CLK_HZ(CLK_HZ),
        .BAUD(BAUD)
    ) TXH (
        .clk   (clk),
        .rst_n (rst_n),
        .tx    (uart_host_tx),
        .data  (h_tx_data),
        .start (h_tx_start),
        .busy  (h_tx_busy)
    );

    // =========================================================================
    // Board UARTs
    // =========================================================================
    wire [7:0] b0_rx_data, b1_rx_data, b2_rx_data;
    wire       b0_rx_valid, b1_rx_valid, b2_rx_valid;

    reg  [7:0] b0_tx_data, b1_tx_data, b2_tx_data;
    reg        b0_tx_start, b1_tx_start, b2_tx_start;
    wire       b0_tx_busy,  b1_tx_busy,  b2_tx_busy;

    uart_rx #(
        .CLK_HZ(CLK_HZ),
        .BAUD(BAUD)
    ) RX0 (
        .clk   (clk),
        .rst_n (rst_n),
        .rx    (uart0_rx),
        .data  (b0_rx_data),
        .valid (b0_rx_valid)
    );

    uart_tx #(
        .CLK_HZ(CLK_HZ),
        .BAUD(BAUD)
    ) TX0 (
        .clk   (clk),
        .rst_n (rst_n),
        .tx    (uart0_tx),
        .data  (b0_tx_data),
        .start (b0_tx_start),
        .busy  (b0_tx_busy)
    );

    uart_rx #(
        .CLK_HZ(CLK_HZ),
        .BAUD(BAUD)
    ) RX1 (
        .clk   (clk),
        .rst_n (rst_n),
        .rx    (uart1_rx),
        .data  (b1_rx_data),
        .valid (b1_rx_valid)
    );

    uart_tx #(
        .CLK_HZ(CLK_HZ),
        .BAUD(BAUD)
    ) TX1 (
        .clk   (clk),
        .rst_n (rst_n),
        .tx    (uart1_tx),
        .data  (b1_tx_data),
        .start (b1_tx_start),
        .busy  (b1_tx_busy)
    );

    uart_rx #(
        .CLK_HZ(CLK_HZ),
        .BAUD(BAUD)
    ) RX2 (
        .clk   (clk),
        .rst_n (rst_n),
        .rx    (uart2_rx),
        .data  (b2_rx_data),
        .valid (b2_rx_valid)
    );

    uart_tx #(
        .CLK_HZ(CLK_HZ),
        .BAUD(BAUD)
    ) TX2 (
        .clk   (clk),
        .rst_n (rst_n),
        .tx    (uart2_tx),
        .data  (b2_tx_data),
        .start (b2_tx_start),
        .busy  (b2_tx_busy)
    );

    // =========================================================================
    // Selected board muxes
    // =========================================================================
    reg [1:0] active_board;

    wire [7:0] sel_rx_data =
        (active_board == 2'd0) ? b0_rx_data :
        (active_board == 2'd1) ? b1_rx_data :
                                 b2_rx_data;

    wire sel_rx_valid =
        (active_board == 2'd0) ? b0_rx_valid :
        (active_board == 2'd1) ? b1_rx_valid :
                                 b2_rx_valid;

    wire sel_tx_busy =
        (active_board == 2'd0) ? b0_tx_busy :
        (active_board == 2'd1) ? b1_tx_busy :
                                 b2_tx_busy;

    // =========================================================================
    // One-byte buffer so board RX is not lost while host TX is busy
    // =========================================================================
    reg       board_rx_pending;
    reg [7:0] board_rx_byte;

    // =========================================================================
    // FSM states
    // =========================================================================
    localparam [2:0]
        ST_IDLE       = 3'd0,
        ST_GET_BRD    = 3'd1,
        ST_SEND       = 3'd2,
        ST_WAIT_RESP  = 3'd3,
        ST_TIMEOUT_T  = 3'd4,
        ST_TIMEOUT_NL = 3'd5;

    reg [2:0] state;

    // =========================================================================
    // Timeout counter
    // =========================================================================
    reg [TIMEOUT_W-1:0] timeout_cnt;

    // =========================================================================
    // Heartbeat + LED pulse stretch
    // =========================================================================
    reg [25:0] hb_cnt;
    reg [LED_W-1:0] led1_cnt;
    reg [LED_W-1:0] led2_cnt;
    reg [LED_W-1:0] led3_cnt;

    wire heartbeat = hb_cnt[25];

    assign led[0] = heartbeat;                       // heartbeat
    assign led[1] = (led1_cnt != {LED_W{1'b0}});    // receive from TTL
    assign led[2] = (led2_cnt != {LED_W{1'b0}});    // send to board
    assign led[3] = (led3_cnt != {LED_W{1'b0}});    // receive from board

    // Heartbeat counter
    always @(posedge clk) begin
        if (!rst_n)
            hb_cnt <= 26'd0;
        else
            hb_cnt <= hb_cnt + 1'b1;
    end

    // LED pulse stretchers
    always @(posedge clk) begin
        if (!rst_n) begin
            led1_cnt <= {LED_W{1'b0}};
            led2_cnt <= {LED_W{1'b0}};
            led3_cnt <= {LED_W{1'b0}};
        end else begin
            if (h_rx_valid)
                led1_cnt <= LED_HOLD_CYCLES[LED_W-1:0];
            else if (led1_cnt != {LED_W{1'b0}})
                led1_cnt <= led1_cnt - 1'b1;

            if (b0_tx_start || b1_tx_start || b2_tx_start)
                led2_cnt <= LED_HOLD_CYCLES[LED_W-1:0];
            else if (led2_cnt != {LED_W{1'b0}})
                led2_cnt <= led2_cnt - 1'b1;

            if (sel_rx_valid)
                led3_cnt <= LED_HOLD_CYCLES[LED_W-1:0];
            else if (led3_cnt != {LED_W{1'b0}})
                led3_cnt <= led3_cnt - 1'b1;
        end
    end

    // =========================================================================
    // Main control FSM
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            state           <= ST_IDLE;
            active_board    <= 2'd0;
            trig_out        <= 1'b0;

            h_tx_data       <= 8'h00;
            h_tx_start      <= 1'b0;

            b0_tx_data      <= 8'h00;
            b1_tx_data      <= 8'h00;
            b2_tx_data      <= 8'h00;

            b0_tx_start     <= 1'b0;
            b1_tx_start     <= 1'b0;
            b2_tx_start     <= 1'b0;

            timeout_cnt     <= {TIMEOUT_W{1'b0}};
            board_rx_pending<= 1'b0;
            board_rx_byte   <= 8'h00;
        end else begin
            h_tx_start  <= 1'b0;
            b0_tx_start <= 1'b0;
            b1_tx_start <= 1'b0;
            b2_tx_start <= 1'b0;

            // Capture selected board RX byte into one-byte buffer
            if (sel_rx_valid && !board_rx_pending) begin
                board_rx_pending <= 1'b1;
                board_rx_byte    <= sel_rx_data;
            end

            case (state)

                // -------------------------------------------------------------
                // Wait for 0x01 packet start
                // -------------------------------------------------------------
                ST_IDLE: begin
                    trig_out         <= 1'b0;
                    timeout_cnt      <= {TIMEOUT_W{1'b0}};
                    board_rx_pending <= 1'b0;

                    if (h_rx_valid && h_rx_data == 8'h01)
                        state <= ST_GET_BRD;
                end

                // -------------------------------------------------------------
                // Read board ID
                // -------------------------------------------------------------
                ST_GET_BRD: begin
                    if (h_rx_valid) begin
                        if (h_rx_data < 8'd3) begin
                            active_board <= h_rx_data[1:0];
                            trig_out     <= 1'b1;
                            state        <= ST_SEND;
                        end else begin
                            state <= ST_IDLE;
                        end
                    end
                end

                // -------------------------------------------------------------
                // Forward payload to selected board until newline
                // -------------------------------------------------------------
                ST_SEND: begin
                    if (h_rx_valid && !sel_tx_busy) begin
                        case (active_board)
                            2'd0: begin
                                b0_tx_data  <= h_rx_data;
                                b0_tx_start <= 1'b1;
                            end

                            2'd1: begin
                                b1_tx_data  <= h_rx_data;
                                b1_tx_start <= 1'b1;
                            end

                            default: begin
                                b2_tx_data  <= h_rx_data;
                                b2_tx_start <= 1'b1;
                            end
                        endcase

                        if (h_rx_data == 8'h0A) begin
                            timeout_cnt      <= {TIMEOUT_W{1'b0}};
                            board_rx_pending <= 1'b0;
                            state            <= ST_WAIT_RESP;
                        end
                    end
                end

                // -------------------------------------------------------------
                // Relay board response to host until newline
                // -------------------------------------------------------------
                ST_WAIT_RESP: begin
                    timeout_cnt <= timeout_cnt + 1'b1;

                    if (board_rx_pending && !h_tx_busy) begin
                        h_tx_data        <= board_rx_byte;
                        h_tx_start       <= 1'b1;
                        timeout_cnt      <= {TIMEOUT_W{1'b0}};
                        board_rx_pending <= 1'b0;

                        if (board_rx_byte == 8'h0A) begin
                            trig_out <= 1'b0;
                            state    <= ST_IDLE;
                        end
                    end else if (timeout_cnt == TIMEOUT_CYCLES - 1) begin
                        state <= ST_TIMEOUT_T;
                    end
                end

                // -------------------------------------------------------------
                // Timeout response to host: T\n
                // -------------------------------------------------------------
                ST_TIMEOUT_T: begin
                    if (!h_tx_busy) begin
                        h_tx_data  <= "T";
                        h_tx_start <= 1'b1;
                        state      <= ST_TIMEOUT_NL;
                    end
                end

                ST_TIMEOUT_NL: begin
                    if (!h_tx_busy) begin
                        h_tx_data  <= 8'h0A;
                        h_tx_start <= 1'b1;
                        trig_out   <= 1'b0;
                        state      <= ST_IDLE;
                    end
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule