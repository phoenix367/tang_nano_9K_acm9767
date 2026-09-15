/*
 * Copyright (c) 2026 Ivan Gubochkin
 * SPDX-License-Identifier: Apache-2.0
 *
 * Full-duplex UART, 8 data bits, even parity, 1 stop bit (8-E-1). Plain
 * Verilog-2005 port of the FPGA project's src/uart.sv. Baud = CLK_FREQ / BAUD
 * clocks per bit; the receiver samples at bit centres after a 2-FF synchroniser.
 */

`default_nettype none

module uart #(
    parameter integer CLK_FREQ = 50_000_000,
    parameter integer BAUD     = 1_000_000
) (
    input  wire       clk,
    input  wire       reset_n,
    input  wire [7:0] tx_data,
    input  wire       tx_start,
    output reg        tx_busy,
    output reg        tx,
    input  wire       rx,
    output reg  [7:0] rx_data,
    output reg        rx_valid,
    output reg        rx_parity_error
);

    localparam integer CLKS_PER_BIT = CLK_FREQ / BAUD;
    localparam integer HALF_BIT     = CLKS_PER_BIT / 2;
    localparam integer CW = (CLKS_PER_BIT < 4) ? 2 : $clog2(CLKS_PER_BIT + 1);
    /* verilator lint_off WIDTHTRUNC */
    localparam [CW-1:0] LAST_BIT  = CLKS_PER_BIT - 1;
    localparam [CW-1:0] HALF_LAST = HALF_BIT - 1;
    /* verilator lint_on WIDTHTRUNC */

    localparam [2:0] T_IDLE = 3'd0, T_START = 3'd1, T_DATA = 3'd2, T_PAR = 3'd3, T_STOP = 3'd4;
    localparam [2:0] R_IDLE = 3'd0, R_START = 3'd1, R_DATA = 3'd2, R_PAR = 3'd3, R_STOP = 3'd4;

    // -------------------- transmitter --------------------
    reg [2:0]    t_state;
    reg [CW-1:0] t_cnt;
    reg [2:0]    t_bit;
    reg [7:0]    t_shift;
    reg          t_par;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            t_state <= T_IDLE; t_cnt <= 0; t_bit <= 0; t_shift <= 0; t_par <= 1'b0;
            tx <= 1'b1; tx_busy <= 1'b0;
        end else begin
            case (t_state)
                T_IDLE: begin
                    tx <= 1'b1; tx_busy <= 1'b0;
                    if (tx_start) begin
                        t_shift <= tx_data; t_par <= ^tx_data;
                        tx_busy <= 1'b1; tx <= 1'b0; t_cnt <= 0; t_state <= T_START;
                    end
                end
                T_START: begin
                    if (t_cnt == LAST_BIT) begin
                        t_cnt <= 0; t_bit <= 0; tx <= t_shift[0]; t_state <= T_DATA;
                    end else t_cnt <= t_cnt + 1'b1;
                end
                T_DATA: begin
                    tx <= t_shift[0];
                    if (t_cnt == LAST_BIT) begin
                        t_cnt <= 0; t_shift <= {1'b0, t_shift[7:1]};
                        if (t_bit == 3'd7) begin tx <= t_par; t_state <= T_PAR; end
                        else t_bit <= t_bit + 1'b1;
                    end else t_cnt <= t_cnt + 1'b1;
                end
                T_PAR: begin
                    tx <= t_par;
                    if (t_cnt == LAST_BIT) begin t_cnt <= 0; tx <= 1'b1; t_state <= T_STOP; end
                    else t_cnt <= t_cnt + 1'b1;
                end
                T_STOP: begin
                    tx <= 1'b1;
                    if (t_cnt == LAST_BIT) begin t_cnt <= 0; tx_busy <= 1'b0; t_state <= T_IDLE; end
                    else t_cnt <= t_cnt + 1'b1;
                end
                default: t_state <= T_IDLE;
            endcase
        end
    end

    // -------------------- receiver --------------------
    reg          rx_d1, rx_d2;
    reg [2:0]    r_state;
    reg [CW-1:0] r_cnt;
    reg [2:0]    r_bit;
    reg [7:0]    r_shift;
    reg          r_par;

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin rx_d1 <= 1'b1; rx_d2 <= 1'b1; end
        else begin rx_d1 <= rx; rx_d2 <= rx_d1; end
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            r_state <= R_IDLE; r_cnt <= 0; r_bit <= 0; r_shift <= 0; r_par <= 1'b0;
            rx_data <= 0; rx_valid <= 1'b0; rx_parity_error <= 1'b0;
        end else begin
            rx_valid <= 1'b0;
            case (r_state)
                R_IDLE:  if (rx_d2 == 1'b0) begin r_cnt <= 0; r_state <= R_START; end
                R_START: begin
                    if (r_cnt == HALF_LAST) begin
                        if (rx_d2 == 1'b0) begin r_cnt <= 0; r_bit <= 0; r_state <= R_DATA; end
                        else r_state <= R_IDLE;
                    end else r_cnt <= r_cnt + 1'b1;
                end
                R_DATA: begin
                    if (r_cnt == LAST_BIT) begin
                        r_cnt <= 0; r_shift <= {rx_d2, r_shift[7:1]};
                        if (r_bit == 3'd7) r_state <= R_PAR; else r_bit <= r_bit + 1'b1;
                    end else r_cnt <= r_cnt + 1'b1;
                end
                R_PAR: begin
                    if (r_cnt == LAST_BIT) begin r_cnt <= 0; r_par <= rx_d2; r_state <= R_STOP; end
                    else r_cnt <= r_cnt + 1'b1;
                end
                R_STOP: begin
                    if (r_cnt == LAST_BIT) begin
                        r_cnt <= 0; rx_data <= r_shift; rx_valid <= 1'b1;
                        rx_parity_error <= (r_par != (^r_shift));
                        r_state <= R_IDLE;
                    end else r_cnt <= r_cnt + 1'b1;
                end
                default: r_state <= R_IDLE;
            endcase
        end
    end

endmodule
