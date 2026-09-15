/*
 * Copyright (c) 2026 Ivan Gubochkin
 * SPDX-License-Identifier: Apache-2.0
 *
 * 17 x 17 -> 34-bit signed shift-add multiplier, one partial product per
 * clock (17 clocks + 1). Used for the per-audio-sample products of the DDS
 * (baseband x modulation parameter, envelope x gain, the all-pass sections),
 * which change once every few thousand clocks and do not justify a parallel
 * array multiplier on an ASIC.
 * Pulse `start` with the operands stable; `done` pulses on the clock at which
 * `p` (the accumulator itself) holds the final product, and `p` keeps it until
 * the next start.
 */

`default_nettype none

module serial_mul (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire signed [16:0] a,
    input  wire signed [16:0] b,
    output wire signed [33:0] p,
    output reg         done
);
    reg [4:0]  cnt;
    reg        busy;
    reg signed [16:0] a_r;
    reg [16:0] b_r;
    reg signed [33:0] acc;
    assign p = acc;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt <= 0; busy <= 1'b0; a_r <= 0; b_r <= 0; acc <= 0; done <= 1'b0;
        end else begin
            done <= 1'b0;
            if (start && !busy) begin
                busy <= 1'b1; cnt <= 0; a_r <= a; b_r <= b; acc <= 0;
            end else if (busy) begin
                // bit 16 of b is the sign bit: weight -2^16
                if (cnt == 5'd16)
                    acc <= acc - (b_r[16] ? ({{17{a_r[16]}}, a_r} <<< 16) : 34'sd0);
                else
                    acc <= acc + (b_r[cnt] ? ({{17{a_r[16]}}, a_r} <<< cnt) : 34'sd0);
                if (cnt == 5'd16) begin busy <= 1'b0; done <= 1'b1; end
                cnt <= cnt + 1'b1;
            end
        end
    end
endmodule
