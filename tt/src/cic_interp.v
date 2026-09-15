/*
 * Copyright (c) 2026 Ivan Gubochkin
 * SPDX-License-Identifier: Apache-2.0
 *
 * 3-stage CIC interpolator, ratio R (one in_strobe every R clocks), input
 * Q15, output the top 16 bits of the last integrator. DC gain is R^2, so the
 * caller pre-scales the input by K = 2^OUT_SHIFT / R^2 for unity gain and
 * takes bits [OUT_SHIFT+15 : OUT_SHIFT]. Wrap-around arithmetic: the
 * intermediate widths only need to hold the final output, ACC_W = 16 + 2*log2(R)
 * + margin. Images of the baseband around multiples of the input rate are
 * suppressed by sinc^3 (~-40 dB for 3 kHz audio at 16 kS/s); passband droop
 * -1.5 dB at 3 kHz.
 */

`default_nettype none

module cic_interp #(
    parameter integer OUT_SHIFT = 23,
    parameter integer ACC_W     = 42
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               in_strobe,
    input  wire signed [15:0] in_data,
    output reg  signed [15:0] out_data
);
    reg signed [ACC_W-1:0] x_d, c1, c1_d, c2, c2_d, c3;
    wire signed [ACC_W-1:0] x_ext = {{(ACC_W-16){in_data[15]}}, in_data};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_d <= 0; c1 <= 0; c1_d <= 0; c2 <= 0; c2_d <= 0; c3 <= 0;
        end else if (in_strobe) begin
            x_d  <= x_ext;
            c1   <= x_ext - x_d;
            c1_d <= c1;
            c2   <= c1 - c1_d;
            c2_d <= c2;
            c3   <= c2 - c2_d;
        end
    end

    reg strobe_d;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) strobe_d <= 1'b0; else strobe_d <= in_strobe;

    reg signed [ACC_W-1:0] i1, i2, i3;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin i1 <= 0; i2 <= 0; i3 <= 0; end
        else begin
            i1 <= i1 + (strobe_d ? c3 : {ACC_W{1'b0}});
            i2 <= i2 + i1;
            i3 <= i3 + i2;
        end
    end

    wire [ACC_W-1:OUT_SHIFT+15] top = i3[ACC_W-1:OUT_SHIFT+15];
    wire in_range = (&top) | (~|top);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)        out_data <= 0;
        else if (in_range) out_data <= i3[OUT_SHIFT+15:OUT_SHIFT];
        else               out_data <= i3[ACC_W-1] ? -16'sd32768 : 16'sd32767;
    end
endmodule
