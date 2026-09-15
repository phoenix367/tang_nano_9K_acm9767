/*
 * Copyright (c) 2026 Ivan Gubochkin
 * SPDX-License-Identifier: Apache-2.0
 *
 * 16-bit phase (2^16 = one turn) -> signed 14-bit sine and cosine (+-8191),
 * 3 clocks of latency, no RAM: a 64-entry quarter-wave table (256 points per
 * turn, entries at the interval midpoints, 2 extra fraction bits) plus linear
 * interpolation with the slope from the same table (cos = mirrored index):
 *
 *   sin(a + d) ~= S[k] + C[k] * d * 2*pi/65536,   d = fine - 128 (signed)
 *   cos(a + d) ~= C[k] - S[k] * d * 2*pi/65536     (same tables, second multiplier)
 *
 * Midpoint tables make the residual second-order error < 0.6 LSB, so the
 * result is within +-1 LSB of round(8191*sin) for every one of the 65536
 * phases (tt/test/test_sine.py). The 2*pi factor is 4 + 2 + 1/4 + 1/32 =
 * 6.28125 (0.03 % low, negligible). Replaces a 16-stage CORDIC at ~1/5 the area.
 */

`default_nettype none

module sine_interp (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [15:0] phase,
    output reg  signed [13:0] sin_o,
    output reg  signed [13:0] cos_o
);
    function [15:0] sine_tab(input [5:0] k);   // 4 * 8191 * sin((k+0.5) * 2pi/256)
        case (k)
            6'd0: sine_tab = 16'd402;
            6'd1: sine_tab = 16'd1206;
            6'd2: sine_tab = 16'd2009;
            6'd3: sine_tab = 16'd2811;
            6'd4: sine_tab = 16'd3611;
            6'd5: sine_tab = 16'd4409;
            6'd6: sine_tab = 16'd5205;
            6'd7: sine_tab = 16'd5997;
            6'd8: sine_tab = 16'd6786;
            6'd9: sine_tab = 16'd7570;
            6'd10: sine_tab = 16'd8350;
            6'd11: sine_tab = 16'd9125;
            6'd12: sine_tab = 16'd9895;
            6'd13: sine_tab = 16'd10658;
            6'd14: sine_tab = 16'd11416;
            6'd15: sine_tab = 16'd12166;
            6'd16: sine_tab = 16'd12909;
            6'd17: sine_tab = 16'd13644;
            6'd18: sine_tab = 16'd14371;
            6'd19: sine_tab = 16'd15089;
            6'd20: sine_tab = 16'd15798;
            6'd21: sine_tab = 16'd16498;
            6'd22: sine_tab = 16'd17188;
            6'd23: sine_tab = 16'd17867;
            6'd24: sine_tab = 16'd18536;
            6'd25: sine_tab = 16'd19193;
            6'd26: sine_tab = 16'd19839;
            6'd27: sine_tab = 16'd20473;
            6'd28: sine_tab = 16'd21094;
            6'd29: sine_tab = 16'd21703;
            6'd30: sine_tab = 16'd22299;
            6'd31: sine_tab = 16'd22882;
            6'd32: sine_tab = 16'd23450;
            6'd33: sine_tab = 16'd24005;
            6'd34: sine_tab = 16'd24545;
            6'd35: sine_tab = 16'd25070;
            6'd36: sine_tab = 16'd25580;
            6'd37: sine_tab = 16'd26075;
            6'd38: sine_tab = 16'd26554;
            6'd39: sine_tab = 16'd27017;
            6'd40: sine_tab = 16'd27464;
            6'd41: sine_tab = 16'd27894;
            6'd42: sine_tab = 16'd28307;
            6'd43: sine_tab = 16'd28704;
            6'd44: sine_tab = 16'd29083;
            6'd45: sine_tab = 16'd29444;
            6'd46: sine_tab = 16'd29788;
            6'd47: sine_tab = 16'd30114;
            6'd48: sine_tab = 16'd30422;
            6'd49: sine_tab = 16'd30711;
            6'd50: sine_tab = 16'd30982;
            6'd51: sine_tab = 16'd31234;
            6'd52: sine_tab = 16'd31468;
            6'd53: sine_tab = 16'd31682;
            6'd54: sine_tab = 16'd31877;
            6'd55: sine_tab = 16'd32054;
            6'd56: sine_tab = 16'd32210;
            6'd57: sine_tab = 16'd32348;
            6'd58: sine_tab = 16'd32466;
            6'd59: sine_tab = 16'd32564;
            6'd60: sine_tab = 16'd32643;
            6'd61: sine_tab = 16'd32702;
            6'd62: sine_tab = 16'd32742;
            6'd63: sine_tab = 16'd32762;
            default: sine_tab = 16'd0;
        endcase
    endfunction

    // stage 1: fold to the first quadrant, look up S and C, keep the fine offset
    reg  [1:0]        q1;
    reg  signed [15:0] s1, c1;
    reg  signed [8:0]  d1;
    wire [5:0] k   = phase[13:8];
    wire [5:0] kf  = phase[14] ? ~k : k;                       // mirror in quadrants 1 and 3
    wire signed [8:0] d  = {1'b0, phase[7:0]} - 9'sd128;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin q1 <= 0; s1 <= 0; c1 <= 0; d1 <= 0; end
        else begin
            q1 <= phase[15:14];
            s1 <= $signed(sine_tab(kf));
            c1 <= $signed(sine_tab(~kf));                     // cos((k+0.5)D) = sin((63-k+0.5)D)
            d1 <= phase[14] ? -d : d;
        end
    end

    // stage 2: slope products and the 2*pi scaling
    reg  signed [15:0] s2, c2;
    reg  [1:0]         q2;
    reg  signed [24:0] ps, pc;                       // C*d (sine slope), S*d (cosine slope)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin s2 <= 0; c2 <= 0; q2 <= 0; ps <= 0; pc <= 0; end
        else begin s2 <= s1; c2 <= c1; q2 <= q1; ps <= c1 * d1; pc <= s1 * d1; end
    end
    function signed [13:0] interp(input signed [15:0] base, input signed [24:0] p, input neg);
        reg signed [27:0] pe, p6;
        reg signed [11:0] corr;
        reg signed [16:0] v, vr;
        reg signed [14:0] r15;
        begin
            pe   = {{3{p[24]}}, p};
            p6   = (pe <<< 2) + (pe <<< 1) + (pe >>> 2) + (pe >>> 5);                   // p * 6.28125
            corr = p6[27:16];                                                          // >> 16, signed
            v    = neg ? ({base[15], base} - {{5{corr[11]}}, corr})
                       : ({base[15], base} + {{5{corr[11]}}, corr});                   // 1/4 LSB units
            vr   = (v + 17'sd2) >>> 2;
            r15  = vr[14:0];
            interp = (r15 > 15'sd8191) ? 14'sd8191 : (r15 < -15'sd8191) ? -14'sd8191 : r15[13:0];
        end
    endfunction
    wire signed [13:0] rs = interp(s2, ps, 1'b0);
    // cos in the mirrored quadrants (1 and 3) is negative: the fold makes the
    // mirrored angle's sine positive, its cosine flips sign
    wire signed [13:0] rc = interp(c2, pc, 1'b1);

    // stage 3: signs for the half turn (sine) and the odd quadrants (cosine)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin sin_o <= 0; cos_o <= 0; end
        else begin
            sin_o <= q2[1] ? -rs : rs;
            cos_o <= (q2[1] ^ q2[0]) ? -rc : rc;
        end
    end
endmodule
