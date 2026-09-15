/*
 * Copyright (c) 2026 Ivan Gubochkin
 * SPDX-License-Identifier: Apache-2.0
 *
 * Per-audio-sample DSP on one shared 17x17 serial multiplier. After every
 * baseband tick (bb updated) it runs, in ~200 clocks of the 3125 available:
 *
 *   1. mod_prod = bb * modparam                 (FM deviation / AM depth)
 *   2. am_scale = (32768 + mod_prod>>16) * gain >> 16
 *   3. gk       = gain * K >> 16                (Q15; K = 2^23 / 3125^2 pre-compensates the CIC gain)
 *   4-9. Hilbert phase splitter: two chains of three first-order all-pass
 *        sections, y = a*(x - y_prev) + x_prev (Q15, 16-bit states):
 *        chain B -> I (in phase), chain A -> Q (lags I by 90 deg over
 *        250..3600 Hz, max phase error 0.21 deg = 54 dB sideband suppression)
 *   10-11. i_out = I * gk, q_out = Q * gk      (amplitude applied at baseband)
 *
 * iq_strobe pulses when i_out/q_out are updated; the CIC interpolators take
 * them from there to the DAC rate. HILBERT=0 leaves out steps 4-11 (and the
 * chain registers) for the AM/FM-only build.
 */

`default_nettype none

module baseband_dsp #(
    parameter HILBERT = 1          // 0: no phase splitter / I-Q outputs (AM/FM only)
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire signed [15:0] bb,
    input  wire        bb_tick,
    input  wire [15:0] modparam,
    input  wire [16:0] gain,              // amplitude + 1
    output reg  signed [32:0] mod_prod,
    output reg  [16:0] am_scale,
    output reg  signed [15:0] i_out,
    output reg  signed [15:0] q_out,
    output reg         iq_strobe
);
    localparam signed [16:0] K_CIC = 17'sd28147;      // 2^23 / 3125^2 in Q15
    // all-pass coefficients, Q15 (designed by differential evolution, see tt/README.md)
    function signed [16:0] coef(input [2:0] k);
        case (k)
            3'd0: coef = -17'sd31624;  3'd1: coef = -17'sd24684;  3'd2: coef = -17'sd6449;    // chain A -> Q
            3'd3: coef =  17'sd13297;  3'd4: coef = -17'sd17705;  3'd5: coef = -17'sd28908;   // chain B -> I
            default: coef = 17'sd0;
        endcase
    endfunction

    reg  start;
    reg  signed [16:0] ma, mb;
    wire signed [33:0] mp;
    wire done;
    serial_mul smul (.clk(clk), .rst_n(rst_n), .start(start), .a(ma), .b(mb), .p(mp), .done(done));

    // all-pass section states: previous input and previous output per section
    reg signed [15:0] xp [0:5];
    reg signed [15:0] yp [0:5];
    reg signed [15:0] sx;                 // running input of the current chain
    reg signed [15:0] ya, yb;             // chain outputs (Q, I)
    reg [3:0] step;
    reg [2:0] sec;
    reg [16:0] gk;

    function signed [15:0] sat16(input signed [17:0] v);
        sat16 = (v > 18'sd32767) ? 16'sd32767 : (v < -18'sd32768) ? -16'sd32768 : v[15:0];
    endfunction
    wire signed [16:0] diff = {sx[15], sx} - {yp[sec][15], yp[sec]};                   // x - y_prev, 17 bits (no clipping)
    // y = ((a * t) >> 15, rounded) + x_prev ; the product is Q15 x integer
    wire signed [17:0] ysum = {{2{mp[33]}}, mp[30:15]} + {{2{xp[sec][15]}}, xp[sec]} + {17'd0, mp[14]};

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            start <= 1'b0; ma <= 0; mb <= 0; step <= 0; sec <= 0; sx <= 0; ya <= 0; yb <= 0;
            mod_prod <= 0; am_scale <= 17'd32768; gk <= 17'd28147; i_out <= 0; q_out <= 0; iq_strobe <= 1'b0;
            for (i = 0; i < 6; i = i + 1) begin xp[i] <= 0; yp[i] <= 0; end
        end else begin
            start <= 1'b0; iq_strobe <= 1'b0;
            case (step)
                4'd0: if (bb_tick) begin                          // 1. bb * modparam
                    ma <= {bb[15], bb}; mb <= {1'b0, modparam}; start <= 1'b1; step <= 4'd1;
                end
                4'd1: if (done) begin                             // 2. env * gain
                    mod_prod <= mp[32:0];
                    ma <= $signed(17'd32768 + {mp[32], mp[31:16]}); mb <= $signed(gain);
                    start <= 1'b1; step <= 4'd2;
                end
                4'd2: if (done) begin                             // 3. gain * K
                    am_scale <= mp[32:16];
                    ma <= $signed(gain); mb <= K_CIC; start <= 1'b1; step <= 4'd3;
                end
                4'd3: if (done) begin                             // start chain A (sections 0..2)
                    if (HILBERT) begin gk <= mp[32:16]; sx <= bb; sec <= 3'd0; step <= 4'd4; end   // gain*K >> 16 -> Q15
                    else step <= 4'd0;
                end
                4'd4: if (HILBERT) begin                          // issue a * (x - y_prev)
                    ma <= coef(sec); mb <= diff; start <= 1'b1; step <= 4'd5;
                end else step <= 4'd0;
                4'd5: if (HILBERT && done) begin                  // y = (a*t >> 15) + x_prev, advance
                    xp[sec] <= sx;
                    yp[sec] <= sat16(ysum);
                    sx      <= sat16(ysum);
                    if (sec == 3'd2) begin ya <= sat16(ysum); sx <= bb; sec <= 3'd3; step <= 4'd4; end
                    else if (sec == 3'd5) begin yb <= sat16(ysum); step <= 4'd6; end
                    else begin sec <= sec + 1'b1; step <= 4'd4; end
                end
                4'd6: if (HILBERT) begin                          // 10. I * gk
                    ma <= {yb[15], yb}; mb <= $signed(gk); start <= 1'b1; step <= 4'd7;
                end else step <= 4'd0;
                4'd7: if (HILBERT && done) begin                  // 11. Q * gk
                    i_out <= mp[30:15];
                    ma <= {ya[15], ya}; mb <= $signed(gk); start <= 1'b1; step <= 4'd8;
                end
                4'd8: if (HILBERT && done) begin
                    q_out <= mp[30:15]; iq_strobe <= 1'b1; step <= 4'd0;
                end
                default: step <= 4'd0;
            endcase
        end
    end
endmodule
