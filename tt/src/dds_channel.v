/*
 * Copyright (c) 2026 Ivan Gubochkin
 * SPDX-License-Identifier: Apache-2.0
 *
 * One DDS channel producing straight-binary 14-bit DAC codes, one per clock.
 * Sine from sine_interp (64-entry table + interpolation, no RAM); triangle,
 * sawtooth and square from the phase; AM, FM and SSB modulate an internal
 * baseband sample (bb, Q15) that the register file updates at the audio rate.
 *
 *   phase_acc += ftw (+ FM deviation)     f_out = ftw * f_clk / 2^32
 *   ph  = phase_acc[31:16] + phase_ofs
 *   raw = waveform(ph)                     signed 14-bit, +-8191 full scale
 *   dac = sat14(raw * scale >> 16 + offset) + 8192, ^ invert; 0x2000 when disabled
 *
 * Sample-rate multipliers: raw x scale, and for SSB the mixer pair
 * I x cos and Q x sin. Everything that changes once per audio sample
 * (bb x modparam for FM/AM, the AM envelope x gain, the all-pass Hilbert
 * phase splitter and the I/Q amplitude) runs in baseband_dsp on one shared
 * serial multiplier; two 3-stage CIC interpolators bring I and Q to the DAC
 * rate. For AM, `scale` is envelope x gain / 2 so the carrier sits at half
 * scale and 100 % depth peaks at full scale. SSB = I cos - Q sin (USB) or
 * I cos + Q sin (LSB) with the amplitude already applied at baseband.
 *
 * Latency ~8 clocks (sine) / ~5 (others); irrelevant for a free-running
 * generator. Plain Verilog-2005.
 */

`default_nettype none

module dds_channel #(
    parameter SSB = 1              // 0: no SSB (drops the phase splitter, both CICs and the mixer)
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        en,
    input  wire        invert,
    input  wire [31:0] ftw,
    input  wire [15:0] phase_ofs,
    input  wire [15:0] amplitude,     // gain = (amplitude+1)/65536
    input  wire [15:0] offset,        // signed, DAC LSBs
    input  wire [3:0]  waveform,      // 0 sine 1 tri 2 saw 3 square 4 dc 5 usb 6 lsb 7 am 8 fm
    input  wire [15:0] duty,
    input  wire [15:0] modparam,      // AM depth (Q16) / FM deviation (x256 ftw)
    input  wire signed [15:0] bb,     // baseband sample (Q15)
    input  wire        bb_tick,       // pulses when bb has been updated
    output reg  [13:0] dac_code
);

    localparam [3:0] WAVE_SINE = 4'd0, WAVE_TRI = 4'd1, WAVE_SAW = 4'd2, WAVE_SQUARE = 4'd3,
                     WAVE_DC = 4'd4, WAVE_USB = 4'd5, WAVE_LSB = 4'd6, WAVE_AM = 4'd7, WAVE_FM = 4'd8;

    // ---- per-audio-sample DSP (serial multiplier) + I/Q interpolation ----
    wire [16:0] gain = {1'b0, amplitude} + 17'd1;
    wire signed [32:0] mod_prod;
    wire [16:0]        am_scale;
    wire signed [15:0] i_bb, q_bb;
    wire               iq_strobe;
    baseband_dsp #(.HILBERT(SSB)) bbd (
        .clk(clk), .rst_n(rst_n), .bb(bb), .bb_tick(bb_tick), .modparam(modparam), .gain(gain),
        .mod_prod(mod_prod), .am_scale(am_scale), .i_out(i_bb), .q_out(q_bb), .iq_strobe(iq_strobe)
    );
    wire signed [15:0] i_hi, q_hi;                    // at the DAC rate
    generate if (SSB) begin : g_cic
        cic_interp cic_i (.clk(clk), .rst_n(rst_n), .in_strobe(iq_strobe), .in_data(i_bb), .out_data(i_hi));
        cic_interp cic_q (.clk(clk), .rst_n(rst_n), .in_strobe(iq_strobe), .in_data(q_bb), .out_data(q_hi));
    end else begin : g_nocic
        assign i_hi = 16'sd0;
        assign q_hi = 16'sd0;
        wire _unused_iq = &{i_bb, q_bb, iq_strobe, 1'b0};
    end endgenerate

    // ---- phase accumulator with FM deviation ----
    reg [31:0] phase_acc;
    reg signed [25:0] fm_dev;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) fm_dev <= 0;
        else        fm_dev <= (waveform == WAVE_FM) ? mod_prod[32:7] : 26'sd0;
    always @(posedge clk or negedge rst_n)
        if (!rst_n)  phase_acc <= 0;
        else if (en) phase_acc <= phase_acc + ftw + {{6{fm_dev[25]}}, fm_dev};
        else         phase_acc <= 0;

    reg [15:0] ph;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) ph <= 0;
        else        ph <= phase_acc[31:16] + phase_ofs;

    // ---- waveform sources (sine is 3 clocks later than the others; never mixed) ----
    wire signed [13:0] raw_sine, raw_cos;
    sine_interp sine (.clk(clk), .rst_n(rst_n), .phase(ph), .sin_o(raw_sine), .cos_o(raw_cos));

    // ---- SSB mixer: I cos -/+ Q sin (Q15 x Q13 -> >>15 = Q13), saturated ----
    wire signed [13:0] raw_ssb;
    generate if (SSB) begin : g_mix
        reg signed [29:0] p_ic, p_qs;
        always @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin p_ic <= 0; p_qs <= 0; end
            else begin p_ic <= i_hi * raw_cos; p_qs <= q_hi * raw_sine; end
        end
        reg signed [30:0] ssb_sum;
        always @(posedge clk or negedge rst_n)
            if (!rst_n) ssb_sum <= 0;
            else        ssb_sum <= (waveform == WAVE_LSB) ? (p_ic + p_qs) : (p_ic - p_qs);
        wire signed [15:0] ssb_q13 = ssb_sum[30:15];
        reg signed [13:0] raw_ssb_r;
        always @(posedge clk or negedge rst_n)
            if (!rst_n) raw_ssb_r <= 0;
            else        raw_ssb_r <= (ssb_q13 > 16'sd8191) ? 14'sd8191 :
                                     (ssb_q13 < -16'sd8192) ? -14'sd8192 : ssb_q13[13:0];
        assign raw_ssb = raw_ssb_r;
    end else begin : g_nomix
        assign raw_ssb = 14'sd0;
        wire _unused_cos = &{raw_cos, i_hi, q_hi, 1'b0};
    end endgenerate

    reg signed [13:0] raw_tri, raw_saw, raw_sq;
    wire signed [14:0] tri_rise = $signed({1'b0, ph[14:1]}) - 15'sd8192;
    wire signed [14:0] tri_fall = 15'sd8191 - $signed({1'b0, ph[14:1]});
    wire signed [14:0] saw_ramp = $signed({1'b0, ph[15:2]}) - 15'sd8192;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin raw_tri <= 0; raw_saw <= 0; raw_sq <= 0; end
        else begin
            raw_tri <= ph[15] ? tri_fall[13:0] : tri_rise[13:0];
            raw_saw <= saw_ramp[13:0];
            raw_sq  <= (ph < duty) ? 14'sd8191 : -14'sd8192;
        end
    end

    reg signed [13:0] raw;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) raw <= 0;
        else case (waveform)
            WAVE_SINE, WAVE_FM, WAVE_AM: raw <= raw_sine;
            WAVE_USB, WAVE_LSB:          raw <= raw_ssb;
            WAVE_TRI:                    raw <= raw_tri;
            WAVE_SAW:                    raw <= raw_saw;
            WAVE_SQUARE:                 raw <= raw_sq;
            default:                     raw <= 0;
        endcase
    end

    // ---- output scaling: raw x scale (unity for SSB: its gain is applied at baseband) ----
    wire is_ssb = (waveform == WAVE_USB) || (waveform == WAVE_LSB);
    wire [16:0] scale = (waveform == WAVE_AM) ? am_scale : is_ssb ? 17'd65536 : gain;
    reg signed [31:0] product;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) product <= 0;
        else        product <= raw * $signed({1'b0, scale});
    // AM: one extra >>1 so the unmodulated carrier sits at half scale
    wire signed [13:0] scaled = (waveform == WAVE_AM) ? product[30:17] : product[29:16];
    reg signed [16:0] sum;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) sum <= 0;
        else        sum <= {{3{scaled[13]}}, scaled} + {offset[15], offset};
    wire signed [13:0] sat = (sum > 17'sd8191) ? 14'sd8191 :
                             (sum < -17'sd8192) ? -14'sd8192 : sum[13:0];
    wire [13:0] code = {~sat[13], sat[12:0]};
    always @(posedge clk or negedge rst_n)
        if (!rst_n)  dac_code <= 14'h2000;
        else if (en) dac_code <= code ^ {14{invert}};
        else         dac_code <= 14'h2000;

endmodule
