`include "timescale.v"
`include "acm9767_defs.vh"
`include "platform_config.vh"

// Baseband side of the SSB modulator (dac_clk domain):
//   every INTERP clocks pop one audio sample from the FIFO (hold the last one
//   on underrun), first-order DC blocker, hilbert_fir -> (I, Q) at the audio
//   rate, envelope_limiter (look-ahead, keeps |I+jQ| below the combiner's
//   saturation), two cic_interp -> (bb_i, bb_q) at the DAC rate, fed to
//   dds_channel's SSB waveform (I*cos - Q*sin); AM/FM use bb_i.
// underrun_tog flips once per empty-FIFO tick; dac_regs counts the edges. A
// run of >= 8 underruns (host stream stopped) makes the limiter fade the next
// data in from zero, so a resumed stream never starts with a step.

module ssb_baseband (
    input  wire               clk,
    input  wire               rst_n,
    // FIFO read side (async_fifo)
    output reg                fifo_rd,
    input  wire signed [15:0] fifo_rdata,
    input  wire               fifo_empty,
    // analytic baseband at the DAC rate
    output wire signed [15:0] bb_i,
    output wire signed [15:0] bb_q,
    output reg                underrun_tog
);

localparam integer INTERP   = `PLATFORM_SSB_INTERP;
localparam integer DC_SHIFT = `PLATFORM_SSB_DC_SHIFT;   // HP corner = f_audio / (2 pi 2^DC_SHIFT)

// ---- sample tick ----
reg [23:0] tick_cnt;
wire tick = (tick_cnt == INTERP - 1);
always @(posedge clk or negedge rst_n)
    if (!rst_n)    tick_cnt <= `WRAP_SIM(#1) 24'd0;
    else if (tick) tick_cnt <= `WRAP_SIM(#1) 24'd0;
    else           tick_cnt <= `WRAP_SIM(#1) tick_cnt + 24'd1;

// ---- FIFO pop: rd on the tick, rdata valid the clock after; on underrun keep
// the previous sample (the DC blocker then decays it, no step into the FIR) ----
reg rd_d, ur_evt, ur_d, hp_strobe, fir_strobe, fade;
reg signed [15:0] x, x_prev, fir_in;
reg signed [17:0] y_prev;
reg [3:0] ur_run;
// (sign-extend through signed wires: a concatenation is unsigned and would turn
// the arithmetic shift of the feedback term into a logical one)
wire signed [17:0] x_ext  = {{2{x[15]}}, x};
wire signed [17:0] xp_ext = {{2{x_prev[15]}}, x_prev};
wire signed [17:0] y_leak = y_prev >>> DC_SHIFT;
wire signed [17:0] y_full = x_ext - xp_ext + y_prev - y_leak;
wire signed [15:0] y_sat  = (y_full > 18'sd32767)  ? 16'sd32767 :
                            (y_full < -18'sd32768) ? -16'sd32768 : y_full[15:0];
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fifo_rd      <= `WRAP_SIM(#1) 1'b0;
        ur_evt       <= `WRAP_SIM(#1) 1'b0;
        rd_d         <= `WRAP_SIM(#1) 1'b0;
        ur_d         <= `WRAP_SIM(#1) 1'b0;
        hp_strobe    <= `WRAP_SIM(#1) 1'b0;
        fir_strobe   <= `WRAP_SIM(#1) 1'b0;
        fade         <= `WRAP_SIM(#1) 1'b0;
        x            <= `WRAP_SIM(#1) 16'sd0;
        x_prev       <= `WRAP_SIM(#1) 16'sd0;
        y_prev       <= `WRAP_SIM(#1) 18'sd0;
        fir_in       <= `WRAP_SIM(#1) 16'sd0;
        ur_run       <= `WRAP_SIM(#1) 4'd0;
        underrun_tog <= `WRAP_SIM(#1) 1'b0;
    end else begin
        fifo_rd    <= `WRAP_SIM(#1) tick && !fifo_empty;
        ur_evt     <= `WRAP_SIM(#1) tick && fifo_empty;
        rd_d       <= `WRAP_SIM(#1) fifo_rd;
        ur_d       <= `WRAP_SIM(#1) ur_evt;
        hp_strobe  <= `WRAP_SIM(#1) rd_d | ur_d;
        fir_strobe <= `WRAP_SIM(#1) hp_strobe;
        fade       <= `WRAP_SIM(#1) rd_d && (ur_run == 4'd8);
        if (rd_d) begin
            x      <= `WRAP_SIM(#1) fifo_rdata;
            ur_run <= `WRAP_SIM(#1) 4'd0;
        end else if (ur_d) begin
            underrun_tog <= `WRAP_SIM(#1) ~underrun_tog;
            if (ur_run != 4'd8) ur_run <= `WRAP_SIM(#1) ur_run + 4'd1;
        end
        if (hp_strobe) begin                         // DC blocker: y = x - x_prev + y_prev*(1 - 2^-DC_SHIFT)
            x_prev <= `WRAP_SIM(#1) x;
            y_prev <= `WRAP_SIM(#1) y_full;
            fir_in <= `WRAP_SIM(#1) y_sat;
        end
    end
end

// ---- Hilbert / delay ----
wire               an_strobe;
wire signed [15:0] an_i, an_q;
hilbert_fir #(.TAPS(`PLATFORM_SSB_HILBERT_TAPS), .K(`PLATFORM_SSB_GAIN_K)) fir (
    .clk(clk), .rst_n(rst_n),
    .in_strobe(fir_strobe), .in_data(fir_in),
    .out_strobe(an_strobe), .i_out(an_i), .q_out(an_q)
);

// ---- envelope limiter (look-ahead), keeps the SSB combiner out of saturation ----
wire               lim_strobe;
wire signed [15:0] lim_i, lim_q;
wire        [16:0] lim_gain;
envelope_limiter #(.LOOKAHEAD(`PLATFORM_SSB_LIM_LOOKAHEAD), .THRESHOLD(`PLATFORM_SSB_LIM_THRESHOLD),
                   .RELEASE(`PLATFORM_SSB_LIM_RELEASE)) lim (
    .clk(clk), .rst_n(rst_n),
    .in_strobe(an_strobe), .i_in(an_i), .q_in(an_q), .fade(fade),
    .out_strobe(lim_strobe), .i_out(lim_i), .q_out(lim_q), .gain(lim_gain)
);

// ---- interpolate both rails to the DAC rate ----
cic_interp #(.OUT_SHIFT(`PLATFORM_SSB_CIC_SHIFT)) cic_i (
    .clk(clk), .rst_n(rst_n), .in_strobe(lim_strobe), .in_data(lim_i), .out_data(bb_i)
);
cic_interp #(.OUT_SHIFT(`PLATFORM_SSB_CIC_SHIFT)) cic_q (
    .clk(clk), .rst_n(rst_n), .in_strobe(lim_strobe), .in_data(lim_q), .out_data(bb_q)
);

endmodule
