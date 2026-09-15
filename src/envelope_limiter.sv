`include "timescale.v"
`include "acm9767_defs.vh"

// Look-ahead envelope limiter for the analytic baseband (I, Q at the audio
// rate, between hilbert_fir and the CIC interpolators).
//
//   env  ~= max(|i|,|q|) + 3/8*min(|i|,|q|)   (-2.8 % .. +6.8 % of sqrt(i^2+q^2))
//   gain  = min( THRESHOLD*2^15 / max(env over the LOOKAHEAD window),
//                gain + RELEASE )                                     (Q15)
//   i_out = i[n-LOOKAHEAD] * gain >> 15,   q_out likewise
//
// The window covers the sample being output and the LOOKAHEAD samples behind
// it, so the gain is already down when a peak reaches the output (zero attack
// time, no overshoot) and the envelope leaving the block never exceeds
// ~1.03*THRESHOLD; it recovers by RELEASE per sample. `fade` restarts from
// gain 0 so a resumed stream ramps in over 32768/RELEASE samples.
//
// gain is 17 bits so that unity (32768) is exact.
// Sequential and constant-latency (the CICs need a strictly periodic strobe): one sample costs LOOKAHEAD + 24 clocks (abs, max/min, env,
// window scan, 16-step restoring divider, gain update, DSP multiply); the
// audio period is thousands of clocks. Two 17x16 multipliers, no other DSP.

module envelope_limiter #(
    parameter integer LOOKAHEAD = 8,
    parameter [15:0]  THRESHOLD = 16'd31000,
    parameter integer RELEASE   = 32
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               in_strobe,
    input  wire signed [15:0] i_in,
    input  wire signed [15:0] q_in,
    input  wire               fade,        // pulse: the next sample starts from gain 0
    output reg                out_strobe,
    output reg  signed [15:0] i_out,
    output reg  signed [15:0] q_out,
    output reg         [16:0] gain          // 32768 = unity (Q15)
);

localparam [3:0] S_IDLE = 4'd0, S_ABS = 4'd1, S_MAXMIN = 4'd2, S_ENV = 4'd3, S_SHIFT = 4'd4,
                 S_SCAN = 4'd5, S_DIV = 4'd6, S_GAIN = 4'd7, S_MUL = 4'd8, S_OUT = 4'd9;
localparam [30:0] NUMER = {THRESHOLD, 15'd0};        // THRESHOLD * 2^15

reg [3:0]  state;
reg signed [15:0] i_new, q_new, i_old, q_old;
reg        [15:0] ai, aq, mx, mn;
reg        [16:0] env_new, env_max, env_old;
reg signed [15:0] i_dl [0:LOOKAHEAD-1];
reg signed [15:0] q_dl [0:LOOKAHEAD-1];
reg        [16:0] env_dl [0:LOOKAHEAD-1];
reg        [7:0]  idx;
reg        [17:0] rem;                               // divider remainder
reg        [16:0] quot;
reg        [17:0] g_rel;
reg               fade_pend;
reg signed [33:0] p_i, p_q;
integer k;

wire [18:0] rem_sh  = {rem, NUMER[idx[3:0]]};        // shift in the next numerator bit
wire        rem_ge  = (rem_sh >= {2'b00, env_max});
wire [17:0] g_sum   = {1'b0, gain} + RELEASE[17:0];

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= `WRAP_SIM(#1) S_IDLE;
        gain <= `WRAP_SIM(#1) 17'd32768;
        out_strobe <= `WRAP_SIM(#1) 1'b0;
        i_out <= `WRAP_SIM(#1) 16'sd0; q_out <= `WRAP_SIM(#1) 16'sd0;
        i_new <= `WRAP_SIM(#1) 16'sd0; q_new <= `WRAP_SIM(#1) 16'sd0;
        i_old <= `WRAP_SIM(#1) 16'sd0; q_old <= `WRAP_SIM(#1) 16'sd0;
        ai <= `WRAP_SIM(#1) 16'd0; aq <= `WRAP_SIM(#1) 16'd0; mx <= `WRAP_SIM(#1) 16'd0; mn <= `WRAP_SIM(#1) 16'd0;
        env_new <= `WRAP_SIM(#1) 17'd0; env_max <= `WRAP_SIM(#1) 17'd0; env_old <= `WRAP_SIM(#1) 17'd0;
        idx <= `WRAP_SIM(#1) 8'd0; rem <= `WRAP_SIM(#1) 18'd0; quot <= `WRAP_SIM(#1) 17'd0;
        g_rel <= `WRAP_SIM(#1) 18'd0; fade_pend <= `WRAP_SIM(#1) 1'b0;
        p_i <= `WRAP_SIM(#1) 34'sd0; p_q <= `WRAP_SIM(#1) 34'sd0;
        for (k = 0; k < LOOKAHEAD; k = k + 1) begin
            i_dl[k] <= `WRAP_SIM(#1) 16'sd0; q_dl[k] <= `WRAP_SIM(#1) 16'sd0; env_dl[k] <= `WRAP_SIM(#1) 17'd0;
        end
    end else begin
        out_strobe <= `WRAP_SIM(#1) 1'b0;
        if (fade) fade_pend <= `WRAP_SIM(#1) 1'b1;
        case (state)
        S_IDLE: if (in_strobe) begin
            i_new <= `WRAP_SIM(#1) i_in; q_new <= `WRAP_SIM(#1) q_in;
            state <= `WRAP_SIM(#1) S_ABS;
        end
        S_ABS: begin
            ai <= `WRAP_SIM(#1) i_new[15] ? -i_new : i_new;
            aq <= `WRAP_SIM(#1) q_new[15] ? -q_new : q_new;
            state <= `WRAP_SIM(#1) S_MAXMIN;
        end
        S_MAXMIN: begin
            mx <= `WRAP_SIM(#1) (ai > aq) ? ai : aq;
            mn <= `WRAP_SIM(#1) (ai > aq) ? aq : ai;
            state <= `WRAP_SIM(#1) S_ENV;
        end
        S_ENV: begin
            env_new <= `WRAP_SIM(#1) {1'b0, mx} + {3'b000, mn[15:2]} + {4'b0000, mn[15:3]};
            state <= `WRAP_SIM(#1) S_SHIFT;
        end
        S_SHIFT: begin                               // pop the oldest, push the new
            i_old <= `WRAP_SIM(#1) i_dl[LOOKAHEAD-1];
            q_old <= `WRAP_SIM(#1) q_dl[LOOKAHEAD-1];
            env_old <= `WRAP_SIM(#1) env_dl[LOOKAHEAD-1];
            env_max <= `WRAP_SIM(#1) env_dl[LOOKAHEAD-1];
            for (k = LOOKAHEAD - 1; k > 0; k = k - 1) begin
                i_dl[k] <= `WRAP_SIM(#1) i_dl[k-1]; q_dl[k] <= `WRAP_SIM(#1) q_dl[k-1]; env_dl[k] <= `WRAP_SIM(#1) env_dl[k-1];
            end
            i_dl[0] <= `WRAP_SIM(#1) i_new; q_dl[0] <= `WRAP_SIM(#1) q_new; env_dl[0] <= `WRAP_SIM(#1) env_new;
            idx <= `WRAP_SIM(#1) 8'd0;
            state <= `WRAP_SIM(#1) S_SCAN;
        end
        S_SCAN: begin                                // max over the window (output sample + LOOKAHEAD behind it)
            if (env_dl[idx] > env_max) env_max <= `WRAP_SIM(#1) env_dl[idx];
            if (idx == LOOKAHEAD - 1) begin
                idx <= `WRAP_SIM(#1) 8'd15; rem <= `WRAP_SIM(#1) {3'b000, NUMER[30:16]}; quot <= `WRAP_SIM(#1) 17'd0;
                state <= `WRAP_SIM(#1) S_DIV;
            end else
                idx <= `WRAP_SIM(#1) idx + 8'd1;
        end
        S_DIV: begin                                 // quot = NUMER / env_max, 16 restoring steps (env_max > THRESHOLD => quot < 2^15).
            // Always run all 16 steps: the latency from in_strobe to out_strobe must be
            // constant, the CIC interpolators behind us integrate for exactly R clocks per
            // sample and never recover from an irregular interval (a permanent DC residue).
            rem <= `WRAP_SIM(#1) rem_ge ? (rem_sh[17:0] - {1'b0, env_max}) : rem_sh[17:0];
            quot[idx[3:0]] <= `WRAP_SIM(#1) rem_ge;
            if (idx[3:0] == 4'd0) state <= `WRAP_SIM(#1) S_GAIN;
            idx <= `WRAP_SIM(#1) idx - 8'd1;
        end
        S_GAIN: begin
            g_rel = (g_sum > 18'd32768) ? 18'd32768 : g_sum;       // blocking: used below in this state
            if (fade_pend) begin
                gain <= `WRAP_SIM(#1) 17'd0;
                fade_pend <= `WRAP_SIM(#1) 1'b0;
            end else if (env_max > {1'b0, THRESHOLD} && quot < g_rel[16:0])
                gain <= `WRAP_SIM(#1) quot;
            else
                gain <= `WRAP_SIM(#1) g_rel[16:0];
            state <= `WRAP_SIM(#1) S_MUL;
        end
        S_MUL: begin
            p_i <= `WRAP_SIM(#1) i_old * $signed({1'b0, gain});   // 16 x 18 signed
            p_q <= `WRAP_SIM(#1) q_old * $signed({1'b0, gain});
            state <= `WRAP_SIM(#1) S_OUT;
        end
        S_OUT: begin
            i_out <= `WRAP_SIM(#1) p_i[30:15];
            q_out <= `WRAP_SIM(#1) p_q[30:15];
            out_strobe <= `WRAP_SIM(#1) 1'b1;
            state <= `WRAP_SIM(#1) S_IDLE;
        end
        default: state <= `WRAP_SIM(#1) S_IDLE;
        endcase
    end
end

endmodule
