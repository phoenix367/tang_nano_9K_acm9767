`include "timescale.v"
`include "acm9767_defs.vh"
`include "svlogger.sv"
`include "test_utils.sv"
`include "test_config.sv"

// Unit test for envelope_limiter (LOOKAHEAD 8, THRESHOLD 31000, RELEASE 32):
//  1. small signals pass unchanged, delayed by exactly LOOKAHEAD samples;
//  2. a peak whose envelope exceeds the threshold is scaled so that the true
//     envelope sqrt(i^2+q^2) leaving the block stays below 32768 (the SSB
//     combiner's saturation point), and the gain is already reduced on the
//     LOOKAHEAD samples before the peak (no attack overshoot);
//  3. the gain releases back to unity afterwards;
//  4. `fade` restarts the gain from zero and ramps up by RELEASE per sample.

module main();

localparam LOG_LEVEL = `DEFAULT_LOG_LEVEL;
localparam integer L = 8;
localparam integer THR = 31000;
localparam integer REL = 32;

reg clk = 0, rst_n = 0;
reg in_strobe = 0, fade = 0;
reg signed [15:0] i_in = 0, q_in = 0;
wire out_strobe;
wire signed [15:0] i_out, q_out;
wire [16:0] gain;

string module_name;
DataLogger #(.verbosity(LOG_LEVEL)) logger();

envelope_limiter #(.LOOKAHEAD(L), .THRESHOLD(THR), .RELEASE(REL)) dut (
    .clk(clk), .rst_n(rst_n), .in_strobe(in_strobe), .i_in(i_in), .q_in(q_in), .fade(fade),
    .out_strobe(out_strobe), .i_out(i_out), .q_out(q_out), .gain(gain)
);
always #9.26 clk = ~clk;

integer errors, n, got, seen_strobe;
reg signed [15:0] hist_i [0:4095];
reg signed [15:0] hist_q [0:4095];
reg signed [15:0] out_i [0:4095];
reg signed [15:0] out_q [0:4095];
real env, worst_env, expv;
string str;

// push one sample and wait for its output (limiter takes ~L+24 clocks)
task automatic sample(input signed [15:0] i, input signed [15:0] q, input integer idx);
    integer w;
    begin
        @(negedge clk); i_in = i; q_in = q; in_strobe = 1;
        @(negedge clk); in_strobe = 0;
        hist_i[idx] = i; hist_q[idx] = q;
        seen_strobe = 0;
        for (w = 0; w < 80 && !seen_strobe; w = w + 1) begin
            @(posedge clk); #2;
            if (out_strobe) begin seen_strobe = 1; out_i[idx] = i_out; out_q[idx] = q_out; end
        end
        if (!seen_strobe) begin logger.error(module_name, "no out_strobe within 80 clocks"); errors++; end
    end
endtask

initial begin
    errors = 0;
`ifdef ENABLE_DUMPVARS
    $dumpvars(0, main);
`endif
    $sformat(module_name, "%m");
    logger.info(module_name, " << Starting the Simulation >>");
    repeat (3) @(posedge clk); rst_n = 1;
    if (gain != 17'd32768) begin logger.error(module_name, "gain not unity after reset"); errors++; end

    // 1. transparent path: a 20000-peak circle (env 20000 < THR), 64 samples
    for (n = 0; n < 64; n = n + 1)
        sample($rtoi(20000.0 * $cos(2.0 * 3.14159265358979 * n / 16.0)),
               $rtoi(20000.0 * $sin(2.0 * 3.14159265358979 * n / 16.0)), n);
    for (n = L; n < 64; n = n + 1)
        if (out_i[n] != hist_i[n - L] || out_q[n] != hist_q[n - L]) begin
            $sformat(str, "sample %0d: out (%0d,%0d) != in[n-%0d] (%0d,%0d)", n, out_i[n], out_q[n], L, hist_i[n - L], hist_q[n - L]);
            logger.error(module_name, str); errors++;
        end
    if (gain != 17'd32768) begin logger.error(module_name, "gain moved on a small signal"); errors++; end
    logger.info(module_name, "small signal passes with LOOKAHEAD delay");

    // 2. peak: at sample 100 a (32767, 32767) hit (true env 46340), otherwise the 20000 circle
    worst_env = 0.0;
    for (n = 64; n < 160; n = n + 1) begin
        if (n == 100) sample(16'sd32767, 16'sd32767, n);
        else sample($rtoi(20000.0 * $cos(2.0 * 3.14159265358979 * n / 16.0)),
                    $rtoi(20000.0 * $sin(2.0 * 3.14159265358979 * n / 16.0)), n);
        env = $sqrt(1.0 * out_i[n] * out_i[n] + 1.0 * out_q[n] * out_q[n]);
        if (env > worst_env) worst_env = env;
        // gain must already be below unity when the peak enters the window (outputs n = 100 .. 100+L)
        if (n >= 100 && n <= 100 + L && gain > 17'd23000) begin
            $sformat(str, "sample %0d: gain %0d not reduced ahead of the peak", n, gain); logger.error(module_name, str); errors++;
        end
    end
    $sformat(str, "worst output envelope around the peak: %0.0f (limit 32768, threshold %0d)", worst_env, THR);
    logger.info(module_name, str);
    if (worst_env >= 32768.0) begin logger.error(module_name, "envelope reached the combiner's saturation"); errors++; end
    if (worst_env < 30000.0) begin logger.error(module_name, "peak was over-attenuated (limiter too conservative)"); errors++; end
    // the peak sample itself (output index 100+L) must carry the reduced gain: |i| ~ 32767 * THR/46340*... <= 23200
    if (out_i[100 + L] > 16'sd23500 || out_q[100 + L] > 16'sd23500) begin
        $sformat(str, "peak output (%0d,%0d) too large", out_i[100 + L], out_q[100 + L]); logger.error(module_name, str); errors++;
    end

    // 3. release: within 1200 more samples the gain is back to unity, in steps of REL
    got = gain;
    for (n = 160; n < 1360; n = n + 1) begin
        sample(16'sd1000, 16'sd0, n);
        if (gain > got + REL) begin $sformat(str, "release step %0d > %0d", gain - got, REL); logger.error(module_name, str); errors++; end
        got = gain;
    end
    if (gain != 17'd32768) begin $sformat(str, "gain %0d after release", gain); logger.error(module_name, str); errors++; end
    else logger.info(module_name, "released to unity");

    // 4. fade: pulse, then a constant 20000 on I; the output ramps from ~0 by 20000*REL/32768 per sample
    @(negedge clk); fade = 1; @(negedge clk); fade = 0;
    for (n = 1360; n < 1360 + 1100; n = n + 1) begin
        sample(16'sd20000, 16'sd0, n);
        // the faded sample emerges L samples later, by which time the gain has ramped (L+1)*REL
        if (n == 1360 + L + 1 && out_i[n] > $rtoi(20000.0 * (L + 2) * REL / 32768.0) + 50) begin
            $sformat(str, "fade: first output %0d not near zero", out_i[n]); logger.error(module_name, str); errors++;
        end
        if (n == 1360 + L + 200) begin
            expv = 20000.0 * (200.0 * REL) / 32768.0;
            if (out_i[n] < expv - 200 || out_i[n] > expv + 200) begin
                $sformat(str, "fade: output %0d at +200 samples, expected ~%0.0f", out_i[n], expv); logger.error(module_name, str); errors++;
            end
        end
    end
    if (out_i[1360 + 1099] != 16'sd20000) begin
        $sformat(str, "fade: final output %0d", out_i[1360 + 1099]); logger.error(module_name, str); errors++;
    end else logger.info(module_name, "fade-in ramps from zero to unity");

    if (errors == 0) begin `TEST_PASS end else begin `TEST_FAIL end
end

initial begin #200000000; logger.error(module_name, "watchdog"); `TEST_FAIL end

endmodule
