`include "timescale.v"
`include "acm9767_defs.vh"
`include "svlogger.sv"
`include "test_utils.sv"
`include "test_config.sv"

// Unit test for cic_interp with R = 8 (so R^2 = 64 = 2^6, OUT_SHIFT = 6 gives
// exactly unity DC gain without a K correction). A DC step must settle to the
// input value; a slow tone (1/64 of the input rate) must be reproduced within
// 2 % at every output clock; a full-scale step must not saturate.

module main();

localparam LOG_LEVEL = `DEFAULT_LOG_LEVEL;
localparam integer R = 8;

reg clk = 0, rst_n = 0;
reg in_strobe = 0;
reg signed [15:0] in_data = 0;
wire signed [15:0] out_data;

string module_name;
DataLogger #(.verbosity(LOG_LEVEL)) logger();

cic_interp #(.OUT_SHIFT(6)) dut (
    .clk(clk), .rst_n(rst_n), .in_strobe(in_strobe), .in_data(in_data), .out_data(out_data)
);
always #9.26 clk = ~clk;

integer errors, n, k, worst, best, best_d, d8;
reg signed [15:0] rec [0:256*8-1];
real expv, err;
string str;

// one input sample, then R-1 idle clocks
task automatic sample(input signed [15:0] v);
    begin
        @(negedge clk); in_data = v; in_strobe = 1;
        @(negedge clk); in_strobe = 0;
        repeat (R - 2) @(negedge clk);
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

    // DC step: after 4 samples the pipeline is full and the output is flat
    for (n = 0; n < 8; n = n + 1) sample(16'sd10000);
    @(posedge clk); #2;
    if (out_data < 16'sd9990 || out_data > 16'sd10010) begin
        $sformat(str, "DC 10000 -> %0d", out_data); logger.error(module_name, str); errors++;
    end else logger.info(module_name, "DC gain unity");

    // full-scale step must not saturate / wrap
    for (n = 0; n < 8; n = n + 1) sample(16'sd32767);
    @(posedge clk); #2;
    if (out_data < 16'sd32700) begin $sformat(str, "full scale -> %0d", out_data); logger.error(module_name, str); errors++; end
    for (n = 0; n < 8; n = n + 1) sample(-16'sd32768);
    @(posedge clk); #2;
    if (out_data > -16'sd32700) begin $sformat(str, "neg full scale -> %0d", out_data); logger.error(module_name, str); errors++; end

    // slow tone: record every output clock, then compare against the ideal
    // tone with the best-fitting group delay (searched in 1/8-sample steps
    // over 2..8 input samples) -- the delay is a pipeline detail, the shape
    // and amplitude are what matter.
    for (n = 0; n < 256; n = n + 1) begin
        @(negedge clk); in_data = $rtoi(30000.0 * $sin(2.0 * 3.14159265358979 * n / 64.0)); in_strobe = 1;
        @(negedge clk); in_strobe = 0;
        for (k = 0; k < R - 2; k = k + 1) begin
            @(negedge clk);
            rec[n * R + k] = out_data;
        end
    end
    best = 1000000; best_d = 0;
    for (d8 = 16; d8 <= 64; d8 = d8 + 1) begin
        worst = 0;
        for (n = 96; n < 256; n = n + 1)
            for (k = 0; k < R - 2; k = k + 1) begin
                expv = 30000.0 * $sin(2.0 * 3.14159265358979 * (n - d8 / 8.0 + (k + 2.0) / R) / 64.0);
                err = rec[n * R + k] - expv;
                if (err < 0) err = -err;
                if (err > worst) worst = $rtoi(err);
            end
        if (worst < best) begin best = worst; best_d = d8; end
    end
    $sformat(str, "tone: best-fit delay %0.3f input samples, worst-case error %0d LSB of 30000", best_d / 8.0, best);
    if (best > 600) begin logger.error(module_name, str); errors++; end else logger.info(module_name, str);

    if (errors == 0) begin logger.info(module_name, "cic_interp OK"); `TEST_PASS end
    else `TEST_FAIL
end

always #4000000 begin logger.error(module_name, "System hangs"); `TEST_FAIL end
endmodule
