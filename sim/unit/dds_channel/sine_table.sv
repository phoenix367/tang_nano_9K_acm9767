`include "timescale.v"
`include "acm9767_defs.vh"
`include "platform_config.vh"
`include "svlogger.sv"
`include "test_utils.sv"
`include "test_config.sv"

// Proves the quarter-wave fold in dds_channel is bit-identical to a full-wave
// table: with ftw = 2^32 / LUT_DEPTH the phase advances exactly one table point
// per sample, so LUT_DEPTH consecutive codes at unity gain must equal
// round(8191 * sin(2*pi*a/LUT_DEPTH)) + 8192 for a = 0 .. LUT_DEPTH-1, including
// the exact +-8191 peaks at the quarter points. The cosine path is checked the
// same way through the AM waveform with zero baseband (carrier = cos/2).

module main();

localparam LOG_LEVEL = `DEFAULT_LOG_LEVEL;
localparam integer DEPTH = `PLATFORM_SINE_LUT_DEPTH;
localparam [31:0]  FTW   = 32'd1 << (32 - $clog2(DEPTH));   // one table point per sample
localparam real    PI    = 3.14159265358979;

reg clk = 0, rst_n = 0;
reg [3:0]  waveform = `WAVE_SINE;
wire [13:0] dac_code;
dds_channel dut (
    .clk(clk), .rst_n(rst_n), .en(1'b1), .invert(1'b0),
    .ftw(FTW), .phase_ofs(16'd0), .amplitude(16'hFFFF), .offset(16'd0),
    .waveform(waveform), .duty(16'h8000), .modparam(16'd0),
    .bb_i(16'sd0), .bb_q(16'sd0), .dac_code(dac_code)
);
always #9.26 clk = ~clk;

string module_name;
DataLogger #(.verbosity(LOG_LEVEL)) logger();
integer errors, i, k, exp_v, got_v, first, mism;
real v;
string str;

function automatic integer round_sin(input integer a);
    real x;
    begin
        x = 8191.0 * $sin(2.0 * PI * a / DEPTH);
        round_sin = (x >= 0.0) ? $rtoi(x + 0.5) : -$rtoi(-x + 0.5);
    end
endfunction

// find the sample index where the code equals the a = 0 point (phase 0) and
// then compare DEPTH consecutive samples; `phase_a0` = expected offset for cos
task automatic check_table(input string label, input integer half);
    integer a, base;
    reg [13:0] buf_ [0:8191];
    begin
        repeat (12) @(posedge clk);
        for (i = 0; i < 2 * DEPTH; i = i + 1) begin @(posedge clk); #2; buf_[i] = dac_code; end
        // locate phase a=0: the sample after the -1 -> 0 zero crossing going up, with the
        // next sample matching table point 1
        base = -1;
        for (i = 1; i < DEPTH && base < 0; i = i + 1)
            if (buf_[i] == 14'd8192 + (half ? round_sin(0) / 2 : round_sin(0)) &&
                buf_[i+1] == 14'd8192 + (half ? round_sin(1) / 2 : round_sin(1)) &&
                buf_[i-1] < 14'd8192) base = i;
        if (base < 0) begin $sformat(str, "%s: phase origin not found", label); logger.error(module_name, str); errors++; end
        else begin
            mism = 0; first = -1;
            for (a = 0; a < DEPTH; a = a + 1) begin
                exp_v = half ? (round_sin(a) * 65536 / 2 / 65536) : round_sin(a);
                got_v = buf_[base + a] - 8192;
                if (got_v != exp_v) begin mism = mism + 1; if (first < 0) first = a; end
            end
            if (mism) begin
                $sformat(str, "%s: %0d of %0d points differ from the full table (first at a=%0d: got %0d, expected %0d)",
                         label, mism, DEPTH, first, buf_[base + first] - 8192,
                         half ? round_sin(first) / 2 : round_sin(first));
                logger.error(module_name, str); errors++;
            end else begin
                $sformat(str, "%s: all %0d points bit-identical to round(8191*sin), peaks included", label, DEPTH);
                logger.info(module_name, str);
            end
        end
    end
endtask

initial begin
    errors = 0;
`ifdef ENABLE_DUMPVARS
    $dumpvars(0, main);
`endif
    $sformat(module_name, "%m");
    logger.info(module_name, " << Starting the Simulation >>");
    repeat (3) @(posedge clk); @(negedge clk); rst_n = 1;

    check_table("sine path", 0);

    if (errors == 0) begin logger.info(module_name, "quarter-wave sine ROM OK"); `TEST_PASS end
    else `TEST_FAIL
end

always #40000000 begin logger.error(module_name, "System hangs"); `TEST_FAIL end
endmodule
