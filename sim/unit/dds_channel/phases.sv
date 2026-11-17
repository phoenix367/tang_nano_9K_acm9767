`include "timescale.v"
`include "acm9767_defs.vh"
`include "platform_config.vh"
`include "svlogger.sv"
`include "test_utils.sv"
`include "test_config.sv"

// Proves the two-samples-per-clock dds_channel (PHASES = 2, the OSER4 build)
// produces exactly the sample stream of the one-sample-per-clock version
// (PHASES = 1) run at twice the clock: a PHASES=1 instance on clk2 (108 MHz)
// and a PHASES=2 instance on clk (54 MHz, phase-aligned like CLKDIV) get the
// same register values and a constant baseband; after enabling both on the
// same instant, their streams (lane 0 then lane 1 per clk) must match
// bit-for-bit at one fixed sample offset (the pipeline latencies differ).
// Covers every waveform (SSB/AM/FM with a constant baseband, where the
// two-sample hold of the baseband changes nothing), phase offset, gain,
// offset with saturation, invert, an FTW close to Nyquist, one above 2^31 (the
// 2*step add wraps) and FM with a deviation that makes the step wrap. Except
// for DC the match must be unique (a swapped lane pair would otherwise hide
// behind a one-sample offset). Then a disabled channel must park both lanes at
// mid-scale. Finally the SKEW correction (lane_skew): the serial product
// must equal ftw * skew / 65536 (truncated toward zero, 64-bit reference),
// the odd lane must lead the even one by exactly ftw + that, for positive,
// negative and extreme values, and skew 0 must restore the exact stream.

module main();

localparam LOG_LEVEL = `DEFAULT_LOG_LEVEL;
localparam integer N      = 512;     // samples compared per scenario
localparam integer MAXLAG = 40;      // sample offsets searched (+-)

reg clk = 0, clk2 = 0, rst_n = 0;
always #4.63 clk2 = ~clk2;           // 108 MHz
always #9.26 clk  = ~clk;            // 54 MHz, rising edges coincide with clk2's

reg en = 0, invert = 0;
reg [31:0] ftw = 32'h0123_4567;
reg [15:0] phase_ofs = 0, amplitude = 16'hFFFF, offset = 0, duty = 16'h8000, modparam = 0;
reg [3:0]  waveform = `WAVE_SINE;
reg signed [15:0] bb_i = 0, bb_q = 0;
reg [15:0] skew = 16'd0;

wire [13:0] code1;
wire [27:0] code2;
dds_channel #(.PHASES(1)) ref1 (
    .clk(clk2), .rst_n(rst_n), .en(en), .invert(invert),
    .ftw(ftw), .phase_ofs(phase_ofs), .amplitude(amplitude), .offset(offset),
    .waveform(waveform), .duty(duty), .modparam(modparam), .lane_skew(16'sd0), .bb_i(bb_i), .bb_q(bb_q), .dac_code(code1)
);
dds_channel #(.PHASES(2)) dut2 (
    .clk(clk), .rst_n(rst_n), .en(en), .invert(invert),
    .ftw(ftw), .phase_ofs(phase_ofs), .amplitude(amplitude), .offset(offset),
    .waveform(waveform), .duty(duty), .modparam(modparam), .lane_skew(skew), .bb_i(bb_i), .bb_q(bb_q), .dac_code(code2)
);

string module_name;
DataLogger #(.verbosity(LOG_LEVEL)) logger();
integer errors;
string str;

reg [13:0] ref_buf [0:N-1];
reg [13:0] dut_buf [0:N-1];

// capture N samples from both from the same instant (a clk rising edge)
task automatic capture;
    begin
        fork
            begin : cap1
                integer i;
                for (i = 0; i < N; i = i + 1) begin @(posedge clk2); #2; ref_buf[i] = code1; end
            end
            begin : cap2
                integer j;
                for (j = 0; j < N / 2; j = j + 1) begin
                    @(posedge clk); #2; dut_buf[2*j] = code2[13:0]; dut_buf[2*j+1] = code2[27:14];
                end
            end
        join
    end
endtask

// dut[i] == ref[i + d] over the middle of the capture, for some d in +-MAXLAG
task automatic scenario(input string label, input integer unique_req);
    integer d, i, found, nmatch, mism, distinct;
    begin
        en = 0;
        repeat (12) @(posedge clk);
        @(negedge clk); #1 en = 1;           // both instances see it on the same rising edge
        repeat (24) @(posedge clk);
        capture();
        found = -1000; nmatch = 0;
        for (d = -MAXLAG; d <= MAXLAG; d = d + 1) begin
            mism = 0;
            for (i = MAXLAG; i < N - MAXLAG; i = i + 1)
                if (dut_buf[i] !== ref_buf[i + d]) mism = mism + 1;
            if (mism == 0) begin nmatch = nmatch + 1; if (found == -1000) found = d; end
        end
        distinct = 0;
        for (i = 1; i < N; i = i + 1) if (ref_buf[i] != ref_buf[i-1]) distinct = distinct + 1;
        if (found == -1000) begin
            $sformat(str, "%s: two-lane stream differs from the one-lane stream at every offset (ref %h %h %h %h, dut %h %h %h %h)",
                     label, ref_buf[100], ref_buf[101], ref_buf[102], ref_buf[103],
                     dut_buf[100], dut_buf[101], dut_buf[102], dut_buf[103]);
            logger.error(module_name, str); errors = errors + 1;
        end else if (unique_req && nmatch != 1) begin
            $sformat(str, "%s: matches at %0d offsets (first %0d) -- scenario does not pin the lane order",
                     label, nmatch, found);
            logger.error(module_name, str); errors = errors + 1;
        end else begin
            $sformat(str, "%s: bit-identical at offset %0d samples (%0d code changes in %0d samples)",
                     label, found, distinct, N);
            logger.info(module_name, str);
        end
    end
endtask

// expected odd-lane correction: ftw * skew / 65536, truncated toward zero
function automatic [31:0] exp_delta(input [31:0] f, input [15:0] sk);
    reg [63:0] p; reg [15:0] mag;
    begin
        mag = sk[15] ? (16'd0 - sk) : sk;
        p = {32'd0, f} * {48'd0, mag};
        exp_delta = sk[15] ? (32'd0 - p[47:16]) : p[47:16];
    end
endfunction

task automatic skew_case(input [31:0] f, input [15:0] sk);
    reg [31:0] d, lead;
    begin
        ftw = f; skew = sk;
        repeat (30) @(posedge clk); #2;
        d = exp_delta(f, sk);
        lead = dut2.g_acc2.ph_o - dut2.g_acc2.ph_e;
        if (dut2.g_acc2.skew_delta !== d || lead !== f + d) begin
            $sformat(str, "skew %0d at ftw %h: delta %h (expected %h), odd lead %h (expected %h)",
                     $signed(sk), f, dut2.g_acc2.skew_delta, d, lead, f + d);
            logger.error(module_name, str); errors = errors + 1;
        end else begin
            $sformat(str, "skew %0d at ftw %h: odd lane leads by ftw + %0d", $signed(sk), f, $signed(d));
            logger.info(module_name, str);
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

    waveform = `WAVE_SINE; ftw = 32'h0123_4567; phase_ofs = 16'h1234; amplitude = 16'hC000; offset = 16'd100;
    scenario("sine, phase offset, gain 0.75, offset +100", 1);
    waveform = `WAVE_TRIANGLE; ftw = 32'h0345_6789; phase_ofs = 0; amplitude = 16'hFFFF; offset = 0;
    scenario("triangle", 1);
    waveform = `WAVE_SAWTOOTH; ftw = 32'h0567_89AB;
    scenario("sawtooth", 1);
    waveform = `WAVE_SQUARE; ftw = 32'h0278_9ABD; duty = 16'h5000;
    scenario("square, duty 0x5000", 1);
    waveform = `WAVE_SINE; ftw = 32'h7E54_3211; duty = 16'h8000; invert = 1; offset = -16'sd3000;
    scenario("sine at 0.494 f_dac, invert, offset -3000 (saturating)", 1);
    invert = 0; offset = 0; ftw = 32'h8123_4567; phase_ofs = 16'h4321;
    scenario("sine, FTW above 2^31 (2*step wraps)", 1);
    phase_ofs = 0;
    waveform = `WAVE_SSB_USB; ftw = 32'h0456_789B; bb_i = 16'sd12000; bb_q = -16'sd7000;
    scenario("SSB USB, constant baseband", 1);
    waveform = `WAVE_SSB_LSB;
    scenario("SSB LSB, constant baseband", 1);
    waveform = `WAVE_AM; bb_i = 16'sd9000; bb_q = 0; modparam = 16'hC000;
    scenario("AM, depth 0.75, constant baseband", 1);
    waveform = `WAVE_FM; bb_i = -16'sd12000; modparam = 16'd3106;
    scenario("FM, constant deviation", 1);
    waveform = `WAVE_FM; ftw = 32'h7FF0_0000; bb_i = 16'sd32767; modparam = 16'hFFFF;
    scenario("FM, step wrapping past 2^31", 1);
    waveform = `WAVE_DC; ftw = 32'h0123_4567; offset = 16'd1234; bb_i = 0; modparam = 0;
    scenario("DC", 0);

    // SKEW correction
    waveform = `WAVE_SINE; offset = 0; bb_i = 0; modparam = 0; en = 1;
    skew_case(32'h0123_4567, 16'd1130);
    skew_case(32'h7E54_3211, -16'sd1130);
    skew_case(32'hFFFF_FFFF, 16'h7FFF);
    skew_case(32'h8000_0000, 16'h8000);
    skew_case(32'h0000_0001, 16'hFFFF);
    skew = 0;
    waveform = `WAVE_SSB_USB; ftw = 32'h0456_789B; bb_i = 16'sd12000; bb_q = -16'sd7000;
    scenario("SSB USB after skew back to 0", 1);

    // disabled: both lanes at mid-scale
    en = 0;
    repeat (12) @(posedge clk); #2;
    if (code2 !== {14'h2000, 14'h2000}) begin
        $sformat(str, "disabled: lanes %h %h, expected 2000 2000", code2[27:14], code2[13:0]);
        logger.error(module_name, str); errors = errors + 1;
    end else logger.info(module_name, "disabled channel parks both lanes at mid-scale");

    if (errors == 0) begin logger.info(module_name, "two-lane dds_channel == one-lane at 2x clock"); `TEST_PASS end
    else `TEST_FAIL
end

always #2000000 begin logger.error(module_name, "System hangs"); `TEST_FAIL end
endmodule
