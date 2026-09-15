`include "timescale.v"
`include "acm9767_defs.vh"
`include "svlogger.sv"
`include "test_utils.sv"
`include "test_config.sv"

// Unit test for dds_channel. FTW = 2^32/64 gives exactly 64 samples per period,
// so every waveform can be checked sample-by-sample against its definition
// after the 5-clock pipeline settles.
//
// Checks: disabled -> mid-scale 0x2000; sine period (mid-scale crossings over
// 10 periods) and full-scale peaks; triangle monotonic up/down halves;
// sawtooth monotonic with one wrap; square duty; amplitude halves the sine
// swing; DC offset with saturation on a square; invert flips every bit; DC
// waveform is offset only.

module main();

localparam LOG_LEVEL = `DEFAULT_LOG_LEVEL;
localparam integer PERIOD = 64;
localparam [31:0] FTW = 32'h0400_0000;   // 2^32 / 64

reg clk = 0, rst_n = 0;
reg en = 0, invert = 0;
reg [31:0] ftw = FTW;
reg [15:0] phase_ofs = 0, amplitude = 16'hFFFF, offset = 0, duty = 16'h8000;
reg [3:0]  waveform = `WAVE_SINE;
reg [15:0] modparam = 16'd0;
reg signed [15:0] bb_i = 16'sd0;
wire [13:0] dac_code;

string module_name;
DataLogger #(.verbosity(LOG_LEVEL)) logger();

dds_channel dut (
    .clk(clk), .rst_n(rst_n), .en(en), .invert(invert),
    .ftw(ftw), .phase_ofs(phase_ofs), .amplitude(amplitude), .offset(offset),
    .waveform(waveform), .duty(duty), .modparam(modparam), .bb_i(bb_i), .bb_q(16'sd0), .dac_code(dac_code)
);

always #9.26 clk = ~clk;   // 54 MHz

integer errors;
string str;
reg [13:0] buf_ [0:PERIOD*10-1];

// capture N consecutive samples after the pipeline settled
task automatic capture(input integer n);
    integer i;
    begin
        repeat (16) @(posedge clk);      // pipeline: up to ~10 clocks for the modulated waveforms
        for (i = 0; i < n; i = i + 1) begin
            @(posedge clk); #2;
            buf_[i] = dac_code;
        end
    end
endtask

// rising mid-scale crossings over n samples (after the pipeline settles)
task automatic count_periods(input integer n, output integer c);
    integer j; reg [13:0] prev, cur;
    begin
        c = 0;
        repeat (8) @(posedge clk);
        @(posedge clk); #2; prev = dac_code;
        for (j = 0; j < n; j = j + 1) begin
            @(posedge clk); #2; cur = dac_code;
            if (prev < 14'h2000 && cur >= 14'h2000) c = c + 1;
            prev = cur;
        end
    end
endtask

function automatic integer as_signed(input [13:0] code);
    begin as_signed = code - 8192; end
endfunction

task automatic fail(input string s);
    begin logger.error(module_name, s); errors = errors + 1; end
endtask

integer i, cnt, mx, mn, v, prev, wraps;

initial begin
    errors = 0;
`ifdef ENABLE_DUMPVARS
    $dumpvars(0, main);
`endif
    $sformat(module_name, "%m");
    logger.info(module_name, " << Starting the Simulation >>");

    repeat (3) @(posedge clk);
    @(negedge clk); rst_n = 1;

    // ---- disabled: mid-scale ----
    capture(16);
    for (i = 0; i < 16; i = i + 1) if (buf_[i] !== 14'h2000) begin fail("disabled output is not mid-scale"); i = 16; end

    // ---- sine: period + peaks ----
    en = 1; waveform = `WAVE_SINE;
    capture(PERIOD * 10);
    cnt = 0; mx = -9000; mn = 9000;
    for (i = 1; i < PERIOD * 10; i = i + 1) begin
        v = as_signed(buf_[i]);
        if (v > mx) mx = v;
        if (v < mn) mn = v;
        if (as_signed(buf_[i-1]) < 0 && v >= 0) cnt = cnt + 1;   // rising mid-scale crossing
    end
    if (cnt < 9 || cnt > 10) begin $sformat(str, "sine: %0d rising crossings in 10 periods", cnt); fail(str); end
    if (mx < 8180 || mx > 8191 || mn > -8180 || mn < -8192) begin $sformat(str, "sine peaks %0d..%0d", mn, mx); fail(str); end
    logger.info(module_name, "sine period and peaks OK");

    // ---- phase offset of a quarter turn turns the sine into a cosine ----
    phase_ofs = 16'h4000;
    capture(PERIOD);
    // the accumulator keeps running, so just check a full-scale peak still exists
    mx = -9000;
    for (i = 0; i < PERIOD; i = i + 1) if (as_signed(buf_[i]) > mx) mx = as_signed(buf_[i]);
    if (mx < 8180) begin $sformat(str, "sine with phase offset peak %0d", mx); fail(str); end
    phase_ofs = 0;

    // ---- triangle ----
    waveform = `WAVE_TRIANGLE;
    capture(PERIOD * 2);
    // locate the minimum (start of the rising half) and check monotonicity from there
    mn = 9000; cnt = 0;
    for (i = 0; i < PERIOD; i = i + 1) if (as_signed(buf_[i]) < mn) begin mn = as_signed(buf_[i]); cnt = i; end
    for (i = 1; i < PERIOD / 2; i = i + 1)
        if (as_signed(buf_[cnt + i]) <= as_signed(buf_[cnt + i - 1])) begin fail("triangle rising half not monotonic"); i = PERIOD; end
    for (i = PERIOD / 2 + 1; i < PERIOD; i = i + 1)
        if (as_signed(buf_[cnt + i]) >= as_signed(buf_[cnt + i - 1])) begin fail("triangle falling half not monotonic"); i = PERIOD; end
    mx = as_signed(buf_[cnt + PERIOD / 2]);
    if (mn > -8100 || mx < 8100) begin $sformat(str, "triangle peaks %0d..%0d", mn, mx); fail(str); end
    // continuity: 64 samples/period -> every step is +-512 LSB (no seam at the peaks)
    for (i = 1; i < PERIOD * 2; i = i + 1) begin
        v = as_signed(buf_[i]) - as_signed(buf_[i-1]);
        if (v > 520 || v < -520) begin $sformat(str, "triangle step %0d at sample %0d", v, i); fail(str); i = PERIOD * 2; end
    end
    logger.info(module_name, "triangle OK");

    // ---- sawtooth ----
    waveform = `WAVE_SAWTOOTH;
    capture(PERIOD * 2);
    wraps = 0;
    for (i = 1; i < PERIOD * 2; i = i + 1) begin
        v = as_signed(buf_[i]) - as_signed(buf_[i-1]);
        if (v < 0) wraps = wraps + 1;
        else if (v != 256) begin $sformat(str, "sawtooth step %0d at sample %0d, expected 256", v, i); fail(str); i = PERIOD * 2; end
    end
    if (wraps != 2) begin $sformat(str, "sawtooth: %0d wraps in 2 periods, expected 2", wraps); fail(str); end
    mx = -9000; mn = 9000;
    for (i = 0; i < PERIOD * 2; i = i + 1) begin
        if (as_signed(buf_[i]) > mx) mx = as_signed(buf_[i]);
        if (as_signed(buf_[i]) < mn) mn = as_signed(buf_[i]);
    end
    // 64 samples/period: last sample before the wrap is 8192 - 256
    if (mn != -8192 || mx != 8192 - 256) begin $sformat(str, "sawtooth range %0d..%0d, expected -8192..7936", mn, mx); fail(str); end
    logger.info(module_name, "sawtooth OK");

    // ---- square, 25 % duty ----
    waveform = `WAVE_SQUARE; duty = 16'h4000;
    capture(PERIOD * 2);
    cnt = 0;
    for (i = 0; i < PERIOD * 2; i = i + 1) begin
        v = as_signed(buf_[i]);
        if (v == 8191) cnt = cnt + 1;
        else if (v != -8192) begin $sformat(str, "square sample %0d = %0d", i, v); fail(str); i = PERIOD * 2; end
    end
    if (cnt != PERIOD / 2) begin $sformat(str, "square: %0d high samples in 2 periods, expected %0d", cnt, PERIOD / 2); fail(str); end
    duty = 16'h8000;
    logger.info(module_name, "square duty OK");

    // ---- amplitude 0x7FFF = gain 0.5 halves the sine ----
    waveform = `WAVE_SINE; amplitude = 16'h7FFF;
    capture(PERIOD);
    mx = -9000;
    for (i = 0; i < PERIOD; i = i + 1) if (as_signed(buf_[i]) > mx) mx = as_signed(buf_[i]);
    if (mx < 4093 || mx > 4096) begin $sformat(str, "half amplitude peak %0d, expected ~4095", mx); fail(str); end
    amplitude = 16'hFFFF;

    // ---- offset with saturation (square) ----
    waveform = `WAVE_SQUARE; offset = 16'd1000;
    capture(PERIOD);
    mx = -9000; mn = 9000;
    for (i = 0; i < PERIOD; i = i + 1) begin
        if (as_signed(buf_[i]) > mx) mx = as_signed(buf_[i]);
        if (as_signed(buf_[i]) < mn) mn = as_signed(buf_[i]);
    end
    if (mx != 8191 || mn != -7192) begin $sformat(str, "offset +1000: %0d..%0d, expected -7192..8191", mn, mx); fail(str); end
    offset = -16'd1000;
    capture(PERIOD);
    mx = -9000; mn = 9000;
    for (i = 0; i < PERIOD; i = i + 1) begin
        if (as_signed(buf_[i]) > mx) mx = as_signed(buf_[i]);
        if (as_signed(buf_[i]) < mn) mn = as_signed(buf_[i]);
    end
    if (mx != 7191 || mn != -8192) begin $sformat(str, "offset -1000: %0d..%0d, expected -8192..7191", mn, mx); fail(str); end
    logger.info(module_name, "offset + saturation OK");

    // ---- DC waveform = offset only ----
    waveform = `WAVE_DC; offset = 16'd512;
    capture(8);
    for (i = 0; i < 8; i = i + 1) if (buf_[i] !== 14'h2000 + 14'd512) begin fail("DC waveform is not offset only"); i = 8; end
    offset = 0;

    // ---- invert flips every bit ----
    waveform = `WAVE_SQUARE; invert = 1;
    capture(PERIOD);
    // expected codes: +8191 -> 0x3FFF ^ 0x3FFF = 0x0000 ; -8192 -> 0x0000 ^ 0x3FFF = 0x3FFF
    for (i = 0; i < PERIOD; i = i + 1)
        if (buf_[i] !== 14'h0000 && buf_[i] !== 14'h3FFF) begin $sformat(str, "inverted square code %h", buf_[i]); fail(str); i = PERIOD; end
    invert = 0;

    // ---- AM: envelope follows bb_i; depth 100 % ----
    waveform = `WAVE_AM; modparam = 16'hFFFF; bb_i = 16'sd0;
    capture(PERIOD);
    mx = -9000;
    for (i = 0; i < PERIOD; i = i + 1) if (as_signed(buf_[i]) > mx) mx = as_signed(buf_[i]);
    if (mx < 4085 || mx > 4100) begin $sformat(str, "AM unmodulated carrier peak %0d, expected ~4095", mx); fail(str); end
    bb_i = 16'sd32767;
    capture(PERIOD);
    mx = -9000;
    for (i = 0; i < PERIOD; i = i + 1) if (as_signed(buf_[i]) > mx) mx = as_signed(buf_[i]);
    if (mx < 8170 || mx > 8191) begin $sformat(str, "AM 100 %% peak %0d, expected ~8190", mx); fail(str); end
    bb_i = -16'sd32768;
    capture(PERIOD);
    mx = -9000;
    for (i = 0; i < PERIOD; i = i + 1) begin v = as_signed(buf_[i]); if (v < 0) v = -v; if (v > mx) mx = v; end
    if (mx > 4) begin $sformat(str, "AM trough |peak| %0d, expected ~0", mx); fail(str); end
    modparam = 16'h8000; bb_i = 16'sd32767;             // 50 % depth
    capture(PERIOD);
    mx = -9000;
    for (i = 0; i < PERIOD; i = i + 1) if (as_signed(buf_[i]) > mx) mx = as_signed(buf_[i]);
    if (mx < 6120 || mx > 6160) begin $sformat(str, "AM 50 %% peak %0d, expected ~6143", mx); fail(str); end
    logger.info(module_name, "AM envelope OK");

    // ---- FM: a DC baseband shifts the frequency by modparam*256*bb_i/32768 FTW ----
    // ftw = 2^32/64 (64 samples/period); modparam 0x8000 -> deviation 2^23 = fs/512
    // -> 9/512 fs: 90 periods in 5120 samples instead of 80
    waveform = `WAVE_FM; modparam = 16'h8000; bb_i = 16'sd32767;
    count_periods(5120, cnt);
    if (cnt < 89 || cnt > 91) begin $sformat(str, "FM +dev: %0d periods in 5120 samples, expected 90", cnt); fail(str); end
    bb_i = -16'sd32768;
    count_periods(5120, cnt);
    if (cnt < 69 || cnt > 71) begin $sformat(str, "FM -dev: %0d periods in 5120 samples, expected 70", cnt); fail(str); end
    bb_i = 16'sd0;
    count_periods(5120, cnt);
    if (cnt < 79 || cnt > 81) begin $sformat(str, "FM no dev: %0d periods in 5120 samples, expected 80", cnt); fail(str); end
    logger.info(module_name, "FM deviation OK");
    waveform = `WAVE_SINE; modparam = 0;

    // ---- disabling returns to mid-scale and resets the phase ----
    en = 0;
    capture(8);
    for (i = 0; i < 8; i = i + 1) if (buf_[i] !== 14'h2000) begin fail("disable did not return to mid-scale"); i = 8; end

    if (errors == 0) begin
        logger.info(module_name, "dds_channel waveforms OK");
        `TEST_PASS
    end else
        `TEST_FAIL
end

always #2000000 begin
    logger.error(module_name, "System hangs");
    `TEST_FAIL
end

endmodule
