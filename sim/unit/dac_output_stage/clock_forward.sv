`include "timescale.v"
`include "acm9767_defs.vh"
`include "svlogger.sv"
`include "test_utils.sv"
`include "test_config.sv"

// Unit test for dac_output_stage.
// PHASES = 1 (ODDR): data is registered (one clock late); with the default
// polarities CLK is a clk replica and WRT the inverted clk, so WRT rises half a
// period after the data changes and half a period after CLK (clear of the
// AD9767's forbidden 0..2 ns CLK-after-WRT window). A second instance with both
// inverted (the tied-together configuration) is checked too.
// PHASES = 2 (OSER4, Gowin simulation models, fclk 108 MHz, clk = CLKDIV /2):
// the samples latched on the WRT rising edges must be lane 0, lane 1, lane 0,
// ... of consecutive clk words in order; CLK and WRT are complementary
// 108 MHz clocks with CLK rising half a sample before WRT; the data is stable
// for >= 4 ns either side of every WRT rising edge.

module main();

localparam LOG_LEVEL = `DEFAULT_LOG_LEVEL;

reg clk = 0, rst_n = 0;
reg [13:0] code = 0;
wire clk_inv, wrt_inv, clk_rep, wrt_rep;
wire [13:0] d_inv, d_rep;

string module_name;
DataLogger #(.verbosity(LOG_LEVEL)) logger();

// default: CLK replica, WRT inverted
dac_output_stage dut_def (
    .clk(clk), .fclk(1'b0), .rst_n(rst_n), .code(code), .da_clk(clk_rep), .da_wrt(wrt_inv), .da_data(d_inv)
);
// tied-together configuration: both inverted
dac_output_stage #(.CLK_INVERT(1'b1), .WRT_INVERT(1'b1)) dut_tied (
    .clk(clk), .fclk(1'b0), .rst_n(rst_n), .code(code), .da_clk(clk_inv), .da_wrt(wrt_rep), .da_data(d_rep)
);

always #9.26 clk = ~clk;

// ---- PHASES = 2: OSER4 serializers, 108 MHz fclk, CLKDIV /2 ----
reg fclk = 0, rst2_n = 0;
always #4.63 fclk = ~fclk;
wire pclk;
CLKDIV #(.DIV_MODE("2"), .GSREN("false")) cdiv (.HCLKIN(fclk), .RESETN(1'b1), .CALIB(1'b1), .CLKOUT(pclk));
reg  [27:0] code2 = {14'h2000, 14'h2000};
wire da2_clk, da2_wrt;
wire [13:0] da2_data;
dac_output_stage #(.PHASES(2)) dut_ser (
    .clk(pclk), .fclk(fclk), .rst_n(rst2_n), .code(code2), .da_clk(da2_clk), .da_wrt(da2_wrt), .da_data(da2_data)
);
// word j carries lane 0 = 2j, lane 1 = 2j+1 (as 14-bit sample numbers + 0x100)
integer wj = 0;
reg [13:0] l0;
always @(posedge pclk) if (rst2_n) begin
    l0 = 14'h100 + 2 * wj;
    code2 <= #1 {l0 + 14'd1, l0};
    wj = wj + 1;
end
realtime t_dchg = 0, t_wrt_rise = 0;
reg [13:0] lat [0:63];
integer n_lat = 0, n_setup_bad = 0, n_hold_bad = 0;
always @(da2_data) begin
    if (n_lat > 0 && $realtime - t_wrt_rise < 4.0) n_hold_bad = n_hold_bad + 1;
    t_dchg = $realtime;
end
always @(posedge da2_wrt) if (rst2_n && n_lat < 64) begin
    if ($realtime - t_dchg < 4.0) n_setup_bad = n_setup_bad + 1;
    t_wrt_rise = $realtime;
    lat[n_lat] = da2_data;
    n_lat = n_lat + 1;
end

integer errors, i;
string str;
realtime t_clk, t_wrt;

task automatic fail(input string s);
    begin logger.error(module_name, s); errors = errors + 1; end
endtask

initial begin
    errors = 0;
`ifdef ENABLE_DUMPVARS
    $dumpvars(0, main);
`endif
    $sformat(module_name, "%m");
    logger.info(module_name, " << Starting the Simulation >>");

    repeat (3) @(posedge clk);
    rst_n = 1;
    repeat (6) @(posedge clk);   // let the ODDR pipelines fill

    // polarities, sampled just after each edge: default CLK = clk, WRT = ~clk;
    // tied configuration CLK = WRT = ~clk
    for (i = 0; i < 20; i = i + 1) begin
        @(posedge clk); #2;
        if (clk_rep !== 1'b1 || wrt_inv !== 1'b0) begin fail("default: CLK low / WRT high after posedge"); i = 20; end
        if (clk_inv !== 1'b0 || wrt_rep !== 1'b0) begin fail("tied: CLK/WRT high after posedge"); i = 20; end
        @(negedge clk); #2;
        if (clk_rep !== 1'b0 || wrt_inv !== 1'b1) begin fail("default: CLK high / WRT low after negedge"); i = 20; end
        if (clk_inv !== 1'b1 || wrt_rep !== 1'b1) begin fail("tied: CLK/WRT low after negedge"); i = 20; end
    end
    if (errors == 0) logger.info(module_name, "forwarded CLK/WRT polarities OK");

    // default: every WRT rising edge must be preceded by a CLK rising edge half a period earlier
    for (i = 0; i < 10; i = i + 1) begin
        @(posedge clk_rep); t_clk = $realtime;
        @(posedge wrt_inv); t_wrt = $realtime;
        if (t_wrt - t_clk < 9.0 || t_wrt - t_clk > 9.6) begin
            $sformat(str, "WRT rises %0.2f ns after CLK, expected ~9.26", t_wrt - t_clk); fail(str); i = 10;
        end
    end
    if (errors == 0) logger.info(module_name, "CLK leads WRT by half a period");

    // data registered one clock after the code changes
    @(negedge clk); code = 14'h1234;
    @(posedge clk); #2;
    if (d_inv !== 14'h1234 || d_rep !== 14'h1234) begin
        $sformat(str, "data after one clock: %h %h", d_inv, d_rep); fail(str);
    end

    // data must be stable across the DAC's input-latch edge (WRT rising = clk negedge)
    @(negedge clk); code = 14'h2AAA;
    @(posedge clk); #2;            // data changes here
    @(negedge clk); #1;            // WRT rises here
    if (d_inv !== 14'h2AAA) fail("data changed around the WRT latch edge");

    // ---- PHASES = 2 ----
    @(posedge pclk); #1 rst2_n = 1;
    wait (n_lat == 64);
    // find the first serialized sample of the counter sequence, then demand
    // consecutive sample numbers (lane order + no dropped/duplicated words)
    begin : ser_check
        integer k, s0;
        s0 = -1;
        for (k = 0; k < 64 && s0 < 0; k = k + 1) if (lat[k] >= 14'h100 && lat[k] < 14'h2000) s0 = k;
        if (s0 < 0 || s0 > 16) fail("PHASES=2: serialized samples never appeared");
        else begin
            for (k = s0 + 1; k < 64; k = k + 1)
                if (lat[k] !== lat[k-1] + 14'd1) begin
                    $sformat(str, "PHASES=2: latched sample %0d = %h after %h (expected consecutive)", k, lat[k], lat[k-1]);
                    fail(str); k = 64;
                end
            if (errors == 0) begin
                $sformat(str, "PHASES=2: %0d samples latched on WRT in lane order (first %h)", 64 - s0, lat[s0]);
                logger.info(module_name, str);
            end
        end
    end
    if (n_setup_bad || n_hold_bad) begin
        $sformat(str, "PHASES=2: data changed < 4 ns before (%0d) / after (%0d) a WRT rising edge", n_setup_bad, n_hold_bad);
        fail(str);
    end else logger.info(module_name, "PHASES=2: data stable >= 4 ns around every WRT rising edge");
    for (i = 0; i < 40; i = i + 1) begin
        @(fclk); #1;
        if (da2_wrt !== ~da2_clk) begin fail("PHASES=2: WRT is not the complement of CLK"); i = 40; end
    end
    for (i = 0; i < 10; i = i + 1) begin
        @(posedge da2_clk); t_clk = $realtime;
        @(posedge da2_wrt); t_wrt = $realtime;
        if (t_wrt - t_clk < 4.4 || t_wrt - t_clk > 4.9) begin
            $sformat(str, "PHASES=2: WRT rises %0.2f ns after CLK, expected ~4.63", t_wrt - t_clk); fail(str); i = 10;
        end
        @(posedge da2_wrt);
        if ($realtime - t_wrt < 9.1 || $realtime - t_wrt > 9.4) begin
            $sformat(str, "PHASES=2: WRT period %0.2f ns, expected 9.26", $realtime - t_wrt); fail(str); i = 10;
        end
    end
    if (errors == 0) logger.info(module_name, "PHASES=2: 108 MHz CLK leads WRT by half a sample");

    if (errors == 0) begin
        logger.info(module_name, "dac_output_stage OK");
        `TEST_PASS
    end else
        `TEST_FAIL
end

always #100000 begin
    logger.error(module_name, "System hangs");
    `TEST_FAIL
end

endmodule
