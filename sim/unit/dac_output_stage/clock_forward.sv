`include "timescale.v"
`include "acm9767_defs.vh"
`include "svlogger.sv"
`include "test_utils.sv"
`include "test_config.sv"

// Unit test for dac_output_stage: data is registered (one clock late); with
// the default polarities CLK is a clk replica and WRT the inverted clk, so WRT
// rises half a period after the data changes and half a period after CLK
// (clear of the AD9767's forbidden 0..2 ns CLK-after-WRT window). A second
// instance with both inverted (the tied-together configuration) is checked too.

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
    .clk(clk), .rst_n(rst_n), .code(code), .da_clk(clk_rep), .da_wrt(wrt_inv), .da_data(d_inv)
);
// tied-together configuration: both inverted
dac_output_stage #(.CLK_INVERT(1'b1), .WRT_INVERT(1'b1)) dut_tied (
    .clk(clk), .rst_n(rst_n), .code(code), .da_clk(clk_inv), .da_wrt(wrt_rep), .da_data(d_rep)
);

always #9.26 clk = ~clk;

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
