`include "timescale.v"
`include "svlogger.sv"
`include "test_utils.sv"
`include "test_config.sv"

// Unit test for cdc_commit_sync: a 32-bit word crossing from a slow sending
// clock (period 20) to a fast receiving clock (period 7, deliberately not an
// integer ratio). Checks: the word arrives intact with a single r_load pulse,
// s_busy covers the whole round trip, a commit issued while busy is ignored,
// and back-to-back commits after busy drops each land.

module main();

localparam LOG_LEVEL = `DEFAULT_LOG_LEVEL;
localparam integer W = 32;

reg s_clk = 0, r_clk = 0;
reg s_rst_n = 0, r_rst_n = 0;
reg s_commit = 0;
reg [W-1:0] s_data = 0;
wire s_busy;
wire [W-1:0] r_data;
wire r_load;

string module_name;
DataLogger #(.verbosity(LOG_LEVEL)) logger();

cdc_commit_sync #(.WORD_WIDTH(W)) dut (
    .s_clk(s_clk), .s_rst_n(s_rst_n), .s_commit(s_commit), .s_data(s_data), .s_busy(s_busy),
    .r_clk(r_clk), .r_rst_n(r_rst_n), .r_data(r_data), .r_load(r_load)
);

always #10  s_clk = ~s_clk;
always #3.5 r_clk = ~r_clk;

integer loads;
always @(posedge r_clk) if (r_load) loads = loads + 1;

integer errors;
string str;

task automatic commit(input [W-1:0] word);
    begin
        @(negedge s_clk); s_data = word; s_commit = 1'b1;
        @(negedge s_clk); s_commit = 1'b0;
    end
endtask

task automatic wait_idle;
    integer n;
    begin
        n = 0;
        while (s_busy && n < 100) begin @(posedge s_clk); #2; n = n + 1; end
    end
endtask

initial begin
    errors = 0; loads = 0;
`ifdef ENABLE_DUMPVARS
    $dumpvars(0, main);
`endif
    $sformat(module_name, "%m");
    logger.info(module_name, " << Starting the Simulation >>");

    repeat (3) @(posedge s_clk);
    s_rst_n = 1; r_rst_n = 1;
    repeat (2) @(posedge s_clk);

    // 1) single commit
    commit(32'hCAFE_0001);
    @(posedge s_clk); #2;
    if (!s_busy) begin logger.error(module_name, "s_busy did not assert after commit"); errors++; end
    wait_idle();
    if (s_busy) begin logger.error(module_name, "s_busy stuck high"); errors++; end
    if (r_data !== 32'hCAFE_0001) begin
        $sformat(str, "r_data = %h, expected CAFE0001", r_data); logger.error(module_name, str); errors++;
    end
    if (loads != 1) begin
        $sformat(str, "r_load pulsed %0d times, expected 1", loads); logger.error(module_name, str); errors++;
    end

    // 2) commit while busy is dropped (second word must not land)
    commit(32'h0000_0002);
    @(negedge s_clk); s_data = 32'h0000_0003; s_commit = 1'b1;   // still busy here
    @(negedge s_clk); s_commit = 1'b0;
    wait_idle();
    if (r_data !== 32'h0000_0002) begin
        $sformat(str, "r_data = %h after busy commit, expected 00000002", r_data);
        logger.error(module_name, str); errors++;
    end
    if (loads != 2) begin
        $sformat(str, "loads = %0d, expected 2", loads); logger.error(module_name, str); errors++;
    end

    // 3) back-to-back commits, each waiting for idle
    commit(32'h1111_1111); wait_idle();
    commit(32'h2222_2222); wait_idle();
    if (r_data !== 32'h2222_2222 || loads != 4) begin
        $sformat(str, "back-to-back: r_data=%h loads=%0d", r_data, loads);
        logger.error(module_name, str); errors++;
    end

    if (errors == 0) begin
        logger.info(module_name, "cdc_commit_sync handshake OK");
        `TEST_PASS
    end else
        `TEST_FAIL
end

always #200000 begin
    logger.error(module_name, "System hangs");
    `TEST_FAIL
end

endmodule
