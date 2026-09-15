`include "timescale.v"
`include "acm9767_defs.vh"
`include "svlogger.sv"
`include "test_utils.sv"
`include "test_config.sv"

// Unit test for async_fifo (16 deep): write clock 27 MHz, read clock 54 MHz.
// Order and data integrity across the clocks, wlevel tracking, wfull at 16
// entries with the extra write dropped, rempty after draining.

module main();

localparam LOG_LEVEL = `DEFAULT_LOG_LEVEL;
localparam integer AW = 4;

reg wclk = 0, rclk = 0, wrst_n = 0, rrst_n = 0;
reg wr_en = 0, rd_en = 0;
reg [15:0] wdata = 0;
wire wfull, rempty;
wire [AW:0] wlevel;
wire [15:0] rdata;

string module_name;
DataLogger #(.verbosity(LOG_LEVEL)) logger();

async_fifo #(.WIDTH(16), .DEPTH_LOG2(AW)) dut (
    .wclk(wclk), .wrst_n(wrst_n), .wr_en(wr_en), .wdata(wdata), .wfull(wfull), .wlevel(wlevel),
    .rclk(rclk), .rrst_n(rrst_n), .rd_en(rd_en), .rdata(rdata), .rempty(rempty)
);

always #18.5 wclk = ~wclk;
always #9.26 rclk = ~rclk;

integer errors, i, n;
string str;

task automatic push(input [15:0] v);
    begin @(negedge wclk); wr_en = 1; wdata = v; @(negedge wclk); wr_en = 0; end
endtask
task automatic pop(output [15:0] v);
    begin
        @(negedge rclk); rd_en = 1; @(negedge rclk); rd_en = 0;
        @(posedge rclk); #2; v = rdata;
    end
endtask

reg [15:0] v;

initial begin
    errors = 0;
`ifdef ENABLE_DUMPVARS
    $dumpvars(0, main);
`endif
    $sformat(module_name, "%m");
    logger.info(module_name, " << Starting the Simulation >>");
    repeat (3) @(posedge wclk);
    wrst_n = 1; rrst_n = 1;
    repeat (3) @(posedge wclk);

    if (!rempty || wlevel != 0) begin logger.error(module_name, "not empty after reset"); errors++; end

    for (i = 0; i < 10; i = i + 1) push(16'h1000 + i);
    repeat (6) @(posedge wclk); #2;
    if (wlevel != 10) begin $sformat(str, "wlevel %0d after 10 pushes", wlevel); logger.error(module_name, str); errors++; end
    if (rempty) begin logger.error(module_name, "rempty with 10 entries"); errors++; end

    for (i = 0; i < 10; i = i + 1) begin
        pop(v);
        if (v !== 16'h1000 + i) begin $sformat(str, "pop %0d = %h", i, v); logger.error(module_name, str); errors++; end
    end
    repeat (4) @(posedge rclk); #2;
    if (!rempty) begin logger.error(module_name, "not empty after draining"); errors++; end
    repeat (6) @(posedge wclk); #2;
    if (wlevel != 0) begin $sformat(str, "wlevel %0d after drain", wlevel); logger.error(module_name, str); errors++; end

    // fill to full; the 17th write must be dropped
    for (i = 0; i < 17; i = i + 1) push(16'h2000 + i);
    repeat (3) @(posedge wclk); #2;
    if (!wfull || wlevel != 16) begin $sformat(str, "full: wfull=%0b wlevel=%0d", wfull, wlevel); logger.error(module_name, str); errors++; end
    for (i = 0; i < 16; i = i + 1) begin
        pop(v);
        if (v !== 16'h2000 + i) begin $sformat(str, "full-pop %0d = %h", i, v); logger.error(module_name, str); errors++; end
    end
    repeat (4) @(posedge rclk); #2;
    if (!rempty) begin logger.error(module_name, "17th write was not dropped"); errors++; end

    if (errors == 0) begin logger.info(module_name, "async_fifo OK"); `TEST_PASS end
    else `TEST_FAIL
end

always #200000 begin logger.error(module_name, "System hangs"); `TEST_FAIL end
endmodule
