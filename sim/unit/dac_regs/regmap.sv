`include "timescale.v"
`include "acm9767_defs.vh"
`include "platform_config.vh"
`include "svlogger.sv"
`include "test_utils.sv"
`include "test_config.sv"

// Unit test for dac_regs through the be_* backend handshake (driven the way
// modbus_rtu_slave drives it: be_req held until be_ready).
//
// Covers: read-only identity/clock registers, reset defaults, staged writes and
// read-back for both channels, unmapped addresses reading 0, COMMIT packing the
// staged bank into commit_word with one commit_pulse, commit_busy while the CDC
// reports busy, and STATUS bit layout.

module main();

localparam LOG_LEVEL = `DEFAULT_LOG_LEVEL;

reg clk = 0, rst_n = 0;
reg be_req = 0, be_we = 0;
reg [15:0] be_addr = 0, be_wdata = 0;
wire be_ready;
wire [15:0] be_rdata;
reg pll_lock = 0, cdc_busy = 0;
reg [`PLATFORM_SSB_FIFO_LOG2:0] fifo_level = 0;
reg underrun_tog = 0;
wire fifo_wr; wire [15:0] fifo_wdata;
integer pushes; reg [15:0] last_push;
always @(posedge clk) if (fifo_wr) begin pushes = pushes + 1; last_push = fifo_wdata; end
wire [`COMMIT_WORD_W-1:0] commit_word;
wire commit_pulse, commit_busy;
wire [`CTRL_WORD_W-1:0] ctrl_committed;

string module_name;
DataLogger #(.verbosity(LOG_LEVEL)) logger();

dac_regs dut (
    .clk(clk), .rst_n(rst_n),
    .be_req(be_req), .be_we(be_we), .be_addr(be_addr), .be_wdata(be_wdata),
    .be_ready(be_ready), .be_rdata(be_rdata),
    .pll_lock(pll_lock), .cdc_busy(cdc_busy),
    .fifo_wr(fifo_wr), .fifo_wdata(fifo_wdata), .fifo_level(fifo_level), .underrun_tog(underrun_tog),
    .commit_word(commit_word), .commit_pulse(commit_pulse), .commit_busy(commit_busy),
    .ctrl_committed(ctrl_committed)
);

always #18.5 clk = ~clk;

integer pulses;
always @(posedge clk) if (commit_pulse) pulses = pulses + 1;

integer errors;
string str;

// one backend access, modbus_rtu_slave style
task automatic access(input we, input [15:0] addr, input [15:0] wdata, output [15:0] rdata);
    integer n;
    begin
        @(negedge clk); be_req = 1; be_we = we; be_addr = addr; be_wdata = wdata;
        n = 0;
        @(posedge clk); #2;
        while (!be_ready && n < 10) begin @(posedge clk); #2; n = n + 1; end
        if (!be_ready) begin logger.error(module_name, "be_ready never came"); errors++; end
        rdata = be_rdata;
        @(negedge clk); be_req = 0; be_we = 0;
        @(posedge clk); #2;
        if (be_ready) begin logger.error(module_name, "be_ready lasted more than one cycle"); errors++; end
    end
endtask

task automatic rd(input [15:0] addr, output [15:0] v);
    begin access(1'b0, addr, 16'h0, v); end
endtask
task automatic wr(input [15:0] addr, input [15:0] v);
    reg [15:0] dummy;
    begin access(1'b1, addr, v, dummy); end
endtask
task automatic expect_reg(input string label, input [15:0] addr, input [15:0] exp);
    reg [15:0] v;
    begin
        rd(addr, v);
        if (v !== exp) begin
            $sformat(str, "%s: reg 0x%02h = 0x%04h, expected 0x%04h", label, addr, v, exp);
            logger.error(module_name, str); errors++;
        end else begin
            $sformat(str, "%s OK (0x%04h)", label, v); logger.info(module_name, str);
        end
    end
endtask

localparam [31:0] DAC_CLK = `PLATFORM_DAC_CLK_HZ;
localparam [15:0] CH1 = `REG_CH_BASE;
localparam [15:0] CH2 = `REG_CH_BASE + `REG_CH_STRIDE;
wire [`CH_WORD_W-1:0] w_ch1 = commit_word[`CH_WORD_W-1:0];
wire [`CH_WORD_W-1:0] w_ch2 = commit_word[2*`CH_WORD_W-1:`CH_WORD_W];

initial begin
    errors = 0; pulses = 0; pushes = 0;
`ifdef ENABLE_DUMPVARS
    $dumpvars(0, main);
`endif
    $sformat(module_name, "%m");
    logger.info(module_name, " << Starting the Simulation >>");

    repeat (3) @(posedge clk);
    rst_n = 1;
    repeat (2) @(posedge clk);

    // identity / clock / defaults
    expect_reg("ID",         `REG_ID,          `ACM9767_ID);
    expect_reg("VERSION",    `REG_VERSION,     `ACM9767_VERSION);
    expect_reg("DAC_CLK_LO", `REG_DAC_CLK_LO,  DAC_CLK[15:0]);
    expect_reg("DAC_CLK_HI", `REG_DAC_CLK_HI,  DAC_CLK[31:16]);
    expect_reg("CHANNELS",   `REG_CHANNELS,    `PLATFORM_DAC_CHANNELS);
    expect_reg("STATUS no lock", `REG_STATUS,  16'h0000);
    pll_lock = 1;
    expect_reg("STATUS lock",    `REG_STATUS,  16'h0001);
    expect_reg("ch1 amplitude default", CH1 + `REG_CH_AMPLITUDE, 16'hFFFF);
    expect_reg("ch2 duty default",      CH2 + `REG_CH_DUTY,      16'h8000);
    expect_reg("ch1 waveform default",  CH1 + `REG_CH_WAVEFORM,  {12'd0, `WAVE_SINE});
    expect_reg("unmapped 0x08",         16'h0008,                16'h0000);
    expect_reg("ch1 modparam default",  CH1 + `REG_CH_MODPARAM,  16'h0000);
    expect_reg("ch1 reserved +8",       CH1 + 16'h0008,          16'h0000);

    // SSB / FIFO registers and the audio write window
    expect_reg("AUDIO_RATE", `REG_AUDIO_RATE, `PLATFORM_SSB_AUDIO_RATE_HZ);
    expect_reg("FIFO_DEPTH", `REG_FIFO_DEPTH, `PLATFORM_SSB_FIFO_DEPTH);
    fifo_level = 123;
    expect_reg("FIFO_LEVEL", `REG_FIFO_LEVEL, 16'd123);
    wr(`REG_AUDIO_BASE, 16'h1234);
    wr(`REG_AUDIO_END,  16'hABCD);
    wr(`REG_AUDIO_BASE + 16'd40, 16'h5555);
    wr(`REG_AUDIO_END + 16'd1, 16'h9999);        // outside the window: no push
    @(posedge clk); #2;
    if (pushes != 3 || last_push !== 16'h5555) begin
        $sformat(str, "audio window: %0d pushes, last %h (expected 3, 5555)", pushes, last_push);
        logger.error(module_name, str); errors++;
    end else logger.info(module_name, "audio window pushes OK");
    expect_reg("audio window reads 0", `REG_AUDIO_BASE, 16'h0000);
    // underrun toggle edges are counted, write clears
    repeat (3) begin underrun_tog = ~underrun_tog; repeat (4) @(posedge clk); end
    expect_reg("UNDERRUNS = 3", `REG_UNDERRUNS, 16'd3);
    wr(`REG_UNDERRUNS, 16'h0000);
    expect_reg("UNDERRUNS cleared", `REG_UNDERRUNS, 16'd0);

    // staged writes + read-back
    wr(CH1 + `REG_CH_FTW_LO, 16'h5678);
    wr(CH1 + `REG_CH_FTW_HI, 16'h1234);
    wr(CH1 + `REG_CH_PHASE,  16'h4000);
    wr(CH1 + `REG_CH_WAVEFORM, 16'h0003);
    wr(CH2 + `REG_CH_OFFSET, 16'hFC18);   // -1000
    wr(CH2 + `REG_CH_AMPLITUDE, 16'h8000);
    wr(CH2 + `REG_CH_WAVEFORM, 16'hFFF1);  // only [3:0] kept
    wr(CH2 + `REG_CH_MODPARAM, 16'h1234);
    wr(`REG_CONTROL, 16'h000B);
    expect_reg("ch1 FTW_LO", CH1 + `REG_CH_FTW_LO, 16'h5678);
    expect_reg("ch1 FTW_HI", CH1 + `REG_CH_FTW_HI, 16'h1234);
    expect_reg("ch1 PHASE",  CH1 + `REG_CH_PHASE,  16'h4000);
    expect_reg("ch1 WAVEFORM", CH1 + `REG_CH_WAVEFORM, 16'h0003);
    expect_reg("ch2 OFFSET", CH2 + `REG_CH_OFFSET, 16'hFC18);
    expect_reg("ch2 WAVEFORM masked", CH2 + `REG_CH_WAVEFORM, 16'h0001);
    expect_reg("ch2 MODPARAM", CH2 + `REG_CH_MODPARAM, 16'h1234);
    expect_reg("CONTROL", `REG_CONTROL, 16'h000B);
    wr(`REG_ID, 16'hDEAD);
    expect_reg("ID is read-only", `REG_ID, `ACM9767_ID);

    // nothing committed yet
    if (pulses != 0 || commit_word !== {`COMMIT_WORD_W{1'b0}}) begin
        logger.error(module_name, "commit happened before COMMIT was written"); errors++;
    end

    // commit while the CDC is busy: stays pending
    cdc_busy = 1;
    wr(`REG_COMMIT, 16'h0001);
    repeat (4) @(posedge clk); #2;
    if (!commit_busy) begin logger.error(module_name, "commit_busy low while pending"); errors++; end
    if (pulses != 0) begin logger.error(module_name, "commit_pulse fired while cdc busy"); errors++; end
    expect_reg("STATUS busy+lock", `REG_STATUS, 16'h0003);
    cdc_busy = 0;
    repeat (3) @(posedge clk); #2;
    if (pulses != 1) begin
        $sformat(str, "commit_pulse count %0d, expected 1", pulses); logger.error(module_name, str); errors++;
    end
    if (commit_busy) begin logger.error(module_name, "commit_busy still high after handover"); errors++; end

    // committed snapshot contents
    if (ctrl_committed !== 4'hB) begin
        $sformat(str, "ctrl_committed = %h, expected b", ctrl_committed); logger.error(module_name, str); errors++;
    end
    if (`CH_FTW(w_ch1) !== 32'h12345678 || `CH_PHASE(w_ch1) !== 16'h4000 ||
        `CH_WAVEFORM(w_ch1) !== 4'd3 || `CH_AMPLITUDE(w_ch1) !== 16'hFFFF || `CH_DUTY(w_ch1) !== 16'h8000) begin
        $sformat(str, "ch1 word mismatch: ftw=%h phase=%h wave=%0d amp=%h duty=%h",
                 `CH_FTW(w_ch1), `CH_PHASE(w_ch1), `CH_WAVEFORM(w_ch1), `CH_AMPLITUDE(w_ch1), `CH_DUTY(w_ch1));
        logger.error(module_name, str); errors++;
    end
    if (`CH_OFFSET(w_ch2) !== 16'hFC18 || `CH_AMPLITUDE(w_ch2) !== 16'h8000 || `CH_WAVEFORM(w_ch2) !== 4'd1 ||
        `CH_FTW(w_ch2) !== 32'h0 || `CH_MODPARAM(w_ch2) !== 16'h1234) begin
        $sformat(str, "ch2 word mismatch: ftw=%h off=%h amp=%h wave=%0d",
                 `CH_FTW(w_ch2), `CH_OFFSET(w_ch2), `CH_AMPLITUDE(w_ch2), `CH_WAVEFORM(w_ch2));
        logger.error(module_name, str); errors++;
    end

    // a later staged write does not disturb the committed word until the next COMMIT
    wr(CH1 + `REG_CH_FTW_HI, 16'h0000);
    repeat (2) @(posedge clk); #2;
    if (`CH_FTW(w_ch1) !== 32'h12345678) begin logger.error(module_name, "committed word changed without COMMIT"); errors++; end
    wr(`REG_COMMIT, 16'hFFFF);
    repeat (3) @(posedge clk); #2;
    if (`CH_FTW(w_ch1) !== 32'h00005678 || pulses != 2) begin logger.error(module_name, "second COMMIT not applied"); errors++; end

    if (errors == 0) begin
        logger.info(module_name, "dac_regs register map OK");
        `TEST_PASS
    end else
        `TEST_FAIL
end

always #500000 begin
    logger.error(module_name, "System hangs");
    `TEST_FAIL
end

endmodule
