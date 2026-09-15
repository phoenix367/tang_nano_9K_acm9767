`include "timescale.v"
`include "acm9767_defs.vh"
`include "platform_config.vh"
`include "svlogger.sv"
`include "test_utils.sv"
`include "test_config.sv"

// End-to-end test of acm9767_top with the real platform.json parameters:
// 27 MHz sys_clk, the rPLL simulation model making dac_clk, 1 Mbaud 8-E-1
// Modbus RTU from a testbench "host" UART.
//
// Flow: wait for the PLL to lock (polled through STATUS over Modbus), read the
// identity / clock registers, program channel 1 with FC10 (32 samples per
// period sine), enable it in CONTROL, write COMMIT, wait for STATUS.busy to
// clear, then watch da1_data in the dac_clk domain: 10 periods must contain
// 10 (+-1) rising mid-scale crossings, da1_clk / da1_wrt must toggle in antiphase. On a
// two-channel build (PLATFORM_DAC_CH2) channel 2 must sit at mid-scale and then
// gets a square wave and must toggle too.

module main();

localparam LOG_LEVEL = `DEFAULT_LOG_LEVEL;
localparam integer SYS_HZ = `PLATFORM_SYS_CLK_HZ;
localparam integer BAUD   = `PLATFORM_UART_BAUD;
localparam [7:0]   SLAVE  = `PLATFORM_MODBUS_DEVICE_ID;
localparam [15:0]  CH1 = `REG_CH_BASE;
localparam [15:0]  CH2 = `REG_CH_BASE + `REG_CH_STRIDE;
localparam integer PERIOD = 32;
localparam [31:0]  FTW = 32'h0800_0000;   // 2^32 / 32

reg sys_clk = 0, sys_rst_n = 0;
wire uart_tx, uart_rx;
wire [5:0] leds;
wire da1_clk, da1_wrt;
wire [13:0] da1_data;
`ifdef PLATFORM_DAC_CH2
wire da2_clk, da2_wrt;
wire [13:0] da2_data;
`else
wire da2_clk = 1'b0, da2_wrt = 1'b0;
wire [13:0] da2_data = 14'h2000;
`endif

string module_name;
DataLogger #(.verbosity(LOG_LEVEL)) logger();

acm9767_top dut (
    .sys_clk(sys_clk), .sys_rst_n(sys_rst_n),
    .uart_tx(uart_tx), .uart_rx(uart_rx), .leds(leds),
    .da1_clk(da1_clk), .da1_wrt(da1_wrt), .da1_data(da1_data)
`ifdef PLATFORM_DAC_CH2
    , .da2_clk(da2_clk), .da2_wrt(da2_wrt), .da2_data(da2_data)
`endif
);

// host-side UART (same core, cross-wired)
reg  [7:0] h_tx_data = 0; reg h_tx_start = 0; wire h_tx_busy;
wire [7:0] h_rx_data; wire h_rx_valid;
uart #(.CLK_FREQ(SYS_HZ), .BAUD(BAUD)) host_uart (
    .clk(sys_clk), .reset_n(sys_rst_n),
    .tx_data(h_tx_data), .tx_start(h_tx_start), .tx_busy(h_tx_busy), .tx(uart_rx),
    .rx(uart_tx), .rx_data(h_rx_data), .rx_valid(h_rx_valid),
    .rx_parity_error(), .rx_frame_error()
);

always #18.5185 sys_clk = ~sys_clk;   // 27 MHz

reg [7:0] rxq [0:255];
integer   rxn;
always @(posedge sys_clk or negedge sys_rst_n)
    if (!sys_rst_n) rxn <= 0;
    else if (h_rx_valid) begin rxq[rxn] <= h_rx_data; rxn <= rxn + 1; end

function [15:0] crc_upd(input [15:0] c0, input [7:0] b);
    logic [15:0] c; integer i;
    begin
        c = c0 ^ {8'h00, b};
        for (i = 0; i < 8; i = i + 1) c = c[0] ? ((c >> 1) ^ 16'hA001) : (c >> 1);
        crc_upd = c;
    end
endfunction

reg [7:0] req  [0:63];
reg [7:0] resp [0:63];
integer errors;
string str;

task automatic send_byte(input [7:0] b);
    begin
        @(posedge sys_clk); #2;
        while (h_tx_busy) begin @(posedge sys_clk); #2; end
        @(negedge sys_clk); h_tx_data = b; h_tx_start = 1'b1;
        @(negedge sys_clk); h_tx_start = 1'b0;
    end
endtask

// send req[0..n-1] + CRC, wait for `explen` response bytes (2 ms timeout)
task automatic txn(input integer n, input integer explen, output integer rn);
    integer i, base; reg [15:0] c; realtime t0;
    begin
        c = 16'hFFFF;
        for (i = 0; i < n; i = i + 1) c = crc_upd(c, req[i]);
        req[n] = c[7:0]; req[n+1] = c[15:8];
        base = rxn;
        for (i = 0; i < n + 2; i = i + 1) send_byte(req[i]);
        t0 = $realtime;
        while ((rxn - base) < explen && ($realtime - t0) < 2_000_000.0) @(posedge sys_clk);
        repeat (200) @(posedge sys_clk);   // let any trailing bytes arrive
        rn = rxn - base;
        for (i = 0; i < rn && i < 64; i = i + 1) resp[i] = rxq[base + i];
        c = 16'hFFFF;
        for (i = 0; i < rn; i = i + 1) c = crc_upd(c, resp[i]);
        if (rn != explen || c != 16'h0000) begin
            $sformat(str, "func 0x%02h: response %0d bytes (expected %0d), crc %h", req[1], rn, explen, c);
            logger.error(module_name, str); errors = errors + 1;
        end
    end
endtask

task automatic read_reg(input [15:0] addr, output [15:0] v);
    integer rn;
    begin
        req[0]=SLAVE; req[1]=8'h03; req[2]=addr[15:8]; req[3]=addr[7:0]; req[4]=8'h00; req[5]=8'h01;
        txn(6, 7, rn);
        v = (rn == 7) ? {resp[3], resp[4]} : 16'hFFFF;
    end
endtask

task automatic write_reg(input [15:0] addr, input [15:0] v);
    integer rn;
    begin
        req[0]=SLAVE; req[1]=8'h06; req[2]=addr[15:8]; req[3]=addr[7:0]; req[4]=v[15:8]; req[5]=v[7:0];
        txn(6, 8, rn);
    end
endtask

// FC10 write of the 7-register channel block
task automatic write_channel(input [15:0] base, input [31:0] ftw, input [15:0] phase, input [15:0] amp,
                             input [15:0] ofs, input [15:0] wave, input [15:0] duty);
    integer rn;
    begin
        req[0]=SLAVE; req[1]=8'h10; req[2]=base[15:8]; req[3]=base[7:0]; req[4]=8'h00; req[5]=8'd7; req[6]=8'd14;
        {req[7],  req[8]}  = ftw[15:0];
        {req[9],  req[10]} = ftw[31:16];
        {req[11], req[12]} = phase;
        {req[13], req[14]} = amp;
        {req[15], req[16]} = ofs;
        {req[17], req[18]} = wave;
        {req[19], req[20]} = duty;
        txn(21, 8, rn);
    end
endtask

task automatic expect_reg(input string label, input [15:0] addr, input [15:0] exp);
    reg [15:0] v;
    begin
        read_reg(addr, v);
        if (v !== exp) begin
            $sformat(str, "%s: reg 0x%02h = 0x%04h, expected 0x%04h", label, addr, v, exp);
            logger.error(module_name, str); errors = errors + 1;
        end else begin
            $sformat(str, "%s OK (0x%04h)", label, v); logger.info(module_name, str);
        end
    end
endtask

task automatic wait_status(input [15:0] mask, input [15:0] want, input string label);
    reg [15:0] v; integer n;
    begin
        n = 0; v = ~want;
        while ((v & mask) !== want && n < 200) begin read_reg(`REG_STATUS, v); n = n + 1; end
        if ((v & mask) !== want) begin
            $sformat(str, "%s: STATUS = 0x%04h never matched", label, v); logger.error(module_name, str); errors = errors + 1;
        end else begin
            $sformat(str, "%s after %0d polls", label, n); logger.info(module_name, str);
        end
    end
endtask

// forwarded DAC clock edges (event driven: sampling once per dac_clk would
// always see the same phase)
integer clk_edges [0:1], wrt_edges;
initial begin clk_edges[0] = 0; clk_edges[1] = 0; wrt_edges = 0; end
always @(da1_clk) clk_edges[0] = clk_edges[0] + 1;
always @(da1_wrt) wrt_edges = wrt_edges + 1;
always @(da2_clk) clk_edges[1] = clk_edges[1] + 1;

// count rising mid-scale crossings on a data bus over n dac_clk cycles, and the
// DAC clock edges seen meanwhile (expect 2 per cycle)
task automatic count_crossings(input integer ch, input integer n, output integer cnt, output integer toggles);
    integer i, e0; reg [13:0] prev, cur;
    begin
        cnt = 0;
        @(posedge dut.dac_clk); #2; prev = ch ? da2_data : da1_data; e0 = clk_edges[ch];
        for (i = 0; i < n; i = i + 1) begin
            @(posedge dut.dac_clk); #2;
            cur = ch ? da2_data : da1_data;
            if (prev < 14'h2000 && cur >= 14'h2000) cnt = cnt + 1;
            prev = cur;
        end
        toggles = clk_edges[ch] - e0;
    end
endtask

localparam [31:0] DAC_CLK = `PLATFORM_DAC_CLK_HZ;
integer cnt, tog, i;
reg [13:0] v14;
reg still;

initial begin
    errors = 0;
`ifdef ENABLE_DUMPVARS
    $dumpvars(0, main);
`endif
    $sformat(module_name, "%m");
    logger.info(module_name, " << Starting the Simulation >>");

    repeat (5) @(posedge sys_clk);
    sys_rst_n = 1;
    repeat (50) @(posedge sys_clk);

    // identity and clock, then PLL lock (the rPLL model takes ~50 us)
    expect_reg("ID",         `REG_ID,         `ACM9767_ID);
    expect_reg("VERSION",    `REG_VERSION,    `ACM9767_VERSION);
    expect_reg("DAC_CLK_LO", `REG_DAC_CLK_LO, DAC_CLK[15:0]);
    expect_reg("DAC_CLK_HI", `REG_DAC_CLK_HI, DAC_CLK[31:16]);
    wait_status(16'h0001, 16'h0001, "PLL locked");
    if (leds[0] !== 1'b0) begin logger.error(module_name, "LED0 not lit with PLL locked"); errors++; end

    // before any commit both DACs sit at mid-scale and the DAC clocks run
    count_crossings(0, 64, cnt, tog);
    if (cnt != 0 || da1_data !== 14'h2000 || da2_data !== 14'h2000) begin logger.error(module_name, "DAC not at mid-scale before commit"); errors++; end
    if (tog < 126 || tog > 130) begin $sformat(str, "da1_clk toggled %0d times in 64 cycles, expected 128", tog); logger.error(module_name, str); errors++; end
    else logger.info(module_name, "da1_clk forwarded clock running");
    if (wrt_edges < 126) begin $sformat(str, "da1_wrt toggled only %0d times", wrt_edges); logger.error(module_name, str); errors++; end
    if (da1_wrt !== ~da1_clk) begin logger.error(module_name, "da1_wrt is not the inverse of da1_clk (CLK must lead WRT by half a period)"); errors++; end
    expect_reg("CHANNELS", `REG_CHANNELS, `PLATFORM_DAC_CHANNELS);

    // channel 1: sine, 32 samples/period; enable; commit
    write_channel(CH1, FTW, 16'h0000, 16'hFFFF, 16'h0000, {12'd0, `WAVE_SINE}, 16'h8000);
    expect_reg("ch1 FTW_HI staged", CH1 + `REG_CH_FTW_HI, FTW[31:16]);
    write_reg(`REG_CONTROL, 16'h0001);
    write_reg(`REG_COMMIT, 16'h0001);
    wait_status(16'h0002, 16'h0000, "commit applied");
    if (leds[1] !== 1'b0) begin logger.error(module_name, "LED1 not lit with ch1 enabled"); errors++; end

    count_crossings(0, PERIOD * 10, cnt, tog);
    if (cnt < 9 || cnt > 11) begin $sformat(str, "ch1: %0d rising crossings in 10 periods", cnt); logger.error(module_name, str); errors++; end
    else begin $sformat(str, "ch1 sine: %0d periods observed", cnt); logger.info(module_name, str); end
`ifdef PLATFORM_DAC_CH2
    count_crossings(1, 64, cnt, tog);
    if (cnt != 0 || da2_data !== 14'h2000) begin logger.error(module_name, "ch2 not idle at mid-scale"); errors++; end
    if (tog < 126 || tog > 130) begin $sformat(str, "da2_clk toggled %0d times in 64 cycles, expected 128", tog); logger.error(module_name, str); errors++; end

    // channel 2: square 50 %, 4 samples/period; commit; must toggle between the two rails
    write_channel(CH2, 32'h4000_0000, 16'h0000, 16'hFFFF, 16'h0000, {12'd0, `WAVE_SQUARE}, 16'h8000);
    write_reg(`REG_CONTROL, 16'h0003);
    write_reg(`REG_COMMIT, 16'h0001);
    wait_status(16'h0002, 16'h0000, "second commit applied");
    count_crossings(1, 40, cnt, tog);
    if (cnt != 10) begin $sformat(str, "ch2 square: %0d rising crossings in 40 cycles, expected 10", cnt); logger.error(module_name, str); errors++; end
    else logger.info(module_name, "ch2 square OK");
    still = 1;
    for (i = 0; i < 8; i = i + 1) begin
        @(posedge dut.dac_clk); #2;
        if (da2_data !== 14'h3FFF && da2_data !== 14'h0000) still = 0;
    end
    if (!still) begin logger.error(module_name, "ch2 square not on the rails"); errors++; end
`else
    logger.info(module_name, "single-channel build: channel 2 checks skipped");
`endif

    // disable everything and confirm mid-scale again
    write_reg(`REG_CONTROL, 16'h0000);
    write_reg(`REG_COMMIT, 16'h0001);
    wait_status(16'h0002, 16'h0000, "third commit applied");
    count_crossings(0, 64, cnt, tog);
    if (cnt != 0 || da1_data !== 14'h2000 || da2_data !== 14'h2000) begin logger.error(module_name, "DACs not back at mid-scale"); errors++; end

    if (errors == 0) begin
        logger.info(module_name, "Modbus -> DAC end-to-end OK");
        `TEST_PASS
    end else
        `TEST_FAIL
end

initial begin
    #30_000_000;   // 30 ms
    logger.error(module_name, "Watchdog timeout -- integration test hung");
    `TEST_FAIL
end

endmodule
