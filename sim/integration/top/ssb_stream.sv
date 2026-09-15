`include "timescale.v"
`include "acm9767_defs.vh"
`include "platform_config.vh"
`include "svlogger.sv"
`include "test_utils.sv"
`include "test_config.sv"

// Streaming plumbing through the real top over Modbus: AUDIO_RATE / FIFO_DEPTH
// identity registers, UNDERRUNS counting while the FIFO is empty and clearing
// on write, an FC10 block of 64 samples into the 0x0100 window raising
// FIFO_LEVEL, channel 1 switched to WAVE_SSB_USB producing a non-idle DAC
// output while samples remain, and the FIFO draining at the audio rate.

module main();

localparam LOG_LEVEL = `DEFAULT_LOG_LEVEL;
localparam integer SYS_HZ = `PLATFORM_SYS_CLK_HZ;
localparam integer BAUD   = `PLATFORM_UART_BAUD;
localparam [7:0]   SLAVE  = `PLATFORM_MODBUS_DEVICE_ID;
localparam [15:0]  CH1    = `REG_CH_BASE;
localparam integer BLOCK  = 64;

reg sys_clk = 0, sys_rst_n = 0;
wire uart_tx, uart_rx;
wire [5:0] leds;
wire da1_clk, da1_wrt;
wire [13:0] da1_data;

string module_name;
DataLogger #(.verbosity(LOG_LEVEL)) logger();

acm9767_top dut (
    .sys_clk(sys_clk), .sys_rst_n(sys_rst_n),
    .uart_tx(uart_tx), .uart_rx(uart_rx), .leds(leds),
    .da1_clk(da1_clk), .da1_wrt(da1_wrt), .da1_data(da1_data)
`ifdef PLATFORM_DAC_CH2
    , .da2_clk(), .da2_wrt(), .da2_data()
`endif
);

reg  [7:0] h_tx_data = 0; reg h_tx_start = 0; wire h_tx_busy;
wire [7:0] h_rx_data; wire h_rx_valid;
uart #(.CLK_FREQ(SYS_HZ), .BAUD(BAUD)) host_uart (
    .clk(sys_clk), .reset_n(sys_rst_n),
    .tx_data(h_tx_data), .tx_start(h_tx_start), .tx_busy(h_tx_busy), .tx(uart_rx),
    .rx(uart_tx), .rx_data(h_rx_data), .rx_valid(h_rx_valid),
    .rx_parity_error(), .rx_frame_error()
);
always #18.5185 sys_clk = ~sys_clk;

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

reg [7:0] req  [0:255];
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
        repeat (200) @(posedge sys_clk);
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
task automatic expect_reg(input string label, input [15:0] addr, input [15:0] exp);
    reg [15:0] v;
    begin
        read_reg(addr, v);
        if (v !== exp) begin
            $sformat(str, "%s: reg 0x%03h = 0x%04h, expected 0x%04h", label, addr, v, exp);
            logger.error(module_name, str); errors = errors + 1;
        end else begin $sformat(str, "%s OK (0x%04h)", label, v); logger.info(module_name, str); end
    end
endtask

// FC10 block of BLOCK tone samples into the audio window
task automatic write_audio_block(input integer start_idx);
    integer rn, i; reg signed [15:0] v;
    begin
        req[0]=SLAVE; req[1]=8'h10; req[2]=8'h01; req[3]=8'h00; req[4]=8'h00; req[5]=BLOCK; req[6]=2*BLOCK;
        for (i = 0; i < BLOCK; i = i + 1) begin
            v = $rtoi(20000.0 * $sin(2.0 * 3.14159265358979 * 1000.0 * (start_idx + i) / 16000.0));
            req[7 + 2*i] = v[15:8]; req[8 + 2*i] = v[7:0];
        end
        txn(7 + 2*BLOCK, 8, rn);
    end
endtask

task automatic wait_status_idle;
    reg [15:0] v; integer n;
    begin
        n = 0; v = 16'hFFFF;
        while ((v & 16'h0002) && n < 200) begin read_reg(`REG_STATUS, v); n = n + 1; end
        if (v & 16'h0002) begin logger.error(module_name, "commit never completed"); errors++; end
    end
endtask

reg [15:0] v, u0, u1, lvl0, lvl1;
integer i, active;

initial begin
    errors = 0;
`ifdef ENABLE_DUMPVARS
    $dumpvars(0, main);
`endif
    $sformat(module_name, "%m");
    logger.info(module_name, " << Starting the Simulation >>");
    repeat (5) @(posedge sys_clk); sys_rst_n = 1; repeat (50) @(posedge sys_clk);

    expect_reg("AUDIO_RATE", `REG_AUDIO_RATE, `PLATFORM_SSB_AUDIO_RATE_HZ);
    expect_reg("FIFO_DEPTH", `REG_FIFO_DEPTH, `PLATFORM_SSB_FIFO_DEPTH);
    expect_reg("FIFO_LEVEL empty", `REG_FIFO_LEVEL, 16'd0);

    // PLL lock, then the baseband tick runs and every tick with an empty FIFO is an underrun
    v = 0; i = 0;
    while (!(v & 16'h0001) && i < 200) begin read_reg(`REG_STATUS, v); i = i + 1; end
    write_reg(`REG_UNDERRUNS, 16'h0000);
    read_reg(`REG_UNDERRUNS, u0);
    repeat (27000 * 2) @(posedge sys_clk);   // 2 ms = 32 audio ticks
    read_reg(`REG_UNDERRUNS, u1);
    if (u1 <= u0 || u1 > u0 + 60) begin
        $sformat(str, "underruns %0d -> %0d over 2 ms, expected ~+32", u0, u1); logger.error(module_name, str); errors++;
    end else begin $sformat(str, "underruns counted %0d -> %0d", u0, u1); logger.info(module_name, str); end

    // SSB on channel 1, then stream one block
    req[0]=SLAVE; req[1]=8'h10; req[2]=8'h00; req[3]=8'h10; req[4]=8'h00; req[5]=8'd7; req[6]=8'd14;
    {req[7],  req[8]}  = 16'h0000; {req[9],  req[10]} = 16'h0800;     // ftw 2^27 -> 1.6875 MHz
    {req[11], req[12]} = 16'h0000; {req[13], req[14]} = 16'hFFFF;
    {req[15], req[16]} = 16'h0000; {req[17], req[18]} = {12'd0, `WAVE_SSB_USB};
    {req[19], req[20]} = 16'h8000;
    txn(21, 8, i);
    write_reg(`REG_CONTROL, 16'h0001);
    write_reg(`REG_COMMIT, 16'h0001);
    wait_status_idle();

    write_audio_block(0);
    read_reg(`REG_FIFO_LEVEL, lvl0);
    // the block takes ~1.5 ms on the wire = ~24 samples consumed while it arrives
    if (lvl0 < 20 || lvl0 > BLOCK) begin
        $sformat(str, "FIFO_LEVEL %0d after a %0d-sample block", lvl0, BLOCK); logger.error(module_name, str); errors++;
    end else begin $sformat(str, "FIFO_LEVEL %0d after the block", lvl0); logger.info(module_name, str); end
    write_audio_block(BLOCK);
    write_reg(`REG_UNDERRUNS, 16'h0000);
    read_reg(`REG_FIFO_LEVEL, lvl1);
    read_reg(`REG_UNDERRUNS, u0);
    if (u0 != 0) begin $sformat(str, "underruns %0d while %0d samples buffered", u0, lvl1); logger.error(module_name, str); errors++; end

    // DAC output must be active (SSB of the tone) -- not parked at mid-scale.
    // The data follows a run of underruns, so the limiter fades it in from
    // zero (64 ms to unity): the gain must be ramping and the output already
    // off mid-scale for some samples.
    active = 0;
    for (i = 0; i < 2000; i = i + 1) begin
        @(posedge dut.dac_clk); #2;
        if (da1_data !== 14'h2000) active = active + 1;
    end
    if (dut.baseband.lim.gain == 0 || dut.baseband.lim.gain == 17'd32768) begin
        $sformat(str, "limiter gain %0d: no fade-in after the underrun run", dut.baseband.lim.gain); logger.error(module_name, str); errors++;
    end else begin $sformat(str, "fade-in in progress, limiter gain %0d", dut.baseband.lim.gain); logger.info(module_name, str); end
    if (active < 100) begin $sformat(str, "DAC output idle (%0d/2000 samples off mid-scale)", active); logger.error(module_name, str); errors++; end
    else logger.info(module_name, "SSB output active");

    // let the FIFO drain: two 64-sample blocks = 8 ms of audio, minus what
    // was consumed while they were written; wait 10 ms to be sure
    repeat (27000 * 10) @(posedge sys_clk);
    read_reg(`REG_FIFO_LEVEL, v);
    if (v != 0) begin $sformat(str, "FIFO_LEVEL %0d after draining", v); logger.error(module_name, str); errors++; end
    read_reg(`REG_UNDERRUNS, u1);
    if (u1 == 0) begin logger.error(module_name, "no underruns after the FIFO ran dry"); errors++; end
    else begin $sformat(str, "FIFO drained, %0d underruns since", u1); logger.info(module_name, str); end

    if (errors == 0) begin logger.info(module_name, "SSB streaming plumbing OK"); `TEST_PASS end
    else `TEST_FAIL
end

initial begin
    #60_000_000;
    logger.error(module_name, "Watchdog timeout -- ssb_stream test hung");
    `TEST_FAIL
end
endmodule
