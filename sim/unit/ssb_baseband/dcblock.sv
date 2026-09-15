`include "timescale.v"
`include "acm9767_defs.vh"
`include "platform_config.vh"
`include "svlogger.sv"
`include "test_utils.sv"
`include "test_config.sv"

// ssb_baseband front end: DC blocker, underrun hold and the fade trigger.
//  - a DC step at the FIFO output enters the FIR as a step that decays with
//    tau = 2^DC_SHIFT samples (no steady DC into the Hilbert filter);
//  - when the FIFO goes empty the FIR input keeps decaying smoothly (no jump
//    to zero): the largest sample-to-sample change during the outage is small;
//  - data resuming after >= 8 underruns pulses `fade` into the limiter once,
//    and the limiter gain restarts from 0.
// Runs with the real platform INTERP, so it takes a few seconds of iverilog.

module main();

localparam LOG_LEVEL = `DEFAULT_LOG_LEVEL;
localparam integer INTERP = `PLATFORM_SSB_INTERP;
localparam integer SHIFT  = `PLATFORM_SSB_DC_SHIFT;

reg clk = 0, rst_n = 0;
wire fifo_rd;
reg  signed [15:0] fifo_rdata = 0;
reg  fifo_empty = 0;
wire signed [15:0] bb_i, bb_q;
wire underrun_tog;

string module_name;
DataLogger #(.verbosity(LOG_LEVEL)) logger();

ssb_baseband dut (
    .clk(clk), .rst_n(rst_n),
    .fifo_rd(fifo_rd), .fifo_rdata(fifo_rdata), .fifo_empty(fifo_empty),
    .bb_i(bb_i), .bb_q(bb_q), .underrun_tog(underrun_tog)
);
always #9.26 clk = ~clk;

integer errors, n, fades, max_jump, prev_in, first_in, later_in, nfir;
reg signed [15:0] fir_seen;
reg fir_prev;
string str;

// count fade pulses and FIR strobes, track the FIR input
always @(posedge clk) begin
    if (dut.fade) fades = fades + 1;
end

task automatic wait_samples(input integer k);
    begin repeat (k * INTERP) @(posedge clk); end
endtask

initial begin
    errors = 0; fades = 0; max_jump = 0; nfir = 0;
`ifdef ENABLE_DUMPVARS
    $dumpvars(0, main);
`endif
    $sformat(module_name, "%m");
    logger.info(module_name, " << Starting the Simulation >>");
    repeat (3) @(posedge clk); rst_n = 1;

    // 1. constant 20000 from the FIFO: the FIR input steps to ~20000 then decays
    fifo_rdata = 16'sd20000;
    wait_samples(3);
    first_in = dut.fir_in;
    wait_samples(4 * (1 << SHIFT));            // 4 tau
    later_in = dut.fir_in;
    $sformat(str, "DC 20000 -> FIR input %0d at once, %0d after 4 tau", first_in, later_in);
    logger.info(module_name, str);
    if (first_in < 16'sd19000) begin logger.error(module_name, "step not passed"); errors++; end
    if (later_in > 16'sd600 || later_in < -16'sd600) begin logger.error(module_name, "DC not blocked"); errors++; end

    // 1b. negative step: -20000 passes as a -20000 step and decays; a 2 kHz +-20000 tone passes
    //     with its negative half-cycles intact (signedness of the leak term)
    fifo_rdata = 16'sd0;
    wait_samples(5 * (1 << SHIFT));            // settle to zero first
    fifo_rdata = -16'sd20000;
    wait_samples(3);
    first_in = dut.fir_in;
    wait_samples(4 * (1 << SHIFT));
    later_in = dut.fir_in;
    $sformat(str, "DC -20000 -> FIR input %0d at once, %0d after 4 tau", first_in, later_in);
    logger.info(module_name, str);
    if (first_in > -16'sd19000) begin logger.error(module_name, "negative step not passed"); errors++; end
    if (later_in > 16'sd600 || later_in < -16'sd600) begin logger.error(module_name, "negative DC not blocked"); errors++; end
    fifo_rdata = 16'sd0;
    wait_samples(5 * (1 << SHIFT));            // settle to zero, then the tone starts at sin(0) = 0
    max_jump = 0;
    for (n = 0; n < 64; n = n + 1) begin
        fifo_rdata = $rtoi(20000.0 * $sin(2.0 * 3.14159265358979 * n / 8.0));
        wait_samples(1);
        if (n >= 8 && (dut.fir_in > 16'sd21500 || dut.fir_in < -16'sd21500)) max_jump = max_jump + 1;
    end
    if (max_jump != 0) begin $sformat(str, "tone through the blocker exceeded +-21500 on %0d samples", max_jump); logger.error(module_name, str); errors++; end
    else logger.info(module_name, "tone passes the blocker within bounds");
    fifo_rdata = 16'sd20000; wait_samples(40);

    // 2. outage: 40 empty ticks with a mid-decay value held; the FIR input must not jump
    wait_samples(20);
    fifo_empty = 1;
    repeat (4) @(posedge clk);
    fifo_rdata = 16'sd0;                       // would be a -20000 step if it were consumed
    prev_in = dut.fir_in; max_jump = 0;
    for (n = 0; n < 40; n = n + 1) begin
        wait_samples(1);
        if (dut.fir_in - prev_in > max_jump) max_jump = dut.fir_in - prev_in;
        if (prev_in - dut.fir_in > max_jump) max_jump = prev_in - dut.fir_in;
        prev_in = dut.fir_in;
    end
    $sformat(str, "largest FIR input change during the outage: %0d LSB", max_jump);
    logger.info(module_name, str);
    if (max_jump > 300) begin logger.error(module_name, "underrun produced a step"); errors++; end
    if (fades != 0) begin logger.error(module_name, "fade pulsed during the outage"); errors++; end

    // 3. resume: one fade pulse, limiter gain restarts from 0
    fifo_rdata = 16'sd12000;
    repeat (4) @(posedge clk);
    fifo_empty = 0;
    wait_samples(3);
    if (fades != 1) begin $sformat(str, "fade pulses after resume: %0d", fades); logger.error(module_name, str); errors++; end
    else logger.info(module_name, "fade pulsed once on resume");
    if (dut.lim.gain > 17'd400) begin $sformat(str, "limiter gain %0d after fade", dut.lim.gain); logger.error(module_name, str); errors++; end
    wait_samples(1200);
    if (dut.lim.gain != 17'd32768) begin $sformat(str, "limiter gain %0d did not recover", dut.lim.gain); logger.error(module_name, str); errors++; end
    else logger.info(module_name, "gain recovered to unity");
    // a short hiccup (3 empty ticks) must not fade
    fifo_empty = 1; wait_samples(3); fifo_empty = 0; wait_samples(3);
    if (fades != 1) begin logger.error(module_name, "short underrun triggered a fade"); errors++; end

    if (errors == 0) begin `TEST_PASS end else begin `TEST_FAIL end
end

initial begin #2000000000; logger.error(module_name, "watchdog"); `TEST_FAIL end

endmodule
