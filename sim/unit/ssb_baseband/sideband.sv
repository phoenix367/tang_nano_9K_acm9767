`include "timescale.v"
`include "acm9767_defs.vh"
`include "platform_config.vh"
`include "svlogger.sv"
`include "test_utils.sv"
`include "test_config.sv"

// End-to-end SSB quality test in the dac_clk domain with the real platform
// constants (16 kS/s audio, x3375 CIC, 127-tap Hilbert): ssb_baseband fed from
// a testbench FIFO model with a 2 kHz tone, feeding dds_channel in
// WAVE_SSB_USB / WAVE_SSB_LSB on a 1.6875 MHz carrier. Hann-windowed Goertzel
// filters at fc+fm, fc-fm and fc over a 2 ms window measure the wanted
// sideband, the unwanted sideband and the carrier: the unwanted sideband and
// the carrier must each be >= 35 dB down, and the wanted tone must have the
// expected amplitude (CIC droop included).

module main();

localparam LOG_LEVEL = `DEFAULT_LOG_LEVEL;
localparam real    PI     = 3.14159265358979;
localparam real    FDAC   = `PLATFORM_DAC_CLK_HZ;
localparam real    FAUDIO = `PLATFORM_SSB_AUDIO_RATE_HZ;
localparam real    FM     = 2000.0;
localparam real    FC     = FDAC / 32.0;
localparam [31:0]  FTW    = 32'h0800_0000;           // 2^32 / 32
localparam real    AMP    = 24000.0;                 // of 32767
localparam integer NWIN   = 108000;                  // 2 ms at 54 MHz
localparam integer FILL   = 160 * `PLATFORM_SSB_INTERP;  // 160 audio samples: 127-tap FIR + limiter look-ahead + DC block + CIC settle

reg clk = 0, rst_n = 0;
always #9.26 clk = ~clk;

// ---- FIFO model: serves tone samples with the async_fifo read timing ----
wire fifo_rd;
reg  signed [15:0] fifo_rdata = 0;
reg  fifo_empty = 1;
integer sidx = 0;
always @(posedge clk) begin
    if (fifo_rd) begin
        fifo_rdata <= $rtoi(AMP * $cos(2.0 * PI * FM * sidx / FAUDIO));
        sidx <= sidx + 1;
    end
end

wire signed [15:0] bb_i, bb_q;
wire underrun_tog;
ssb_baseband dut (
    .clk(clk), .rst_n(rst_n),
    .fifo_rd(fifo_rd), .fifo_rdata(fifo_rdata), .fifo_empty(fifo_empty),
    .bb_i(bb_i), .bb_q(bb_q), .underrun_tog(underrun_tog)
);

reg [3:0] waveform = `WAVE_SSB_USB;
wire [13:0] dac_code;
dds_channel chan (
    .clk(clk), .rst_n(rst_n), .en(1'b1), .invert(1'b0),
    .ftw(FTW), .phase_ofs(16'd0), .amplitude(16'hFFFF), .offset(16'd0),
    .waveform(waveform), .duty(16'h8000), .modparam(16'd0), .bb_i(bb_i), .bb_q(bb_q), .dac_code(dac_code)
);

string module_name;
DataLogger #(.verbosity(LOG_LEVEL)) logger();
integer errors;
string str;

// Hann-windowed Goertzel magnitude (in DAC LSB amplitude) at freq over NWIN samples
task automatic measure(output real a_usb, output real a_lsb, output real a_car);
    real c_u, c_l, c_c, s0u, s1u, s2u, s0l, s1l, s2l, s0c, s1c, s2c, w, x, pu, pl, pc;
    integer n;
    begin
        c_u = 2.0 * $cos(2.0 * PI * (FC + FM) / FDAC);
        c_l = 2.0 * $cos(2.0 * PI * (FC - FM) / FDAC);
        c_c = 2.0 * $cos(2.0 * PI * FC / FDAC);
        s1u = 0; s2u = 0; s1l = 0; s2l = 0; s1c = 0; s2c = 0;
        for (n = 0; n < NWIN; n = n + 1) begin
            @(posedge clk); #2;
            w = 0.5 - 0.5 * $cos(2.0 * PI * n / NWIN);
            x = w * ($signed({1'b0, dac_code}) - 8192);
            s0u = x + c_u * s1u - s2u; s2u = s1u; s1u = s0u;
            s0l = x + c_l * s1l - s2l; s2l = s1l; s1l = s0l;
            s0c = x + c_c * s1c - s2c; s2c = s1c; s1c = s0c;
        end
        pu = s1u * s1u + s2u * s2u - c_u * s1u * s2u;
        pl = s1l * s1l + s2l * s2l - c_l * s1l * s2l;
        pc = s1c * s1c + s2c * s2c - c_c * s1c * s2c;
        // amplitude = 2*sqrt(p)/sum(window) with sum(Hann) = NWIN/2
        a_usb = 4.0 * $sqrt(pu) / NWIN;
        a_lsb = 4.0 * $sqrt(pl) / NWIN;
        a_car = 4.0 * $sqrt(pc) / NWIN;
    end
endtask

real au, al, ac, expected, ratio_db, car_db;

initial begin
    errors = 0;
`ifdef ENABLE_DUMPVARS
    $dumpvars(0, main);
`endif
    $sformat(module_name, "%m");
    logger.info(module_name, " << Starting the Simulation >>");
    repeat (3) @(posedge clk); @(negedge clk); rst_n = 1; fifo_empty = 0;

    repeat (FILL) @(posedge clk);
    // expected wanted-tone amplitude: AMP/32768 * 8191 * CIC droop sinc^3(FM/FAUDIO)
    expected = AMP / 32768.0 * 8191.0 * ($sin(PI * FM / FAUDIO) / (PI * FM / FAUDIO)) ** 3;

    measure(au, al, ac);
    ratio_db = 20.0 * $log10(au / (al + 1e-9));
    car_db   = 20.0 * $log10(au / (ac + 1e-9));
    $sformat(str, "USB: wanted %0.0f (expected %0.0f), unwanted %0.1f, carrier %0.1f -> sideband %0.1f dB, carrier %0.1f dB",
             au, expected, al, ac, ratio_db, car_db);
    if (ratio_db < 35.0 || car_db < 35.0 || au < 0.85 * expected || au > 1.15 * expected) begin
        logger.error(module_name, str); errors++;
    end else logger.info(module_name, str);

    waveform = `WAVE_SSB_LSB;
    repeat (1000) @(posedge clk);
    measure(au, al, ac);
    ratio_db = 20.0 * $log10(al / (au + 1e-9));
    car_db   = 20.0 * $log10(al / (ac + 1e-9));
    $sformat(str, "LSB: wanted %0.0f (expected %0.0f), unwanted %0.1f, carrier %0.1f -> sideband %0.1f dB, carrier %0.1f dB",
             al, expected, au, ac, ratio_db, car_db);
    if (ratio_db < 35.0 || car_db < 35.0 || al < 0.85 * expected || al > 1.15 * expected) begin
        logger.error(module_name, str); errors++;
    end else logger.info(module_name, str);

    if (errors == 0) begin logger.info(module_name, "SSB modulator OK"); `TEST_PASS end
    else `TEST_FAIL
end

always #40000000 begin logger.error(module_name, "System hangs"); `TEST_FAIL end
endmodule
