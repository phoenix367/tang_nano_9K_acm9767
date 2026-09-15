`include "timescale.v"
`include "acm9767_defs.vh"
`include "platform_config.vh"
`include "svlogger.sv"
`include "test_utils.sv"
`include "test_config.sv"

// File-driven modulator run for the host-side loopback check
// (scripts/ssb_loopback.py): audio samples come from a binary file, the DAC
// codes of the real ssb_baseband + dds_channel chain go to a binary file, and
// the Python analytic demodulator scores the result. Not a self-checking test
// on its own -- it is driven by ctest through the script.
//
// Plusargs: +in=<file>   int16 big-endian audio samples
//           +n=<count>   number of samples in the file
//           +out=<file>  uint16 big-endian DAC codes, one per dac_clk
//           +ftw=<int>   carrier tuning word
//           +wave=<int>  WAVE_* code (5 usb, 6 lsb, 7 am, 8 fm)
//           +modparam=<int>  AM depth / FM deviation register (default 0)

module main();

localparam LOG_LEVEL = `DEFAULT_LOG_LEVEL;
localparam integer INTERP = `PLATFORM_SSB_INTERP;
localparam integer TAIL   = 140;                     // samples of flush after the input

reg clk = 0, rst_n = 0;
always #9.26 clk = ~clk;

string in_path, out_path;
integer n_in, fd_in, fd_out, ftw_arg, wave_arg, modparam_arg, rc;
reg [15:0] audio [0:262143];

// ---- FIFO model: serves the file, zeros after the end ----
wire fifo_rd;
reg  signed [15:0] fifo_rdata = 0;
reg  fifo_empty = 1;
integer sidx = 0;
always @(posedge clk) begin
    if (fifo_rd) begin
        fifo_rdata <= (sidx < n_in) ? $signed(audio[sidx]) : 16'sd0;
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

reg [3:0]  waveform = `WAVE_SSB_USB;
reg [31:0] ftw = 32'h0800_0000;
reg [15:0] modparam = 16'd0;
wire [13:0] dac_code;
dds_channel chan (
    .clk(clk), .rst_n(rst_n), .en(1'b1), .invert(1'b0),
    .ftw(ftw), .phase_ofs(16'd0), .amplitude(16'hFFFF), .offset(16'd0),
    .waveform(waveform), .duty(16'h8000), .modparam(modparam),
    .bb_i(bb_i), .bb_q(bb_q), .dac_code(dac_code)
);

string module_name;
DataLogger #(.verbosity(LOG_LEVEL)) logger();
string str;
integer cycles, total;

initial begin
    $sformat(module_name, "%m");
    if (!$value$plusargs("in=%s", in_path) || !$value$plusargs("out=%s", out_path) ||
        !$value$plusargs("n=%d", n_in)) begin
        logger.error(module_name, "usage: +in=<audio.bin> +n=<samples> +out=<dac.bin> [+ftw= +wave= +modparam=]");
        `TEST_FAIL
    end
    if ($value$plusargs("ftw=%d", ftw_arg))           ftw      = ftw_arg;
    if ($value$plusargs("wave=%d", wave_arg))         waveform = wave_arg[3:0];
    if ($value$plusargs("modparam=%d", modparam_arg)) modparam = modparam_arg[15:0];

    fd_in = $fopen(in_path, "rb");
    if (fd_in == 0) begin $sformat(str, "cannot open %s", in_path); logger.error(module_name, str); `TEST_FAIL end
    rc = $fread(audio, fd_in);
    $fclose(fd_in);
    if (rc < 2 * n_in) begin $sformat(str, "read %0d bytes, expected %0d", rc, 2 * n_in); logger.error(module_name, str); `TEST_FAIL end
    fd_out = $fopen(out_path, "wb");
    if (fd_out == 0) begin $sformat(str, "cannot create %s", out_path); logger.error(module_name, str); `TEST_FAIL end

    $sformat(str, "%0d samples, ftw 0x%08h, waveform %0d, modparam %0d", n_in, ftw, waveform, modparam);
    logger.info(module_name, str);

    repeat (3) @(posedge clk);
    @(negedge clk); rst_n = 1; fifo_empty = 0;   // release reset away from the clock edge

    total = (n_in + TAIL) * INTERP;
    for (cycles = 0; cycles < total; cycles = cycles + 1) begin
        @(posedge clk); #2;
        $fwrite(fd_out, "%c%c", {2'b00, dac_code[13:8]}, dac_code[7:0]);
    end
    $fclose(fd_out);
    $sformat(str, "wrote %0d DAC codes to %s", total, out_path);
    logger.info(module_name, str);
    `TEST_PASS
end

endmodule
