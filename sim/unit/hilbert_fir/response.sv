`include "timescale.v"
`include "acm9767_defs.vh"
`include "platform_config.vh"
`include "svlogger.sv"
`include "test_utils.sv"
`include "test_config.sv"

// Unit test for hilbert_fir with the real 127-tap coefficients and K = 0x7FFF
// (unity gain): a 20000-amplitude tone at 500 / 1000 / 3000 Hz (16 kS/s) must
// come out as i = delayed tone and q = its Hilbert transform (sin -> -cos),
// with RMS error below 1 % of the amplitude once the delay line is full.

module main();

localparam LOG_LEVEL = `DEFAULT_LOG_LEVEL;
localparam integer TAPS = `PLATFORM_SSB_HILBERT_TAPS;
localparam integer CENTER = (TAPS - 1) / 2;
localparam real FS = 16000.0;
localparam real AMP = 20000.0;

reg clk = 0, rst_n = 0;
reg in_strobe = 0;
reg signed [15:0] in_data = 0;
wire out_strobe;
wire signed [15:0] i_out, q_out;

string module_name;
DataLogger #(.verbosity(LOG_LEVEL)) logger();

hilbert_fir #(.TAPS(TAPS), .K(16'h7FFF)) dut (
    .clk(clk), .rst_n(rst_n), .in_strobe(in_strobe), .in_data(in_data),
    .out_strobe(out_strobe), .i_out(i_out), .q_out(q_out)
);
always #9.26 clk = ~clk;

integer errors, n, cnt;
real f, ei, eq, ex_i, ex_q, rms_i, rms_q;
string str;

task automatic run_tone(input real freq);
    begin
        rms_i = 0; rms_q = 0; cnt = 0;
        for (n = 0; n < 400; n = n + 1) begin
            @(negedge clk);
            in_data = $rtoi(AMP * $sin(2.0 * 3.14159265358979 * freq * n / FS));
            in_strobe = 1;
            @(negedge clk); in_strobe = 0;
            @(posedge out_strobe); #2;
            if (n >= TAPS + 10) begin
                ex_i =  AMP * $sin(2.0 * 3.14159265358979 * freq * (n - CENTER) / FS);
                ex_q = -AMP * $cos(2.0 * 3.14159265358979 * freq * (n - CENTER) / FS);
                ei = i_out - ex_i; eq = q_out - ex_q;
                rms_i = rms_i + ei * ei; rms_q = rms_q + eq * eq;
                cnt = cnt + 1;
            end
        end
        rms_i = $sqrt(rms_i / cnt); rms_q = $sqrt(rms_q / cnt);
        $sformat(str, "%0.0f Hz: rms error i=%0.1f q=%0.1f LSB (of %0.0f)", freq, rms_i, rms_q, AMP);
        if (rms_i > AMP * 0.01 || rms_q > AMP * 0.01) begin logger.error(module_name, str); errors++; end
        else logger.info(module_name, str);
    end
endtask

initial begin
    errors = 0;
`ifdef ENABLE_DUMPVARS
    $dumpvars(0, main);
`endif
    $sformat(module_name, "%m");
    logger.info(module_name, " << Starting the Simulation >>");
    repeat (3) @(posedge clk); rst_n = 1; repeat (2) @(posedge clk);

    run_tone(1000.0);
    run_tone(3000.0);
    run_tone(500.0);

    if (errors == 0) begin logger.info(module_name, "hilbert_fir OK"); `TEST_PASS end
    else `TEST_FAIL
end

always #40000000 begin logger.error(module_name, "System hangs"); `TEST_FAIL end
endmodule
