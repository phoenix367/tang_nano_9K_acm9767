`default_nettype none
`timescale 1ns / 1ps
/*
 * File-driven run of the ASIC modulator chain (baseband_dsp -> cic_interp x2
 * -> dds_channel) for scripts/ssb_loopback.py: audio samples come from a
 * binary file through a FIFO model, the DAC codes go to a binary file.
 * Plusargs as in the FPGA loopback: +in +n +out +ftw +wave +modparam.
 */
module tb_loopback;
    localparam integer CLK_HZ = 50_000_000;
    localparam integer AUDIO_RATE = 16_000;
    localparam integer INTERP = CLK_HZ / AUDIO_RATE;
    localparam integer TAIL = 60;

    reg clk = 0, rst_n = 0;
    always #10 clk = ~clk;

    string in_path, out_path;
    integer n_in, fd_in, fd_out, ftw_arg, wave_arg, modparam_arg, rc;
    reg [15:0] audio [0:262143];

    // audio tick + FIFO model
    reg [15:0] tick_cnt = 0;
    wire tick = (tick_cnt == INTERP - 1);
    reg signed [15:0] bb = 0;
    reg bb_tick = 0;
    integer sidx = 0;
    always @(posedge clk) begin
        tick_cnt <= tick ? 16'd0 : tick_cnt + 1'b1;
        bb_tick <= tick;
        if (tick) begin
            bb <= (sidx < n_in) ? $signed(audio[sidx]) : 16'sd0;
            sidx <= sidx + 1;
        end
    end

    reg [3:0]  waveform = 4'd5;
    reg [31:0] ftw = 32'h0800_0000;
    reg [15:0] modparam = 16'd0;
    wire [13:0] dac_code;
    dds_channel #(.SSB(1)) chan (
        .clk(clk), .rst_n(rst_n), .en(1'b1), .invert(1'b0),
        .ftw(ftw), .phase_ofs(16'd0), .amplitude(16'hFFFF), .offset(16'd0),
        .waveform(waveform), .duty(16'h8000), .modparam(modparam),
        .bb(bb), .bb_tick(bb_tick), .dac_code(dac_code)
    );

    integer cycles, total;
    initial begin
        if (!$value$plusargs("in=%s", in_path) || !$value$plusargs("out=%s", out_path) ||
            !$value$plusargs("n=%d", n_in)) begin
            $display("usage: +in=<audio.bin> +n=<samples> +out=<dac.bin> [+ftw= +wave= +modparam=]");
            $display("Test failed"); $finish;
        end
        if ($value$plusargs("ftw=%d", ftw_arg))           ftw      = ftw_arg;
        if ($value$plusargs("wave=%d", wave_arg))         waveform = wave_arg[3:0];
        if ($value$plusargs("modparam=%d", modparam_arg)) modparam = modparam_arg[15:0];
        fd_in = $fopen(in_path, "rb");
        rc = $fread(audio, fd_in); $fclose(fd_in);
        if (rc < 2 * n_in) begin $display("short read"); $display("Test failed"); $finish; end
        fd_out = $fopen(out_path, "wb");
        repeat (3) @(posedge clk); @(negedge clk); rst_n = 1;
        total = (n_in + TAIL) * INTERP;
        for (cycles = 0; cycles < total; cycles = cycles + 1) begin
            @(posedge clk); #1;
            $fwrite(fd_out, "%c%c", {2'b00, dac_code[13:8]}, dac_code[7:0]);
        end
        $fclose(fd_out);
        $display("wrote %0d DAC codes", total);
        $display("Test passed");
        $finish;
    end
endmodule
