`default_nettype none
`timescale 1ns / 1ps
// Sweeps the full 16-bit phase through sine_interp and writes the phase
// being applied and the current outputs per clock; test_sine.py finds the
// pipeline lag and checks every output against round(8191*sin/cos).
module tb_sine;
    reg clk = 0, rst_n = 0;
    reg [15:0] phase = 0;
    wire signed [13:0] s, c;
    sine_interp dut (.clk(clk), .rst_n(rst_n), .phase(phase), .sin_o(s), .cos_o(c));
    always #10 clk = ~clk;
    integer fd, n;
    initial begin
        fd = $fopen("sine_out.txt", "w");
        repeat (3) @(posedge clk); @(negedge clk); rst_n = 1;
        for (n = 0; n < 65536 + 40; n = n + 1) begin
            @(negedge clk);
            phase = n[15:0];
            // write the raw stream; test_sine.py aligns it to the phase sequence
            $fwrite(fd, "%0d %0d %0d\n", phase, s, c);
        end
        $fclose(fd);
        $finish;
    end
endmodule
