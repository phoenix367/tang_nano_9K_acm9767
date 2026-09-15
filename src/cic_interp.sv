`include "timescale.v"
`include "acm9767_defs.vh"

// 3-stage CIC interpolator, ratio R (one in_strobe every R clocks), input
// Q15. DC gain is R^2; the caller pre-scales by K so that the output can be
// taken at bits [OUT_SHIFT+15 : OUT_SHIFT] with unity overall gain. All stages
// are ACC_W bits two's complement: intermediate wrap-around is harmless in a
// CIC as long as the last stage can hold the true output, and ACC_W has 5 bits
// of headroom over 16 + 2*log2(R) for the saturation check.
//
// Images of the baseband spectrum around multiples of the audio rate are
// suppressed by sinc^3: about -40 dB for the first image at 16 kS/s with a
// 3 kHz audio bandwidth; passband droop -1.5 dB at 3 kHz.

module cic_interp #(
    parameter integer OUT_SHIFT = 23,
    parameter integer ACC_W     = 44
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               in_strobe,
    input  wire signed [15:0] in_data,
    output reg  signed [15:0] out_data
);

// ---- comb section at the input rate ----
reg signed [ACC_W-1:0] x_d, c1, c1_d, c2, c2_d, c3;
wire signed [ACC_W-1:0] x_ext = {{(ACC_W-16){in_data[15]}}, in_data};

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        x_d <= `WRAP_SIM(#1) 0; c1 <= `WRAP_SIM(#1) 0; c1_d <= `WRAP_SIM(#1) 0;
        c2  <= `WRAP_SIM(#1) 0; c2_d <= `WRAP_SIM(#1) 0; c3 <= `WRAP_SIM(#1) 0;
    end else if (in_strobe) begin
        x_d  <= `WRAP_SIM(#1) x_ext;
        c1   <= `WRAP_SIM(#1) x_ext - x_d;
        c1_d <= `WRAP_SIM(#1) c1;
        c2   <= `WRAP_SIM(#1) c1 - c1_d;
        c2_d <= `WRAP_SIM(#1) c2;
        c3   <= `WRAP_SIM(#1) c2 - c2_d;
    end
end

// c3 is valid 3 strobes after the input; present it to the integrators for
// exactly one clock per strobe (zero-stuffing), one clock after it updates.
reg strobe_d;
always @(posedge clk or negedge rst_n)
    if (!rst_n) strobe_d <= `WRAP_SIM(#1) 1'b0;
    else        strobe_d <= `WRAP_SIM(#1) in_strobe;

// ---- integrator section at the output rate ----
reg signed [ACC_W-1:0] i1, i2, i3;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        i1 <= `WRAP_SIM(#1) 0; i2 <= `WRAP_SIM(#1) 0; i3 <= `WRAP_SIM(#1) 0;
    end else begin
        i1 <= `WRAP_SIM(#1) i1 + (strobe_d ? c3 : {ACC_W{1'b0}});
        i2 <= `WRAP_SIM(#1) i2 + i1;
        i3 <= `WRAP_SIM(#1) i3 + i2;
    end
end

// ---- output slice with saturation ----
wire [ACC_W-1:OUT_SHIFT+15] top = i3[ACC_W-1:OUT_SHIFT+15];
wire in_range = (&top) | (~|top);
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)         out_data <= `WRAP_SIM(#1) 16'sd0;
    else if (in_range)  out_data <= `WRAP_SIM(#1) i3[OUT_SHIFT+15:OUT_SHIFT];
    else                out_data <= `WRAP_SIM(#1) i3[ACC_W-1] ? -16'sd32768 : 16'sd32767;
end

endmodule
