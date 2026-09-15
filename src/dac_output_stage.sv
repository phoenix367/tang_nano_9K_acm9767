`include "timescale.v"
`include "acm9767_defs.vh"

// One DAC channel's output stage: registers the 14-bit data bus and forwards
// the sample clock to the AD9767 as CLKx and WRTx through two ODDR primitives
// (clean, glitch-free clocks on ordinary IOs).
//
// AD9767 dual-port mode: WRTx loads the channel input latch, CLKx moves it to
// the DAC latch, both on their rising edge. Data timing (tS >= 2 ns, tH >= 1.5
// ns) is relative to WRT, so WRT is the *inverted* clock (WRT_INVERT=1): it
// rises half a period after the data changes, ~9 ns of setup and hold at
// 54 MHz. CLK is forwarded *non-inverted* (CLK_INVERT=0): it rises half a
// period before WRT, well clear of the datasheet's forbidden window (CLK must
// not rise 0..2 ns after WRT), so driver and wire skew between the two leads
// cannot violate it; the DAC latch then takes the previous input-latch word,
// i.e. one extra sample of latency, which is irrelevant here. Driving both
// from the same edge (CLK_INVERT=1) is what most modules do internally but is
// only safe with well-matched traces.

module dac_output_stage #(
    parameter CLK_INVERT = 1'b0,
    parameter WRT_INVERT = 1'b1
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [13:0] code,
    output wire        da_clk,
    output wire        da_wrt,
    output reg  [13:0] da_data
);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) da_data <= `WRAP_SIM(#1) 14'h2000;
    else        da_data <= `WRAP_SIM(#1) code;
end

// ODDR: Q0 = D0 while clk is high, D1 while clk is low.
//   *_INVERT=0 -> D0=1, D1=0 -> replica of clk (rises on the rising edge).
//   *_INVERT=1 -> D0=0, D1=1 -> inverted clk (rises on the falling edge).
ODDR #(.TXCLK_POL(1'b0), .INIT(1'b0)) oddr_clk (
    .Q0(da_clk), .Q1(),
    .D0(~CLK_INVERT), .D1(CLK_INVERT), .TX(1'b0), .CLK(clk)
);
ODDR #(.TXCLK_POL(1'b0), .INIT(1'b0)) oddr_wrt (
    .Q0(da_wrt), .Q1(),
    .D0(~WRT_INVERT), .D1(WRT_INVERT), .TX(1'b0), .CLK(clk)
);

endmodule
