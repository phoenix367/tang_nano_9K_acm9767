`include "timescale.v"
`include "acm9767_defs.vh"

// One DAC channel's output stage: registers the data bus and forwards the
// sample clock to the AD9767 as CLKx and WRTx.
//
// AD9767 dual-port mode: WRTx loads the channel input latch, CLKx moves it to
// the DAC latch, both on their rising edge. Data timing (tS >= 2 ns, tH >= 1.5
// ns) is relative to WRT, so WRT is the *inverted* sample clock (WRT_INVERT=1):
// it rises half a sample after the data changes. CLK is forwarded
// *non-inverted* (CLK_INVERT=0): it rises half a sample before WRT, clear of
// the datasheet's forbidden window (CLK must not rise 0..2 ns after WRT), so
// driver and wire skew between the two leads cannot violate it; the DAC latch
// then takes the previous input-latch word, i.e. one extra sample of latency,
// which is irrelevant here. Driving both from the same edge (CLK_INVERT=1) is
// what most modules do internally but is only safe with well-matched traces.
//
// PHASES = 1 (one sample per clk): data registered on clk, CLK/WRT forwarded
//   through two ODDRs. Half a sample = half a clk period (9.26 ns at 54 MHz).
// PHASES = 2 (two samples per clk, code = {lane1, lane0}, lane 0 first): every
//   pin goes through an OSER4 clocked by fclk = 2 * clk (CLKDIV /2 of the same
//   rPLL output). OSER4 sends D0, D1, D2, D3 in that order, DDR on fclk, i.e.
//   4 slots of half an fclk period per clk: each data bit is sent twice
//   ({b0, b0, b1, b1}) so a sample lasts 2 slots = one fclk period (8.9 ns at
//   112.5 MS/s), and CLK / WRT are the 2-slot patterns {1,0} / {0,1} per sample,
//   giving the same edge relationships as the ODDR version at twice the rate.
//   Every pin uses the same serializer path, so the CLK/WRT-to-data alignment
//   is structural (no PLL phase relationship involved). All OSER4s leave reset
//   on the same fclk edge (rst_n is released synchronously to clk), which keeps
//   their 4-slot frames in step.

module dac_output_stage #(
    parameter integer PHASES     = 1,
    parameter         CLK_INVERT = 1'b0,
    parameter         WRT_INVERT = 1'b1
) (
    input  wire                  clk,     // sample clock / PHASES
    input  wire                  fclk,    // PHASES = 2: serializer clock (2 * clk); unused otherwise
    input  wire                  rst_n,
    input  wire [14*PHASES-1:0]  code,    // lane k at [14k +: 14]
    output wire                  da_clk,
    output wire                  da_wrt,
    output wire [13:0]           da_data
);

reg [14*PHASES-1:0] code_r;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) code_r <= `WRAP_SIM(#1) {PHASES{14'h2000}};
    else        code_r <= `WRAP_SIM(#1) code;
end

generate if (PHASES == 1) begin : g_oddr
    assign da_data = code_r;
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
end else begin : g_oser
    // OSER4 reset: async, active high; outputs low (clocks stopped) while held
    wire ser_rst = ~rst_n;
    genvar b;
    for (b = 0; b < 14; b = b + 1) begin : bit_
        OSER4 #(.GSREN("false"), .LSREN("true"), .HWL("false"), .TXCLK_POL(1'b0)) oser (
            .Q0(da_data[b]), .Q1(),
            .D0(code_r[b]), .D1(code_r[b]), .D2(code_r[14 + b]), .D3(code_r[14 + b]),
            .TX0(1'b0), .TX1(1'b0), .PCLK(clk), .FCLK(fclk), .RESET(ser_rst)
        );
    end
    // per sample: *_INVERT=0 -> {1,0} (rises at the data transition),
    //             *_INVERT=1 -> {0,1} (rises mid-sample)
    OSER4 #(.GSREN("false"), .LSREN("true"), .HWL("false"), .TXCLK_POL(1'b0)) oser_clk (
        .Q0(da_clk), .Q1(),
        .D0(~CLK_INVERT), .D1(CLK_INVERT), .D2(~CLK_INVERT), .D3(CLK_INVERT),
        .TX0(1'b0), .TX1(1'b0), .PCLK(clk), .FCLK(fclk), .RESET(ser_rst)
    );
    OSER4 #(.GSREN("false"), .LSREN("true"), .HWL("false"), .TXCLK_POL(1'b0)) oser_wrt (
        .Q0(da_wrt), .Q1(),
        .D0(~WRT_INVERT), .D1(WRT_INVERT), .D2(~WRT_INVERT), .D3(WRT_INVERT),
        .TX0(1'b0), .TX1(1'b0), .PCLK(clk), .FCLK(fclk), .RESET(ser_rst)
    );
end endgenerate

endmodule
