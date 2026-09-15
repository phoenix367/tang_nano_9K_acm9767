`include "timescale.v"
`include "acm9767_defs.vh"

// Asynchronous-assert / synchronous-deassert reset bridge (2-FF). Feed it the
// raw async reset (button, PLL lock) and use rst_n inside the clk domain.

module reset_sync (
    input  wire clk,
    input  wire async_rst_n,
    output reg  rst_n
);

reg r0;

always @(posedge clk or negedge async_rst_n) begin
    if (!async_rst_n) begin
        r0    <= `WRAP_SIM(#1) 1'b0;
        rst_n <= `WRAP_SIM(#1) 1'b0;
    end else begin
        r0    <= `WRAP_SIM(#1) 1'b1;
        rst_n <= `WRAP_SIM(#1) r0;
    end
end

endmodule
