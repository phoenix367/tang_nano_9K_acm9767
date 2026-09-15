`include "timescale.v"
`include "acm9767_defs.vh"

// Toggle-handshake word synchroniser: moves a wide, quasi-static control word
// from the sending clock domain to the receiving one on demand.
//
//   sending side  : pulse s_commit while !s_busy. s_data must stay stable until
//                   s_busy drops (dac_regs guarantees this: it only reloads the
//                   committed bank when !s_busy).
//   receiving side: r_data updates and r_load pulses (one r_clk) when the word
//                   has crossed; the ack toggle then travels back to clear s_busy.
//
// The request toggle is 2-FF synchronised and edge-detected in the receiving
// domain, so by the time r_data samples s_data the word has been stable for at
// least two r_clk periods -- the usual mux-free multi-bit CDC pattern.

module cdc_commit_sync #(
    parameter integer WORD_WIDTH = 8
) (
    // sending domain
    input  wire                  s_clk,
    input  wire                  s_rst_n,
    input  wire                  s_commit,
    input  wire [WORD_WIDTH-1:0] s_data,
    output wire                  s_busy,
    // receiving domain
    input  wire                  r_clk,
    input  wire                  r_rst_n,
    output reg  [WORD_WIDTH-1:0] r_data,
    output reg                   r_load
);

// ---- sending side ----
reg req_tog;
reg ack_s0, ack_s1;
assign s_busy = req_tog ^ ack_s1;

always @(posedge s_clk or negedge s_rst_n) begin
    if (!s_rst_n) begin
        req_tog <= `WRAP_SIM(#1) 1'b0;
        ack_s0  <= `WRAP_SIM(#1) 1'b0;
        ack_s1  <= `WRAP_SIM(#1) 1'b0;
    end else begin
        ack_s0 <= `WRAP_SIM(#1) ack_tog;
        ack_s1 <= `WRAP_SIM(#1) ack_s0;
        if (s_commit && !s_busy)
            req_tog <= `WRAP_SIM(#1) ~req_tog;
    end
end

// ---- receiving side ----
reg req_r0, req_r1, req_r2;
reg ack_tog;

always @(posedge r_clk or negedge r_rst_n) begin
    if (!r_rst_n) begin
        req_r0  <= `WRAP_SIM(#1) 1'b0;
        req_r1  <= `WRAP_SIM(#1) 1'b0;
        req_r2  <= `WRAP_SIM(#1) 1'b0;
        ack_tog <= `WRAP_SIM(#1) 1'b0;
        r_data  <= `WRAP_SIM(#1) {WORD_WIDTH{1'b0}};
        r_load  <= `WRAP_SIM(#1) 1'b0;
    end else begin
        req_r0 <= `WRAP_SIM(#1) req_tog;
        req_r1 <= `WRAP_SIM(#1) req_r0;
        req_r2 <= `WRAP_SIM(#1) req_r1;
        r_load <= `WRAP_SIM(#1) req_r1 ^ req_r2;
        if (req_r1 ^ req_r2) begin
            r_data  <= `WRAP_SIM(#1) s_data;
            ack_tog <= `WRAP_SIM(#1) ~ack_tog;
        end
    end
end

endmodule
