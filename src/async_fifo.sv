`include "timescale.v"
`include "acm9767_defs.vh"

// Dual-clock FIFO (Cummings-style gray-code pointers) for the audio samples:
// written from sys_clk by dac_regs, read from dac_clk by ssb_baseband.
//
//   write side : wr_en pushes wdata when !wfull; wlevel = samples buffered as
//                seen from the write clock, two cycles stale (the host reads it
//                through Modbus).
//   read side  : rd_en pops when !rempty; rdata is valid the *next* rclk.
// The two sides have independent resets (sys button vs. PLL-lock-gated dac
// reset); resetting only the read side re-reads from slot 0, which is harmless
// at power-up and the only time it happens in practice.

module async_fifo #(
    parameter integer WIDTH = 16,
    parameter integer DEPTH_LOG2 = 11
) (
    input  wire                  wclk,
    input  wire                  wrst_n,
    input  wire                  wr_en,
    input  wire [WIDTH-1:0]      wdata,
    output wire                  wfull,
    output wire [DEPTH_LOG2:0]   wlevel,

    input  wire                  rclk,
    input  wire                  rrst_n,
    input  wire                  rd_en,
    output reg  [WIDTH-1:0]      rdata,
    output wire                  rempty
);

localparam integer AW = DEPTH_LOG2;

reg [WIDTH-1:0] mem [0:(1<<AW)-1];

// ---- write side ----
reg  [AW:0] wbin, wgray;
reg  [AW:0] rgray_w0, rgray_w1;     // rgray synced into wclk
wire [AW:0] wbin_next  = wbin + (wr_en && !wfull);
wire [AW:0] wgray_next = (wbin_next >> 1) ^ wbin_next;

function [AW:0] gray2bin(input [AW:0] g);
    integer i;
    begin
        gray2bin[AW] = g[AW];
        for (i = AW - 1; i >= 0; i = i - 1) gray2bin[i] = gray2bin[i+1] ^ g[i];
    end
endfunction

assign wfull  = (wgray == {~rgray_w1[AW:AW-1], rgray_w1[AW-2:0]});

// Level readout, pipelined in two stages (gray->binary chain, then the
// subtraction) so the host-visible level is a short path; it lags by two
// wclk cycles, which is immaterial for pacing a 16 kS/s stream.
reg [AW:0] rbin_w, wlevel_r;
assign wlevel = wlevel_r;

always @(posedge wclk or negedge wrst_n) begin
    if (!wrst_n) begin
        wbin     <= `WRAP_SIM(#1) 0;
        wgray    <= `WRAP_SIM(#1) 0;
        rgray_w0 <= `WRAP_SIM(#1) 0;
        rgray_w1 <= `WRAP_SIM(#1) 0;
        rbin_w   <= `WRAP_SIM(#1) 0;
        wlevel_r <= `WRAP_SIM(#1) 0;
    end else begin
        wbin     <= `WRAP_SIM(#1) wbin_next;
        wgray    <= `WRAP_SIM(#1) wgray_next;
        rgray_w0 <= `WRAP_SIM(#1) rgray;
        rgray_w1 <= `WRAP_SIM(#1) rgray_w0;
        rbin_w   <= `WRAP_SIM(#1) gray2bin(rgray_w1);
        wlevel_r <= `WRAP_SIM(#1) wbin - rbin_w;
    end
end

always @(posedge wclk)
    if (wr_en && !wfull) mem[wbin[AW-1:0]] <= wdata;

// ---- read side ----
reg  [AW:0] rbin, rgray;
reg  [AW:0] wgray_r0, wgray_r1;     // wgray synced into rclk
wire [AW:0] rbin_next  = rbin + (rd_en && !rempty);
wire [AW:0] rgray_next = (rbin_next >> 1) ^ rbin_next;
assign rempty = (rgray == wgray_r1);

always @(posedge rclk or negedge rrst_n) begin
    if (!rrst_n) begin
        rbin     <= `WRAP_SIM(#1) 0;
        rgray    <= `WRAP_SIM(#1) 0;
        wgray_r0 <= `WRAP_SIM(#1) 0;
        wgray_r1 <= `WRAP_SIM(#1) 0;
    end else begin
        rbin     <= `WRAP_SIM(#1) rbin_next;
        rgray    <= `WRAP_SIM(#1) rgray_next;
        wgray_r0 <= `WRAP_SIM(#1) wgray;
        wgray_r1 <= `WRAP_SIM(#1) wgray_r0;
    end
end

always @(posedge rclk)
    if (rd_en && !rempty) rdata <= `WRAP_SIM(#1) mem[rbin[AW-1:0]];

endmodule
