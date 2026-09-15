/*
 * Copyright (c) 2026 Ivan Gubochkin
 * SPDX-License-Identifier: Apache-2.0
 *
 * Holding-register file behind modbus_rtu_slave's backend port, same map as
 * the FPGA project (README.md there) minus what the ASIC does not have:
 *
 *   0x00 ID (RO) 0x9767         0x01 VERSION (RO) 0x0201 (ASIC)
 *   0x02 STATUS (RO) bit1 commit busy   0x03 CONTROL (RW*) bit0 en, bit2 invert
 *   0x04 COMMIT (WO)            0x05/06 DAC_CLK_LO/HI (RO, from DAC_CLK_HZ)
 *   0x07 CHANNELS (RO) 1        0x08 FIFO_LEVEL (RO)  0x09 UNDERRUNS (RO/W1C)
 *   0x0A AUDIO_RATE (RO)        0x0B FIFO_DEPTH (RO) 16
 *   0x10.. channel 1: FTW_LO FTW_HI PHASE AMPLITUDE OFFSET WAVEFORM DUTY MODPARAM
 *   0x100..0x17F audio window: a write pushes one baseband sample (AM/FM/SSB)
 * Staged registers (*) become live on COMMIT, all in the same clock. No clock
 * crossing on the ASIC: the DDS runs on the same clock as the Modbus port.
 * The baseband FIFO is 16 x 16 bits of flip-flops, consumed at AUDIO_RATE
 * (a tick every CLK_HZ/AUDIO_RATE clocks); an empty FIFO yields 0 and counts
 * an underrun.
 */

`default_nettype none

module dac_regs #(
    parameter integer CLK_HZ     = 50_000_000,
    parameter integer AUDIO_RATE = 16_000
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        be_req,
    input  wire        be_we,
    input  wire [15:0] be_addr,
    input  wire [15:0] be_wdata,
    output reg         be_ready,
    output reg  [15:0] be_rdata,
    // live (committed) channel settings
    output reg         en,
    output reg         invert,
    output reg  [31:0] ftw,
    output reg  [15:0] phase_ofs,
    output reg  [15:0] amplitude,
    output reg  [15:0] offset,
    output reg  [3:0]  waveform,
    output reg  [15:0] duty,
    output reg  [15:0] modparam,
    output reg  signed [15:0] bb,
    output reg         bb_tick          // pulses one clock after bb updates
);

    localparam integer TICK = CLK_HZ / AUDIO_RATE;
    localparam [31:0] DAC_CLK_HZ = CLK_HZ;
    /* verilator lint_off WIDTHTRUNC */
    localparam [15:0] TICK_LAST = TICK - 1;
    localparam [15:0] AUDIO_RATE16 = AUDIO_RATE;
    /* verilator lint_on WIDTHTRUNC */

    // ---- staged bank ----
    reg        s_en, s_inv;
    reg [31:0] s_ftw;
    reg [15:0] s_phase, s_amp, s_off, s_duty, s_mod;
    reg [3:0]  s_wave;

    wire accept   = be_req && !be_ready;
    wire is_ch    = (be_addr[15:4] == 12'h001);           // 0x10..0x1F
    wire is_audio = (be_addr[15:7] == 9'd2);              // 0x100..0x17F
    wire [3:0] off = be_addr[3:0];

    // ---- baseband FIFO: 16 entries, flip-flops ----
    reg [15:0] fifo [0:15];
    reg [4:0]  wptr, rptr;
    wire [4:0] level = wptr - rptr;
    wire       full  = (level == 5'd16);
    wire       empty = (wptr == rptr);
    reg [15:0] underruns;
    reg [15:0] tick_cnt;
    wire tick = (tick_cnt == TICK_LAST);
    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tick_cnt <= 0; wptr <= 0; rptr <= 0; underruns <= 0; bb <= 0; bb_tick <= 1'b0;
            for (i = 0; i < 16; i = i + 1) fifo[i] <= 0;
        end else begin
            tick_cnt <= tick ? 16'd0 : tick_cnt + 1'b1;
            bb_tick  <= tick;
            if (accept && be_we && is_audio && !full) begin
                fifo[wptr[3:0]] <= be_wdata;
                wptr <= wptr + 1'b1;
            end
            if (tick) begin
                if (!empty) begin bb <= fifo[rptr[3:0]]; rptr <= rptr + 1'b1; end
                else begin bb <= 0; if (underruns != 16'hFFFF) underruns <= underruns + 1'b1; end
            end
            if (accept && be_we && be_addr == 16'h0009) underruns <= 0;
        end
    end

    // ---- register access ----
    reg commit_pending;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            be_ready <= 1'b0; be_rdata <= 0; commit_pending <= 1'b0;
            s_en <= 1'b0; s_inv <= 1'b0; s_ftw <= 0; s_phase <= 0; s_amp <= 16'hFFFF;
            s_off <= 0; s_wave <= 0; s_duty <= 16'h8000; s_mod <= 0;
            en <= 1'b0; invert <= 1'b0; ftw <= 0; phase_ofs <= 0; amplitude <= 16'hFFFF;
            offset <= 0; waveform <= 0; duty <= 16'h8000; modparam <= 0;
        end else begin
            be_ready <= accept;
            if (accept) begin
                be_rdata <= 16'h0000;
                if (is_ch) case (off)
                    4'd0: be_rdata <= s_ftw[15:0];
                    4'd1: be_rdata <= s_ftw[31:16];
                    4'd2: be_rdata <= s_phase;
                    4'd3: be_rdata <= s_amp;
                    4'd4: be_rdata <= s_off;
                    4'd5: be_rdata <= {12'd0, s_wave};
                    4'd6: be_rdata <= s_duty;
                    4'd7: be_rdata <= s_mod;
                    default: be_rdata <= 16'h0000;
                endcase
                else case (be_addr)
                    16'h0000: be_rdata <= 16'h9767;
                    16'h0001: be_rdata <= 16'h0201;
                    16'h0002: be_rdata <= {14'd0, commit_pending, 1'b1};   // "PLL locked": external clock
                    16'h0003: be_rdata <= {13'd0, s_inv, 1'b0, s_en};
                    16'h0004: be_rdata <= {15'd0, commit_pending};
                    16'h0005: be_rdata <= DAC_CLK_HZ[15:0];
                    16'h0006: be_rdata <= DAC_CLK_HZ[31:16];
                    16'h0007: be_rdata <= 16'd1;
                    16'h0008: be_rdata <= {11'd0, level};
                    16'h0009: be_rdata <= underruns;
                    16'h000A: be_rdata <= AUDIO_RATE16;
                    16'h000B: be_rdata <= 16'd16;
                    default:  be_rdata <= 16'h0000;
                endcase
            end
            if (accept && be_we) begin
                if (is_ch) case (off)
                    4'd0: s_ftw[15:0]  <= be_wdata;
                    4'd1: s_ftw[31:16] <= be_wdata;
                    4'd2: s_phase <= be_wdata;
                    4'd3: s_amp   <= be_wdata;
                    4'd4: s_off   <= be_wdata;
                    4'd5: s_wave  <= be_wdata[3:0];
                    4'd6: s_duty  <= be_wdata;
                    4'd7: s_mod   <= be_wdata;
                    default: ;
                endcase
                else case (be_addr)
                    16'h0003: begin s_en <= be_wdata[0]; s_inv <= be_wdata[2]; end
                    16'h0004: commit_pending <= 1'b1;
                    default: ;
                endcase
            end
            // commit: one clock after the request is accepted
            if (commit_pending) begin
                en <= s_en; invert <= s_inv; ftw <= s_ftw; phase_ofs <= s_phase;
                amplitude <= s_amp; offset <= s_off; waveform <= s_wave; duty <= s_duty;
                modparam <= s_mod;
                commit_pending <= 1'b0;
            end
        end
    end

endmodule
