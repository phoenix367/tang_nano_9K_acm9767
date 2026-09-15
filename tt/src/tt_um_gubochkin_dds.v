/*
 * Copyright (c) 2026 Ivan Gubochkin
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tiny Tapeout top: single-channel DDS signal generator / SSB-AM-FM modulator
 * for a 14-bit parallel DAC (AD9767 class), controlled over Modbus RTU on a
 * UART. Everything runs on the Tiny Tapeout clock (50 MHz nominal = the DAC
 * sample rate; any 10..50 MHz works, the host reads the rate from registers
 * 0x05/06 so set CLK_HZ to match the board clock).
 *
 *   ui_in[0]     UART RX (host -> chip), 1 Mbaud 8-E-1 @ 50 MHz
 *   ui_in[7:1]   unused
 *   uo_out[7:0]  DAC data bits 13..6 (bit 13 = MSB, straight binary)
 *   uio[5:0]     DAC data bits 5..0                 (outputs)
 *   uio[6]       DAC CLK: a copy of clk             (output)
 *   uio[7]       UART TX (chip -> host)             (output)
 *   DAC WRT: tie to CLK on the board (or leave the module's internal tie).
 *
 * Data changes on the rising clk edge; the DAC latches on the rising edge of
 * its CLK, so with CLK = clk the data must be sampled by the *next* edge -- at
 * 50 MHz that leaves ~20 ns minus the pad delays for setup and a full period
 * of hold, which the AD9767 (2.0 / 1.5 ns) accepts comfortably.
 */

`default_nettype none

module tt_um_gubochkin_dds #(
    parameter integer CLK_HZ = 50_000_000,
    parameter         SSB    = 1            // 1: SSB modulator included (8x4 tiles); 0: AM/FM/DDS only (fits 8x2)
) (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
);
    // ---- UART + Modbus ----
    wire [7:0] rx_data, tx_data;
    wire       rx_valid, rx_perr, tx_start, tx_busy, uart_tx;
    uart #(.CLK_FREQ(CLK_HZ), .BAUD(1_000_000)) uart_inst (
        .clk(clk), .reset_n(rst_n),
        .tx_data(tx_data), .tx_start(tx_start), .tx_busy(tx_busy), .tx(uart_tx),
        .rx(ui_in[0]), .rx_data(rx_data), .rx_valid(rx_valid), .rx_parity_error(rx_perr)
    );

    wire        be_req, be_we, be_ready;
    wire [15:0] be_addr, be_wdata, be_rdata;
    modbus_rtu_slave #(.CLK_FREQ(CLK_HZ), .BAUD(1_000_000), .SLAVE_ADDR(8'd7),
                       .ADDR_LIMIT(384), .MAX_FRAME(24), .MAX_QTY(4)) modbus_inst (
        .clk(clk), .reset_n(rst_n),
        .rx_data(rx_data), .rx_valid(rx_valid), .rx_parity_error(rx_perr),
        .tx_data(tx_data), .tx_start(tx_start), .tx_busy(tx_busy),
        .be_req(be_req), .be_we(be_we), .be_addr(be_addr), .be_wdata(be_wdata),
        .be_ready(be_ready), .be_rdata(be_rdata)
    );

    // ---- registers + baseband FIFO ----
    wire        en, invert;
    wire [31:0] ftw;
    wire [15:0] phase_ofs, amplitude, offset, duty, modparam;
    wire [3:0]  waveform;
    wire signed [15:0] bb;
    wire        bb_tick;
    dac_regs #(.CLK_HZ(CLK_HZ), .AUDIO_RATE(16_000)) regs (
        .clk(clk), .rst_n(rst_n),
        .be_req(be_req), .be_we(be_we), .be_addr(be_addr), .be_wdata(be_wdata),
        .be_ready(be_ready), .be_rdata(be_rdata),
        .en(en), .invert(invert), .ftw(ftw), .phase_ofs(phase_ofs), .amplitude(amplitude),
        .offset(offset), .waveform(waveform), .duty(duty), .modparam(modparam), .bb(bb), .bb_tick(bb_tick)
    );

    // ---- DDS ----
    wire [13:0] dac_code;
    dds_channel #(.SSB(SSB)) dds (
        .clk(clk), .rst_n(rst_n), .en(en), .invert(invert),
        .ftw(ftw), .phase_ofs(phase_ofs), .amplitude(amplitude), .offset(offset),
        .waveform(waveform), .duty(duty), .modparam(modparam), .bb(bb), .bb_tick(bb_tick), .dac_code(dac_code)
    );

    // ---- pins ----
    assign uo_out  = dac_code[13:6];
    assign uio_out = {uart_tx, clk, dac_code[5:0]};
    assign uio_oe  = 8'hFF;

    wire _unused = &{ena, ui_in[7:1], uio_in, 1'b0};
endmodule
