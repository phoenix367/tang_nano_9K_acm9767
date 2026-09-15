/*
 * Copyright (c) 2026 Ivan Gubochkin
 * SPDX-License-Identifier: Apache-2.0
 *
 * Modbus RTU slave (FC03 read holding, FC06 write single, FC16 write multiple)
 * on the UART byte layer, with an external register backend (be_*): every
 * register access is issued on be_addr/be_wdata with be_req and completes on
 * be_ready (be_rdata valid that cycle). Frames end on 3.5 character times of
 * silence; CRC-16/Modbus is checked on RX and appended on TX. A frame with a
 * bad CRC, wrong address, parity error or overflow is dropped silently.
 *
 * Sized for the ASIC: MAX_FRAME bytes of request buffer in flip-flops (a
 * FC16 with up to (MAX_FRAME-9)/2 registers), MAX_QTY registers per FC03
 * response (payload in flip-flops). Plain Verilog-2005 port of the FPGA
 * project's modbus_rtu_slave.sv with the block RAMs removed.
 */

`default_nettype none

module modbus_rtu_slave #(
    parameter integer CLK_FREQ   = 50_000_000,
    parameter integer BAUD       = 1_000_000,
    parameter [7:0]   SLAVE_ADDR = 8'd7,
    parameter integer ADDR_LIMIT = 384,
    parameter integer MAX_FRAME  = 24,
    parameter integer MAX_QTY    = 4
) (
    input  wire        clk,
    input  wire        reset_n,
    input  wire [7:0]  rx_data,
    input  wire        rx_valid,
    input  wire        rx_parity_error,
    output reg  [7:0]  tx_data,
    output reg         tx_start,
    input  wire        tx_busy,
    output reg         be_req,
    output reg         be_we,
    output reg  [15:0] be_addr,
    output reg  [15:0] be_wdata,
    input  wire        be_ready,
    input  wire [15:0] be_rdata
);
    localparam integer FW       = $clog2(MAX_FRAME) + 1;
    localparam integer RESP_MAX = 3 + 2 * MAX_QTY + 2;
    localparam integer RW       = $clog2(RESP_MAX) + 1;
    localparam integer BIT_CYC  = CLK_FREQ / BAUD;
    localparam integer CHAR_CYC = 11 * BIT_CYC;
    localparam integer T35      = (7 * CHAR_CYC) / 2;
    localparam integer TW       = $clog2(T35 + 1);
    localparam integer QW       = $clog2(MAX_QTY);
    // sized copies of the integer parameters for width-exact compares
    /* verilator lint_off WIDTHTRUNC */
    localparam [FW-1:0] MAXF    = MAX_FRAME;
    localparam [TW-1:0] T35W    = T35;
    localparam [16:0]   ADDR_LIM = ADDR_LIMIT;
    localparam [15:0]   MAXQ    = MAX_QTY;
    /* verilator lint_on WIDTHTRUNC */

    localparam [7:0] EXC_ILLEGAL_FUNC = 8'h01, EXC_ILLEGAL_ADDR = 8'h02, EXC_ILLEGAL_VAL = 8'h03;

    localparam [3:0] S_RX = 4'd0, S_CHECK = 4'd1, S_RD_REQ = 4'd2, S_RD_CAP = 4'd3,
                     S_WR_REQ = 4'd4, S_WR_WAIT = 4'd5, S_TX_LOAD = 4'd6, S_TX_PEND = 4'd7,
                     S_TX_WAIT = 4'd8, S_DONE = 4'd9, S_DECIDE = 4'd10;

    reg [3:0]    state;
    reg [7:0]    frame [0:MAX_FRAME-1];
    reg [7:0]    resp_hdr [0:5];
    reg [15:0]   pay [0:MAX_QTY-1];
    reg [3:0]    hdr_len;
    reg [FW-1:0] flen;
    reg [RW-1:0] rlen, tidx;
    reg [15:0]   crc_acc, tx_crc;
    reg          tx_crc_en;
    reg [TW-1:0] t35_cnt;
    reg          frame_ovf, frame_perr;
    reg [15:0]   saddr, qty, cur, wval;
    reg [7:0]    bidx;
    reg          is_bcast, wr_multi;
    reg [7:0]    f_dev, f_func, qlast;
    reg          v_bad, v_qty0, v_addr_oor, v_saddr_oor, v_oversize, v_bc_bad;

    // ---- CRC-16/Modbus, parallel form (poly 0xA001 reflected): each next-state
    // bit is the XOR of a fixed subset of {byte, crc_in}; masks derived from the
    // bitwise shift/xor definition and verified against it ----
    function [15:0] crc16_update(input [15:0] crc_in, input [7:0] b);
        begin
            crc16_update[ 0] = ^({b, crc_in} & 24'hFF01FF);
            crc16_update[ 1] = ^({b, crc_in} & 24'h000200);
            crc16_update[ 2] = ^({b, crc_in} & 24'h000400);
            crc16_update[ 3] = ^({b, crc_in} & 24'h000800);
            crc16_update[ 4] = ^({b, crc_in} & 24'h001000);
            crc16_update[ 5] = ^({b, crc_in} & 24'h002000);
            crc16_update[ 6] = ^({b, crc_in} & 24'h014001);
            crc16_update[ 7] = ^({b, crc_in} & 24'h038003);
            crc16_update[ 8] = ^({b, crc_in} & 24'h060006);
            crc16_update[ 9] = ^({b, crc_in} & 24'h0C000C);
            crc16_update[10] = ^({b, crc_in} & 24'h180018);
            crc16_update[11] = ^({b, crc_in} & 24'h300030);
            crc16_update[12] = ^({b, crc_in} & 24'h600060);
            crc16_update[13] = ^({b, crc_in} & 24'hC000C0);
            crc16_update[14] = ^({b, crc_in} & 24'h7F007F);
            crc16_update[15] = ^({b, crc_in} & 24'hFF00FF);
        end
    endfunction

    // payload byte for the TX index (header bytes come from resp_hdr)
    wire [RW-1:0] pay_off = tidx - {{(RW-4){1'b0}}, hdr_len};
    wire [15:0]   pay_word = pay[pay_off[QW:1]];
    wire [7:0]    pay_byte = pay_off[0] ? pay_word[7:0] : pay_word[15:8];
    wire [FW-1:0] fidx = 7 + {bidx[FW-2:0], 1'b0};      // FC16 data byte index for register bidx

    integer i;
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            state <= S_RX; flen <= 0; rlen <= 0; tidx <= 0; hdr_len <= 4'd3;
            crc_acc <= 16'hFFFF; tx_crc <= 16'hFFFF; tx_crc_en <= 1'b0; t35_cnt <= 0;
            frame_ovf <= 1'b0; frame_perr <= 1'b0; tx_start <= 1'b0; tx_data <= 0;
            is_bcast <= 1'b0; wr_multi <= 1'b0; be_req <= 1'b0; be_we <= 1'b0;
            be_addr <= 0; be_wdata <= 0; saddr <= 0; qty <= 0; cur <= 0; wval <= 0;
            bidx <= 0; f_dev <= 0; f_func <= 0; qlast <= 0;
            v_bad <= 0; v_qty0 <= 0; v_addr_oor <= 0; v_saddr_oor <= 0; v_oversize <= 0; v_bc_bad <= 0;
            for (i = 0; i < 6; i = i + 1) resp_hdr[i] <= 0;
            for (i = 0; i < MAX_FRAME; i = i + 1) frame[i] <= 0;
            for (i = 0; i < MAX_QTY; i = i + 1) pay[i] <= 0;
        end else begin
            tx_start <= 1'b0;
            case (state)
                S_RX: begin
                    if (rx_valid) begin
                        t35_cnt <= 0;
                        crc_acc <= (flen == 0) ? crc16_update(16'hFFFF, rx_data) : crc16_update(crc_acc, rx_data);
                        if (rx_parity_error) frame_perr <= 1'b1;
                        if (flen < MAXF) begin frame[flen[FW-2:0]] <= rx_data; flen <= flen + 1'b1; end
                        else frame_ovf <= 1'b1;
                    end else if (flen != 0) begin
                        if (t35_cnt >= T35W) state <= S_CHECK;
                        else t35_cnt <= t35_cnt + 1'b1;
                    end
                end
                S_CHECK: begin
                    f_dev <= frame[0]; f_func <= frame[1];
                    saddr <= {frame[2], frame[3]}; qty <= {frame[4], frame[5]};
                    cur <= {frame[2], frame[3]}; wval <= {frame[4], frame[5]};
                    qlast <= frame[5] - 8'd1;
                    bidx <= 0; tidx <= 0; hdr_len <= 4'd3; tx_crc <= 16'hFFFF;
                    is_bcast <= (frame[0] == 8'h00);
                    v_bad <= frame_ovf || frame_perr || flen < 4 || crc_acc != 16'h0000 ||
                             (frame[0] != SLAVE_ADDR && frame[0] != 8'h00);
                    v_qty0 <= ({frame[4], frame[5]} == 16'd0);
                    v_addr_oor <= (({1'b0, frame[2], frame[3]} + {1'b0, frame[4], frame[5]}) > ADDR_LIM);
                    v_saddr_oor <= ({1'b0, frame[2], frame[3]} >= ADDR_LIM);
                    v_oversize <= ({frame[4], frame[5]} > MAXQ);
                    v_bc_bad <= (frame[6] != {frame[5][6:0], 1'b0}) || (frame[4] != 8'h00);
                    state <= S_DECIDE;
                end
                S_DECIDE: begin
                    if (v_bad) state <= S_DONE;
                    else begin
                        resp_hdr[0] <= f_dev;
                        case (f_func)
                            8'h03: begin
                                if (v_qty0 || v_oversize) begin
                                    resp_hdr[1] <= 8'h83; resp_hdr[2] <= EXC_ILLEGAL_VAL; rlen <= 3;
                                    state <= is_bcast ? S_DONE : S_TX_LOAD;
                                end else if (v_addr_oor) begin
                                    resp_hdr[1] <= 8'h83; resp_hdr[2] <= EXC_ILLEGAL_ADDR; rlen <= 3;
                                    state <= is_bcast ? S_DONE : S_TX_LOAD;
                                end else if (is_bcast) state <= S_DONE;
                                else begin
                                    resp_hdr[1] <= 8'h03; resp_hdr[2] <= {qty[6:0], 1'b0};
                                    rlen <= 3 + {qty[RW-2:0], 1'b0};
                                    state <= S_RD_REQ;
                                end
                            end
                            8'h06: begin
                                if (v_saddr_oor) begin
                                    resp_hdr[1] <= 8'h86; resp_hdr[2] <= EXC_ILLEGAL_ADDR; rlen <= 3;
                                    state <= is_bcast ? S_DONE : S_TX_LOAD;
                                end else begin
                                    resp_hdr[1] <= f_func; resp_hdr[2] <= saddr[15:8]; resp_hdr[3] <= saddr[7:0];
                                    resp_hdr[4] <= wval[15:8]; resp_hdr[5] <= wval[7:0];
                                    rlen <= 6; hdr_len <= 4'd6; wr_multi <= 1'b0; state <= S_WR_REQ;
                                end
                            end
                            8'h10: begin
                                // (MAX_QTY limits FC03 only; FC16 is bounded by the frame buffer)
                                if (v_qty0 || v_addr_oor || v_bc_bad) begin
                                    resp_hdr[1] <= 8'h90;
                                    resp_hdr[2] <= v_qty0 ? EXC_ILLEGAL_VAL : EXC_ILLEGAL_ADDR;
                                    rlen <= 3; state <= is_bcast ? S_DONE : S_TX_LOAD;
                                end else begin
                                    resp_hdr[1] <= 8'h10; resp_hdr[2] <= saddr[15:8]; resp_hdr[3] <= saddr[7:0];
                                    resp_hdr[4] <= wval[15:8]; resp_hdr[5] <= wval[7:0];
                                    rlen <= 6; hdr_len <= 4'd6; wr_multi <= 1'b1; state <= S_WR_REQ;
                                end
                            end
                            default: begin
                                resp_hdr[1] <= f_func | 8'h80; resp_hdr[2] <= EXC_ILLEGAL_FUNC; rlen <= 3;
                                state <= is_bcast ? S_DONE : S_TX_LOAD;
                            end
                        endcase
                    end
                end
                S_RD_REQ: begin be_req <= 1'b1; be_we <= 1'b0; be_addr <= cur; state <= S_RD_CAP; end
                S_RD_CAP: if (be_ready) begin
                    be_req <= 1'b0;
                    pay[bidx[QW-1:0]] <= be_rdata;
                    if (bidx == qlast) state <= S_TX_LOAD;
                    else begin bidx <= bidx + 1'b1; cur <= cur + 1'b1; state <= S_RD_REQ; end
                end
                S_WR_REQ: begin
                    be_req <= 1'b1; be_we <= 1'b1; be_addr <= cur;
                    be_wdata <= wr_multi ? {frame[fidx[FW-2:0]], frame[fidx[FW-2:0] + 1'b1]} : wval;
                    state <= S_WR_WAIT;
                end
                S_WR_WAIT: if (be_ready) begin
                    be_req <= 1'b0; be_we <= 1'b0;
                    if (!wr_multi || bidx == qlast) state <= is_bcast ? S_DONE : S_TX_LOAD;
                    else begin bidx <= bidx + 1'b1; cur <= cur + 1'b1; state <= S_WR_REQ; end
                end
                S_TX_LOAD: begin
                    if (tidx == rlen + 2) state <= S_DONE;
                    else if (!tx_busy) begin
                        if (tidx < {{(RW-4){1'b0}}, hdr_len}) begin tx_data <= resp_hdr[tidx[2:0]]; tx_crc_en <= 1'b1; end
                        else if (tidx < rlen) begin tx_data <= pay_byte; tx_crc_en <= 1'b1; end
                        else if (tidx == rlen) begin tx_data <= tx_crc[7:0]; tx_crc_en <= 1'b0; end
                        else begin tx_data <= tx_crc[15:8]; tx_crc_en <= 1'b0; end
                        tx_start <= 1'b1; state <= S_TX_PEND;
                    end
                end
                S_TX_PEND: begin
                    if (tx_crc_en) begin tx_crc <= crc16_update(tx_crc, tx_data); tx_crc_en <= 1'b0; end
                    if (tx_busy) state <= S_TX_WAIT;
                end
                S_TX_WAIT: if (!tx_busy) begin tidx <= tidx + 1'b1; state <= S_TX_LOAD; end
                S_DONE: begin
                    flen <= 0; t35_cnt <= 0; crc_acc <= 16'hFFFF;
                    frame_ovf <= 1'b0; frame_perr <= 1'b0; state <= S_RX;
                end
                default: state <= S_RX;
            endcase
        end
    end
endmodule
