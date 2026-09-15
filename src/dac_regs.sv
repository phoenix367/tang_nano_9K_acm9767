`include "timescale.v"
`include "acm9767_defs.vh"
`include "platform_config.vh"

// Holding-register file behind modbus_rtu_slave's external backend (be_*).
//
// Every register access from the Modbus FSM is answered one cycle later
// (be_ready pulses with be_rdata valid). Writes land in the *staged* bank; a
// write to REG_COMMIT copies the staged bank into the *committed* bank and
// hands it to cdc_commit_sync, which carries it into the dac_clk domain. That
// way a 32-bit tuning word split over two 16-bit registers never reaches the
// DDS half-updated, and all channel settings switch on the same DAC sample.
//
// Register map (16-bit holding registers, see README.md):
//   0x00 ID (RO) 0x9767            0x01 VERSION (RO)
//   0x02 STATUS (RO)               0x03 CONTROL (RW, committed)
//   0x04 COMMIT (WO, any write)    0x05/0x06 DAC_CLK_LO/HI (RO, Hz)
//   0x07 CHANNELS (RO, 1 or 2)     0x08 FIFO_LEVEL (RO)  0x09 UNDERRUNS (RO, write clears)
//   0x0A AUDIO_RATE (RO, Hz)       0x0B FIFO_DEPTH (RO)
//   0x0100..0x017F audio FIFO write window (WO): every write pushes one sample
//   0x10.. channel 1, 0x20.. channel 2: FTW_LO FTW_HI PHASE AMPLITUDE OFFSET
//   WAVEFORM DUTY MODPARAM
// Unmapped addresses inside ADDR_LIMIT read 0 and ignore writes.

module dac_regs (
    input  wire        clk,
    input  wire        rst_n,

    // backend handshake from modbus_rtu_slave
    input  wire        be_req,
    input  wire        be_we,
    input  wire [15:0] be_addr,
    input  wire [15:0] be_wdata,
    output reg         be_ready,
    output reg  [15:0] be_rdata,

    // status inputs (already in the clk domain)
    input  wire        pll_lock,
    input  wire        cdc_busy,

    // audio FIFO (async_fifo write side, clk domain) + underrun toggle from dac_clk
    output reg         fifo_wr,
    output reg  [15:0] fifo_wdata,
    input  wire [`PLATFORM_SSB_FIFO_LOG2:0] fifo_level,
    input  wire        underrun_tog,

    // committed snapshot -> cdc_commit_sync
    output reg  [`COMMIT_WORD_W-1:0] commit_word,
    output reg                       commit_pulse,
    output wire                      commit_busy,   // pending or in flight
    // committed CONTROL bits, for the LEDs
    output wire [`CTRL_WORD_W-1:0]   ctrl_committed
);

localparam [31:0] DAC_CLK_HZ = `PLATFORM_DAC_CLK_HZ;

// ---- staged registers ----
reg [`CTRL_WORD_W-1:0] ctrl;
reg [31:0] ftw       [0:1];
reg [15:0] phase     [0:1];
reg [15:0] amplitude [0:1];
reg [15:0] offset    [0:1];
reg [3:0]  waveform  [0:1];
reg [15:0] duty      [0:1];
reg [15:0] modparam  [0:1];

reg commit_pending;
assign commit_busy    = commit_pending | cdc_busy;
assign ctrl_committed = commit_word[`COMMIT_WORD_W-1 -: `CTRL_WORD_W];

// address decode helpers
// Address decode by bit slicing (the map is aligned for it, see acm9767_defs.vh):
//   channel blocks 0x10..0x1F / 0x20..0x2F -> be_addr[15:5] == 0 and be_addr[4] ^ be_addr[5]
//   audio window   0x100..0x17F           -> be_addr[15:7] == 2
// No comparators or subtractors in the path to be_rdata / the staged registers.
wire        is_ch     = (be_addr[15:6] == 10'd0) && (be_addr[5] ^ be_addr[4]);
wire        ch_sel    = be_addr[5];                          // 0 = ch1 (0x1x), 1 = ch2 (0x2x)
wire [3:0]  ch_off    = be_addr[3:0];
wire        accept    = be_req && !be_ready;   // one-cycle accept per request
wire        is_audio  = (be_addr[15:7] == 9'd2);

// underrun events: toggle from the dac domain, 2-FF synced, edge-counted
reg  ur_s0, ur_s1, ur_s2;
reg  [15:0] underruns;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        ur_s0 <= `WRAP_SIM(#1) 1'b0; ur_s1 <= `WRAP_SIM(#1) 1'b0; ur_s2 <= `WRAP_SIM(#1) 1'b0;
        underruns <= `WRAP_SIM(#1) 16'd0;
    end else begin
        ur_s0 <= `WRAP_SIM(#1) underrun_tog;
        ur_s1 <= `WRAP_SIM(#1) ur_s0;
        ur_s2 <= `WRAP_SIM(#1) ur_s1;
        if (accept && be_we && be_addr == `REG_UNDERRUNS)
            underruns <= `WRAP_SIM(#1) 16'd0;
        else if ((ur_s1 ^ ur_s2) && underruns != 16'hFFFF)
            underruns <= `WRAP_SIM(#1) underruns + 16'd1;
    end
end

// packed channel word
function [`CH_WORD_W-1:0] pack_ch(input integer c);
    begin
        pack_ch = {ftw[c], phase[c], amplitude[c], offset[c], waveform[c], duty[c], modparam[c]};
    end
endfunction

integer i;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        be_ready       <= `WRAP_SIM(#1) 1'b0;
        be_rdata       <= `WRAP_SIM(#1) 16'h0000;
        fifo_wr        <= `WRAP_SIM(#1) 1'b0;
        fifo_wdata     <= `WRAP_SIM(#1) 16'h0000;
        ctrl           <= `WRAP_SIM(#1) {`CTRL_WORD_W{1'b0}};
        commit_pending <= `WRAP_SIM(#1) 1'b0;
        commit_pulse   <= `WRAP_SIM(#1) 1'b0;
        commit_word    <= `WRAP_SIM(#1) {`COMMIT_WORD_W{1'b0}};
        for (i = 0; i < 2; i = i + 1) begin
            ftw[i]       <= `WRAP_SIM(#1) 32'd0;
            phase[i]     <= `WRAP_SIM(#1) 16'd0;
            amplitude[i] <= `WRAP_SIM(#1) 16'hFFFF;   // unity gain
            offset[i]    <= `WRAP_SIM(#1) 16'd0;
            waveform[i]  <= `WRAP_SIM(#1) `WAVE_SINE;
            duty[i]      <= `WRAP_SIM(#1) 16'h8000;   // 50 %
            modparam[i]  <= `WRAP_SIM(#1) 16'd0;
        end
    end else begin
        be_ready     <= `WRAP_SIM(#1) accept;
        commit_pulse <= `WRAP_SIM(#1) 1'b0;
        fifo_wr      <= `WRAP_SIM(#1) accept && be_we && is_audio;
        fifo_wdata   <= `WRAP_SIM(#1) be_wdata;

        // ---- read mux (valid together with be_ready) ----
        if (accept) begin
            be_rdata <= `WRAP_SIM(#1) 16'h0000;
            if (is_ch) begin
                case ({12'd0, ch_off})
                    `REG_CH_FTW_LO:    be_rdata <= `WRAP_SIM(#1) ftw[ch_sel][15:0];
                    `REG_CH_FTW_HI:    be_rdata <= `WRAP_SIM(#1) ftw[ch_sel][31:16];
                    `REG_CH_PHASE:     be_rdata <= `WRAP_SIM(#1) phase[ch_sel];
                    `REG_CH_AMPLITUDE: be_rdata <= `WRAP_SIM(#1) amplitude[ch_sel];
                    `REG_CH_OFFSET:    be_rdata <= `WRAP_SIM(#1) offset[ch_sel];
                    `REG_CH_WAVEFORM:  be_rdata <= `WRAP_SIM(#1) {12'd0, waveform[ch_sel]};
                    `REG_CH_DUTY:      be_rdata <= `WRAP_SIM(#1) duty[ch_sel];
                    `REG_CH_MODPARAM:  be_rdata <= `WRAP_SIM(#1) modparam[ch_sel];
                    default:           be_rdata <= `WRAP_SIM(#1) 16'h0000;
                endcase
            end else begin
                case (be_addr)
                    `REG_ID:         be_rdata <= `WRAP_SIM(#1) `ACM9767_ID;
                    `REG_VERSION:    be_rdata <= `WRAP_SIM(#1) `ACM9767_VERSION;
                    `REG_STATUS:     be_rdata <= `WRAP_SIM(#1) {14'd0, commit_busy, pll_lock};
                    `REG_CONTROL:    be_rdata <= `WRAP_SIM(#1) {{(16-`CTRL_WORD_W){1'b0}}, ctrl};
                    `REG_COMMIT:     be_rdata <= `WRAP_SIM(#1) {15'd0, commit_busy};
                    `REG_DAC_CLK_LO: be_rdata <= `WRAP_SIM(#1) DAC_CLK_HZ[15:0];
                    `REG_DAC_CLK_HI: be_rdata <= `WRAP_SIM(#1) DAC_CLK_HZ[31:16];
                    `REG_CHANNELS:   be_rdata <= `WRAP_SIM(#1) `PLATFORM_DAC_CHANNELS;
                    `REG_FIFO_LEVEL: be_rdata <= `WRAP_SIM(#1) fifo_level;
                    `REG_UNDERRUNS:  be_rdata <= `WRAP_SIM(#1) underruns;
                    `REG_AUDIO_RATE: be_rdata <= `WRAP_SIM(#1) `PLATFORM_SSB_AUDIO_RATE_HZ;
                    `REG_FIFO_DEPTH: be_rdata <= `WRAP_SIM(#1) `PLATFORM_SSB_FIFO_DEPTH;
                    default:         be_rdata <= `WRAP_SIM(#1) 16'h0000;
                endcase
            end
        end

        // ---- writes into the staged bank ----
        if (accept && be_we) begin
            if (is_ch) begin
                case ({12'd0, ch_off})
                    `REG_CH_FTW_LO:    ftw[ch_sel][15:0]  <= `WRAP_SIM(#1) be_wdata;
                    `REG_CH_FTW_HI:    ftw[ch_sel][31:16] <= `WRAP_SIM(#1) be_wdata;
                    `REG_CH_PHASE:     phase[ch_sel]      <= `WRAP_SIM(#1) be_wdata;
                    `REG_CH_AMPLITUDE: amplitude[ch_sel]  <= `WRAP_SIM(#1) be_wdata;
                    `REG_CH_OFFSET:    offset[ch_sel]     <= `WRAP_SIM(#1) be_wdata;
                    `REG_CH_WAVEFORM:  waveform[ch_sel]   <= `WRAP_SIM(#1) be_wdata[3:0];
                    `REG_CH_DUTY:      duty[ch_sel]       <= `WRAP_SIM(#1) be_wdata;
                    `REG_CH_MODPARAM:  modparam[ch_sel]   <= `WRAP_SIM(#1) be_wdata;
                    default: ;
                endcase
            end else begin
                case (be_addr)
                    `REG_CONTROL: ctrl           <= `WRAP_SIM(#1) be_wdata[`CTRL_WORD_W-1:0];
                    `REG_COMMIT:  commit_pending <= `WRAP_SIM(#1) 1'b1;
                    default: ;
                endcase
            end
        end

        // ---- hand the staged bank over once the previous commit has landed ----
        if (commit_pending && !cdc_busy && !commit_pulse) begin
            commit_word    <= `WRAP_SIM(#1) {ctrl, pack_ch(1), pack_ch(0)};
            commit_pulse   <= `WRAP_SIM(#1) 1'b1;
            commit_pending <= `WRAP_SIM(#1) 1'b0;
        end
    end
end

endmodule
