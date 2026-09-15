`ifndef __ACM9767_DEFS_VH__
`define __ACM9767_DEFS_VH__

// `WRAP_SIM(x)` expands to `x` under Icarus and to nothing under synthesis. The
// RTL uses it to add a `#1` after every non-blocking assignment so testbenches
// can sample "after the edge" without racing the NBA (see CLAUDE.md).
`ifndef WRAP_SIM
`define WRAP_SIM(x) \
    `ifdef __ICARUS__ \
        x \
    `endif
`endif

// ---- Modbus holding-register map (see README.md "Register map") ----
`define REG_ID            16'h0000
`define REG_VERSION       16'h0001
`define REG_STATUS        16'h0002
`define REG_CONTROL       16'h0003
`define REG_COMMIT        16'h0004
`define REG_DAC_CLK_LO    16'h0005
`define REG_DAC_CLK_HI    16'h0006
`define REG_CHANNELS      16'h0007   // RO: number of wired DAC channels (1 or 2)
`define REG_FIFO_LEVEL    16'h0008   // RO: audio samples waiting in the FIFO
`define REG_UNDERRUNS     16'h0009   // RO: baseband ticks with an empty FIFO; any write clears
`define REG_AUDIO_RATE    16'h000A   // RO: baseband sample rate, Hz
`define REG_FIFO_DEPTH    16'h000B   // RO: FIFO capacity in samples
// Audio FIFO write window: any FC06/FC10 write to 0x0100..0x017F pushes one
// signed 16-bit sample (FC10 with consecutive addresses streams a block).
`define REG_AUDIO_BASE    16'h0100
`define REG_AUDIO_END     16'h017F
`define REG_CH_BASE       16'h0010   // channel n block = REG_CH_BASE + n*REG_CH_STRIDE (dac_regs decodes by bit slice: keep 0x10/0x20)
`define REG_CH_STRIDE     16'h0010
`define REG_CH_FTW_LO     16'h0000
`define REG_CH_FTW_HI     16'h0001
`define REG_CH_PHASE      16'h0002
`define REG_CH_AMPLITUDE  16'h0003
`define REG_CH_OFFSET     16'h0004
`define REG_CH_WAVEFORM   16'h0005
`define REG_CH_DUTY       16'h0006
`define REG_CH_MODPARAM   16'h0007   // AM depth (Q16, 0xFFFF = 100 %) or FM deviation (x256 FTW units)

`define ACM9767_ID        16'h9767
`define ACM9767_VERSION   16'h0100

// CONTROL register bits
`define CTRL_CH1_EN       0
`define CTRL_CH2_EN       1
`define CTRL_CH1_INVERT   2
`define CTRL_CH2_INVERT   3

// STATUS register bits
`define STAT_PLL_LOCK     0
`define STAT_COMMIT_BUSY  1

// ---- packed per-channel control word handed from the sys_clk register file to
// the dac_clk DDS through cdc_commit_sync. Flat vectors (not a packed struct)
// so the same slices work under Icarus and Gowin synthesis.
//   [115:84] ftw       32-bit phase increment (f_out = ftw * f_dac / 2^32)
//   [83:68]  phase     16-bit phase offset (2*pi / 65536 units)
//   [67:52]  amplitude 16-bit unsigned, gain = (amplitude+1)/65536, 0xFFFF = unity
//   [51:36]  offset    16-bit signed DC offset in DAC LSBs (saturating)
//   [35:32]  waveform  WAVE_* code
//   [31:16]  duty      square-wave high threshold (0x8000 = 50 %)
//   [15:0]   modparam  AM depth (Q16) / FM deviation (modparam*256 FTW at full-scale audio)
`define CH_WORD_W         116
`define CH_FTW(w)         w[115:84]
`define CH_PHASE(w)       w[83:68]
`define CH_AMPLITUDE(w)   w[67:52]
`define CH_OFFSET(w)      w[51:36]
`define CH_WAVEFORM(w)    w[35:32]
`define CH_DUTY(w)        w[31:16]
`define CH_MODPARAM(w)    w[15:0]
// Whole commit snapshot: {control[3:0], ch2 word, ch1 word}
`define CTRL_WORD_W       4
`define COMMIT_WORD_W     (`CTRL_WORD_W + 2*`CH_WORD_W)

// WAVEFORM register values (4 bits). 5..8 modulate the streamed audio (bb_i/bb_q)
// onto this channel's carrier (FTW).
`define WAVE_SINE         4'd0
`define WAVE_TRIANGLE     4'd1
`define WAVE_SAWTOOTH     4'd2
`define WAVE_SQUARE       4'd3
`define WAVE_DC           4'd4
`define WAVE_SSB_USB      4'd5   // upper sideband
`define WAVE_SSB_LSB      4'd6   // lower sideband
`define WAVE_AM           4'd7   // (1 + depth*m) * cos, carrier at half scale
`define WAVE_FM           4'd8   // phase increment ftw + m*deviation (NFM at 5 kHz)

`endif /* __ACM9767_DEFS_VH__ */
