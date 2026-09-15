`include "timescale.v"
`include "acm9767_defs.vh"
`include "platform_config.vh"

// Baseband analytic-signal generator: per input sample computes
//   q = K * hilbert(x)         (TAPS-tap type-III FIR, coefficients from
//                               src/hilbert_coefs.vh, Q15)
//   i = K * x[n - (TAPS-1)/2]  (matching group delay)
// where K (Q15, PLATFORM_SSB_GAIN_K) pre-compensates the residual gain of the
// CIC interpolator that follows. One multiplier, time-shared: TAPS MAC cycles
// plus a few for the gain stages, well inside the R = dac_clk/audio_rate
// cycles between samples. in_strobe must not repeat before out_strobe.

module hilbert_fir #(
    parameter integer TAPS = 127,
    parameter [15:0]  K    = 16'd24131
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               in_strobe,
    input  wire signed [15:0] in_data,
    output reg                out_strobe,
    output reg  signed [15:0] i_out,
    output reg  signed [15:0] q_out
);

localparam integer CENTER = (TAPS - 1) / 2;

// ---- coefficient ROM + circular delay line (256 deep, 8-bit index) ----
reg signed [15:0] hilbert_rom [0:TAPS-1];
initial begin
`include "hilbert_coefs.vh"
end

reg signed [15:0] line [0:255];
reg [7:0] wr_idx;                  // slot of the newest sample
// Zero the delay line so the first outputs are defined (in simulation an X in
// any tap would poison the accumulator and, downstream, the CIC integrators).
integer li;
initial for (li = 0; li < 256; li = li + 1) line[li] = 16'sd0;

// ---- sequencer ----
localparam [2:0] S_IDLE = 3'd0, S_MAC = 3'd1, S_DRAIN = 3'd2, S_GAIN_Q = 3'd3,
                 S_GAIN_I = 3'd4, S_OUT = 3'd5;
reg [2:0] state;
reg [7:0] k;                       // tap index
reg [7:0] rd_addr;
reg signed [15:0] x_rd, h_rd;      // registered operands (line / rom reads)
reg               mac_valid, mac_valid_d;
reg signed [31:0] product;
reg signed [39:0] acc;
reg signed [15:0] center_x;

wire signed [31:0] mul = mul_a * mul_b;
reg  signed [15:0] mul_a, mul_b;

function signed [15:0] sat16(input signed [24:0] v);
    begin
        if (v > 25'sd32767)       sat16 = 16'sd32767;
        else if (v < -25'sd32768) sat16 = -16'sd32768;
        else                      sat16 = v[15:0];
    end
endfunction

// Q15 result of the MAC: acc is Q30 (Q15 x Q15) -> >>> 15, saturated
wire signed [24:0] acc_q15 = acc[39:15];
wire signed [15:0] q_pre   = sat16(acc_q15);
wire signed [31:0] gain_p  = mul;                 // Q15 * Q15 -> Q30
wire signed [15:0] gain_q15 = sat16({{9{gain_p[31]}}, gain_p[30:15]});

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state       <= `WRAP_SIM(#1) S_IDLE;
        wr_idx      <= `WRAP_SIM(#1) 8'd0;
        k           <= `WRAP_SIM(#1) 8'd0;
        rd_addr     <= `WRAP_SIM(#1) 8'd0;
        acc         <= `WRAP_SIM(#1) 40'sd0;
        product     <= `WRAP_SIM(#1) 32'sd0;
        mac_valid   <= `WRAP_SIM(#1) 1'b0;
        mac_valid_d <= `WRAP_SIM(#1) 1'b0;
        x_rd        <= `WRAP_SIM(#1) 16'sd0;
        h_rd        <= `WRAP_SIM(#1) 16'sd0;
        center_x    <= `WRAP_SIM(#1) 16'sd0;
        mul_a       <= `WRAP_SIM(#1) 16'sd0;
        mul_b       <= `WRAP_SIM(#1) 16'sd0;
        out_strobe  <= `WRAP_SIM(#1) 1'b0;
        i_out       <= `WRAP_SIM(#1) 16'sd0;
        q_out       <= `WRAP_SIM(#1) 16'sd0;
    end else begin
        out_strobe <= `WRAP_SIM(#1) 1'b0;

        // MAC pipeline: (x_rd, h_rd) -> product -> acc
        x_rd        <= `WRAP_SIM(#1) line[rd_addr];
        h_rd        <= `WRAP_SIM(#1) hilbert_rom[k];
        mac_valid   <= `WRAP_SIM(#1) (state == S_MAC);
        product     <= `WRAP_SIM(#1) x_rd * h_rd;
        mac_valid_d <= `WRAP_SIM(#1) mac_valid;
        if (mac_valid_d) acc <= `WRAP_SIM(#1) acc + {{8{product[31]}}, product};

        case (state)
            S_IDLE: if (in_strobe) begin                      // sample lands in line[] (block below)
                wr_idx  <= `WRAP_SIM(#1) wr_idx + 8'd1;
                k       <= `WRAP_SIM(#1) 8'd0;
                rd_addr <= `WRAP_SIM(#1) wr_idx + 8'd1;          // newest sample = tap 0
                acc     <= `WRAP_SIM(#1) 40'sd0;
                state   <= `WRAP_SIM(#1) S_MAC;
            end
            S_MAC: begin
                // this cycle reads line[rd_addr] / rom[k]; advance to the next tap
                rd_addr <= `WRAP_SIM(#1) rd_addr - 8'd1;
                if (k == CENTER) center_x <= `WRAP_SIM(#1) line[rd_addr];   // x[n - CENTER]
                if (k == TAPS - 1) state <= `WRAP_SIM(#1) S_DRAIN;
                k <= `WRAP_SIM(#1) k + 8'd1;
            end
            S_DRAIN: begin                    // let the last product land in acc
                k <= `WRAP_SIM(#1) k + 8'd1;
                if (k == TAPS + 2) begin
                    mul_a <= `WRAP_SIM(#1) q_pre;
                    mul_b <= `WRAP_SIM(#1) $signed(K);
                    state <= `WRAP_SIM(#1) S_GAIN_Q;
                end
            end
            S_GAIN_Q: begin                   // mul = q_pre * K available now
                q_out <= `WRAP_SIM(#1) gain_q15;
                mul_a <= `WRAP_SIM(#1) center_x;
                state <= `WRAP_SIM(#1) S_GAIN_I;
            end
            S_GAIN_I: begin
                i_out      <= `WRAP_SIM(#1) gain_q15;
                out_strobe <= `WRAP_SIM(#1) 1'b1;
                state      <= `WRAP_SIM(#1) S_IDLE;
            end
            default: state <= `WRAP_SIM(#1) S_IDLE;
        endcase
    end
end

// Delay-line write port (separate block without async reset so it infers RAM).
// wr_next = wr_idx + 1 is kept as a register so the RAM write address/enable
// path is a plain decode.
reg [7:0] wr_next;
always @(posedge clk or negedge rst_n)
    if (!rst_n) wr_next <= `WRAP_SIM(#1) 8'd1;
    else        wr_next <= `WRAP_SIM(#1) wr_idx + 8'd1;
always @(posedge clk)
    if (state == S_IDLE && in_strobe) line[wr_next] <= `WRAP_SIM(#1) in_data;

endmodule
