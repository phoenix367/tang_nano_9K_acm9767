# tt_um_gubochkin_dds — Tiny Tapeout (IHP SG13G2) port of the ACM9767 DDS

ASIC version of the FPGA design in the parent repository: a Modbus-controlled
32-bit DDS with sine / triangle / sawtooth / square / DC waveforms and SSB /
AM / FM of a host-streamed baseband, driving a 14-bit parallel DAC at the chip
clock. Same register map and host CLI as the FPGA build. See `docs/info.md`
for the pinout and usage, and the parent `CLAUDE.md` / `README.md` for the
design background.

What changed against the FPGA design to fit an ASIC with no RAM, PLL or DSP:

| FPGA | ASIC |
| --- | --- |
| 27 MHz crystal + PLL → 54 MHz | the Tiny Tapeout clock is the sample clock (50 MHz) |
| 4096-point quarter-wave sine ROM in block RAM | 64-entry midpoint table + linear interpolation (±1 LSB), sine and cosine |
| SSB: 127-tap Hilbert FIR, CIC, 2048-sample FIFO | phasing method with two 3-section IIR all-pass chains on the serial multiplier, two 3-stage CICs, I/Q mixer (`SSB` parameter) |
| AM / FM with two parallel multipliers | one shared serial multiplier for everything that changes per audio sample |
| Modbus frame buffer in block RAM, 208 bytes | 24 bytes of flip-flops (FC16 up to 7 registers) |
| two DAC channels optional, CLK + WRT outputs | one channel, CLK only (tie WRT to CLK) |

## Layout

- `src/` — Verilog-2005 RTL: `tt_um_gubochkin_dds.v` (top, pins), `dds_channel.v`,
  `sine_interp.v`, `serial_mul.v`, `baseband_dsp.v`, `cic_interp.v` (SSB only),
  `dac_regs.v`, `modbus_rtu_slave.v`, `uart.v`
- `test/` — cocotb tests (`make`, `make SSB=0`), `test_sine.py` (an exhaustive
  Icarus sweep of the sine/cosine generator against `round(8191·sin)`), and
  `tb_loopback.v` for the modulator→demodulator loopback below
- `info.yaml`, `docs/info.md` — Tiny Tapeout project metadata and datasheet

## Tests

```sh
python3 -m venv .venv && .venv/bin/pip install -r test/requirements.txt
cd test && PATH=$PWD/../.venv/bin:$PATH make        # cocotb: Modbus identity/regs, sine on the DAC pins, audio FIFO
python3 test/test_sine.py                          # all 65536 phases within +-1 LSB (sin and cos)
```

## Configuration and area

Submitted configuration: `SSB = 1` (the top-level default) on **8×4 tiles**
(1336 × 432 µm = 0.577 mm²), a non-standard size that the Tiny Tapeout team
enables on request. Pre-layout statistics (yosys + the SG13G2 liberty):
17 463 cells, 2 519 flip-flops, 0.282 mm² of cells = 49 % of the grid
(TT's default placement target is 60 %). Without SSB (`SSB = 0`) the design
is 8 881 cells / 1 543 flip-flops / 0.156 mm² and fits 8×2 at 54 %; set both
the parameter and `tiles: "8x2"` for that build. The GDS is produced by the
Tiny Tapeout GitHub Action in `.github/workflows/`.

Pre-layout timing (ABC with the liberty delays, typical corner, no wires):
the longest register-to-register path is 5.4 ns against the 20 ns clock.
Post-route timing needs the LibreLane flow (the GitHub action, or locally
with Docker and the IHP PDK — several GB of disk).

## SSB modulator (`SSB = 1`)

The streamed baseband goes through two chains of three first-order all-pass
sections (poles A = −0.9651, −0.7533, −0.1968 → Q; B = 0.4058, −0.5403,
−0.8822 → I; designed by differential evolution for a 90° difference over
250–3600 Hz, max error 0.21°) computed on the shared serial multiplier, then
two 3-stage CIC interpolators (×3125, 42-bit) and an I·cos ∓ Q·sin mixer on
the cosine output of `sine_interp`. Waveform codes 5 (USB) and 6 (LSB) select
it, with the amplitude applied at baseband.

Verified with the FPGA project's modulator→demodulator loopback on this RTL
(7 MHz carrier, seven tones 300–2600 Hz, reference equalised by the same
all-pass chain):

| | USB | LSB |
| --- | --- | --- |
| steady-state SNR | 54.7 dB | 54.7 dB |
| unwanted sideband | 57.5 dB | 57.5 dB |
| carrier | 65.5 dB | 67.3 dB |

```sh
iverilog -g2012 -o test/sim_build/loopback.vvp src/*.v test/tb_loopback.v
../scripts/ssb_loopback.py --vvp vvp --sim test/sim_build/loopback.vvp --fs-dac 50e6 --audio-rate 16000 \
    --max-delay 40 --allpass 0.4058,-0.5403,-0.8822 --mode usb --carrier 7e6 --level 0.5
```

It costs about 126 000 µm² (the CICs are 53 000 µm² / 570 flip-flops of it,
the mixer's multiplier pair most of the rest), hence the 8×4 grid.

## Submitting

Tiny Tapeout's GitHub Actions (`.github/workflows/`) expect `info.yaml`, `src/`
and `test/` at the root of their own repository. To submit, push this `tt/`
directory as a separate repository (e.g. `git subtree split --prefix tt`), or
copy it; the workflows then run the tests, build the GDS with librelane and
generate the datasheet from `docs/info.md`.
