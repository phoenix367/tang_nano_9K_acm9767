<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

## How it works

A single-channel direct digital synthesiser and modulator for a 14-bit
parallel DAC (Analog Devices AD9767 class, e.g. the ACM9767 module),
controlled over a UART with Modbus RTU. Everything runs on the Tiny Tapeout
clock, which is also the DAC sample rate (50 MHz nominal; any 10–50 MHz works,
the host reads the actual rate back from a register).

- 32-bit phase accumulator: `f_out = FTW × f_clk / 2^32`, sub-Hz resolution,
  carriers up to ~20 MHz at 50 MS/s.
- Waveforms: sine (64-entry quarter-wave table with linear interpolation,
  within ±1 LSB of ideal over the whole turn, no RAM), triangle, sawtooth,
  square with programmable duty, DC.
- SSB (upper or lower sideband), AM (settable depth) and FM (settable
  deviation, 5 kHz NFM default) of a baseband stream the host pushes over
  Modbus at 16 kS/s into a 16-sample FIFO; underruns are counted. SSB uses
  the phasing method with two IIR all-pass chains (54 dB sideband
  suppression in simulation) and CIC interpolation to the DAC rate.
- 16-bit phase offset, amplitude (16-bit gain), signed DC offset with
  saturation, output inversion; all settings are staged in holding registers
  and applied atomically by a COMMIT.
- Modbus RTU slave id 7, 1 Mbaud 8-E-1, FC03 / FC06 / FC16, CRC-16 checked and
  generated, exceptions for illegal function / address / value.

The register map is the one of the FPGA version of this design
(`acm9767_demo` on GitHub), so the same host CLI (`acm9767_ctl.py`) drives
both: 0x00 ID (0x9767), 0x01 VERSION (0x0201), 0x02 STATUS, 0x03 CONTROL,
0x04 COMMIT, 0x05/06 sample clock in Hz, 0x08 FIFO level, 0x09 underruns,
0x10–0x17 the channel block (FTW lo/hi, phase, amplitude, offset, waveform,
duty, modulation parameter), 0x100–0x17F the audio FIFO write window.

Only three multipliers run at the sample rate (output scaling and the SSB
mixer); every product that changes once per audio sample uses a small serial
multiplier. About 280 000 µm² of SG13G2 standard cells on an 8×4 grid.

## How to test

Connect a 3.3 V UART (1 Mbaud, 8 data bits, even parity, 1 stop) to `ui[0]`
(RX) and `uio[7]` (TX). Wire `uo[7:0]` to DAC data bits 13..6, `uio[5:0]` to
bits 5..0, `uio[6]` to the DAC clock and WRT inputs. Then, with the host tool
from the FPGA project (`pip install pyserial`):

```
acm9767_ctl.py -p /dev/ttyUSB0 status
acm9767_ctl.py -p /dev/ttyUSB0 set 1 --freq 1e6 --wave sine
acm9767_ctl.py -p /dev/ttyUSB0 enable 1
acm9767_ctl.py -p /dev/ttyUSB0 stream --carrier 7.1e6 --mode usb --wav voice.wav
```

Without a DAC, watch `uo[7]` (DAC bit 13, the sign of the sine) toggle at the
programmed frequency on a scope or logic analyser. Without a host, the chip
sits at mid-scale (0x2000) after reset.

Any Modbus RTU master works too: write 0x0400 to register 0x11 (FTW high
word, ≈ 780 kHz at 50 MHz), 1 to register 0x03 (enable), 1 to register 0x04
(commit).

## External hardware

A 14-bit parallel-input DAC with CMOS inputs, e.g. the ACM9767 / AN9767
module (AD9767, dual, 125 MSPS, 3.3 V logic) — one channel is used. A
3.3 V USB-UART adapter for control. Optionally a 50 MHz clock source if the
board clock is not used.
