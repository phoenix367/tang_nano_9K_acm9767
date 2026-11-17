# Wiring the ACM9767 to the Tang Nano 9K

The ACM9767 is a dual-channel 14-bit, 125 MSPS DAC module built around the
Analog Devices AD9767 (dual-port mode, WRTx tied to CLKx on the module),
with op-amp output stages giving a bipolar +-5 V swing on two BNC/pin-header
outputs. It takes a single 5 V supply and its digital inputs run at 3.3 V
CMOS levels (VIH >= 2.1 V, VIL <= 0.9 V).

The module's 40-pin 2.54 mm header does not mate with the Tang Nano 9K's two
24-pin headers, so the connection is jumper wire. The assignment below is what
`src/acm9767.cst` constrains; any 3.3 V header pin works, so remap freely --
just keep the DAC bus off the 1.8 V pins 79-86 (BANK3), which cannot reach the
AD9767's 2.1 V logic threshold.

## Tang Nano 9K header pins by bank

| Header side | 3.3 V pins (usable for the DAC)                                        | 1.8 V pins (NOT for the DAC) |
| ----------- | ---------------------------------------------------------------------- | ---------------------------- |
| left        | 38 37 36 39 25 26 27 28 29 30 33 34 40 35 41 42 51 53 54 55 56 57 68 69 | --                           |
| right       | 63 77 76 75 74 73 72 71 70 48 49 31 32                                  | 86 85 84 83 82 81 80 79      |

Pins 36-39 go to the TF-card slot, 33-35/40-42/51/53-57/68-77 to the LCD
FPC and HDMI connector, 54/55/56 are also the SSPI configuration pins
(released as GPIO by `set_option -use_sspi_as_gpio 1` in `scripts/gw_run.tcl`).
None of that matters as long as nothing is plugged into those connectors.

## Default assignment (`src/acm9767.cst`)

Only **channel 1** is wired (`platform.json` `dac.channels = 1`). Data, CLK1
and WRT1 run down the left header in physical order.

| Signal        | FPGA pin | ACM9767 header |
| ------------- | -------- | -------------- |
| `da1_data[0]` | 25       | DA1_D0         |
| `da1_data[1]` | 26       | DA1_D1         |
| `da1_data[2]` | 27       | DA1_D2         |
| `da1_data[3]` | 28       | DA1_D3         |
| `da1_data[4]` | 29       | DA1_D4         |
| `da1_data[5]` | 30       | DA1_D5         |
| `da1_data[6]` | 33       | DA1_D6         |
| `da1_data[7]` | 34       | DA1_D7         |
| `da1_data[8]` | 40       | DA1_D8         |
| `da1_data[9]` | 35       | DA1_D9         |
| `da1_data[10]`| 41       | DA1_D10        |
| `da1_data[11]`| 42       | DA1_D11        |
| `da1_data[12]`| 51       | DA1_D12        |
| `da1_data[13]`| 53       | DA1_D13 (MSB)  |
| `da1_clk`     | 54       | DA1_CLK        |
| `da1_wrt`     | 55       | DA1_WRT        |
| GND           | GND      | GND            |

`da1_wrt` is the AD9767's WRT1 input-latch strobe. The gateware drives WRT1
as the inverted sample clock (data is centred on its rising edge) and CLK1 as
the non-inverted clock, so CLK1 rises half a sample *before* WRT1 (4.6 ns at
the default 108 MS/s, 9.3 ns at 54 MS/s). That keeps the pair clear of the
datasheet's forbidden window (CLK must not rise 0..2 ns after WRT) regardless
of lead-length or driver skew; the DAC latch then takes the previous
input-latch word, one sample of extra latency. If the module ties
WRT1 to CLK1 internally, connect only WRT1 (data timing is relative to WRT). Power the module from its own 5 V input (the Tang
Nano 9K's 5 V pin can supply it if your USB port has the headroom: the AD9767
alone draws ~130 mA at 3.3 V plus the op-amps).

Channel 2 is not brought out. To add it: set `dac.channels = 2` in
`platform.json`, uncomment the `da2_*` block at the bottom of
`src/acm9767.cst` (56 57 68 69 70..77 48 49 for data, 63 CLK2, 31 WRT2, or
any other 3.3 V pins), and rebuild. Free 3.3 V pins on the 1-channel build:
56, 57, 68-77, 48, 49, 63, 31, 32, 36-39.

## Signal conventions

* **Data format**: the AD9767 takes straight (offset) binary -- code 0x0000
  is one rail, 0x3FFF the other, 0x2000 mid-scale. The gateware emits
  mid-scale when a channel is disabled, so an idle channel sits at ~0 V on
  the module's bipolar output.
* **Polarity**: which rail is +5 V depends on the module's op-amp wiring.
  If a sawtooth ramps the wrong way on the scope, set the channel's invert
  bit (`acm9767_ctl.py invert 1 --on`) -- no rebuild needed.
* **Clocking**: every DAC pin (data, CLK, WRT) is driven by an OSER4
  serializer clocked at the sample rate (108 MHz, DDR), fed two samples per
  54 MHz logic clock; each data bit is sent for two half-periods, CLK as `1,0`
  and WRT as `0,1` per sample (`dac_output_stage.sv`). So the data changes
  with CLK's rising edge and WRT rises mid-way between data transitions:
  4.6 ns of nominal setup and hold at 108 MS/s against the AD9767's 2.0 / 1.5
  ns minimums, i.e. ~2.6 ns / ~3.1 ns of margin for driver skew, edge time
  and lead mismatch together. `dac.samples_per_clock = 1` (with `pll.fbdiv` 2,
  `pll.odiv` 16) restores the 54 MS/s ODDR build: one sample per clock, 9.3 ns
  either side.
* **Lead lengths**: at 54 MS/s the budget tolerated about ±5 ns (±1 m) of
  clock-vs-data mismatch; at 108 MS/s it is about ±2.5 ns *before* edge
  times, and WRT/CLK are 108 MHz square waves with only 4.6 ns between edges,
  so ringing on an unterminated jumper can double-clock the DAC. The FPGA
  has no on-chip series resistor (`SINGLE_RESISTOR` / `DIFF_RESISTOR` are the
  100 Ohm *input* termination of banks 1/3, and the GW1NR-9 flow ignores
  `SLEW_RATE`), but the LVCMOS33 driver impedance follows the drive strength
  (Gowin IBIS: ~48 Ohm at 4 mA, ~24 at 8 mA), so every DAC pin is set to
  `DRIVE=4` in `src/acm9767.cst`: roughly the source impedance of 8 mA plus an
  external 33 Ohm, with 150 MHz still guaranteed into 30 pF (DS100 Table
  4-18). Keep every lead under ~10 cm and of similar length, and run ground
  returns next to WRT/CLK and every few data lines; with longer wiring add
  external 22-33 Ohm series resistors at the FPGA end of CLK and WRT on top.
  If the output shows sparkle codes or a garbled spectrum at 108 MS/s that the
  54 MS/s build does not, the wiring is the first suspect -- try `DRIVE=8`
  (if the edges are too slow), go back to `samples_per_clock = 1`, or build a
  proper adapter board.
* **Sample rate**: `platform.json` `pll.fbdiv`/`pll.idiv` set the DAC sample
  rate `f_dac = 27 MHz * fbdiv / idiv` (default 108 MS/s = 27 x 4, PFD
  27 MHz, VCO 864 MHz); the logic clock `dac_clk` is f_dac /
  `dac.samples_per_clock`. The bitstream reports f_dac in registers 0x05/0x06
  so the host CLI computes tuning words against the real rate.

## Board resources used by the gateware

| Function      | Pin(s)                 |
| ------------- | ---------------------- |
| 27 MHz clock  | 52                     |
| reset (S1)    | 4, active low          |
| UART TX / RX  | 17 / 18 (FT2232H ch B) |
| LEDs 0..5     | 10 11 13 14 15 16 (active low) |

LEDs: 0 = PLL locked, 1/2 = channel 1/2 enabled, 3 = UART activity,
4 = 27 MHz heartbeat, 5 = dac_clk heartbeat.
