"""Single source of truth for host-side platform constants.

Loads platform.json -- the *same* file CMake reads to generate
src/platform_config.vh for the gateware -- so the host UART/Modbus defaults
(baud, device id, framing) and the DAC sample rate used to compute frequency
tuning words cannot drift from what the FPGA actually implements.
"""

import json
import os

_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "platform.json")
with open(_PATH) as _f:
    _CFG = json.load(_f)

# --- clock / pll ---
SYS_CLK_HZ = _CFG["clock"]["sys_clk_hz"]
_pll = _CFG["pll"]
PLL_IDIV = _pll["idiv"]
PLL_FBDIV = _pll["fbdiv"]
PLL_ODIV = _pll["odiv"]
# Gowin rPLL: CLKOUT = CLKIN * FBDIV / IDIV (ODIV only sets the VCO rate).
DAC_CLK_HZ = SYS_CLK_HZ * PLL_FBDIV // PLL_IDIV
PLL_VCO_HZ = DAC_CLK_HZ * PLL_ODIV

# --- dac ---
_dac = _CFG["dac"]
DAC_BITS = _dac["bits"]
DAC_CHANNELS = _dac["channels"]
SINE_LUT_DEPTH = _dac["sine_lut_depth"]
DAC_FULL_SCALE = (1 << (DAC_BITS - 1)) - 1   # +8191 for 14 bits

# --- modbus ---
_modbus = _CFG["modbus"]
MODBUS_DEVICE_ID = _modbus["device_id"]
_addr_limit = _modbus["addr_limit"]
MODBUS_ADDR_LIMIT = int(_addr_limit, 0) if isinstance(_addr_limit, str) else int(_addr_limit)
MODBUS_MAX_READ_QTY = _modbus["max_read_qty"]
MODBUS_MAX_FRAME = _modbus["max_frame"]

# --- uart ---
_uart = _CFG["uart"]
UART_BAUD = _uart["baud"]
UART_DATA_BITS = _uart["data_bits"]
UART_STOP_BITS = _uart["stop_bits"]
# pyserial takes a single-char parity code.
_PARITY_CODE = {"none": "N", "odd": "O", "even": "E"}
UART_PARITY = _PARITY_CODE[_uart["parity"]]

# --- ssb ---
_ssb = _CFG["ssb"]
SSB_AUDIO_RATE_HZ = _ssb["audio_rate_hz"]
SSB_HILBERT_TAPS = _ssb["hilbert_taps"]
SSB_FIFO_DEPTH = _ssb["fifo_depth"]
SSB_DC_BLOCK_SHIFT = _ssb["dc_block_shift"]        # HP corner = audio_rate / (2 pi 2^shift)
SSB_LIMITER_LOOKAHEAD = _ssb["limiter_lookahead"]
SSB_LIMITER_THRESHOLD = _ssb["limiter_threshold"]  # Q15 envelope
SSB_LIMITER_RELEASE = _ssb["limiter_release"]
# Interpolation ratio baseband -> DAC rate; must be an integer (CMake checks too).
SSB_INTERP = DAC_CLK_HZ // SSB_AUDIO_RATE_HZ
