# SPDX-FileCopyrightText: 2026 Ivan Gubochkin
# SPDX-License-Identifier: Apache-2.0
"""cocotb tests for tt_um_gubochkin_dds: Modbus RTU over the UART pins.

The test bit-bangs 1 Mbaud 8-E-1 frames into ui_in[0], collects the reply on
uio_out[7], and checks: identity registers, FC06/FC16 writes with FC03 read
back, a COMMIT enabling a 1/64-rate sine on the DAC pins (period and peaks),
an illegal-address exception, and the audio FIFO level/underrun counters.

Runs under the template's Makefile (make -C tt/test [SSB=0|1]). The 50 MHz
clock and 1 Mbaud UART make each frame ~100 us of simulated time.
"""

import math
import struct

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge, Timer

CLK_NS = 20                 # 50 MHz
BIT_NS = 1000               # 1 Mbaud
SLAVE = 7


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


async def uart_send(dut, data: bytes):
    """8-E-1, LSB first, on ui_in[0]."""
    for b in data:
        bits = [0] + [(b >> i) & 1 for i in range(8)] + [bin(b).count("1") & 1, 1]
        for bit in bits:
            dut.ui_in.value = bit
            await Timer(BIT_NS, unit="ns")


async def uart_recv(dut, n: int, timeout_us: int = 400) -> bytes:
    """Receive n bytes from uio_out[7] (start-bit driven, sample at bit centres)."""
    out = bytearray()
    deadline = timeout_us * 1000
    while len(out) < n and deadline > 0:
        while int(dut.uio_out.value) >> 7 & 1 and deadline > 0:
            await Timer(CLK_NS, unit="ns")
            deadline -= CLK_NS
        if deadline <= 0:
            break
        await Timer(BIT_NS + BIT_NS // 2, unit="ns")     # centre of data bit 0
        v = 0
        for i in range(8):
            v |= ((int(dut.uio_out.value) >> 7) & 1) << i
            await Timer(BIT_NS, unit="ns")
        await Timer(BIT_NS, unit="ns")                   # parity, then we are in the stop bit
        out.append(v)
        deadline -= 11 * BIT_NS
    return bytes(out)


async def txn(dut, pdu: bytes, expect: int) -> bytes:
    req = bytes([SLAVE]) + pdu
    req += struct.pack("<H", crc16(req))
    await uart_send(dut, req)
    resp = await uart_recv(dut, expect)
    assert len(resp) == expect, f"got {len(resp)} bytes, expected {expect}: {resp.hex()}"
    assert crc16(resp) == 0, f"bad response CRC: {resp.hex()}"
    return resp


async def read_reg(dut, addr: int) -> int:
    r = await txn(dut, struct.pack(">BHH", 0x03, addr, 1), 7)
    return (r[3] << 8) | r[4]


async def write_reg(dut, addr: int, value: int):
    await txn(dut, struct.pack(">BHH", 0x06, addr, value & 0xFFFF), 8)


async def write_multi(dut, addr: int, values):
    pdu = struct.pack(">BHHB", 0x10, addr, len(values), 2 * len(values))
    pdu += b"".join(struct.pack(">H", v & 0xFFFF) for v in values)
    await txn(dut, pdu, 8)


def dac(dut) -> int:
    return (int(dut.uo_out.value) << 6) | (int(dut.uio_out.value) & 0x3F)


async def reset(dut):
    dut.ena.value = 1
    dut.ui_in.value = 1            # UART idle high
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)


@cocotb.test()
async def test_identity_and_regs(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())
    await reset(dut)
    assert dac(dut) == 0x2000, "DAC not at mid-scale after reset"
    assert int(dut.uio_oe.value) == 0xFF

    assert await read_reg(dut, 0x00) == 0x9767
    assert await read_reg(dut, 0x01) == 0x0201
    assert await read_reg(dut, 0x07) == 1
    clk_hz = (await read_reg(dut, 0x06) << 16) | await read_reg(dut, 0x05)
    assert clk_hz == 50_000_000
    assert await read_reg(dut, 0x0A) == 16000
    assert await read_reg(dut, 0x0B) == 16

    # staged writes read back; nothing live until COMMIT
    await write_multi(dut, 0x10, [0x0000, 0x1000, 0x4000, 0xFFFF, 0x0000, 0, 0x8000])
    assert await read_reg(dut, 0x11) == 0x1000
    assert await read_reg(dut, 0x12) == 0x4000
    await write_reg(dut, 0x13, 0x7FFF)
    assert await read_reg(dut, 0x13) == 0x7FFF
    await write_reg(dut, 0x03, 0x0001)
    await ClockCycles(dut.clk, 50)
    assert dac(dut) == 0x2000, "channel ran before COMMIT"

    # illegal address -> exception 0x83/0x02
    req = bytes([SLAVE]) + struct.pack(">BHH", 0x03, 0x17F, 4)
    req += struct.pack("<H", crc16(req))
    await uart_send(dut, req)
    r = await uart_recv(dut, 5)
    assert r[1] == 0x83 and r[2] == 0x02, f"expected illegal-address exception, got {r.hex()}"


@cocotb.test()
async def test_sine_output(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())
    await reset(dut)
    # ftw = 2^32/64 -> 64 samples per period at full amplitude, sine
    await write_multi(dut, 0x10, [0x0000, 0x0400, 0x0000, 0xFFFF, 0x0000, 0, 0x8000])
    await write_reg(dut, 0x03, 0x0001)
    await write_reg(dut, 0x04, 1)
    await ClockCycles(dut.clk, 60)                   # pipeline fill
    samples = []
    for _ in range(64 * 4):
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        samples.append(dac(dut) - 8192)
    crossings = sum(1 for a, b in zip(samples, samples[1:]) if a < 0 <= b)
    assert 3 <= crossings <= 5, f"{crossings} rising crossings in 4 periods"
    assert max(samples) >= 8180 and min(samples) <= -8180, f"peaks {min(samples)}..{max(samples)}"
    start = next(i for i, (a, b) in enumerate(zip(samples, samples[1:])) if a < 0 <= b) + 1
    worst = max(abs(samples[start + k] - round(8191 * math.sin(2 * math.pi * k / 64))) for k in range(64))
    assert worst <= 2, f"worst deviation from ideal sine {worst} LSB"
    dut._log.info(f"sine OK: {crossings} periods, peaks {min(samples)}..{max(samples)}, worst {worst} LSB")

    # invert flips every bit; disable returns to mid-scale
    await write_reg(dut, 0x03, 0x0005)
    await write_reg(dut, 0x04, 1)
    await ClockCycles(dut.clk, 60)
    inv = []
    for _ in range(64):
        await RisingEdge(dut.clk)
        await Timer(1, unit="ns")
        inv.append(dac(dut))
    assert max(inv) <= 0x3FFF and min(inv) >= 0 and (max(inv) - min(inv)) > 16000
    await write_reg(dut, 0x03, 0x0000)
    await write_reg(dut, 0x04, 1)
    await ClockCycles(dut.clk, 60)
    assert dac(dut) == 0x2000


@cocotb.test()
async def test_audio_fifo(dut):
    """16-deep FIFO at 16 kS/s = 1 ms of audio; a Modbus frame is 100..340 us on
    the wire, so: the empty FIFO counts underruns; a 7-sample FC16 (the most a
    24-byte frame carries, 437 us of audio) shows a non-zero level on the poll
    that lands ~130 us later; no underrun is counted while samples remain
    (checked on the internal counter, RTL simulation only); then it drains."""
    cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())
    await reset(dut)
    await write_reg(dut, 0x09, 0)                    # clear underruns
    await Timer(500, unit="us")                      # 8 audio ticks with an empty FIFO
    u = await read_reg(dut, 0x09)
    assert 6 <= u <= 12, f"underruns {u} after 500 us, expected ~8"

    await write_multi(dut, 0x100, [1000 * (k + 1) for k in range(7)])
    lvl = await read_reg(dut, 0x08)
    assert 2 <= lvl <= 7, f"FIFO level {lvl} on the poll after a 7-sample push"
    try:                                             # not available on a gate-level netlist
        u0 = int(dut.user_project.regs.underruns.value)
        await Timer(100, unit="us")                  # samples still queued for ~200 us more
        assert int(dut.user_project.regs.underruns.value) == u0, "underrun counted while samples were queued"
    except AttributeError:
        dut._log.info("internal underrun counter not visible (gate level) - skipped")
    await Timer(800, unit="us")
    assert await read_reg(dut, 0x08) == 0, "FIFO did not drain"
    assert await read_reg(dut, 0x09) > 0
