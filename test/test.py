# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0
#
# Smoke test of tt_um_srujan0006_dtc through its pins only, so it also runs on the gate-level netlist (GL test).
# The full verification (cycle-accurate model, lockstep, mutation, formal, A13 timing) lives in the DTC repo tree
# (dtc_model.py, dtc_spi.py, tb/); this file is a self-contained subset.
#
# Config port: SPI mode 0, MSB first.  Header RW[15] AINC[14] reserved[13:8] addr[7:0]; writes = 32-bit words;
# reads = 8 turnaround clocks then the 32-bit word, per word.  SCK <= clk/8.

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge

DIV = 8                   # SCK = clk/8
F_FRAME_ERR = 1 << 24

# UART TX (8N1, 8 clocks per bit) as built by dtc_asm.py, and its configuration
UART_TX_PROG = [0x9FA0, 0xF727, 0x6001, 0x0642]
EXECCTRL_UART, SHIFTCTRL_UART, PINCTRL_UART = 0x100C0, 0x0C, 0x201010
SET_PINS_1, SET_PINDIRS_1 = 0xE001, 0xE081


class SpiHost:
    """Drives ui_in on falling clk edges; samples MISO when it raises SCK."""

    def __init__(self, dut):
        self.dut = dut

    async def _cycle(self, cs, sck, mosi):
        await FallingEdge(self.dut.clk)
        miso = int(self.dut.uo_out.value) & 1
        self.dut.ui_in.value = (cs & 1) | ((sck & 1) << 1) | ((mosi & 1) << 2)
        return miso

    async def frame(self, bits):
        sampled = []
        for _ in range(DIV // 2):
            await self._cycle(0, 0, bits[0] if bits else 0)
        for b in bits:
            sampled.append(await self._cycle(0, 1, b))
            for _ in range(DIV // 2 - 1):
                await self._cycle(0, 1, b)
            for _ in range(DIV // 2):
                await self._cycle(0, 0, b)
        for _ in range(DIV):
            await self._cycle(1, 0, 0)
        return sampled

    @staticmethod
    def _bits(v, n):
        return [(v >> (n - 1 - i)) & 1 for i in range(n)]

    async def write(self, addr, *words, ainc=True, nbits=None):
        bits = self._bits((int(ainc) << 14) | addr, 16)
        for w in words:
            bits += self._bits(w, 32)
        await self.frame(bits if nbits is None else bits[:nbits])

    async def read(self, addr, n=1, ainc=True):
        bits = self._bits((1 << 15) | (int(ainc) << 14) | addr, 16) + [0] * (40 * n)
        s = await self.frame(bits)
        out = []
        for k in range(n):
            w = 0
            for b in s[16 + 40 * k + 8:16 + 40 * k + 40]:
                w = (w << 1) | b
            out.append(w)
        return out


async def start(dut):
    cocotb.start_soon(Clock(dut.clk, 23810, unit="ps").start())     # 42 MHz
    dut.ena.value = 1
    dut.ui_in.value = 1                                              # CS_N high
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 4)
    return SpiHost(dut)


@cocotb.test()
async def test_id_and_register_readback(dut):
    host = await start(dut)
    ident = (await host.read(0x00))[0]
    assert ident >> 16 == 0xD7C0 and ident & 0xFF == 0x01, f"ID {ident:#010x}"
    dut._log.info(f"ID {ident:#010x} (NUM_SM={(ident >> 8) & 0xFF})")
    await host.write(0x0E, 0xA5C30F81)                               # WAIT_CFG of SM0: all 32 bits read/write
    assert await host.read(0x0E) == [0xA5C30F81]
    await host.write(0x80, 0x1234, 0xBEEF)                           # IMEM, auto-increment
    assert [w & 0xFFFF for w in await host.read(0x80, 2)] == [0x1234, 0xBEEF]


@cocotb.test()
async def test_partial_word_sets_frame_err(dut):
    host = await start(dut)
    await host.write(0x0E, 0x11111111)
    await host.write(0x0E, 0x22222222, nbits=16 + 20)                # partial word: discarded
    assert await host.read(0x0E) == [0x11111111]
    st = (await host.read(0x02))[0]
    assert st & F_FRAME_ERR, f"STATUS {st:#010x}"
    await host.write(0x02, F_FRAME_ERR)                              # write 1 to clear
    assert not (await host.read(0x02))[0] & F_FRAME_ERR


@cocotb.test()
async def test_uart_program_runs(dut):
    host = await start(dut)
    data = [0x55, 0xA3]
    await host.write(0x80, *UART_TX_PROG)
    await host.write(0x05, EXECCTRL_UART, SHIFTCTRL_UART, PINCTRL_UART)
    await host.write(0x04, 1)                                        # CLKDIV
    await host.write(0x0A, SET_PINS_1)                               # forced instructions
    await host.write(0x0A, SET_PINDIRS_1)
    await host.write(0x08, *data, ainc=False)                        # TX FIFO
    lv = []

    async def sample():
        while True:
            await FallingEdge(dut.clk)
            lv.append(int(dut.uio_out.value) & 1 if int(dut.uio_oe.value) & 1 else None)

    sampler = cocotb.start_soon(sample())                            # the program starts during the next frame
    await host.write(0x01, 0x01)                                     # SM0_EN
    await ClockCycles(dut.clk, 10 * 8 * len(data) + 50)
    sampler.cancel()
    exp = []
    for byte in data:
        for bit in [0] + [(byte >> i) & 1 for i in range(8)] + [1]:
            exp += [bit] * 8
    t0 = lv.index(0)
    assert lv[t0:t0 + len(exp)] == exp, "uio[0] does not show the expected 8N1 frames"
    dut._log.info(f"UART on uio[0]: {[hex(d) for d in data]} correct")
