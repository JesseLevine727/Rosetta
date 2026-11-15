# Bring-up test for the Rosetta chip, run on the RTL and on the gate-level
# netlist (the Tiny Tapeout `test` and `gl_test` jobs).
#
# This is deliberately small: the design is verified by the formal proofs in
# formal/, and the compiler's protocols are checked on the RTL by
# toolchain/tests/. This test only shows that the built chip comes out of
# reset quiet, that the host port reaches both memories and the registers,
# and that a lane started by the host drives a pin.

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

# Host registers (docs/blocks/host.md section 4)
MADDR, MDATA = 0x01, 0x02
XOUT0 = 0x42
CTRL, PC, FLAGS, R10, R32, QCFG, QIN, QOUT = range(8)

H = 8  # clock cycles per half period of the host SPI clock


def lane_reg(lane, r):
    return 0x10 + 8 * lane + r


async def spi_frame(dut, out, nbits):
    """One host frame, MSB first. ui[0] = SCLK, ui[1] = CS_n, ui[2] = MOSI,
    uo[0] = MISO."""
    got = 0
    dut.ui_in.value = 0x00
    await ClockCycles(dut.clk, H)
    for i in range(nbits - 1, -1, -1):
        b = (out >> i) & 1
        dut.ui_in.value = b << 2
        await ClockCycles(dut.clk, H)
        got = (got << 1) | (int(dut.uo_out.value) & 1)
        dut.ui_in.value = (b << 2) | 1
        await ClockCycles(dut.clk, H)
    dut.ui_in.value = b << 2
    await ClockCycles(dut.clk, H)
    dut.ui_in.value = 0x02
    await ClockCycles(dut.clk, H)
    return got


async def wr(dut, addr, data):
    await spi_frame(dut, ((addr & 0x7F) << 32) | (data & 0xFFFFFFFF), 40)


async def rd(dut, addr):
    return (await spi_frame(dut, (0x80 | (addr & 0x7F)) << 40, 48)) & 0xFFFFFFFF


async def reset(dut):
    cocotb.start_soon(Clock(dut.clk, 16, unit="ns").start())  # about 60 MHz
    dut.ena.value = 1
    dut.ui_in.value = 0x02  # host CS_n high
    dut.uio_in.value = 0xFF
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)


@cocotb.test()
async def test_reset_quiet(dut):
    """After reset no bidirectional pin is driven and no protocol output is high."""
    await reset(dut)
    for _ in range(20):
        await ClockCycles(dut.clk, 1)
        assert int(dut.uio_oe.value) == 0
        assert int(dut.uo_out.value) & 0xFC == 0


@cocotb.test()
async def test_memories(dut):
    """The host writes and reads back both memories."""
    await reset(dut)
    prog = [0x21110000, 0xDEADBEEF, 0x00000000, 0xFFFFFFFF, 0x12345678]
    await wr(dut, MADDR, 0x000)  # program memory, address 0
    for w in prog:
        await wr(dut, MDATA, w)
    data = [0xA5A5A5A5, 0x0F0F0F0F, 0xCAFEF00D]
    await wr(dut, MADDR, 0x200)  # data memory, address 0
    for w in data:
        await wr(dut, MDATA, w)
    await wr(dut, MADDR, 0x000)
    assert [await rd(dut, MDATA) for _ in prog] == prog
    await wr(dut, MADDR, 0x200)
    assert [await rd(dut, MDATA) for _ in data] == data


@cocotb.test()
async def test_lane_drives_pin(dut):
    """The host loads one instruction, starts lane 0, and the lane drives uo[2]."""
    await reset(dut)
    await wr(dut, XOUT0, 0x10)  # lane 0 output 0 onto uo[2]
    await wr(dut, MADDR, 0x000)
    await wr(dut, MDATA, 0x21110000)  # PIN out0=1
    await wr(dut, MDATA, 0x70000001)  # loop: BR always, loop
    assert int(dut.uo_out.value) & 0x04 == 0
    await wr(dut, lane_reg(0, CTRL), 0)
    await wr(dut, lane_reg(0, QCFG), 0xF)
    await wr(dut, lane_reg(0, QIN), 0)
    await wr(dut, lane_reg(0, QOUT), 0)
    await wr(dut, lane_reg(0, PC), 0)
    await wr(dut, lane_reg(0, CTRL), 1)
    await ClockCycles(dut.clk, 40)
    assert int(dut.uo_out.value) & 0x04 == 0x04
    assert int(dut.uio_oe.value) == 0
