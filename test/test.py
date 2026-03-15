# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Edge, Timer

def compute_next(prn):
    fb = ((prn >> 7) & 1) ^ ((prn >> 5) & 1) ^ ((prn >> 4) & 1) ^ ((prn >> 3) & 1)
    return ((prn & 0x7F) << 1) | fb

async def wait_rising(dut, signal, bit):
    prev = signal.value[bit]
    while True:
        await Edge(signal)
        curr = signal.value[bit]
        if curr == 1 and prev == 0:
            break
        prev = curr

async def wait_falling(dut, signal, bit):
    prev = signal.value[bit]
    while True:
        await Edge(signal)
        curr = signal.value[bit]
        if curr == 0 and prev == 1:
            break
        prev = curr

async def drive_slave(dut, miso_idx, bits):
    await wait_falling(dut, dut.uio_out, 0)
    dut.uio_in[miso_idx].value = bits[0]
    for i in range(23):
        await wait_rising(dut, dut.uio_out, 1)
        await wait_falling(dut, dut.uio_out, 1)
        dut.uio_in[miso_idx].value = bits[i + 1]
    await wait_rising(dut, dut.uio_out, 1)

@cocotb.test()
async def test_project(dut):
    dut._log.info("Start")

    # Set the clock period to 122 ns (8.192 MHz)
    clock = Clock(dut.clk, 122, unit="ns")
    cocotb.start_soon(clock.start())

    # Reset
    dut._log.info("Reset")
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

    dut._log.info("Test project behavior")

    # First cycle: All valid, desired=0xA5
    dut.ui_in.value = 0xA5
    current_prn = 0x01
    next_prn = compute_next(current_prn)
    frame = (next_prn << 16) | (0xA5 << 8) | 0x00
    bits = [(frame >> (23 - i)) & 1 for i in range(24)]
    cocotb.start_soon(drive_slave(dut, 2, bits))
    cocotb.start_soon(drive_slave(dut, 4, bits))
    cocotb.start_soon(drive_slave(dut, 6, bits))
    await wait_rising(dut, dut.uio_out, 0)  # Wait for transaction end
    await Timer(1, units="ns")
    assert dut.uo_out.value == 0xA5

    # Second cycle: All valid, desired=0x5A
    dut.ui_in.value = 0x5A
    current_prn = next_prn
    next_prn = compute_next(current_prn)
    frame = (next_prn << 16) | (0x5A << 8) | 0x00
    bits = [(frame >> (23 - i)) & 1 for i in range(24)]
    cocotb.start_soon(drive_slave(dut, 2, bits))
    cocotb.start_soon(drive_slave(dut, 4, bits))
    cocotb.start_soon(drive_slave(dut, 6, bits))
    await wait_falling(dut, dut.uio_out, 0)  # Wait for next start
    await wait_rising(dut, dut.uio_out, 0)  # Wait for end
    await Timer(1, units="ns")
    assert dut.uo_out.value == 0x5A

    # Third cycle: One valid with desired=0x3C, two invalid -> no update, stays 0x5A
    dut.ui_in.value = 0x3C
    current_prn = next_prn
    next_prn = compute_next(current_prn)
    bits_good = [(((next_prn << 16) | (0x3C << 8) | 0x00) >> (23 - i)) & 1 for i in range(24)]
    bits_bad = [(((0xFF << 16) | (0x3C << 8) | 0x00) >> (23 - i)) & 1 for i in range(24)]
    cocotb.start_soon(drive_slave(dut, 2, bits_good))
    cocotb.start_soon(drive_slave(dut, 4, bits_bad))
    cocotb.start_soon(drive_slave(dut, 6, bits_bad))
    await wait_falling(dut, dut.uio_out, 0)
    await wait_rising(dut, dut.uio_out, 0)
    await Timer(1, units="ns")
    assert dut.uo_out.value == 0x5A

    dut._log.info("Test complete")
