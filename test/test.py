# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer

def compute_next(prn):
    fb = ((prn >> 7) & 1) ^ ((prn >> 5) & 1) ^ ((prn >> 4) & 1) ^ ((prn >> 3) & 1)
    return ((prn & 0x7F) << 1) | fb

async def drive_all_slaves(dut, bits):
    """ONE single task drives ALL three MISO lines at the same time"""
    await wait_falling(dut, dut.uio_out, 0)          # wait for CS low

    for i in range(24):
        await wait_falling(dut, dut.uio_out, 1)      # drive on falling SCLK

        # Read current uio_in, set all three bits, write back once
        current = int(dut.uio_in.value)
        new_val = current
        new_val |= (bits[i] << 2)   # miso0 = uio_in[2]
        new_val |= (bits[i] << 4)   # miso1 = uio_in[4]
        new_val |= (bits[i] << 6)   # miso2 = uio_in[6]
        dut.uio_in.value = new_val

async def wait_falling(dut, signal, bit):
    prev = signal.value[bit]
    while True:
        await signal.value_change
        curr = signal.value[bit]
        if curr == 0 and prev == 1:
            break
        prev = curr

@cocotb.test()
async def test_project(dut):
    dut._log.info("Start")

    clock = Clock(dut.clk, 122, unit="ns")
    cocotb.start_soon(clock.start())

    # Reset
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 20)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

    dut._log.info("=== First transaction: All valid, should set voted = 0xA5 ===")
    dut.ui_in.value = 0xA5
    current_prn = 0x01
    next_prn = compute_next(current_prn)
    frame = (next_prn << 16) | (0xA5 << 8)
    bits = [(frame >> (23 - i)) & 1 for i in range(24)]

    # ONLY ONE task now (no more race!)
    cocotb.start_soon(drive_all_slaves(dut, bits))

    await Timer(1, unit="ms")
    voted = int(dut.uo_out.value)
    dut._log.info(f"voted after transaction = 0x{voted:02X}")
    assert voted == 0xA5, f"Expected 0xA5, got 0x{voted:02X}"

    dut._log.info("SUCCESS - voted correctly set to 0xA5")
