# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer

def compute_next(prn):
    fb = ((prn >> 7) & 1) ^ ((prn >> 5) & 1) ^ ((prn >> 4) & 1) ^ ((prn >> 3) & 1)
    return ((prn & 0x7F) << 1) | fb

async def wait_falling(dut, signal, bit):
    prev = signal.value[bit]
    while True:
        await signal.value_change
        curr = signal.value[bit]
        if curr == 0 and prev == 1:
            break
        prev = curr

async def drive_all_slaves(dut, bits):
    await wait_falling(dut, dut.uio_out, 0)          # CS low
    for i in range(24):
        await wait_falling(dut, dut.uio_out, 1)      # drive on falling SCLK
        current = int(dut.uio_in.value)
        # Clear the three MISO bits first, then set the new value (this fixes 0xFF)
        mask = ~((1<<2) | (1<<4) | (1<<6)) & 0xFF
        new_val = (current & mask) | (bits[i] << 2) | (bits[i] << 4) | (bits[i] << 6)
        dut.uio_in.value = new_val

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
    next_prn = compute_next(current_prn)          # will be 0x02

    # Full 24-bit Slave→Master frame: next_prn + desired (0xA5) + unused (0x00)
    bytes_to_send = [next_prn, 0xA5, 0x00]
    bits = []
    for b in bytes_to_send:
        for j in range(8):
            bits.append((b >> (7 - j)) & 1)

    dut._log.info(f"Sending on all MISO lines: next_prn=0x{next_prn:02X}, desired=0xA5, unused=0x00")

    cocotb.start_soon(drive_all_slaves(dut, bits))

    await Timer(1, unit="ms")
    voted = int(dut.uo_out.value)
    dut._log.info(f"voted after transaction = 0x{voted:02X}")
    assert voted == 0xA5, f"Expected 0xA5, got 0x{voted:02X}"

    dut._log.info("SUCCESS - voted correctly set to 0xA5")
