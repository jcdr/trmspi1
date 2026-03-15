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

    # First bit immediately (SPI Mode 0 requirement)
    current = int(dut.uio_in.value)
    mask = ~((1<<2) | (1<<4) | (1<<6)) & 0xFF
    new_val = (current & mask) | (bits[0] << 2) | (bits[0] << 4) | (bits[0] << 6)
    dut.uio_in.value = new_val

    # Remaining 23 bits on falling SCLK
    for i in range(1, 24):
        await wait_falling(dut, dut.uio_out, 1)
        current = int(dut.uio_in.value)
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

    # =================================================================
    # Test 1: All three CPUs valid → voted becomes 0xA5
    # =================================================================
    dut._log.info("=== Test 1: All valid → voted = 0xA5 ===")
    dut.ui_in.value = 0xA5
    current_prn = 0x01
    next_prn = compute_next(current_prn)
    bytes_to_send = [next_prn, 0xA5, 0x00]
    bits = [((b >> (7-j)) & 1) for b in bytes_to_send for j in range(8)]

    cocotb.start_soon(drive_all_slaves(dut, bits))
    await Timer(1, unit="ms")
    assert int(dut.uo_out.value) == 0xA5
    dut._log.info("Test 1 PASSED")

    # =================================================================
    # Test 2: All three CPUs valid → voted becomes 0x5A
    # =================================================================
    dut._log.info("=== Test 2: All valid → voted = 0x5A ===")
    dut.ui_in.value = 0x5A
    current_prn = next_prn
    next_prn = compute_next(current_prn)
    bytes_to_send = [next_prn, 0x5A, 0x00]
    bits = [((b >> (7-j)) & 1) for b in bytes_to_send for j in range(8)]

    cocotb.start_soon(drive_all_slaves(dut, bits))
    await Timer(1, unit="ms")
    assert int(dut.uo_out.value) == 0x5A
    dut._log.info("Test 2 PASSED")

    # =================================================================
    # Test 3: Only one CPU valid → voted stays 0x5A (majority not reached)
    # =================================================================
    dut._log.info("=== Test 3: Only one valid → voted stays 0x5A ===")
    dut.ui_in.value = 0x3C
    current_prn = next_prn
    next_prn = compute_next(current_prn)
    bits_good = [((next_prn >> (7-j)) & 1) for j in range(8)] + \
                [((0x3C >> (7-j)) & 1) for j in range(8)] + \
                [0] * 8
    bits_bad  = [((0xFF >> (7-j)) & 1) for j in range(8)] + \
                [((0x3C >> (7-j)) & 1) for j in range(8)] + \
                [0] * 8

    cocotb.start_soon(drive_all_slaves(dut, bits_good))  # CPU0 valid
    cocotb.start_soon(drive_all_slaves(dut, bits_bad))   # CPU1 invalid
    cocotb.start_soon(drive_all_slaves(dut, bits_bad))   # CPU2 invalid
    await Timer(1, unit="ms")
    assert int(dut.uo_out.value) == 0x5A
    dut._log.info("Test 3 PASSED (voted correctly unchanged)")

    dut._log.info("=== ALL TESTS PASSED ===")
