# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer

RESET_PRG = 0x2A

def compute_next(prn):
    fb = ((prn >> 7) & 1) ^ ((prn >> 5) & 1) ^ ((prn >> 4) & 1) ^ ((prn >> 3) & 1)
    return ((prn & 0x7F) << 1) | fb


def bits_to_bytes(bits):
    data = []
    for i in range(0, len(bits), 8):
        value = 0
        for bit in bits[i:i + 8]:
            value = (value << 1) | bit
        data.append(value)
    return data


# =================================================================
# SimulatedCPU – now with its own miso_bit index and full encapsulation
# =================================================================
class SimulatedCPU:
    def __init__(self, name, miso_bit_idx):
        self.name = name
        self.miso_bit_idx = miso_bit_idx          # 2, 4 or 6
        self.mosi_bit_idx = miso_bit_idx + 1
        self.staged_prn = RESET_PRG
        self._desired = 0x00
        self._good_prg = True

    def frame_set_desired_out(self, desired_out, valid=True):
        """Store desired output for this frame (no logging)"""
        self._desired = desired_out
        self._good_prg = valid

    def frame_get_bits(self):
        """Return 24-bit MISO stream + log (called by perform_transaction)"""
        if self._good_prg:
            send_bytes = [self.staged_prn, self._desired, 0x00]
        else:
            send_bytes = [self.staged_prn ^ 0xFF, self._desired, 0x00]

        bits = [((b >> (7 - j)) & 1) for b in send_bytes for j in range(8)]

        cocotb.log.info(f"CPU {self.name} → sending bytes: echoed_prn=0x{send_bytes[0]:02X}, "
                        f"desired=0x{self._desired:02X}, unused=0x00")

        return bits

    def frame_accept_master_bytes(self, master_bytes):
        self.staged_prn = master_bytes[0]


async def wait_falling(dut, signal, bit):
    prev = signal.value[bit]
    while True:
        await signal.value_change
        curr = signal.value[bit]
        if curr == 0 and prev == 1:
            break
        prev = curr

async def wait_raising(dut, signal, bit):
    prev = signal.value[bit]
    while True:
        await signal.value_change
        curr = signal.value[bit]
        if curr == 1 and prev == 0:
            break
        prev = curr


async def perform_transaction(dut, cpu_list):
    """One transfer that works with ANY list of CPU instances"""
    await wait_falling(dut, dut.uio_out, 0)          # CS low

    # Get bits from every CPU (this is the clean dereference you asked for)
    bits_list = [cpu.frame_get_bits() for cpu in cpu_list]
    master_bits = [[] for _ in cpu_list]

    # First bit immediately (Mode 0)
    current = int(dut.uio_in.value)
    mask = ~((1<<2) | (1<<4) | (1<<6)) & 0xFF
    new_val = current & mask
    for cpu, bits in zip(cpu_list, bits_list):
        new_val |= (bits[0] << cpu.miso_bit_idx)
    dut.uio_in.value = new_val
    await wait_raising(dut, dut.uio_out, 1)
    current = int(dut.uio_out.value)
    for idx, cpu in enumerate(cpu_list):
        master_bits[idx].append((current >> cpu.mosi_bit_idx) & 1)

    # Remaining 23 bits on falling SCLK
    for i in range(1, 24):
        await wait_falling(dut, dut.uio_out, 1)
        current = int(dut.uio_in.value)
        new_val = current & mask
        for cpu, bits in zip(cpu_list, bits_list):
            new_val |= (bits[i] << cpu.miso_bit_idx)
        dut.uio_in.value = new_val
        await wait_raising(dut, dut.uio_out, 1)
        current = int(dut.uio_out.value)
        for idx, cpu in enumerate(cpu_list):
            master_bits[idx].append((current >> cpu.mosi_bit_idx) & 1)

    await wait_raising(dut, dut.uio_out, 0)          # CS high
    await Timer(1, unit="us")

    master_bytes = [bits_to_bytes(bits) for bits in master_bits]
    for cpu, sent in zip(cpu_list, master_bytes):
        cpu.frame_accept_master_bytes(sent)
    return master_bytes


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

    # Three CPU instances with their own miso_bit index
    cpus = [
        SimulatedCPU("CPU0", 2),
        SimulatedCPU("CPU1", 4),
        SimulatedCPU("CPU2", 6)
    ]
    current_prg = RESET_PRG

    # =================================================================
    # Test 1: All valid → voted = 0xA5
    # =================================================================
    dut._log.info("=== Test 1: All valid → voted = 0xA5 ===")
    dut.ui_in.value = 0xA5
    for cpu in cpus:
        cpu.frame_set_desired_out(0xA5, valid=True)

    master_bytes = await perform_transaction(dut, cpus)
    current_prg = compute_next(current_prg)
    for sent in master_bytes:
        assert sent[0] == current_prg
    assert int(dut.uo_out.value) == 0xA5
    dut._log.info("Test 1 PASSED")

    # =================================================================
    # Test 2: All valid → voted = 0x5A
    # =================================================================
    dut._log.info("=== Test 2: All valid → voted = 0x5A ===")
    dut.ui_in.value = 0x5A
    for cpu in cpus:
        cpu.frame_set_desired_out(0x5A, valid=True)

    master_bytes = await perform_transaction(dut, cpus)
    current_prg = compute_next(current_prg)
    for sent in master_bytes:
        assert sent[0] == current_prg
    assert int(dut.uo_out.value) == 0x5A
    dut._log.info("Test 2 PASSED")

    # =================================================================
    # Test 3: Only CPU0 valid → voted stays 0x5A
    # =================================================================
    dut._log.info("=== Test 3: Only one valid → voted stays 0x5A ===")
    dut.ui_in.value = 0x3C
    cpus[0].frame_set_desired_out(0x3C, valid=True)
    cpus[1].frame_set_desired_out(0x3C, valid=False)
    cpus[2].frame_set_desired_out(0x3C, valid=False)

    master_bytes = await perform_transaction(dut, cpus)
    current_prg = compute_next(current_prg)
    for sent in master_bytes:
        assert sent[0] == current_prg
    assert int(dut.uo_out.value) == 0x5A
    dut._log.info("Test 3 PASSED (voted correctly unchanged)")

    dut._log.info("=== ALL TESTS PASSED ===")
