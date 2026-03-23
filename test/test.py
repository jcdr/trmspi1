# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer

RESET_PRGS = [0x2A, 0x54, 0xA8]


def format_bits(value):
    return f"{value:08b}"


def bits_to_bytes(bits):
    data = []
    for i in range(0, len(bits), 8):
        value = 0
        for bit in bits[i:i + 8]:
            value = (value << 1) | bit
        data.append(value)
    return data


def vote_resolved_bits(resolved):
    voted = 0
    for bit in range(8):
        inputs = [(value >> bit) & 1 for value in resolved]
        if inputs.count(1) >= 2:
            voted |= 1 << bit
    return voted


class VoterModel:
    def __init__(self):
        self.voted = 0
        self.majority_bytes = [0, 0, 0]

    def apply_frame(self, outputs, valids, frame_valids):
        valid_masks = [
            (valid if frame_valid else 0)
            for valid, frame_valid in zip(valids, frame_valids)
        ]
        resolved = [
            (output & valid_mask) | (self.voted & (~valid_mask & 0xFF))
            for output, valid_mask in zip(outputs, valid_masks)
        ]
        self.voted = vote_resolved_bits(resolved)
        self.majority_bytes = [
            valid_mask & ~(output ^ self.voted) & 0xFF
            for output, valid_mask in zip(outputs, valid_masks)
        ]
        return self.voted


# =================================================================
# SimulatedCPU – now with its own miso_bit index and full encapsulation
# =================================================================
class SimulatedCPU:
    def __init__(self, name, miso_bit_idx, reset_prg):
        self.name = name
        self.miso_bit_idx = miso_bit_idx          # 2, 4 or 6
        self.mosi_bit_idx = miso_bit_idx + 1
        self.staged_prn = reset_prg
        self.last_sent_bytes = [reset_prg, 0x00, 0xFF]
        self._desired = 0x00
        self._desired_valid = 0xFF
        self._good_prg = True

    def frame_set_desired_out(self, desired_out, valid=True, desired_valid=0xFF):
        """Store desired output for this frame (no logging)"""
        self._desired = desired_out
        self._good_prg = valid
        self._desired_valid = desired_valid

    def frame_get_bits(self):
        """Return 24-bit MISO stream + log (called by perform_transaction)"""
        if self._good_prg:
            self.last_sent_bytes = [self.staged_prn, self._desired, self._desired_valid]
        else:
            self.last_sent_bytes = [
                self.staged_prn ^ 0xFF,
                self._desired,
                self._desired_valid,
            ]

        bits = [((b >> (7 - j)) & 1) for b in self.last_sent_bytes for j in range(8)]

        cocotb.log.info(f"CPU {self.name} → sending bytes: echoed_prn=0x{self.last_sent_bytes[0]:02X}, "
                        f"desired=0x{self._desired:02X}, valid=0x{self._desired_valid:02X}")

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


async def setup_testbench(dut):
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
        SimulatedCPU("CPU0", 2, RESET_PRGS[0]),
        SimulatedCPU("CPU1", 4, RESET_PRGS[1]),
        SimulatedCPU("CPU2", 6, RESET_PRGS[2])
    ]
    return cpus, VoterModel()

async def run_frame(dut, cpus, model, title, switches):
    dut._log.info(title)
    dut.ui_in.value = switches
    previous_out = int(dut.uo_out.value)

    dut._log.info(
        f"Chip state before frame: switches=0x{switches:02X} ({format_bits(switches)}), "
        f"out=0x{previous_out:02X} ({format_bits(previous_out)})"
    )

    expected_majority = list(model.majority_bytes)
    expected_echoes = []
    outputs = []
    valids = []
    frame_valids = []

    for cpu in cpus:
        expected_echoes.append(cpu.staged_prn if cpu._good_prg else cpu.staged_prn ^ 0xFF)
        outputs.append(cpu._desired)
        valids.append(cpu._desired_valid)
        frame_valids.append(cpu._good_prg)

    master_bytes = await perform_transaction(dut, cpus)
    log_master_frames(cpus, master_bytes)

    for idx, sent in enumerate(master_bytes):
        assert cpus[idx].last_sent_bytes[0] == expected_echoes[idx]
        assert sent[1] == switches
        assert sent[2] == expected_majority[idx]

    expected_voted = model.apply_frame(outputs, valids, frame_valids)
    assert int(dut.uo_out.value) == expected_voted
    dut._log.info(
        f"Chip state after frame:  switches=0x{switches:02X} ({format_bits(switches)}), "
        f"out=0x{expected_voted:02X} ({format_bits(expected_voted)})"
    )
    return master_bytes


def configure_all_cpus(cpus, desired_out, valid=True, desired_valid=0xFF):
    for cpu in cpus:
        cpu.frame_set_desired_out(desired_out, valid=valid, desired_valid=desired_valid)


def configure_cpu_frame(cpus, outputs, frame_valids, valids):
    for cpu, output, frame_valid, valid_mask in zip(cpus, outputs, frame_valids, valids):
        cpu.frame_set_desired_out(output, valid=frame_valid, desired_valid=valid_mask)


def log_master_frames(cpus, master_bytes):
    for cpu, sent in zip(cpus, master_bytes):
        cocotb.log.info(
            f"Master -> {cpu.name}: next_prn=0x{sent[0]:02X}, "
            f"switches=0x{sent[1]:02X}, majority=0x{sent[2]:02X}"
        )


async def reach_masked_vote(dut, cpus, model):
    await prime_vote(dut, cpus, model, 0xA5)

    configure_cpu_frame(
        cpus,
        [0x0F, 0x03, 0x05],
        [True, True, True],
        [0x0F, 0x03, 0x05],
    )
    await run_frame(dut, cpus, model, "=== Prime vote: per-bit valid masks -> 0xA7 ===", 0x5A)
    assert int(dut.uo_out.value) == 0xA7


async def prime_vote(dut, cpus, model, voted):
    configure_all_cpus(cpus, voted)
    await run_frame(dut, cpus, model, f"=== Prime vote: all valid -> 0x{voted:02X} ===", voted)
    assert int(dut.uo_out.value) == voted


@cocotb.test()
async def test_all_valid_vote(dut):
    cpus, model = await setup_testbench(dut)

    await prime_vote(dut, cpus, model, 0xA5)
    assert int(dut.uo_out.value) == 0xA5
    dut._log.info("Test 1 PASSED")


@cocotb.test()
async def test_per_bit_valid_masks(dut):
    cpus, model = await setup_testbench(dut)

    await prime_vote(dut, cpus, model, 0xA5)

    configure_cpu_frame(
        cpus,
        [0x0F, 0x03, 0x05],
        [True, True, True],
        [0x0F, 0x03, 0x05],
    )
    await run_frame(dut, cpus, model, "=== Test 2: Per-bit valid masks drive the vote ===", 0x5A)
    assert int(dut.uo_out.value) == 0xA7
    dut._log.info("Test 2 PASSED")


@cocotb.test()
async def test_one_valid_frame_keeps_vote(dut):
    cpus, model = await setup_testbench(dut)

    await reach_masked_vote(dut, cpus, model)

    configure_cpu_frame(
        cpus,
        [0x08, 0xF0, 0x00],
        [True, False, False],
        [0x08, 0xF0, 0xFF],
    )
    await run_frame(dut, cpus, model, "=== Test 3: Only one valid -> voted stays unchanged ===", 0x3C)
    assert int(dut.uo_out.value) == 0xA7
    dut._log.info("Test 3 PASSED (voted correctly unchanged)")


@cocotb.test()
async def test_majority_bytes_are_sent_one_frame_late(dut):
    cpus, model = await setup_testbench(dut)

    await prime_vote(dut, cpus, model, 0xA5)

    configure_all_cpus(cpus, 0x3C)
    master_bytes = await run_frame(
        dut,
        cpus,
        model,
        "=== Test 4: Majority bytes report previous frame membership ===",
        0x3C,
    )
    for sent in master_bytes:
        assert sent[2] == 0xFF
    assert int(dut.uo_out.value) == 0x3C
    dut._log.info("Test 4 PASSED")


@cocotb.test()
async def test_bad_prg_clears_that_cpu_majority_byte(dut):
    cpus, model = await setup_testbench(dut)

    await prime_vote(dut, cpus, model, 0xA5)

    configure_cpu_frame(
        cpus,
        [0xA5, 0xA5, 0xA5],
        [True, False, True],
        [0xFF, 0xFF, 0xFF],
    )
    await run_frame(
        dut,
        cpus,
        model,
        "=== Test 5: Bad PRG clears one CPU majority byte ===",
        0xA5,
    )

    configure_all_cpus(cpus, 0x3C)
    master_bytes = await run_frame(
        dut,
        cpus,
        model,
        "=== Test 5 follow-up: next frame sends the cleared majority byte ===",
        0x3C,
    )
    assert master_bytes[0][2] == 0xFF
    assert master_bytes[1][2] == 0x00
    assert master_bytes[2][2] == 0xFF
    dut._log.info("Test 5 PASSED")


@cocotb.test()
async def test_invalid_bits_fall_back_to_previous_vote(dut):
    cpus, model = await setup_testbench(dut)

    await prime_vote(dut, cpus, model, 0xF0)

    configure_cpu_frame(
        cpus,
        [0x0F, 0x0F, 0x00],
        [True, True, True],
        [0x0F, 0x0F, 0x00],
    )
    await run_frame(
        dut,
        cpus,
        model,
        "=== Test 6: Invalid bits fall back to the previous vote ===",
        0x00,
    )
    assert int(dut.uo_out.value) == 0xFF
    dut._log.info("Test 6 PASSED")


@cocotb.test()
async def test_valid_zero_bits_can_clear_previous_ones(dut):
    cpus, model = await setup_testbench(dut)

    await prime_vote(dut, cpus, model, 0xFF)

    configure_cpu_frame(
        cpus,
        [0x00, 0x00, 0xFF],
        [True, True, True],
        [0xF0, 0xF0, 0x00],
    )
    await run_frame(
        dut,
        cpus,
        model,
        "=== Test 7: Valid zero bits clear previous ones ===",
        0x00,
    )
    assert int(dut.uo_out.value) == 0x0F
    dut._log.info("Test 7 PASSED")
