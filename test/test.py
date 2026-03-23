# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, Timer

RESET_PRGS = [0x2A, 0x54, 0xA8]
STRESS_TEST_SEED = 0x5EED1234
ANSI_YELLOW = "\033[33m"
ANSI_RESET = "\033[0m"


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


def format_byte(value, highlight=False):
    text = f"{value:02X}"
    if highlight:
        return f"{ANSI_YELLOW}{text}{ANSI_RESET}"
    return text


def format_frame_bytes(values, highlights=None):
    if highlights is None:
        highlights = [False] * len(values)
    return ",".join(
        format_byte(value, highlight=highlight)
        for value, highlight in zip(values, highlights)
    )


def inject_fault_bit(bits, bit_index):
    bits[bit_index] ^= 1


def bit_index_to_byte_index(bit_index):
    return bit_index // 8


def fault_targets_byte(fault, direction, cpu_index, byte_index):
    return (
        fault is not None
        and fault["direction"] == direction
        and fault["cpu_index"] == cpu_index
        and bit_index_to_byte_index(fault["bit_index"]) == byte_index
    )


def choose_random_fault(rng, cpu_count, cpu_index=None):
    return {
        "direction": rng.choice(["miso", "mosi"]),
        "cpu_index": rng.randrange(cpu_count) if cpu_index is None else cpu_index,
        "bit_index": rng.randrange(24),
    }


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
        self.expected_echo_prn = reset_prg
        self.observed_switches = 0x00
        self.last_sent_bytes = [reset_prg, 0x00, 0xFF]
        self._desired = 0x00
        self._desired_valid = 0xFF
        self._good_prg = True

    def frame_set_desired_out(self, desired_out, valid=True, desired_valid=0xFF):
        """Store desired output for this frame (no logging)"""
        self._desired = desired_out
        self._good_prg = valid
        self._desired_valid = desired_valid

    def frame_get_bits(self, log_frame=False):
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

        return bits

    def frame_accept_master_bytes(self, raw_master_bytes, received_master_bytes):
        self.expected_echo_prn = raw_master_bytes[0]
        self.staged_prn = received_master_bytes[0]
        self.observed_switches = received_master_bytes[1]


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


async def perform_transaction(dut, cpu_list, log_frame=True, fault=None):
    """One transfer that works with ANY list of CPU instances"""
    await wait_falling(dut, dut.uio_out, 0)          # CS low

    # Get bits from every CPU (this is the clean dereference you asked for)
    bits_list = [cpu.frame_get_bits(log_frame=log_frame) for cpu in cpu_list]
    if fault is not None and fault["direction"] == "miso":
        inject_fault_bit(bits_list[fault["cpu_index"]], fault["bit_index"])
    raw_master_bits = [[] for _ in cpu_list]
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
        raw_bit = (current >> cpu.mosi_bit_idx) & 1
        raw_master_bits[idx].append(raw_bit)
        bit = raw_bit
        if (
            fault is not None
            and fault["direction"] == "mosi"
            and fault["cpu_index"] == idx
            and fault["bit_index"] == 0
        ):
            bit ^= 1
        master_bits[idx].append(bit)

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
            raw_bit = (current >> cpu.mosi_bit_idx) & 1
            raw_master_bits[idx].append(raw_bit)
            bit = raw_bit
            if (
                fault is not None
                and fault["direction"] == "mosi"
                and fault["cpu_index"] == idx
                and fault["bit_index"] == i
            ):
                bit ^= 1
            master_bits[idx].append(bit)

    await wait_raising(dut, dut.uio_out, 0)          # CS high
    await Timer(1, unit="us")

    slave_bytes = [bits_to_bytes(bits) for bits in bits_list]
    raw_master_bytes = [bits_to_bytes(bits) for bits in raw_master_bits]
    master_bytes = [bits_to_bytes(bits) for bits in master_bits]
    for cpu, raw_sent, received_sent in zip(cpu_list, raw_master_bytes, master_bytes):
        cpu.frame_accept_master_bytes(raw_sent, received_sent)
    return master_bytes, slave_bytes, raw_master_bytes


async def setup_testbench(dut):
    dut._log.info("Start")

    clock = Clock(dut.clk, 122, unit="ns")
    cocotb.start_soon(clock.start())

    # Reset
    dut._frame_number = 0
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

async def run_frame(
    dut,
    cpus,
    model,
    title,
    switches,
    log_frame=True,
    return_errors=False,
    fault=None,
    frame_number=None,
    log_summary=True,
):
    if log_frame:
        dut._log.info(title)
    if frame_number is None:
        dut._frame_number += 1
        frame_number = dut._frame_number
    dut.ui_in.value = switches

    expected_majority = list(model.majority_bytes)
    expected_echoes = []
    expected_prns = []
    outputs = []
    valids = []
    frame_valids = []

    for cpu in cpus:
        expected_prns.append(cpu.expected_echo_prn)
        expected_echoes.append(cpu.staged_prn if cpu._good_prg else cpu.staged_prn ^ 0xFF)
        outputs.append(cpu._desired)
        valids.append(cpu._desired_valid)
        frame_valids.append(cpu._good_prg)

    master_bytes, slave_bytes, raw_master_bytes = await perform_transaction(
        dut,
        cpus,
        log_frame=False,
        fault=fault,
    )

    errors = []
    for idx, sent in enumerate(master_bytes):
        if cpus[idx].last_sent_bytes[0] != expected_echoes[idx]:
            errors.append(
                f"{cpus[idx].name} echoed_prn=0x{cpus[idx].last_sent_bytes[0]:02X} "
                f"expected 0x{expected_echoes[idx]:02X}"
            )
        if sent[1] != switches:
            if not fault_targets_byte(fault, "mosi", idx, 1):
                errors.append(
                    f"{cpus[idx].name} switches=0x{sent[1]:02X} expected 0x{switches:02X}"
                )
        if sent[2] != expected_majority[idx]:
            if not fault_targets_byte(fault, "mosi", idx, 2):
                errors.append(
                    f"{cpus[idx].name} majority=0x{sent[2]:02X} expected 0x{expected_majority[idx]:02X}"
                )

    actual_outputs = [sent[1] for sent in slave_bytes]
    actual_valids = [sent[2] for sent in slave_bytes]
    actual_frame_valids = [
        sent[0] == expected_prn
        for sent, expected_prn in zip(slave_bytes, expected_prns)
    ]
    expected_voted = model.apply_frame(actual_outputs, actual_valids, actual_frame_valids)
    actual_voted = int(dut.uo_out.value)
    if actual_voted != expected_voted:
        errors.append(f"out=0x{actual_voted:02X} expected 0x{expected_voted:02X}")
    if log_summary:
        log_frame_summary(
            dut,
            frame_number,
            switches,
            vote_resolved_bits(outputs),
            vote_cpu_observed_switches(cpus),
            actual_voted,
            cpus,
            master_bytes,
            slave_bytes,
            fault,
            errors,
        )
    if return_errors:
        return {
            "master_bytes": master_bytes,
            "slave_bytes": slave_bytes,
            "raw_master_bytes": raw_master_bytes,
            "errors": errors,
            "actual_voted": actual_voted,
        }
    assert not errors, "; ".join(errors)
    return master_bytes


def log_frame_summary(
    dut,
    frame_number,
    hardware_in,
    software_out,
    software_in,
    hardware_out,
    cpus,
    master_bytes,
    slave_bytes,
    fault,
    errors,
):
    parts = []
    if frame_number is not None:
        parts.append(f"F{frame_number:03d}")
    parts.append(f"hIsO[{hardware_in:02X},{software_out:02X}]")
    for idx, (cpu, master, slave) in enumerate(zip(cpus, master_bytes, slave_bytes)):
        parts.append(
            f"{cpu.name}:M[{format_frame_bytes(master, [fault_targets_byte(fault, 'mosi', idx, byte) for byte in range(3)])}]"
            f":S[{format_frame_bytes(slave, [fault_targets_byte(fault, 'miso', idx, byte) for byte in range(3)])}]"
        )
    parts.append(f"sIhO[{software_in:02X},{hardware_out:02X}]")
    parts.append("FAIL" if errors else "OK")
    dut._log.info(" ".join(parts))


def configure_all_cpus(cpus, desired_out, valid=True, desired_valid=0xFF):
    for cpu in cpus:
        cpu.frame_set_desired_out(desired_out, valid=valid, desired_valid=desired_valid)


def configure_cpu_frame(cpus, outputs, frame_valids, valids):
    for cpu, output, frame_valid, valid_mask in zip(cpus, outputs, frame_valids, valids):
        cpu.frame_set_desired_out(output, valid=frame_valid, desired_valid=valid_mask)


def configure_reflexive_frame(cpus):
    for cpu in cpus:
        cpu.frame_set_desired_out(cpu.observed_switches, valid=True, desired_valid=0xFF)


def vote_cpu_observed_switches(cpus):
    return vote_resolved_bits([cpu.observed_switches for cpu in cpus])


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


async def run_followup_frame(dut, cpus, model, title, switches):
    configure_all_cpus(cpus, switches)
    return await run_frame(dut, cpus, model, title, switches)


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


@cocotb.test()
async def test_two_valid_sources_can_update_the_vote(dut):
    cpus, model = await setup_testbench(dut)

    await prime_vote(dut, cpus, model, 0x00)

    configure_cpu_frame(
        cpus,
        [0x96, 0x96, 0x69],
        [True, True, False],
        [0xFF, 0xFF, 0xFF],
    )
    await run_frame(
        dut,
        cpus,
        model,
        "=== Test 8: Two valid sources can update the vote ===",
        0x96,
    )
    assert int(dut.uo_out.value) == 0x96
    dut._log.info("Test 8 PASSED")


@cocotb.test()
async def test_two_disagreeing_valid_sources_preserve_the_previous_vote(dut):
    cpus, model = await setup_testbench(dut)

    await prime_vote(dut, cpus, model, 0x5A)

    configure_cpu_frame(
        cpus,
        [0xFF, 0x00, 0x00],
        [True, True, False],
        [0xFF, 0xFF, 0xFF],
    )
    await run_frame(
        dut,
        cpus,
        model,
        "=== Test 9: Two disagreeing valid sources preserve the previous vote ===",
        0xC3,
    )
    assert int(dut.uo_out.value) == 0x5A
    dut._log.info("Test 9 PASSED")


@cocotb.test()
async def test_zero_valid_masks_keep_the_vote_and_clear_majority_bytes(dut):
    cpus, model = await setup_testbench(dut)

    await prime_vote(dut, cpus, model, 0xC3)

    configure_cpu_frame(
        cpus,
        [0x00, 0xFF, 0x3C],
        [True, True, True],
        [0x00, 0x00, 0x00],
    )
    await run_frame(
        dut,
        cpus,
        model,
        "=== Test 10: Zero valid masks keep the vote ===",
        0x12,
    )
    assert int(dut.uo_out.value) == 0xC3

    master_bytes = await run_followup_frame(
        dut,
        cpus,
        model,
        "=== Test 10 follow-up: zero valid masks clear the next majority bytes ===",
        0x55,
    )
    assert [sent[2] for sent in master_bytes] == [0x00, 0x00, 0x00]
    dut._log.info("Test 10 PASSED")


@cocotb.test()
async def test_dissenting_cpu_gets_a_zero_majority_byte(dut):
    cpus, model = await setup_testbench(dut)

    await prime_vote(dut, cpus, model, 0x00)

    configure_cpu_frame(
        cpus,
        [0xAA, 0xAA, 0x55],
        [True, True, True],
        [0xFF, 0xFF, 0xFF],
    )
    await run_frame(
        dut,
        cpus,
        model,
        "=== Test 11: A dissenting CPU gets no majority bits ===",
        0xAA,
    )
    assert int(dut.uo_out.value) == 0xAA

    master_bytes = await run_followup_frame(
        dut,
        cpus,
        model,
        "=== Test 11 follow-up: the dissenting CPU sees a zero majority byte ===",
        0x33,
    )
    assert [sent[2] for sent in master_bytes] == [0xFF, 0xFF, 0x00]
    dut._log.info("Test 11 PASSED")


@cocotb.test()
async def test_partial_majority_bytes_are_reported_per_cpu(dut):
    cpus, model = await setup_testbench(dut)

    await prime_vote(dut, cpus, model, 0x00)

    configure_cpu_frame(
        cpus,
        [0xF0, 0xCC, 0xC0],
        [True, True, True],
        [0xFF, 0xFF, 0xFF],
    )
    await run_frame(
        dut,
        cpus,
        model,
        "=== Test 12: Majority bytes can be partial per CPU ===",
        0xC0,
    )
    assert int(dut.uo_out.value) == 0xC0

    master_bytes = await run_followup_frame(
        dut,
        cpus,
        model,
        "=== Test 12 follow-up: partial majority bytes are sent back ===",
        0x11,
    )
    assert [sent[2] for sent in master_bytes] == [0xCF, 0xF3, 0xFF]
    dut._log.info("Test 12 PASSED")


@cocotb.test()
async def test_all_bad_prgs_keep_the_vote_and_clear_majority_bytes(dut):
    cpus, model = await setup_testbench(dut)

    await reach_masked_vote(dut, cpus, model)

    configure_cpu_frame(
        cpus,
        [0x00, 0xFF, 0x3C],
        [False, False, False],
        [0xFF, 0xFF, 0xFF],
    )
    await run_frame(
        dut,
        cpus,
        model,
        "=== Test 13: All bad PRGs reject the whole frame ===",
        0x12,
    )
    assert int(dut.uo_out.value) == 0xA7

    master_bytes = await run_followup_frame(
        dut,
        cpus,
        model,
        "=== Test 13 follow-up: a rejected frame clears all next majority bytes ===",
        0x34,
    )
    assert [sent[2] for sent in master_bytes] == [0x00, 0x00, 0x00]
    dut._log.info("Test 13 PASSED")


@cocotb.test()
async def test_random_traffic_keeps_running_without_reset(dut):
    cpus, model = await setup_testbench(dut)
    rng = random.Random(STRESS_TEST_SEED)
    unstable_cpu = None

    dut._log.info(
        f"=== Test 14: 256 random one-bit frame faults without reset (seed=0x{STRESS_TEST_SEED:08X}) ==="
    )

    for frame in range(256):
        switches = rng.randrange(256)
        software_out = vote_cpu_observed_switches(cpus)
        configure_reflexive_frame(cpus)
        fault = choose_random_fault(rng, len(cpus), cpu_index=unstable_cpu)

        details = await run_frame(
            dut,
            cpus,
            model,
            "",
            switches,
            log_frame=False,
            return_errors=True,
            fault=fault,
            frame_number=frame + 1,
            log_summary=False,
        )
        actual_out = details["actual_voted"]
        software_in = vote_cpu_observed_switches(cpus)
        errors = list(details["errors"])

        if software_in != switches:
            errors.append(f"sI=0x{software_in:02X} expected hI=0x{switches:02X}")
        if actual_out != software_out:
            errors.append(f"hO=0x{actual_out:02X} expected sO=0x{software_out:02X}")
        log_frame_summary(
            dut,
            frame + 1,
            switches,
            software_out,
            software_in,
            actual_out,
            cpus,
            details["master_bytes"],
            details["slave_bytes"],
            fault,
            errors,
        )
        assert not errors, "; ".join(errors)
        if (
            fault["direction"] == "mosi"
            and bit_index_to_byte_index(fault["bit_index"]) in (0, 1)
        ):
            unstable_cpu = fault["cpu_index"]
        else:
            unstable_cpu = None

    dut._log.info("Test 14 PASSED")
