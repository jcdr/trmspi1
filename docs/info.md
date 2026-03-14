<!---

This file is used to generate your project datasheet. Please fill in the information below and delete any unused
sections.

You can also include images in this folder and reference them in the markdown. Each image must be less than
512 kb in size, and the combined size of all images must be less than 1 MB.
-->

# TMR SPI Voter for Redundant Processors

## What is this?

This is a Triple Modular Redundancy (TMR) voter chip for safety-critical embedded systems (e.g., medical devices).
It interfaces with 3 redundant low-power processors (like Ambiq Apollo) via SPI to vote on their outputs,
ensuring fault tolerance against processor or software malfunctions.

- **Inputs**: 8 switch inputs from the demo board (ui_in), sent to processors.
- **Outputs**: 8 voted discrete signals (uo_out, connected to 7-segment + dot display on demo board).
- **SPI Interface** (on bidirectional pins uio):
  - Shared cs_n (uio[0], out)
  - Shared sclk (uio[1], out)
  - miso0 (uio[2], in), mosi0 (uio[3], out)
  - miso1 (uio[4], in), mosi1 (uio[5], out)
  - miso2 (uio[6], in), mosi2 (uio[7], out)

## How does it work?

The chip acts as SPI master:
- Generates a pseudo-random seed (shared LFSR PRNG, initial 0x01).
- Sends frame to each processor: seed + agreement_byte + switches + ECC (Hamming code).
- Receives: seed_echo + desired_outputs + unused + ECC.
- Validates/corrects with ECC and seed match.
- Majority votes the 3 desired_outputs.
- Outputs voted bits to display.
- Feedback: Per-processor agreement bit (1 if matches majority/voted).
- Voting cycle: ~10Hz (timed internally assuming ~10MHz clk).
- Safe state: Outputs 0 if no valid majority.

Processors must echo seed, compute outputs, and check PRNG sequence for coherency.

## Inputs / Outputs

- **Dedicated Inputs (ui_in[7:0])**: Switches from demo board, forwarded to processors via SPI.
- **Dedicated Outputs (uo_out[7:0])**: Voted outputs to 7-segment display.
- **Bidirectional IOs (uio)**: SPI signals as above (oe configured for in/out).

## How to test

Connect the demo board:
- Toggle switches (ui_in) to simulate inputs.
- Use external MCUs/processors on SPI pins (uio) to simulate the 3 redundant CPUs.
- Observe voted outputs on the 7-segment display.
- For simulation: Use the testbench in `test/` to verify voting, ECC, and SPI.

## External hardware

- 3x Ambiq Apollo MCUs (or similar) connected via SPI.
- Demo board for testing (clock ~10MHz, switches, display). On the
demo board the RP2350 SPI1 in slave mode is connected to cs_n, sclk,
miso0, mosi0. For basic majority simulation miso0 and miso2 receive
the same signal from RP2350 SPI1.tx, simulating two agreeing processors.
