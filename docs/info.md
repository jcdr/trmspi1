# TMR SPI Voter for Redundant Processors

## What is this?

This project is a Triple Modular Redundancy (TMR) voter chip that exchanges one SPI frame per millisecond with 3 redundant CPUs and publishes an 8-bit voted output.

- **Inputs**: `ui_in[7:0]` is the hardware input byte sent to all 3 CPUs.
- **Outputs**: `uo_out[7:0]` is the voted output byte.
- **SPI interface** on `uio`:
  - shared `cs_n` on `uio[0]`
  - shared `sclk` on `uio[1]`
  - `miso0` / `mosi0` on `uio[2]` / `uio[3]`
  - `miso1` / `mosi1` on `uio[4]` / `uio[5]`
  - `miso2` / `mosi2` on `uio[6]` / `uio[7]`

The chip is the SPI master. It polls all 3 CPUs in parallel and votes the 3 returned responses bit by bit.

## SPI Protocol

The design uses SPI mode 0:
- `sclk` idle low
- data sampled on the rising edge
- data shifted on the falling edge

Each frame is 24 bits.

### Master to CPU

The chip sends 3 bytes to each CPU:
1. `next_prn`
2. `switches`
3. `majority_byte`

### CPU to master

Each CPU sends 3 bytes back:
1. `echoed_prn`
2. `desired_out`
3. `desired_valid`

## PRG Handling

Each SPI slice has its own 8-bit LFSR with polynomial `x^8 + x^6 + x^5 + x^4 + 1`.

The 3 reset seeds are different:
- CPU0 slice: `0x2A`
- CPU1 slice: `0x54`
- CPU2 slice: `0xA8`

The CPU side does not need to compute the PRG sequence. It only needs to:
- receive `next_prn`
- stage that byte locally
- echo that staged byte on the following frame

So the intended sequence is:
- on frame `N`, the master sends a new PRG byte
- the CPU stores that byte
- on frame `N+1`, the CPU echoes that stored byte
- the master compares the echoed byte with the PRG state it already kept locally for that slice

If the echoed byte is wrong, that slice is considered invalid for that frame.

## Voting Rules

Each CPU returns:
- `desired_out`: the output bits it wants
- `desired_valid`: one validity bit per output bit

For each slice and each bit:
- if the frame is valid and the `desired_valid` bit is `1`, the slice contributes the CPU's `desired_out` bit
- otherwise the slice falls back to its own stored copy of the previously voted output bit

That per-slice fallback state is kept redundantly inside each slice so that a bad frame does not force a common single-point output register into the voting path.

The final output vote is a pure bitwise 2-of-3 majority of the 3 slice-resolved bits.

## Majority Feedback Byte

The third byte sent by the master is a per-CPU `majority_byte`.

For each CPU bit:
- `1` means that CPU sent a valid bit and that bit matched the final voted output
- `0` means that CPU bit was invalid or disagreed with the final voted output

This byte is sent one frame later, because it is computed from the frame that just completed and transmitted on the next frame.

If a CPU frame is rejected because of a bad echoed PRG byte, that CPU's next `majority_byte` is cleared.

## Timing

- project clock: `8.192 MHz`
- SPI clock: about `1.024 MHz`
- frame period: `1 ms`
- first frame after reset: about `0.5 ms`

## How to test

Run the RTL cocotb tests from the project root:

```sh
./build -t
```

Print the latest utilization and cell counts without building:

```sh
./build -s
```

Run tests and then print the same summary:

```sh
./build -t -s
```

The current cocotb suite covers:
- basic valid voting
- per-bit validity masks
- majority-byte timing and contents
- rejected frames after bad echoed PRG bytes
- fallback to the previously voted state
- a 256-frame randomized no-reset stress test with one injected SPI bit fault per frame

Waveforms are written to `test/tb.fst`.

## External hardware

This design expects 3 external CPUs or equivalent SPI slaves. Each one needs only:
- one MISO line to the chip
- one MOSI line from the chip
- the shared `cs_n`
- the shared `sclk`

Each CPU should:
- stage the received `next_prn`
- compute its `desired_out`
- compute its `desired_valid`
- return those 3 bytes on the next poll
