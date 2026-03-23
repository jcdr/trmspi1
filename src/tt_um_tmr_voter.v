// Simplified tt_um_tmr_voter.v without ECC
// Top module for Tiny Tapeout user project
// Implements SPI master majority voter for 3 redundant processors
// Pin assignment:
// uio_out[0] = cs_n (shared, out)
// uio_out[1] = sclk (shared, out)
// uio_out[2] = miso0 (in)
// uio_out[3] = mosi0 (out)
// uio_out[4] = miso1 (in)
// uio_out[5] = mosi1 (out)
// uio_out[6] = miso2 (in)
// uio_out[7] = mosi2 (out)
// uio_oe = 8'b10101011 (outputs for 0,1,3,5,7; inputs for 2,4,6)
// uo_out = voted[7:0] (to 7-segment display)
// ui_in[7:0] = switches (inputs to send to CPUs)
// Assumes SPI mode 0 (CPOL=0, CPHA=0): SCLK idles low (clock polarity 0),
// data sampled on rising clock edge (first edge), shifted on falling clock edge (second edge).
// In other words: Data is clocked out on the falling edge and clocked in on the rising edge, with SCLK starting low.
// This matches the default SPI mode on the RP2350 microcontroller (e.g., in the Raspberry Pi Pico 2 SDK and hardware).
// Frame size: 24 bits
// Master to Slave: next_prn[7:0], switches[7:0], majority_valid[7:0]
// Slave to Master: echoed_prn[7:0], desired_out[7:0], desired_valid[7:0]
// Majority valid bit: 1 if that voted output bit currently has a valid 2-of-3 majority, 0 otherwise
// PRNG: 8-bit LFSR, polynomial x^8 + x^6 + x^5 + x^4 + 1, initial 8'h2A shared with CPUs
// Validation: Compute next_prn from current_prn, send it, and compare the received echoed_prn
// against the previous current_prn. Invalid frames don't update pX_out.
// Cycle: 1kHz voting (timer 13-bit, ~1ms at 8.192MHz clk)
// SCLK: 1.024MHz (main clk / 8)

module tt_um_tmr_voter_bit (
    input  wire desired0,
    input  wire valid0,
    input  wire desired1,
    input  wire valid1,
    input  wire desired2,
    input  wire valid2,
    input  wire previous_voted,
    output wire voted,
    output wire majority0,
    output wire majority1,
    output wire majority2
);

    wire voted_one = (valid0 & valid1 & desired0 & desired1) |
                     (valid0 & valid2 & desired0 & desired2) |
                     (valid1 & valid2 & desired1 & desired2);
    wire voted_zero = (valid0 & valid1 & ~desired0 & ~desired1) |
                      (valid0 & valid2 & ~desired0 & ~desired2) |
                      (valid1 & valid2 & ~desired1 & ~desired2);
    wire voted_valid = voted_one | voted_zero;

    assign voted = voted_one | (~voted_valid & previous_voted);
    assign majority0 = valid0 & ((valid1 & (desired0 == desired1)) | (valid2 & (desired0 == desired2)));
    assign majority1 = valid1 & ((valid0 & (desired1 == desired0)) | (valid2 & (desired1 == desired2)));
    assign majority2 = valid2 & ((valid0 & (desired2 == desired0)) | (valid1 & (desired2 == desired1)));

endmodule

module tt_um_tmr_voter (
    input  wire [7:0] ui_in,    // Dedicated inputs (switches)
    output wire [7:0] uo_out,   // Dedicated outputs (to display)
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire ena,            // Always 1 when the design is powered, so you can ignore it
    /* verilator lint_on  UNUSEDSIGNAL */
    input  wire clk,            // System clock (assume ~8.192 MHz)
    input  wire rst_n           // Active low reset
);

    assign uio_oe = 8'b10101011;  // Outputs: 0,1,3,5,7; Inputs: 2,4,6
    assign uo_out = voted;        // Voted outputs to 7-segment display

    wire [7:0] switches = ui_in;

    assign uio_out[0] = cs_n_out;
    assign uio_out[1] = sclk_out;
    assign uio_out[2] = 1'b0;     // Unused for input
    assign uio_out[3] = mosi0;
    assign uio_out[4] = 1'b0;     // Unused for input
    assign uio_out[5] = mosi1;
    assign uio_out[6] = 1'b0;     // Unused for input
    assign uio_out[7] = mosi2;

    wire miso0 = uio_in[2];
    wire miso1 = uio_in[4];
    wire miso2 = uio_in[6];
    wire _unused_uio_in_0 = uio_in[0];  // silences the warning
    wire _unused_uio_in_1 = uio_in[1];  // silences the warning
    wire _unused_uio_in_3 = uio_in[3];  // silences the warning
    wire _unused_uio_in_5 = uio_in[5];  // silences the warning
    wire _unused_uio_in_7 = uio_in[7];  // silences the warning

    reg [2:0] sclk_div;           // For SCLK generation (~8.192MHz / 8 = 1.024MHz)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sclk_div <= 0;
        else sclk_div <= sclk_div + 1;
    end

    reg sclk_out, cs_n_out;
    reg [4:0] bit_cnt;
    reg state;  // 0=IDLE, 1=TX_RX

    // MINIMAL FIX: removed unused phase/bit_pos (was causing off-by-one race)
    // bit_cnt[4:3] = current byte (0=next_prn, 1=agreement, 2=switches)
    // bit_cnt[2:0] = bit inside byte

    reg [7:0] tx_shift0, tx_shift1, tx_shift2;
    reg [7:0] rx_shift0, rx_shift1, rx_shift2;
    wire mosi0 = tx_shift0[7];
    wire mosi1 = tx_shift1[7];
    wire mosi2 = tx_shift2[7];

    reg [7:0] received_next0, received_next1, received_next2;
    reg [7:0] desired0, desired1, desired2;
    reg [7:0] desired_valid0, desired_valid1, desired_valid2;

    // NEW: Combinational wires for updates (fixes accumulation and update timing)
    wire valid0 = (received_next0 == current_prn);
    wire valid1 = (received_next1 == current_prn);
    wire valid2 = (received_next2 == current_prn);
    wire [7:0] new_p0_out = valid0 ? desired0 : p0_out;
    wire [7:0] new_p1_out = valid1 ? desired1 : p1_out;
    wire [7:0] new_p2_out = valid2 ? desired2 : p2_out;
    wire [7:0] new_p0_valid = valid0 ? desired_valid0 : p0_valid;
    wire [7:0] new_p1_valid = valid1 ? desired_valid1 : p1_valid;
    wire [7:0] new_p2_valid = valid2 ? desired_valid2 : p2_valid;
    wire [7:0] majority0;
    wire [7:0] majority1;
    wire [7:0] majority2;
    wire [7:0] new_voted;
    wire [7:0] new_voted_valid = majority0 | majority1 | majority2;

    genvar i;
    generate
        for (i = 0; i < 8; i = i + 1) begin : gen_voter_bits
            tt_um_tmr_voter_bit voter_bit (
                .desired0(new_p0_out[i]),
                .valid0(new_p0_valid[i]),
                .desired1(new_p1_out[i]),
                .valid1(new_p1_valid[i]),
                .desired2(new_p2_out[i]),
                .valid2(new_p2_valid[i]),
                .previous_voted(voted[i]),
                .voted(new_voted[i]),
                .majority0(majority0[i]),
                .majority1(majority1[i]),
                .majority2(majority2[i])
            );
        end
    endgenerate

    reg [7:0] p0_out, p1_out, p2_out;
    reg [7:0] p0_valid, p1_valid, p2_valid;
    reg [7:0] voted;
    reg [7:0] voted_valid;

    reg [7:0] current_prn;  // Previous PRN expected back from CPUs this frame
    wire [7:0] next_prn;    // Next PRN sent to CPUs this frame

    // PRNG algorithm: 8-bit LFSR, polynomial x^8 + x^6 + x^5 + x^4 + 1 (taps at positions 8,6,5,4)
    // This is a well-known maximal-length LFSR for 8 bits, period 255, simple bitwise XOR implementation.
    // Parameters: Initial seed 8'h01 (shared with CPUs), feedback = bit7 ^ bit5 ^ bit4 ^ bit3
    wire prng_fb = current_prn[7] ^ current_prn[5] ^ current_prn[4] ^ current_prn[3];
    assign next_prn = {current_prn[6:0], prng_fb};

    reg [12:0] timer;     // For 1kHz voting (~8192 cycles at 8.192MHz)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_prn <= 8'h2A;  // Initial seed, shared with CPUs
            timer <= 13'd4096;  // start first transaction after ~0.5 ms (half of the 1 kHz cycle)
            voted <= 0;
            p0_out <= 0; p1_out <= 0; p2_out <= 0;
            p0_valid <= 0; p1_valid <= 0; p2_valid <= 0;
            voted_valid <= 0;
            state <= 0;
            bit_cnt <= 0;
            // MINIMAL FIX: removed phase/bit_pos init
            tx_shift0 <= 0; tx_shift1 <= 0; tx_shift2 <= 0;
            rx_shift0 <= 0; rx_shift1 <= 0; rx_shift2 <= 0;
            received_next0 <= 0; received_next1 <= 0; received_next2 <= 0;
            desired0 <= 0; desired1 <= 0; desired2 <= 0;
            desired_valid0 <= 0; desired_valid1 <= 0; desired_valid2 <= 0;
            sclk_out <= 0;
            cs_n_out <= 1;
        end else begin
            timer <= timer + 1;
            if (timer == 0) begin  // timer_done: Start new cycle
                if (state == 0) begin  // IDLE
                    tx_shift0 <= next_prn;
                    tx_shift1 <= next_prn;
                    tx_shift2 <= next_prn;
                    rx_shift0 <= 0;
                    rx_shift1 <= 0;
                    rx_shift2 <= 0;
                    cs_n_out <= 0;
                    state <= 1;  // TX_RX
                    bit_cnt <= 0;
                    // MINIMAL FIX: removed phase/bit_pos reset
                end
            end
            if (state == 1) begin  // TX_RX
                if (sclk_div == 3'b111) begin
                    sclk_out <= ~sclk_out;
                    if (sclk_out == 0) begin  // Rising edge: sample
                        rx_shift0 <= (rx_shift0 << 1) | {7'b0, miso0};
                        rx_shift1 <= (rx_shift1 << 1) | {7'b0, miso1};
                        rx_shift2 <= (rx_shift2 << 1) | {7'b0, miso2};
                        // MINIMAL FIX: use bit_cnt[2:0] for byte end + bit_cnt[4:3] for phase
                        if (bit_cnt[2:0] == 3'b111) begin
                            case (bit_cnt[4:3])
                                0: begin
                                    received_next0 <= (rx_shift0 << 1) | {7'b0, miso0};
                                    received_next1 <= (rx_shift1 << 1) | {7'b0, miso1};
                                    received_next2 <= (rx_shift2 << 1) | {7'b0, miso2};
                                end
                                1: begin
                                    desired0 <= (rx_shift0 << 1) | {7'b0, miso0};
                                    desired1 <= (rx_shift1 << 1) | {7'b0, miso1};
                                    desired2 <= (rx_shift2 << 1) | {7'b0, miso2};
                                end
                                2: begin
                                    desired_valid0 <= (rx_shift0 << 1) | {7'b0, miso0};
                                    desired_valid1 <= (rx_shift1 << 1) | {7'b0, miso1};
                                    desired_valid2 <= (rx_shift2 << 1) | {7'b0, miso2};
                                end
                            endcase
                        end
                    end else begin  // Falling edge: shift/load
                        // Load the next TX byte after the previous 8 bits have completed
                        if (bit_cnt == 5'd7) begin
                            tx_shift0 <= switches;
                            tx_shift1 <= switches;
                            tx_shift2 <= switches;
                        end else if (bit_cnt == 5'd15) begin
                            tx_shift0 <= voted_valid;
                            tx_shift1 <= voted_valid;
                            tx_shift2 <= voted_valid;
                        end else begin
                            tx_shift0 <= {tx_shift0[6:0], 1'b0};
                            tx_shift1 <= {tx_shift1[6:0], 1'b0};
                            tx_shift2 <= {tx_shift2[6:0], 1'b0};
                        end
                        bit_cnt <= bit_cnt + 1;
                        if (bit_cnt == 23) begin
                            cs_n_out <= 1;
                            state <= 0;
                            // Process received: Use combinational wires for validation/accumulation/updates
                            p0_out <= new_p0_out;
                            p1_out <= new_p1_out;
                            p2_out <= new_p2_out;
                            p0_valid <= new_p0_valid;
                            p1_valid <= new_p1_valid;
                            p2_valid <= new_p2_valid;
                            voted <= new_voted;
                            voted_valid <= new_voted_valid;
                            // Advance PRNG for next cycle
                            current_prn <= next_prn;
                        end
                    end
                end
            end
        end
    end

endmodule
