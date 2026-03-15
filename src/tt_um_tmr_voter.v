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
// Master to Slave: current_prn[7:0], agreement_byte[7:0] ({7'b0, agreement_bit}), switches[7:0]
// Slave to Master: next_prn[7:0], desired_out[7:0], unused[7:0]
// Agreement bit: 1 if CPU output matches voted (part of majority), 0 otherwise
// PRNG: 8-bit LFSR, polynomial x^8 + x^6 + x^5 + x^4 + 1, initial 8'h01 shared with CPUs
// Validation: Compute next_prn from sent current_prn; if received next_prn != computed next_prn,
// discard that CPU's frame (don't update pX_out)
// Cycle: 1kHz voting (timer 13-bit, ~1ms at 8.192MHz clk)
// SCLK: 1.024MHz (main clk / 8)

module tt_um_tmr_voter (
    input  wire [7:0] ui_in,    // Dedicated inputs (switches)
    output wire [7:0] uo_out,   // Dedicated outputs (to display)
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire ena,            // Always 1 when the design is powered, so you can ignore it
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

    // NEW: Combinational wires for updates (fixes accumulation and update timing)
    wire valid0 = (received_next0 == next_prn);
    wire valid1 = (received_next1 == next_prn);
    wire valid2 = (received_next2 == next_prn);
    wire [7:0] new_p0_out = valid0 ? desired0 : p0_out;
    wire [7:0] new_p1_out = valid1 ? desired1 : p1_out;
    wire [7:0] new_p2_out = valid2 ? desired2 : p2_out;
    wire [2:0] new_valid_count = valid0 + valid1 + valid2;
    wire [7:0] new_voted = (new_p0_out & new_p1_out) | (new_p0_out & new_p2_out) | (new_p1_out & new_p2_out);

    reg [7:0] p0_out, p1_out, p2_out;
    reg [7:0] voted;

    wire p0_agree = (p0_out == voted);
    wire p1_agree = (p1_out == voted);
    wire p2_agree = (p2_out == voted);

    reg [7:0] current_prn;  // Current PRN (sent to CPUs)
    wire [7:0] next_prn;     // Computed next PRN (for comparison)

    // PRNG algorithm: 8-bit LFSR, polynomial x^8 + x^6 + x^5 + x^4 + 1 (taps at positions 8,6,5,4)
    // This is a well-known maximal-length LFSR for 8 bits, period 255, simple bitwise XOR implementation.
    // Parameters: Initial seed 8'h01 (shared with CPUs), feedback = bit7 ^ bit5 ^ bit4 ^ bit3
    wire prng_fb = current_prn[7] ^ current_prn[5] ^ current_prn[4] ^ current_prn[3];
    assign next_prn = {current_prn[6:0], prng_fb};

    reg [12:0] timer;     // For 1kHz voting (~8192 cycles at 8.192MHz)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_prn <= 8'h01;  // Initial seed, shared with CPUs
            timer <= 13'd4096;  // start first transaction after ~0.5 ms (half of the 1 kHz cycle)
            voted <= 0;
            p0_out <= 0; p1_out <= 0; p2_out <= 0;
            state <= 0;
            bit_cnt <= 0;
            // MINIMAL FIX: removed phase/bit_pos init
            tx_shift0 <= 0; tx_shift1 <= 0; tx_shift2 <= 0;
            rx_shift0 <= 0; rx_shift1 <= 0; rx_shift2 <= 0;
            received_next0 <= 0; received_next1 <= 0; received_next2 <= 0;
            desired0 <= 0; desired1 <= 0; desired2 <= 0;
            sclk_out <= 0;
            cs_n_out <= 1;
        end else begin
            timer <= timer + 1;
            if (timer == 0) begin  // timer_done: Start new cycle
                if (state == 0) begin  // IDLE
                    tx_shift0 <= current_prn;
                    tx_shift1 <= current_prn;
                    tx_shift2 <= current_prn;
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
                                // 2: ignore unused
                            endcase
                        end
                    end else begin  // Falling edge: shift/load
                        // MINIMAL FIX: clean byte load + increment
                        if (bit_cnt[2:0] == 3'b000 && bit_cnt[4:3] < 2) begin
                            tx_shift0 <= (bit_cnt[4:3] == 0) ? {7'b0000000, p0_agree} : switches;
                            tx_shift1 <= (bit_cnt[4:3] == 0) ? {7'b0000000, p1_agree} : switches;
                            tx_shift2 <= (bit_cnt[4:3] == 0) ? {7'b0000000, p2_agree} : switches;
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
                            if (new_valid_count >= 2) begin
                                voted <= new_voted;
                            end  // else voted remains untouched (safe, as per previous state)
                            // Advance PRNG for next cycle
                            current_prn <= next_prn;
                        end
                    end
                end
            end
        end
    end

endmodule
