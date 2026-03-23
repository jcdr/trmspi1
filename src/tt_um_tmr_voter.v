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
// Master to Slave: next_prn[7:0], switches[7:0], majority_byte[7:0]
// Slave to Master: echoed_prn[7:0], desired_out[7:0], desired_valid[7:0]
// Majority bit: 1 if that CPU sent a valid bit matching the voted output, 0 otherwise
// PRNG: 8-bit LFSR, polynomial x^8 + x^6 + x^5 + x^4 + 1, one instance per SPI slice
// Reset seeds: SPI0=8'h2A, SPI1=8'h54, SPI2=8'hA8
// Validation: Each slice computes next_prn from its own current_prn, sends it, and compares the
// received echoed_prn against its previous current_prn. Invalid frames don't update that slice.
// Cycle: 1kHz voting (timer 13-bit, ~1ms at 8.192MHz clk)
// SCLK: 1.024MHz (main clk / 8)

module tt_um_tmr_voter_bit (
    input  wire in0,
    input  wire in1,
    input  wire in2,
    output wire voted
);

    assign voted = (in0 & in1) | (in0 & in2) | (in1 & in2);

endmodule

module tt_um_tmr_spi_slice #(
    parameter [7:0] INITIAL_PRN = 8'h2A
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start_frame,
    input  wire       sample_en,
    input  wire       shift_en,
    input  wire [4:0] bit_cnt,
    input  wire       miso,
    input  wire [7:0] switches,
    input  wire [7:0] voted,
    output wire       mosi,
    output wire [7:0] resolved
);

    reg [7:0] tx_shift;
    reg [7:0] rx_shift;
    reg [7:0] desired_r;
    reg [7:0] desired_valid_r;
    reg [7:0] current_prn;
    reg [7:0] previous_voted_r;
    reg [7:0] majority_r;
    reg       frame_valid_r;

    wire prng_fb = current_prn[7] ^ current_prn[5] ^ current_prn[4] ^ current_prn[3];
    wire [7:0] next_prn = {current_prn[6:0], prng_fb};
    wire [7:0] valid_mask = {8{frame_valid_r}} & desired_valid_r;
    wire [7:0] majority_next = valid_mask & ~(desired_r ^ voted);

    assign mosi = tx_shift[7];
    assign resolved = (desired_r & valid_mask) | (previous_voted_r & ~valid_mask);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_shift <= 0;
            rx_shift <= 0;
            desired_r <= 0;
            desired_valid_r <= 0;
            current_prn <= INITIAL_PRN;
            previous_voted_r <= 0;
            majority_r <= 0;
            frame_valid_r <= 0;
        end else begin
            if (start_frame) begin
                tx_shift <= next_prn;
                rx_shift <= 0;
                frame_valid_r <= 0;
            end

            if (sample_en) begin
                rx_shift <= (rx_shift << 1) | {7'b0, miso};
                if (bit_cnt[2:0] == 3'b111) begin
                    case (bit_cnt[4:3])
                        0: frame_valid_r <= ((rx_shift << 1) | {7'b0, miso}) == current_prn;
                        1: desired_r <= (rx_shift << 1) | {7'b0, miso};
                        2: desired_valid_r <= (rx_shift << 1) | {7'b0, miso};
                    endcase
                end
            end

            if (shift_en) begin
                if (bit_cnt == 5'd7) begin
                    tx_shift <= switches;
                end else if (bit_cnt == 5'd15) begin
                    tx_shift <= majority_r;
                end else begin
                    tx_shift <= {tx_shift[6:0], 1'b0};
                end

                if (bit_cnt == 5'd23) begin
                    previous_voted_r <= voted;
                    majority_r <= majority_next;
                    current_prn <= next_prn;
                end
            end
        end
    end

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

    wire start_frame = (timer == 0) && (state == 0);
    wire sample_en = (state == 1) && (sclk_div == 3'b111) && (sclk_out == 0);
    wire shift_en = (state == 1) && (sclk_div == 3'b111) && (sclk_out == 1);

    wire mosi0;
    wire mosi1;
    wire mosi2;
    wire [7:0] resolved0;
    wire [7:0] resolved1;
    wire [7:0] resolved2;
    wire [7:0] new_voted;

    tt_um_tmr_spi_slice #(
        .INITIAL_PRN(8'h2A)
    ) spi0 (
        .clk(clk),
        .rst_n(rst_n),
        .start_frame(start_frame),
        .sample_en(sample_en),
        .shift_en(shift_en),
        .bit_cnt(bit_cnt),
        .miso(miso0),
        .switches(switches),
        .voted(new_voted),
        .mosi(mosi0),
        .resolved(resolved0)
    );

    tt_um_tmr_spi_slice #(
        .INITIAL_PRN(8'h54)
    ) spi1 (
        .clk(clk),
        .rst_n(rst_n),
        .start_frame(start_frame),
        .sample_en(sample_en),
        .shift_en(shift_en),
        .bit_cnt(bit_cnt),
        .miso(miso1),
        .switches(switches),
        .voted(new_voted),
        .mosi(mosi1),
        .resolved(resolved1)
    );

    tt_um_tmr_spi_slice #(
        .INITIAL_PRN(8'hA8)
    ) spi2 (
        .clk(clk),
        .rst_n(rst_n),
        .start_frame(start_frame),
        .sample_en(sample_en),
        .shift_en(shift_en),
        .bit_cnt(bit_cnt),
        .miso(miso2),
        .switches(switches),
        .voted(new_voted),
        .mosi(mosi2),
        .resolved(resolved2)
    );

    genvar i;
    generate
        for (i = 0; i < 8; i = i + 1) begin : gen_voter_bits
            tt_um_tmr_voter_bit voter_bit (
                .in0(resolved0[i]),
                .in1(resolved1[i]),
                .in2(resolved2[i]),
                .voted(new_voted[i])
            );
        end
    endgenerate

    reg [7:0] voted;
    reg [12:0] timer;     // For 1kHz voting (~8192 cycles at 8.192MHz)

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            timer <= 13'd4096;  // start first transaction after ~0.5 ms (half of the 1 kHz cycle)
            voted <= 0;
            state <= 0;
            bit_cnt <= 0;
            sclk_out <= 0;
            cs_n_out <= 1;
        end else begin
            timer <= timer + 1;
            if (start_frame) begin
                cs_n_out <= 0;
                state <= 1;  // TX_RX
                bit_cnt <= 0;
            end
            if (state == 1) begin  // TX_RX
                if (sclk_div == 3'b111) begin
                    sclk_out <= ~sclk_out;
                    if (shift_en) begin
                        bit_cnt <= bit_cnt + 1;
                        if (bit_cnt == 5'd23) begin
                            cs_n_out <= 1;
                            state <= 0;
                            voted <= new_voted;
                        end
                    end
                end
            end
        end
    end

endmodule
