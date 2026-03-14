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
// Master to Slave: seed[7:0], agreement_byte[7:0] ({7'b0, agreement_bit}), switches[7:0]
// Slave to Master: seed_echo[7:0], desired_out[7:0], unused[7:0]
// Agreement bit: 1 if CPU output matches voted (part of majority), 0 otherwise
// If seed_echo != seed, invalid frame
// PRNG: 8-bit LFSR, polynomial x^8 + x^6 + x^5 + x^4 + 1, initial 8'h01 shared with CPUs
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
    wire sclk_int = sclk_div[2];  // Toggle every 4 clk (divide by 8 overall)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sclk_div <= 0;
        else sclk_div <= sclk_div + 1;
    end

    reg sclk_out, cs_n_out;
    reg [4:0] bit_cnt;
    reg state;  // 0=IDLE, 1=TX_RX
    reg [23:0] shift_out0, shift_out1, shift_out2;
    reg [23:0] shift_in0, shift_in1, shift_in2;
    wire mosi0 = shift_out0[23];
    wire mosi1 = shift_out1[23];
    wire mosi2 = shift_out2[23];

    wire [7:0] seed_echo0 = shift_in0[23:16];
    wire [7:0] seed_echo1 = shift_in1[23:16];
    wire [7:0] seed_echo2 = shift_in2[23:16];
    wire [7:0] desired0 = shift_in0[15:8];
    wire [7:0] desired1 = shift_in1[15:8];
    wire [7:0] desired2 = shift_in2[15:8];

    wire all_valid = (seed_echo0 == prng_seed) & (seed_echo1 == prng_seed) & (seed_echo2 == prng_seed);

    wire [7:0] voted_temp;
    majority_voter3 temp_voter (
        .in0(desired0),
        .in1(desired1),
        .in2(desired2),
        .out(voted_temp)
    );

    reg [7:0] p0_out, p1_out, p2_out;
    reg [7:0] voted;

    wire p0_agree = (p0_out == voted);
    wire p1_agree = (p1_out == voted);
    wire p2_agree = (p2_out == voted);

    reg [7:0] prng_seed;  // Current seed
    reg [12:0] timer;     // For 1kHz voting (~8192 cycles at 8.192MHz)

    // PRNG: 8-bit LFSR, polynomial x^8 + x^6 + x^5 + x^4 + 1
    wire prng_fb = prng_seed[7] ^ prng_seed[5] ^ prng_seed[4] ^ prng_seed[3];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prng_seed <= 8'h01;  // Initial seed, shared with CPUs
            timer <= 0;
            voted <= 0;
            p0_out <= 0; p1_out <= 0; p2_out <= 0;
            state <= 0;
            bit_cnt <= 0;
            shift_out0 <= 0; shift_out1 <= 0; shift_out2 <= 0;
            shift_in0 <= 0; shift_in1 <= 0; shift_in2 <= 0;
            sclk_out <= 0;
            cs_n_out <= 1;
        end else begin
            timer <= timer + 1;
            if (timer == 0) begin  // timer_done
                prng_seed <= {prng_seed[6:0], prng_fb};
                if (state == 0) begin  // IDLE
                    shift_out0 <= {prng_seed, {7'b0000000, p0_agree}, switches};
                    shift_out1 <= {prng_seed, {7'b0000000, p1_agree}, switches};
                    shift_out2 <= {prng_seed, {7'b0000000, p2_agree}, switches};
                    cs_n_out <= 0;
                    state <= 1;  // TX_RX
                    bit_cnt <= 0;
                end
            end
            if (state == 1) begin  // TX_RX
                if (sclk_div == 3'b111) begin  // Toggle SCLK every 8 main clk cycles
                    sclk_out <= ~sclk_out;
                    if (sclk_out == 0) begin  // Shift on fall
                        shift_out0 <= {shift_out0[22:0], 1'b0};
                        shift_out1 <= {shift_out1[22:0], 1'b0};
                        shift_out2 <= {shift_out2[22:0], 1'b0};
                        bit_cnt <= bit_cnt + 1;
                    end else begin  // Sample on rise
                        shift_in0 <= {shift_in0[22:0], miso0};
                        shift_in1 <= {shift_in1[22:0], miso1};
                        shift_in2 <= {shift_in2[22:0], miso2};
                    end
                    if (bit_cnt == 24) begin
                        cs_n_out <= 1;
                        state <= 0;
                        // Process received
                        if (seed_echo0 == prng_seed) p0_out <= desired0;
                        if (seed_echo1 == prng_seed) p1_out <= desired1;
                        if (seed_echo2 == prng_seed) p2_out <= desired2;
                        if (all_valid) voted <= voted_temp;
                        else voted <= 0;
                    end
                end
            end
        end
    end

endmodule

module majority_voter3 #(parameter WIDTH = 8) (
    input [WIDTH-1:0] in0, in1, in2,
    output [WIDTH-1:0] out
);
    genvar i;
    generate
        for (i = 0; i < WIDTH; i = i + 1) begin : vote_gen
            assign out[i] = (in0[i] & in1[i]) | (in0[i] & in2[i]) | (in1[i] & in2[i]);
        end
    endgenerate
endmodule
