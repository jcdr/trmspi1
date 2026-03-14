// Simplified tt_um_tmr_voter.v without ECC
// Top module for Tiny Tapeout user project
// Implements SPI master majority voter for 3 redundant processors
// Pin assignment:
// uio_out[0] = sclk (shared, out)
// uio_out[1] = cs_n (shared, out)
// uio_out[2] = mosi1 (out)
// uio_out[3] = unused (MISO1 in)
// uio_out[4] = mosi2 (out)
// uio_out[5] = unused (MISO2 in)
// uio_out[6] = mosi3 (out)
// uio_out[7] = unused (MISO3 in)
// uio_oe = 8'b01010111 (outputs for 0,1,2,4,6; inputs for 3,5,7)
// uo_out = voted[7:0] (to 7-segment display)
// ui_in[7:0] = switches (inputs to send to CPUs)
// Assumes SPI mode 0: SCLK low idle, sample on rise, shift on fall
// Frame size: 24 bits
// Master to Slave: seed[7:0], agreement_byte[7:0] ({7'b0, agreement_bit}), switches[7:0]
// Slave to Master: seed_echo[7:0], desired_out[7:0], unused[7:0]
// Agreement bit: 1 if CPU output matches voted (part of majority), 0 otherwise
// If seed_echo != seed, invalid frame
// PRNG: 8-bit LFSR, polynomial x^8 + x^6 + x^5 + x^4 + 1, initial 8'h01 shared with CPUs
// Cycle: 10Hz voting (timer 20-bit, ~100ms at 10MHz clk)

module tt_um_tmr_voter (
    input  wire [7:0] ui_in,    // Dedicated inputs (switches)
    output wire [7:0] uo_out,   // Dedicated outputs (to display)
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire ena,            // Always 1 when the design is powered, so you can ignore it
    input  wire clk,            // System clock (assume ~10 MHz)
    input  wire rst_n           // Active low reset
);

    assign uio_oe = 8'b01010111;  // Outputs: 0,1,2,4,6; Inputs: 3,5,7
    assign uo_out = voted;        // Voted outputs to 7-segment display

    wire [7:0] switches = ui_in;

    assign uio_out[0] = sclk_out;
    assign uio_out[1] = cs_n_out;
    assign uio_out[2] = mosi1;
    assign uio_out[3] = 1'b0;     // Unused for input
    assign uio_out[4] = mosi2;
    assign uio_out[5] = 1'b0;     // Unused for input
    assign uio_out[6] = mosi3;
    assign uio_out[7] = 1'b0;     // Unused for input

    wire miso1 = uio_in[3];
    wire miso2 = uio_in[5];
    wire miso3 = uio_in[7];

    reg [7:0] sclk_div;           // For SCLK generation (~10MHz / 256 ~39kHz)
    wire sclk_int = sclk_div[7];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) sclk_div <= 0;
        else sclk_div <= sclk_div + 1;
    end

    reg sclk_out, cs_n_out;
    reg [4:0] bit_cnt;
    reg state;  // 0=IDLE, 1=TX_RX
    reg [23:0] shift_out1, shift_out2, shift_out3;
    reg [23:0] shift_in1, shift_in2, shift_in3;
    wire mosi1 = shift_out1[23];
    wire mosi2 = shift_out2[23];
    wire mosi3 = shift_out3[23];

    wire [7:0] seed_echo1 = shift_in1[23:16];
    wire [7:0] seed_echo2 = shift_in2[23:16];
    wire [7:0] seed_echo3 = shift_in3[23:16];
    wire [7:0] desired1 = shift_in1[15:8];
    wire [7:0] desired2 = shift_in2[15:8];
    wire [7:0] desired3 = shift_in3[15:8];

    wire all_valid = (seed_echo1 == prng_seed) & (seed_echo2 == prng_seed) & (seed_echo3 == prng_seed);

    wire [7:0] voted_temp;
    majority_voter3 temp_voter (
        .in1(desired1),
        .in2(desired2),
        .in3(desired3),
        .out(voted_temp)
    );

    reg [7:0] p1_out, p2_out, p3_out;
    reg [7:0] voted;

    wire p1_agree = (p1_out == voted);
    wire p2_agree = (p2_out == voted);
    wire p3_agree = (p3_out == voted);

    reg [7:0] prng_seed;  // Current seed
    reg [19:0] timer;     // For 10Hz voting (~1M cycles at 10MHz)

    // PRNG: 8-bit LFSR, polynomial x^8 + x^6 + x^5 + x^4 + 1
    wire prng_fb = prng_seed[7] ^ prng_seed[5] ^ prng_seed[4] ^ prng_seed[3];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prng_seed <= 8'h01;  // Initial seed, shared with CPUs
            timer <= 0;
            voted <= 0;
            p1_out <= 0; p2_out <= 0; p3_out <= 0;
            state <= 0;
            bit_cnt <= 0;
            shift_out1 <= 0; shift_out2 <= 0; shift_out3 <= 0;
            shift_in1 <= 0; shift_in2 <= 0; shift_in3 <= 0;
            sclk_out <= 0;
            cs_n_out <= 1;
        end else begin
            timer <= timer + 1;
            if (timer == 0) begin  // timer_done
                prng_seed <= {prng_seed[6:0], prng_fb};
                if (state == 0) begin  // IDLE
                    shift_out1 <= {prng_seed, {7'b0000000, p1_agree}, switches};
                    shift_out2 <= {prng_seed, {7'b0000000, p2_agree}, switches};
                    shift_out3 <= {prng_seed, {7'b0000000, p3_agree}, switches};
                    cs_n_out <= 0;
                    state <= 1;  // TX_RX
                    bit_cnt <= 0;
                end
            end
            if (state == 1) begin  // TX_RX
                sclk_out <= ~sclk_out;
                if (sclk_out == 0) begin  // Shift on fall
                    shift_out1 <= {shift_out1[22:0], 1'b0};
                    shift_out2 <= {shift_out2[22:0], 1'b0};
                    shift_out3 <= {shift_out3[22:0], 1'b0};
                    bit_cnt <= bit_cnt + 1;
                end else begin  // Sample on rise
                    shift_in1 <= {shift_in1[22:0], miso1};
                    shift_in2 <= {shift_in2[22:0], miso2};
                    shift_in3 <= {shift_in3[22:0], miso3};
                end
                if (bit_cnt == 24) begin
                    cs_n_out <= 1;
                    state <= 0;
                    // Process received
                    if (seed_echo1 == prng_seed) p1_out <= desired1;
                    if (seed_echo2 == prng_seed) p2_out <= desired2;
                    if (seed_echo3 == prng_seed) p3_out <= desired3;
                    if (all_valid) voted <= voted_temp;
                    else voted <= 0;
                end
            end
        end
    end

endmodule

module majority_voter3 #(parameter WIDTH = 8) (
    input [WIDTH-1:0] in1, in2, in3,
    output [WIDTH-1:0] out
);
    genvar i;
    generate
        for (i = 0; i < WIDTH; i = i + 1) begin : vote_gen
            assign out[i] = (in1[i] & in2[i]) | (in1[i] & in3[i]) | (in2[i] & in3[i]);
        end
    endgenerate
endmodule
