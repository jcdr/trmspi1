// tt_um_tmr_voter.v
// Top module for Tiny Tapeout user project
// Implements SPI master majority voter for 3 redundant processors with ECC
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
// Frame size: 32 bits (Hamming(31,24) codeword + 1 pad bit)
// Master to Slave: seed[7:0], agreement_byte[7:0] ({7'b0, agreement_bit}), switches[7:0], ecc
// Slave to Master: seed_echo[7:0], desired_out[7:0], unused[7:0], ecc
// ECC: Hamming(31,24) for single error correction on 24-bit data (unused positions 30,31 set to 0)
// Agreement bit: 1 if CPU output matches voted (part of majority), 0 otherwise
// If seed_echo != seed after correction, invalid frame
// PRNG: 8-bit LFSR, polynomial x^8 + x^6 + x^5 + x^4 + 1, initial 8'h01 shared with CPUs
// Cycle: 10Hz voting (timer 20-bit, ~100ms at 10MHz clk)

module tt_um_tmr_voter (
    input  wire [7:0] ui_in,    // Dedicated inputs (switches)
    output wire [7:0] uo_out,   // Dedicated outputs (to display)
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire ena,            // Always 1 when powered
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

    wire sclk_out, cs_n_out;
    wire mosi1, mosi2, mosi3;

    reg [7:0] p1_out, p2_out, p3_out;
    reg p1_valid, p2_valid, p3_valid;
    reg [7:0] voted;

    wire p1_agree = (p1_out == voted);
    wire p2_agree = (p2_out == voted);
    wire p3_agree = (p3_out == voted);

    reg [7:0] prng_seed;  // Current seed
    reg [19:0] timer;     // For 10Hz voting (~1M cycles at 10MHz)

    // PRNG: 8-bit LFSR, polynomial x^8 + x^6 + x^5 + x^4 + 1
    wire prng_fb = prng_seed[7] ^ prng_seed[5] ^ prng_seed[4] ^ prng_seed[3];
    task advance_prng;
        prng_seed <= {prng_seed[6:0], prng_fb};
    endtask

    majority_voter3 #(.WIDTH(8)) voter (
        .in1(p1_out),
        .in2(p2_out),
        .in3(p3_out),
        .out(voted)
    );

    spi_master_channel #(.ID(1)) channel1 (
        .clk(clk), .rst_n(rst_n), .sclk_int(sclk_int),
        .sclk_out(sclk_out), .cs_n_out(cs_n_out),
        .mosi(mosi1), .miso(miso1),
        .seed(prng_seed), .agreement(p1_agree), .switches(switches),
        .processor_out(p1_out), .valid(p1_valid),
        .timer_done(timer_done)
    );

    spi_master_channel #(.ID(2)) channel2 (
        .clk(clk), .rst_n(rst_n), .sclk_int(sclk_int),
        .sclk_out(sclk_out), .cs_n_out(cs_n_out),
        .mosi(mosi2), .miso(miso2),
        .seed(prng_seed), .agreement(p2_agree), .switches(switches),
        .processor_out(p2_out), .valid(p2_valid),
        .timer_done(timer_done)
    );

    spi_master_channel #(.ID(3)) channel3 (
        .clk(clk), .rst_n(rst_n), .sclk_int(sclk_int),
        .sclk_out(sclk_out), .cs_n_out(cs_n_out),
        .mosi(mosi3), .miso(miso3),
        .seed(prng_seed), .agreement(p3_agree), .switches(switches),
        .processor_out(p3_out), .valid(p3_valid),
        .timer_done(timer_done)
    );

    wire timer_done = (timer == 0);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prng_seed <= 8'h01;  // Initial seed, shared with CPUs
            timer <= 0;
            voted <= 0;
            p1_valid <= 0; p2_valid <= 0; p3_valid <= 0;
        end else begin
            timer <= timer + 1;
            if (timer_done) begin
                advance_prng;
            end
            if (p1_valid & p2_valid & p3_valid) begin
                p1_valid <= 0; p2_valid <= 0; p3_valid <= 0;
            end else if (!(p1_valid | p2_valid | p3_valid)) begin
                voted <= 0;  // Safe state
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

module spi_master_channel #(parameter ID = 1) (
    input clk, rst_n, sclk_int,
    output reg sclk_out, cs_n_out,
    output mosi,
    input miso,
    input [7:0] seed, 
    input agreement,  // 1-bit per processor
    input [7:0] switches,
    output reg [7:0] processor_out,
    output reg valid,
    input timer_done
);

    localparam STATE_IDLE = 0;
    localparam STATE_TX_RX = 1;

    reg [1:0] state;
    reg [5:0] bit_cnt;  // 32 bits
    reg [31:0] shift_out;
    reg [31:0] shift_in;

    wire [23:0] data_out = {seed, {7'b0000000, agreement}, switches};
    wire [30:0] code_out;
    hamming_encode24 enc_out (
        .data(data_out),
        .code(code_out)
    );

    wire [30:0] received_code;
    wire [23:0] data_in;
    wire error_corrected;

    assign mosi = shift_out[31];  // MSB first

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= STATE_IDLE;
            bit_cnt <= 0;
            shift_out <= 0;
            shift_in <= 0;
            valid <= 0;
            processor_out <= 0;
            cs_n_out <= 1;
            sclk_out <= 0;
        end else begin
            if (timer_done && state == STATE_IDLE) begin
                // Prepare frame
                shift_out <= {code_out, 1'b0};
                cs_n_out <= 0;
                state <= STATE_TX_RX;
                bit_cnt <= 0;
            end
            if (state == STATE_TX_RX) begin
                sclk_out <= ~sclk_out;
                if (sclk_out == 0) begin  // Shift on fall
                    shift_out <= {shift_out[30:0], 1'b0};
                    bit_cnt <= bit_cnt + 1;
                end else begin  // Sample on rise
                    shift_in <= {shift_in[30:0], miso};
                end
                if (bit_cnt == 32) begin
                    cs_n_out <= 1;
                    state <= STATE_IDLE;
                    // Process received
                    hamming_decode24 dec_in (
                        .code(shift_in[31:1]),
                        .data(data_in),
                        .error_corrected(error_corrected)
                    );
                    if (data_in[23:16] == seed) begin
                        processor_out <= data_in[15:8];
                        valid <= 1;
                    end else begin
                        valid <= 0;
                    end
                end
            end
        end
    end

endmodule

module hamming_encode24 (
    input [23:0] data,
    output [30:0] code
);
    wire [31:1] pos;
    assign pos[3] = data[0];
    assign pos[5] = data[1];
    assign pos[6] = data[2];
    assign pos[7] = data[3];
    assign pos[9] = data[4];
    assign pos[10] = data[5];
    assign pos[11] = data[6];
    assign pos[12] = data[7];
    assign pos[13] = data[8];
    assign pos[14] = data[9];
    assign pos[15] = data[10];
    assign pos[17] = data[11];
    assign pos[18] = data[12];
    assign pos[19] = data[13];
    assign pos[20] = data[14];
    assign pos[21] = data[15];
    assign pos[22] = data[16];
    assign pos[23] = data[17];
    assign pos[24] = data[18];
    assign pos[25] = data[19];
    assign pos[26] = data[20];
    assign pos[27] = data[21];
    assign pos[28] = data[22];
    assign pos[29] = data[23];
    assign pos[30] = 0;
    assign pos[31] = 0;

    assign pos[1] = ^({pos[3],pos[5],pos[7],pos[9],pos[11],pos[13],pos[15],pos[17],pos[19],pos[21],pos[23],pos[25],pos[27],pos[29],pos[31]});
    assign pos[2] = ^({pos[3],pos[6],pos[7],pos[10],pos[11],pos[14],pos[15],pos[18],pos[19],pos[22],pos[23],pos[26],pos[27],pos[30],pos[31]});
    assign pos[4] = ^({pos[5],pos[6],pos[7],pos[12],pos[13],pos[14],pos[15],pos[20],pos[21],pos[22],pos[23],pos[28],pos[29],pos[30],pos[31]});
    assign pos[8] = ^({pos[9],pos[10],pos[11],pos[12],pos[13],pos[14],pos[15],pos[24],pos[25],pos[26],pos[27],pos[28],pos[29],pos[30],pos[31]});
    assign pos[16] = ^({pos[17],pos[18],pos[19],pos[20],pos[21],pos[22],pos[23],pos[24],pos[25],pos[26],pos[27],pos[28],pos[29],pos[30],pos[31]});

    assign code = pos[31:1];
endmodule

module hamming_decode24 (
    input [30:0] code,
    output [23:0] data,
    output error_corrected
);
    reg [31:1] r;
    reg [31:1] corrected_r;
    reg [4:0] syndrome;
    wire s1, s2, s4, s8, s16;

    always @(*) begin
        r[31:1] = code[30:0];
        s1 = ^({r[1],r[3],r[5],r[7],r[9],r[11],r[13],r[15],r[17],r[19],r[21],r[23],r[25],r[27],r[29],r[31]});
        s2 = ^({r[2],r[3],r[6],r[7],r[10],r[11],r[14],r[15],r[18],r[19],r[22],r[23],r[26],r[27],r[30],r[31]});
        s4 = ^({r[4],r[5],r[6],r[7],r[12],r[13],r[14],r[15],r[20],r[21],r[22],r[23],r[28],r[29],r[30],r[31]});
        s8 = ^({r[8],r[9],r[10],r[11],r[12],r[13],r[14],r[15],r[24],r[25],r[26],r[27],r[28],r[29],r[30],r[31]});
        s16 = ^({r[16],r[17],r[18],r[19],r[20],r[21],r[22],r[23],r[24],r[25],r[26],r[27],r[28],r[29],r[30],r[31]});
        syndrome = {s16, s8, s4, s2, s1};
        corrected_r = r;
        if (syndrome != 0) begin
            corrected_r[syndrome] = ~r[syndrome];
        end
    end

    assign error_corrected = (syndrome != 0);
    assign data[0] = corrected_r[3];
    assign data[1] = corrected_r[5];
    assign data[2] = corrected_r[6];
    assign data[3] = corrected_r[7];
    assign data[4] = corrected_r[9];
    assign data[5] = corrected_r[10];
    assign data[6] = corrected_r[11];
    assign data[7] = corrected_r[12];
    assign data[8] = corrected_r[13];
    assign data[9] = corrected_r[14];
    assign data[10] = corrected_r[15];
    assign data[11] = corrected_r[17];
    assign data[12] = corrected_r[18];
    assign data[13] = corrected_r[19];
    assign data[14] = corrected_r[20];
    assign data[15] = corrected_r[21];
    assign data[16] = corrected_r[22];
    assign data[17] = corrected_r[23];
    assign data[18] = corrected_r[24];
    assign data[19] = corrected_r[25];
    assign data[20] = corrected_r[26];
    assign data[21] = corrected_r[27];
    assign data[22] = corrected_r[28];
    assign data[23] = corrected_r[29];

endmodule
