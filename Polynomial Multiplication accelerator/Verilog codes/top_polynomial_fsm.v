`timescale 1ns / 1ps

module top_polynomial_fsm (
    input clk,
    input rst,
    input int,                       // User-controlled input enable
    input [11:0] in1, in2,           // external inputs
    output [11:0] final_output,
    output done

    // Exposed for monitoring
   // output [11:0] out1, out2, out3, out4, out5, out6, out7, out8,
   // output [11:0] pol_0, pol_1, pol_2, pol_3, pol_4, pol_5, pol_6, pol_7,
  //  output [11:0] ho, he,            // internal feedback inputs
   // output [7:0] counter,
   // output full, half_full,
   // output out, start, active,
   // output sendr,
 //   output send,
 //   output NTT_INTT_sel,
  //  output [11:0] x, y               // final connected inputs
);

    // FSM state encoding
    parameter IDLE         = 4'd0;
    parameter PRE_LOAD     = 4'd1;
    parameter NTT_0        = 4'd2;
    parameter NTT_1        = 4'd3;
    parameter NTT_2        = 4'd4;
    parameter NTT_3        = 4'd5;
    parameter PRE_LOAD_2   = 4'd6;
    parameter INTT_0       = 4'd7;
    parameter INTT_1       = 4'd8;
    parameter DONE         = 4'd9;

    reg [3:0] state, next_state;

    wire [11:0] pwm_output;
    wire [7:0] counter;
    wire send, sendr;
    wire full, half_full;

    reg start;
    reg NTT_INTT_sel;
    reg active;
    reg clear;
    reg intt_mode;
    reg out;
    reg done;

    reg load; // control for assigning x/y

    // Assign x and y inputs dynamically
    assign x = sendr ? he : in1;
    assign y = sendr ? ho : in2;

    // FSM sequential
    always @(posedge clk or posedge rst) begin
        if (rst)
            state <= IDLE;
        else
            state <= next_state;
    end

    // FSM next-state logic
    always @(*) begin
        next_state = state;
        case (state)
            IDLE: next_state = PRE_LOAD;

            PRE_LOAD:
                if (full) next_state = NTT_0;

            NTT_0:
                if (counter == 8'd112) next_state = NTT_1;

            NTT_1:
                if (counter == 8'd112) next_state = NTT_2;

            NTT_2:
                if (counter == 8'd112) next_state = NTT_3;

            NTT_3:
                if (counter == 8'd112 && sendr) next_state = PRE_LOAD_2;

            PRE_LOAD_2:
                if (sendr) next_state = INTT_0;

            INTT_0:
                if (counter == 8'd96) next_state = INTT_1;

            INTT_1:
                if (send) next_state = DONE;

            DONE:
                next_state = DONE;

            default:
                next_state = IDLE;
        endcase
    end

    // Control Signal Logic
    always @(*) begin
        // Default assignments
        start = 0;
        NTT_INTT_sel = 1;
        active = 0;
        intt_mode = 0;
        out = 0;
        load = 0;
        done = 0;

        case (state)
            PRE_LOAD: begin
                load = 1;       // take external in1/in2
            end

            PRE_LOAD_2: begin
                load = 1;       // take internal he/ho
                NTT_INTT_sel = 0;
                clear=1;
            end

            NTT_0, NTT_1, NTT_2, NTT_3: begin
                start = 1;
                NTT_INTT_sel = 1;
                intt_mode = 0;
                out = ((counter >= 8'd113) || (counter <= 8'd15));
                active = ((counter >= 8'd113) || (counter <= 8'd15));
            end

            INTT_0, INTT_1: begin
                start = 1;
                NTT_INTT_sel = 0;
                intt_mode = 1;
                out = ((counter >= 8'd96) || (counter <= 8'd15));
                active = ((counter >= 8'd96) || (counter <= 8'd15));
            end

            DONE: begin
                done = 1;
            end
        endcase
    end

    assign final_output = pwm_output;

    // Instantiate submodules
    pre_processing_unit ppu (
        .clk(clk),
        .rst(rst),
        .int(int),
        .x(x), .y(y),
        .clear(clear),
        .NTT_INTT_sel(NTT_INTT_sel),
        .counter(counter),
        .out(out),
        .out1(out1), .out2(out2), .out3(out3), .out4(out4),
        .out5(out5), .out6(out6), .out7(out7), .out8(out8),
        .full(full), .half_full(half_full)
    );

    NTT_INTT ntt (
        .clk(clk), .rst(rst),
        .start(start),
        .NTT_INTT_sel(NTT_INTT_sel),
        .f_1(out1), .f_2(out2), .f_3(out3), .f_4(out4),
        .f_5(out5), .f_6(out6), .f_7(out7), .f_8(out8),
        .counter(counter),
        .pol_0(pol_0), .pol_1(pol_1), .pol_2(pol_2), .pol_3(pol_3),
        .pol_4(pol_4), .pol_5(pol_5), .pol_6(pol_6), .pol_7(pol_7)
    );

    piso_pwm pwm (
        .clk(clk),
        .active(active),
        .intt(intt_mode),
        .a0(pol_0), .a1(pol_1), .a2(pol_2), .a3(pol_3),
        .a4(pol_4), .a5(pol_5), .a6(pol_6), .a7(pol_7),
        .ho(ho), .he(he),
        .final_output(pwm_output),
        .sendr(sendr), 
        .send(send)
    );

endmodule
