`timescale 1ns/1ps

module top_polynomial_fsm_tb;

    reg clk, rst;
    reg [11:0] in1 , in2;
    reg int;
    wire [11:0] final_output;

    // Output wires to monitor
    wire full, half_full, out, start;
    wire [7:0] counter;
    wire [11:0] out1, out2, out3, out4, out5, out6, out7, out8;
    wire [11:0] pol_0, pol_1, pol_2, pol_3, pol_4, pol_5, pol_6, pol_7;
    wire active;
    wire [11:0] he,ho;
    wire sendr, send;
    wire NTT_INTT_sel;
    wire done;
    wire [11:0] x,y;

    // Instantiate DUT
    top_polynomial_fsm dut (
        .clk(clk),
        .rst(rst),
        .in1(in1),
        .in2(in2),
        .int(int),
        .final_output(final_output),
        .done(done)
        /*.out1(out1), .out2(out2), .out3(out3), .out4(out4),         //for testing commentout
        .out5(out5), .out6(out6), .out7(out7), .out8(out8),
        .pol_0(pol_0), .pol_1(pol_1), .pol_2(pol_2), .pol_3(pol_3),
        .pol_4(pol_4), .pol_5(pol_5), .pol_6(pol_6), .pol_7(pol_7),
        .he(he), .ho(ho),
        .full(full), .half_full(half_full),
        .out(out), .start(start), .active(active), .counter(counter),
        .sendr(sendr), .send(send), .NTT_INTT_sel(NTT_INTT_sel), .done(done),
        .x(x), .y(y) */
    );


    // Clock generation (100 MHz)
    initial clk = 1;
    always #5 clk = ~clk;

    integer i;

    initial begin

        // Initialization
        rst = 1;
        int = 0;
        in1 = 12'd0;
        in2 = 12'd0;

        #20;
        rst = 0;

        // Input phase
        int = 1;

        // Load x = 1..256, y = 257..512
        for (i = 0; i < 256; i = i + 1) begin
            @(posedge clk);
            in1 = i + 1;
            in2 = i + 257;
        end

        // Stop input
        @(posedge clk);
        int = 0;

        // Wait until done
        wait (done == 1);
        $display("Computation done. Final Output = %d", final_output);

        #100;
        $finish;
    end

endmodule
