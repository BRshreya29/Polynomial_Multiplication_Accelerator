`timescale 1ns / 1ps

module pre_processing_unit_tb;

    // Inputs
    reg clk;
    reg rst;
    reg int;
    reg [11:0] x, y;
    reg NTT_INTT_sel;
    reg [7:0] counter;
    reg out;

    // Outputs
    wire [11:0] out1, out2, out3, out4, out5, out6, out7, out8;
    wire full;
    wire half_full;
    wire len;
    wire indexout=        pre_processing_unit_tb.uut.index
;

    // Instantiate the Unit Under Test (UUT)
    pre_processing_unit uut (
        .clk(clk),
        .rst(rst),
        .int(int),
        .x(x),
        .y(y),
        .NTT_INTT_sel(NTT_INTT_sel),
        .counter(counter),
        .out(out),
        .out1(out1),
        .out2(out2),
        .out3(out3),
        .out4(out4),
        .out5(out5),
        .out6(out6),
        .out7(out7),
        .out8(out8),
        .full(full),
        .half_full(half_full)
        //.lenout(lenout),
       // .indexout(indexout)
    );

    // Clock generation (10ns period)
    always #5 clk = ~clk;

    integer i;

    initial begin
        // Initialize inputs
        clk = 1;
        rst = 1;
        int = 0;
        x = 0;
        y = 0;
        NTT_INTT_sel = 1;
        counter = 0;
        out = 0;
              
        // Apply reset
        #20;
        rst = 0;

        // Start loading inputs with `int = 1`
        int = 1;
        for (i = 0; i < 256; i = i + 1) begin
            @(posedge clk);
            x = i+1;
            y = 12'd256 + i;
        end
        #10;
        int = 0;

        // Trigger processing
        @(posedge clk);
        out = 1;
        //counter = 8'd112;

        // Wait and observe output
        repeat (10) @(posedge clk);

        $display("=== Output Values ===");
        $display("out1 = %d, out2 = %d, out3 = %d, out4 = %d", out1, out2, out3, out4);
        $display("out5 = %d, out6 = %d, out7 = %d, out8 = %d", out5, out6, out7, out8);
        $display("full = %b, half_full = %b", full, half_full);
        $display("Time: %0t | index = %d | x = %d | y = %d", $time, indexout, x, y);

        #370;
        counter = 8'd112;
        #10;
        counter = 8'd2;
        
        #360;
        
        $finish;
    end

endmodule
