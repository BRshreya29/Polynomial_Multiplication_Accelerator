`timescale 1ns / 1ps

module NTT_INTT(
    input [11:0] f_1, f_2, f_3, f_4, f_5, f_6, f_7, f_8,
    input clk,
    input rst, start, NTT_INTT_sel, //NTT_INTT_sel, NTT = 1, iNTT = 0
    output [7:0] counter,
    output [11:0] pol_0, pol_1, pol_2, pol_3,
                  pol_4, pol_5, pol_6, pol_7
    );
    
       
endmodule