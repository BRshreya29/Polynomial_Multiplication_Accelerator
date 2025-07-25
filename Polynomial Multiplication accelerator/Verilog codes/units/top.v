`timescale 1ns / 1ps

module piso_pwm(
           input clk, active, intt,
           input [11:0] a0, a1, a2, a3, a4, a5, a6, a7, /// 8 parallel inputs
           output [11:0] ho,he, //outputs of PWM
           output [11:0] final_output,/// FINAL OUTPUT after INTT
           output sendr , send  /*SEND for Final Output*/
    );
    
    ///// tau values for PWM//////////////////////////
    initial 
    begin
    taus[0] = 12'd975;
    taus[1] = 12'd444;
    taus[2] = 12'd1964;
    taus[3] = 12'd710;
    taus[4] = 12'd1911;
    taus[5] = 12'd2335;
    taus[6] = 12'd2651;
    taus[7] = 12'd60;
    taus[8] = 12'd1615;
    taus[9] = 12'd3245;
    taus[10] = 12'd1068;
    taus[11] = 12'd2115;
    taus[12] = 12'd1168;
    taus[13] = 12'd368;
    taus[14] = 12'd1028;
    taus[15] = 12'd2148;
    taus[16] = 12'd1224;
    taus[17] = 12'd2985;
    taus[18] = 12'd2947;
    taus[19] = 12'd2479;
    taus[20] = 12'd3198;
    taus[21] = 12'd1190;
    taus[22] = 12'd1515;
    taus[23] = 12'd1663;
    taus[24] = 12'd1208;
    taus[25] = 12'd335;
    taus[26] = 12'd972;
    taus[27] = 12'd2860;
    taus[28] = 12'd2634;
    taus[29] = 12'd2654;
    taus[30] = 12'd973;
    taus[31] = 12'd945;
    taus[32] = 12'd1301;
    taus[33] = 12'd2006;
    taus[34] = 12'd176;
    taus[35] = 12'd2518;
    taus[36] = 12'd1751;
    taus[37] = 12'd2467;
    taus[38] = 12'd2875;
    taus[39] = 12'd541;
    taus[40] = 12'd2633;
    taus[41] = 12'd1240;
    taus[42] = 12'd2306;
    taus[43] = 12'd1593;
    taus[44] = 12'd2098;
    taus[45] = 12'd433;
    taus[46] = 12'd3055;
    taus[47] = 12'd2057;
    taus[48] = 12'd2381;
    taus[49] = 12'd1115;
    taus[50] = 12'd1993;
    taus[51] = 12'd1768;
    taus[52] = 12'd3202;
    taus[53] = 12'd188;
    taus[54] = 12'd2841;
    taus[55] = 12'd2400;
    taus[56] = 12'd1349;
    taus[57] = 12'd3298;
    taus[58] = 12'd2772;
    taus[59] = 12'd1375;
    taus[60] = 12'd114;
    taus[61] = 12'd1404;
    taus[62] = 12'd1172;
    taus[63] = 12'd2695;
    taus[64] = 12'd2354;
    taus[65] = 12'd2885;
    taus[66] = 12'd1365;
    taus[67] = 12'd2619;
    taus[68] = 12'd1418;
    taus[69] = 12'd994;
    taus[70] = 12'd678;
    taus[71] = 12'd3269;
    taus[72] = 12'd1714;
    taus[73] = 12'd84;
    taus[74] = 12'd2261;
    taus[75] = 12'd1214;
    taus[76] = 12'd2161;
    taus[77] = 12'd2961;
    taus[78] = 12'd2301;
    taus[79] = 12'd1181;
    taus[80] = 12'd2105;
    taus[81] = 12'd344;
    taus[82] = 12'd382;
    taus[83] = 12'd850;
    taus[84] = 12'd131;
    taus[85] = 12'd2139;
    taus[86] = 12'd1814;
    taus[87] = 12'd1666;
    taus[88] = 12'd2121;
    taus[89] = 12'd2994;
    taus[90] = 12'd2357;
    taus[91] = 12'd469;
    taus[92] = 12'd695;
    taus[93] = 12'd675;
    taus[94] = 12'd2356;
    taus[95] = 12'd2384;
    taus[96] = 12'd2028;
    taus[97] = 12'd1323;
    taus[98] = 12'd3153;
    taus[99] = 12'd811;
    taus[100] = 12'd1578;
    taus[101] = 12'd862;
    taus[102] = 12'd454;
    taus[103] = 12'd2788;
    taus[104] = 12'd696;
    taus[105] = 12'd2089;
    taus[106] = 12'd1023;
    taus[107] = 12'd1736;
    taus[108] = 12'd1231;
    taus[109] = 12'd2896;
    taus[110] = 12'd274;
    taus[111] = 12'd1272;
    taus[112] = 12'd948;
    taus[113] = 12'd2214;
    taus[114] = 12'd1336;
    taus[115] = 12'd1561;
    taus[116] = 12'd127;
    taus[117] = 12'd3141;
    taus[118] = 12'd488;
    taus[119] = 12'd929;
    taus[120] = 12'd1980;
    taus[121] = 12'd31;
    taus[122] = 12'd557;
    taus[123] = 12'd1954;
    taus[124] = 12'd3215;
    taus[125] = 12'd1925;
    taus[126] = 12'd2157;
    taus[127] = 12'd634;
    
    end
    
    always@(posedge clk)
    begin
    if(dir==0)
    begin
        tau=taus[tcount];
        tcount=tcount+1;
    end
    end
 ////////////////////////////////////////////////////////////////////////////////////////////   
    
  endmodule



































/////////////////////////////////// mem block module//////////////////////////////////////////////////////////////////////////


module mem(
    input clk, active, dir,
    input [11:0] a0, a1, a2, a3, a4, a5, a6, a7,
    output reg [11:0] out,
    output reg sent,
    output reg full,
    input rst
);

    // 8 Memory blocks of 16x12-bit
    reg [11:0] memo1 [15:0];
    reg [11:0] memo2 [15:0];
    reg [11:0] memo3 [15:0];
    reg [11:0] memo4 [15:0];
    reg [11:0] memo5 [15:0];
    reg [11:0] memo6 [15:0];
    reg [11:0] memo7 [15:0];
    reg [11:0] memo8 [15:0];

    reg [3:0] count = 0; // 4-bit counter for inputs (0 to 15)
    reg [3:0] ncount = 0; // 4-bit counter for output
    reg [2:0] main = 0; //// for output
        
    initial begin 
    sent=1;
    end
    
    
   
    ///////////// RESET mode
    always@(negedge clk)
    begin
    if(rst)
        begin
            full=0;
            count=0;
            ncount=4'hf;
            main=3'b111;
            sent=0;
        end
    end
    
//     Storing Data (Write Mode)
    always @(posedge clk) 
    begin
        if(dir==1 && active==1 && full==0) 
            begin
                if (count == 4'hf) 
                    full=1;
                 else 
                    full=0;
                memo1[count] = a0;
                memo2[count] = a1;
                memo3[count] = a2;
                memo4[count] = a3;
                memo5[count] = a4;
                memo6[count] = a5;
                memo7[count] = a6;
                memo8[count] = a7;
                count = count + 1;
            end
            
/////////////// OUTPUT mode/////////////////            
            if(dir==0 && sent==0)

             begin
                if(main==3'b111) ncount=ncount+1;
                main=main+1;
                
                case(main)
                    3'b000: out = memo1[ncount];
                    3'b001: out = memo2[ncount];
                    3'b010: out = memo3[ncount];
                    3'b011: out = memo4[ncount];
                    3'b100: out = memo5[ncount];
                    3'b101: out = memo6[ncount];
                    3'b110: out = memo7[ncount];
                    3'b111: out = memo8[ncount];
                endcase
                if(main==3'b111 && ncount==4'hf) 
                    sent=1;
                else 
                    sent=0;
              end
    end


endmodule 