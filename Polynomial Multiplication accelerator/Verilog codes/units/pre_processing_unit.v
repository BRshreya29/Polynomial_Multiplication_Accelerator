`timescale 1ns / 1ps
module pre_processing_unit(
    input clk,
    input rst,
    input int,
    input [11:0] x, y,
    input clear,
    input NTT_INTT_sel,
    input [7:0] counter,
    input out,
    output reg [11:0] out1, out2, out3, out4, out5, out6, out7, out8, // Outputs
    output reg full,       // High when f and g are full
    output reg half_full   // High when f and g are at least half full
    //output reg [7:0] indexout
    );
    
    reg [11:0] f [255:0]; 
    reg [11:0] g [255:0];
    reg [7:0] index, len; // Index for writing into f and g

    wire [11:0] f_even [127:0], f_odd [127:0];
    wire [11:0] g_even [127:0], g_odd [127:0]; 
    wire [11:0] ho [127:0], he [127:0];
    wire [11:0] a [127:0], b [127:0], c [127:0], d [127:0], e [127:0], k [127:0];

always @(posedge clk or posedge rst) begin
    if (rst || clear) begin
        index <= 0;
        full <= 0;
        half_full <= 0;
    end else if (int && !full) begin
        f[index] <= x;
        g[index] <= y;
        index <= index + 1;

        if (index == 127 && NTT_INTT_sel == 0)
            half_full <= 1;

        if (index == 254)
            full <= 1;
    end
end


    
    /*
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            index <= 0;
            //len <= 0;
            full <= 0;
            half_full <= 0;
        end   
        else if (int) begin  // Load data only if not full - && !full
            f[index] <= x;
            g[index] <= y;
            index <= index + 1;
            if (index >= 127 && NTT_INTT_sel==0) begin
                half_full <= 1;  // Set half_full when at least half capacity is reached
            end
            if (index == 254) begin
                full <= 1;  // Set full when registers are completely filled
            end
        end
    end */
    
        
    // Divide f and g into f_even, f_odd, g_even, g_odd when full is high4
    genvar i;
    generate
        for (i = 0; i < 128; i = i + 1) begin
            assign f_even[i] =  f[2 * i] ;
            assign f_odd[i]  =  f[2 * i + 1] ;
            assign g_even[i] =  g[2 * i] ;
            assign g_odd[i]  =  g[2 * i + 1] ;
        end
    endgenerate
    
    // Assign f to e and g to k when half_full is high
    generate
        for (i = 0; i < 128; i = i + 1) begin
            assign ho[i] =  f[i] ;
            assign he[i] =  g[i] ;
        end
    endgenerate
    
    //////// a assignment /////////
       assign a[0] = f_even[0], a[1] = f_even[64], a[2] = f_even[32], a[3] = f_even[96], 
       a[4] = f_even[1], a[5] = f_even[65], a[6] = f_even[33], a[7] = f_even[97];
        
       assign a[8] = f_even[16], a[9] = f_even[80], a[10] = f_even[48], a[11] = f_even[112], 
       a[12] = f_even[17], a[13] = f_even[81], a[14] = f_even[49], a[15] = f_even[113];

       assign a[16] = f_even[8], a[17] = f_even[72], a[18] = f_even[40], a[19] = f_even[104], 
       a[20] = f_even[9], a[21] = f_even[73], a[22] = f_even[41], a[23] = f_even[105];

       assign a[24] = f_even[24], a[25] = f_even[88], a[26] = f_even[56], a[27] = f_even[120], 
       a[28] = f_even[25], a[29] = f_even[89], a[30] = f_even[57], a[31] = f_even[121];

       assign a[32] = f_even[4], a[33] = f_even[68], a[34] = f_even[36], a[35] = f_even[100], 
       a[36] = f_even[5], a[37] = f_even[69], a[38] = f_even[37], a[39] = f_even[101];

       assign a[40] = f_even[20], a[41] = f_even[84], a[42] = f_even[52], a[43] = f_even[116], 
       a[44] = f_even[21], a[45] = f_even[85], a[46] = f_even[53], a[47] = f_even[117];

       assign a[48] = f_even[12], a[49] = f_even[76], a[50] = f_even[44], a[51] = f_even[108], 
       a[52] = f_even[13], a[53] = f_even[77], a[54] = f_even[45], a[55] = f_even[109];

       assign a[56] = f_even[28], a[57] = f_even[92], a[58] = f_even[60], a[59] = f_even[124], 
       a[60] = f_even[29], a[61] = f_even[93], a[62] = f_even[61], a[63] = f_even[125];

       assign a[64] = f_even[2], a[65] = f_even[66], a[66] = f_even[34], a[67] = f_even[98], 
       a[68] = f_even[3], a[69] = f_even[67], a[70] = f_even[35], a[71] = f_even[99];

       assign a[72] = f_even[18], a[73] = f_even[82], a[74] = f_even[50], a[75] = f_even[114], 
       a[76] = f_even[19], a[77] = f_even[83], a[78] = f_even[51], a[79] = f_even[115];

       assign a[80] = f_even[10], a[81] = f_even[74], a[82] = f_even[42], a[83] = f_even[106], 
       a[84] = f_even[11], a[85] = f_even[75], a[86] = f_even[43], a[87] = f_even[107];

       assign a[88] = f_even[26], a[89] = f_even[90], a[90] = f_even[58], a[91] = f_even[122], 
       a[92] = f_even[27], a[93] = f_even[91], a[94] = f_even[59], a[95] = f_even[123];

       assign a[96] = f_even[6], a[97] = f_even[70], a[98] = f_even[38], a[99] = f_even[102], 
       a[100] = f_even[7], a[101] = f_even[71], a[102] = f_even[39], a[103] = f_even[103];

       assign a[104] = f_even[22], a[105] = f_even[86], a[106] = f_even[54], a[107] = f_even[118], 
       a[108] = f_even[23], a[109] = f_even[87], a[110] = f_even[55], a[111] = f_even[119];

       assign a[112] = f_even[14], a[113] = f_even[78], a[114] = f_even[46], a[115] = f_even[110], 
       a[116] = f_even[15], a[117] = f_even[79], a[118] = f_even[47], a[119] = f_even[111];

       assign a[120] = f_even[30], a[121] = f_even[94], a[122] = f_even[62], a[123] = f_even[126], 
       a[124] = f_even[31], a[125] = f_even[95], a[126] = f_even[63], a[127] = f_even[127];

       //////// b assignment /////////
       assign b[0] = f_odd[0], b[1] = f_odd[64], b[2] = f_odd[32], b[3] = f_odd[96], 
       b[4] = f_odd[1], b[5] = f_odd[65], b[6] = f_odd[33], b[7] = f_odd[97];

       assign b[8] = f_odd[16], b[9] = f_odd[80], b[10] = f_odd[48], b[11] = f_odd[112], 
       b[12] = f_odd[17], b[13] = f_odd[81], b[14] = f_odd[49], b[15] = f_odd[113];

       assign b[16] = f_odd[8], b[17] = f_odd[72], b[18] = f_odd[40], b[19] = f_odd[104], 
       b[20] = f_odd[9], b[21] = f_odd[73], b[22] = f_odd[41], b[23] = f_odd[105];

       assign b[24] = f_odd[24], b[25] = f_odd[88], b[26] = f_odd[56], b[27] = f_odd[120], 
       b[28] = f_odd[25], b[29] = f_odd[89], b[30] = f_odd[57], b[31] = f_odd[121];

       assign b[32] = f_odd[4], b[33] = f_odd[68], b[34] = f_odd[36], b[35] = f_odd[100], 
       b[36] = f_odd[5], b[37] = f_odd[69], b[38] = f_odd[37], b[39] = f_odd[101];

       assign b[40] = f_odd[20], b[41] = f_odd[84], b[42] = f_odd[52], b[43] = f_odd[116], 
       b[44] = f_odd[21], b[45] = f_odd[85], b[46] = f_odd[53], b[47] = f_odd[117];

       assign b[48] = f_odd[12], b[49] = f_odd[76], b[50] = f_odd[44], b[51] = f_odd[108], 
       b[52] = f_odd[13], b[53] = f_odd[77], b[54] = f_odd[45], b[55] = f_odd[109];

       assign b[56] = f_odd[28], b[57] = f_odd[92], b[58] = f_odd[60], b[59] = f_odd[124], 
       b[60] = f_odd[29], b[61] = f_odd[93], b[62] = f_odd[61], b[63] = f_odd[125];

       assign b[64] = f_odd[2], b[65] = f_odd[66], b[66] = f_odd[34], b[67] = f_odd[98], 
       b[68] = f_odd[3], b[69] = f_odd[67], b[70] = f_odd[35], b[71] = f_odd[99];

       assign b[72] = f_odd[18], b[73] = f_odd[82], b[74] = f_odd[50], b[75] = f_odd[114], 
       b[76] = f_odd[19], b[77] = f_odd[83], b[78] = f_odd[51], b[79] = f_odd[115];

       assign b[80] = f_odd[10], b[81] = f_odd[74], b[82] = f_odd[42], b[83] = f_odd[106], 
       b[84] = f_odd[11], b[85] = f_odd[75], b[86] = f_odd[43], b[87] = f_odd[107];

       assign b[88] = f_odd[26], b[89] = f_odd[90], b[90] = f_odd[58], b[91] = f_odd[122], 
       b[92] = f_odd[27], b[93] = f_odd[91], b[94] = f_odd[59], b[95] = f_odd[123];

       assign b[96] = f_odd[6], b[97] = f_odd[70], b[98] = f_odd[38], b[99] = f_odd[102], 
       b[100] = f_odd[7], b[101] = f_odd[71], b[102] = f_odd[39], b[103] = f_odd[103];

       assign b[104] = f_odd[22], b[105] = f_odd[86], b[106] = f_odd[54], b[107] = f_odd[118], 
       b[108] = f_odd[23], b[109] = f_odd[87], b[110] = f_odd[55], b[111] = f_odd[119];

       assign b[112] = f_odd[14], b[113] = f_odd[78], b[114] = f_odd[46], b[115] = f_odd[110], 
       b[116] = f_odd[15], b[117] = f_odd[79], b[118] = f_odd[47], b[119] = f_odd[111];

       assign b[120] = f_odd[30], b[121] = f_odd[94], b[122] = f_odd[62], b[123] = f_odd[126], 
       b[124] = f_odd[31], b[125] = f_odd[95], b[126] = f_odd[63], b[127] = f_odd[127];

       //////// c assignment /////////
       
       assign c[0] = g_even[0], c[1] = g_even[64], c[2] = g_even[32], c[3] = g_even[96], 
       c[4] = g_even[1], c[5] = g_even[65], c[6] = g_even[33], c[7] = g_even[97];

       assign c[8] = g_even[16], c[9] = g_even[80], c[10] = g_even[48], c[11] = g_even[112], 
       c[12] = g_even[17], c[13] = g_even[81], c[14] = g_even[49], c[15] = g_even[113];

       assign c[16] = g_even[8], c[17] = g_even[72], c[18] = g_even[40], c[19] = g_even[104], 
       c[20] = g_even[9], c[21] = g_even[73], c[22] = g_even[41], c[23] = g_even[105];

       assign c[24] = g_even[24], c[25] = g_even[88], c[26] = g_even[56], c[27] = g_even[120], 
       c[28] = g_even[25], c[29] = g_even[89], c[30] = g_even[57], c[31] = g_even[121];

       assign c[32] = g_even[4], c[33] = g_even[68], c[34] = g_even[36], c[35] = g_even[100], 
       c[36] = g_even[5], c[37] = g_even[69], c[38] = g_even[37], c[39] = g_even[101];

       assign c[40] = g_even[20], c[41] = g_even[84], c[42] = g_even[52], c[43] = g_even[116], 
       c[44] = g_even[21], c[45] = g_even[85], c[46] = g_even[53], c[47] = g_even[117];

       assign c[48] = g_even[12], c[49] = g_even[76], c[50] = g_even[44], c[51] = g_even[108], 
       c[52] = g_even[13], c[53] = g_even[77], c[54] = g_even[45], c[55] = g_even[109];

       assign c[56] = g_even[28], c[57] = g_even[92], c[58] = g_even[60], c[59] = g_even[124], 
       c[60] = g_even[29], c[61] = g_even[93], c[62] = g_even[61], c[63] = g_even[125];

       assign c[64] = g_even[2], c[65] = g_even[66], c[66] = g_even[34], c[67] = g_even[98], 
       c[68] = g_even[3], c[69] = g_even[67], c[70] = g_even[35], c[71] = g_even[99];

       assign c[72] = g_even[18], c[73] = g_even[82], c[74] = g_even[50], c[75] = g_even[114], 
       c[76] = g_even[19], c[77] = g_even[83], c[78] = g_even[51], c[79] = g_even[115];

       assign c[80] = g_even[10], c[81] = g_even[74], c[82] = g_even[42], c[83] = g_even[106], 
       c[84] = g_even[11], c[85] = g_even[75], c[86] = g_even[43], c[87] = g_even[107];

       assign c[88] = g_even[26], c[89] = g_even[90], c[90] = g_even[58], c[91] = g_even[122], 
       c[92] = g_even[27], c[93] = g_even[91], c[94] = g_even[59], c[95] = g_even[123];

       assign c[96] = g_even[6], c[97] = g_even[70], c[98] = g_even[38], c[99] = g_even[102], 
       c[100] = g_even[7], c[101] = g_even[71], c[102] = g_even[39], c[103] = g_even[103];

       assign c[104] = g_even[22], c[105] = g_even[86], c[106] = g_even[54], c[107] = g_even[118], 
       c[108] = g_even[23], c[109] = g_even[87], c[110] = g_even[55], c[111] = g_even[119];

       assign c[112] = g_even[14], c[113] = g_even[78], c[114] = g_even[46], c[115] = g_even[110], 
       c[116] = g_even[15], c[117] = g_even[79], c[118] = g_even[47], c[119] = g_even[111];

       assign c[120] = g_even[30], c[121] = g_even[94], c[122] = g_even[62], c[123] = g_even[126], 
       c[124] = g_even[31], c[125] = g_even[95], c[126] = g_even[63], c[127] = g_even[127];
       
       //////// d assignment /////////

       assign d[0] = g_odd[0], d[1] = g_odd[64], d[2] = g_odd[32], d[3] = g_odd[96], 
       d[4] = g_odd[1], d[5] = g_odd[65], d[6] = g_odd[33], d[7] = g_odd[97];

       assign d[8] = g_odd[16], d[9] = g_odd[80], d[10] = g_odd[48], d[11] = g_odd[112], 
       d[12] = g_odd[17], d[13] = g_odd[81], d[14] = g_odd[49], d[15] = g_odd[113];

       assign d[16] = g_odd[8], d[17] = g_odd[72], d[18] = g_odd[40], d[19] = g_odd[104], 
       d[20] = g_odd[9], d[21] = g_odd[73], d[22] = g_odd[41], d[23] = g_odd[105];

       assign d[24] = g_odd[24], d[25] = g_odd[88], d[26] = g_odd[56], d[27] = g_odd[120], 
       d[28] = g_odd[25], d[29] = g_odd[89], d[30] = g_odd[57], d[31] = g_odd[121];

       assign d[32] = g_odd[4], d[33] = g_odd[68], d[34] = g_odd[36], d[35] = g_odd[100], 
       d[36] = g_odd[5], d[37] = g_odd[69], d[38] = g_odd[37], d[39] = g_odd[101];

       assign d[40] = g_odd[20], d[41] = g_odd[84], d[42] = g_odd[52], d[43] = g_odd[116], 
       d[44] = g_odd[21], d[45] = g_odd[85], d[46] = g_odd[53], d[47] = g_odd[117];

       assign d[48] = g_odd[12], d[49] = g_odd[76], d[50] = g_odd[44], d[51] = g_odd[108], 
       d[52] = g_odd[13], d[53] = g_odd[77], d[54] = g_odd[45], d[55] = g_odd[109];

       assign d[56] = g_odd[28], d[57] = g_odd[92], d[58] = g_odd[60], d[59] = g_odd[124], 
       d[60] = g_odd[29], d[61] = g_odd[93], d[62] = g_odd[61], d[63] = g_odd[125];

       assign d[64] = g_odd[2], d[65] = g_odd[66], d[66] = g_odd[34], d[67] = g_odd[98], 
       d[68] = g_odd[3], d[69] = g_odd[67], d[70] = g_odd[35], d[71] = g_odd[99];

       assign d[72] = g_odd[18], d[73] = g_odd[82], d[74] = g_odd[50], d[75] = g_odd[114], 
       d[76] = g_odd[19], d[77] = g_odd[83], d[78] = g_odd[51], d[79] = g_odd[115];

       assign d[80] = g_odd[10], d[81] = g_odd[74], d[82] = g_odd[42], d[83] = g_odd[106], 
       d[84] = g_odd[11], d[85] = g_odd[75], d[86] = g_odd[43], d[87] = g_odd[107];

       assign d[88] = g_odd[26], d[89] = g_odd[90], d[90] = g_odd[58], d[91] = g_odd[122], 
       d[92] = g_odd[27], d[93] = g_odd[91], d[94] = g_odd[59], d[95] = g_odd[123];

       assign d[96] = g_odd[6], d[97] = g_odd[70], d[98] = g_odd[38], d[99] = g_odd[102], 
       d[100] = g_odd[7], d[101] = g_odd[71], d[102] = g_odd[39], d[103] = g_odd[103];

       assign d[104] = g_odd[22], d[105] = g_odd[86], d[106] = g_odd[54], d[107] = g_odd[118], 
       d[108] = g_odd[23], d[109] = g_odd[87], d[110] = g_odd[55], d[111] = g_odd[119];

       assign d[112] = g_odd[14], d[113] = g_odd[78], d[114] = g_odd[46], d[115] = g_odd[110], 
       d[116] = g_odd[15], d[117] = g_odd[79], d[118] = g_odd[47], d[119] = g_odd[111];

       assign d[120] = g_odd[30], d[121] = g_odd[94], d[122] = g_odd[62], d[123] = g_odd[126], 
       d[124] = g_odd[31], d[125] = g_odd[95], d[126] = g_odd[63], d[127] = g_odd[127];
  
       //////// e assignment /////////   
       assign e[0] = ho[0], e[1] = ho[64], e[2] = ho[32], e[3] = ho[96], 
       e[4] = ho[1], e[5] = ho[65], e[6] = ho[33], e[7] = ho[97];

       assign e[8] = ho[8], e[9] = ho[72], e[10] = ho[40], e[11] = ho[104], 
       e[12] = ho[9], e[13] = ho[73], e[14] = ho[41], e[15] = ho[105];

       assign e[16] = ho[16], e[17] = ho[80], e[18] = ho[48], e[19] = ho[112], 
       e[20] = ho[17], e[21] = ho[81], e[22] = ho[49], e[23] = ho[113];

       assign e[24] = ho[24], e[25] = ho[88], e[26] = ho[56], e[27] = ho[120], 
       e[28] = ho[25], e[29] = ho[89], e[30] = ho[57], e[31] = ho[121];

       assign e[32] = ho[4], e[33] = ho[68], e[34] = ho[36], e[35] = ho[100], 
       e[36] = ho[5], e[37] = ho[69], e[38] = ho[37], e[39] = ho[101];

       assign e[40] = ho[12], e[41] = ho[76], e[42] = ho[44], e[43] = ho[108], 
       e[44] = ho[13], e[45] = ho[77], e[46] = ho[45], e[47] = ho[109];

       assign e[48] = ho[20], e[49] = ho[84], e[50] = ho[52], e[51] = ho[116], 
       e[52] = ho[21], e[53] = ho[85], e[54] = ho[53], e[55] = ho[117];

       assign e[56] = ho[28], e[57] = ho[92], e[58] = ho[60], e[59] = ho[124], 
       e[60] = ho[29], e[61] = ho[93], e[62] = ho[61], e[63] = ho[125];

       assign e[64] = ho[2], e[65] = ho[66], e[66] = ho[34], e[67] = ho[98], 
       e[68] = ho[3], e[69] = ho[67], e[70] = ho[35], e[71] = ho[99];

       assign e[72] = ho[10], e[73] = ho[74], e[74] = ho[42], e[75] = ho[106], 
       e[76] = ho[11], e[77] = ho[75], e[78] = ho[43], e[79] = ho[107];

       assign e[80] = ho[18], e[81] = ho[82], e[82] = ho[50], e[83] = ho[114], 
       e[84] = ho[19], e[85] = ho[83], e[86] = ho[51], e[87] = ho[115];

       assign e[88] = ho[26], e[89] = ho[90], e[90] = ho[58], e[91] = ho[122], 
       e[92] = ho[27], e[93] = ho[91], e[94] = ho[59], e[95] = ho[123];

       assign e[96] = ho[6], e[97] = ho[70], e[98] = ho[38], e[99] = ho[102], 
       e[100] = ho[7], e[101] = ho[71], e[102] = ho[39], e[103] = ho[103];

       assign e[104] = ho[14], e[105] = ho[78], e[106] = ho[46], e[107] = ho[110], 
       e[108] = ho[15], e[109] = ho[79], e[110] = ho[47], e[111] = ho[111];

       assign e[112] = ho[22], e[113] = ho[86], e[114] = ho[54], e[115] = ho[118], 
       e[116] = ho[23], e[117] = ho[87], e[118] = ho[55], e[119] = ho[119];

       assign e[120] = ho[30], e[121] = ho[94], e[122] = ho[62], e[123] = ho[126], 
       e[124] = ho[31], e[125] = ho[95], e[126] = ho[63], e[127] = ho[127];


       //////// k assignment /////////
       assign k[0] = he[0], k[1] = he[64], k[2] = he[32], k[3] = he[96], 
       k[4] = he[1], k[5] = he[65], k[6] = he[33], k[7] = he[97];

       assign k[8] = he[8], k[9] = he[72], k[10] = he[40], k[11] = he[104], 
       k[12] = he[9], k[13] = he[73], k[14] = he[41], k[15] = he[105];

       assign k[16] = he[16], k[17] = he[80], k[18] = he[48], k[19] = he[112], 
       k[20] = he[17], k[21] = he[81], k[22] = he[49], k[23] = he[113];

       assign k[24] = he[24], k[25] = he[88], k[26] = he[56], k[27] = he[120], 
       k[28] = he[25], k[29] = he[89], k[30] = he[57], k[31] = he[121];

       assign k[32] = he[4], k[33] = he[68], k[34] = he[36], k[35] = he[100], 
       k[36] = he[5], k[37] = he[69], k[38] = he[37], k[39] = he[101];

       assign k[40] = he[12], k[41] = he[76], k[42] = he[44], k[43] = he[108], 
       k[44] = he[13], k[45] = he[77], k[46] = he[45], k[47] = he[109];

       assign k[48] = he[20], k[49] = he[84], k[50] = he[52], k[51] = he[116], 
       k[52] = he[21], k[53] = he[85], k[54] = he[53], k[55] = he[117];

       assign k[56] = he[28], k[57] = he[92], k[58] = he[60], k[59] = he[124], 
       k[60] = he[29], k[61] = he[93], k[62] = he[61], k[63] = he[125];

       assign k[64] = he[2], k[65] = he[66], k[66] = he[34], k[67] = he[98], 
       k[68] = he[3], k[69] = he[67], k[70] = he[35], k[71] = he[99];

       assign k[72] = he[10], k[73] = he[74], k[74] = he[42], k[75] = he[106], 
       k[76] = he[11], k[77] = he[75], k[78] = he[43], k[79] = he[107];

       assign k[80] = he[18], k[81] = he[82], k[82] = he[50], k[83] = he[114], 
       k[84] = he[19], k[85] = he[83], k[86] = he[51], k[87] = he[115];

       assign k[88] = he[26], k[89] = he[90], k[90] = he[58], k[91] = he[122], 
       k[92] = he[27], k[93] = he[91], k[94] = he[59], k[95] = he[123];

       assign k[96] = he[6], k[97] = he[70], k[98] = he[38], k[99] = he[102], 
       k[100] = he[7], k[101] = he[71], k[102] = he[39], k[103] = he[103];

       assign k[104] = he[14], k[105] = he[78], k[106] = he[46], k[107] = he[110], 
       k[108] = he[15], k[109] = he[79], k[110] = he[47], k[111] = he[111];

       assign k[112] = he[22], k[113] = he[86], k[114] = he[54], k[115] = he[118], 
       k[116] = he[23], k[117] = he[87], k[118] = he[55], k[119] = he[119];

       assign k[120] = he[30], k[121] = he[94], k[122] = he[62], k[123] = he[126], 
       k[124] = he[31], k[125] = he[95], k[126] = he[63], k[127] = he[127];
       
       ////////////////////// logic to shift select //////////////////////////
      reg [1:0] prev_select, select;
      reg iselect;
      
      // Sequential logic to update select based on counter
        always @(posedge clk or posedge rst) begin
            if (rst) begin
                select <= 2'b00; // Reset select to 00
            end else begin
                if (counter == 8'd0) begin
                    select <= 2'b00; // Set select to 00 when counter is 0
                end else if (counter == 8'd110) begin
                    // Increment select every time counter reaches 112
                    case (select)
                        2'b00: select <= 2'b01;
                        2'b01: select <= 2'b10;
                        2'b10: select <= 2'b11;
                        2'b11: select <= 2'b00; // Wrap around to 00
                        default: select <= 2'b00;
                    endcase
                end
            end
        end
        
      ////////////////////// logic to shift iselect //////////////////////////

        // Sequential logic to update iselect based on counter
        always @(posedge clk or posedge rst) begin
            if (rst) begin
                iselect <= 1'b0; // Default is 0
            end else begin
                if (counter == 8'd101) begin ///check this counter value
                    iselect <= 1'b1; // Change iselect to 1 when counter is 101
                end else begin
                    iselect <= 1'b0; // Default remains 0
                end
            end
        end
        

always @(posedge clk or posedge rst) begin
    if (rst) begin
        len <= 0;
        prev_select <= 2'b00;
    end else begin
        if (select != prev_select) begin
            len <= 0;  // Reset len if select changed
        end else if (out && len < 120) begin
            len <= len + 8;  // Output next group of 8 values
        end else if (len >= 120) begin
            len <= 0;  // Optional: reset len when done
        end
        prev_select <= select;  // Update previous select
    end
end


always @(posedge clk) begin
    if (out) begin
        case (select) 
            2'b00: begin 
                if (NTT_INTT_sel == 1) begin 
                    // When NTT_INTT_sel = 0, select from a[]
                    out1 <= a[len]; out2 <= a[len+1]; out3 <= a[len+2]; out4 <= a[len+3];
                    out5 <= a[len+4]; out6 <= a[len+5]; out7 <= a[len+6]; out8 <= a[len+7];
                end else begin
                    // When NTT_INTT_sel = 1, select from k[]
                   out1 <= k[len]; out2 <= k[len+1]; out3 <= k[len+2]; out4 <= k[len+3];
                   out5 <= k[len+4]; out6 <= k[len+5]; out7 <= k[len+6]; out8 <= k[len+7];
                end
            end
            2'b01: begin 
                if (NTT_INTT_sel == 1) begin 
                    // When NTT_INTT_sel = 0, select from b[]
                    out1 <= b[len]; out2 <= b[len+1]; out3 <= b[len+2]; out4 <= b[len+3];
                    out5 <= b[len+4]; out6 <= b[len+5]; out7 <= b[len+6]; out8 <= b[len+7];
                end else begin
                    // When NTT_INTT_sel = 1, select from e[]
                    out1 <= e[len]; out2 <= e[len+1]; out3 <= e[len+2]; out4 <= e[len+3];
                   out5 <= e[len+4]; out6 <= e[len+5]; out7 <= e[len+6]; out8 <= e[len+7];
                end
            end
            2'b10: begin // Only active when NTT_INTT_sel == 0
                out1 <= c[len]; out2 <= c[len+1]; out3 <= c[len+2]; out4 <= c[len+3];
                out5 <= c[len+4]; out6 <= c[len+5]; out7 <= c[len+6]; out8 <= c[len+7];
            end
            2'b11: begin // Only active when NTT_INTT_sel == 0
                out1 <= d[len]; out2 <= d[len+1]; out3 <= d[len+2]; out4 <= d[len+3];
                out5 <= d[len+4]; out6 <= d[len+5]; out7 <= d[len+6]; out8 <= d[len+7];
            end
            //default: begin
               // out1 <= 0; out2 <= 0; out3 <= 0; out4 <= 0;
              //  out5 <= 0; out6 <= 0; out7 <= 0; out8 <= 0;
            //end
        endcase
    end
end
    
endmodule
