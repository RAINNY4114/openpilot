#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6167525615574876253) {
   out_6167525615574876253[0] = delta_x[0] + nom_x[0];
   out_6167525615574876253[1] = delta_x[1] + nom_x[1];
   out_6167525615574876253[2] = delta_x[2] + nom_x[2];
   out_6167525615574876253[3] = delta_x[3] + nom_x[3];
   out_6167525615574876253[4] = delta_x[4] + nom_x[4];
   out_6167525615574876253[5] = delta_x[5] + nom_x[5];
   out_6167525615574876253[6] = delta_x[6] + nom_x[6];
   out_6167525615574876253[7] = delta_x[7] + nom_x[7];
   out_6167525615574876253[8] = delta_x[8] + nom_x[8];
   out_6167525615574876253[9] = delta_x[9] + nom_x[9];
   out_6167525615574876253[10] = delta_x[10] + nom_x[10];
   out_6167525615574876253[11] = delta_x[11] + nom_x[11];
   out_6167525615574876253[12] = delta_x[12] + nom_x[12];
   out_6167525615574876253[13] = delta_x[13] + nom_x[13];
   out_6167525615574876253[14] = delta_x[14] + nom_x[14];
   out_6167525615574876253[15] = delta_x[15] + nom_x[15];
   out_6167525615574876253[16] = delta_x[16] + nom_x[16];
   out_6167525615574876253[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8657746229211315654) {
   out_8657746229211315654[0] = -nom_x[0] + true_x[0];
   out_8657746229211315654[1] = -nom_x[1] + true_x[1];
   out_8657746229211315654[2] = -nom_x[2] + true_x[2];
   out_8657746229211315654[3] = -nom_x[3] + true_x[3];
   out_8657746229211315654[4] = -nom_x[4] + true_x[4];
   out_8657746229211315654[5] = -nom_x[5] + true_x[5];
   out_8657746229211315654[6] = -nom_x[6] + true_x[6];
   out_8657746229211315654[7] = -nom_x[7] + true_x[7];
   out_8657746229211315654[8] = -nom_x[8] + true_x[8];
   out_8657746229211315654[9] = -nom_x[9] + true_x[9];
   out_8657746229211315654[10] = -nom_x[10] + true_x[10];
   out_8657746229211315654[11] = -nom_x[11] + true_x[11];
   out_8657746229211315654[12] = -nom_x[12] + true_x[12];
   out_8657746229211315654[13] = -nom_x[13] + true_x[13];
   out_8657746229211315654[14] = -nom_x[14] + true_x[14];
   out_8657746229211315654[15] = -nom_x[15] + true_x[15];
   out_8657746229211315654[16] = -nom_x[16] + true_x[16];
   out_8657746229211315654[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_8244082865274323662) {
   out_8244082865274323662[0] = 1.0;
   out_8244082865274323662[1] = 0.0;
   out_8244082865274323662[2] = 0.0;
   out_8244082865274323662[3] = 0.0;
   out_8244082865274323662[4] = 0.0;
   out_8244082865274323662[5] = 0.0;
   out_8244082865274323662[6] = 0.0;
   out_8244082865274323662[7] = 0.0;
   out_8244082865274323662[8] = 0.0;
   out_8244082865274323662[9] = 0.0;
   out_8244082865274323662[10] = 0.0;
   out_8244082865274323662[11] = 0.0;
   out_8244082865274323662[12] = 0.0;
   out_8244082865274323662[13] = 0.0;
   out_8244082865274323662[14] = 0.0;
   out_8244082865274323662[15] = 0.0;
   out_8244082865274323662[16] = 0.0;
   out_8244082865274323662[17] = 0.0;
   out_8244082865274323662[18] = 0.0;
   out_8244082865274323662[19] = 1.0;
   out_8244082865274323662[20] = 0.0;
   out_8244082865274323662[21] = 0.0;
   out_8244082865274323662[22] = 0.0;
   out_8244082865274323662[23] = 0.0;
   out_8244082865274323662[24] = 0.0;
   out_8244082865274323662[25] = 0.0;
   out_8244082865274323662[26] = 0.0;
   out_8244082865274323662[27] = 0.0;
   out_8244082865274323662[28] = 0.0;
   out_8244082865274323662[29] = 0.0;
   out_8244082865274323662[30] = 0.0;
   out_8244082865274323662[31] = 0.0;
   out_8244082865274323662[32] = 0.0;
   out_8244082865274323662[33] = 0.0;
   out_8244082865274323662[34] = 0.0;
   out_8244082865274323662[35] = 0.0;
   out_8244082865274323662[36] = 0.0;
   out_8244082865274323662[37] = 0.0;
   out_8244082865274323662[38] = 1.0;
   out_8244082865274323662[39] = 0.0;
   out_8244082865274323662[40] = 0.0;
   out_8244082865274323662[41] = 0.0;
   out_8244082865274323662[42] = 0.0;
   out_8244082865274323662[43] = 0.0;
   out_8244082865274323662[44] = 0.0;
   out_8244082865274323662[45] = 0.0;
   out_8244082865274323662[46] = 0.0;
   out_8244082865274323662[47] = 0.0;
   out_8244082865274323662[48] = 0.0;
   out_8244082865274323662[49] = 0.0;
   out_8244082865274323662[50] = 0.0;
   out_8244082865274323662[51] = 0.0;
   out_8244082865274323662[52] = 0.0;
   out_8244082865274323662[53] = 0.0;
   out_8244082865274323662[54] = 0.0;
   out_8244082865274323662[55] = 0.0;
   out_8244082865274323662[56] = 0.0;
   out_8244082865274323662[57] = 1.0;
   out_8244082865274323662[58] = 0.0;
   out_8244082865274323662[59] = 0.0;
   out_8244082865274323662[60] = 0.0;
   out_8244082865274323662[61] = 0.0;
   out_8244082865274323662[62] = 0.0;
   out_8244082865274323662[63] = 0.0;
   out_8244082865274323662[64] = 0.0;
   out_8244082865274323662[65] = 0.0;
   out_8244082865274323662[66] = 0.0;
   out_8244082865274323662[67] = 0.0;
   out_8244082865274323662[68] = 0.0;
   out_8244082865274323662[69] = 0.0;
   out_8244082865274323662[70] = 0.0;
   out_8244082865274323662[71] = 0.0;
   out_8244082865274323662[72] = 0.0;
   out_8244082865274323662[73] = 0.0;
   out_8244082865274323662[74] = 0.0;
   out_8244082865274323662[75] = 0.0;
   out_8244082865274323662[76] = 1.0;
   out_8244082865274323662[77] = 0.0;
   out_8244082865274323662[78] = 0.0;
   out_8244082865274323662[79] = 0.0;
   out_8244082865274323662[80] = 0.0;
   out_8244082865274323662[81] = 0.0;
   out_8244082865274323662[82] = 0.0;
   out_8244082865274323662[83] = 0.0;
   out_8244082865274323662[84] = 0.0;
   out_8244082865274323662[85] = 0.0;
   out_8244082865274323662[86] = 0.0;
   out_8244082865274323662[87] = 0.0;
   out_8244082865274323662[88] = 0.0;
   out_8244082865274323662[89] = 0.0;
   out_8244082865274323662[90] = 0.0;
   out_8244082865274323662[91] = 0.0;
   out_8244082865274323662[92] = 0.0;
   out_8244082865274323662[93] = 0.0;
   out_8244082865274323662[94] = 0.0;
   out_8244082865274323662[95] = 1.0;
   out_8244082865274323662[96] = 0.0;
   out_8244082865274323662[97] = 0.0;
   out_8244082865274323662[98] = 0.0;
   out_8244082865274323662[99] = 0.0;
   out_8244082865274323662[100] = 0.0;
   out_8244082865274323662[101] = 0.0;
   out_8244082865274323662[102] = 0.0;
   out_8244082865274323662[103] = 0.0;
   out_8244082865274323662[104] = 0.0;
   out_8244082865274323662[105] = 0.0;
   out_8244082865274323662[106] = 0.0;
   out_8244082865274323662[107] = 0.0;
   out_8244082865274323662[108] = 0.0;
   out_8244082865274323662[109] = 0.0;
   out_8244082865274323662[110] = 0.0;
   out_8244082865274323662[111] = 0.0;
   out_8244082865274323662[112] = 0.0;
   out_8244082865274323662[113] = 0.0;
   out_8244082865274323662[114] = 1.0;
   out_8244082865274323662[115] = 0.0;
   out_8244082865274323662[116] = 0.0;
   out_8244082865274323662[117] = 0.0;
   out_8244082865274323662[118] = 0.0;
   out_8244082865274323662[119] = 0.0;
   out_8244082865274323662[120] = 0.0;
   out_8244082865274323662[121] = 0.0;
   out_8244082865274323662[122] = 0.0;
   out_8244082865274323662[123] = 0.0;
   out_8244082865274323662[124] = 0.0;
   out_8244082865274323662[125] = 0.0;
   out_8244082865274323662[126] = 0.0;
   out_8244082865274323662[127] = 0.0;
   out_8244082865274323662[128] = 0.0;
   out_8244082865274323662[129] = 0.0;
   out_8244082865274323662[130] = 0.0;
   out_8244082865274323662[131] = 0.0;
   out_8244082865274323662[132] = 0.0;
   out_8244082865274323662[133] = 1.0;
   out_8244082865274323662[134] = 0.0;
   out_8244082865274323662[135] = 0.0;
   out_8244082865274323662[136] = 0.0;
   out_8244082865274323662[137] = 0.0;
   out_8244082865274323662[138] = 0.0;
   out_8244082865274323662[139] = 0.0;
   out_8244082865274323662[140] = 0.0;
   out_8244082865274323662[141] = 0.0;
   out_8244082865274323662[142] = 0.0;
   out_8244082865274323662[143] = 0.0;
   out_8244082865274323662[144] = 0.0;
   out_8244082865274323662[145] = 0.0;
   out_8244082865274323662[146] = 0.0;
   out_8244082865274323662[147] = 0.0;
   out_8244082865274323662[148] = 0.0;
   out_8244082865274323662[149] = 0.0;
   out_8244082865274323662[150] = 0.0;
   out_8244082865274323662[151] = 0.0;
   out_8244082865274323662[152] = 1.0;
   out_8244082865274323662[153] = 0.0;
   out_8244082865274323662[154] = 0.0;
   out_8244082865274323662[155] = 0.0;
   out_8244082865274323662[156] = 0.0;
   out_8244082865274323662[157] = 0.0;
   out_8244082865274323662[158] = 0.0;
   out_8244082865274323662[159] = 0.0;
   out_8244082865274323662[160] = 0.0;
   out_8244082865274323662[161] = 0.0;
   out_8244082865274323662[162] = 0.0;
   out_8244082865274323662[163] = 0.0;
   out_8244082865274323662[164] = 0.0;
   out_8244082865274323662[165] = 0.0;
   out_8244082865274323662[166] = 0.0;
   out_8244082865274323662[167] = 0.0;
   out_8244082865274323662[168] = 0.0;
   out_8244082865274323662[169] = 0.0;
   out_8244082865274323662[170] = 0.0;
   out_8244082865274323662[171] = 1.0;
   out_8244082865274323662[172] = 0.0;
   out_8244082865274323662[173] = 0.0;
   out_8244082865274323662[174] = 0.0;
   out_8244082865274323662[175] = 0.0;
   out_8244082865274323662[176] = 0.0;
   out_8244082865274323662[177] = 0.0;
   out_8244082865274323662[178] = 0.0;
   out_8244082865274323662[179] = 0.0;
   out_8244082865274323662[180] = 0.0;
   out_8244082865274323662[181] = 0.0;
   out_8244082865274323662[182] = 0.0;
   out_8244082865274323662[183] = 0.0;
   out_8244082865274323662[184] = 0.0;
   out_8244082865274323662[185] = 0.0;
   out_8244082865274323662[186] = 0.0;
   out_8244082865274323662[187] = 0.0;
   out_8244082865274323662[188] = 0.0;
   out_8244082865274323662[189] = 0.0;
   out_8244082865274323662[190] = 1.0;
   out_8244082865274323662[191] = 0.0;
   out_8244082865274323662[192] = 0.0;
   out_8244082865274323662[193] = 0.0;
   out_8244082865274323662[194] = 0.0;
   out_8244082865274323662[195] = 0.0;
   out_8244082865274323662[196] = 0.0;
   out_8244082865274323662[197] = 0.0;
   out_8244082865274323662[198] = 0.0;
   out_8244082865274323662[199] = 0.0;
   out_8244082865274323662[200] = 0.0;
   out_8244082865274323662[201] = 0.0;
   out_8244082865274323662[202] = 0.0;
   out_8244082865274323662[203] = 0.0;
   out_8244082865274323662[204] = 0.0;
   out_8244082865274323662[205] = 0.0;
   out_8244082865274323662[206] = 0.0;
   out_8244082865274323662[207] = 0.0;
   out_8244082865274323662[208] = 0.0;
   out_8244082865274323662[209] = 1.0;
   out_8244082865274323662[210] = 0.0;
   out_8244082865274323662[211] = 0.0;
   out_8244082865274323662[212] = 0.0;
   out_8244082865274323662[213] = 0.0;
   out_8244082865274323662[214] = 0.0;
   out_8244082865274323662[215] = 0.0;
   out_8244082865274323662[216] = 0.0;
   out_8244082865274323662[217] = 0.0;
   out_8244082865274323662[218] = 0.0;
   out_8244082865274323662[219] = 0.0;
   out_8244082865274323662[220] = 0.0;
   out_8244082865274323662[221] = 0.0;
   out_8244082865274323662[222] = 0.0;
   out_8244082865274323662[223] = 0.0;
   out_8244082865274323662[224] = 0.0;
   out_8244082865274323662[225] = 0.0;
   out_8244082865274323662[226] = 0.0;
   out_8244082865274323662[227] = 0.0;
   out_8244082865274323662[228] = 1.0;
   out_8244082865274323662[229] = 0.0;
   out_8244082865274323662[230] = 0.0;
   out_8244082865274323662[231] = 0.0;
   out_8244082865274323662[232] = 0.0;
   out_8244082865274323662[233] = 0.0;
   out_8244082865274323662[234] = 0.0;
   out_8244082865274323662[235] = 0.0;
   out_8244082865274323662[236] = 0.0;
   out_8244082865274323662[237] = 0.0;
   out_8244082865274323662[238] = 0.0;
   out_8244082865274323662[239] = 0.0;
   out_8244082865274323662[240] = 0.0;
   out_8244082865274323662[241] = 0.0;
   out_8244082865274323662[242] = 0.0;
   out_8244082865274323662[243] = 0.0;
   out_8244082865274323662[244] = 0.0;
   out_8244082865274323662[245] = 0.0;
   out_8244082865274323662[246] = 0.0;
   out_8244082865274323662[247] = 1.0;
   out_8244082865274323662[248] = 0.0;
   out_8244082865274323662[249] = 0.0;
   out_8244082865274323662[250] = 0.0;
   out_8244082865274323662[251] = 0.0;
   out_8244082865274323662[252] = 0.0;
   out_8244082865274323662[253] = 0.0;
   out_8244082865274323662[254] = 0.0;
   out_8244082865274323662[255] = 0.0;
   out_8244082865274323662[256] = 0.0;
   out_8244082865274323662[257] = 0.0;
   out_8244082865274323662[258] = 0.0;
   out_8244082865274323662[259] = 0.0;
   out_8244082865274323662[260] = 0.0;
   out_8244082865274323662[261] = 0.0;
   out_8244082865274323662[262] = 0.0;
   out_8244082865274323662[263] = 0.0;
   out_8244082865274323662[264] = 0.0;
   out_8244082865274323662[265] = 0.0;
   out_8244082865274323662[266] = 1.0;
   out_8244082865274323662[267] = 0.0;
   out_8244082865274323662[268] = 0.0;
   out_8244082865274323662[269] = 0.0;
   out_8244082865274323662[270] = 0.0;
   out_8244082865274323662[271] = 0.0;
   out_8244082865274323662[272] = 0.0;
   out_8244082865274323662[273] = 0.0;
   out_8244082865274323662[274] = 0.0;
   out_8244082865274323662[275] = 0.0;
   out_8244082865274323662[276] = 0.0;
   out_8244082865274323662[277] = 0.0;
   out_8244082865274323662[278] = 0.0;
   out_8244082865274323662[279] = 0.0;
   out_8244082865274323662[280] = 0.0;
   out_8244082865274323662[281] = 0.0;
   out_8244082865274323662[282] = 0.0;
   out_8244082865274323662[283] = 0.0;
   out_8244082865274323662[284] = 0.0;
   out_8244082865274323662[285] = 1.0;
   out_8244082865274323662[286] = 0.0;
   out_8244082865274323662[287] = 0.0;
   out_8244082865274323662[288] = 0.0;
   out_8244082865274323662[289] = 0.0;
   out_8244082865274323662[290] = 0.0;
   out_8244082865274323662[291] = 0.0;
   out_8244082865274323662[292] = 0.0;
   out_8244082865274323662[293] = 0.0;
   out_8244082865274323662[294] = 0.0;
   out_8244082865274323662[295] = 0.0;
   out_8244082865274323662[296] = 0.0;
   out_8244082865274323662[297] = 0.0;
   out_8244082865274323662[298] = 0.0;
   out_8244082865274323662[299] = 0.0;
   out_8244082865274323662[300] = 0.0;
   out_8244082865274323662[301] = 0.0;
   out_8244082865274323662[302] = 0.0;
   out_8244082865274323662[303] = 0.0;
   out_8244082865274323662[304] = 1.0;
   out_8244082865274323662[305] = 0.0;
   out_8244082865274323662[306] = 0.0;
   out_8244082865274323662[307] = 0.0;
   out_8244082865274323662[308] = 0.0;
   out_8244082865274323662[309] = 0.0;
   out_8244082865274323662[310] = 0.0;
   out_8244082865274323662[311] = 0.0;
   out_8244082865274323662[312] = 0.0;
   out_8244082865274323662[313] = 0.0;
   out_8244082865274323662[314] = 0.0;
   out_8244082865274323662[315] = 0.0;
   out_8244082865274323662[316] = 0.0;
   out_8244082865274323662[317] = 0.0;
   out_8244082865274323662[318] = 0.0;
   out_8244082865274323662[319] = 0.0;
   out_8244082865274323662[320] = 0.0;
   out_8244082865274323662[321] = 0.0;
   out_8244082865274323662[322] = 0.0;
   out_8244082865274323662[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_6288059084794879923) {
   out_6288059084794879923[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_6288059084794879923[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_6288059084794879923[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_6288059084794879923[3] = dt*state[12] + state[3];
   out_6288059084794879923[4] = dt*state[13] + state[4];
   out_6288059084794879923[5] = dt*state[14] + state[5];
   out_6288059084794879923[6] = state[6];
   out_6288059084794879923[7] = state[7];
   out_6288059084794879923[8] = state[8];
   out_6288059084794879923[9] = state[9];
   out_6288059084794879923[10] = state[10];
   out_6288059084794879923[11] = state[11];
   out_6288059084794879923[12] = state[12];
   out_6288059084794879923[13] = state[13];
   out_6288059084794879923[14] = state[14];
   out_6288059084794879923[15] = state[15];
   out_6288059084794879923[16] = state[16];
   out_6288059084794879923[17] = state[17];
}
void F_fun(double *state, double dt, double *out_7236874212108329750) {
   out_7236874212108329750[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7236874212108329750[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7236874212108329750[2] = 0;
   out_7236874212108329750[3] = 0;
   out_7236874212108329750[4] = 0;
   out_7236874212108329750[5] = 0;
   out_7236874212108329750[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7236874212108329750[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7236874212108329750[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7236874212108329750[9] = 0;
   out_7236874212108329750[10] = 0;
   out_7236874212108329750[11] = 0;
   out_7236874212108329750[12] = 0;
   out_7236874212108329750[13] = 0;
   out_7236874212108329750[14] = 0;
   out_7236874212108329750[15] = 0;
   out_7236874212108329750[16] = 0;
   out_7236874212108329750[17] = 0;
   out_7236874212108329750[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7236874212108329750[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7236874212108329750[20] = 0;
   out_7236874212108329750[21] = 0;
   out_7236874212108329750[22] = 0;
   out_7236874212108329750[23] = 0;
   out_7236874212108329750[24] = 0;
   out_7236874212108329750[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7236874212108329750[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7236874212108329750[27] = 0;
   out_7236874212108329750[28] = 0;
   out_7236874212108329750[29] = 0;
   out_7236874212108329750[30] = 0;
   out_7236874212108329750[31] = 0;
   out_7236874212108329750[32] = 0;
   out_7236874212108329750[33] = 0;
   out_7236874212108329750[34] = 0;
   out_7236874212108329750[35] = 0;
   out_7236874212108329750[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7236874212108329750[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7236874212108329750[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7236874212108329750[39] = 0;
   out_7236874212108329750[40] = 0;
   out_7236874212108329750[41] = 0;
   out_7236874212108329750[42] = 0;
   out_7236874212108329750[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7236874212108329750[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7236874212108329750[45] = 0;
   out_7236874212108329750[46] = 0;
   out_7236874212108329750[47] = 0;
   out_7236874212108329750[48] = 0;
   out_7236874212108329750[49] = 0;
   out_7236874212108329750[50] = 0;
   out_7236874212108329750[51] = 0;
   out_7236874212108329750[52] = 0;
   out_7236874212108329750[53] = 0;
   out_7236874212108329750[54] = 0;
   out_7236874212108329750[55] = 0;
   out_7236874212108329750[56] = 0;
   out_7236874212108329750[57] = 1;
   out_7236874212108329750[58] = 0;
   out_7236874212108329750[59] = 0;
   out_7236874212108329750[60] = 0;
   out_7236874212108329750[61] = 0;
   out_7236874212108329750[62] = 0;
   out_7236874212108329750[63] = 0;
   out_7236874212108329750[64] = 0;
   out_7236874212108329750[65] = 0;
   out_7236874212108329750[66] = dt;
   out_7236874212108329750[67] = 0;
   out_7236874212108329750[68] = 0;
   out_7236874212108329750[69] = 0;
   out_7236874212108329750[70] = 0;
   out_7236874212108329750[71] = 0;
   out_7236874212108329750[72] = 0;
   out_7236874212108329750[73] = 0;
   out_7236874212108329750[74] = 0;
   out_7236874212108329750[75] = 0;
   out_7236874212108329750[76] = 1;
   out_7236874212108329750[77] = 0;
   out_7236874212108329750[78] = 0;
   out_7236874212108329750[79] = 0;
   out_7236874212108329750[80] = 0;
   out_7236874212108329750[81] = 0;
   out_7236874212108329750[82] = 0;
   out_7236874212108329750[83] = 0;
   out_7236874212108329750[84] = 0;
   out_7236874212108329750[85] = dt;
   out_7236874212108329750[86] = 0;
   out_7236874212108329750[87] = 0;
   out_7236874212108329750[88] = 0;
   out_7236874212108329750[89] = 0;
   out_7236874212108329750[90] = 0;
   out_7236874212108329750[91] = 0;
   out_7236874212108329750[92] = 0;
   out_7236874212108329750[93] = 0;
   out_7236874212108329750[94] = 0;
   out_7236874212108329750[95] = 1;
   out_7236874212108329750[96] = 0;
   out_7236874212108329750[97] = 0;
   out_7236874212108329750[98] = 0;
   out_7236874212108329750[99] = 0;
   out_7236874212108329750[100] = 0;
   out_7236874212108329750[101] = 0;
   out_7236874212108329750[102] = 0;
   out_7236874212108329750[103] = 0;
   out_7236874212108329750[104] = dt;
   out_7236874212108329750[105] = 0;
   out_7236874212108329750[106] = 0;
   out_7236874212108329750[107] = 0;
   out_7236874212108329750[108] = 0;
   out_7236874212108329750[109] = 0;
   out_7236874212108329750[110] = 0;
   out_7236874212108329750[111] = 0;
   out_7236874212108329750[112] = 0;
   out_7236874212108329750[113] = 0;
   out_7236874212108329750[114] = 1;
   out_7236874212108329750[115] = 0;
   out_7236874212108329750[116] = 0;
   out_7236874212108329750[117] = 0;
   out_7236874212108329750[118] = 0;
   out_7236874212108329750[119] = 0;
   out_7236874212108329750[120] = 0;
   out_7236874212108329750[121] = 0;
   out_7236874212108329750[122] = 0;
   out_7236874212108329750[123] = 0;
   out_7236874212108329750[124] = 0;
   out_7236874212108329750[125] = 0;
   out_7236874212108329750[126] = 0;
   out_7236874212108329750[127] = 0;
   out_7236874212108329750[128] = 0;
   out_7236874212108329750[129] = 0;
   out_7236874212108329750[130] = 0;
   out_7236874212108329750[131] = 0;
   out_7236874212108329750[132] = 0;
   out_7236874212108329750[133] = 1;
   out_7236874212108329750[134] = 0;
   out_7236874212108329750[135] = 0;
   out_7236874212108329750[136] = 0;
   out_7236874212108329750[137] = 0;
   out_7236874212108329750[138] = 0;
   out_7236874212108329750[139] = 0;
   out_7236874212108329750[140] = 0;
   out_7236874212108329750[141] = 0;
   out_7236874212108329750[142] = 0;
   out_7236874212108329750[143] = 0;
   out_7236874212108329750[144] = 0;
   out_7236874212108329750[145] = 0;
   out_7236874212108329750[146] = 0;
   out_7236874212108329750[147] = 0;
   out_7236874212108329750[148] = 0;
   out_7236874212108329750[149] = 0;
   out_7236874212108329750[150] = 0;
   out_7236874212108329750[151] = 0;
   out_7236874212108329750[152] = 1;
   out_7236874212108329750[153] = 0;
   out_7236874212108329750[154] = 0;
   out_7236874212108329750[155] = 0;
   out_7236874212108329750[156] = 0;
   out_7236874212108329750[157] = 0;
   out_7236874212108329750[158] = 0;
   out_7236874212108329750[159] = 0;
   out_7236874212108329750[160] = 0;
   out_7236874212108329750[161] = 0;
   out_7236874212108329750[162] = 0;
   out_7236874212108329750[163] = 0;
   out_7236874212108329750[164] = 0;
   out_7236874212108329750[165] = 0;
   out_7236874212108329750[166] = 0;
   out_7236874212108329750[167] = 0;
   out_7236874212108329750[168] = 0;
   out_7236874212108329750[169] = 0;
   out_7236874212108329750[170] = 0;
   out_7236874212108329750[171] = 1;
   out_7236874212108329750[172] = 0;
   out_7236874212108329750[173] = 0;
   out_7236874212108329750[174] = 0;
   out_7236874212108329750[175] = 0;
   out_7236874212108329750[176] = 0;
   out_7236874212108329750[177] = 0;
   out_7236874212108329750[178] = 0;
   out_7236874212108329750[179] = 0;
   out_7236874212108329750[180] = 0;
   out_7236874212108329750[181] = 0;
   out_7236874212108329750[182] = 0;
   out_7236874212108329750[183] = 0;
   out_7236874212108329750[184] = 0;
   out_7236874212108329750[185] = 0;
   out_7236874212108329750[186] = 0;
   out_7236874212108329750[187] = 0;
   out_7236874212108329750[188] = 0;
   out_7236874212108329750[189] = 0;
   out_7236874212108329750[190] = 1;
   out_7236874212108329750[191] = 0;
   out_7236874212108329750[192] = 0;
   out_7236874212108329750[193] = 0;
   out_7236874212108329750[194] = 0;
   out_7236874212108329750[195] = 0;
   out_7236874212108329750[196] = 0;
   out_7236874212108329750[197] = 0;
   out_7236874212108329750[198] = 0;
   out_7236874212108329750[199] = 0;
   out_7236874212108329750[200] = 0;
   out_7236874212108329750[201] = 0;
   out_7236874212108329750[202] = 0;
   out_7236874212108329750[203] = 0;
   out_7236874212108329750[204] = 0;
   out_7236874212108329750[205] = 0;
   out_7236874212108329750[206] = 0;
   out_7236874212108329750[207] = 0;
   out_7236874212108329750[208] = 0;
   out_7236874212108329750[209] = 1;
   out_7236874212108329750[210] = 0;
   out_7236874212108329750[211] = 0;
   out_7236874212108329750[212] = 0;
   out_7236874212108329750[213] = 0;
   out_7236874212108329750[214] = 0;
   out_7236874212108329750[215] = 0;
   out_7236874212108329750[216] = 0;
   out_7236874212108329750[217] = 0;
   out_7236874212108329750[218] = 0;
   out_7236874212108329750[219] = 0;
   out_7236874212108329750[220] = 0;
   out_7236874212108329750[221] = 0;
   out_7236874212108329750[222] = 0;
   out_7236874212108329750[223] = 0;
   out_7236874212108329750[224] = 0;
   out_7236874212108329750[225] = 0;
   out_7236874212108329750[226] = 0;
   out_7236874212108329750[227] = 0;
   out_7236874212108329750[228] = 1;
   out_7236874212108329750[229] = 0;
   out_7236874212108329750[230] = 0;
   out_7236874212108329750[231] = 0;
   out_7236874212108329750[232] = 0;
   out_7236874212108329750[233] = 0;
   out_7236874212108329750[234] = 0;
   out_7236874212108329750[235] = 0;
   out_7236874212108329750[236] = 0;
   out_7236874212108329750[237] = 0;
   out_7236874212108329750[238] = 0;
   out_7236874212108329750[239] = 0;
   out_7236874212108329750[240] = 0;
   out_7236874212108329750[241] = 0;
   out_7236874212108329750[242] = 0;
   out_7236874212108329750[243] = 0;
   out_7236874212108329750[244] = 0;
   out_7236874212108329750[245] = 0;
   out_7236874212108329750[246] = 0;
   out_7236874212108329750[247] = 1;
   out_7236874212108329750[248] = 0;
   out_7236874212108329750[249] = 0;
   out_7236874212108329750[250] = 0;
   out_7236874212108329750[251] = 0;
   out_7236874212108329750[252] = 0;
   out_7236874212108329750[253] = 0;
   out_7236874212108329750[254] = 0;
   out_7236874212108329750[255] = 0;
   out_7236874212108329750[256] = 0;
   out_7236874212108329750[257] = 0;
   out_7236874212108329750[258] = 0;
   out_7236874212108329750[259] = 0;
   out_7236874212108329750[260] = 0;
   out_7236874212108329750[261] = 0;
   out_7236874212108329750[262] = 0;
   out_7236874212108329750[263] = 0;
   out_7236874212108329750[264] = 0;
   out_7236874212108329750[265] = 0;
   out_7236874212108329750[266] = 1;
   out_7236874212108329750[267] = 0;
   out_7236874212108329750[268] = 0;
   out_7236874212108329750[269] = 0;
   out_7236874212108329750[270] = 0;
   out_7236874212108329750[271] = 0;
   out_7236874212108329750[272] = 0;
   out_7236874212108329750[273] = 0;
   out_7236874212108329750[274] = 0;
   out_7236874212108329750[275] = 0;
   out_7236874212108329750[276] = 0;
   out_7236874212108329750[277] = 0;
   out_7236874212108329750[278] = 0;
   out_7236874212108329750[279] = 0;
   out_7236874212108329750[280] = 0;
   out_7236874212108329750[281] = 0;
   out_7236874212108329750[282] = 0;
   out_7236874212108329750[283] = 0;
   out_7236874212108329750[284] = 0;
   out_7236874212108329750[285] = 1;
   out_7236874212108329750[286] = 0;
   out_7236874212108329750[287] = 0;
   out_7236874212108329750[288] = 0;
   out_7236874212108329750[289] = 0;
   out_7236874212108329750[290] = 0;
   out_7236874212108329750[291] = 0;
   out_7236874212108329750[292] = 0;
   out_7236874212108329750[293] = 0;
   out_7236874212108329750[294] = 0;
   out_7236874212108329750[295] = 0;
   out_7236874212108329750[296] = 0;
   out_7236874212108329750[297] = 0;
   out_7236874212108329750[298] = 0;
   out_7236874212108329750[299] = 0;
   out_7236874212108329750[300] = 0;
   out_7236874212108329750[301] = 0;
   out_7236874212108329750[302] = 0;
   out_7236874212108329750[303] = 0;
   out_7236874212108329750[304] = 1;
   out_7236874212108329750[305] = 0;
   out_7236874212108329750[306] = 0;
   out_7236874212108329750[307] = 0;
   out_7236874212108329750[308] = 0;
   out_7236874212108329750[309] = 0;
   out_7236874212108329750[310] = 0;
   out_7236874212108329750[311] = 0;
   out_7236874212108329750[312] = 0;
   out_7236874212108329750[313] = 0;
   out_7236874212108329750[314] = 0;
   out_7236874212108329750[315] = 0;
   out_7236874212108329750[316] = 0;
   out_7236874212108329750[317] = 0;
   out_7236874212108329750[318] = 0;
   out_7236874212108329750[319] = 0;
   out_7236874212108329750[320] = 0;
   out_7236874212108329750[321] = 0;
   out_7236874212108329750[322] = 0;
   out_7236874212108329750[323] = 1;
}
void h_4(double *state, double *unused, double *out_6261259594144104730) {
   out_6261259594144104730[0] = state[6] + state[9];
   out_6261259594144104730[1] = state[7] + state[10];
   out_6261259594144104730[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_6499476752447183532) {
   out_6499476752447183532[0] = 0;
   out_6499476752447183532[1] = 0;
   out_6499476752447183532[2] = 0;
   out_6499476752447183532[3] = 0;
   out_6499476752447183532[4] = 0;
   out_6499476752447183532[5] = 0;
   out_6499476752447183532[6] = 1;
   out_6499476752447183532[7] = 0;
   out_6499476752447183532[8] = 0;
   out_6499476752447183532[9] = 1;
   out_6499476752447183532[10] = 0;
   out_6499476752447183532[11] = 0;
   out_6499476752447183532[12] = 0;
   out_6499476752447183532[13] = 0;
   out_6499476752447183532[14] = 0;
   out_6499476752447183532[15] = 0;
   out_6499476752447183532[16] = 0;
   out_6499476752447183532[17] = 0;
   out_6499476752447183532[18] = 0;
   out_6499476752447183532[19] = 0;
   out_6499476752447183532[20] = 0;
   out_6499476752447183532[21] = 0;
   out_6499476752447183532[22] = 0;
   out_6499476752447183532[23] = 0;
   out_6499476752447183532[24] = 0;
   out_6499476752447183532[25] = 1;
   out_6499476752447183532[26] = 0;
   out_6499476752447183532[27] = 0;
   out_6499476752447183532[28] = 1;
   out_6499476752447183532[29] = 0;
   out_6499476752447183532[30] = 0;
   out_6499476752447183532[31] = 0;
   out_6499476752447183532[32] = 0;
   out_6499476752447183532[33] = 0;
   out_6499476752447183532[34] = 0;
   out_6499476752447183532[35] = 0;
   out_6499476752447183532[36] = 0;
   out_6499476752447183532[37] = 0;
   out_6499476752447183532[38] = 0;
   out_6499476752447183532[39] = 0;
   out_6499476752447183532[40] = 0;
   out_6499476752447183532[41] = 0;
   out_6499476752447183532[42] = 0;
   out_6499476752447183532[43] = 0;
   out_6499476752447183532[44] = 1;
   out_6499476752447183532[45] = 0;
   out_6499476752447183532[46] = 0;
   out_6499476752447183532[47] = 1;
   out_6499476752447183532[48] = 0;
   out_6499476752447183532[49] = 0;
   out_6499476752447183532[50] = 0;
   out_6499476752447183532[51] = 0;
   out_6499476752447183532[52] = 0;
   out_6499476752447183532[53] = 0;
}
void h_10(double *state, double *unused, double *out_6242140905112775300) {
   out_6242140905112775300[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_6242140905112775300[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_6242140905112775300[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_630116466754361676) {
   out_630116466754361676[0] = 0;
   out_630116466754361676[1] = 9.8100000000000005*cos(state[1]);
   out_630116466754361676[2] = 0;
   out_630116466754361676[3] = 0;
   out_630116466754361676[4] = -state[8];
   out_630116466754361676[5] = state[7];
   out_630116466754361676[6] = 0;
   out_630116466754361676[7] = state[5];
   out_630116466754361676[8] = -state[4];
   out_630116466754361676[9] = 0;
   out_630116466754361676[10] = 0;
   out_630116466754361676[11] = 0;
   out_630116466754361676[12] = 1;
   out_630116466754361676[13] = 0;
   out_630116466754361676[14] = 0;
   out_630116466754361676[15] = 1;
   out_630116466754361676[16] = 0;
   out_630116466754361676[17] = 0;
   out_630116466754361676[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_630116466754361676[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_630116466754361676[20] = 0;
   out_630116466754361676[21] = state[8];
   out_630116466754361676[22] = 0;
   out_630116466754361676[23] = -state[6];
   out_630116466754361676[24] = -state[5];
   out_630116466754361676[25] = 0;
   out_630116466754361676[26] = state[3];
   out_630116466754361676[27] = 0;
   out_630116466754361676[28] = 0;
   out_630116466754361676[29] = 0;
   out_630116466754361676[30] = 0;
   out_630116466754361676[31] = 1;
   out_630116466754361676[32] = 0;
   out_630116466754361676[33] = 0;
   out_630116466754361676[34] = 1;
   out_630116466754361676[35] = 0;
   out_630116466754361676[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_630116466754361676[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_630116466754361676[38] = 0;
   out_630116466754361676[39] = -state[7];
   out_630116466754361676[40] = state[6];
   out_630116466754361676[41] = 0;
   out_630116466754361676[42] = state[4];
   out_630116466754361676[43] = -state[3];
   out_630116466754361676[44] = 0;
   out_630116466754361676[45] = 0;
   out_630116466754361676[46] = 0;
   out_630116466754361676[47] = 0;
   out_630116466754361676[48] = 0;
   out_630116466754361676[49] = 0;
   out_630116466754361676[50] = 1;
   out_630116466754361676[51] = 0;
   out_630116466754361676[52] = 0;
   out_630116466754361676[53] = 1;
}
void h_13(double *state, double *unused, double *out_8966555842540160286) {
   out_8966555842540160286[0] = state[3];
   out_8966555842540160286[1] = state[4];
   out_8966555842540160286[2] = state[5];
}
void H_13(double *state, double *unused, double *out_8734993495930035283) {
   out_8734993495930035283[0] = 0;
   out_8734993495930035283[1] = 0;
   out_8734993495930035283[2] = 0;
   out_8734993495930035283[3] = 1;
   out_8734993495930035283[4] = 0;
   out_8734993495930035283[5] = 0;
   out_8734993495930035283[6] = 0;
   out_8734993495930035283[7] = 0;
   out_8734993495930035283[8] = 0;
   out_8734993495930035283[9] = 0;
   out_8734993495930035283[10] = 0;
   out_8734993495930035283[11] = 0;
   out_8734993495930035283[12] = 0;
   out_8734993495930035283[13] = 0;
   out_8734993495930035283[14] = 0;
   out_8734993495930035283[15] = 0;
   out_8734993495930035283[16] = 0;
   out_8734993495930035283[17] = 0;
   out_8734993495930035283[18] = 0;
   out_8734993495930035283[19] = 0;
   out_8734993495930035283[20] = 0;
   out_8734993495930035283[21] = 0;
   out_8734993495930035283[22] = 1;
   out_8734993495930035283[23] = 0;
   out_8734993495930035283[24] = 0;
   out_8734993495930035283[25] = 0;
   out_8734993495930035283[26] = 0;
   out_8734993495930035283[27] = 0;
   out_8734993495930035283[28] = 0;
   out_8734993495930035283[29] = 0;
   out_8734993495930035283[30] = 0;
   out_8734993495930035283[31] = 0;
   out_8734993495930035283[32] = 0;
   out_8734993495930035283[33] = 0;
   out_8734993495930035283[34] = 0;
   out_8734993495930035283[35] = 0;
   out_8734993495930035283[36] = 0;
   out_8734993495930035283[37] = 0;
   out_8734993495930035283[38] = 0;
   out_8734993495930035283[39] = 0;
   out_8734993495930035283[40] = 0;
   out_8734993495930035283[41] = 1;
   out_8734993495930035283[42] = 0;
   out_8734993495930035283[43] = 0;
   out_8734993495930035283[44] = 0;
   out_8734993495930035283[45] = 0;
   out_8734993495930035283[46] = 0;
   out_8734993495930035283[47] = 0;
   out_8734993495930035283[48] = 0;
   out_8734993495930035283[49] = 0;
   out_8734993495930035283[50] = 0;
   out_8734993495930035283[51] = 0;
   out_8734993495930035283[52] = 0;
   out_8734993495930035283[53] = 0;
}
void h_14(double *state, double *unused, double *out_5640486197908056385) {
   out_5640486197908056385[0] = state[6];
   out_5640486197908056385[1] = state[7];
   out_5640486197908056385[2] = state[8];
}
void H_14(double *state, double *unused, double *out_7984026464922883555) {
   out_7984026464922883555[0] = 0;
   out_7984026464922883555[1] = 0;
   out_7984026464922883555[2] = 0;
   out_7984026464922883555[3] = 0;
   out_7984026464922883555[4] = 0;
   out_7984026464922883555[5] = 0;
   out_7984026464922883555[6] = 1;
   out_7984026464922883555[7] = 0;
   out_7984026464922883555[8] = 0;
   out_7984026464922883555[9] = 0;
   out_7984026464922883555[10] = 0;
   out_7984026464922883555[11] = 0;
   out_7984026464922883555[12] = 0;
   out_7984026464922883555[13] = 0;
   out_7984026464922883555[14] = 0;
   out_7984026464922883555[15] = 0;
   out_7984026464922883555[16] = 0;
   out_7984026464922883555[17] = 0;
   out_7984026464922883555[18] = 0;
   out_7984026464922883555[19] = 0;
   out_7984026464922883555[20] = 0;
   out_7984026464922883555[21] = 0;
   out_7984026464922883555[22] = 0;
   out_7984026464922883555[23] = 0;
   out_7984026464922883555[24] = 0;
   out_7984026464922883555[25] = 1;
   out_7984026464922883555[26] = 0;
   out_7984026464922883555[27] = 0;
   out_7984026464922883555[28] = 0;
   out_7984026464922883555[29] = 0;
   out_7984026464922883555[30] = 0;
   out_7984026464922883555[31] = 0;
   out_7984026464922883555[32] = 0;
   out_7984026464922883555[33] = 0;
   out_7984026464922883555[34] = 0;
   out_7984026464922883555[35] = 0;
   out_7984026464922883555[36] = 0;
   out_7984026464922883555[37] = 0;
   out_7984026464922883555[38] = 0;
   out_7984026464922883555[39] = 0;
   out_7984026464922883555[40] = 0;
   out_7984026464922883555[41] = 0;
   out_7984026464922883555[42] = 0;
   out_7984026464922883555[43] = 0;
   out_7984026464922883555[44] = 1;
   out_7984026464922883555[45] = 0;
   out_7984026464922883555[46] = 0;
   out_7984026464922883555[47] = 0;
   out_7984026464922883555[48] = 0;
   out_7984026464922883555[49] = 0;
   out_7984026464922883555[50] = 0;
   out_7984026464922883555[51] = 0;
   out_7984026464922883555[52] = 0;
   out_7984026464922883555[53] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_6167525615574876253) {
  err_fun(nom_x, delta_x, out_6167525615574876253);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_8657746229211315654) {
  inv_err_fun(nom_x, true_x, out_8657746229211315654);
}
void pose_H_mod_fun(double *state, double *out_8244082865274323662) {
  H_mod_fun(state, out_8244082865274323662);
}
void pose_f_fun(double *state, double dt, double *out_6288059084794879923) {
  f_fun(state,  dt, out_6288059084794879923);
}
void pose_F_fun(double *state, double dt, double *out_7236874212108329750) {
  F_fun(state,  dt, out_7236874212108329750);
}
void pose_h_4(double *state, double *unused, double *out_6261259594144104730) {
  h_4(state, unused, out_6261259594144104730);
}
void pose_H_4(double *state, double *unused, double *out_6499476752447183532) {
  H_4(state, unused, out_6499476752447183532);
}
void pose_h_10(double *state, double *unused, double *out_6242140905112775300) {
  h_10(state, unused, out_6242140905112775300);
}
void pose_H_10(double *state, double *unused, double *out_630116466754361676) {
  H_10(state, unused, out_630116466754361676);
}
void pose_h_13(double *state, double *unused, double *out_8966555842540160286) {
  h_13(state, unused, out_8966555842540160286);
}
void pose_H_13(double *state, double *unused, double *out_8734993495930035283) {
  H_13(state, unused, out_8734993495930035283);
}
void pose_h_14(double *state, double *unused, double *out_5640486197908056385) {
  h_14(state, unused, out_5640486197908056385);
}
void pose_H_14(double *state, double *unused, double *out_7984026464922883555) {
  H_14(state, unused, out_7984026464922883555);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
