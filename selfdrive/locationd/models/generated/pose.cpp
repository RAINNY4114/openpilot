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
void err_fun(double *nom_x, double *delta_x, double *out_8008231329674625977) {
   out_8008231329674625977[0] = delta_x[0] + nom_x[0];
   out_8008231329674625977[1] = delta_x[1] + nom_x[1];
   out_8008231329674625977[2] = delta_x[2] + nom_x[2];
   out_8008231329674625977[3] = delta_x[3] + nom_x[3];
   out_8008231329674625977[4] = delta_x[4] + nom_x[4];
   out_8008231329674625977[5] = delta_x[5] + nom_x[5];
   out_8008231329674625977[6] = delta_x[6] + nom_x[6];
   out_8008231329674625977[7] = delta_x[7] + nom_x[7];
   out_8008231329674625977[8] = delta_x[8] + nom_x[8];
   out_8008231329674625977[9] = delta_x[9] + nom_x[9];
   out_8008231329674625977[10] = delta_x[10] + nom_x[10];
   out_8008231329674625977[11] = delta_x[11] + nom_x[11];
   out_8008231329674625977[12] = delta_x[12] + nom_x[12];
   out_8008231329674625977[13] = delta_x[13] + nom_x[13];
   out_8008231329674625977[14] = delta_x[14] + nom_x[14];
   out_8008231329674625977[15] = delta_x[15] + nom_x[15];
   out_8008231329674625977[16] = delta_x[16] + nom_x[16];
   out_8008231329674625977[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_9148252894232034006) {
   out_9148252894232034006[0] = -nom_x[0] + true_x[0];
   out_9148252894232034006[1] = -nom_x[1] + true_x[1];
   out_9148252894232034006[2] = -nom_x[2] + true_x[2];
   out_9148252894232034006[3] = -nom_x[3] + true_x[3];
   out_9148252894232034006[4] = -nom_x[4] + true_x[4];
   out_9148252894232034006[5] = -nom_x[5] + true_x[5];
   out_9148252894232034006[6] = -nom_x[6] + true_x[6];
   out_9148252894232034006[7] = -nom_x[7] + true_x[7];
   out_9148252894232034006[8] = -nom_x[8] + true_x[8];
   out_9148252894232034006[9] = -nom_x[9] + true_x[9];
   out_9148252894232034006[10] = -nom_x[10] + true_x[10];
   out_9148252894232034006[11] = -nom_x[11] + true_x[11];
   out_9148252894232034006[12] = -nom_x[12] + true_x[12];
   out_9148252894232034006[13] = -nom_x[13] + true_x[13];
   out_9148252894232034006[14] = -nom_x[14] + true_x[14];
   out_9148252894232034006[15] = -nom_x[15] + true_x[15];
   out_9148252894232034006[16] = -nom_x[16] + true_x[16];
   out_9148252894232034006[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_1922727016412823836) {
   out_1922727016412823836[0] = 1.0;
   out_1922727016412823836[1] = 0.0;
   out_1922727016412823836[2] = 0.0;
   out_1922727016412823836[3] = 0.0;
   out_1922727016412823836[4] = 0.0;
   out_1922727016412823836[5] = 0.0;
   out_1922727016412823836[6] = 0.0;
   out_1922727016412823836[7] = 0.0;
   out_1922727016412823836[8] = 0.0;
   out_1922727016412823836[9] = 0.0;
   out_1922727016412823836[10] = 0.0;
   out_1922727016412823836[11] = 0.0;
   out_1922727016412823836[12] = 0.0;
   out_1922727016412823836[13] = 0.0;
   out_1922727016412823836[14] = 0.0;
   out_1922727016412823836[15] = 0.0;
   out_1922727016412823836[16] = 0.0;
   out_1922727016412823836[17] = 0.0;
   out_1922727016412823836[18] = 0.0;
   out_1922727016412823836[19] = 1.0;
   out_1922727016412823836[20] = 0.0;
   out_1922727016412823836[21] = 0.0;
   out_1922727016412823836[22] = 0.0;
   out_1922727016412823836[23] = 0.0;
   out_1922727016412823836[24] = 0.0;
   out_1922727016412823836[25] = 0.0;
   out_1922727016412823836[26] = 0.0;
   out_1922727016412823836[27] = 0.0;
   out_1922727016412823836[28] = 0.0;
   out_1922727016412823836[29] = 0.0;
   out_1922727016412823836[30] = 0.0;
   out_1922727016412823836[31] = 0.0;
   out_1922727016412823836[32] = 0.0;
   out_1922727016412823836[33] = 0.0;
   out_1922727016412823836[34] = 0.0;
   out_1922727016412823836[35] = 0.0;
   out_1922727016412823836[36] = 0.0;
   out_1922727016412823836[37] = 0.0;
   out_1922727016412823836[38] = 1.0;
   out_1922727016412823836[39] = 0.0;
   out_1922727016412823836[40] = 0.0;
   out_1922727016412823836[41] = 0.0;
   out_1922727016412823836[42] = 0.0;
   out_1922727016412823836[43] = 0.0;
   out_1922727016412823836[44] = 0.0;
   out_1922727016412823836[45] = 0.0;
   out_1922727016412823836[46] = 0.0;
   out_1922727016412823836[47] = 0.0;
   out_1922727016412823836[48] = 0.0;
   out_1922727016412823836[49] = 0.0;
   out_1922727016412823836[50] = 0.0;
   out_1922727016412823836[51] = 0.0;
   out_1922727016412823836[52] = 0.0;
   out_1922727016412823836[53] = 0.0;
   out_1922727016412823836[54] = 0.0;
   out_1922727016412823836[55] = 0.0;
   out_1922727016412823836[56] = 0.0;
   out_1922727016412823836[57] = 1.0;
   out_1922727016412823836[58] = 0.0;
   out_1922727016412823836[59] = 0.0;
   out_1922727016412823836[60] = 0.0;
   out_1922727016412823836[61] = 0.0;
   out_1922727016412823836[62] = 0.0;
   out_1922727016412823836[63] = 0.0;
   out_1922727016412823836[64] = 0.0;
   out_1922727016412823836[65] = 0.0;
   out_1922727016412823836[66] = 0.0;
   out_1922727016412823836[67] = 0.0;
   out_1922727016412823836[68] = 0.0;
   out_1922727016412823836[69] = 0.0;
   out_1922727016412823836[70] = 0.0;
   out_1922727016412823836[71] = 0.0;
   out_1922727016412823836[72] = 0.0;
   out_1922727016412823836[73] = 0.0;
   out_1922727016412823836[74] = 0.0;
   out_1922727016412823836[75] = 0.0;
   out_1922727016412823836[76] = 1.0;
   out_1922727016412823836[77] = 0.0;
   out_1922727016412823836[78] = 0.0;
   out_1922727016412823836[79] = 0.0;
   out_1922727016412823836[80] = 0.0;
   out_1922727016412823836[81] = 0.0;
   out_1922727016412823836[82] = 0.0;
   out_1922727016412823836[83] = 0.0;
   out_1922727016412823836[84] = 0.0;
   out_1922727016412823836[85] = 0.0;
   out_1922727016412823836[86] = 0.0;
   out_1922727016412823836[87] = 0.0;
   out_1922727016412823836[88] = 0.0;
   out_1922727016412823836[89] = 0.0;
   out_1922727016412823836[90] = 0.0;
   out_1922727016412823836[91] = 0.0;
   out_1922727016412823836[92] = 0.0;
   out_1922727016412823836[93] = 0.0;
   out_1922727016412823836[94] = 0.0;
   out_1922727016412823836[95] = 1.0;
   out_1922727016412823836[96] = 0.0;
   out_1922727016412823836[97] = 0.0;
   out_1922727016412823836[98] = 0.0;
   out_1922727016412823836[99] = 0.0;
   out_1922727016412823836[100] = 0.0;
   out_1922727016412823836[101] = 0.0;
   out_1922727016412823836[102] = 0.0;
   out_1922727016412823836[103] = 0.0;
   out_1922727016412823836[104] = 0.0;
   out_1922727016412823836[105] = 0.0;
   out_1922727016412823836[106] = 0.0;
   out_1922727016412823836[107] = 0.0;
   out_1922727016412823836[108] = 0.0;
   out_1922727016412823836[109] = 0.0;
   out_1922727016412823836[110] = 0.0;
   out_1922727016412823836[111] = 0.0;
   out_1922727016412823836[112] = 0.0;
   out_1922727016412823836[113] = 0.0;
   out_1922727016412823836[114] = 1.0;
   out_1922727016412823836[115] = 0.0;
   out_1922727016412823836[116] = 0.0;
   out_1922727016412823836[117] = 0.0;
   out_1922727016412823836[118] = 0.0;
   out_1922727016412823836[119] = 0.0;
   out_1922727016412823836[120] = 0.0;
   out_1922727016412823836[121] = 0.0;
   out_1922727016412823836[122] = 0.0;
   out_1922727016412823836[123] = 0.0;
   out_1922727016412823836[124] = 0.0;
   out_1922727016412823836[125] = 0.0;
   out_1922727016412823836[126] = 0.0;
   out_1922727016412823836[127] = 0.0;
   out_1922727016412823836[128] = 0.0;
   out_1922727016412823836[129] = 0.0;
   out_1922727016412823836[130] = 0.0;
   out_1922727016412823836[131] = 0.0;
   out_1922727016412823836[132] = 0.0;
   out_1922727016412823836[133] = 1.0;
   out_1922727016412823836[134] = 0.0;
   out_1922727016412823836[135] = 0.0;
   out_1922727016412823836[136] = 0.0;
   out_1922727016412823836[137] = 0.0;
   out_1922727016412823836[138] = 0.0;
   out_1922727016412823836[139] = 0.0;
   out_1922727016412823836[140] = 0.0;
   out_1922727016412823836[141] = 0.0;
   out_1922727016412823836[142] = 0.0;
   out_1922727016412823836[143] = 0.0;
   out_1922727016412823836[144] = 0.0;
   out_1922727016412823836[145] = 0.0;
   out_1922727016412823836[146] = 0.0;
   out_1922727016412823836[147] = 0.0;
   out_1922727016412823836[148] = 0.0;
   out_1922727016412823836[149] = 0.0;
   out_1922727016412823836[150] = 0.0;
   out_1922727016412823836[151] = 0.0;
   out_1922727016412823836[152] = 1.0;
   out_1922727016412823836[153] = 0.0;
   out_1922727016412823836[154] = 0.0;
   out_1922727016412823836[155] = 0.0;
   out_1922727016412823836[156] = 0.0;
   out_1922727016412823836[157] = 0.0;
   out_1922727016412823836[158] = 0.0;
   out_1922727016412823836[159] = 0.0;
   out_1922727016412823836[160] = 0.0;
   out_1922727016412823836[161] = 0.0;
   out_1922727016412823836[162] = 0.0;
   out_1922727016412823836[163] = 0.0;
   out_1922727016412823836[164] = 0.0;
   out_1922727016412823836[165] = 0.0;
   out_1922727016412823836[166] = 0.0;
   out_1922727016412823836[167] = 0.0;
   out_1922727016412823836[168] = 0.0;
   out_1922727016412823836[169] = 0.0;
   out_1922727016412823836[170] = 0.0;
   out_1922727016412823836[171] = 1.0;
   out_1922727016412823836[172] = 0.0;
   out_1922727016412823836[173] = 0.0;
   out_1922727016412823836[174] = 0.0;
   out_1922727016412823836[175] = 0.0;
   out_1922727016412823836[176] = 0.0;
   out_1922727016412823836[177] = 0.0;
   out_1922727016412823836[178] = 0.0;
   out_1922727016412823836[179] = 0.0;
   out_1922727016412823836[180] = 0.0;
   out_1922727016412823836[181] = 0.0;
   out_1922727016412823836[182] = 0.0;
   out_1922727016412823836[183] = 0.0;
   out_1922727016412823836[184] = 0.0;
   out_1922727016412823836[185] = 0.0;
   out_1922727016412823836[186] = 0.0;
   out_1922727016412823836[187] = 0.0;
   out_1922727016412823836[188] = 0.0;
   out_1922727016412823836[189] = 0.0;
   out_1922727016412823836[190] = 1.0;
   out_1922727016412823836[191] = 0.0;
   out_1922727016412823836[192] = 0.0;
   out_1922727016412823836[193] = 0.0;
   out_1922727016412823836[194] = 0.0;
   out_1922727016412823836[195] = 0.0;
   out_1922727016412823836[196] = 0.0;
   out_1922727016412823836[197] = 0.0;
   out_1922727016412823836[198] = 0.0;
   out_1922727016412823836[199] = 0.0;
   out_1922727016412823836[200] = 0.0;
   out_1922727016412823836[201] = 0.0;
   out_1922727016412823836[202] = 0.0;
   out_1922727016412823836[203] = 0.0;
   out_1922727016412823836[204] = 0.0;
   out_1922727016412823836[205] = 0.0;
   out_1922727016412823836[206] = 0.0;
   out_1922727016412823836[207] = 0.0;
   out_1922727016412823836[208] = 0.0;
   out_1922727016412823836[209] = 1.0;
   out_1922727016412823836[210] = 0.0;
   out_1922727016412823836[211] = 0.0;
   out_1922727016412823836[212] = 0.0;
   out_1922727016412823836[213] = 0.0;
   out_1922727016412823836[214] = 0.0;
   out_1922727016412823836[215] = 0.0;
   out_1922727016412823836[216] = 0.0;
   out_1922727016412823836[217] = 0.0;
   out_1922727016412823836[218] = 0.0;
   out_1922727016412823836[219] = 0.0;
   out_1922727016412823836[220] = 0.0;
   out_1922727016412823836[221] = 0.0;
   out_1922727016412823836[222] = 0.0;
   out_1922727016412823836[223] = 0.0;
   out_1922727016412823836[224] = 0.0;
   out_1922727016412823836[225] = 0.0;
   out_1922727016412823836[226] = 0.0;
   out_1922727016412823836[227] = 0.0;
   out_1922727016412823836[228] = 1.0;
   out_1922727016412823836[229] = 0.0;
   out_1922727016412823836[230] = 0.0;
   out_1922727016412823836[231] = 0.0;
   out_1922727016412823836[232] = 0.0;
   out_1922727016412823836[233] = 0.0;
   out_1922727016412823836[234] = 0.0;
   out_1922727016412823836[235] = 0.0;
   out_1922727016412823836[236] = 0.0;
   out_1922727016412823836[237] = 0.0;
   out_1922727016412823836[238] = 0.0;
   out_1922727016412823836[239] = 0.0;
   out_1922727016412823836[240] = 0.0;
   out_1922727016412823836[241] = 0.0;
   out_1922727016412823836[242] = 0.0;
   out_1922727016412823836[243] = 0.0;
   out_1922727016412823836[244] = 0.0;
   out_1922727016412823836[245] = 0.0;
   out_1922727016412823836[246] = 0.0;
   out_1922727016412823836[247] = 1.0;
   out_1922727016412823836[248] = 0.0;
   out_1922727016412823836[249] = 0.0;
   out_1922727016412823836[250] = 0.0;
   out_1922727016412823836[251] = 0.0;
   out_1922727016412823836[252] = 0.0;
   out_1922727016412823836[253] = 0.0;
   out_1922727016412823836[254] = 0.0;
   out_1922727016412823836[255] = 0.0;
   out_1922727016412823836[256] = 0.0;
   out_1922727016412823836[257] = 0.0;
   out_1922727016412823836[258] = 0.0;
   out_1922727016412823836[259] = 0.0;
   out_1922727016412823836[260] = 0.0;
   out_1922727016412823836[261] = 0.0;
   out_1922727016412823836[262] = 0.0;
   out_1922727016412823836[263] = 0.0;
   out_1922727016412823836[264] = 0.0;
   out_1922727016412823836[265] = 0.0;
   out_1922727016412823836[266] = 1.0;
   out_1922727016412823836[267] = 0.0;
   out_1922727016412823836[268] = 0.0;
   out_1922727016412823836[269] = 0.0;
   out_1922727016412823836[270] = 0.0;
   out_1922727016412823836[271] = 0.0;
   out_1922727016412823836[272] = 0.0;
   out_1922727016412823836[273] = 0.0;
   out_1922727016412823836[274] = 0.0;
   out_1922727016412823836[275] = 0.0;
   out_1922727016412823836[276] = 0.0;
   out_1922727016412823836[277] = 0.0;
   out_1922727016412823836[278] = 0.0;
   out_1922727016412823836[279] = 0.0;
   out_1922727016412823836[280] = 0.0;
   out_1922727016412823836[281] = 0.0;
   out_1922727016412823836[282] = 0.0;
   out_1922727016412823836[283] = 0.0;
   out_1922727016412823836[284] = 0.0;
   out_1922727016412823836[285] = 1.0;
   out_1922727016412823836[286] = 0.0;
   out_1922727016412823836[287] = 0.0;
   out_1922727016412823836[288] = 0.0;
   out_1922727016412823836[289] = 0.0;
   out_1922727016412823836[290] = 0.0;
   out_1922727016412823836[291] = 0.0;
   out_1922727016412823836[292] = 0.0;
   out_1922727016412823836[293] = 0.0;
   out_1922727016412823836[294] = 0.0;
   out_1922727016412823836[295] = 0.0;
   out_1922727016412823836[296] = 0.0;
   out_1922727016412823836[297] = 0.0;
   out_1922727016412823836[298] = 0.0;
   out_1922727016412823836[299] = 0.0;
   out_1922727016412823836[300] = 0.0;
   out_1922727016412823836[301] = 0.0;
   out_1922727016412823836[302] = 0.0;
   out_1922727016412823836[303] = 0.0;
   out_1922727016412823836[304] = 1.0;
   out_1922727016412823836[305] = 0.0;
   out_1922727016412823836[306] = 0.0;
   out_1922727016412823836[307] = 0.0;
   out_1922727016412823836[308] = 0.0;
   out_1922727016412823836[309] = 0.0;
   out_1922727016412823836[310] = 0.0;
   out_1922727016412823836[311] = 0.0;
   out_1922727016412823836[312] = 0.0;
   out_1922727016412823836[313] = 0.0;
   out_1922727016412823836[314] = 0.0;
   out_1922727016412823836[315] = 0.0;
   out_1922727016412823836[316] = 0.0;
   out_1922727016412823836[317] = 0.0;
   out_1922727016412823836[318] = 0.0;
   out_1922727016412823836[319] = 0.0;
   out_1922727016412823836[320] = 0.0;
   out_1922727016412823836[321] = 0.0;
   out_1922727016412823836[322] = 0.0;
   out_1922727016412823836[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_4569588112805388687) {
   out_4569588112805388687[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_4569588112805388687[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_4569588112805388687[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_4569588112805388687[3] = dt*state[12] + state[3];
   out_4569588112805388687[4] = dt*state[13] + state[4];
   out_4569588112805388687[5] = dt*state[14] + state[5];
   out_4569588112805388687[6] = state[6];
   out_4569588112805388687[7] = state[7];
   out_4569588112805388687[8] = state[8];
   out_4569588112805388687[9] = state[9];
   out_4569588112805388687[10] = state[10];
   out_4569588112805388687[11] = state[11];
   out_4569588112805388687[12] = state[12];
   out_4569588112805388687[13] = state[13];
   out_4569588112805388687[14] = state[14];
   out_4569588112805388687[15] = state[15];
   out_4569588112805388687[16] = state[16];
   out_4569588112805388687[17] = state[17];
}
void F_fun(double *state, double dt, double *out_4952891530857479207) {
   out_4952891530857479207[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4952891530857479207[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4952891530857479207[2] = 0;
   out_4952891530857479207[3] = 0;
   out_4952891530857479207[4] = 0;
   out_4952891530857479207[5] = 0;
   out_4952891530857479207[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4952891530857479207[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4952891530857479207[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4952891530857479207[9] = 0;
   out_4952891530857479207[10] = 0;
   out_4952891530857479207[11] = 0;
   out_4952891530857479207[12] = 0;
   out_4952891530857479207[13] = 0;
   out_4952891530857479207[14] = 0;
   out_4952891530857479207[15] = 0;
   out_4952891530857479207[16] = 0;
   out_4952891530857479207[17] = 0;
   out_4952891530857479207[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4952891530857479207[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4952891530857479207[20] = 0;
   out_4952891530857479207[21] = 0;
   out_4952891530857479207[22] = 0;
   out_4952891530857479207[23] = 0;
   out_4952891530857479207[24] = 0;
   out_4952891530857479207[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4952891530857479207[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4952891530857479207[27] = 0;
   out_4952891530857479207[28] = 0;
   out_4952891530857479207[29] = 0;
   out_4952891530857479207[30] = 0;
   out_4952891530857479207[31] = 0;
   out_4952891530857479207[32] = 0;
   out_4952891530857479207[33] = 0;
   out_4952891530857479207[34] = 0;
   out_4952891530857479207[35] = 0;
   out_4952891530857479207[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4952891530857479207[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4952891530857479207[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4952891530857479207[39] = 0;
   out_4952891530857479207[40] = 0;
   out_4952891530857479207[41] = 0;
   out_4952891530857479207[42] = 0;
   out_4952891530857479207[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4952891530857479207[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4952891530857479207[45] = 0;
   out_4952891530857479207[46] = 0;
   out_4952891530857479207[47] = 0;
   out_4952891530857479207[48] = 0;
   out_4952891530857479207[49] = 0;
   out_4952891530857479207[50] = 0;
   out_4952891530857479207[51] = 0;
   out_4952891530857479207[52] = 0;
   out_4952891530857479207[53] = 0;
   out_4952891530857479207[54] = 0;
   out_4952891530857479207[55] = 0;
   out_4952891530857479207[56] = 0;
   out_4952891530857479207[57] = 1;
   out_4952891530857479207[58] = 0;
   out_4952891530857479207[59] = 0;
   out_4952891530857479207[60] = 0;
   out_4952891530857479207[61] = 0;
   out_4952891530857479207[62] = 0;
   out_4952891530857479207[63] = 0;
   out_4952891530857479207[64] = 0;
   out_4952891530857479207[65] = 0;
   out_4952891530857479207[66] = dt;
   out_4952891530857479207[67] = 0;
   out_4952891530857479207[68] = 0;
   out_4952891530857479207[69] = 0;
   out_4952891530857479207[70] = 0;
   out_4952891530857479207[71] = 0;
   out_4952891530857479207[72] = 0;
   out_4952891530857479207[73] = 0;
   out_4952891530857479207[74] = 0;
   out_4952891530857479207[75] = 0;
   out_4952891530857479207[76] = 1;
   out_4952891530857479207[77] = 0;
   out_4952891530857479207[78] = 0;
   out_4952891530857479207[79] = 0;
   out_4952891530857479207[80] = 0;
   out_4952891530857479207[81] = 0;
   out_4952891530857479207[82] = 0;
   out_4952891530857479207[83] = 0;
   out_4952891530857479207[84] = 0;
   out_4952891530857479207[85] = dt;
   out_4952891530857479207[86] = 0;
   out_4952891530857479207[87] = 0;
   out_4952891530857479207[88] = 0;
   out_4952891530857479207[89] = 0;
   out_4952891530857479207[90] = 0;
   out_4952891530857479207[91] = 0;
   out_4952891530857479207[92] = 0;
   out_4952891530857479207[93] = 0;
   out_4952891530857479207[94] = 0;
   out_4952891530857479207[95] = 1;
   out_4952891530857479207[96] = 0;
   out_4952891530857479207[97] = 0;
   out_4952891530857479207[98] = 0;
   out_4952891530857479207[99] = 0;
   out_4952891530857479207[100] = 0;
   out_4952891530857479207[101] = 0;
   out_4952891530857479207[102] = 0;
   out_4952891530857479207[103] = 0;
   out_4952891530857479207[104] = dt;
   out_4952891530857479207[105] = 0;
   out_4952891530857479207[106] = 0;
   out_4952891530857479207[107] = 0;
   out_4952891530857479207[108] = 0;
   out_4952891530857479207[109] = 0;
   out_4952891530857479207[110] = 0;
   out_4952891530857479207[111] = 0;
   out_4952891530857479207[112] = 0;
   out_4952891530857479207[113] = 0;
   out_4952891530857479207[114] = 1;
   out_4952891530857479207[115] = 0;
   out_4952891530857479207[116] = 0;
   out_4952891530857479207[117] = 0;
   out_4952891530857479207[118] = 0;
   out_4952891530857479207[119] = 0;
   out_4952891530857479207[120] = 0;
   out_4952891530857479207[121] = 0;
   out_4952891530857479207[122] = 0;
   out_4952891530857479207[123] = 0;
   out_4952891530857479207[124] = 0;
   out_4952891530857479207[125] = 0;
   out_4952891530857479207[126] = 0;
   out_4952891530857479207[127] = 0;
   out_4952891530857479207[128] = 0;
   out_4952891530857479207[129] = 0;
   out_4952891530857479207[130] = 0;
   out_4952891530857479207[131] = 0;
   out_4952891530857479207[132] = 0;
   out_4952891530857479207[133] = 1;
   out_4952891530857479207[134] = 0;
   out_4952891530857479207[135] = 0;
   out_4952891530857479207[136] = 0;
   out_4952891530857479207[137] = 0;
   out_4952891530857479207[138] = 0;
   out_4952891530857479207[139] = 0;
   out_4952891530857479207[140] = 0;
   out_4952891530857479207[141] = 0;
   out_4952891530857479207[142] = 0;
   out_4952891530857479207[143] = 0;
   out_4952891530857479207[144] = 0;
   out_4952891530857479207[145] = 0;
   out_4952891530857479207[146] = 0;
   out_4952891530857479207[147] = 0;
   out_4952891530857479207[148] = 0;
   out_4952891530857479207[149] = 0;
   out_4952891530857479207[150] = 0;
   out_4952891530857479207[151] = 0;
   out_4952891530857479207[152] = 1;
   out_4952891530857479207[153] = 0;
   out_4952891530857479207[154] = 0;
   out_4952891530857479207[155] = 0;
   out_4952891530857479207[156] = 0;
   out_4952891530857479207[157] = 0;
   out_4952891530857479207[158] = 0;
   out_4952891530857479207[159] = 0;
   out_4952891530857479207[160] = 0;
   out_4952891530857479207[161] = 0;
   out_4952891530857479207[162] = 0;
   out_4952891530857479207[163] = 0;
   out_4952891530857479207[164] = 0;
   out_4952891530857479207[165] = 0;
   out_4952891530857479207[166] = 0;
   out_4952891530857479207[167] = 0;
   out_4952891530857479207[168] = 0;
   out_4952891530857479207[169] = 0;
   out_4952891530857479207[170] = 0;
   out_4952891530857479207[171] = 1;
   out_4952891530857479207[172] = 0;
   out_4952891530857479207[173] = 0;
   out_4952891530857479207[174] = 0;
   out_4952891530857479207[175] = 0;
   out_4952891530857479207[176] = 0;
   out_4952891530857479207[177] = 0;
   out_4952891530857479207[178] = 0;
   out_4952891530857479207[179] = 0;
   out_4952891530857479207[180] = 0;
   out_4952891530857479207[181] = 0;
   out_4952891530857479207[182] = 0;
   out_4952891530857479207[183] = 0;
   out_4952891530857479207[184] = 0;
   out_4952891530857479207[185] = 0;
   out_4952891530857479207[186] = 0;
   out_4952891530857479207[187] = 0;
   out_4952891530857479207[188] = 0;
   out_4952891530857479207[189] = 0;
   out_4952891530857479207[190] = 1;
   out_4952891530857479207[191] = 0;
   out_4952891530857479207[192] = 0;
   out_4952891530857479207[193] = 0;
   out_4952891530857479207[194] = 0;
   out_4952891530857479207[195] = 0;
   out_4952891530857479207[196] = 0;
   out_4952891530857479207[197] = 0;
   out_4952891530857479207[198] = 0;
   out_4952891530857479207[199] = 0;
   out_4952891530857479207[200] = 0;
   out_4952891530857479207[201] = 0;
   out_4952891530857479207[202] = 0;
   out_4952891530857479207[203] = 0;
   out_4952891530857479207[204] = 0;
   out_4952891530857479207[205] = 0;
   out_4952891530857479207[206] = 0;
   out_4952891530857479207[207] = 0;
   out_4952891530857479207[208] = 0;
   out_4952891530857479207[209] = 1;
   out_4952891530857479207[210] = 0;
   out_4952891530857479207[211] = 0;
   out_4952891530857479207[212] = 0;
   out_4952891530857479207[213] = 0;
   out_4952891530857479207[214] = 0;
   out_4952891530857479207[215] = 0;
   out_4952891530857479207[216] = 0;
   out_4952891530857479207[217] = 0;
   out_4952891530857479207[218] = 0;
   out_4952891530857479207[219] = 0;
   out_4952891530857479207[220] = 0;
   out_4952891530857479207[221] = 0;
   out_4952891530857479207[222] = 0;
   out_4952891530857479207[223] = 0;
   out_4952891530857479207[224] = 0;
   out_4952891530857479207[225] = 0;
   out_4952891530857479207[226] = 0;
   out_4952891530857479207[227] = 0;
   out_4952891530857479207[228] = 1;
   out_4952891530857479207[229] = 0;
   out_4952891530857479207[230] = 0;
   out_4952891530857479207[231] = 0;
   out_4952891530857479207[232] = 0;
   out_4952891530857479207[233] = 0;
   out_4952891530857479207[234] = 0;
   out_4952891530857479207[235] = 0;
   out_4952891530857479207[236] = 0;
   out_4952891530857479207[237] = 0;
   out_4952891530857479207[238] = 0;
   out_4952891530857479207[239] = 0;
   out_4952891530857479207[240] = 0;
   out_4952891530857479207[241] = 0;
   out_4952891530857479207[242] = 0;
   out_4952891530857479207[243] = 0;
   out_4952891530857479207[244] = 0;
   out_4952891530857479207[245] = 0;
   out_4952891530857479207[246] = 0;
   out_4952891530857479207[247] = 1;
   out_4952891530857479207[248] = 0;
   out_4952891530857479207[249] = 0;
   out_4952891530857479207[250] = 0;
   out_4952891530857479207[251] = 0;
   out_4952891530857479207[252] = 0;
   out_4952891530857479207[253] = 0;
   out_4952891530857479207[254] = 0;
   out_4952891530857479207[255] = 0;
   out_4952891530857479207[256] = 0;
   out_4952891530857479207[257] = 0;
   out_4952891530857479207[258] = 0;
   out_4952891530857479207[259] = 0;
   out_4952891530857479207[260] = 0;
   out_4952891530857479207[261] = 0;
   out_4952891530857479207[262] = 0;
   out_4952891530857479207[263] = 0;
   out_4952891530857479207[264] = 0;
   out_4952891530857479207[265] = 0;
   out_4952891530857479207[266] = 1;
   out_4952891530857479207[267] = 0;
   out_4952891530857479207[268] = 0;
   out_4952891530857479207[269] = 0;
   out_4952891530857479207[270] = 0;
   out_4952891530857479207[271] = 0;
   out_4952891530857479207[272] = 0;
   out_4952891530857479207[273] = 0;
   out_4952891530857479207[274] = 0;
   out_4952891530857479207[275] = 0;
   out_4952891530857479207[276] = 0;
   out_4952891530857479207[277] = 0;
   out_4952891530857479207[278] = 0;
   out_4952891530857479207[279] = 0;
   out_4952891530857479207[280] = 0;
   out_4952891530857479207[281] = 0;
   out_4952891530857479207[282] = 0;
   out_4952891530857479207[283] = 0;
   out_4952891530857479207[284] = 0;
   out_4952891530857479207[285] = 1;
   out_4952891530857479207[286] = 0;
   out_4952891530857479207[287] = 0;
   out_4952891530857479207[288] = 0;
   out_4952891530857479207[289] = 0;
   out_4952891530857479207[290] = 0;
   out_4952891530857479207[291] = 0;
   out_4952891530857479207[292] = 0;
   out_4952891530857479207[293] = 0;
   out_4952891530857479207[294] = 0;
   out_4952891530857479207[295] = 0;
   out_4952891530857479207[296] = 0;
   out_4952891530857479207[297] = 0;
   out_4952891530857479207[298] = 0;
   out_4952891530857479207[299] = 0;
   out_4952891530857479207[300] = 0;
   out_4952891530857479207[301] = 0;
   out_4952891530857479207[302] = 0;
   out_4952891530857479207[303] = 0;
   out_4952891530857479207[304] = 1;
   out_4952891530857479207[305] = 0;
   out_4952891530857479207[306] = 0;
   out_4952891530857479207[307] = 0;
   out_4952891530857479207[308] = 0;
   out_4952891530857479207[309] = 0;
   out_4952891530857479207[310] = 0;
   out_4952891530857479207[311] = 0;
   out_4952891530857479207[312] = 0;
   out_4952891530857479207[313] = 0;
   out_4952891530857479207[314] = 0;
   out_4952891530857479207[315] = 0;
   out_4952891530857479207[316] = 0;
   out_4952891530857479207[317] = 0;
   out_4952891530857479207[318] = 0;
   out_4952891530857479207[319] = 0;
   out_4952891530857479207[320] = 0;
   out_4952891530857479207[321] = 0;
   out_4952891530857479207[322] = 0;
   out_4952891530857479207[323] = 1;
}
void h_4(double *state, double *unused, double *out_8707903708277267811) {
   out_8707903708277267811[0] = state[6] + state[9];
   out_8707903708277267811[1] = state[7] + state[10];
   out_8707903708277267811[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_7075245401451904571) {
   out_7075245401451904571[0] = 0;
   out_7075245401451904571[1] = 0;
   out_7075245401451904571[2] = 0;
   out_7075245401451904571[3] = 0;
   out_7075245401451904571[4] = 0;
   out_7075245401451904571[5] = 0;
   out_7075245401451904571[6] = 1;
   out_7075245401451904571[7] = 0;
   out_7075245401451904571[8] = 0;
   out_7075245401451904571[9] = 1;
   out_7075245401451904571[10] = 0;
   out_7075245401451904571[11] = 0;
   out_7075245401451904571[12] = 0;
   out_7075245401451904571[13] = 0;
   out_7075245401451904571[14] = 0;
   out_7075245401451904571[15] = 0;
   out_7075245401451904571[16] = 0;
   out_7075245401451904571[17] = 0;
   out_7075245401451904571[18] = 0;
   out_7075245401451904571[19] = 0;
   out_7075245401451904571[20] = 0;
   out_7075245401451904571[21] = 0;
   out_7075245401451904571[22] = 0;
   out_7075245401451904571[23] = 0;
   out_7075245401451904571[24] = 0;
   out_7075245401451904571[25] = 1;
   out_7075245401451904571[26] = 0;
   out_7075245401451904571[27] = 0;
   out_7075245401451904571[28] = 1;
   out_7075245401451904571[29] = 0;
   out_7075245401451904571[30] = 0;
   out_7075245401451904571[31] = 0;
   out_7075245401451904571[32] = 0;
   out_7075245401451904571[33] = 0;
   out_7075245401451904571[34] = 0;
   out_7075245401451904571[35] = 0;
   out_7075245401451904571[36] = 0;
   out_7075245401451904571[37] = 0;
   out_7075245401451904571[38] = 0;
   out_7075245401451904571[39] = 0;
   out_7075245401451904571[40] = 0;
   out_7075245401451904571[41] = 0;
   out_7075245401451904571[42] = 0;
   out_7075245401451904571[43] = 0;
   out_7075245401451904571[44] = 1;
   out_7075245401451904571[45] = 0;
   out_7075245401451904571[46] = 0;
   out_7075245401451904571[47] = 1;
   out_7075245401451904571[48] = 0;
   out_7075245401451904571[49] = 0;
   out_7075245401451904571[50] = 0;
   out_7075245401451904571[51] = 0;
   out_7075245401451904571[52] = 0;
   out_7075245401451904571[53] = 0;
}
void h_10(double *state, double *unused, double *out_5731269717084300037) {
   out_5731269717084300037[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_5731269717084300037[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_5731269717084300037[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_7323133101098852149) {
   out_7323133101098852149[0] = 0;
   out_7323133101098852149[1] = 9.8100000000000005*cos(state[1]);
   out_7323133101098852149[2] = 0;
   out_7323133101098852149[3] = 0;
   out_7323133101098852149[4] = -state[8];
   out_7323133101098852149[5] = state[7];
   out_7323133101098852149[6] = 0;
   out_7323133101098852149[7] = state[5];
   out_7323133101098852149[8] = -state[4];
   out_7323133101098852149[9] = 0;
   out_7323133101098852149[10] = 0;
   out_7323133101098852149[11] = 0;
   out_7323133101098852149[12] = 1;
   out_7323133101098852149[13] = 0;
   out_7323133101098852149[14] = 0;
   out_7323133101098852149[15] = 1;
   out_7323133101098852149[16] = 0;
   out_7323133101098852149[17] = 0;
   out_7323133101098852149[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_7323133101098852149[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_7323133101098852149[20] = 0;
   out_7323133101098852149[21] = state[8];
   out_7323133101098852149[22] = 0;
   out_7323133101098852149[23] = -state[6];
   out_7323133101098852149[24] = -state[5];
   out_7323133101098852149[25] = 0;
   out_7323133101098852149[26] = state[3];
   out_7323133101098852149[27] = 0;
   out_7323133101098852149[28] = 0;
   out_7323133101098852149[29] = 0;
   out_7323133101098852149[30] = 0;
   out_7323133101098852149[31] = 1;
   out_7323133101098852149[32] = 0;
   out_7323133101098852149[33] = 0;
   out_7323133101098852149[34] = 1;
   out_7323133101098852149[35] = 0;
   out_7323133101098852149[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_7323133101098852149[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_7323133101098852149[38] = 0;
   out_7323133101098852149[39] = -state[7];
   out_7323133101098852149[40] = state[6];
   out_7323133101098852149[41] = 0;
   out_7323133101098852149[42] = state[4];
   out_7323133101098852149[43] = -state[3];
   out_7323133101098852149[44] = 0;
   out_7323133101098852149[45] = 0;
   out_7323133101098852149[46] = 0;
   out_7323133101098852149[47] = 0;
   out_7323133101098852149[48] = 0;
   out_7323133101098852149[49] = 0;
   out_7323133101098852149[50] = 1;
   out_7323133101098852149[51] = 0;
   out_7323133101098852149[52] = 0;
   out_7323133101098852149[53] = 1;
}
void h_13(double *state, double *unused, double *out_2954187921383884135) {
   out_2954187921383884135[0] = state[3];
   out_2954187921383884135[1] = state[4];
   out_2954187921383884135[2] = state[5];
}
void H_13(double *state, double *unused, double *out_8159224846925314244) {
   out_8159224846925314244[0] = 0;
   out_8159224846925314244[1] = 0;
   out_8159224846925314244[2] = 0;
   out_8159224846925314244[3] = 1;
   out_8159224846925314244[4] = 0;
   out_8159224846925314244[5] = 0;
   out_8159224846925314244[6] = 0;
   out_8159224846925314244[7] = 0;
   out_8159224846925314244[8] = 0;
   out_8159224846925314244[9] = 0;
   out_8159224846925314244[10] = 0;
   out_8159224846925314244[11] = 0;
   out_8159224846925314244[12] = 0;
   out_8159224846925314244[13] = 0;
   out_8159224846925314244[14] = 0;
   out_8159224846925314244[15] = 0;
   out_8159224846925314244[16] = 0;
   out_8159224846925314244[17] = 0;
   out_8159224846925314244[18] = 0;
   out_8159224846925314244[19] = 0;
   out_8159224846925314244[20] = 0;
   out_8159224846925314244[21] = 0;
   out_8159224846925314244[22] = 1;
   out_8159224846925314244[23] = 0;
   out_8159224846925314244[24] = 0;
   out_8159224846925314244[25] = 0;
   out_8159224846925314244[26] = 0;
   out_8159224846925314244[27] = 0;
   out_8159224846925314244[28] = 0;
   out_8159224846925314244[29] = 0;
   out_8159224846925314244[30] = 0;
   out_8159224846925314244[31] = 0;
   out_8159224846925314244[32] = 0;
   out_8159224846925314244[33] = 0;
   out_8159224846925314244[34] = 0;
   out_8159224846925314244[35] = 0;
   out_8159224846925314244[36] = 0;
   out_8159224846925314244[37] = 0;
   out_8159224846925314244[38] = 0;
   out_8159224846925314244[39] = 0;
   out_8159224846925314244[40] = 0;
   out_8159224846925314244[41] = 1;
   out_8159224846925314244[42] = 0;
   out_8159224846925314244[43] = 0;
   out_8159224846925314244[44] = 0;
   out_8159224846925314244[45] = 0;
   out_8159224846925314244[46] = 0;
   out_8159224846925314244[47] = 0;
   out_8159224846925314244[48] = 0;
   out_8159224846925314244[49] = 0;
   out_8159224846925314244[50] = 0;
   out_8159224846925314244[51] = 0;
   out_8159224846925314244[52] = 0;
   out_8159224846925314244[53] = 0;
}
void h_14(double *state, double *unused, double *out_5785405054235647194) {
   out_5785405054235647194[0] = state[6];
   out_5785405054235647194[1] = state[7];
   out_5785405054235647194[2] = state[8];
}
void H_14(double *state, double *unused, double *out_7408257815918162516) {
   out_7408257815918162516[0] = 0;
   out_7408257815918162516[1] = 0;
   out_7408257815918162516[2] = 0;
   out_7408257815918162516[3] = 0;
   out_7408257815918162516[4] = 0;
   out_7408257815918162516[5] = 0;
   out_7408257815918162516[6] = 1;
   out_7408257815918162516[7] = 0;
   out_7408257815918162516[8] = 0;
   out_7408257815918162516[9] = 0;
   out_7408257815918162516[10] = 0;
   out_7408257815918162516[11] = 0;
   out_7408257815918162516[12] = 0;
   out_7408257815918162516[13] = 0;
   out_7408257815918162516[14] = 0;
   out_7408257815918162516[15] = 0;
   out_7408257815918162516[16] = 0;
   out_7408257815918162516[17] = 0;
   out_7408257815918162516[18] = 0;
   out_7408257815918162516[19] = 0;
   out_7408257815918162516[20] = 0;
   out_7408257815918162516[21] = 0;
   out_7408257815918162516[22] = 0;
   out_7408257815918162516[23] = 0;
   out_7408257815918162516[24] = 0;
   out_7408257815918162516[25] = 1;
   out_7408257815918162516[26] = 0;
   out_7408257815918162516[27] = 0;
   out_7408257815918162516[28] = 0;
   out_7408257815918162516[29] = 0;
   out_7408257815918162516[30] = 0;
   out_7408257815918162516[31] = 0;
   out_7408257815918162516[32] = 0;
   out_7408257815918162516[33] = 0;
   out_7408257815918162516[34] = 0;
   out_7408257815918162516[35] = 0;
   out_7408257815918162516[36] = 0;
   out_7408257815918162516[37] = 0;
   out_7408257815918162516[38] = 0;
   out_7408257815918162516[39] = 0;
   out_7408257815918162516[40] = 0;
   out_7408257815918162516[41] = 0;
   out_7408257815918162516[42] = 0;
   out_7408257815918162516[43] = 0;
   out_7408257815918162516[44] = 1;
   out_7408257815918162516[45] = 0;
   out_7408257815918162516[46] = 0;
   out_7408257815918162516[47] = 0;
   out_7408257815918162516[48] = 0;
   out_7408257815918162516[49] = 0;
   out_7408257815918162516[50] = 0;
   out_7408257815918162516[51] = 0;
   out_7408257815918162516[52] = 0;
   out_7408257815918162516[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_8008231329674625977) {
  err_fun(nom_x, delta_x, out_8008231329674625977);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_9148252894232034006) {
  inv_err_fun(nom_x, true_x, out_9148252894232034006);
}
void pose_H_mod_fun(double *state, double *out_1922727016412823836) {
  H_mod_fun(state, out_1922727016412823836);
}
void pose_f_fun(double *state, double dt, double *out_4569588112805388687) {
  f_fun(state,  dt, out_4569588112805388687);
}
void pose_F_fun(double *state, double dt, double *out_4952891530857479207) {
  F_fun(state,  dt, out_4952891530857479207);
}
void pose_h_4(double *state, double *unused, double *out_8707903708277267811) {
  h_4(state, unused, out_8707903708277267811);
}
void pose_H_4(double *state, double *unused, double *out_7075245401451904571) {
  H_4(state, unused, out_7075245401451904571);
}
void pose_h_10(double *state, double *unused, double *out_5731269717084300037) {
  h_10(state, unused, out_5731269717084300037);
}
void pose_H_10(double *state, double *unused, double *out_7323133101098852149) {
  H_10(state, unused, out_7323133101098852149);
}
void pose_h_13(double *state, double *unused, double *out_2954187921383884135) {
  h_13(state, unused, out_2954187921383884135);
}
void pose_H_13(double *state, double *unused, double *out_8159224846925314244) {
  H_13(state, unused, out_8159224846925314244);
}
void pose_h_14(double *state, double *unused, double *out_5785405054235647194) {
  h_14(state, unused, out_5785405054235647194);
}
void pose_H_14(double *state, double *unused, double *out_7408257815918162516) {
  H_14(state, unused, out_7408257815918162516);
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
