#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_8229174994289557138) {
   out_8229174994289557138[0] = delta_x[0] + nom_x[0];
   out_8229174994289557138[1] = delta_x[1] + nom_x[1];
   out_8229174994289557138[2] = delta_x[2] + nom_x[2];
   out_8229174994289557138[3] = delta_x[3] + nom_x[3];
   out_8229174994289557138[4] = delta_x[4] + nom_x[4];
   out_8229174994289557138[5] = delta_x[5] + nom_x[5];
   out_8229174994289557138[6] = delta_x[6] + nom_x[6];
   out_8229174994289557138[7] = delta_x[7] + nom_x[7];
   out_8229174994289557138[8] = delta_x[8] + nom_x[8];
   out_8229174994289557138[9] = delta_x[9] + nom_x[9];
   out_8229174994289557138[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6217284632776378574) {
   out_6217284632776378574[0] = -nom_x[0] + true_x[0];
   out_6217284632776378574[1] = -nom_x[1] + true_x[1];
   out_6217284632776378574[2] = -nom_x[2] + true_x[2];
   out_6217284632776378574[3] = -nom_x[3] + true_x[3];
   out_6217284632776378574[4] = -nom_x[4] + true_x[4];
   out_6217284632776378574[5] = -nom_x[5] + true_x[5];
   out_6217284632776378574[6] = -nom_x[6] + true_x[6];
   out_6217284632776378574[7] = -nom_x[7] + true_x[7];
   out_6217284632776378574[8] = -nom_x[8] + true_x[8];
   out_6217284632776378574[9] = -nom_x[9] + true_x[9];
   out_6217284632776378574[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_5011568525599502586) {
   out_5011568525599502586[0] = 1.0;
   out_5011568525599502586[1] = 0;
   out_5011568525599502586[2] = 0;
   out_5011568525599502586[3] = 0;
   out_5011568525599502586[4] = 0;
   out_5011568525599502586[5] = 0;
   out_5011568525599502586[6] = 0;
   out_5011568525599502586[7] = 0;
   out_5011568525599502586[8] = 0;
   out_5011568525599502586[9] = 0;
   out_5011568525599502586[10] = 0;
   out_5011568525599502586[11] = 0;
   out_5011568525599502586[12] = 1.0;
   out_5011568525599502586[13] = 0;
   out_5011568525599502586[14] = 0;
   out_5011568525599502586[15] = 0;
   out_5011568525599502586[16] = 0;
   out_5011568525599502586[17] = 0;
   out_5011568525599502586[18] = 0;
   out_5011568525599502586[19] = 0;
   out_5011568525599502586[20] = 0;
   out_5011568525599502586[21] = 0;
   out_5011568525599502586[22] = 0;
   out_5011568525599502586[23] = 0;
   out_5011568525599502586[24] = 1.0;
   out_5011568525599502586[25] = 0;
   out_5011568525599502586[26] = 0;
   out_5011568525599502586[27] = 0;
   out_5011568525599502586[28] = 0;
   out_5011568525599502586[29] = 0;
   out_5011568525599502586[30] = 0;
   out_5011568525599502586[31] = 0;
   out_5011568525599502586[32] = 0;
   out_5011568525599502586[33] = 0;
   out_5011568525599502586[34] = 0;
   out_5011568525599502586[35] = 0;
   out_5011568525599502586[36] = 1.0;
   out_5011568525599502586[37] = 0;
   out_5011568525599502586[38] = 0;
   out_5011568525599502586[39] = 0;
   out_5011568525599502586[40] = 0;
   out_5011568525599502586[41] = 0;
   out_5011568525599502586[42] = 0;
   out_5011568525599502586[43] = 0;
   out_5011568525599502586[44] = 0;
   out_5011568525599502586[45] = 0;
   out_5011568525599502586[46] = 0;
   out_5011568525599502586[47] = 0;
   out_5011568525599502586[48] = 1.0;
   out_5011568525599502586[49] = 0;
   out_5011568525599502586[50] = 0;
   out_5011568525599502586[51] = 0;
   out_5011568525599502586[52] = 0;
   out_5011568525599502586[53] = 0;
   out_5011568525599502586[54] = 0;
   out_5011568525599502586[55] = 0;
   out_5011568525599502586[56] = 0;
   out_5011568525599502586[57] = 0;
   out_5011568525599502586[58] = 0;
   out_5011568525599502586[59] = 0;
   out_5011568525599502586[60] = 1.0;
   out_5011568525599502586[61] = 0;
   out_5011568525599502586[62] = 0;
   out_5011568525599502586[63] = 0;
   out_5011568525599502586[64] = 0;
   out_5011568525599502586[65] = 0;
   out_5011568525599502586[66] = 0;
   out_5011568525599502586[67] = 0;
   out_5011568525599502586[68] = 0;
   out_5011568525599502586[69] = 0;
   out_5011568525599502586[70] = 0;
   out_5011568525599502586[71] = 0;
   out_5011568525599502586[72] = 1.0;
   out_5011568525599502586[73] = 0;
   out_5011568525599502586[74] = 0;
   out_5011568525599502586[75] = 0;
   out_5011568525599502586[76] = 0;
   out_5011568525599502586[77] = 0;
   out_5011568525599502586[78] = 0;
   out_5011568525599502586[79] = 0;
   out_5011568525599502586[80] = 0;
   out_5011568525599502586[81] = 0;
   out_5011568525599502586[82] = 0;
   out_5011568525599502586[83] = 0;
   out_5011568525599502586[84] = 1.0;
   out_5011568525599502586[85] = 0;
   out_5011568525599502586[86] = 0;
   out_5011568525599502586[87] = 0;
   out_5011568525599502586[88] = 0;
   out_5011568525599502586[89] = 0;
   out_5011568525599502586[90] = 0;
   out_5011568525599502586[91] = 0;
   out_5011568525599502586[92] = 0;
   out_5011568525599502586[93] = 0;
   out_5011568525599502586[94] = 0;
   out_5011568525599502586[95] = 0;
   out_5011568525599502586[96] = 1.0;
   out_5011568525599502586[97] = 0;
   out_5011568525599502586[98] = 0;
   out_5011568525599502586[99] = 0;
   out_5011568525599502586[100] = 0;
   out_5011568525599502586[101] = 0;
   out_5011568525599502586[102] = 0;
   out_5011568525599502586[103] = 0;
   out_5011568525599502586[104] = 0;
   out_5011568525599502586[105] = 0;
   out_5011568525599502586[106] = 0;
   out_5011568525599502586[107] = 0;
   out_5011568525599502586[108] = 1.0;
   out_5011568525599502586[109] = 0;
   out_5011568525599502586[110] = 0;
   out_5011568525599502586[111] = 0;
   out_5011568525599502586[112] = 0;
   out_5011568525599502586[113] = 0;
   out_5011568525599502586[114] = 0;
   out_5011568525599502586[115] = 0;
   out_5011568525599502586[116] = 0;
   out_5011568525599502586[117] = 0;
   out_5011568525599502586[118] = 0;
   out_5011568525599502586[119] = 0;
   out_5011568525599502586[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_3473355585470786110) {
   out_3473355585470786110[0] = dt*state[3] + state[0];
   out_3473355585470786110[1] = dt*state[4] + state[1];
   out_3473355585470786110[2] = dt*state[5] + state[2];
   out_3473355585470786110[3] = state[3];
   out_3473355585470786110[4] = state[4];
   out_3473355585470786110[5] = state[5];
   out_3473355585470786110[6] = dt*state[7] + state[6];
   out_3473355585470786110[7] = dt*state[8] + state[7];
   out_3473355585470786110[8] = state[8];
   out_3473355585470786110[9] = state[9];
   out_3473355585470786110[10] = state[10];
}
void F_fun(double *state, double dt, double *out_5105028025396229212) {
   out_5105028025396229212[0] = 1;
   out_5105028025396229212[1] = 0;
   out_5105028025396229212[2] = 0;
   out_5105028025396229212[3] = dt;
   out_5105028025396229212[4] = 0;
   out_5105028025396229212[5] = 0;
   out_5105028025396229212[6] = 0;
   out_5105028025396229212[7] = 0;
   out_5105028025396229212[8] = 0;
   out_5105028025396229212[9] = 0;
   out_5105028025396229212[10] = 0;
   out_5105028025396229212[11] = 0;
   out_5105028025396229212[12] = 1;
   out_5105028025396229212[13] = 0;
   out_5105028025396229212[14] = 0;
   out_5105028025396229212[15] = dt;
   out_5105028025396229212[16] = 0;
   out_5105028025396229212[17] = 0;
   out_5105028025396229212[18] = 0;
   out_5105028025396229212[19] = 0;
   out_5105028025396229212[20] = 0;
   out_5105028025396229212[21] = 0;
   out_5105028025396229212[22] = 0;
   out_5105028025396229212[23] = 0;
   out_5105028025396229212[24] = 1;
   out_5105028025396229212[25] = 0;
   out_5105028025396229212[26] = 0;
   out_5105028025396229212[27] = dt;
   out_5105028025396229212[28] = 0;
   out_5105028025396229212[29] = 0;
   out_5105028025396229212[30] = 0;
   out_5105028025396229212[31] = 0;
   out_5105028025396229212[32] = 0;
   out_5105028025396229212[33] = 0;
   out_5105028025396229212[34] = 0;
   out_5105028025396229212[35] = 0;
   out_5105028025396229212[36] = 1;
   out_5105028025396229212[37] = 0;
   out_5105028025396229212[38] = 0;
   out_5105028025396229212[39] = 0;
   out_5105028025396229212[40] = 0;
   out_5105028025396229212[41] = 0;
   out_5105028025396229212[42] = 0;
   out_5105028025396229212[43] = 0;
   out_5105028025396229212[44] = 0;
   out_5105028025396229212[45] = 0;
   out_5105028025396229212[46] = 0;
   out_5105028025396229212[47] = 0;
   out_5105028025396229212[48] = 1;
   out_5105028025396229212[49] = 0;
   out_5105028025396229212[50] = 0;
   out_5105028025396229212[51] = 0;
   out_5105028025396229212[52] = 0;
   out_5105028025396229212[53] = 0;
   out_5105028025396229212[54] = 0;
   out_5105028025396229212[55] = 0;
   out_5105028025396229212[56] = 0;
   out_5105028025396229212[57] = 0;
   out_5105028025396229212[58] = 0;
   out_5105028025396229212[59] = 0;
   out_5105028025396229212[60] = 1;
   out_5105028025396229212[61] = 0;
   out_5105028025396229212[62] = 0;
   out_5105028025396229212[63] = 0;
   out_5105028025396229212[64] = 0;
   out_5105028025396229212[65] = 0;
   out_5105028025396229212[66] = 0;
   out_5105028025396229212[67] = 0;
   out_5105028025396229212[68] = 0;
   out_5105028025396229212[69] = 0;
   out_5105028025396229212[70] = 0;
   out_5105028025396229212[71] = 0;
   out_5105028025396229212[72] = 1;
   out_5105028025396229212[73] = dt;
   out_5105028025396229212[74] = 0;
   out_5105028025396229212[75] = 0;
   out_5105028025396229212[76] = 0;
   out_5105028025396229212[77] = 0;
   out_5105028025396229212[78] = 0;
   out_5105028025396229212[79] = 0;
   out_5105028025396229212[80] = 0;
   out_5105028025396229212[81] = 0;
   out_5105028025396229212[82] = 0;
   out_5105028025396229212[83] = 0;
   out_5105028025396229212[84] = 1;
   out_5105028025396229212[85] = dt;
   out_5105028025396229212[86] = 0;
   out_5105028025396229212[87] = 0;
   out_5105028025396229212[88] = 0;
   out_5105028025396229212[89] = 0;
   out_5105028025396229212[90] = 0;
   out_5105028025396229212[91] = 0;
   out_5105028025396229212[92] = 0;
   out_5105028025396229212[93] = 0;
   out_5105028025396229212[94] = 0;
   out_5105028025396229212[95] = 0;
   out_5105028025396229212[96] = 1;
   out_5105028025396229212[97] = 0;
   out_5105028025396229212[98] = 0;
   out_5105028025396229212[99] = 0;
   out_5105028025396229212[100] = 0;
   out_5105028025396229212[101] = 0;
   out_5105028025396229212[102] = 0;
   out_5105028025396229212[103] = 0;
   out_5105028025396229212[104] = 0;
   out_5105028025396229212[105] = 0;
   out_5105028025396229212[106] = 0;
   out_5105028025396229212[107] = 0;
   out_5105028025396229212[108] = 1;
   out_5105028025396229212[109] = 0;
   out_5105028025396229212[110] = 0;
   out_5105028025396229212[111] = 0;
   out_5105028025396229212[112] = 0;
   out_5105028025396229212[113] = 0;
   out_5105028025396229212[114] = 0;
   out_5105028025396229212[115] = 0;
   out_5105028025396229212[116] = 0;
   out_5105028025396229212[117] = 0;
   out_5105028025396229212[118] = 0;
   out_5105028025396229212[119] = 0;
   out_5105028025396229212[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_3612315771256631339) {
   out_3612315771256631339[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_3598249379002766159) {
   out_3598249379002766159[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3598249379002766159[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3598249379002766159[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3598249379002766159[3] = 0;
   out_3598249379002766159[4] = 0;
   out_3598249379002766159[5] = 0;
   out_3598249379002766159[6] = 1;
   out_3598249379002766159[7] = 0;
   out_3598249379002766159[8] = 0;
   out_3598249379002766159[9] = 0;
   out_3598249379002766159[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_583828302854464326) {
   out_583828302854464326[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_2729544730293299337) {
   out_2729544730293299337[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2729544730293299337[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2729544730293299337[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2729544730293299337[3] = 0;
   out_2729544730293299337[4] = 0;
   out_2729544730293299337[5] = 0;
   out_2729544730293299337[6] = 1;
   out_2729544730293299337[7] = 0;
   out_2729544730293299337[8] = 0;
   out_2729544730293299337[9] = 1;
   out_2729544730293299337[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_2465849935794505934) {
   out_2465849935794505934[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_1235251322282411141) {
   out_1235251322282411141[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1235251322282411141[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1235251322282411141[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1235251322282411141[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1235251322282411141[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1235251322282411141[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1235251322282411141[6] = 0;
   out_1235251322282411141[7] = 1;
   out_1235251322282411141[8] = 0;
   out_1235251322282411141[9] = 0;
   out_1235251322282411141[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_2465849935794505934) {
   out_2465849935794505934[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_1235251322282411141) {
   out_1235251322282411141[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1235251322282411141[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1235251322282411141[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1235251322282411141[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1235251322282411141[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1235251322282411141[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_1235251322282411141[6] = 0;
   out_1235251322282411141[7] = 1;
   out_1235251322282411141[8] = 0;
   out_1235251322282411141[9] = 0;
   out_1235251322282411141[10] = 0;
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

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_8229174994289557138) {
  err_fun(nom_x, delta_x, out_8229174994289557138);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_6217284632776378574) {
  inv_err_fun(nom_x, true_x, out_6217284632776378574);
}
void gnss_H_mod_fun(double *state, double *out_5011568525599502586) {
  H_mod_fun(state, out_5011568525599502586);
}
void gnss_f_fun(double *state, double dt, double *out_3473355585470786110) {
  f_fun(state,  dt, out_3473355585470786110);
}
void gnss_F_fun(double *state, double dt, double *out_5105028025396229212) {
  F_fun(state,  dt, out_5105028025396229212);
}
void gnss_h_6(double *state, double *sat_pos, double *out_3612315771256631339) {
  h_6(state, sat_pos, out_3612315771256631339);
}
void gnss_H_6(double *state, double *sat_pos, double *out_3598249379002766159) {
  H_6(state, sat_pos, out_3598249379002766159);
}
void gnss_h_20(double *state, double *sat_pos, double *out_583828302854464326) {
  h_20(state, sat_pos, out_583828302854464326);
}
void gnss_H_20(double *state, double *sat_pos, double *out_2729544730293299337) {
  H_20(state, sat_pos, out_2729544730293299337);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_2465849935794505934) {
  h_7(state, sat_pos_vel, out_2465849935794505934);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_1235251322282411141) {
  H_7(state, sat_pos_vel, out_1235251322282411141);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_2465849935794505934) {
  h_21(state, sat_pos_vel, out_2465849935794505934);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_1235251322282411141) {
  H_21(state, sat_pos_vel, out_1235251322282411141);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
