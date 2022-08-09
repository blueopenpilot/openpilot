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
 *                      Code generated with SymPy 1.10.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6435077088755969679) {
   out_6435077088755969679[0] = delta_x[0] + nom_x[0];
   out_6435077088755969679[1] = delta_x[1] + nom_x[1];
   out_6435077088755969679[2] = delta_x[2] + nom_x[2];
   out_6435077088755969679[3] = delta_x[3] + nom_x[3];
   out_6435077088755969679[4] = delta_x[4] + nom_x[4];
   out_6435077088755969679[5] = delta_x[5] + nom_x[5];
   out_6435077088755969679[6] = delta_x[6] + nom_x[6];
   out_6435077088755969679[7] = delta_x[7] + nom_x[7];
   out_6435077088755969679[8] = delta_x[8] + nom_x[8];
   out_6435077088755969679[9] = delta_x[9] + nom_x[9];
   out_6435077088755969679[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5461030829569705127) {
   out_5461030829569705127[0] = -nom_x[0] + true_x[0];
   out_5461030829569705127[1] = -nom_x[1] + true_x[1];
   out_5461030829569705127[2] = -nom_x[2] + true_x[2];
   out_5461030829569705127[3] = -nom_x[3] + true_x[3];
   out_5461030829569705127[4] = -nom_x[4] + true_x[4];
   out_5461030829569705127[5] = -nom_x[5] + true_x[5];
   out_5461030829569705127[6] = -nom_x[6] + true_x[6];
   out_5461030829569705127[7] = -nom_x[7] + true_x[7];
   out_5461030829569705127[8] = -nom_x[8] + true_x[8];
   out_5461030829569705127[9] = -nom_x[9] + true_x[9];
   out_5461030829569705127[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_1113439463888521147) {
   out_1113439463888521147[0] = 1.0;
   out_1113439463888521147[1] = 0;
   out_1113439463888521147[2] = 0;
   out_1113439463888521147[3] = 0;
   out_1113439463888521147[4] = 0;
   out_1113439463888521147[5] = 0;
   out_1113439463888521147[6] = 0;
   out_1113439463888521147[7] = 0;
   out_1113439463888521147[8] = 0;
   out_1113439463888521147[9] = 0;
   out_1113439463888521147[10] = 0;
   out_1113439463888521147[11] = 0;
   out_1113439463888521147[12] = 1.0;
   out_1113439463888521147[13] = 0;
   out_1113439463888521147[14] = 0;
   out_1113439463888521147[15] = 0;
   out_1113439463888521147[16] = 0;
   out_1113439463888521147[17] = 0;
   out_1113439463888521147[18] = 0;
   out_1113439463888521147[19] = 0;
   out_1113439463888521147[20] = 0;
   out_1113439463888521147[21] = 0;
   out_1113439463888521147[22] = 0;
   out_1113439463888521147[23] = 0;
   out_1113439463888521147[24] = 1.0;
   out_1113439463888521147[25] = 0;
   out_1113439463888521147[26] = 0;
   out_1113439463888521147[27] = 0;
   out_1113439463888521147[28] = 0;
   out_1113439463888521147[29] = 0;
   out_1113439463888521147[30] = 0;
   out_1113439463888521147[31] = 0;
   out_1113439463888521147[32] = 0;
   out_1113439463888521147[33] = 0;
   out_1113439463888521147[34] = 0;
   out_1113439463888521147[35] = 0;
   out_1113439463888521147[36] = 1.0;
   out_1113439463888521147[37] = 0;
   out_1113439463888521147[38] = 0;
   out_1113439463888521147[39] = 0;
   out_1113439463888521147[40] = 0;
   out_1113439463888521147[41] = 0;
   out_1113439463888521147[42] = 0;
   out_1113439463888521147[43] = 0;
   out_1113439463888521147[44] = 0;
   out_1113439463888521147[45] = 0;
   out_1113439463888521147[46] = 0;
   out_1113439463888521147[47] = 0;
   out_1113439463888521147[48] = 1.0;
   out_1113439463888521147[49] = 0;
   out_1113439463888521147[50] = 0;
   out_1113439463888521147[51] = 0;
   out_1113439463888521147[52] = 0;
   out_1113439463888521147[53] = 0;
   out_1113439463888521147[54] = 0;
   out_1113439463888521147[55] = 0;
   out_1113439463888521147[56] = 0;
   out_1113439463888521147[57] = 0;
   out_1113439463888521147[58] = 0;
   out_1113439463888521147[59] = 0;
   out_1113439463888521147[60] = 1.0;
   out_1113439463888521147[61] = 0;
   out_1113439463888521147[62] = 0;
   out_1113439463888521147[63] = 0;
   out_1113439463888521147[64] = 0;
   out_1113439463888521147[65] = 0;
   out_1113439463888521147[66] = 0;
   out_1113439463888521147[67] = 0;
   out_1113439463888521147[68] = 0;
   out_1113439463888521147[69] = 0;
   out_1113439463888521147[70] = 0;
   out_1113439463888521147[71] = 0;
   out_1113439463888521147[72] = 1.0;
   out_1113439463888521147[73] = 0;
   out_1113439463888521147[74] = 0;
   out_1113439463888521147[75] = 0;
   out_1113439463888521147[76] = 0;
   out_1113439463888521147[77] = 0;
   out_1113439463888521147[78] = 0;
   out_1113439463888521147[79] = 0;
   out_1113439463888521147[80] = 0;
   out_1113439463888521147[81] = 0;
   out_1113439463888521147[82] = 0;
   out_1113439463888521147[83] = 0;
   out_1113439463888521147[84] = 1.0;
   out_1113439463888521147[85] = 0;
   out_1113439463888521147[86] = 0;
   out_1113439463888521147[87] = 0;
   out_1113439463888521147[88] = 0;
   out_1113439463888521147[89] = 0;
   out_1113439463888521147[90] = 0;
   out_1113439463888521147[91] = 0;
   out_1113439463888521147[92] = 0;
   out_1113439463888521147[93] = 0;
   out_1113439463888521147[94] = 0;
   out_1113439463888521147[95] = 0;
   out_1113439463888521147[96] = 1.0;
   out_1113439463888521147[97] = 0;
   out_1113439463888521147[98] = 0;
   out_1113439463888521147[99] = 0;
   out_1113439463888521147[100] = 0;
   out_1113439463888521147[101] = 0;
   out_1113439463888521147[102] = 0;
   out_1113439463888521147[103] = 0;
   out_1113439463888521147[104] = 0;
   out_1113439463888521147[105] = 0;
   out_1113439463888521147[106] = 0;
   out_1113439463888521147[107] = 0;
   out_1113439463888521147[108] = 1.0;
   out_1113439463888521147[109] = 0;
   out_1113439463888521147[110] = 0;
   out_1113439463888521147[111] = 0;
   out_1113439463888521147[112] = 0;
   out_1113439463888521147[113] = 0;
   out_1113439463888521147[114] = 0;
   out_1113439463888521147[115] = 0;
   out_1113439463888521147[116] = 0;
   out_1113439463888521147[117] = 0;
   out_1113439463888521147[118] = 0;
   out_1113439463888521147[119] = 0;
   out_1113439463888521147[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_3853921857578862715) {
   out_3853921857578862715[0] = dt*state[3] + state[0];
   out_3853921857578862715[1] = dt*state[4] + state[1];
   out_3853921857578862715[2] = dt*state[5] + state[2];
   out_3853921857578862715[3] = state[3];
   out_3853921857578862715[4] = state[4];
   out_3853921857578862715[5] = state[5];
   out_3853921857578862715[6] = dt*state[7] + state[6];
   out_3853921857578862715[7] = dt*state[8] + state[7];
   out_3853921857578862715[8] = state[8];
   out_3853921857578862715[9] = state[9];
   out_3853921857578862715[10] = state[10];
}
void F_fun(double *state, double dt, double *out_1990023416032551193) {
   out_1990023416032551193[0] = 1;
   out_1990023416032551193[1] = 0;
   out_1990023416032551193[2] = 0;
   out_1990023416032551193[3] = dt;
   out_1990023416032551193[4] = 0;
   out_1990023416032551193[5] = 0;
   out_1990023416032551193[6] = 0;
   out_1990023416032551193[7] = 0;
   out_1990023416032551193[8] = 0;
   out_1990023416032551193[9] = 0;
   out_1990023416032551193[10] = 0;
   out_1990023416032551193[11] = 0;
   out_1990023416032551193[12] = 1;
   out_1990023416032551193[13] = 0;
   out_1990023416032551193[14] = 0;
   out_1990023416032551193[15] = dt;
   out_1990023416032551193[16] = 0;
   out_1990023416032551193[17] = 0;
   out_1990023416032551193[18] = 0;
   out_1990023416032551193[19] = 0;
   out_1990023416032551193[20] = 0;
   out_1990023416032551193[21] = 0;
   out_1990023416032551193[22] = 0;
   out_1990023416032551193[23] = 0;
   out_1990023416032551193[24] = 1;
   out_1990023416032551193[25] = 0;
   out_1990023416032551193[26] = 0;
   out_1990023416032551193[27] = dt;
   out_1990023416032551193[28] = 0;
   out_1990023416032551193[29] = 0;
   out_1990023416032551193[30] = 0;
   out_1990023416032551193[31] = 0;
   out_1990023416032551193[32] = 0;
   out_1990023416032551193[33] = 0;
   out_1990023416032551193[34] = 0;
   out_1990023416032551193[35] = 0;
   out_1990023416032551193[36] = 1;
   out_1990023416032551193[37] = 0;
   out_1990023416032551193[38] = 0;
   out_1990023416032551193[39] = 0;
   out_1990023416032551193[40] = 0;
   out_1990023416032551193[41] = 0;
   out_1990023416032551193[42] = 0;
   out_1990023416032551193[43] = 0;
   out_1990023416032551193[44] = 0;
   out_1990023416032551193[45] = 0;
   out_1990023416032551193[46] = 0;
   out_1990023416032551193[47] = 0;
   out_1990023416032551193[48] = 1;
   out_1990023416032551193[49] = 0;
   out_1990023416032551193[50] = 0;
   out_1990023416032551193[51] = 0;
   out_1990023416032551193[52] = 0;
   out_1990023416032551193[53] = 0;
   out_1990023416032551193[54] = 0;
   out_1990023416032551193[55] = 0;
   out_1990023416032551193[56] = 0;
   out_1990023416032551193[57] = 0;
   out_1990023416032551193[58] = 0;
   out_1990023416032551193[59] = 0;
   out_1990023416032551193[60] = 1;
   out_1990023416032551193[61] = 0;
   out_1990023416032551193[62] = 0;
   out_1990023416032551193[63] = 0;
   out_1990023416032551193[64] = 0;
   out_1990023416032551193[65] = 0;
   out_1990023416032551193[66] = 0;
   out_1990023416032551193[67] = 0;
   out_1990023416032551193[68] = 0;
   out_1990023416032551193[69] = 0;
   out_1990023416032551193[70] = 0;
   out_1990023416032551193[71] = 0;
   out_1990023416032551193[72] = 1;
   out_1990023416032551193[73] = dt;
   out_1990023416032551193[74] = 0;
   out_1990023416032551193[75] = 0;
   out_1990023416032551193[76] = 0;
   out_1990023416032551193[77] = 0;
   out_1990023416032551193[78] = 0;
   out_1990023416032551193[79] = 0;
   out_1990023416032551193[80] = 0;
   out_1990023416032551193[81] = 0;
   out_1990023416032551193[82] = 0;
   out_1990023416032551193[83] = 0;
   out_1990023416032551193[84] = 1;
   out_1990023416032551193[85] = dt;
   out_1990023416032551193[86] = 0;
   out_1990023416032551193[87] = 0;
   out_1990023416032551193[88] = 0;
   out_1990023416032551193[89] = 0;
   out_1990023416032551193[90] = 0;
   out_1990023416032551193[91] = 0;
   out_1990023416032551193[92] = 0;
   out_1990023416032551193[93] = 0;
   out_1990023416032551193[94] = 0;
   out_1990023416032551193[95] = 0;
   out_1990023416032551193[96] = 1;
   out_1990023416032551193[97] = 0;
   out_1990023416032551193[98] = 0;
   out_1990023416032551193[99] = 0;
   out_1990023416032551193[100] = 0;
   out_1990023416032551193[101] = 0;
   out_1990023416032551193[102] = 0;
   out_1990023416032551193[103] = 0;
   out_1990023416032551193[104] = 0;
   out_1990023416032551193[105] = 0;
   out_1990023416032551193[106] = 0;
   out_1990023416032551193[107] = 0;
   out_1990023416032551193[108] = 1;
   out_1990023416032551193[109] = 0;
   out_1990023416032551193[110] = 0;
   out_1990023416032551193[111] = 0;
   out_1990023416032551193[112] = 0;
   out_1990023416032551193[113] = 0;
   out_1990023416032551193[114] = 0;
   out_1990023416032551193[115] = 0;
   out_1990023416032551193[116] = 0;
   out_1990023416032551193[117] = 0;
   out_1990023416032551193[118] = 0;
   out_1990023416032551193[119] = 0;
   out_1990023416032551193[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_5372040911404388122) {
   out_5372040911404388122[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_1538307200137441102) {
   out_1538307200137441102[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1538307200137441102[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1538307200137441102[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1538307200137441102[3] = 0;
   out_1538307200137441102[4] = 0;
   out_1538307200137441102[5] = 0;
   out_1538307200137441102[6] = 1;
   out_1538307200137441102[7] = 0;
   out_1538307200137441102[8] = 0;
   out_1538307200137441102[9] = 0;
   out_1538307200137441102[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_5942081968181849174) {
   out_5942081968181849174[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_3657082352704797406) {
   out_3657082352704797406[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3657082352704797406[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3657082352704797406[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3657082352704797406[3] = 0;
   out_3657082352704797406[4] = 0;
   out_3657082352704797406[5] = 0;
   out_3657082352704797406[6] = 1;
   out_3657082352704797406[7] = 0;
   out_3657082352704797406[8] = 0;
   out_3657082352704797406[9] = 1;
   out_3657082352704797406[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_4161012093764852918) {
   out_4161012093764852918[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_3960385618526437650) {
   out_3960385618526437650[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3960385618526437650[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3960385618526437650[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3960385618526437650[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3960385618526437650[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3960385618526437650[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3960385618526437650[6] = 0;
   out_3960385618526437650[7] = 1;
   out_3960385618526437650[8] = 0;
   out_3960385618526437650[9] = 0;
   out_3960385618526437650[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_4161012093764852918) {
   out_4161012093764852918[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_3960385618526437650) {
   out_3960385618526437650[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3960385618526437650[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3960385618526437650[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3960385618526437650[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3960385618526437650[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3960385618526437650[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3960385618526437650[6] = 0;
   out_3960385618526437650[7] = 1;
   out_3960385618526437650[8] = 0;
   out_3960385618526437650[9] = 0;
   out_3960385618526437650[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_6435077088755969679) {
  err_fun(nom_x, delta_x, out_6435077088755969679);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_5461030829569705127) {
  inv_err_fun(nom_x, true_x, out_5461030829569705127);
}
void gnss_H_mod_fun(double *state, double *out_1113439463888521147) {
  H_mod_fun(state, out_1113439463888521147);
}
void gnss_f_fun(double *state, double dt, double *out_3853921857578862715) {
  f_fun(state,  dt, out_3853921857578862715);
}
void gnss_F_fun(double *state, double dt, double *out_1990023416032551193) {
  F_fun(state,  dt, out_1990023416032551193);
}
void gnss_h_6(double *state, double *sat_pos, double *out_5372040911404388122) {
  h_6(state, sat_pos, out_5372040911404388122);
}
void gnss_H_6(double *state, double *sat_pos, double *out_1538307200137441102) {
  H_6(state, sat_pos, out_1538307200137441102);
}
void gnss_h_20(double *state, double *sat_pos, double *out_5942081968181849174) {
  h_20(state, sat_pos, out_5942081968181849174);
}
void gnss_H_20(double *state, double *sat_pos, double *out_3657082352704797406) {
  H_20(state, sat_pos, out_3657082352704797406);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_4161012093764852918) {
  h_7(state, sat_pos_vel, out_4161012093764852918);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_3960385618526437650) {
  H_7(state, sat_pos_vel, out_3960385618526437650);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_4161012093764852918) {
  h_21(state, sat_pos_vel, out_4161012093764852918);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_3960385618526437650) {
  H_21(state, sat_pos_vel, out_3960385618526437650);
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
