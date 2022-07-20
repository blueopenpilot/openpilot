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
void err_fun(double *nom_x, double *delta_x, double *out_2220497270947833429) {
   out_2220497270947833429[0] = delta_x[0] + nom_x[0];
   out_2220497270947833429[1] = delta_x[1] + nom_x[1];
   out_2220497270947833429[2] = delta_x[2] + nom_x[2];
   out_2220497270947833429[3] = delta_x[3] + nom_x[3];
   out_2220497270947833429[4] = delta_x[4] + nom_x[4];
   out_2220497270947833429[5] = delta_x[5] + nom_x[5];
   out_2220497270947833429[6] = delta_x[6] + nom_x[6];
   out_2220497270947833429[7] = delta_x[7] + nom_x[7];
   out_2220497270947833429[8] = delta_x[8] + nom_x[8];
   out_2220497270947833429[9] = delta_x[9] + nom_x[9];
   out_2220497270947833429[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4694480141159607186) {
   out_4694480141159607186[0] = -nom_x[0] + true_x[0];
   out_4694480141159607186[1] = -nom_x[1] + true_x[1];
   out_4694480141159607186[2] = -nom_x[2] + true_x[2];
   out_4694480141159607186[3] = -nom_x[3] + true_x[3];
   out_4694480141159607186[4] = -nom_x[4] + true_x[4];
   out_4694480141159607186[5] = -nom_x[5] + true_x[5];
   out_4694480141159607186[6] = -nom_x[6] + true_x[6];
   out_4694480141159607186[7] = -nom_x[7] + true_x[7];
   out_4694480141159607186[8] = -nom_x[8] + true_x[8];
   out_4694480141159607186[9] = -nom_x[9] + true_x[9];
   out_4694480141159607186[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_8762642917035623636) {
   out_8762642917035623636[0] = 1.0;
   out_8762642917035623636[1] = 0;
   out_8762642917035623636[2] = 0;
   out_8762642917035623636[3] = 0;
   out_8762642917035623636[4] = 0;
   out_8762642917035623636[5] = 0;
   out_8762642917035623636[6] = 0;
   out_8762642917035623636[7] = 0;
   out_8762642917035623636[8] = 0;
   out_8762642917035623636[9] = 0;
   out_8762642917035623636[10] = 0;
   out_8762642917035623636[11] = 0;
   out_8762642917035623636[12] = 1.0;
   out_8762642917035623636[13] = 0;
   out_8762642917035623636[14] = 0;
   out_8762642917035623636[15] = 0;
   out_8762642917035623636[16] = 0;
   out_8762642917035623636[17] = 0;
   out_8762642917035623636[18] = 0;
   out_8762642917035623636[19] = 0;
   out_8762642917035623636[20] = 0;
   out_8762642917035623636[21] = 0;
   out_8762642917035623636[22] = 0;
   out_8762642917035623636[23] = 0;
   out_8762642917035623636[24] = 1.0;
   out_8762642917035623636[25] = 0;
   out_8762642917035623636[26] = 0;
   out_8762642917035623636[27] = 0;
   out_8762642917035623636[28] = 0;
   out_8762642917035623636[29] = 0;
   out_8762642917035623636[30] = 0;
   out_8762642917035623636[31] = 0;
   out_8762642917035623636[32] = 0;
   out_8762642917035623636[33] = 0;
   out_8762642917035623636[34] = 0;
   out_8762642917035623636[35] = 0;
   out_8762642917035623636[36] = 1.0;
   out_8762642917035623636[37] = 0;
   out_8762642917035623636[38] = 0;
   out_8762642917035623636[39] = 0;
   out_8762642917035623636[40] = 0;
   out_8762642917035623636[41] = 0;
   out_8762642917035623636[42] = 0;
   out_8762642917035623636[43] = 0;
   out_8762642917035623636[44] = 0;
   out_8762642917035623636[45] = 0;
   out_8762642917035623636[46] = 0;
   out_8762642917035623636[47] = 0;
   out_8762642917035623636[48] = 1.0;
   out_8762642917035623636[49] = 0;
   out_8762642917035623636[50] = 0;
   out_8762642917035623636[51] = 0;
   out_8762642917035623636[52] = 0;
   out_8762642917035623636[53] = 0;
   out_8762642917035623636[54] = 0;
   out_8762642917035623636[55] = 0;
   out_8762642917035623636[56] = 0;
   out_8762642917035623636[57] = 0;
   out_8762642917035623636[58] = 0;
   out_8762642917035623636[59] = 0;
   out_8762642917035623636[60] = 1.0;
   out_8762642917035623636[61] = 0;
   out_8762642917035623636[62] = 0;
   out_8762642917035623636[63] = 0;
   out_8762642917035623636[64] = 0;
   out_8762642917035623636[65] = 0;
   out_8762642917035623636[66] = 0;
   out_8762642917035623636[67] = 0;
   out_8762642917035623636[68] = 0;
   out_8762642917035623636[69] = 0;
   out_8762642917035623636[70] = 0;
   out_8762642917035623636[71] = 0;
   out_8762642917035623636[72] = 1.0;
   out_8762642917035623636[73] = 0;
   out_8762642917035623636[74] = 0;
   out_8762642917035623636[75] = 0;
   out_8762642917035623636[76] = 0;
   out_8762642917035623636[77] = 0;
   out_8762642917035623636[78] = 0;
   out_8762642917035623636[79] = 0;
   out_8762642917035623636[80] = 0;
   out_8762642917035623636[81] = 0;
   out_8762642917035623636[82] = 0;
   out_8762642917035623636[83] = 0;
   out_8762642917035623636[84] = 1.0;
   out_8762642917035623636[85] = 0;
   out_8762642917035623636[86] = 0;
   out_8762642917035623636[87] = 0;
   out_8762642917035623636[88] = 0;
   out_8762642917035623636[89] = 0;
   out_8762642917035623636[90] = 0;
   out_8762642917035623636[91] = 0;
   out_8762642917035623636[92] = 0;
   out_8762642917035623636[93] = 0;
   out_8762642917035623636[94] = 0;
   out_8762642917035623636[95] = 0;
   out_8762642917035623636[96] = 1.0;
   out_8762642917035623636[97] = 0;
   out_8762642917035623636[98] = 0;
   out_8762642917035623636[99] = 0;
   out_8762642917035623636[100] = 0;
   out_8762642917035623636[101] = 0;
   out_8762642917035623636[102] = 0;
   out_8762642917035623636[103] = 0;
   out_8762642917035623636[104] = 0;
   out_8762642917035623636[105] = 0;
   out_8762642917035623636[106] = 0;
   out_8762642917035623636[107] = 0;
   out_8762642917035623636[108] = 1.0;
   out_8762642917035623636[109] = 0;
   out_8762642917035623636[110] = 0;
   out_8762642917035623636[111] = 0;
   out_8762642917035623636[112] = 0;
   out_8762642917035623636[113] = 0;
   out_8762642917035623636[114] = 0;
   out_8762642917035623636[115] = 0;
   out_8762642917035623636[116] = 0;
   out_8762642917035623636[117] = 0;
   out_8762642917035623636[118] = 0;
   out_8762642917035623636[119] = 0;
   out_8762642917035623636[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_3481548129316230265) {
   out_3481548129316230265[0] = dt*state[3] + state[0];
   out_3481548129316230265[1] = dt*state[4] + state[1];
   out_3481548129316230265[2] = dt*state[5] + state[2];
   out_3481548129316230265[3] = state[3];
   out_3481548129316230265[4] = state[4];
   out_3481548129316230265[5] = state[5];
   out_3481548129316230265[6] = dt*state[7] + state[6];
   out_3481548129316230265[7] = dt*state[8] + state[7];
   out_3481548129316230265[8] = state[8];
   out_3481548129316230265[9] = state[9];
   out_3481548129316230265[10] = state[10];
}
void F_fun(double *state, double dt, double *out_4057106452161948183) {
   out_4057106452161948183[0] = 1;
   out_4057106452161948183[1] = 0;
   out_4057106452161948183[2] = 0;
   out_4057106452161948183[3] = dt;
   out_4057106452161948183[4] = 0;
   out_4057106452161948183[5] = 0;
   out_4057106452161948183[6] = 0;
   out_4057106452161948183[7] = 0;
   out_4057106452161948183[8] = 0;
   out_4057106452161948183[9] = 0;
   out_4057106452161948183[10] = 0;
   out_4057106452161948183[11] = 0;
   out_4057106452161948183[12] = 1;
   out_4057106452161948183[13] = 0;
   out_4057106452161948183[14] = 0;
   out_4057106452161948183[15] = dt;
   out_4057106452161948183[16] = 0;
   out_4057106452161948183[17] = 0;
   out_4057106452161948183[18] = 0;
   out_4057106452161948183[19] = 0;
   out_4057106452161948183[20] = 0;
   out_4057106452161948183[21] = 0;
   out_4057106452161948183[22] = 0;
   out_4057106452161948183[23] = 0;
   out_4057106452161948183[24] = 1;
   out_4057106452161948183[25] = 0;
   out_4057106452161948183[26] = 0;
   out_4057106452161948183[27] = dt;
   out_4057106452161948183[28] = 0;
   out_4057106452161948183[29] = 0;
   out_4057106452161948183[30] = 0;
   out_4057106452161948183[31] = 0;
   out_4057106452161948183[32] = 0;
   out_4057106452161948183[33] = 0;
   out_4057106452161948183[34] = 0;
   out_4057106452161948183[35] = 0;
   out_4057106452161948183[36] = 1;
   out_4057106452161948183[37] = 0;
   out_4057106452161948183[38] = 0;
   out_4057106452161948183[39] = 0;
   out_4057106452161948183[40] = 0;
   out_4057106452161948183[41] = 0;
   out_4057106452161948183[42] = 0;
   out_4057106452161948183[43] = 0;
   out_4057106452161948183[44] = 0;
   out_4057106452161948183[45] = 0;
   out_4057106452161948183[46] = 0;
   out_4057106452161948183[47] = 0;
   out_4057106452161948183[48] = 1;
   out_4057106452161948183[49] = 0;
   out_4057106452161948183[50] = 0;
   out_4057106452161948183[51] = 0;
   out_4057106452161948183[52] = 0;
   out_4057106452161948183[53] = 0;
   out_4057106452161948183[54] = 0;
   out_4057106452161948183[55] = 0;
   out_4057106452161948183[56] = 0;
   out_4057106452161948183[57] = 0;
   out_4057106452161948183[58] = 0;
   out_4057106452161948183[59] = 0;
   out_4057106452161948183[60] = 1;
   out_4057106452161948183[61] = 0;
   out_4057106452161948183[62] = 0;
   out_4057106452161948183[63] = 0;
   out_4057106452161948183[64] = 0;
   out_4057106452161948183[65] = 0;
   out_4057106452161948183[66] = 0;
   out_4057106452161948183[67] = 0;
   out_4057106452161948183[68] = 0;
   out_4057106452161948183[69] = 0;
   out_4057106452161948183[70] = 0;
   out_4057106452161948183[71] = 0;
   out_4057106452161948183[72] = 1;
   out_4057106452161948183[73] = dt;
   out_4057106452161948183[74] = 0;
   out_4057106452161948183[75] = 0;
   out_4057106452161948183[76] = 0;
   out_4057106452161948183[77] = 0;
   out_4057106452161948183[78] = 0;
   out_4057106452161948183[79] = 0;
   out_4057106452161948183[80] = 0;
   out_4057106452161948183[81] = 0;
   out_4057106452161948183[82] = 0;
   out_4057106452161948183[83] = 0;
   out_4057106452161948183[84] = 1;
   out_4057106452161948183[85] = dt;
   out_4057106452161948183[86] = 0;
   out_4057106452161948183[87] = 0;
   out_4057106452161948183[88] = 0;
   out_4057106452161948183[89] = 0;
   out_4057106452161948183[90] = 0;
   out_4057106452161948183[91] = 0;
   out_4057106452161948183[92] = 0;
   out_4057106452161948183[93] = 0;
   out_4057106452161948183[94] = 0;
   out_4057106452161948183[95] = 0;
   out_4057106452161948183[96] = 1;
   out_4057106452161948183[97] = 0;
   out_4057106452161948183[98] = 0;
   out_4057106452161948183[99] = 0;
   out_4057106452161948183[100] = 0;
   out_4057106452161948183[101] = 0;
   out_4057106452161948183[102] = 0;
   out_4057106452161948183[103] = 0;
   out_4057106452161948183[104] = 0;
   out_4057106452161948183[105] = 0;
   out_4057106452161948183[106] = 0;
   out_4057106452161948183[107] = 0;
   out_4057106452161948183[108] = 1;
   out_4057106452161948183[109] = 0;
   out_4057106452161948183[110] = 0;
   out_4057106452161948183[111] = 0;
   out_4057106452161948183[112] = 0;
   out_4057106452161948183[113] = 0;
   out_4057106452161948183[114] = 0;
   out_4057106452161948183[115] = 0;
   out_4057106452161948183[116] = 0;
   out_4057106452161948183[117] = 0;
   out_4057106452161948183[118] = 0;
   out_4057106452161948183[119] = 0;
   out_4057106452161948183[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_2042956050660128342) {
   out_2042956050660128342[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_6208909197614479079) {
   out_6208909197614479079[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6208909197614479079[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6208909197614479079[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6208909197614479079[3] = 0;
   out_6208909197614479079[4] = 0;
   out_6208909197614479079[5] = 0;
   out_6208909197614479079[6] = 1;
   out_6208909197614479079[7] = 0;
   out_6208909197614479079[8] = 0;
   out_6208909197614479079[9] = 0;
   out_6208909197614479079[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_2432843798433092424) {
   out_2432843798433092424[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_5201508118287945176) {
   out_5201508118287945176[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5201508118287945176[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5201508118287945176[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5201508118287945176[3] = 0;
   out_5201508118287945176[4] = 0;
   out_5201508118287945176[5] = 0;
   out_5201508118287945176[6] = 1;
   out_5201508118287945176[7] = 0;
   out_5201508118287945176[8] = 0;
   out_5201508118287945176[9] = 1;
   out_5201508118287945176[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_8574522793821478815) {
   out_8574522793821478815[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_8833501221798871914) {
   out_8833501221798871914[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8833501221798871914[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8833501221798871914[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8833501221798871914[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8833501221798871914[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8833501221798871914[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8833501221798871914[6] = 0;
   out_8833501221798871914[7] = 1;
   out_8833501221798871914[8] = 0;
   out_8833501221798871914[9] = 0;
   out_8833501221798871914[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_8574522793821478815) {
   out_8574522793821478815[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_8833501221798871914) {
   out_8833501221798871914[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8833501221798871914[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8833501221798871914[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8833501221798871914[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8833501221798871914[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8833501221798871914[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8833501221798871914[6] = 0;
   out_8833501221798871914[7] = 1;
   out_8833501221798871914[8] = 0;
   out_8833501221798871914[9] = 0;
   out_8833501221798871914[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_2220497270947833429) {
  err_fun(nom_x, delta_x, out_2220497270947833429);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_4694480141159607186) {
  inv_err_fun(nom_x, true_x, out_4694480141159607186);
}
void gnss_H_mod_fun(double *state, double *out_8762642917035623636) {
  H_mod_fun(state, out_8762642917035623636);
}
void gnss_f_fun(double *state, double dt, double *out_3481548129316230265) {
  f_fun(state,  dt, out_3481548129316230265);
}
void gnss_F_fun(double *state, double dt, double *out_4057106452161948183) {
  F_fun(state,  dt, out_4057106452161948183);
}
void gnss_h_6(double *state, double *sat_pos, double *out_2042956050660128342) {
  h_6(state, sat_pos, out_2042956050660128342);
}
void gnss_H_6(double *state, double *sat_pos, double *out_6208909197614479079) {
  H_6(state, sat_pos, out_6208909197614479079);
}
void gnss_h_20(double *state, double *sat_pos, double *out_2432843798433092424) {
  h_20(state, sat_pos, out_2432843798433092424);
}
void gnss_H_20(double *state, double *sat_pos, double *out_5201508118287945176) {
  H_20(state, sat_pos, out_5201508118287945176);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_8574522793821478815) {
  h_7(state, sat_pos_vel, out_8574522793821478815);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_8833501221798871914) {
  H_7(state, sat_pos_vel, out_8833501221798871914);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_8574522793821478815) {
  h_21(state, sat_pos_vel, out_8574522793821478815);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_8833501221798871914) {
  H_21(state, sat_pos_vel, out_8833501221798871914);
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
