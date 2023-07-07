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
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3447090500816114356) {
   out_3447090500816114356[0] = delta_x[0] + nom_x[0];
   out_3447090500816114356[1] = delta_x[1] + nom_x[1];
   out_3447090500816114356[2] = delta_x[2] + nom_x[2];
   out_3447090500816114356[3] = delta_x[3] + nom_x[3];
   out_3447090500816114356[4] = delta_x[4] + nom_x[4];
   out_3447090500816114356[5] = delta_x[5] + nom_x[5];
   out_3447090500816114356[6] = delta_x[6] + nom_x[6];
   out_3447090500816114356[7] = delta_x[7] + nom_x[7];
   out_3447090500816114356[8] = delta_x[8] + nom_x[8];
   out_3447090500816114356[9] = delta_x[9] + nom_x[9];
   out_3447090500816114356[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8035028355194170582) {
   out_8035028355194170582[0] = -nom_x[0] + true_x[0];
   out_8035028355194170582[1] = -nom_x[1] + true_x[1];
   out_8035028355194170582[2] = -nom_x[2] + true_x[2];
   out_8035028355194170582[3] = -nom_x[3] + true_x[3];
   out_8035028355194170582[4] = -nom_x[4] + true_x[4];
   out_8035028355194170582[5] = -nom_x[5] + true_x[5];
   out_8035028355194170582[6] = -nom_x[6] + true_x[6];
   out_8035028355194170582[7] = -nom_x[7] + true_x[7];
   out_8035028355194170582[8] = -nom_x[8] + true_x[8];
   out_8035028355194170582[9] = -nom_x[9] + true_x[9];
   out_8035028355194170582[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_2818950684216781197) {
   out_2818950684216781197[0] = 1.0;
   out_2818950684216781197[1] = 0;
   out_2818950684216781197[2] = 0;
   out_2818950684216781197[3] = 0;
   out_2818950684216781197[4] = 0;
   out_2818950684216781197[5] = 0;
   out_2818950684216781197[6] = 0;
   out_2818950684216781197[7] = 0;
   out_2818950684216781197[8] = 0;
   out_2818950684216781197[9] = 0;
   out_2818950684216781197[10] = 0;
   out_2818950684216781197[11] = 0;
   out_2818950684216781197[12] = 1.0;
   out_2818950684216781197[13] = 0;
   out_2818950684216781197[14] = 0;
   out_2818950684216781197[15] = 0;
   out_2818950684216781197[16] = 0;
   out_2818950684216781197[17] = 0;
   out_2818950684216781197[18] = 0;
   out_2818950684216781197[19] = 0;
   out_2818950684216781197[20] = 0;
   out_2818950684216781197[21] = 0;
   out_2818950684216781197[22] = 0;
   out_2818950684216781197[23] = 0;
   out_2818950684216781197[24] = 1.0;
   out_2818950684216781197[25] = 0;
   out_2818950684216781197[26] = 0;
   out_2818950684216781197[27] = 0;
   out_2818950684216781197[28] = 0;
   out_2818950684216781197[29] = 0;
   out_2818950684216781197[30] = 0;
   out_2818950684216781197[31] = 0;
   out_2818950684216781197[32] = 0;
   out_2818950684216781197[33] = 0;
   out_2818950684216781197[34] = 0;
   out_2818950684216781197[35] = 0;
   out_2818950684216781197[36] = 1.0;
   out_2818950684216781197[37] = 0;
   out_2818950684216781197[38] = 0;
   out_2818950684216781197[39] = 0;
   out_2818950684216781197[40] = 0;
   out_2818950684216781197[41] = 0;
   out_2818950684216781197[42] = 0;
   out_2818950684216781197[43] = 0;
   out_2818950684216781197[44] = 0;
   out_2818950684216781197[45] = 0;
   out_2818950684216781197[46] = 0;
   out_2818950684216781197[47] = 0;
   out_2818950684216781197[48] = 1.0;
   out_2818950684216781197[49] = 0;
   out_2818950684216781197[50] = 0;
   out_2818950684216781197[51] = 0;
   out_2818950684216781197[52] = 0;
   out_2818950684216781197[53] = 0;
   out_2818950684216781197[54] = 0;
   out_2818950684216781197[55] = 0;
   out_2818950684216781197[56] = 0;
   out_2818950684216781197[57] = 0;
   out_2818950684216781197[58] = 0;
   out_2818950684216781197[59] = 0;
   out_2818950684216781197[60] = 1.0;
   out_2818950684216781197[61] = 0;
   out_2818950684216781197[62] = 0;
   out_2818950684216781197[63] = 0;
   out_2818950684216781197[64] = 0;
   out_2818950684216781197[65] = 0;
   out_2818950684216781197[66] = 0;
   out_2818950684216781197[67] = 0;
   out_2818950684216781197[68] = 0;
   out_2818950684216781197[69] = 0;
   out_2818950684216781197[70] = 0;
   out_2818950684216781197[71] = 0;
   out_2818950684216781197[72] = 1.0;
   out_2818950684216781197[73] = 0;
   out_2818950684216781197[74] = 0;
   out_2818950684216781197[75] = 0;
   out_2818950684216781197[76] = 0;
   out_2818950684216781197[77] = 0;
   out_2818950684216781197[78] = 0;
   out_2818950684216781197[79] = 0;
   out_2818950684216781197[80] = 0;
   out_2818950684216781197[81] = 0;
   out_2818950684216781197[82] = 0;
   out_2818950684216781197[83] = 0;
   out_2818950684216781197[84] = 1.0;
   out_2818950684216781197[85] = 0;
   out_2818950684216781197[86] = 0;
   out_2818950684216781197[87] = 0;
   out_2818950684216781197[88] = 0;
   out_2818950684216781197[89] = 0;
   out_2818950684216781197[90] = 0;
   out_2818950684216781197[91] = 0;
   out_2818950684216781197[92] = 0;
   out_2818950684216781197[93] = 0;
   out_2818950684216781197[94] = 0;
   out_2818950684216781197[95] = 0;
   out_2818950684216781197[96] = 1.0;
   out_2818950684216781197[97] = 0;
   out_2818950684216781197[98] = 0;
   out_2818950684216781197[99] = 0;
   out_2818950684216781197[100] = 0;
   out_2818950684216781197[101] = 0;
   out_2818950684216781197[102] = 0;
   out_2818950684216781197[103] = 0;
   out_2818950684216781197[104] = 0;
   out_2818950684216781197[105] = 0;
   out_2818950684216781197[106] = 0;
   out_2818950684216781197[107] = 0;
   out_2818950684216781197[108] = 1.0;
   out_2818950684216781197[109] = 0;
   out_2818950684216781197[110] = 0;
   out_2818950684216781197[111] = 0;
   out_2818950684216781197[112] = 0;
   out_2818950684216781197[113] = 0;
   out_2818950684216781197[114] = 0;
   out_2818950684216781197[115] = 0;
   out_2818950684216781197[116] = 0;
   out_2818950684216781197[117] = 0;
   out_2818950684216781197[118] = 0;
   out_2818950684216781197[119] = 0;
   out_2818950684216781197[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_3087561945035795840) {
   out_3087561945035795840[0] = dt*state[3] + state[0];
   out_3087561945035795840[1] = dt*state[4] + state[1];
   out_3087561945035795840[2] = dt*state[5] + state[2];
   out_3087561945035795840[3] = state[3];
   out_3087561945035795840[4] = state[4];
   out_3087561945035795840[5] = state[5];
   out_3087561945035795840[6] = dt*state[7] + state[6];
   out_3087561945035795840[7] = dt*state[8] + state[7];
   out_3087561945035795840[8] = state[8];
   out_3087561945035795840[9] = state[9];
   out_3087561945035795840[10] = state[10];
}
void F_fun(double *state, double dt, double *out_8128229715354435733) {
   out_8128229715354435733[0] = 1;
   out_8128229715354435733[1] = 0;
   out_8128229715354435733[2] = 0;
   out_8128229715354435733[3] = dt;
   out_8128229715354435733[4] = 0;
   out_8128229715354435733[5] = 0;
   out_8128229715354435733[6] = 0;
   out_8128229715354435733[7] = 0;
   out_8128229715354435733[8] = 0;
   out_8128229715354435733[9] = 0;
   out_8128229715354435733[10] = 0;
   out_8128229715354435733[11] = 0;
   out_8128229715354435733[12] = 1;
   out_8128229715354435733[13] = 0;
   out_8128229715354435733[14] = 0;
   out_8128229715354435733[15] = dt;
   out_8128229715354435733[16] = 0;
   out_8128229715354435733[17] = 0;
   out_8128229715354435733[18] = 0;
   out_8128229715354435733[19] = 0;
   out_8128229715354435733[20] = 0;
   out_8128229715354435733[21] = 0;
   out_8128229715354435733[22] = 0;
   out_8128229715354435733[23] = 0;
   out_8128229715354435733[24] = 1;
   out_8128229715354435733[25] = 0;
   out_8128229715354435733[26] = 0;
   out_8128229715354435733[27] = dt;
   out_8128229715354435733[28] = 0;
   out_8128229715354435733[29] = 0;
   out_8128229715354435733[30] = 0;
   out_8128229715354435733[31] = 0;
   out_8128229715354435733[32] = 0;
   out_8128229715354435733[33] = 0;
   out_8128229715354435733[34] = 0;
   out_8128229715354435733[35] = 0;
   out_8128229715354435733[36] = 1;
   out_8128229715354435733[37] = 0;
   out_8128229715354435733[38] = 0;
   out_8128229715354435733[39] = 0;
   out_8128229715354435733[40] = 0;
   out_8128229715354435733[41] = 0;
   out_8128229715354435733[42] = 0;
   out_8128229715354435733[43] = 0;
   out_8128229715354435733[44] = 0;
   out_8128229715354435733[45] = 0;
   out_8128229715354435733[46] = 0;
   out_8128229715354435733[47] = 0;
   out_8128229715354435733[48] = 1;
   out_8128229715354435733[49] = 0;
   out_8128229715354435733[50] = 0;
   out_8128229715354435733[51] = 0;
   out_8128229715354435733[52] = 0;
   out_8128229715354435733[53] = 0;
   out_8128229715354435733[54] = 0;
   out_8128229715354435733[55] = 0;
   out_8128229715354435733[56] = 0;
   out_8128229715354435733[57] = 0;
   out_8128229715354435733[58] = 0;
   out_8128229715354435733[59] = 0;
   out_8128229715354435733[60] = 1;
   out_8128229715354435733[61] = 0;
   out_8128229715354435733[62] = 0;
   out_8128229715354435733[63] = 0;
   out_8128229715354435733[64] = 0;
   out_8128229715354435733[65] = 0;
   out_8128229715354435733[66] = 0;
   out_8128229715354435733[67] = 0;
   out_8128229715354435733[68] = 0;
   out_8128229715354435733[69] = 0;
   out_8128229715354435733[70] = 0;
   out_8128229715354435733[71] = 0;
   out_8128229715354435733[72] = 1;
   out_8128229715354435733[73] = dt;
   out_8128229715354435733[74] = 0;
   out_8128229715354435733[75] = 0;
   out_8128229715354435733[76] = 0;
   out_8128229715354435733[77] = 0;
   out_8128229715354435733[78] = 0;
   out_8128229715354435733[79] = 0;
   out_8128229715354435733[80] = 0;
   out_8128229715354435733[81] = 0;
   out_8128229715354435733[82] = 0;
   out_8128229715354435733[83] = 0;
   out_8128229715354435733[84] = 1;
   out_8128229715354435733[85] = dt;
   out_8128229715354435733[86] = 0;
   out_8128229715354435733[87] = 0;
   out_8128229715354435733[88] = 0;
   out_8128229715354435733[89] = 0;
   out_8128229715354435733[90] = 0;
   out_8128229715354435733[91] = 0;
   out_8128229715354435733[92] = 0;
   out_8128229715354435733[93] = 0;
   out_8128229715354435733[94] = 0;
   out_8128229715354435733[95] = 0;
   out_8128229715354435733[96] = 1;
   out_8128229715354435733[97] = 0;
   out_8128229715354435733[98] = 0;
   out_8128229715354435733[99] = 0;
   out_8128229715354435733[100] = 0;
   out_8128229715354435733[101] = 0;
   out_8128229715354435733[102] = 0;
   out_8128229715354435733[103] = 0;
   out_8128229715354435733[104] = 0;
   out_8128229715354435733[105] = 0;
   out_8128229715354435733[106] = 0;
   out_8128229715354435733[107] = 0;
   out_8128229715354435733[108] = 1;
   out_8128229715354435733[109] = 0;
   out_8128229715354435733[110] = 0;
   out_8128229715354435733[111] = 0;
   out_8128229715354435733[112] = 0;
   out_8128229715354435733[113] = 0;
   out_8128229715354435733[114] = 0;
   out_8128229715354435733[115] = 0;
   out_8128229715354435733[116] = 0;
   out_8128229715354435733[117] = 0;
   out_8128229715354435733[118] = 0;
   out_8128229715354435733[119] = 0;
   out_8128229715354435733[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_6299004301386785170) {
   out_6299004301386785170[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_4160133055002661189) {
   out_4160133055002661189[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4160133055002661189[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4160133055002661189[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_4160133055002661189[3] = 0;
   out_4160133055002661189[4] = 0;
   out_4160133055002661189[5] = 0;
   out_4160133055002661189[6] = 1;
   out_4160133055002661189[7] = 0;
   out_4160133055002661189[8] = 0;
   out_4160133055002661189[9] = 0;
   out_4160133055002661189[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_7323928502644680378) {
   out_7323928502644680378[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_1240025102187636063) {
   out_1240025102187636063[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1240025102187636063[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1240025102187636063[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1240025102187636063[3] = 0;
   out_1240025102187636063[4] = 0;
   out_1240025102187636063[5] = 0;
   out_1240025102187636063[6] = 1;
   out_1240025102187636063[7] = 0;
   out_1240025102187636063[8] = 0;
   out_1240025102187636063[9] = 1;
   out_1240025102187636063[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_4309803303408579533) {
   out_4309803303408579533[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_2751727956889753603) {
   out_2751727956889753603[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2751727956889753603[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2751727956889753603[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2751727956889753603[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2751727956889753603[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2751727956889753603[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2751727956889753603[6] = 0;
   out_2751727956889753603[7] = 1;
   out_2751727956889753603[8] = 0;
   out_2751727956889753603[9] = 0;
   out_2751727956889753603[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_4309803303408579533) {
   out_4309803303408579533[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_2751727956889753603) {
   out_2751727956889753603[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2751727956889753603[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2751727956889753603[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2751727956889753603[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2751727956889753603[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2751727956889753603[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2751727956889753603[6] = 0;
   out_2751727956889753603[7] = 1;
   out_2751727956889753603[8] = 0;
   out_2751727956889753603[9] = 0;
   out_2751727956889753603[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_3447090500816114356) {
  err_fun(nom_x, delta_x, out_3447090500816114356);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_8035028355194170582) {
  inv_err_fun(nom_x, true_x, out_8035028355194170582);
}
void gnss_H_mod_fun(double *state, double *out_2818950684216781197) {
  H_mod_fun(state, out_2818950684216781197);
}
void gnss_f_fun(double *state, double dt, double *out_3087561945035795840) {
  f_fun(state,  dt, out_3087561945035795840);
}
void gnss_F_fun(double *state, double dt, double *out_8128229715354435733) {
  F_fun(state,  dt, out_8128229715354435733);
}
void gnss_h_6(double *state, double *sat_pos, double *out_6299004301386785170) {
  h_6(state, sat_pos, out_6299004301386785170);
}
void gnss_H_6(double *state, double *sat_pos, double *out_4160133055002661189) {
  H_6(state, sat_pos, out_4160133055002661189);
}
void gnss_h_20(double *state, double *sat_pos, double *out_7323928502644680378) {
  h_20(state, sat_pos, out_7323928502644680378);
}
void gnss_H_20(double *state, double *sat_pos, double *out_1240025102187636063) {
  H_20(state, sat_pos, out_1240025102187636063);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_4309803303408579533) {
  h_7(state, sat_pos_vel, out_4309803303408579533);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_2751727956889753603) {
  H_7(state, sat_pos_vel, out_2751727956889753603);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_4309803303408579533) {
  h_21(state, sat_pos_vel, out_4309803303408579533);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_2751727956889753603) {
  H_21(state, sat_pos_vel, out_2751727956889753603);
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
