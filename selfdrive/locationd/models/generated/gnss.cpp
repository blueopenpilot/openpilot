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
void err_fun(double *nom_x, double *delta_x, double *out_7621931960387363190) {
   out_7621931960387363190[0] = delta_x[0] + nom_x[0];
   out_7621931960387363190[1] = delta_x[1] + nom_x[1];
   out_7621931960387363190[2] = delta_x[2] + nom_x[2];
   out_7621931960387363190[3] = delta_x[3] + nom_x[3];
   out_7621931960387363190[4] = delta_x[4] + nom_x[4];
   out_7621931960387363190[5] = delta_x[5] + nom_x[5];
   out_7621931960387363190[6] = delta_x[6] + nom_x[6];
   out_7621931960387363190[7] = delta_x[7] + nom_x[7];
   out_7621931960387363190[8] = delta_x[8] + nom_x[8];
   out_7621931960387363190[9] = delta_x[9] + nom_x[9];
   out_7621931960387363190[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5740643755297289225) {
   out_5740643755297289225[0] = -nom_x[0] + true_x[0];
   out_5740643755297289225[1] = -nom_x[1] + true_x[1];
   out_5740643755297289225[2] = -nom_x[2] + true_x[2];
   out_5740643755297289225[3] = -nom_x[3] + true_x[3];
   out_5740643755297289225[4] = -nom_x[4] + true_x[4];
   out_5740643755297289225[5] = -nom_x[5] + true_x[5];
   out_5740643755297289225[6] = -nom_x[6] + true_x[6];
   out_5740643755297289225[7] = -nom_x[7] + true_x[7];
   out_5740643755297289225[8] = -nom_x[8] + true_x[8];
   out_5740643755297289225[9] = -nom_x[9] + true_x[9];
   out_5740643755297289225[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_3754482379115474220) {
   out_3754482379115474220[0] = 1.0;
   out_3754482379115474220[1] = 0;
   out_3754482379115474220[2] = 0;
   out_3754482379115474220[3] = 0;
   out_3754482379115474220[4] = 0;
   out_3754482379115474220[5] = 0;
   out_3754482379115474220[6] = 0;
   out_3754482379115474220[7] = 0;
   out_3754482379115474220[8] = 0;
   out_3754482379115474220[9] = 0;
   out_3754482379115474220[10] = 0;
   out_3754482379115474220[11] = 0;
   out_3754482379115474220[12] = 1.0;
   out_3754482379115474220[13] = 0;
   out_3754482379115474220[14] = 0;
   out_3754482379115474220[15] = 0;
   out_3754482379115474220[16] = 0;
   out_3754482379115474220[17] = 0;
   out_3754482379115474220[18] = 0;
   out_3754482379115474220[19] = 0;
   out_3754482379115474220[20] = 0;
   out_3754482379115474220[21] = 0;
   out_3754482379115474220[22] = 0;
   out_3754482379115474220[23] = 0;
   out_3754482379115474220[24] = 1.0;
   out_3754482379115474220[25] = 0;
   out_3754482379115474220[26] = 0;
   out_3754482379115474220[27] = 0;
   out_3754482379115474220[28] = 0;
   out_3754482379115474220[29] = 0;
   out_3754482379115474220[30] = 0;
   out_3754482379115474220[31] = 0;
   out_3754482379115474220[32] = 0;
   out_3754482379115474220[33] = 0;
   out_3754482379115474220[34] = 0;
   out_3754482379115474220[35] = 0;
   out_3754482379115474220[36] = 1.0;
   out_3754482379115474220[37] = 0;
   out_3754482379115474220[38] = 0;
   out_3754482379115474220[39] = 0;
   out_3754482379115474220[40] = 0;
   out_3754482379115474220[41] = 0;
   out_3754482379115474220[42] = 0;
   out_3754482379115474220[43] = 0;
   out_3754482379115474220[44] = 0;
   out_3754482379115474220[45] = 0;
   out_3754482379115474220[46] = 0;
   out_3754482379115474220[47] = 0;
   out_3754482379115474220[48] = 1.0;
   out_3754482379115474220[49] = 0;
   out_3754482379115474220[50] = 0;
   out_3754482379115474220[51] = 0;
   out_3754482379115474220[52] = 0;
   out_3754482379115474220[53] = 0;
   out_3754482379115474220[54] = 0;
   out_3754482379115474220[55] = 0;
   out_3754482379115474220[56] = 0;
   out_3754482379115474220[57] = 0;
   out_3754482379115474220[58] = 0;
   out_3754482379115474220[59] = 0;
   out_3754482379115474220[60] = 1.0;
   out_3754482379115474220[61] = 0;
   out_3754482379115474220[62] = 0;
   out_3754482379115474220[63] = 0;
   out_3754482379115474220[64] = 0;
   out_3754482379115474220[65] = 0;
   out_3754482379115474220[66] = 0;
   out_3754482379115474220[67] = 0;
   out_3754482379115474220[68] = 0;
   out_3754482379115474220[69] = 0;
   out_3754482379115474220[70] = 0;
   out_3754482379115474220[71] = 0;
   out_3754482379115474220[72] = 1.0;
   out_3754482379115474220[73] = 0;
   out_3754482379115474220[74] = 0;
   out_3754482379115474220[75] = 0;
   out_3754482379115474220[76] = 0;
   out_3754482379115474220[77] = 0;
   out_3754482379115474220[78] = 0;
   out_3754482379115474220[79] = 0;
   out_3754482379115474220[80] = 0;
   out_3754482379115474220[81] = 0;
   out_3754482379115474220[82] = 0;
   out_3754482379115474220[83] = 0;
   out_3754482379115474220[84] = 1.0;
   out_3754482379115474220[85] = 0;
   out_3754482379115474220[86] = 0;
   out_3754482379115474220[87] = 0;
   out_3754482379115474220[88] = 0;
   out_3754482379115474220[89] = 0;
   out_3754482379115474220[90] = 0;
   out_3754482379115474220[91] = 0;
   out_3754482379115474220[92] = 0;
   out_3754482379115474220[93] = 0;
   out_3754482379115474220[94] = 0;
   out_3754482379115474220[95] = 0;
   out_3754482379115474220[96] = 1.0;
   out_3754482379115474220[97] = 0;
   out_3754482379115474220[98] = 0;
   out_3754482379115474220[99] = 0;
   out_3754482379115474220[100] = 0;
   out_3754482379115474220[101] = 0;
   out_3754482379115474220[102] = 0;
   out_3754482379115474220[103] = 0;
   out_3754482379115474220[104] = 0;
   out_3754482379115474220[105] = 0;
   out_3754482379115474220[106] = 0;
   out_3754482379115474220[107] = 0;
   out_3754482379115474220[108] = 1.0;
   out_3754482379115474220[109] = 0;
   out_3754482379115474220[110] = 0;
   out_3754482379115474220[111] = 0;
   out_3754482379115474220[112] = 0;
   out_3754482379115474220[113] = 0;
   out_3754482379115474220[114] = 0;
   out_3754482379115474220[115] = 0;
   out_3754482379115474220[116] = 0;
   out_3754482379115474220[117] = 0;
   out_3754482379115474220[118] = 0;
   out_3754482379115474220[119] = 0;
   out_3754482379115474220[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_3214597710273968771) {
   out_3214597710273968771[0] = dt*state[3] + state[0];
   out_3214597710273968771[1] = dt*state[4] + state[1];
   out_3214597710273968771[2] = dt*state[5] + state[2];
   out_3214597710273968771[3] = state[3];
   out_3214597710273968771[4] = state[4];
   out_3214597710273968771[5] = state[5];
   out_3214597710273968771[6] = dt*state[7] + state[6];
   out_3214597710273968771[7] = dt*state[8] + state[7];
   out_3214597710273968771[8] = state[8];
   out_3214597710273968771[9] = state[9];
   out_3214597710273968771[10] = state[10];
}
void F_fun(double *state, double dt, double *out_5275588445702707408) {
   out_5275588445702707408[0] = 1;
   out_5275588445702707408[1] = 0;
   out_5275588445702707408[2] = 0;
   out_5275588445702707408[3] = dt;
   out_5275588445702707408[4] = 0;
   out_5275588445702707408[5] = 0;
   out_5275588445702707408[6] = 0;
   out_5275588445702707408[7] = 0;
   out_5275588445702707408[8] = 0;
   out_5275588445702707408[9] = 0;
   out_5275588445702707408[10] = 0;
   out_5275588445702707408[11] = 0;
   out_5275588445702707408[12] = 1;
   out_5275588445702707408[13] = 0;
   out_5275588445702707408[14] = 0;
   out_5275588445702707408[15] = dt;
   out_5275588445702707408[16] = 0;
   out_5275588445702707408[17] = 0;
   out_5275588445702707408[18] = 0;
   out_5275588445702707408[19] = 0;
   out_5275588445702707408[20] = 0;
   out_5275588445702707408[21] = 0;
   out_5275588445702707408[22] = 0;
   out_5275588445702707408[23] = 0;
   out_5275588445702707408[24] = 1;
   out_5275588445702707408[25] = 0;
   out_5275588445702707408[26] = 0;
   out_5275588445702707408[27] = dt;
   out_5275588445702707408[28] = 0;
   out_5275588445702707408[29] = 0;
   out_5275588445702707408[30] = 0;
   out_5275588445702707408[31] = 0;
   out_5275588445702707408[32] = 0;
   out_5275588445702707408[33] = 0;
   out_5275588445702707408[34] = 0;
   out_5275588445702707408[35] = 0;
   out_5275588445702707408[36] = 1;
   out_5275588445702707408[37] = 0;
   out_5275588445702707408[38] = 0;
   out_5275588445702707408[39] = 0;
   out_5275588445702707408[40] = 0;
   out_5275588445702707408[41] = 0;
   out_5275588445702707408[42] = 0;
   out_5275588445702707408[43] = 0;
   out_5275588445702707408[44] = 0;
   out_5275588445702707408[45] = 0;
   out_5275588445702707408[46] = 0;
   out_5275588445702707408[47] = 0;
   out_5275588445702707408[48] = 1;
   out_5275588445702707408[49] = 0;
   out_5275588445702707408[50] = 0;
   out_5275588445702707408[51] = 0;
   out_5275588445702707408[52] = 0;
   out_5275588445702707408[53] = 0;
   out_5275588445702707408[54] = 0;
   out_5275588445702707408[55] = 0;
   out_5275588445702707408[56] = 0;
   out_5275588445702707408[57] = 0;
   out_5275588445702707408[58] = 0;
   out_5275588445702707408[59] = 0;
   out_5275588445702707408[60] = 1;
   out_5275588445702707408[61] = 0;
   out_5275588445702707408[62] = 0;
   out_5275588445702707408[63] = 0;
   out_5275588445702707408[64] = 0;
   out_5275588445702707408[65] = 0;
   out_5275588445702707408[66] = 0;
   out_5275588445702707408[67] = 0;
   out_5275588445702707408[68] = 0;
   out_5275588445702707408[69] = 0;
   out_5275588445702707408[70] = 0;
   out_5275588445702707408[71] = 0;
   out_5275588445702707408[72] = 1;
   out_5275588445702707408[73] = dt;
   out_5275588445702707408[74] = 0;
   out_5275588445702707408[75] = 0;
   out_5275588445702707408[76] = 0;
   out_5275588445702707408[77] = 0;
   out_5275588445702707408[78] = 0;
   out_5275588445702707408[79] = 0;
   out_5275588445702707408[80] = 0;
   out_5275588445702707408[81] = 0;
   out_5275588445702707408[82] = 0;
   out_5275588445702707408[83] = 0;
   out_5275588445702707408[84] = 1;
   out_5275588445702707408[85] = dt;
   out_5275588445702707408[86] = 0;
   out_5275588445702707408[87] = 0;
   out_5275588445702707408[88] = 0;
   out_5275588445702707408[89] = 0;
   out_5275588445702707408[90] = 0;
   out_5275588445702707408[91] = 0;
   out_5275588445702707408[92] = 0;
   out_5275588445702707408[93] = 0;
   out_5275588445702707408[94] = 0;
   out_5275588445702707408[95] = 0;
   out_5275588445702707408[96] = 1;
   out_5275588445702707408[97] = 0;
   out_5275588445702707408[98] = 0;
   out_5275588445702707408[99] = 0;
   out_5275588445702707408[100] = 0;
   out_5275588445702707408[101] = 0;
   out_5275588445702707408[102] = 0;
   out_5275588445702707408[103] = 0;
   out_5275588445702707408[104] = 0;
   out_5275588445702707408[105] = 0;
   out_5275588445702707408[106] = 0;
   out_5275588445702707408[107] = 0;
   out_5275588445702707408[108] = 1;
   out_5275588445702707408[109] = 0;
   out_5275588445702707408[110] = 0;
   out_5275588445702707408[111] = 0;
   out_5275588445702707408[112] = 0;
   out_5275588445702707408[113] = 0;
   out_5275588445702707408[114] = 0;
   out_5275588445702707408[115] = 0;
   out_5275588445702707408[116] = 0;
   out_5275588445702707408[117] = 0;
   out_5275588445702707408[118] = 0;
   out_5275588445702707408[119] = 0;
   out_5275588445702707408[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_170926249370392253) {
   out_170926249370392253[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_3742869924876393196) {
   out_3742869924876393196[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3742869924876393196[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3742869924876393196[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3742869924876393196[3] = 0;
   out_3742869924876393196[4] = 0;
   out_3742869924876393196[5] = 0;
   out_3742869924876393196[6] = 1;
   out_3742869924876393196[7] = 0;
   out_3742869924876393196[8] = 0;
   out_3742869924876393196[9] = 0;
   out_3742869924876393196[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_1863480405032848687) {
   out_1863480405032848687[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_6580296787323519817) {
   out_6580296787323519817[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6580296787323519817[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6580296787323519817[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6580296787323519817[3] = 0;
   out_6580296787323519817[4] = 0;
   out_6580296787323519817[5] = 0;
   out_6580296787323519817[6] = 1;
   out_6580296787323519817[7] = 0;
   out_6580296787323519817[8] = 0;
   out_6580296787323519817[9] = 1;
   out_6580296787323519817[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_3420059550442289419) {
   out_3420059550442289419[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_2065618174834969080) {
   out_2065618174834969080[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2065618174834969080[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2065618174834969080[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2065618174834969080[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2065618174834969080[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2065618174834969080[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2065618174834969080[6] = 0;
   out_2065618174834969080[7] = 1;
   out_2065618174834969080[8] = 0;
   out_2065618174834969080[9] = 0;
   out_2065618174834969080[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_3420059550442289419) {
   out_3420059550442289419[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_2065618174834969080) {
   out_2065618174834969080[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2065618174834969080[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2065618174834969080[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2065618174834969080[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2065618174834969080[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2065618174834969080[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_2065618174834969080[6] = 0;
   out_2065618174834969080[7] = 1;
   out_2065618174834969080[8] = 0;
   out_2065618174834969080[9] = 0;
   out_2065618174834969080[10] = 0;
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
void gnss_err_fun(double *nom_x, double *delta_x, double *out_7621931960387363190) {
  err_fun(nom_x, delta_x, out_7621931960387363190);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_5740643755297289225) {
  inv_err_fun(nom_x, true_x, out_5740643755297289225);
}
void gnss_H_mod_fun(double *state, double *out_3754482379115474220) {
  H_mod_fun(state, out_3754482379115474220);
}
void gnss_f_fun(double *state, double dt, double *out_3214597710273968771) {
  f_fun(state,  dt, out_3214597710273968771);
}
void gnss_F_fun(double *state, double dt, double *out_5275588445702707408) {
  F_fun(state,  dt, out_5275588445702707408);
}
void gnss_h_6(double *state, double *sat_pos, double *out_170926249370392253) {
  h_6(state, sat_pos, out_170926249370392253);
}
void gnss_H_6(double *state, double *sat_pos, double *out_3742869924876393196) {
  H_6(state, sat_pos, out_3742869924876393196);
}
void gnss_h_20(double *state, double *sat_pos, double *out_1863480405032848687) {
  h_20(state, sat_pos, out_1863480405032848687);
}
void gnss_H_20(double *state, double *sat_pos, double *out_6580296787323519817) {
  H_20(state, sat_pos, out_6580296787323519817);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_3420059550442289419) {
  h_7(state, sat_pos_vel, out_3420059550442289419);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_2065618174834969080) {
  H_7(state, sat_pos_vel, out_2065618174834969080);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_3420059550442289419) {
  h_21(state, sat_pos_vel, out_3420059550442289419);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_2065618174834969080) {
  H_21(state, sat_pos_vel, out_2065618174834969080);
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
